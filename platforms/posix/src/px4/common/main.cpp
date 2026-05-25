/****************************************************************************
 *
 *   Copyright (C) 2015-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file main.cpp
 *
 * This is the main() of PX4 for POSIX.
 *
 * The application is designed as a daemon/server app with multiple clients.
 * Both, the server and the client is started using this main() function.
 *
 * If the executable is called with its usual name 'px4', it will start the
 * server. However, if it is started with an executable name (symlink) starting
 * with 'px4-' such as 'px4-navigator', it will start as a client and try to
 * connect to the server.
 *
 * The symlinks for all modules are created using the build system.
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 * @author Roman Bapst <bapstroman@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <string>
#include <algorithm>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#if (_POSIX_MEMLOCK > 0) && !defined(__PX4_WINDOWS)
#include <sys/mman.h>
#endif

#ifdef __PX4_WINDOWS
// MinGW ships no shell at /bin/sh, no geteuid, no sigaction.
// mkdir is already a 2-arg POSIX wrapper via the windows_shim sys/stat.h.
// Provide the remaining forwards inline so the stock POSIX main.cpp
// compiles unchanged.
#include <windows.h>
#include <px4_windows/platform.h>
static inline unsigned int geteuid(void) { return 1000; }
#endif

#if defined(_MSC_VER)
// MSVC CRT debug heap: dumps unfreed allocations to stderr at process exit.
// Only active in Debug builds linked against the debug CRT (/MDd or /MTd).
#include <crtdbg.h>
#endif

#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/shell.h>
#include <uORB/uORB.h>

#include "apps.h"
#include "px4_daemon/client.h"
#include "px4_daemon/server.h"
#include "px4_daemon/pxh.h"

#define MODULE_NAME "px4"

#ifndef __PX4_WINDOWS
static const char *LOCK_FILE_PATH = "/tmp/px4_lock";
#endif

#ifndef PATH_MAX
#define PATH_MAX 1024
#endif


static volatile bool _exit_requested = false;
static volatile sig_atomic_t _shutdown_started = 0;


namespace px4
{
void init_once();
}

static void sig_int_handler(int sig_num);

static void register_sig_handler();
static void set_cpu_scaling();
static int create_symlinks_if_needed(std::string &data_path);
static int create_dirs();
static int run_startup_script(const std::string &commands_file, const std::string &absolute_binary_path, int instance);
static std::string get_absolute_binary_path(const std::string &argv0);
static void wait_to_exit();
static std::string get_lock_file_path(int instance);
static int get_server_running(int instance, bool *is_running);
static int set_server_running(int instance);
static void print_usage();
static bool dir_exists(const std::string &path);
static bool file_exists(const std::string &name);
static bool is_absolute_path(const std::string &path);
static bool is_path_separator(char ch);
static std::string file_basename(std::string const &pathname);
static std::string pwd();
static int change_directory(const std::string &directory);

#ifdef __PX4_WINDOWS
// Unblock the main thread's getchar() so the pxh loop can notice
// _should_exit. Windows delivers Ctrl+C on a dedicated handler thread,
// which means just flipping a flag leaves the main thread blocked in its
// stdin ReadFile forever. Two nudges, tried in order:
//   1) inject a synthetic '\n' keypress into the console input buffer so
//      getchar() returns cleanly; works when stdin is an attached console;
//   2) CancelIoEx on the stdin handle; works when stdin has been
//      redirected to a pipe/file (e.g. `wine px4.exe < script`).
static void kick_stdin_reader()
{
	HANDLE stdin_h = GetStdHandle(STD_INPUT_HANDLE);

	if (stdin_h == INVALID_HANDLE_VALUE || stdin_h == nullptr) {
		return;
	}

	INPUT_RECORD rec[2] = {};
	rec[0].EventType = KEY_EVENT;
	rec[0].Event.KeyEvent.bKeyDown = TRUE;
	rec[0].Event.KeyEvent.wRepeatCount = 1;
	rec[0].Event.KeyEvent.wVirtualKeyCode = VK_RETURN;
	rec[0].Event.KeyEvent.uChar.UnicodeChar = L'\n';
	rec[1] = rec[0];
	rec[1].Event.KeyEvent.bKeyDown = FALSE;

	DWORD written = 0;
	WriteConsoleInputW(stdin_h, rec, 2, &written);
	CancelIoEx(stdin_h, nullptr);
}

static void prepare_console_for_host_shell()
{
	px4_windows_restore_console_modes();
	px4_windows_discard_pending_input();
	px4_windows_restore_console_modes();
}

static BOOL WINAPI px4_console_ctrl_handler(DWORD ctrl_type)
{
	switch (ctrl_type) {
	case CTRL_C_EVENT:
	case CTRL_BREAK_EVENT:
	case CTRL_CLOSE_EVENT:
	case CTRL_LOGOFF_EVENT:
	case CTRL_SHUTDOWN_EVENT:
		sig_int_handler(SIGINT);
		kick_stdin_reader();
		prepare_console_for_host_shell();
		px4_windows_release_console();
		return TRUE;

	default:
		return FALSE;
	}
}
#endif


#ifdef __PX4_SITL_MAIN_OVERRIDE
int SITL_MAIN(int argc, char **argv);

int SITL_MAIN(int argc, char **argv)
#else
int main(int argc, char **argv)
#endif
{
#if defined(_MSC_VER)
	// Enable CRT debug heap allocation tracking and at-exit leak dump.
	// _CRTDBG_LEAK_CHECK_DF causes _CrtDumpMemoryLeaks() to run automatically
	// on process exit; output is routed to stderr.
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	_CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_FILE);
	_CrtSetReportFile(_CRT_WARN, _CRTDBG_FILE_STDERR);
	_CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_FILE);
	_CrtSetReportFile(_CRT_ERROR, _CRTDBG_FILE_STDERR);
	_CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
	_CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
#endif

	bool is_client = false;
	bool pxh_off = false;
	bool server_is_running = false;

	/* Symlinks point to all commands that can be used as a client with a prefix. */
	const char prefix[] = PX4_SHELL_COMMAND_PREFIX;
	int path_length = 0;

	std::string absolute_binary_path; // full path to the px4 binary being executed

	int ret = PX4_OK;
	int instance = 0;

	if (argc > 0) {
		/* The executed binary name could start with a path, so strip it away */
		const std::string full_binary_name = argv[0];
		const std::string binary_name = file_basename(full_binary_name);

		if (binary_name.compare(0, strlen(prefix), prefix) == 0) {
			is_client = true;
		}

		path_length = full_binary_name.length() - binary_name.length();

		absolute_binary_path = get_absolute_binary_path(full_binary_name);
	}

	if (is_client) {
		if (argc >= 3 && strcmp(argv[1], "--instance") == 0) {
			instance = strtoul(argv[2], nullptr, 10);
			/* update argv so that "--instance <instance>" is not visible anymore */
			argc -= 2;

			for (int i = 1; i < argc; ++i) {
				argv[i] = argv[i + 2];
			}
		}

		PX4_DEBUG("instance: %i", instance);

		ret = get_server_running(instance, &server_is_running);

		if (ret != PX4_OK) {
			PX4_ERR("PX4 client failed to get server status");
			return ret;
		}

		if (!server_is_running) {
			PX4_ERR("PX4 server not running");
			return PX4_ERROR;
		}

		/* Remove the path and prefix. */
		argv[0] += path_length + strlen(prefix);

		px4_daemon::Client client(instance);
		return client.process_args(argc, (const char **)argv);

	} else {
#if (_POSIX_MEMLOCK > 0) && !defined(ENABLE_LOCKSTEP_SCHEDULER)

		// try to lock address space into RAM, to avoid page swap delay
		// TODO: Check CAP_IPC_LOCK instead of euid
		if (geteuid() == 0) {   // root user
			if (mlockall(MCL_CURRENT | MCL_FUTURE)) {	// check if both works
				PX4_ERR("mlockall() failed! errno: %d (%s)", errno, strerror(errno));
				munlockall();	// avoid mlock limitation caused alloc failure in future

			} else {
				PX4_INFO("mlockall() enabled. PX4's virtual address space is locked into RAM.");
			}
		}

#endif // (_POSIX_MEMLOCK > 0) && !ENABLE_LOCKSTEP_SCHEDULER

		/* Server/daemon apps need to parse the command line arguments. */
		std::string data_path{};
		std::string working_directory{};
		std::string test_data_path{};
		std::string commands_file{};

		bool working_directory_default = false;

		bool instance_provided = false;
		bool clean_params_requested = false;

		int myoptind = 1;
		int ch;
		const char *myoptarg = nullptr;

		while ((ch = px4_getopt(argc, argv, "hdct:s:i:w:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'h':
				print_usage();
				return 0;

			case 'd':
				pxh_off = true;
				break;

			case 'c':
				clean_params_requested = true;
				break;

			case 't':
				test_data_path = myoptarg;
				break;

			case 's':
				commands_file = myoptarg;
				break;

			case 'i':
				instance = strtoul(myoptarg, nullptr, 10);
				instance_provided = true;
				break;

			case 'w':
				working_directory = myoptarg;
				break;

			default:
				PX4_ERR("unrecognized flag");
				print_usage();
				return -1;
			}
		}

		if (myoptind < argc) {
			std::string optional_arg = argv[myoptind];

			if (optional_arg.compare(0, 2, "__") != 0 || optional_arg.find(":=") == std::string::npos) {
				data_path = optional_arg;
			} // else: ROS argument (in the form __<name>:=<value>)
		}

		if (instance_provided) {
			PX4_INFO("instance: %i", instance);
		}

#if defined(PX4_INSTALL_PREFIX)

		// When installed as a .deb package, default to the baked-in install prefix.
		// Working directory defaults to XDG_DATA_HOME/px4/rootfs/<instance>.
		if (commands_file.empty() && data_path.empty() && working_directory.empty()
		    && dir_exists(PX4_INSTALL_PREFIX"/etc")
		   ) {
			data_path = PX4_INSTALL_PREFIX"/etc";

			const char *xdg_data_home = getenv("XDG_DATA_HOME");
			std::string state_base;

			if (xdg_data_home) {
				state_base = xdg_data_home;

			} else {
				const char *home = getenv("HOME");
				state_base = std::string(home ? home : "/tmp") + "/.local/share";
			}

			working_directory = state_base + "/px4/rootfs";
			working_directory_default = true;
		}

#endif // PX4_INSTALL_PREFIX

#if defined(PX4_BINARY_DIR)

		// data_path & working_directory: if no commands specified or in current working directory),
		//  rootfs, or working directory specified then default to build directory (if it still exists)
		if (commands_file.empty() && data_path.empty() && working_directory.empty()
		    && dir_exists(PX4_BINARY_DIR"/etc")
		   ) {
			data_path = PX4_BINARY_DIR"/etc";
			working_directory = PX4_BINARY_DIR"/rootfs";

			working_directory_default = true;
		}

#endif // PX4_BINARY_DIR

#if defined(PX4_SOURCE_DIR)

		// test_data_path: default to build source test_data directory (if it exists)
		if (test_data_path.empty() && dir_exists(PX4_SOURCE_DIR"/test_data")) {
			test_data_path = PX4_SOURCE_DIR"/test_data";
		}

#endif // PX4_SOURCE_DIR

		if (commands_file.empty()) {
			commands_file = "etc/init.d-posix/rcS";
		}

		// When `-i N` (N >= 1) is given with `-d <path>/etc` and no `-w`,
		// derive working_directory from data_path's parent so the per-instance
		// subdir (added below) lands under the build dir rather than alongside
		// whatever directory the user happened to launch from.
		// The single-instance default (`-i 0`, no `-i`) keeps the bare-cwd
		// behavior so existing rostest / mavsdk_test_runner scripts that rely
		// on PX4 logging to the launch cwd continue to work; setting -w is
		// still the supported way to relocate state for single-instance runs.
		bool working_directory_from_data_path = false;

		if (instance > 0 && working_directory.empty() && !data_path.empty()) {
			std::string p = data_path;

			if (!p.empty() && (p.back() == '/' || p.back() == '\\')) {
				p.pop_back();
			}

			const size_t last_sep = p.find_last_of("/\\");
			const std::string tail = (last_sep == std::string::npos) ? p : p.substr(last_sep + 1);

			if (tail == "etc") {
				working_directory = (last_sep == std::string::npos) ? std::string(".") : p.substr(0, last_sep);
				working_directory_from_data_path = true;
				PX4_INFO("auto work_dir from data_path: %s", working_directory.c_str());
			}
		}

		// Sister-isolation: when -i N (N >= 1) is given but neither -w nor a
		// platform default (PX4_INSTALL_PREFIX rootfs, PX4_BINARY_DIR/rootfs)
		// supplied a working_directory, derive a per-instance "instance_<N>/"
		// under the launch cwd. Without this, multiple daemons in the same
		// cwd (a common ad-hoc / IDE launch pattern) clobber each other's
		// parameters.bson, dataman, and log/ files: a sister daemon can
		// boot, load another instance's saved parameters (e.g. MAV_SYS_ID),
		// and a mavlink client connecting to the shared port can land
		// arm/mission commands on the wrong vehicle. sitl_multiple_run.sh
		// already does this externally; promoting it to the binary default
		// closes the footgun for direct multi-instance launches.
		//
		// The primary instance (-i 0 or no -i) keeps the bare-cwd behavior
		// to match the historical Linux default for single-daemon dev runs;
		// only -i N >= 1 signals multi-instance intent. Explicit -w still
		// wins for either case. When the work_dir was just derived from
		// data_path (above), still apply the per-instance subdir so
		// `-i 5 -d <build>/etc` lands at `<build>/instance_5/` rather than
		// having all sisters share `<build>/`.
		if (instance > 0 && (working_directory.empty() || working_directory_from_data_path)) {
			if (working_directory.empty()) {
				working_directory = "instance_" + std::to_string(instance);

			} else {
				working_directory += "/instance_" + std::to_string(instance);
			}

			PX4_INFO("auto work_dir: %s (use -w to override)", working_directory.c_str());
		}

		// Anchor user-supplied relative paths to the launch cwd whenever we are
		// about to chdir, so the post-chdir resolution still finds them. This
		// covers both the auto-work_dir case above and explicit -w <work_dir>
		// (which previously left data_path / commands_file cwd-relative and
		// broke the rcS lookup on Windows when the convenience `etc` symlink
		// fell back to PX4_OK without actually creating the link — see
		// create_symlinks_if_needed's EPERM/EACCES branch). Absolute paths and
		// the platform defaults (already absolute) pass through unchanged.
		if (!working_directory.empty()) {
			const std::string launch_cwd = pwd();

			if (!data_path.empty() && !is_absolute_path(data_path)) {
				data_path = launch_cwd + "/" + data_path;
			}

			if (!test_data_path.empty() && !is_absolute_path(test_data_path)) {
				test_data_path = launch_cwd + "/" + test_data_path;
			}

			if (!commands_file.empty() && !is_absolute_path(commands_file)) {
				// Prefer rebasing onto data_path (which now is absolute) so the
				// rcS default resolves even when the work_dir has no `etc`
				// symlink. data_path may itself end in /etc (the typical
				// Windows / multi-instance launcher convention passes
				// <build>/etc as the rootfs), in which case the default
				// commands_file = "etc/init.d-posix/rcS" must drop its
				// leading "etc/" before combining or the result is double-
				// prefixed. Try the as-is form first, then the stripped form,
				// then fall back to launch_cwd for non-default scripts.
				const std::string as_is = !data_path.empty()
							  ? data_path + "/" + commands_file
							  : std::string();

				std::string stripped;

				if (!data_path.empty() && commands_file.compare(0, 4, "etc/") == 0) {
					stripped = data_path + "/" + commands_file.substr(4);
				}

				if (!as_is.empty() && file_exists(as_is)) {
					commands_file = as_is;

				} else if (!stripped.empty() && file_exists(stripped)) {
					commands_file = stripped;

				} else {
					commands_file = launch_cwd + "/" + commands_file;
				}
			}
		}

		// change the CWD befre setting up links and other directories
		if (!working_directory.empty()) {

			// if instance specified, but
			if (instance_provided && working_directory_default) {
				working_directory += "/" + std::to_string(instance);
				PX4_INFO("working directory %s", working_directory.c_str());
			}

			ret = change_directory(working_directory);

			if (ret != PX4_OK) {
				return ret;
			}
		}

		// -c / --clean-params: wipe cached param state from a previous run before
		// the startup script triggers param load. Without this, a stale
		// parameters.bson in the (per-instance) work_dir silently overrides any
		// `param set-default` in the airframe — verification recipes end up
		// testing the OLD tuning. Opt-in: a normal start preserves saved params.
		if (clean_params_requested) {
			PX4_INFO("Clearing cached param state (-c flag)");
			const char *cached[] = {"parameters.bson", "parameters_backup.bson"};

			for (const char *f : cached) {
				if (file_exists(f) && unlink(f) != 0) {
					PX4_WARN("failed to unlink %s: %s", f, strerror(errno));
				}
			}
		}

		ret = get_server_running(instance, &server_is_running);

		if (ret != PX4_OK) {
			PX4_ERR("Failed to get server status");
			return ret;
		}

		if (server_is_running) {
			// allow running multiple instances, but the server is only started for the first
			PX4_INFO("PX4 server already running for instance %i", instance);
			return PX4_ERROR;
		}

		ret = create_symlinks_if_needed(data_path);

		if (ret != PX4_OK) {
			return ret;
		}

		if (test_data_path != "") {
			const std::string required_test_data_path = "./test_data";

			if (!dir_exists(required_test_data_path)) {
				ret = symlink(test_data_path.c_str(), required_test_data_path.c_str());

				if (ret != PX4_OK) {
#ifdef __PX4_WINDOWS

					// See create_symlinks_if_needed(): symlink creation may require
					// Developer Mode / admin on Windows. Treat EACCES/EPERM as
					// non-fatal — tests that need test_data must use the absolute
					// path; the convenience link is best-effort.
					if (errno == EACCES || errno == EPERM) {
						PX4_INFO("symlink %s -> %s not supported (errno=%d), skipping",
							 test_data_path.c_str(), required_test_data_path.c_str(), errno);

					} else
#endif
						return ret;
				}
			}
		}

		if (!file_exists(commands_file)) {
			PX4_ERR("Error opening startup file, does not exist: %s", commands_file.c_str());
			return -1;
		}

		register_sig_handler();
		set_cpu_scaling();

		px4_daemon::Server server(instance);
		server.start();

		ret = create_dirs();

		if (ret != PX4_OK) {
			return ret;
		}

		px4::init_once();
		px4::init(argc, argv, "px4");

		// Don't set this up until PX4 is up and running
		ret = set_server_running(instance);

		if (ret != PX4_OK) {
			return ret;
		}

		ret = run_startup_script(commands_file, absolute_binary_path, instance);

		if (ret == 0) {
			// We now block here until we need to exit.
			if (pxh_off) {
				wait_to_exit();

			} else {
				px4_daemon::Pxh pxh;
				pxh.run_pxh();
			}
		}

		// delete lock
		const std::string file_lock_path = get_lock_file_path(instance);
		int fd_flock = open(file_lock_path.c_str(), O_RDWR, 0666);

		if (fd_flock >= 0) {
			unlink(file_lock_path.c_str());
			flock(fd_flock, LOCK_UN);
			close(fd_flock);
		}

#ifdef __PX4_WINDOWS
		// also remove the companion PID file used for stale-lock detection
		unlink((file_lock_path + ".pid").c_str());
#endif

		if (ret != 0) {
			return PX4_ERROR;
		}

		std::string cmd("shutdown");
		px4_daemon::Pxh::process_line(cmd, true);
#ifdef __PX4_WINDOWS

		// The shutdown command runs asynchronously on the worker queue. While
		// waiting for the worker to terminate the process, keep restoring and
		// draining stdin so Enters typed during shutdown do not leak back into
		// the host Linux shell once Wine returns.
		for (int i = 0; i < 120; ++i) {
			prepare_console_for_host_shell();
			Sleep(50);
		}

		px4_windows_release_console();
		px4_windows_exit(0);
#endif
	}

	return PX4_OK;
}

int create_symlinks_if_needed(std::string &data_path)
{
	std::string current_path = pwd();

	if (data_path.empty()) {
		// No data path given, we'll just try to use the current working dir.
		data_path = current_path;
		PX4_INFO("assuming working directory is rootfs, no symlinks needed.");
		return PX4_OK;
	}

	if (data_path == current_path) {
		// We are already running in the data path, so no need to symlink
		PX4_INFO("working directory seems to be rootfs, no symlinks needed");
		return PX4_OK;
	}

	const std::string path_sym_link = "etc";

	PX4_DEBUG("path sym link: %s", path_sym_link.c_str());

	std::string src_path = data_path;
	std::string dest_path = current_path + "/" + path_sym_link;

	struct stat info;

	if (lstat(dest_path.c_str(), &info) == 0) {
		if (S_ISLNK(info.st_mode)) {
			// recreate the symlink, as it might point to some other location than what we want now
			unlink(dest_path.c_str());

		} else if (S_ISDIR(info.st_mode)) {
			return PX4_OK;
		}

	}

	PX4_DEBUG("Creating symlink %s -> %s\n", src_path.c_str(), dest_path.c_str());

	// create sym-link
	int ret = symlink(src_path.c_str(), dest_path.c_str());

	if (ret != 0) {
#ifdef __PX4_WINDOWS

		// On Windows, creating symlinks requires admin rights, Developer Mode
		// (Win10 1703+), or SE_CREATE_SYMBOLIC_LINK_PRIVILEGE. The windows_shim
		// symlink() already falls back to hard links / recursive copy, but if
		// that also fails (EACCES/EPERM), don't kill PX4: the rest of the boot
		// uses data_path directly. The cwd/etc convenience link is just that.
		if (errno == EACCES || errno == EPERM) {
			PX4_INFO("symlink %s -> %s not supported (errno=%d), using data_path directly",
				 src_path.c_str(), dest_path.c_str(), errno);
			return PX4_OK;
		}

#endif
		PX4_ERR("Error creating symlink %s -> %s", src_path.c_str(), dest_path.c_str());
		return ret;

	} else {
		PX4_DEBUG("Successfully created symlink %s -> %s", src_path.c_str(), dest_path.c_str());
	}

	return PX4_OK;
}

int create_dirs()
{
	std::string current_path = pwd();

	std::vector<std::string> dirs{"log", "eeprom"};

	for (const auto &dir : dirs) {
		PX4_DEBUG("mkdir: %s", dir.c_str());;
		std::string dir_path = current_path + "/" + dir;

		if (dir_exists(dir_path)) {
			continue;
		}

		// create dirs
		int ret = mkdir(dir_path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);

		if (ret != OK) {
			PX4_WARN("failed creating new dir: %s", dir_path.c_str());
			return ret;

		} else {
			PX4_DEBUG("Successfully created dir %s", dir_path.c_str());
		}
	}

	return PX4_OK;
}

void register_sig_handler()
{
#ifdef __PX4_WINDOWS
	// MinGW's signal.h has no sigaction. SIGPIPE does not exist on
	// Windows (closed sockets return WSAECONNRESET instead). Fall back
	// to plain signal() for SIGINT/SIGTERM.
	signal(SIGINT,  sig_int_handler);
	signal(SIGTERM, sig_int_handler);
	SetConsoleCtrlHandler(px4_console_ctrl_handler, TRUE);
#else
	// SIGINT
	struct sigaction sig_int {};
	sig_int.sa_handler = sig_int_handler;
	sig_int.sa_flags = 0; // not SA_RESTART!

	// SIGPIPE
	// We want to ignore if a PIPE has been closed.
	struct sigaction sig_pipe {};
	sig_pipe.sa_handler = SIG_IGN;

#ifdef __PX4_CYGWIN
	// Do not catch SIGINT on Cygwin such that the process gets killed
	// TODO: All threads should exit gracefully see https://github.com/PX4/Firmware/issues/11027
	(void)sig_int; // this variable is unused
#else
	sigaction(SIGINT, &sig_int, nullptr);
#endif

	sigaction(SIGTERM, &sig_int, nullptr);
	sigaction(SIGPIPE, &sig_pipe, nullptr);
#endif // __PX4_WINDOWS
}

void sig_int_handler(int sig_num)
{
	(void)sig_num;

	if (_shutdown_started) {
		return;
	}

	_shutdown_started = 1;
	fflush(stdout);
	printf("\nPX4 Exiting...\n");
	fflush(stdout);
#ifdef __PX4_WINDOWS
	prepare_console_for_host_shell();
	// Lock-file and PID-companion cleanup is left to px4_windows_exit() ->
	// px4_run_exit_unlinks(): doing the close()+unlink() from this handler as
	// well would double-close the registered byte-range lock fd and trip the
	// UCRT debug assertion `(_osfile(fh) & FOPEN)`. The main loop wakes from
	// _exit_requested below and routes through px4_windows_exit() well inside
	// Windows' ~5s CTRL_CLOSE/LOGOFF/SHUTDOWN grace window.
#endif
	uorb_shutdown();
	px4_daemon::Pxh::stop();
	_exit_requested = true;
}

void set_cpu_scaling()
{
#if 0
	system("echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
	system("echo performance > /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor");

	// Alternatively we could also raise the minimum frequency to save some power,
	// unfortunately this still lead to some drops.
	//system("echo 1190400 > /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq");
#endif
}

std::string get_absolute_binary_path(const std::string &argv0)
{
	// On Linux we could also use readlink("/proc/self/exe", buf, bufsize) to get the absolute path

	std::size_t last_slash = argv0.find_last_of("/\\");

	if (last_slash == std::string::npos) {
		// either relative path or in PATH (PATH is ignored here)
		return pwd();
	}

	std::string base = argv0.substr(0, last_slash);

	if (is_absolute_path(base)) {
		// Absolute POSIX path, or an absolute Windows path when running the
		// Windows backend natively / under Wine.
		return base;
	}

	// relative path
	return pwd() + "/" + base;
}

int run_startup_script(const std::string &commands_file, const std::string &absolute_binary_path,
		       int instance)
{
	int ret = px4::run_shell_script(commands_file, absolute_binary_path, instance);

	if (ret == 0) {
		PX4_INFO("Startup script returned successfully");

	} else {
		PX4_ERR("Startup script returned with return value: %d", ret);
	}

	return ret;
}

void wait_to_exit()
{
	while (!_exit_requested) {
		// needs to be a regular sleep not dependent on lockstep (not px4_usleep)
		usleep(100000);
	}
}

void print_usage()
{
	printf("Usage for Server/daemon process: \n");
	printf("\n");
	printf("    px4 [-h|-d|-c] [-s <startup_file>] [-t <test_data_directory>] [<rootfs_directory>] [-i <instance>] [-w <working_directory>]\n");
	printf("\n");
	printf("    -s <startup_file>      shell script to be used as startup (default=etc/init.d-posix/rcS)\n");
	printf("    <rootfs_directory>     directory where startup files and mixers are located,\n");
	printf("                           (if not given, CWD is used)\n");
	printf("    -i <instance>          px4 instance id to run multiple instances [0...N], default=0\n");
	printf("    -w <working_directory> directory to change to\n");
	printf("    -h                     help/usage information\n");
	printf("    -d                     daemon mode, don't start pxh shell\n");
	printf("    -c                     clean cached param state (parameters.bson) before boot;\n");
	printf("                           use to verify airframe `param set-default` values without\n");
	printf("                           a previous run's saved params shadowing them (dev/test)\n");
	printf("\n");
	printf("Usage for client: \n");
	printf("\n");
	printf("    px4-MODULE [--instance <instance>] command using symlink.\n");
	printf("        e.g.: px4-commander status\n");
}

std::string get_lock_file_path(int instance)
{
#ifdef __PX4_WINDOWS
	char temp_path[MAX_PATH + 1] {};
	const DWORD temp_path_length = GetTempPathA(sizeof(temp_path), temp_path);
	std::string lock_dir = (temp_path_length > 0 && temp_path_length < sizeof(temp_path)) ? temp_path : pwd();

	while (!lock_dir.empty() && is_path_separator(lock_dir.back())) {
		lock_dir.pop_back();
	}

	return lock_dir + "\\px4_lock-" + std::to_string(instance);
#else
	return std::string(LOCK_FILE_PATH) + '-' + std::to_string(instance);
#endif
}

int get_server_running(int instance, bool *is_server_running)
{
	const std::string file_lock_path = get_lock_file_path(instance);
	int fd = open(file_lock_path.c_str(), O_RDWR | O_CREAT, 0666);

	if (fd < 0) {
		PX4_ERR("%s: failed to create lock file: %s, reason=%s", __func__, file_lock_path.c_str(), strerror(errno));
		return PX4_ERROR;
	}

	int status = PX4_OK;
	struct flock lock;
	memset(&lock, 0, sizeof(struct flock));

	// Exclusive write lock, cover the entire file (regardless of size)
	lock.l_type = F_WRLCK;
	lock.l_whence = SEEK_SET;

	if (fcntl(fd, F_GETLK, &lock) < 0) {
		PX4_ERR("%s: failed to get check for lock on file: %s, reason=%s", __func__, file_lock_path.c_str(), strerror(errno));
		status = PX4_ERROR;

	} else {
		// F_GETLK will set l_type to F_UNLCK if no one had a lock on the file. Otherwise,
		// it means that the server is running and has a lock on the file
		if (lock.l_type != F_UNLCK) {
			*is_server_running = true;

		} else {
			*is_server_running = false;
		}
	}

#ifdef __PX4_WINDOWS

	// Stale-lock recovery on Windows: a hard kill (taskkill /F, Stop-Process
	// -Force) releases the byte-range lock that the kernel holds on behalf
	// of the process, but the lock file itself survives in %TEMP%. The
	// existing byte-range check above already reports F_UNLCK in that case
	// (the kernel released the lock), so the stale file would otherwise
	// look "free" -- we just need to remove it so set_server_running can
	// recreate it cleanly. When the lock IS held, cross-check the recorded
	// holder PID stored in a side file: if that PID is gone (e.g. the
	// holding process was force-killed but the kernel hasn't fully torn
	// down the handle yet, or a child briefly held the lock), treat the
	// lock as stale.
	if (status == PX4_OK) {
		const std::string pid_path = file_lock_path + ".pid";
		DWORD recorded_pid = 0;
		FILE *pf = fopen(pid_path.c_str(), "r");

		if (pf) {
			char buf[32] = {0};

			if (fgets(buf, sizeof(buf), pf)) {
				recorded_pid = (DWORD)strtoul(buf, nullptr, 10);
			}

			fclose(pf);
		}

		bool process_alive = false;

		if (recorded_pid != 0) {
			HANDLE h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, recorded_pid);

			if (h != nullptr) {
				DWORD exit_code = 0;

				if (GetExitCodeProcess(h, &exit_code) && exit_code == STILL_ACTIVE) {
					process_alive = true;
				}

				CloseHandle(h);
			}
		}

		if (!process_alive) {
			// No live holder: drop any stale files so set_server_running
			// can recreate them cleanly. Safe regardless of *is_server_running:
			// if the byte-range check above reported the lock free, the file
			// is just leftover bytes from a previous run; if it reported the
			// lock held, the kernel will keep enforcing it for whoever still
			// has the handle while we just drop the now-unrelated PID file.
			close(fd);
			unlink(pid_path.c_str());

			if (!*is_server_running) {
				unlink(file_lock_path.c_str());
			}

			return PX4_OK;
		}
	}

#endif // __PX4_WINDOWS

	close(fd);

	return status;
}

int set_server_running(int instance)
{
	const std::string file_lock_path = get_lock_file_path(instance);
	int fd = open(file_lock_path.c_str(), O_RDWR | O_CREAT, 0666);

	if (fd < 0) {
		PX4_ERR("%s: failed to create lock file: %s, reason=%s", __func__, file_lock_path.c_str(), strerror(errno));
		return PX4_ERROR;
	}

	int status = PX4_OK;

	struct flock lock;
	memset(&lock, 0, sizeof(struct flock));

	// Exclusive lock, cover the entire file (regardless of size).
	lock.l_type = F_WRLCK;
	lock.l_whence = SEEK_SET;

	if (fcntl(fd, F_SETLK, &lock) < 0) {
		PX4_ERR("%s: failed to set lock on file: %s, reason=%s", __func__, file_lock_path.c_str(), strerror(errno));
		status = PX4_ERROR;
		close(fd);
	}

#ifdef __PX4_WINDOWS

	// Record the holder PID in a companion file so that a subsequent
	// launch can detect a stale lock left behind by a forced kill. The
	// PID lives outside the lock file because Windows' mandatory
	// byte-range lock would otherwise block readers from inspecting the
	// PID while the lock is held.
	if (status == PX4_OK) {
		const std::string pid_path = file_lock_path + ".pid";
		FILE *pf = fopen(pid_path.c_str(), "w");

		if (pf) {
			fprintf(pf, "%lu\n", (unsigned long)GetCurrentProcessId());
			fclose(pf);
		}

		// Register both the byte-range lock file and its PID companion
		// for unlink at process exit. Required because the graceful
		// `pxh shutdown` path leaves through px4_platform_exit() ->
		// ExitProcess(), bypassing the explicit cleanup at the bottom
		// of main(). Without this, %TEMP% accumulates stale px4_lock-*
		// files that the next launch then has to recover from.
		// Also queue the lock fd for close: Windows blocks unlink() while
		// the handle is open in the same process, so without closing it
		// first the lock file (but not the .pid) would leak. The
		// sig_int_handler() intentionally does NOT close this fd itself
		// to avoid the UCRT double-close assertion on shutdown.
		px4_windows_register_exit_close_fd(fd);
		px4_windows_register_exit_unlink(file_lock_path.c_str());
		px4_windows_register_exit_unlink(pid_path.c_str());
	}

#endif // __PX4_WINDOWS

	// note: server leaks the file handle, on purpose, in order to keep the lock on the file until the process terminates.
	// In this case we return false so the server code path continues now that we have the lock.

	return status;
}

bool file_exists(const std::string &name)
{
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

static std::string file_basename(std::string const &pathname)
{
	struct MatchPathSeparator {
		bool operator()(char ch) const
		{
			return is_path_separator(ch);
		}
	};
	return std::string(std::find_if(pathname.rbegin(), pathname.rend(),
					MatchPathSeparator()).base(), pathname.end());
}

static bool is_path_separator(char ch)
{
	return ch == '/' || ch == '\\';
}

static bool is_absolute_path(const std::string &path)
{
	if (path.empty()) {
		return false;
	}

	if (path[0] == '/') {
		return true;
	}

#ifdef __PX4_WINDOWS
	const bool drive_letter = path.length() >= 3
				  && ((path[0] >= 'A' && path[0] <= 'Z') || (path[0] >= 'a' && path[0] <= 'z'))
				  && path[1] == ':'
				  && is_path_separator(path[2]);

	if (drive_letter) {
		return true;
	}

	// UNC paths begin with two separators, for example \\server\share.
	if (path.length() >= 2 && is_path_separator(path[0]) && is_path_separator(path[1])) {
		return true;
	}

#endif

	return false;
}

bool dir_exists(const std::string &path)
{
	struct stat info;

	if (stat(path.c_str(), &info) != 0) {
		return false;

	} else if (info.st_mode & S_IFDIR) {
		return true;

	}

	return false;
}

std::string pwd()
{
	char temp[PATH_MAX];
	return (getcwd(temp, PATH_MAX) ? std::string(temp) : std::string(""));
}

static int mkdir_p(const std::string &path)
{
	std::string tmp = path;

	for (size_t i = 1; i < tmp.size(); ++i) {
		if (tmp[i] == '/') {
			tmp[i] = '\0';

			if (mkdir(tmp.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) != 0 && errno != EEXIST) {
				return -1;
			}

			tmp[i] = '/';
		}
	}

	if (mkdir(tmp.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) != 0 && errno != EEXIST) {
		return -1;
	}

	return 0;
}

int change_directory(const std::string &directory)
{
	// create directory (including intermediate components)
	if (!dir_exists(directory)) {
		if (mkdir_p(directory) != 0) {
			PX4_ERR("Error creating directory: %s (%s)", directory.c_str(), strerror(errno));
			return -1;
		}
	}

	// change directory
	int ret = chdir(directory.c_str());

	if (ret == -1) {
		PX4_ERR("Error changing current path to: %s (%s)", directory.c_str(), strerror(errno));
		return -1;
	}

	return PX4_OK;
}
