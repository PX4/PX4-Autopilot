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
#include <fstream>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#if (_POSIX_MEMLOCK > 0)
#include <sys/mman.h>
#endif

#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>

#include "apps.h"
#include "px4_daemon/client.h"
#include "px4_daemon/server.h"
#include "px4_daemon/pxh.h"

#define MODULE_NAME "px4"

#ifdef __ANDROID__
static const char *LOCK_FILE_PATH = "/data/local/tmp/px4_lock";
#else
static const char *LOCK_FILE_PATH = "/tmp/px4_lock";
#endif

#ifndef PATH_MAX
#define PATH_MAX 1024
#endif


static volatile bool _exit_requested = false;


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
static int get_server_running(int instance, bool *is_running);
static int set_server_running(int instance);
static void print_usage();
static bool dir_exists(const std::string &path);
static bool file_exists(const std::string &name);
static std::string file_basename(std::string const &pathname);
static std::string pwd();
static int change_directory(const std::string &directory);


#ifdef __PX4_SITL_MAIN_OVERRIDE
int SITL_MAIN(int argc, char **argv);

int SITL_MAIN(int argc, char **argv)
#else
int main(int argc, char **argv)
#endif
{
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

		int myoptind = 1;
		int ch;
		const char *myoptarg = nullptr;

		while ((ch = px4_getopt(argc, argv, "hdt:s:i:w:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'h':
				print_usage();
				return 0;

			case 'd':
				pxh_off = true;
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
		const std::string file_lock_path = std::string(LOCK_FILE_PATH) + '-' + std::to_string(instance);
		int fd_flock = open(file_lock_path.c_str(), O_RDWR, 0666);

		if (fd_flock >= 0) {
			unlink(file_lock_path.c_str());
			flock(fd_flock, LOCK_UN);
			close(fd_flock);
		}

		if (ret != 0) {
			return PX4_ERROR;
		}

		std::string cmd("shutdown");
		px4_daemon::Pxh::process_line(cmd, true);
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
}

void sig_int_handler(int sig_num)
{
	fflush(stdout);
	printf("\nPX4 Exiting...\n");
	fflush(stdout);
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

	std::size_t last_slash = argv0.find_last_of('/');

	if (last_slash == std::string::npos) {
		// either relative path or in PATH (PATH is ignored here)
		return pwd();
	}

	std::string base = argv0.substr(0, last_slash);

	if (base.length() > 0 && base[0] == '/') {
		// absolute path
		return base;
	}

	// relative path
	return pwd() + "/" + base;
}

int run_startup_script(const std::string &commands_file, const std::string &absolute_binary_path,
		       int instance)
{
	std::string shell_command("/bin/sh ");

	shell_command += commands_file + ' ' + std::to_string(instance);

	// Update the PATH variable to include the absolute_binary_path
	// (required for the px4-alias.sh script and px4-* commands).
	// They must be within the same directory as the px4 binary
	const char *path_variable = "PATH";
	std::string updated_path = absolute_binary_path;
	const char *path = getenv(path_variable);

	if (path) {
		std::string spath = path;

		// Check if absolute_binary_path already in PATH
		bool already_in_path = false;
		std::size_t current, previous = 0;
		current = spath.find(':');

		while (current != std::string::npos) {
			if (spath.substr(previous, current - previous) == absolute_binary_path) {
				already_in_path = true;
			}

			previous = current + 1;
			current = spath.find(':', previous);
		}

		if (spath.substr(previous, current - previous) == absolute_binary_path) {
			already_in_path = true;
		}

		if (!already_in_path) {
			// Prepend to path to prioritize PX4 commands over potentially already installed PX4 commands.
			updated_path = updated_path + ":" + path;
			setenv(path_variable, updated_path.c_str(), 1);
		}
	}


	PX4_INFO("startup script: %s", shell_command.c_str());

	int ret = 0;

	if (!shell_command.empty()) {
		ret = system(shell_command.c_str());

		if (ret == 0) {
			PX4_INFO("Startup script returned successfully");

		} else {
			PX4_ERR("Startup script returned with return value: %d", ret);
		}

	} else {
		PX4_INFO("Startup script empty");
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
	printf("    px4 [-h|-d] [-s <startup_file>] [-t <test_data_directory>] [<rootfs_directory>] [-i <instance>] [-w <working_directory>]\n");
	printf("\n");
	printf("    -s <startup_file>      shell script to be used as startup (default=etc/init.d-posix/rcS)\n");
	printf("    <rootfs_directory>     directory where startup files and mixers are located,\n");
	printf("                           (if not given, CWD is used)\n");
	printf("    -i <instance>          px4 instance id to run multiple instances [0...N], default=0\n");
	printf("    -w <working_directory> directory to change to\n");
	printf("    -h                     help/usage information\n");
	printf("    -d                     daemon mode, don't start pxh shell\n");
	printf("\n");
	printf("Usage for client: \n");
	printf("\n");
	printf("    px4-MODULE [--instance <instance>] command using symlink.\n");
	printf("        e.g.: px4-commander status\n");
}

int get_server_running(int instance, bool *is_server_running)
{
	const std::string file_lock_path = std::string(LOCK_FILE_PATH) + '-' + std::to_string(instance);
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

	close(fd);

	return status;
}

int set_server_running(int instance)
{
	const std::string file_lock_path = std::string(LOCK_FILE_PATH) + '-' + std::to_string(instance);
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
			return ch == '/';
		}
	};
	return std::string(std::find_if(pathname.rbegin(), pathname.rend(),
					MatchPathSeparator()).base(), pathname.end());
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

int change_directory(const std::string &directory)
{
	// create directory
	if (!dir_exists(directory)) {
		int ret = mkdir(directory.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);

		if (ret == -1) {
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
