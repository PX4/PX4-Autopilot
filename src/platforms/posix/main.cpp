/****************************************************************************
 *
 *   Copyright (C) 2015-2016 PX4 Development Team. All rights reserved.
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
#include <fstream>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <px4_log.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_log.h>

#include "apps.h"
#include "px4_middleware.h"
#include "DriverFramework.hpp"
#include "px4_middleware.h"
#include "px4_daemon/client.h"
#include "px4_daemon/server.h"
#include "px4_daemon/pxh.h"


static const char *LOCK_FILE_PATH = "/tmp/px4_lock";

#ifndef PATH_MAX
#define PATH_MAX 1024
#endif


static bool _exit_requested = false;


namespace px4
{
void init_once(void);
}

extern "C" {
	static void _SigIntHandler(int sig_num);
	static void _SigFpeHandler(int sig_num);
}

static void register_sig_handler();
static void set_cpu_scaling();
static void run_startup_bash_script(const char *commands_file);
static void wait_to_exit();
static bool is_already_running();
static void print_usage();

static inline bool fileExists(const std::string &name)
{
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

static inline bool dirExists(const std::string &path)
{
	struct stat info;

	if (stat(path.c_str(), &info) != 0) {
		return false;

	} else if (info.st_mode & S_IFDIR) {
		return true;

	} else {
		return false;
	}
}

static inline void touch(const std::string &name)
{
	std::fstream fs;
	fs.open(name, std::ios::out);
	fs.close();
}

static int mkpath(const char *path, mode_t mode);

static int do_mkdir(const char *path, mode_t mode)
{
	struct stat             st;
	int             status = 0;

	if (stat(path, &st) != 0) {
		/* Directory does not exist. EEXIST for race condition */
		if (mkdir(path, mode) != 0 && errno != EEXIST) {
			status = -1;
		}

	} else if (!S_ISDIR(st.st_mode)) {
		errno = ENOTDIR;
		status = -1;
	}

	return (status);
}

/**
** mkpath - ensure all directories in path exist
** Algorithm takes the pessimistic view and works top-down to ensure
** each directory in path exists, rather than optimistically creating
** the last element and working backwards.
*/
static int mkpath(const char *path, mode_t mode)
{
	char           *pp;
	char           *sp;
	int             status;
	char           *copypath = strdup(path);

	status = 0;
	pp = copypath;

	while (status == 0 && (sp = strchr(pp, '/')) != nullptr) {
		if (sp != pp) {
			/* Neither root nor double slash in path */
			*sp = '\0';
			status = do_mkdir(copypath, mode);
			*sp = '/';
		}

		pp = sp + 1;
	}

	if (status == 0) {
		status = do_mkdir(path, mode);
	}

	free(copypath);
	return (status);
}

static std::string pwd()
{
	char temp[PATH_MAX];
	return (getcwd(temp, PATH_MAX) ? std::string(temp) : std::string(""));
}

#ifdef __PX4_SITL_MAIN_OVERRIDE
int SITL_MAIN(int argc, char **argv);

int SITL_MAIN(int argc, char **argv)
#else
int main(int argc, char **argv)
#endif
{
	bool is_client = false;
	bool pxh_off = false;

	/* Symlinks point to all commands that can be used as a client
	 * with a 'px4-' prefix. */

	const char prefix[] = "px4-";

	if (strstr(argv[0], prefix)) {
		is_client = true;
	}

{
	bool daemon_mode = false;
	bool chroot_on = false;
	if (argc < 2) {
		PX4_ERR("Not enough arguments.");
		print_usage();
		return -1;
	}

	if (is_client) {

		if (!is_already_running()) {
			PX4_ERR("PX4 daemon not running yet");
			return -1;
		}

		/* Remove the prefix. */
		argv[0] += strlen(prefix);

		px4_daemon::Client client;
		client.generate_uuid();
		client.register_sig_handler();
		return client.process_args(argc, (const char **)argv);

	} else {
		/* Server/daemon apps need to parse the command line arguments. */

	int index = 1;
	std::string  commands_file = "";
	int positional_arg_count = 0;
	std::string data_path = "";
	std::string node_name = "";

	// parse arguments
	while (index < argc) {
		for (int i = 1; i < argc; ++i) {
			if (argv[i][0] == '-') {

				if (strcmp(argv[i], "-h") == 0) {
					print_usage();
					return 0;

				} else if (strcmp(argv[i], "-d") == 0) {
					pxh_off = true;

				} else {
					PX4_ERR("Unknown/unhandled parameter: %s", argv[i]);
					print_usage();
					return 1;
				}

			} else if (!strncmp(argv[index], "__", 2)) {
				PX4_DEBUG("ros argument");

				// ros arguments
				if (!strncmp(argv[index], "__name:=", 8)) {
					std::string name_arg = argv[index];
					node_name = name_arg.substr(8);
					PX4_INFO("node name: %s", node_name.c_str());
				}

			} else {
				PX4_DEBUG("positional argument");

				positional_arg_count += 1;

				if (positional_arg_count == 1) {
					data_path = argv[index];

				} else if (positional_arg_count == 2) {
					commands_file = argv[index];
				}
			}
		}

		++index;
	}

	if (positional_arg_count != 2 && positional_arg_count != 1) {
		PX4_ERR("Error expected 1 or 2 position arguments, got %d", positional_arg_count);
		print_usage();
		return -1;
	}

	bool symlinks_needed = true;

	if (positional_arg_count == 1) { //data path is optional
		commands_file = data_path;
		symlinks_needed = false;

	} else {
		PX4_INFO("data path: %s", data_path.c_str());
	}

	PX4_INFO("commands file: %s", commands_file.c_str());

	if (commands_file.empty()) {
		PX4_ERR("Error commands file not specified");
		return -1;
	}

	if (!fileExists(commands_file)) {
		PX4_ERR("Error opening commands file, does not exist: %s", commands_file.c_str());
		return -1;
	}

	// create sym-links
	if (symlinks_needed) {
		std::vector<std::string> path_sym_links;
		path_sym_links.push_back("ROMFS");
		path_sym_links.push_back("posix-configs");
		path_sym_links.push_back("test_data");

		for (int i = 0; i < path_sym_links.size(); i++) {
			std::string path_sym_link = path_sym_links[i];
			PX4_DEBUG("path sym link: %s", path_sym_link.c_str());;
			std::string src_path = data_path + "/" + path_sym_link;
			std::string dest_path =  pwd() + "/" +  path_sym_link;

			PX4_DEBUG("Creating symlink %s -> %s", src_path.c_str(), dest_path.c_str());

			if (dirExists(path_sym_link)) { continue; }

			// create sym-links
			int ret = symlink(src_path.c_str(), dest_path.c_str());

			if (ret != 0) {
				PX4_ERR("Error creating symlink %s -> %s",
					src_path.c_str(), dest_path.c_str());
				return ret;

			} else {
				PX4_DEBUG("Successfully created symlink %s -> %s",
					  src_path.c_str(), dest_path.c_str());
			}
		}
	}

	// setup rootfs
	const std::string eeprom_path = "./rootfs/eeprom/";
	const std::string microsd_path = "./rootfs/fs/microsd/";

	if (!fileExists(eeprom_path + "parameters")) {
		PX4_INFO("creating new parameters file");
		mkpath(eeprom_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
		touch(eeprom_path + "parameters");

		}

		if (is_already_running()) {
			PX4_ERR("PX4 daemon already running");
			return -1;
		}

		px4_daemon::Server server;
		server.start();

		register_sig_handler();
		set_cpu_scaling();

	if (!fileExists(microsd_path + "dataman")) {
		PX4_INFO("creating new dataman file");
		mkpath(microsd_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
		touch(microsd_path + "dataman");
	}

		DriverFramework::Framework::initialize();

		px4::init_once();
		px4::init(argc, argv, "px4");

		run_startup_bash_script(commands_file.c_str());

		// We now block here until we need to exit.
		if (pxh_off) {
			wait_to_exit();

		} else {
			px4_daemon::Pxh pxh;
			pxh.run_pxh();
		}

		// When we exit, we need to stop muorb on Snapdragon.

#ifdef __PX4_POSIX_EAGLE
		// Sending muorb stop is needed if it is running to exit cleanly.
		// TODO: we should check with px4_task_is_running("muorb") before stopping it.
		std::string muorb_stop_cmd("muorb stop");
		px4_daemon::Pxh::process_line(muorb_stop_cmd, true);
#endif

		std::string shutdown_cmd("shutdown");
		px4_daemon::Pxh::process_line(shutdown_cmd, true);

		return OK;
	}
}

static void register_sig_handler()
{
	struct sigaction sig_int;
	memset(&sig_int, 0, sizeof(struct sigaction));
	sig_int.sa_handler = _SigIntHandler;
	sig_int.sa_flags = 0;// not SA_RESTART!;

	struct sigaction sig_fpe;
	memset(&sig_fpe, 0, sizeof(struct sigaction));
	sig_int.sa_handler = _SigFpeHandler;
	sig_int.sa_flags = 0;// not SA_RESTART!;

	// We want to ignore if a PIPE has been closed.
	struct sigaction sig_pipe;
	memset(&sig_pipe, 0, sizeof(struct sigaction));
	sig_pipe.sa_handler = SIG_IGN;

	sigaction(SIGINT, &sig_int, NULL);
	//sigaction(SIGTERM, &sig_int, NULL);
	sigaction(SIGFPE, &sig_fpe, NULL);
	sigaction(SIGPIPE, &sig_pipe, NULL);
}

static void _SigIntHandler(int sig_num)
{
	fflush(stdout);
	printf("\nExiting...\n");
	fflush(stdout);
	px4_daemon::Pxh::stop();
	_exit_requested = true;
}

static void _SigFpeHandler(int sig_num)
{
	fflush(stdout);
	printf("\nfloating point exception\n");
	PX4_BACKTRACE();
	fflush(stdout);
}

static void set_cpu_scaling()
{
#ifdef __PX4_POSIX_EAGLE
	// On Snapdragon we miss updates in sdlog2 unless all 4 CPUs are run
	// at the maximum frequency all the time.
	// Interestingely, cpu0 and cpu3 set the scaling for all 4 CPUs on Snapdragon.
	system("echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
	system("echo performance > /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor");

	// Alternatively we could also raise the minimum frequency to save some power,
	// unfortunately this still lead to some drops.
	//system("echo 1190400 > /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq");
#endif
}

static void run_startup_bash_script(const char *commands_file)
{
	std::string bash_command("bash ");

	bash_command += commands_file;

	PX4_INFO("Calling bash script: %s", bash_command.c_str());

	if (!bash_command.empty()) {
		int ret = system(bash_command.c_str());

		if (ret == 0) {
			PX4_INFO("Startup script returned successfully");

		} else {
			PX4_WARN("Startup script returned with return value: %d", ret);
		}
	}
	PX4_INFO("Startup script empty");
}

static void wait_to_exit()
{
	while (!_exit_requested) {
		usleep(100000);
	}
}

static void print_usage()
{
	printf("Usage for Server/daemon process: \n");
	printf("\n");
	printf("    px4 [-h|-c|-d] [data_directory] startup_file\n");
	printf("\n");
	printf("    <startup_file> bash start script to be used as startup\n");
	printf("    <data_directory> directory where ROMFS and posix-configs are located (if not given, CWD is used)");
	printf("        -h           help/usage information\n");
	printf("        -d           daemon mode, don't start pxh shell\n");
	printf("\n");
	printf("Usage for client: \n");
	printf("\n");
	printf("    px4-MODULE command using symlink.\n");
	printf("        e.g.: px4-commander status\n");
}

static bool is_already_running()
{
	struct flock fl;
	int fd = open(LOCK_FILE_PATH, O_RDWR | O_CREAT, 0666);

	if (fd < 0) {
		return false;
	}

	fl.l_type   = F_WRLCK;
	fl.l_whence = SEEK_SET;
	fl.l_start  = 0;
	fl.l_len    = 0;
	fl.l_pid    = getpid();

	if (fcntl(fd, F_SETLK, &fl) == -1) {
		// We failed to create a file lock, must be already locked.

		if (errno == EACCES || errno == EAGAIN) {
			return true;
		}
	}

	return false;
}

bool px4_exit_requested(void)
{
	return _exit_requested;
}

/* vim: set noet fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
