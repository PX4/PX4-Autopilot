/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * Basic shell to execute builtin "apps"
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 * @author Roman Bapst <bapstroman@gmail.com>
 */

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include "apps.h"
#include "px4_middleware.h"
#include "px4_posix.h"
#include "px4_log.h"
#include "DriverFramework.hpp"
#include <termios.h>
#include <sys/stat.h>

namespace px4
{
void init_once();
}

using namespace std;

typedef int (*px4_main_t)(int argc, char *argv[]);

#define CMD_BUFF_SIZE	100

#ifdef PATH_MAX
const unsigned path_max_len = PATH_MAX;
#else
const unsigned path_max_len = 1024;
#endif

static bool _ExitFlag = false;

static struct termios orig_term;

extern "C" {
	void _SigIntHandler(int sig_num);
	void _SigIntHandler(int sig_num)
	{
		cout.flush();
		cout << endl << "Exiting..." << endl;
		cout.flush();
		_ExitFlag = true;
	}
	void _SigFpeHandler(int sig_num);
	void _SigFpeHandler(int sig_num)
	{
		cout.flush();
		cout << endl << "floating point exception" << endl;
		PX4_BACKTRACE();
		cout.flush();
	}
}

static inline bool fileExists(const string &name)
{
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

static inline bool dirExists(const string &path)
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

static inline void touch(const string &name)
{
	fstream fs;
	fs.open(name, ios::out);
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

static string pwd()
{
	char temp[path_max_len];
	return (getcwd(temp, path_max_len) ? string(temp) : string(""));
}

static void print_prompt()
{
	cout.flush();
	cout << "pxh> ";
	cout.flush();
}

static void run_cmd(const vector<string> &appargs, bool exit_on_fail, bool silently_fail = false)
{
	static apps_map_type apps;
	static bool initialized = false;

	if (!initialized) {
		init_app_map(apps);
		initialized = true;
	}

	// command is appargs[0]
	string command = appargs[0];

	if (apps.find(command) != apps.end()) {
		const char *arg[appargs.size() + 2];

		unsigned int i = 0;

		while (i < appargs.size() && appargs[i] != "") {
			arg[i] = (char *)appargs[i].c_str();
			++i;
		}

		arg[i] = (char *)nullptr;

		int retval = apps[command](i, (char **)arg);

		if (retval) {
			cout << "Command '" << command << "' failed, returned " << retval << endl;

			if (exit_on_fail && retval) {
				exit(retval);
			}
		}

	} else if (command == "help") {
		list_builtins(apps);

	} else if (command.length() == 0 || command[0] == '#') {
		// Do nothing

	} else if (!silently_fail) {
		cout << "Invalid command: " << command << "\ntype 'help' for a list of commands" << endl;

	}
}

static void usage()
{

	cout << "./px4 [-d] [data_directory] startup_config [-h]" << endl;
	cout << "   -d            - Optional flag to run the app in daemon mode and does not listen for user input." <<
	     endl;
	cout << "                   This is needed if px4 is intended to be run as a upstart job on linux" << endl;
	cout << "<data_directory> - directory where ROMFS and posix-configs are located (if not given, CWD is used)" << endl;
	cout << "<startup_config> - config file for starting/stopping px4 modules" << endl;
	cout << "   -h            - help/usage information" << endl;
}

static void process_line(string &line, bool exit_on_fail)
{
	vector<string> appargs(20);

	stringstream(line) >> appargs[0] >> appargs[1] >> appargs[2] >> appargs[3] >> appargs[4] >> appargs[5] >> appargs[6] >>
			   appargs[7] >> appargs[8] >> appargs[9] >> appargs[10] >> appargs[11] >> appargs[12] >> appargs[13] >>
			   appargs[14] >> appargs[15] >> appargs[16] >> appargs[17] >> appargs[18] >> appargs[19];
	run_cmd(appargs, exit_on_fail);
}

static void restore_term()
{
	cout << "Restoring terminal\n";
	tcsetattr(0, TCSANOW, &orig_term);
}

bool px4_exit_requested(void)
{
	return _ExitFlag;
}

static void set_cpu_scaling()
{
#if defined(__PX4_POSIX_EAGLE) || defined(__PX4_POSIX_EXCELSIOR)
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

#ifdef __PX4_SITL_MAIN_OVERRIDE
int SITL_MAIN(int argc, char **argv);

int SITL_MAIN(int argc, char **argv)
#else
int main(int argc, char **argv)
#endif
{
	bool daemon_mode = false;
	bool chroot_on = false;

	tcgetattr(0, &orig_term);
	atexit(restore_term);

	struct sigaction sig_int;
	memset(&sig_int, 0, sizeof(struct sigaction));
	sig_int.sa_handler = _SigIntHandler;
	sig_int.sa_flags = 0;// not SA_RESTART!;

	struct sigaction sig_fpe;
	memset(&sig_fpe, 0, sizeof(struct sigaction));
	sig_fpe.sa_handler = _SigFpeHandler;
	sig_fpe.sa_flags = 0;// not SA_RESTART!;

	sigaction(SIGINT, &sig_int, nullptr);
	//sigaction(SIGTERM, &sig_int, NULL);
	sigaction(SIGFPE, &sig_fpe, nullptr);

	set_cpu_scaling();

	int index = 1;
	string  commands_file;
	int positional_arg_count = 0;
	string data_path;
	string node_name;

	// parse arguments
	while (index < argc) {
		//cout << "arg: " << index << " : " << argv[index] << endl;

		if (argv[index][0] == '-') {
			// the arg starts with -
			if (strncmp(argv[index], "-d", 2) == 0) {
				daemon_mode = true;

			} else if (strncmp(argv[index], "-h", 2) == 0) {
				usage();
				return 0;

			} else if (strncmp(argv[index], "-c", 2) == 0) {
				chroot_on = true;

			} else {
				PX4_ERR("Unknown/unhandled parameter: %s", argv[index]);
				return 1;
			}

		} else if (!strncmp(argv[index], "__", 2)) {
			//cout << "ros argument" << endl;

			// ros arguments
			if (!strncmp(argv[index], "__name:=", 8)) {
				string name_arg = argv[index];
				node_name = name_arg.substr(8);
				cout << "node name: " << node_name << endl;
			}

		} else {
			//cout << "positional argument" << endl;

			positional_arg_count += 1;

			if (positional_arg_count == 1) {
				data_path = argv[index];

			} else if (positional_arg_count == 2) {
				commands_file = argv[index];
			}
		}

		++index;
	}

	if (positional_arg_count != 2 && positional_arg_count != 1) {
		PX4_ERR("Error expected 1 or 2 position arguments, got %d", positional_arg_count);
		usage();
		return -1;
	}

	bool symlinks_needed = true;

	if (positional_arg_count == 1) { //data path is optional
		commands_file = data_path;
		symlinks_needed = false;

	} else {
		cout << "data path: " << data_path << endl;
	}

	cout << "commands file: " << commands_file << endl;

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
		vector<string> path_sym_links;
		path_sym_links.push_back("ROMFS");
		path_sym_links.push_back("posix-configs");
		path_sym_links.push_back("test_data");

		for (int i = 0; i < path_sym_links.size(); i++) {
			string path_sym_link = path_sym_links[i];
			//cout << "path sym link: " << path_sym_link << endl;
			string src_path = data_path + "/" + path_sym_link;
			string dest_path =  pwd() + "/" +  path_sym_link;

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
	const string eeprom_path = "./rootfs/eeprom/";
	const string microsd_path = "./rootfs/fs/microsd/";

	if (!fileExists(eeprom_path + "parameters")) {
		cout << "creating new parameters file" << endl;
		mkpath(eeprom_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
		touch(eeprom_path + "parameters");
	}

	if (!fileExists(microsd_path + "dataman")) {
		cout << "creating new dataman file" << endl;
		mkpath(microsd_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
		touch(microsd_path + "dataman");
	}

	// initialize
	DriverFramework::Framework::initialize();
	px4::init_once();

	px4::init(argc, argv, "px4");

	// if commandfile is present, process the commands from the file
	if (!commands_file.empty()) {
		ifstream infile(commands_file.c_str());

		if (infile.is_open()) {
			for (string line; getline(infile, line, '\n');) {

				if (px4_exit_requested()) {
					break;
				}

				// TODO: this should be true but for that we have to check all startup files
				process_line(line, false);
			}

		} else {
			PX4_ERR("Error opening commands file: %s", commands_file.c_str());
		}
	}

	if (chroot_on) {
		// Lock this application in the current working dir
		// this is not an attempt to secure the environment,
		// rather, to replicate a deployed file system.

		char pwd_path[path_max_len];
		const char *folderpath = "/rootfs/";

		if (nullptr == getcwd(pwd_path, sizeof(pwd_path))) {
			PX4_ERR("Failed acquiring working dir, abort.");
			exit(1);
		}

		if (nullptr == strcat(pwd_path, folderpath)) {
			PX4_ERR("Failed completing path, abort.");
			exit(1);
		}

		if (chroot(pwd_path)) {
			PX4_ERR("Failed chrooting application, path: %s, error: %s.", pwd_path, strerror(errno));
			exit(1);
		}

		if (chdir("/")) {
			PX4_ERR("Failed changing to root dir, path: %s, error: %s.", pwd_path, strerror(errno));
			exit(1);
		}
	}

	if (!daemon_mode) {
		string mystr;
		string string_buffer[CMD_BUFF_SIZE];
		int buf_ptr_write = 0;
		int buf_ptr_read = 0;

		print_prompt();

		// change input mode so that we can manage shell
		struct termios term;
		tcgetattr(0, &term);
		term.c_lflag &= ~ICANON;
		term.c_lflag &= ~ECHO;
		tcsetattr(0, TCSANOW, &term);
		setbuf(stdin, nullptr);

		while (!_ExitFlag) {

			char c = getchar();

			switch (c) {
			case 127:	// backslash
				if (mystr.length() > 0) {
					mystr.pop_back();
					printf("%c[2K", 27);	// clear line
					cout << (char)13;
					print_prompt();
					cout << mystr;
				}

				break;

			case'\n':	// user hit enter
				if (buf_ptr_write == CMD_BUFF_SIZE) {
					buf_ptr_write = 0;
				}

				if (buf_ptr_write > 0) {
					if (!mystr.empty() && mystr != string_buffer[buf_ptr_write - 1]) {
						string_buffer[buf_ptr_write] = mystr;
						buf_ptr_write++;
					}

				} else {
					if (!mystr.empty() && mystr != string_buffer[CMD_BUFF_SIZE - 1]) {
						string_buffer[buf_ptr_write] = mystr;
						buf_ptr_write++;
					}
				}

				cout << endl;
				process_line(mystr, false);
				mystr = "";
				buf_ptr_read = buf_ptr_write;

				print_prompt();
				break;

			case '\033': {	// arrow keys
					c = getchar();	// skip first one, does not have the info
					c = getchar();

					// arrow up
					if (c == 'A') {
						buf_ptr_read--;
						// arrow down

					} else if (c == 'B') {
						if (buf_ptr_read < buf_ptr_write) {
							buf_ptr_read++;
						}

					} else {
						// TODO: Support editing current line
					}

					if (buf_ptr_read < 0) {
						buf_ptr_read = 0;
					}

					string saved_cmd = string_buffer[buf_ptr_read];
					printf("%c[2K", 27);
					cout << (char)13;
					mystr = saved_cmd;
					print_prompt();
					cout << mystr;
					break;
				}

			default:	// any other input
				if (c > 3) {
					cout << c;
					mystr += c;
				}

				break;
			}
		}

	} else {
		while (!_ExitFlag) {
			usleep(100000);
		}
	}

	// TODO: Always try to stop muorb for QURT because px4_task_is_running doesn't seem to work.
	if (true) {
		//if (px4_task_is_running("muorb")) {
		// sending muorb stop is needed if it is running to exit cleanly
		vector<string> muorb_stop_cmd = { "muorb", "stop" };
		run_cmd(muorb_stop_cmd, !daemon_mode, true);
	}

	vector<string> shutdown_cmd = { "shutdown" };
	run_cmd(shutdown_cmd, true);
	DriverFramework::Framework::shutdown();

	return OK;
}

/* vim: set noet fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
