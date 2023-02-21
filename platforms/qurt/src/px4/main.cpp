/****************************************************************************
 *
 * Copyright (C) 2022 ModalAI, Inc. All rights reserved.
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

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <vector>
#include <string>
#include <map>
#include <stdio.h>
#include <stdlib.h>

#include "apps.h"

#define MAX_ARGS 8 // max number of whitespace separated args after app name

__BEGIN_DECLS
int slpi_main(int argc, char *argv[]);
__END_DECLS

static void run_cmd(apps_map_type &apps, const std::vector<std::string> &appargs)
{
	// command is appargs[0]
	std::string command = appargs[0];

	//replaces app.find with iterator code to avoid null pointer exception
	for (apps_map_type::iterator it = apps.begin(); it != apps.end(); ++it)
		if (it->first == command) {
			// one for command name, one for null terminator
			const char *arg[MAX_ARGS + 2];

			unsigned int i = 0;

			if (appargs.size() > MAX_ARGS + 1) {
				PX4_ERR("%d too many arguments in run_cmd", appargs.size() - (MAX_ARGS + 1));
				return;
			}

			while (i < appargs.size() && appargs[i].c_str()[0] != '\0') {
				arg[i] = (char *)appargs[i].c_str();
				PX4_DEBUG("  arg%d = '%s'\n", i, arg[i]);
				++i;
			}

			arg[i] = (char *)0;

			//PX4_DEBUG_PRINTF(i);
			if (apps[command] == NULL) {
				PX4_ERR("Null function !!\n");

			} else {
				apps[command](i, (char **)arg);
				break;
			}

		}
}

void eat_whitespace(const char *&b, int &i)
{
	// Eat whitespace
	while (b[i] == ' ' || b[i] == '\t') { ++i; }

	b = &b[i];
	i = 0;
}

static void process_commands(apps_map_type &apps, const char *cmds)
{
	std::vector<std::string> appargs;
	int i = 0;
	const char *b = cmds;
	char arg[256];

	// Eat leading whitespace
	eat_whitespace(b, i);

	for (;;) {
		// End of command line
		if (b[i] == '\n' || b[i] == '\0') {
			strncpy(arg, b, i);
			arg[i] = '\0';
			appargs.push_back(arg);

			// If we have a command to run
			if (appargs.size() > 0) {
				PX4_DEBUG("Processing command: %s", appargs[0].c_str());

				for (int ai = 1; ai < (int)appargs.size(); ai++) {
					PX4_DEBUG("   > arg: %s", appargs[ai].c_str());
				}

				run_cmd(apps, appargs);
			}

			appargs.clear();

			if (b[i] == '\n') {
				eat_whitespace(b, ++i);
				continue;

			} else {
				break;
			}
		}

		// End of arg
		else if (b[i] == ' ') {
			strncpy(arg, b, i);
			arg[i] = '\0';
			appargs.push_back(arg);
			eat_whitespace(b, ++i);
			continue;
		}

		++i;
	}
}


const char *get_commands()
{
	// All that needs to be started automatically on the DSP side
	// are uorb and qshell. After that, everything else can get
	// started from the main startup script on the Linux side.
	static const char *commands = "uorb start\nqshell start\n";

	return commands;
}


int slpi_entry(int argc, char *argv[])
{
	PX4_INFO("Inside slpi_entry");
	sleep(1); // give time for apps side to finish initialization

	apps_map_type apps;
	init_app_map(apps);
	process_commands(apps, get_commands());
	sleep(1); // give time for all commands to execute before starting external function

	for (;;) {
		sleep(1);
	}

	return 0;
}

static void usage()
{
	PX4_INFO("Usage: slpi {start |stop}");
}


extern "C" {

	int slpi_main(int argc, char *argv[])
	{
		int ret = 0;

		if (argc == 2 && strcmp(argv[1], "start") == 0) {
			(void) px4_task_spawn_cmd("slpi",
						  SCHED_DEFAULT,
						  SCHED_PRIORITY_MAX - 5,
						  1500,
						  slpi_entry,
						  argv);

		} else {
			usage();
			ret = -1;
		}

		return ret;
	}
}
