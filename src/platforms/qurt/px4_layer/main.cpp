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
 */

#include <px4_middleware.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <px4_posix.h>
#include <px4_defines.h>
#include <vector>
#include <string>
#include <map>
#include <stdio.h>

using namespace std;

extern void init_app_map(map<string,px4_main_t> &apps);
extern void list_builtins(map<string,px4_main_t> &apps);
static px4_task_t g_dspal_task = -1;

__BEGIN_DECLS
// The commands to run are specified in a target file: commands_<target>.c
extern const char *get_commands(void);
// Enable external library hook
void qurt_external_hook(void) __attribute__((weak));
__END_DECLS

static void run_cmd(map<string,px4_main_t> &apps, const vector<string> &appargs) {
	// command is appargs[0]
	string command = appargs[0];
	if (apps.find(command) != apps.end()) {
		const char *arg[2+1];

		unsigned int i = 0;
		while (i < appargs.size() && appargs[i].c_str()[0] != '\0') {
			arg[i] = (char *)appargs[i].c_str();
			PX4_WARN("  arg = '%s'\n", arg[i]);
			++i;
		}
		arg[i] = (char *)0;
		//PX4_DEBUG_PRINTF(i);
		apps[command](i,(char **)arg);
	}
	else
	{
		PX4_WARN("NOT FOUND.");
		list_builtins(apps);
	}
}

void eat_whitespace(const char *&b, int &i)
{
	// Eat whitespace
	while (b[i] == ' ' || b[i] == '\t') { ++i; }
	b = &b[i];
	i=0;
}

static void process_commands(map<string,px4_main_t> &apps, const char *cmds)
{
	vector<string> appargs;
	int i=0;
	const char *b = cmds;
	bool found_first_char = false;
	char arg[256];

	// This is added because it is a parameter used by commander, yet created by mavlink.  Since mavlink is not
        // running on QURT, we need to manually define it so it is available to commander.  "2" is for quadrotor.

    // Following is hack to prevent duplicate parameter definition error in param parser
    /**
     * @board QuRT_App
     */
	PARAM_DEFINE_INT32(MAV_TYPE,2);

	// Eat leading whitespace
	eat_whitespace(b, i);


	for(;;) {
		// End of command line
		if (b[i] == '\n' || b[i] == '\0') {
			strncpy(arg, b, i);
			arg[i] = '\0';
			appargs.push_back(arg);

			// If we have a command to run
			if (appargs.size() > 0) {
			  PX4_WARN("Processing command: %s",appargs[0].c_str());
			  for(int ai=1;ai<(int)appargs.size();ai++)
   			         PX4_WARN("   > arg: %s",appargs[ai].c_str());
			  run_cmd(apps, appargs);
			}
			appargs.clear();
			if (b[i] == '\n') {
				eat_whitespace(b, ++i);
				continue;
			}
			else {
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

namespace px4 {
extern void init_once(void);
};

__BEGIN_DECLS
extern int dspal_main(int argc, char *argv[]);
__END_DECLS


int dspal_entry( int argc, char* argv[] )
{
	PX4_INFO("In main\n");
	map<string,px4_main_t> apps;
	init_app_map(apps);
	px4::init_once();
	px4::init(argc, (char **)argv, "mainapp");
	process_commands(apps, get_commands());
	usleep( 1000000 ); // give time for all commands to execute before starting external function
	if(qurt_external_hook)
	{
		qurt_external_hook();
	}
	for( ;; ){
		usleep( 1000000 );
	}
        return 0;
}

static void usage()
{
	PX4_WARN("Usage: dspal {start |stop}");
}


extern "C" {

int dspal_main(int argc, char *argv[])
{
	int ret = 0;
	if (argc == 2 && strcmp(argv[1], "start") == 0) {
			g_dspal_task = px4_task_spawn_cmd("dspal",
				SCHED_DEFAULT,
				SCHED_PRIORITY_MAX - 5,
				1500,
				dspal_entry,
				argv);

	}
	else if (argc == 2 && strcmp(argv[1], "stop") == 0) {
		if (g_dspal_task < 0) {
			PX4_WARN("start up thread not running");
		}
		else {
			px4_task_delete(g_dspal_task);
			g_dspal_task = -1;
		}
	}
	else {
		usage();
		ret = -1;
	}

	return ret;
}
}
