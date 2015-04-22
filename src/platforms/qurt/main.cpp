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

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <hexagon_standalone.h>

//using namespace std;

//typedef int (*px4_main_t)(int argc, char *argv[]);

#include "apps.h"
//#include "px4_middleware.h"

static const char *commands = "hello start\n";

static void run_cmd(const vector<string> &appargs) {
	// command is appargs[0]
	string command = appargs[0];
	printf("Looking for %s\n", command.c_str());
	if (apps.find(command) != apps.end()) {
		const char *arg[appargs.size()+2];

		unsigned int i = 0;
		while (i < appargs.size() && appargs[i] != "") {
			arg[i] = (char *)appargs[i].c_str();
			printf("  arg = '%s'\n", arg[i]);
			++i;
		}
		arg[i] = (char *)0;
		apps[command](i,(char **)arg);
	}
	else
	{
		cout << "Invalid command" << endl;
		list_builtins();
	}
}

static void process_commands(const char *cmds)
{
	vector<string> appargs(5);
	int i=0;
	int j=0;
	const char *b = cmds;
	bool found_first_char = false;
	char arg[20];

	// Eat leading whitespace
	while (b[i] == ' ') { ++i; };
	b = &b[i];

	for(;;) {
		// End of command line
		if (b[i] == '\n' || b[i] == '\0') {
			strncpy(arg, b, i);
			arg[i] = '\0';
			appargs[j] = arg;

			// If we have a command to run
			if (i > 0 || j > 0)
				run_cmd(appargs);
			j=0;
			if (b[i] == '\n') {
				// Eat whitespace
				while (b[++i] == ' ');
				b = &b[i];
				i=0;
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
			appargs[j] = arg;
			j++;
			// Eat whitespace
			while (b[++i] == ' ');
			b = &b[i];
			i=0;
			continue;
		}
		++i;
	}
}

int main(int argc, char **argv)
{
	process_commands(commands);
	for (;;) {}
}
