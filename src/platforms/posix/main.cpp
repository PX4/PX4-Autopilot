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

namespace px4 {
void init_once(void);
}

using namespace std;

typedef int (*px4_main_t)(int argc, char *argv[]);

#include "apps.h"
#include "px4_middleware.h"

static void run_cmd(const vector<string> &appargs) {
	// command is appargs[0]
	string command = appargs[0];
	cout << "----------------------------------\n";
	if (apps.find(command) != apps.end()) {
		const char *arg[appargs.size()+2];

		unsigned int i = 0;
		while (i < appargs.size() && appargs[i] != "") {
			arg[i] = (char *)appargs[i].c_str();
			++i;
		}
		arg[i] = (char *)0;
		cout << "Running: " << command << "\n";
		apps[command](i,(char **)arg);
		usleep(40000);
		cout << "Returning: " << command << "\n";

	} else {
		cout << "Invalid command: " << command << endl;
		list_builtins();
	}
}

static void process_line(string &line)
{
	vector<string> appargs(8);

	stringstream(line) >> appargs[0] >> appargs[1] >> appargs[2] >> appargs[3] >> appargs[4] >> appargs[5] >> appargs[6] >> appargs[7];
	run_cmd(appargs);
}

int main(int argc, char **argv)
{
	px4::init_once();

	px4::init(argc, argv, "mainapp");

	// Execute a command list of provided
	if (argc == 2) {
		ifstream infile(argv[1]);

		if (!infile) {
			cout << "failed opening script" << argv[1] << std::endl;
			return 1;
		}

		for (string line; getline(infile, line, '\n'); ) {
			process_line(line);
		}
	}

	string mystr;
	
	while(1) {
		cout << "Enter a command and its args:" << endl;
		getline (cin,mystr);
		process_line(mystr);
		mystr = "";
	}
}
