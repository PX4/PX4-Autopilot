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

using namespace std;

typedef int (*px4_main_t)(int argc, char *argv[]);

#include "apps.h"

// FIXME - the code below only passes 1 arg for now
void run_cmd(const string &command, const string &apparg);
void run_cmd(const string &command, const string &apparg) {
	const char *arg[3];

	if (apps.find(command) != apps.end()) {
		arg[0] = (char *)command.c_str();
		arg[1] = (char *)apparg.c_str();
		arg[2] = (char *)0;
		apps[command](2,(char **)arg);
	}
	else
	{
		cout << "Invalid command" << endl;
	}
}

int main(int argc, char **argv)
{
	string mystr;
	string command;
	string apparg;
	
	// Execute a command list of provided
	if (argc == 2) {
		ifstream infile(argv[1]);

		//vector<string> tokens;

		for (string line; getline(infile, line, '\n'); ) {
  			stringstream(line) >> command >> apparg;
			cout << "Command " << command << ", apparg " << apparg << endl;
			run_cmd(command, apparg);
		}
	}

	while(1) {
		run_cmd("list_builtins", "");

		cout << "Enter a command and its args:" << endl;
		getline (cin,mystr);
  		stringstream(mystr) >> command >> apparg;
		run_cmd(command, apparg);
		mystr = "";
		command = "";
		apparg = "";
	}
}
