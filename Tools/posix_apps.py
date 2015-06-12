#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2015 Mark Charlebois. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

import glob
builtins = glob.glob("builtin_commands/COMMAND*")

apps = []
for f in builtins:
	apps.append(f.split(".")[-1].split("_main")[0])

print("""
#include <string>
#include <map>

#define __EXPORT 

#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdlib.h>

using namespace std;

extern void px4_show_devices(void);

extern "C" {
""")
for app in apps:
	print("extern int "+app+"_main(int argc, char *argv[]);")

print("""
static int shutdown_main(int argc, char *argv[]);
static int list_tasks_main(int argc, char *argv[]);
static int list_files_main(int argc, char *argv[]);
static int list_devices_main(int argc, char *argv[]);
static int list_topics_main(int argc, char *argv[]);
static int sleep_main(int argc, char *argv[]);
}


static map<string,px4_main_t> app_map(void);

static map<string,px4_main_t> app_map(void)
{
	static map<string,px4_main_t> apps;
""")
for app in apps:
	print('\tapps["'+app+'"] = '+app+'_main;')

print('\tapps["shutdown"] = shutdown_main;')
print('\tapps["list_tasks"] = list_tasks_main;')
print('\tapps["list_files"] = list_files_main;')
print('\tapps["list_devices"] = list_devices_main;')
print('\tapps["list_topics"] = list_topics_main;')
print('\tapps["sleep"] = sleep_main;')
print("""
	return apps;
}

map<string,px4_main_t> apps = app_map();

static void list_builtins(void)
{
	cout << "Builtin Commands:" << endl;
	for (map<string,px4_main_t>::iterator it=apps.begin(); it!=apps.end(); ++it)
		cout << '\t' << it->first << endl;
}

static int shutdown_main(int argc, char *argv[])
{
	cout << "Shutting down" << endl;
	exit(0);
}

static int list_tasks_main(int argc, char *argv[])
{
	px4_show_tasks();
	return 0;
}

static int list_devices_main(int argc, char *argv[])
{
	px4_show_devices();
	return 0;
}

static int list_topics_main(int argc, char *argv[])
{
	px4_show_topics();
	return 0;
}
static int list_files_main(int argc, char *argv[])
{
	px4_show_files();
	return 0;
}
static int sleep_main(int argc, char *argv[])
{
	if (argc != 2) {
		cout << "Usage: sleep <seconds>" << endl;
		return 1;
	}
	sleep(atoi(argv[1]));
	return 0;
}
""")

