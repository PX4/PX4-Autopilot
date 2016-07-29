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
#builtins = glob.glob("../Build/linux_default.build/builtin_commands/COMMAND*")
builtins = glob.glob("builtin_commands/COMMAND*")

apps = []
for f in builtins:
	apps.append(f.split(".")[-1].split("_main")[0])

print
print """
#include <string>
#include <map>
#include <stdio.h>

#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_log.h>
#include <stdlib.h>

using namespace std;

extern void px4_show_devices(void);

extern "C" {
"""
for app in apps:
	print "extern int "+app+"_main(int argc, char *argv[]);"

print """
static int shutdown_main(int argc, char *argv[]);
static int list_tasks_main(int argc, char *argv[]);
static int list_files_main(int argc, char *argv[]);
static int list_devices_main(int argc, char *argv[]);
static int list_topics_main(int argc, char *argv[]);
static int sleep_main(int argc, char *argv[]);
}


void init_app_map(map<string,px4_main_t> &apps)
{
"""
for app in apps:
	print '\tapps["'+app+'"] = '+app+'_main;'

print '\tapps["shutdown"] = shutdown_main;'
print '\tapps["list_tasks"] = list_tasks_main;'
print '\tapps["list_files"] = list_files_main;'
print '\tapps["list_devices"] = list_devices_main;'
print '\tapps["list_topics"] = list_topics_main;'
print '\tapps["sleep"] = sleep_main;'

print """
}

void list_builtins(map<string,px4_main_t> &apps)
{
	printf("Builtin Commands:\\n");
	for (map<string,px4_main_t>::iterator it=apps.begin(); it!=apps.end(); ++it)
		printf("\\t%s\\n", (it->first).c_str());
}

static int shutdown_main(int argc, char *argv[])
{
	printf("Shutting down\\n");
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
           PX4_WARN( "Usage: sleep <seconds>" );
           return 1;
        }

        unsigned long usecs = ( (unsigned long) atol( argv[1] ) ) * 1000 * 1000;
        PX4_WARN("Sleeping for %s, %ld",argv[1],usecs);
        usleep( usecs );
        return 0;
}
"""

