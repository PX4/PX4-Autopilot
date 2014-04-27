/****************************************************************************
*
* Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
* used to endorse or promote products derived from this software
* without specific prior written permission.
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
* @file ver.c
*
* Version command, unifies way of showing versions of HW, SW, Build, gcc
* In case you want to add new version just extend version_main function
*
* External use of the version functions is possible, include "vercmd.h"
* and use functions from header with prefix ver_
*
* @author Vladimir Kulla <ufon@kullaonline.net>
*/

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <version/version.h>

#include "ver.h"

// string constants for version commands
static const char sz_ver_hw_str[] 	= "hw";
static const char sz_ver_hwcmp_str[]= "hwcmp";
static const char sz_ver_git_str[] 	= "git";
static const char sz_ver_date_str[] = "date";
static const char sz_ver_gcc_str[] 	= "gcc";
static const char sz_ver_all_str[] 	= "all";

static void usage(const char *reason)
{
	if (reason != NULL) {
		printf("%s\n", reason);
	}

	printf("usage: version {hw|hwcmp|git|date|gcc|all}\n\n");
}

void ver_githash(int bShowPrefix)
{
	if (bShowPrefix == 1) {
		printf("FW git-hash: ");
	}
	printf("%s\n", FW_GIT);
}

void ver_hwarch(int bShowPrefix)
{
	if (bShowPrefix == 1) {
		printf("HW arch: ");
	}
	printf("%s\n", HW_ARCH);
}

void ver_date(int bShowPrefix)
{
	if (bShowPrefix == 1) {
		printf("Build date: ");
	}
	printf("%s %s\n", __DATE__, __TIME__);
}

void ver_gcclong(int bShowPrefix)
{
	if (bShowPrefix == 1) {
		printf("GCC used (long): ");
		//printf("Built with GCC : %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
	}
	printf("%s\n", __VERSION__);
}

void ver_gccshort(int bShowPrefix)
{
	if (bShowPrefix == 1) {
		printf("GCC used (short): ");

	}
	printf("%d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
}

int ver_gccnum()
{
	return (__GNUC__ * 1000) + (__GNUC_MINOR__ * 100) + __GNUC_PATCHLEVEL__;
}

__EXPORT int ver_main(int argc, char *argv[]);

int ver_main(int argc, char *argv[])
{
	int ret = 1;	//defaults to an error

	// first check if there are at least 2 params
	if (argc >= 2) {
		if (argv[1] != NULL) {
			if (!strncmp(argv[1], sz_ver_hw_str, strlen(sz_ver_hw_str))) {
				ver_hwarch(0);
				ret = 0;
			}
			else if (!strncmp(argv[1], sz_ver_hwcmp_str, strlen(sz_ver_hwcmp_str))) {
				if (argc >= 3 && argv[2] != NULL) {
					// compare 3rd parameter with HW_ARCH string, in case of match, return 0
					ret = strcmp(HW_ARCH, argv[2]) != 0;
					if (ret == 0) {
						printf("hw_ver match: %s\n", HW_ARCH);
					}
				} else {
					errx(1, "not enough arguments, try 'version hwcmp PX4FMU_1'");
				}
			}
			else if (!strncmp(argv[1], sz_ver_git_str, strlen(sz_ver_git_str))) {
				ver_githash(0);
				ret = 0;
			}
			else if (!strncmp(argv[1], sz_ver_date_str, strlen(sz_ver_date_str))) {
				ver_date(0);
				ret = 0;
			}
			else if (!strncmp(argv[1], sz_ver_gcc_str, strlen(sz_ver_gcc_str))) {
				ver_gcclong(0);
				ret = 0;
			}
			else if (!strncmp(argv[1], sz_ver_all_str, strlen(sz_ver_all_str))) {
				printf("Pixhawk version info\n");
				ver_hwarch(1);
				ver_date(1);
				ver_githash(1);
				ver_gcclong(1);
				ret = 0;
			}
			else {
				errx(1, "unknown command.\n");
			}
		}
		else {
			usage("Error, input parameter NULL.\n");
		}
	}
	else {
		usage("Error, not enough parameters.");
	}

	return ret;
}
