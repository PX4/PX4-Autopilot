/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Vladimir Kulla <ufon@kullaonline.net>
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
 * @file version.c
 *
 * Version nsh command, unified way of showing versions of HW, SW, Build, Toolchain etc
 */

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <version/version.h>

static char sz_ver_hw_str[] = "hw";
static char sz_ver_git_str[] = "git";
static char sz_ver_date_str[] = "date";
static char sz_ver_gcc_str[] = "gcc";
static char sz_ver_all_str[] = "all";


__EXPORT int version_main(int argc, char *argv[]);

static void usage(const char *reason)
{
	if (reason != NULL) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: version {hw|git|date|gcc|all}\n\n");
}

static void version_githash(int bShowPrefix)
{
	if (bShowPrefix == 1) {
		printf("FW git-hash: ");
	}
	printf("%s\n", FW_GIT);
}

static void version_hwarch(int bShowPrefix)
{
	if (bShowPrefix == 1) {
		printf("HW arch: ");
	}
	printf("%s\n", HW_ARCH);
}

static void version_date(int bShowPrefix)
{
	if (bShowPrefix == 1) {
		printf("Build date: ");
	}
	printf("%s %s\n", __DATE__, __TIME__);
}

static void version_gcc(int bShowPrefix)
{
	if (bShowPrefix == 1) {
		printf("GCC used: ");
		//printf("Built with GCC : %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
	}
	printf("%s\n", __VERSION__);
}

int version_main(int argc, char *argv[])
{
	if (argc >= 2)
	{
		if (argv[1] != NULL)
		{
			if (!strncmp(argv[1], sz_ver_hw_str, strlen(sz_ver_hw_str)))
			{
				version_hwarch(0);
			}
			else if (!strncmp(argv[1], sz_ver_git_str, strlen(sz_ver_git_str)))
			{
				version_githash(0);
			}
			else if (!strncmp(argv[1], sz_ver_date_str, strlen(sz_ver_date_str)))
			{
				version_date(0);
			}
			else if (!strncmp(argv[1], sz_ver_gcc_str, strlen(sz_ver_gcc_str)))
			{
				version_gcc(0);
			}
			else if (!strncmp(argv[1], sz_ver_all_str, strlen(sz_ver_all_str)))
			{
				printf("Pixhawk version info\n");
				version_hwarch(1);
				version_date(1);
				version_githash(1);
				version_gcc(1);
			}
			else
			{
				printf("unknown command: %s\n", argv[1]);
			}
		}
		else
		{
			usage("Error, input parameter NULL.\n");
		}
	}
	else
	{
		usage("Error, not enough parameters.");
	}
	return OK;
}
