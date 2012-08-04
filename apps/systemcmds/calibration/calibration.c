/****************************************************************************
 * calibration.c
 *
 *   Copyright (C) 2012 Ivan Ovinnikov. All rights reserved.
 *   Authors: Nils Wenzler <wenzlern@ethz.ch>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "calibration.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int calibrate_help(int argc, char *argv[]);
static int calibrate_all(int argc, char *argv[]);

__EXPORT int calibration_main(int argc, char *argv[]);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct {
	const char 	*name;
	int	(* fn)(int argc, char *argv[]);
	unsigned	options;
#define OPT_NOHELP	(1<<0)
#define OPT_NOALLTEST	(1<<1)
} calibrates[] = {
	{"range",	          range_cal,	0},
	{"servo",	          servo_cal,	0},
	{"all",	          calibrate_all,	OPT_NOALLTEST},
	{"help",          calibrate_help,	OPT_NOALLTEST | OPT_NOHELP},
	{NULL,		NULL}
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int
calibrate_help(int argc, char *argv[])
{
	unsigned	i;

	printf("Available calibration routines:\n");

	for (i = 0; calibrates[i].name; i++)
		printf("  %s\n", calibrates[i].name);

	return 0;
}

static int
calibrate_all(int argc, char *argv[])
{
	unsigned	i;
	char		*args[2] = {"all", NULL};

	printf("Running all calibration routines...\n\n");

	for (i = 0; calibrates[i].name; i++) {
		printf("  %s:\n", calibrates[i].name);

		if (calibrates[i].fn(1, args)) {
			printf("  FAIL\n");

		} else {
			printf("  DONE\n");
		}
	}

	return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void press_enter(void)
{
	int c;
	printf("Press CTRL+ENTER to continue... \n");
	fflush(stdout);

	do c = getchar(); while ((c != '\n') && (c != EOF));
}

/****************************************************************************
 * Name: calibrate_main
 ****************************************************************************/

int calibration_main(int argc, char *argv[])
{
	unsigned	i;

	if (argc < 2) {
		printf("calibration: missing name - 'calibration help' for a list of routines\n");
		return 1;
	}

	for (i = 0; calibrates[i].name; i++) {
		if (!strcmp(calibrates[i].name, argv[1]))
			return calibrates[i].fn(argc - 1, argv + 1);
	}

	printf("calibrate: no routines called '%s' - 'calibration help' for a list of routines\n", argv[1]);
	return 1;
}
