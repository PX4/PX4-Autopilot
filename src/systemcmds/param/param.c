/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file param.c
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Andreas Antener <andreas@uaventure.com>
 *
 * Parameter tool.
 */

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <sys/stat.h>

#include <arch/board/board.h>

#include "systemlib/systemlib.h"
#include "systemlib/param/param.h"
#include "systemlib/err.h"

__EXPORT int param_main(int argc, char *argv[]);

static void	do_save(const char *param_file_name);
static void	do_load(const char *param_file_name);
static void	do_import(const char *param_file_name);
static void	do_show(const char *search_string);
static void	do_show_print(void *arg, param_t param);
static void	do_set(const char *name, const char *val, bool fail_on_not_found);
static void	do_compare(const char *name, char *vals[], unsigned comparisons);
static void	do_reset(const char *excludes[], int num_excludes);
static void	do_reset_nostart(const char *excludes[], int num_excludes);

int
param_main(int argc, char *argv[])
{
	if (argc >= 2) {
		if (!strcmp(argv[1], "save")) {
			if (argc >= 3) {
				do_save(argv[2]);

			} else {
				if (param_save_default()) {
					warnx("Param export failed.");
					exit(1);

				} else {
					exit(0);
				}
			}
		}

		if (!strcmp(argv[1], "load")) {
			if (argc >= 3) {
				do_load(argv[2]);

			} else {
				do_load(param_get_default_file());
			}
		}

		if (!strcmp(argv[1], "import")) {
			if (argc >= 3) {
				do_import(argv[2]);

			} else {
				do_import(param_get_default_file());
			}
		}

		if (!strcmp(argv[1], "select")) {
			if (argc >= 3) {
				param_set_default_file(argv[2]);

			} else {
				param_set_default_file(NULL);
			}

			warnx("selected parameter default file %s", param_get_default_file());
			exit(0);
		}

		if (!strcmp(argv[1], "show")) {
			if (argc >= 3) {
				do_show(argv[2]);

			} else {
				do_show(NULL);
			}
		}

		if (!strcmp(argv[1], "set")) {
			if (argc >= 5) {

				/* if the fail switch is provided, fails the command if not found */
				bool fail = !strcmp(argv[4], "fail");

				do_set(argv[2], argv[3], fail);

			} else if (argc >= 4) {
				do_set(argv[2], argv[3], false);

			} else {
				errx(1, "not enough arguments.\nTry 'param set PARAM_NAME 3 [fail]'");
			}
		}

		if (!strcmp(argv[1], "compare")) {
			if (argc >= 4) {
				do_compare(argv[2], &argv[3], argc - 3);

			} else {
				errx(1, "not enough arguments.\nTry 'param compare PARAM_NAME 3'");
			}
		}

		if (!strcmp(argv[1], "reset")) {
			if (argc >= 3) {
				do_reset((const char **) &argv[2], argc - 2);

			} else {
				do_reset(NULL, 0);
			}
		}

		if (!strcmp(argv[1], "reset_nostart")) {
			if (argc >= 3) {
				do_reset_nostart((const char **) &argv[2], argc - 2);

			} else {
				do_reset_nostart(NULL, 0);
			}
		}
	}

	errx(1, "expected a command, try 'load', 'import', 'show', 'set', 'compare', 'select' or 'save'");
}

static void
do_save(const char *param_file_name)
{
	/* create the file */
	int fd = open(param_file_name, O_WRONLY | O_CREAT);

	if (fd < 0) {
		err(1, "opening '%s' failed", param_file_name);
	}

	int result = param_export(fd, false);
	close(fd);

	if (result < 0) {
		(void)unlink(param_file_name);
		errx(1, "error exporting to '%s'", param_file_name);
	}

	exit(0);
}

static void
do_load(const char *param_file_name)
{
	int fd = open(param_file_name, O_RDONLY);

	if (fd < 0) {
		err(1, "open '%s'", param_file_name);
	}

	int result = param_load(fd);
	close(fd);

	if (result < 0) {
		errx(1, "error importing from '%s'", param_file_name);
	}

	exit(0);
}

static void
do_import(const char *param_file_name)
{
	int fd = open(param_file_name, O_RDONLY);

	if (fd < 0) {
		err(1, "open '%s'", param_file_name);
	}

	int result = param_import(fd);
	close(fd);

	if (result < 0) {
		errx(1, "error importing from '%s'", param_file_name);
	}

	exit(0);
}

static void
do_show(const char *search_string)
{
	printf(" + = saved, * = unsaved\n");
	param_foreach(do_show_print, (char *)search_string, false);

	exit(0);
}

static void
do_show_print(void *arg, param_t param)
{
	int32_t i;
	float f;
	const char *search_string = (const char *)arg;
	const char *p_name = (const char *)param_name(param);

	/* print nothing if search string is invalid and not matching */
	if (!(arg == NULL)) {

		/* start search */
		const char *ss = search_string;
		const char *pp = p_name;

		/* XXX this comparison is only ok for trailing wildcards */
		while (*ss != '\0' && *pp != '\0') {

			if (*ss == *pp) {
				ss++;
				pp++;

			} else if (*ss == '*') {
				if (*(ss + 1) != '\0') {
					warnx("* symbol only allowed at end of search string.");
					exit(1);
				}

				pp++;

			} else {
				/* param not found */
				return;
			}
		}

		/* the search string must have been consumed */
		if (!(*ss == '\0' || *ss == '*')) {
			return;
		}
	}

	printf("%c %s: ",
	       param_value_unsaved(param) ? '*' : (param_value_is_default(param) ? ' ' : '+'),
	       param_name(param));

	/*
	 * This case can be expanded to handle printing common structure types.
	 */

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		if (!param_get(param, &i)) {
			printf("%d\n", i);
			return;
		}

		break;

	case PARAM_TYPE_FLOAT:
		if (!param_get(param, &f)) {
			printf("%4.4f\n", (double)f);
			return;
		}

		break;

	case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
		printf("<struct type %d size %u>\n", 0 + param_type(param), param_size(param));
		return;

	default:
		printf("<unknown type %d>\n", 0 + param_type(param));
		return;
	}

	printf("<error fetching parameter %d>\n", param);
}

static void
do_set(const char *name, const char *val, bool fail_on_not_found)
{
	int32_t i;
	float f;
	param_t param = param_find(name);

	/* set nothing if parameter cannot be found */
	if (param == PARAM_INVALID) {
		/* param not found - fail silenty in scripts as it prevents booting */
		errx(((fail_on_not_found) ? 1 : 0), "Error: Parameter %s not found.", name);
	}

	printf("%c %s: ",
	       param_value_unsaved(param) ? '*' : (param_value_is_default(param) ? ' ' : '+'),
	       param_name(param));

	/*
	 * Set parameter if type is known and conversion from string to value turns out fine
	 */

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		if (!param_get(param, &i)) {

			/* convert string */
			char *end;
			int32_t newval = strtol(val, &end, 10);

			if (i == newval) {
				printf("unchanged\n");

			} else {
				printf("curr: %d", i);
				param_set(param, &newval);
				printf(" -> new: %d\n", newval);
			}
		}

		break;

	case PARAM_TYPE_FLOAT:
		if (!param_get(param, &f)) {

			/* convert string */
			char *end;
			float newval = strtod(val, &end);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

			if (f == newval) {
#pragma GCC diagnostic pop
				printf("unchanged\n");

			} else {
				printf("curr: %4.4f", (double)f);
				param_set(param, &newval);
				printf(" -> new: %4.4f\n", (double)newval);
			}

		}

		break;

	default:
		errx(1, "<unknown / unsupported type %d>\n", 0 + param_type(param));
	}

	exit(0);
}

static void
do_compare(const char *name, char *vals[], unsigned comparisons)
{
	int32_t i;
	float f;
	param_t param = param_find(name);

	/* set nothing if parameter cannot be found */
	if (param == PARAM_INVALID) {
		/* param not found */
		errx(1, "Error: Parameter %s not found.", name);
	}

	/*
	 * Set parameter if type is known and conversion from string to value turns out fine
	 */

	int ret = 1;

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		if (!param_get(param, &i)) {

			/* convert string */
			char *end;

			for (unsigned k = 0; k < comparisons; k++) {

				int j = strtol(vals[k], &end, 10);

				if (i == j) {
					printf(" %d: ", i);
					ret = 0;
				}
			}
		}

		break;

	case PARAM_TYPE_FLOAT:
		if (!param_get(param, &f)) {

			/* convert string */
			char *end;

			for (unsigned k = 0; k < comparisons; k++) {

				float g = strtod(vals[k], &end);

				if (fabsf(f - g) < 1e-7f) {
					printf(" %4.4f: ", (double)f);
					ret = 0;
				}
			}
		}

		break;

	default:
		errx(1, "<unknown / unsupported type %d>\n", 0 + param_type(param));
	}

	if (ret == 0) {
		printf("%c %s: match\n",
		       param_value_unsaved(param) ? '*' : (param_value_is_default(param) ? ' ' : '+'),
		       param_name(param));
	}

	exit(ret);
}

static void
do_reset(const char *excludes[], int num_excludes)
{
	if (num_excludes > 0) {
		param_reset_excludes(excludes, num_excludes);

	} else {
		param_reset_all();
	}

	if (param_save_default()) {
		warnx("Param export failed.");
		exit(1);

	} else {
		exit(0);
	}
}

static void
do_reset_nostart(const char *excludes[], int num_excludes)
{

	int32_t autostart;
	int32_t autoconfig;

	(void)param_get(param_find("SYS_AUTOSTART"), &autostart);
	(void)param_get(param_find("SYS_AUTOCONFIG"), &autoconfig);

	if (num_excludes > 0) {
		param_reset_excludes(excludes, num_excludes);

	} else {
		param_reset_all();
	}

	(void)param_set(param_find("SYS_AUTOSTART"), &autostart);
	(void)param_set(param_find("SYS_AUTOCONFIG"), &autoconfig);

	if (param_save_default()) {
		warnx("Param export failed.");
		exit(1);

	} else {
		exit(0);
	}
}
