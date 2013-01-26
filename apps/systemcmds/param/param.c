/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
#include <sys/stat.h>

#include <arch/board/board.h>

#include "systemlib/systemlib.h"
#include "systemlib/param/param.h"
#include "systemlib/err.h"

__EXPORT int param_main(int argc, char *argv[]);

static void	do_save(const char* param_file_name);
static void	do_load(const char* param_file_name);
static void	do_import(const char* param_file_name);
static void	do_show(const char* search_string);
static void	do_show_print(void *arg, param_t param);
static void	do_set(const char* name, const char* val);

int
param_main(int argc, char *argv[])
{
	if (argc >= 2) {
		if (!strcmp(argv[1], "save")) {
			if (argc >= 3) {
				do_save(argv[2]);
			} else {
				do_save(param_get_default_file());
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
			if (argc >= 4) {
				do_set(argv[2], argv[3]);
			} else {
				errx(1, "not enough arguments.\nTry 'param set PARAM_NAME 3'");
			}
		}
	}
	
	errx(1, "expected a command, try 'load', 'import', 'show', 'set', 'select' or 'save'");
}

static void
do_save(const char* param_file_name)
{
	/* delete the parameter file in case it exists */
	unlink(param_file_name);

	/* create the file */
	int fd = open(param_file_name, O_WRONLY | O_CREAT | O_EXCL);

	if (fd < 0)
		err(1, "opening '%s' failed", param_file_name);

	int result = param_export(fd, false);
	close(fd);

	if (result < 0) {
		unlink(param_file_name);
		errx(1, "error exporting to '%s'", param_file_name);
	}

	exit(0);
}

static void
do_load(const char* param_file_name)
{
	int fd = open(param_file_name, O_RDONLY);

	if (fd < 0)
		err(1, "open '%s'", param_file_name);

	int result = param_load(fd);
	close(fd);

	if (result < 0) {
		errx(1, "error importing from '%s'", param_file_name);
	}

	exit(0);
}

static void
do_import(const char* param_file_name)
{
	int fd = open(param_file_name, O_RDONLY);

	if (fd < 0)
		err(1, "open '%s'", param_file_name);

	int result = param_import(fd);
	close(fd);

	if (result < 0)
		errx(1, "error importing from '%s'", param_file_name);

	exit(0);
}

static void
do_show(const char* search_string)
{
	printf(" + = saved, * = unsaved\n");
	param_foreach(do_show_print, search_string, false);

	exit(0);
}

static void
do_show_print(void *arg, param_t param)
{
	int32_t i;
	float f;
	const char *search_string = (const char*)arg;

	/* print nothing if search string is invalid and not matching */
	if (!(arg == NULL || (!strcmp(search_string, param_name(param))))) {
		/* param not found */
		return;
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
do_set(const char* name, const char* val)
{
	int32_t i;
	float f;
	param_t param = param_find(name);

	/* set nothing if parameter cannot be found */
	if (param == PARAM_INVALID) {
		/* param not found */
		errx(1, "Error: Parameter %s not found.", name);
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
			printf("old: %d", i);

			/* convert string */
			char* end;
			i = strtol(val,&end,10);
			param_set(param, &i);
			printf(" -> new: %d\n", i);

		}

		break;

	case PARAM_TYPE_FLOAT:
		if (!param_get(param, &f)) {
			printf("float values are not yet supported.");
			// printf("old: %4.4f", (double)f);

			// /* convert string */
			// char* end;
			// f = strtof(val,&end);
			// param_set(param, &f);
			// printf(" -> new: %4.4f\n", f);

		}

		break;

	default:
		errx(1, "<unknown / unsupported type %d>\n", 0 + param_type(param));
	}

	exit(0);
}
