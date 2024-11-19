/****************************************************************************
 *
 * Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * @file px4_getopt.c
 * Minimal, thread safe version of getopt
 */

#include <px4_platform_common/getopt.h>
#include <stdio.h>

// check if p is a valid option and if the option takes an arg
static char isvalidopt(char p, const char *options, int *takesarg)
{
	int idx = 0;
	*takesarg = 0;

	while (options[idx] != 0 && p != options[idx]) {
		++idx;
	}

	if (options[idx] == 0) {
		return '?';
	}

	if (options[idx + 1] == ':') {
		*takesarg = 1;
	}

	return options[idx];
}

// reorder argv and put non-options at the end
static int reorder(int argc, char **argv, const char *options)
{
	char *tmp_argv[argc];
	char c;
	int idx = 1;
	int tmpidx = 1;
	int takesarg;

	// move the options to the front
	while (idx < argc && argv[idx] != 0) {
		if (argv[idx][0] == '-') {
			c = isvalidopt(argv[idx][1], options, &takesarg);

			if (c == '?') {
				return 1;
			}

			tmp_argv[tmpidx] = argv[idx];
			tmpidx++;

			if (takesarg) {
				if (idx + 1 >= argc) { //Error: option takes an argument, but there is no more argument
					return 1;
				}

				tmp_argv[tmpidx] = argv[idx + 1];
				// printf("tmp_argv[%d] = %s\n", tmpidx, tmp_argv[tmpidx]);
				tmpidx++;
				idx++;
			}
		}

		idx++;
	}

	// Add non-options to the end
	idx = 1;

	while (idx < argc && argv[idx] != 0) {
		if (argv[idx][0] == '-') {
			c = isvalidopt(argv[idx][1], options, &takesarg);

			if (c == '?') {
				return c;
			}

			if (takesarg) {
				idx++;
			}

		} else {
			tmp_argv[tmpidx] = argv[idx];
			tmpidx++;
		}

		idx++;
	}

	// Reorder argv
	for (idx = 1; idx < argc; idx++) {
		argv[idx] = tmp_argv[idx];
	}

	return 0;
}

//
// px4_getopt
//
// returns:
//            the valid option character
//            '?' if any option is unknown
//            -1 if no remaining options
//
// If the option takes an arg, myoptarg will be updated accordingly.
// After each call to px4_getopt, myoptind in incremented to the next
// unparsed arg index.
// Argv is changed to put all options and option args at the beginning,
// followed by non-options.
//
__EXPORT int px4_getopt(int argc, char *argv[], const char *options, int *myoptind, const char **myoptarg)
{
	char *p;
	char c;
	int takesarg;

	if (*myoptind == 1) {
		if (reorder(argc, argv, options) != 0) {
			*myoptind += 1;
			return (int)'?';
		}
	}

	p = argv[*myoptind];

	if (*myoptarg == 0) {
		*myoptarg = argv[*myoptind];
	}

	if (p && options && myoptind && p[0] == '-') {
		c = isvalidopt(p[1], options, &takesarg);

		if (c == '?') {
			*myoptind += 1;
			return (int)c;
		}

		*myoptind += 1;

		if (takesarg) {
			*myoptarg = argv[*myoptind];

			if (!*myoptarg) { //Error: option takes an argument, but there is no more argument
				return -1;
			}

			*myoptind += 1;
		}

		return (int)c;
	}

	return -1;
}
