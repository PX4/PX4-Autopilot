/****************************************************************************
 *
 *  Copyright (C) 2015-2021 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

__BEGIN_DECLS

extern int test_microbench_atomic(int argc, char *argv[]);
extern int test_microbench_hrt(int argc, char *argv[]);
extern int test_microbench_math(int argc, char *argv[]);
extern int test_microbench_matrix(int argc, char *argv[]);
extern int test_microbench_uorb(int argc, char *argv[]);

__END_DECLS

static int microbench_help(int argc, char *argv[]);
static int microbench_all(int argc, char *argv[]);
static int microbench_runner(unsigned option);

const struct {
	const char 	*name;
	int	(* fn)(int argc, char *argv[]);
	unsigned	options;
#define OPT_NOHELP	(1<<0)
#define OPT_NOALLTEST	(1<<1)
} microbenchmarks[] = {
	{"help",		microbench_help,		OPT_NOALLTEST | OPT_NOHELP},
	{"all",		microbench_all,		OPT_NOALLTEST},

	{"microbench_atomic",	test_microbench_atomic,	0},
	{"microbench_hrt",	test_microbench_hrt,	0},
	{"microbench_math",	test_microbench_math,	0},
	{"microbench_matrix",	test_microbench_matrix,	0},
	{"microbench_uorb",	test_microbench_uorb,	0},

	{nullptr,			nullptr, 		0}
};

#define NMICROBENCHMARKS (sizeof(microbenchmarks) / sizeof(microbenchmarks[0]))

static int microbench_help(int argc, char *argv[])
{
	printf("Available tests:\n");

	for (int i = 0; microbenchmarks[i].name; i++) {
		printf("  %s\n", microbenchmarks[i].name);
	}

	return 0;
}

static int microbench_all(int argc, char *argv[])
{
	return microbench_runner(OPT_NOALLTEST);
}

static int microbench_runner(unsigned option)
{
	size_t i;
	char *args[2] = {"all", nullptr};
	unsigned int failcount = 0;
	unsigned int testcount = 0;
	unsigned int passed[NMICROBENCHMARKS];

	printf("\nRunning all microbenchmarks...\n\n");

	for (i = 0; microbenchmarks[i].name; i++) {
		// Only run tests that are not excluded.
		if (!(microbenchmarks[i].options & option)) {
			for (int j = 0; j < 80; j++) {
				printf("-");
			}

			printf("\n  [%s] \t\tSTARTING TEST\n", microbenchmarks[i].name);
			fflush(stdout);

			/* Execute test */
			if (microbenchmarks[i].fn(1, args) != 0) {
				fprintf(stderr, "  [%s] \t\tFAIL\n", microbenchmarks[i].name);
				fflush(stderr);
				failcount++;
				passed[i] = 0;

			} else {
				printf("  [%s] \t\tPASS\n", microbenchmarks[i].name);
				fflush(stdout);
				passed[i] = 1;
			}

			for (int j = 0; j < 80; j++) {
				printf("-");
			}

			testcount++;
			printf("\n\n");
		}
	}

	for (size_t k = 0; k < i; k++) {
		if (!passed[k] && !(microbenchmarks[k].options & option)) {
			printf(" [%s] to obtain details, please re-run with\n\t nsh> microbench %s\n\n", microbenchmarks[k].name,
			       microbenchmarks[k].name);
		}
	}

	return 0;
}

extern "C" __EXPORT int microbench_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_WARN("missing test name - 'microbench help' for a list of tests");
		return 1;
	}

	for (size_t i = 0; microbenchmarks[i].name; i++) {
		if (!strcmp(microbenchmarks[i].name, argv[1])) {
			if (microbenchmarks[i].fn(argc - 1, argv + 1) == 0) {
				PX4_INFO("%s PASSED", microbenchmarks[i].name);
				return 0;

			} else {
				PX4_ERR("%s FAILED", microbenchmarks[i].name);
				return -1;
			}
		}
	}

	PX4_WARN("no test called '%s' - 'microbench help' for a list of tests", argv[1]);
	return 1;
}
