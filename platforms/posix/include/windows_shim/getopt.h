/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * @file getopt.h
 *
 * Minimal getopt/getopt_long shim for native Windows builds.
 */

#pragma once

#if defined(_MSC_VER)
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define no_argument 0
#define required_argument 1
#define optional_argument 2

/** @brief getopt_long() option descriptor. */
struct option {
	const char *name;
	int has_arg;
	int *flag;
	int val;
};

static char *optarg = NULL;
static int optind = 1;
static int opterr = 1;
static int optopt = 0;
static int px4_getopt_nextchar = 1;

/**
 * @brief Parse short options from argv.
 *
 * This is a compact implementation sufficient for PX4 command-line parsers in
 * native MSVC builds. MinGW uses its system getopt implementation.
 */
static inline int getopt(int argc, char *const argv[], const char *optstring)
{
	optarg = NULL;

	if (optind >= argc || !argv[optind] || argv[optind][0] != '-' || argv[optind][1] == '\0') {
		return -1;
	}

	if (strcmp(argv[optind], "--") == 0) {
		++optind;
		px4_getopt_nextchar = 1;
		return -1;
	}

	const char opt = argv[optind][px4_getopt_nextchar];
	const char *match = strchr(optstring, opt);
	optopt = opt;

	if (!match) {
		if (argv[optind][++px4_getopt_nextchar] == '\0') {
			++optind;
			px4_getopt_nextchar = 1;
		}

		return '?';
	}

	if (match[1] == ':') {
		if (argv[optind][px4_getopt_nextchar + 1] != '\0') {
			optarg = &argv[optind][px4_getopt_nextchar + 1];
			++optind;
			px4_getopt_nextchar = 1;

		} else if (optind + 1 < argc) {
			optarg = argv[++optind];
			++optind;
			px4_getopt_nextchar = 1;

		} else {
			++optind;
			px4_getopt_nextchar = 1;
			return (optstring[0] == ':') ? ':' : '?';
		}

	} else {
		if (argv[optind][++px4_getopt_nextchar] == '\0') {
			++optind;
			px4_getopt_nextchar = 1;
		}
	}

	return opt;
}

/**
 * @brief Parse long and short options from argv.
 *
 * Supports exact long-option matches and required arguments. Optional long
 * arguments are accepted as source-compatible descriptors but treated like
 * no-argument options unless supplied with '='.
 */
static inline int getopt_long(int argc, char *const argv[], const char *optstring,
			      const struct option *longopts, int *longindex)
{
	if (optind < argc && argv[optind] && strncmp(argv[optind], "--", 2) == 0 && argv[optind][2] != '\0') {
		const char *name = argv[optind] + 2;
		const char *value = strchr(name, '=');
		const size_t name_len = value ? (size_t)(value - name) : strlen(name);

		for (int i = 0; longopts && longopts[i].name; ++i) {
			if (strncmp(name, longopts[i].name, name_len) == 0 && longopts[i].name[name_len] == '\0') {
				if (longindex) {
					*longindex = i;
				}

				if (longopts[i].has_arg == required_argument) {
					if (value) {
						optarg = (char *)(value + 1);

					} else if (optind + 1 < argc) {
						optarg = argv[++optind];

					} else {
						++optind;
						return '?';
					}

				} else {
					optarg = NULL;
				}

				++optind;

				if (longopts[i].flag) {
					*longopts[i].flag = longopts[i].val;
					return 0;
				}

				return longopts[i].val;
			}
		}
	}

	return getopt(argc, argv, optstring);
}

#ifdef __cplusplus
}
#endif

#else
#include_next <getopt.h>
#endif
