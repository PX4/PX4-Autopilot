/****************************************************************************
*
* Copyright (c) 2023 Technology Innovation Institute. All rights reserved.
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
* @file rdfeat.c
*
* rdfeat command, used to check RD feature flags on console, command line or
* scripts
*
* @author Jukka Laitinen <jukka.laitinen@tii.ae>
*/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <stdlib.h>
#include <stdio.h>

#ifdef BOARD_RD_FEATURE_FLAGS
static const char cmp_str[]   = "cmp";
static const char print_str[] = "print";

static void usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_INFO_RAW("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("Tool to check R&D feature flag on command line");

	PRINT_MODULE_USAGE_NAME("rdfeat", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("cmp",
					 "Check a specific flag (string). Returns 0 on match. Otherwise returns 1");
	PRINT_MODULE_USAGE_ARG("flag", "Flag to check", "");
	PRINT_MODULE_USAGE_COMMAND_DESCR("print", "Outputs all possible flags and the current setting");
}
#endif

extern "C" __EXPORT int rdfeat_main(int argc, char *argv[])
{
#ifndef BOARD_RD_FEATURE_FLAGS
	return 0;
#else
	int ret = 1;
	unsigned long flags = BOARD_RD_FEATURE_FLAGS;
	const size_t num_flags = sizeof(rdct_feat_strings) / sizeof(rdct_feat_strings[0]);

	if (argc < 2 || argc > 3) {
		usage("Invalid number of arguments");
		return ret;
	}

	if (!strncmp(argv[1], print_str, sizeof(print_str))) {
		for (size_t i = 0; i < num_flags; i++) {
			PX4_INFO("%s: %s\n", rdct_feat_strings[i].s, (rdct_feat_strings[i].v & flags) != 0 ? "Enabled" : "Disabled");
			ret = 0;
		}
	}

	else if (argc == 3 && !strncmp(argv[1], cmp_str, sizeof(cmp_str))) {
		for (size_t i = 0; i < num_flags; i++) {
			if (!strcmp(rdct_feat_strings[i].s, argv[2]) && (rdct_feat_strings[i].v & flags) != 0) {
				ret = 0;
			}
		}

	} else {
		usage("Invalid command");
	}

	return ret;
#endif
}
