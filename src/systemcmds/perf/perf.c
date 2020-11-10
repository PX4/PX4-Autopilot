/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
#include <px4_platform_common/module.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <perf/perf_counter.h>

__EXPORT int perf_main(int argc, char *argv[]);


static void print_usage(void)
{
	PRINT_MODULE_DESCRIPTION("Tool to print performance counters");

	PRINT_MODULE_USAGE_NAME_SIMPLE("perf", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset all counters");
	PRINT_MODULE_USAGE_COMMAND_DESCR("latency", "Print HRT timer latency histogram");

	PRINT_MODULE_USAGE_PARAM_COMMENT("Prints all performance counters if no arguments given");
}


int perf_main(int argc, char *argv[])
{
	if (argc > 1) {
		if (strcmp(argv[1], "reset") == 0) {
			perf_reset_all();
			return 0;

		} else if (strcmp(argv[1], "latency") == 0) {
			perf_print_latency(1 /* stdout */);
			fflush(stdout);
			return 0;
		}

		print_usage();
		return -1;
	}

	perf_print_all(1 /* stdout */);
	fflush(stdout);
	return 0;
}


