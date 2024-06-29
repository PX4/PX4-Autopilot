/****************************************************************************
 *
 *   Copyright (c) 2016-2021 PX4 Development Team. All rights reserved.
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
 * @file head.cpp
 *
 * head is a simple utility for outputting the first part of files
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_hrt.h>

static const unsigned MAX_LINE_LENGTH = 512;

static void print_head(FILE *file, int num_lines)
{
	char buffer[MAX_LINE_LENGTH] {};
	int count = 0;

	while (fgets(buffer, MAX_LINE_LENGTH, file) != nullptr && count < num_lines) {
		printf("%s", buffer);
		count++;
	}

	printf("\n");
}

static void heap_usage()
{
	PRINT_MODULE_DESCRIPTION("Output the first part of files");

	PRINT_MODULE_USAGE_NAME_SIMPLE("head", "command");
	PRINT_MODULE_USAGE_PARAM_INT('l', 10, 1, 1000, "Number of lines", true);
	PRINT_MODULE_USAGE_PARAM_STRING('f', nullptr, nullptr, "file name", false);
}

extern "C" __EXPORT int head_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	unsigned num_lines = 10;
	char file_name[64] {};
	FILE *file = nullptr;

	while ((ch = px4_getopt(argc, argv, "l:f:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'l':
			num_lines = strtol(myoptarg, nullptr, 0);
			break;

		case 'f':
			strncpy(file_name, myoptarg, sizeof(file_name) - 1);
			file = fopen(file_name, "r");

			if (file == nullptr) {
				printf("Error opening file\n");
				return -1;
			}

			break;

		default:
			heap_usage();
			return -1;
			break;
		}
	}

	if (myoptind <= 1) {
		heap_usage();
		return -1;
	}

	print_head(file, num_lines);

	fclose(file);
	return 0;
}
