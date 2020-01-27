/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
#include <px4_platform_common/console_buffer.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>

#ifndef BOARD_ENABLE_CONSOLE_BUFFER
#error "This module can only be used on boards that enable BOARD_ENABLE_CONSOLE_BUFFER"
#endif

static void	usage();

extern "C" {
	__EXPORT int dmesg_main(int argc, char *argv[]);
}

int
dmesg_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool follow = false;

	while ((ch = px4_getopt(argc, argv, "f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'f':
			follow = true;
			break;

		default:
			usage();
			return -1;
			break;
		}
	}

	px4_console_buffer_print(follow);

	return 0;
}

static void
usage()
{

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Command-line tool to show bootup console messages.
Note that output from NuttX's work queues and syslog are not captured.

### Examples

Keep printing all messages in the background:
$ dmesg -f &
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dmesg", "system");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Follow: wait for new messages", true);

}
