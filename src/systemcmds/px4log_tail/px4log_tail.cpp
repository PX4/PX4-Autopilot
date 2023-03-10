/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file px4log_tail.cpp
 * Tool similar to UNIX logger command
 *
 */

#include <px4_platform_common/px4_config.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>

#include <px4_platform/cpuload.h>
#include <px4_platform_common/printload.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>

#include <lib/mathlib/mathlib.h>
#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/topics/log_message.h>

using namespace time_literals;

// static void print_usage()
// {
// 	PRINT_MODULE_DESCRIPTION("Monitor running processes and their CPU, stack usage, priority and state");

// 	PRINT_MODULE_USAGE_NAME_SIMPLE("px4log_tail", "command");
// }

static constexpr const char *__px4_log_level_color[_PX4_LOG_LEVEL_PANIC + 1] {
	PX4_ANSI_COLOR_GREEN,  // DEBUG
	PX4_ANSI_COLOR_RESET,  // INFO
	PX4_ANSI_COLOR_YELLOW, // WARN
	PX4_ANSI_COLOR_RED,    // ERROR
	PX4_ANSI_COLOR_RED     // PANIC
};

extern "C" __EXPORT int px4log_tail_main(int argc, char *argv[])
{
	uORB::SubscriptionBlocking<log_message_s> sub{ORB_ID(log_message)};
	sub.reset();

	while (true) {
		log_message_s log_message;

		if (sub.updateBlocking(log_message, 1_s)) {

			if (log_message.severity == 4) {
				// WARN
				printf("%s%s%s\n", PX4_ANSI_COLOR_YELLOW, log_message.text, PX4_ANSI_COLOR_RESET);

			} else if (log_message.severity == 3) {
				// ERROR
				printf("%s%s%s\n", PX4_ANSI_COLOR_RED, log_message.text, PX4_ANSI_COLOR_RESET);

			} else {
				printf("%s\n", log_message.text);
			}


		}
	}

	return 0;
}
