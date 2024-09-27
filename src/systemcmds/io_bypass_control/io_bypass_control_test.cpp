/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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
 * @file io_bypass_control_test.cpp
 *
 * Simple daemon that listens uORB actuator_outputs to control PWM output
 * WARNING: No mixer, hence no safety use at your own risk
 */

#include "io_tester.h"

#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_test.h>

#include <poll.h>

extern "C" __EXPORT int ss_io_timer_test_main(int argc, char *argv[]);

int ss_io_timer_test_main(int argc, char *argv[])
{

	if (argc < 2) {
		PX4_WARN("usage: ss_io_timer {start|test|stop|status}\n");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (IOTester::instance()) {
			PX4_ERR("already started");
			return 1;
		}

		return IOTester::start();
	}

	/* commands below require the app to be started */
	IOTester *const inst = IOTester::instance();

	if (!inst) {
		PX4_ERR("application not running");
		return 1;
	}

	if (!strcmp(argv[1], "stop")) {
		delete inst;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		inst->print_info();
		return 0;
	}

	if (!strcmp(argv[1], "test")) {
		uORB::Publication<actuator_test_s> publisher{ORB_ID(actuator_test)};
		publisher.advertise();

		for (int i = 0; i < 1500; i++) {
			actuator_test_s actuator_test{};
			actuator_test.timestamp = hrt_absolute_time();
			actuator_test.value = i;
			actuator_test.action = actuator_test_s::ACTION_DO_CONTROL;
			actuator_test.timeout_ms = 0;

			for (int j = 0; j < actuator_test_s::MAX_NUM_MOTORS; ++j) {
				actuator_test.function = actuator_test_s::FUNCTION_MOTOR1 + j;
				publisher.publish(actuator_test);
				px4_usleep(100);
			}

			px4_usleep(10000);
		}

		return 0;
	}

	PX4_WARN("usage: ss_io_timer {start|test|stop|status}\n");
	return 1;
}
