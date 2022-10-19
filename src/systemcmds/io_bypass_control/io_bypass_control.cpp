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
 * @file io_bypass_control.cpp
 *
 * Simple daemon that listens uORB actuator_outputs to control PWM output
 * WARNING: No mixer, hence no safety use at your own risk
 */

#include "io_controller.hpp"

#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_outputs.h>

#include <poll.h>

extern "C" __EXPORT int io_bypass_control_main(int argc, char *argv[]);


void print_help()
{
	PX4_WARN("usage: io_bypass_control {start|test|stop|rate|status}");
	printf("Simple daemon that listens uORB actuator_outputs to control PWM output\n"
	       "WARNING: No mixer, hence no safety use at your own risk\n"
	       "Useful for full offboard control using RTPS.\n");
}

int io_bypass_control_main(int argc, char *argv[])
{

	if (argc < 2 || !strcmp(argv[1], "help")) {
		print_help();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (IOController::instance()) {
			PX4_ERR("already started");
			return 1;
		}

		return IOController::start();
	}

	/* commands below require the app to be started */
	IOController *const inst = IOController::instance();

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
		uORB::Publication<actuator_outputs_s> publisher{ORB_ID(actuator_outputs)};
		publisher.advertise();

		for (float i = -1; i < 1; i = i + 0.01f) {
			actuator_outputs_s actuator_outputs{};
			actuator_outputs.timestamp = hrt_absolute_time();
			actuator_outputs.noutputs = DIRECT_PWM_OUTPUT_CHANNELS;

			for (int j = 0; j < DIRECT_PWM_OUTPUT_CHANNELS; ++j) {
				actuator_outputs.output[j] = i;
			}

			publisher.publish(actuator_outputs);
			px4_usleep(10000);
		}

		return 0;
	}

	if (!strcmp(argv[1], "rate")) {
		inst->setRate(atoi(argv[2]));
		return 0;
	}

	print_help();
	return 1;
}
