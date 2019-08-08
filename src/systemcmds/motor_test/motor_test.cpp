/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *   Author: Holger Steinhaus <hsteinhaus@gmx.de>
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
 * @file motor_test.c
 *
 * Tool for drive testing
 */

#include <drivers/drv_hrt.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_module.h>
#include <uORB/PublicationQueued.hpp>
#include <uORB/topics/test_motor.h>

extern "C" __EXPORT int motor_test_main(int argc, char *argv[]);

static void motor_test(unsigned channel, float value);
static void usage(const char *reason);

void motor_test(unsigned channel, float value)
{
	test_motor_s test_motor{};
	test_motor.timestamp = hrt_absolute_time();
	test_motor.motor_number = channel;
	test_motor.value = value;

	uORB::PublicationQueued<test_motor_s> test_motor_pub{ORB_ID(test_motor)};
	test_motor_pub.publish(test_motor);

	PX4_INFO("motor %d set to %.2f", channel, (double)value);
}

static void usage(const char *reason)
{
	if (reason != NULL) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Utility to test motors.\n"
				 "\n"
				 "Note: this can only be used for drivers which support the motor_test uorb topic (currently uavcan and tap_esc)\n"
				);

	PRINT_MODULE_USAGE_NAME("motor_test", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Set motor(s) to a specific output value");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 7, "Motor to test (0...7, all if not specified)", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 100, "Power (0...100)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop all motors");
	PRINT_MODULE_USAGE_COMMAND_DESCR("iterate", "Iterate all motors starting and stopping one after the other");

}

int motor_test_main(int argc, char *argv[])
{
	int channel = -1; //default to all channels
	unsigned long lval;
	float value = 0.0f;
	int ch;

	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "m:p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'm':
			/* Read in motor number */
			channel = (int)strtol(myoptarg, NULL, 0);
			break;

		case 'p':
			/* Read in power value */
			lval = strtoul(myoptarg, NULL, 0);

			if (lval > 100) {
				usage("value invalid");
				return 1;
			}

			value = ((float)lval) / 100.f;
			break;

		default:
			usage(NULL);
			return 1;
		}
	}

	bool run_test = true;

	if (myoptind >= 0 && myoptind < argc) {
		if (strcmp("stop", argv[myoptind]) == 0) {
			channel = -1;
			value = 0.f;

		} else if (strcmp("iterate", argv[myoptind]) == 0) {
			value = 0.3f;

			for (int i = 0; i < 8; ++i) {
				motor_test(i, value);
				usleep(500000);
				motor_test(i, 0.f);
				usleep(10000);
			}

			run_test = false;

		} else if (strcmp("test", argv[myoptind]) == 0) {
			// nothing to do
		} else {
			usage(NULL);
			return 0;
		}

	} else {
		usage(NULL);
		return 0;
	}

	if (run_test) {
		if (channel < 0) {
			for (int i = 0; i < 8; ++i) {
				motor_test(i, value);
				usleep(10000);
			}

		} else {
			motor_test(channel, value);
		}
	}

	return 0;
}
