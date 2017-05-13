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

#include <px4_config.h>
#include <px4_getopt.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/test_motor.h>


#include "systemlib/systemlib.h"
#include "systemlib/err.h"


__EXPORT int motor_test_main(int argc, char *argv[]);

static void motor_test(unsigned channel, float value);
static void usage(const char *reason);

static orb_advert_t _test_motor_pub = NULL;

void motor_test(unsigned channel, float value)
{
	struct test_motor_s _test_motor;

	_test_motor.motor_number = channel;
	_test_motor.timestamp = hrt_absolute_time();
	_test_motor.value = value;

	if (_test_motor_pub != NULL) {
		/* publish test state */
		orb_publish(ORB_ID(test_motor), _test_motor_pub, &_test_motor);

	} else {
		/* advertise and publish */
		_test_motor_pub = orb_advertise_queue(ORB_ID(test_motor), &_test_motor, 4);
	}

	printf("motor %d set to %.2f\n", channel, (double)value);
}

static void usage(const char *reason)
{
	if (reason != NULL) {
		warnx("%s", reason);
	}

	errx(1,
	     "usage:\n"
	     "motor_test\n"
	     "    -m <channel>            Motor to test (0..7), all if -m not given\n"
	     "    -p <power>              Power (0..100), 0 if -p not given\n"
	     "motor_test stop             Stop all motors\n"
	     "motor_test iterate          Iterate all motors starting and stopping one after the other\n");
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
			}

			value = ((float)lval) / 100.f;
			break;

		default:
			usage(NULL);
		}
	}

	bool run_test = true;

	if (argc > 1) {
		if (strcmp("stop", argv[1]) == 0) {
			channel = -1;
			value = 0.f;

		} else if (strcmp("iterate", argv[1]) == 0) {
			value = 0.3f;

			for (int i = 0; i < 8; ++i) {
				motor_test(i, value);
				usleep(500000);
				motor_test(i, 0.f);
				usleep(10000);
			}

			run_test = false;
		}
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

	exit(0);
}
