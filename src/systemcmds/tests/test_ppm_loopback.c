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

/**
 * @file test_ppm_loopback.c
 * Tests the PWM outputs and PPM input
 *
 */

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_rc_input.h>
#include <systemlib/err.h>

#include "tests.h"

#include <math.h>
#include <float.h>

int test_ppm_loopback(int argc, char *argv[])
{

	int servo_fd, result;
	servo_position_t data[PWM_OUTPUT_MAX_CHANNELS];
	servo_position_t pos;

	servo_fd = open(PWM_OUTPUT_DEVICE_PATH, O_RDWR);

	if (servo_fd < 0) {
		printf("failed opening /dev/pwm_servo\n");
	}

	result = read(servo_fd, &data, sizeof(data));

	if (result != sizeof(data)) {
		printf("failed bulk-reading channel values\n");
	}

	printf("Servo readback, pairs of values should match defaults\n");

	unsigned servo_count;
	result = ioctl(servo_fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	if (result != OK) {
		warnx("PWM_SERVO_GET_COUNT");
		return ERROR;
	}

	for (unsigned i = 0; i < servo_count; i++) {
		result = ioctl(servo_fd, PWM_SERVO_GET(i), (unsigned long)&pos);

		if (result < 0) {
			printf("failed reading channel %u\n", i);
		}

		printf("%u: %u %u\n", i, pos, data[i]);

	}

	/* tell safety that its ok to disable it with the switch */
	result = ioctl(servo_fd, PWM_SERVO_SET_ARM_OK, 0);
	if (result != OK)
		warnx("FAIL: PWM_SERVO_SET_ARM_OK");
	/* tell output device that the system is armed (it will output values if safety is off) */
	result = ioctl(servo_fd, PWM_SERVO_ARM, 0);
	if (result != OK)
		warnx("FAIL: PWM_SERVO_ARM");

	int pwm_values[] = {1200, 1300, 1900, 1700, 1500, 1250, 1800, 1400};


	printf("Advancing channel 0 to 1100\n");
	result = ioctl(servo_fd, PWM_SERVO_SET(0), 1100);
	printf("Advancing channel 1 to 1900\n");
	result = ioctl(servo_fd, PWM_SERVO_SET(1), 1900);
	printf("Advancing channel 2 to 1200\n");
	result = ioctl(servo_fd, PWM_SERVO_SET(2), 1200);

	for (unsigned i = 0; (i < servo_count) && (i < sizeof(pwm_values) / sizeof(pwm_values[0])); i++) {
		result = ioctl(servo_fd, PWM_SERVO_SET(i), pwm_values[i]);
		if (result) {
			(void)close(servo_fd);
			return ERROR;
		}
	}

	/* give driver 10 ms to propagate */
	usleep(10000);

	/* open PPM input and expect values close to the output values */

	int ppm_fd = open(RC_INPUT_DEVICE_PATH, O_RDONLY);

	struct rc_input_values rc;
	result = read(ppm_fd, &rc, sizeof(rc));

	if (result != sizeof(rc)) {
		warnx("Error reading RC output");
		(void)close(servo_fd);
		(void)close(ppm_fd);
		return ERROR;
	}

	/* go and check values */
	for (unsigned i = 0; (i < servo_count) && (i < sizeof(pwm_values) / sizeof(pwm_values[0])); i++) {
		result = ioctl(servo_fd, PWM_SERVO_GET(i), pwm_values[i]);
		if (result) {
			(void)close(servo_fd);
			return ERROR;
		}
	}

	return 0;
}
