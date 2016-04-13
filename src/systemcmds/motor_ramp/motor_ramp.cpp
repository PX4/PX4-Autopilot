/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file motor_ramp.cpp
 * Application to test motor ramp up.
 *
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>

#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <platforms/px4_defines.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"

static bool _thread_should_exit = false;		/**< motor_ramp exit flag */
static bool _thread_running = false;		/**< motor_ramp status flag */
static int _motor_ramp_task;				/**< Handle of motor_ramp task / thread */
static float _ramp_time;
static int _min_pwm;

enum RampState {
	RAMP_INIT,
	RAMP_MIN,
	RAMP_RAMP,
	RAMP_WAIT
};

/**
 * motor_ramp management function.
 */
extern "C" __EXPORT int motor_ramp_main(int argc, char *argv[]);

/**
 * Mainloop of motor_ramp.
 */
int motor_ramp_thread_main(int argc, char *argv[]);

bool min_pwm_valid(unsigned pwm_value);

int set_min_pwm(int fd, unsigned long max_channels, unsigned pwm_value);

int set_out(int fd, unsigned long max_channels, float output);

int prepare(int fd, unsigned long *max_channels);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		PX4_WARN("[motor_ramp] %s", reason);
	}

	PX4_WARN("[motor_ramp] usage:   motor_ramp <min_pwm> <ramp_time>");
	PX4_WARN("[motor_ramp] example: motor_ramp 1100 0.5\n");
}

/**
 * The motor_ramp app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int motor_ramp_main(int argc, char *argv[])
{
	if (argc < 3) {
		usage("missing parameters");
		return 1;
	}

	if (_thread_running) {
		PX4_WARN("motor_ramp already running\n");
		/* this is not an error */
		return 0;
	}

	_min_pwm = atoi(argv[1]);

	if (!min_pwm_valid(_min_pwm)) {
		usage("min pwm not in range");
		return 1;
	}

	_ramp_time = atof(argv[2]);

	if (!(_ramp_time > 0)) {
		usage("ramp time must be greater than 0");
		return 1;
	}

	_thread_should_exit = false;
	_motor_ramp_task = px4_task_spawn_cmd("motor_ramp",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_DEFAULT,
					      2000,
					      motor_ramp_thread_main,
					      (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
	return 0;

	usage("unrecognized command");
	return 1;
}

bool min_pwm_valid(unsigned pwm_value)
{
	return pwm_value > 900 && pwm_value < 1300;
}

int set_min_pwm(int fd, unsigned long max_channels, unsigned pwm_value)
{
	int ret;

	struct pwm_output_values pwm_values;
	memset(&pwm_values, 0, sizeof(pwm_values));

	pwm_values.channel_count = max_channels;

	for (int i = 0; i < max_channels; i++) {
		pwm_values.values[i] = pwm_value;
	}

	ret = px4_ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values);

	if (ret != OK) {
		PX4_ERR("[motor_ramp] failed setting min values");
		return 1;
	}

	return 0;
}

int set_out(int fd, unsigned long max_channels, float output)
{
	int ret;
	int pwm = (2000 - _min_pwm) * output + _min_pwm;

	for (unsigned i = 0; i < max_channels; i++) {

		ret = ioctl(fd, PWM_SERVO_SET(i), pwm);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_SET(%d), value: %d", i, pwm);
			return 1;
		}
	}

	return 0;
}

int prepare(int fd, unsigned long *max_channels)
{
	/* get number of channels available on the device */
	if (px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)max_channels) != OK) {
		PX4_ERR("[motor_ramp] PWM_SERVO_GET_COUNT");
		return 1;
	}

	/* tell IO/FMU that its ok to disable its safety with the switch */
	if (px4_ioctl(fd, PWM_SERVO_SET_ARM_OK, 0) != OK) {
		PX4_ERR("[motor_ramp] PWM_SERVO_SET_ARM_OK");
		return 1;
	}

	/* tell IO/FMU that the system is armed (it will output values if safety is off) */
	if (px4_ioctl(fd, PWM_SERVO_ARM, 0) != OK) {
		PX4_ERR("[motor_ramp] PWM_SERVO_ARM");
		return 1;
	}

	/* tell IO to switch off safety without using the safety switch */
	if (px4_ioctl(fd, PWM_SERVO_SET_FORCE_SAFETY_OFF, 0) != OK) {
		PX4_ERR("[motor_ramp] PWM_SERVO_SET_FORCE_SAFETY_OFF");
		return 1;
	}

	return 0;
}

int motor_ramp_thread_main(int argc, char *argv[])
{
	PX4_WARN("[motor_ramp] starting");

	_thread_running = true;

	char *dev = PWM_OUTPUT0_DEVICE_PATH;
	unsigned long max_channels = 0;

	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_ERR("[motor_ramp] can't open %s", dev);
	}

	if (prepare(fd, &max_channels) != OK) {
		_thread_should_exit = true;
	}

	PX4_WARN("[motor_ramp] max chan %d", max_channels);

	set_out(fd, max_channels, 0.0f);

	float dt = 0.01f; // prevent division with 0
	float timer = 0.0f;
	hrt_abstime start = 0;
	hrt_abstime last_run = 0;

	enum RampState ramp_state = RAMP_INIT;
	float output = 0.0f;

	while (!_thread_should_exit) {
		if (last_run > 0) {
			dt = hrt_elapsed_time(&last_run) * 1e-6;

		} else {
			start = hrt_absolute_time();
		}

		last_run = hrt_absolute_time();
		timer = hrt_elapsed_time(&start) * 1e-6;

		switch (ramp_state) {
		case RAMP_INIT: {
				PX4_WARN("[motor_ramp] setting pwm min %d", _min_pwm);
				set_min_pwm(fd, max_channels, _min_pwm);
				ramp_state = RAMP_MIN;
				break;
			}

		case RAMP_MIN: {
				if (timer > 3.0f) {
					PX4_WARN("[motor_ramp] starting ramp");
					start = hrt_absolute_time();
					ramp_state = RAMP_RAMP;
				}

				set_out(fd, max_channels, output);
				break;
			}

		case RAMP_RAMP: {
				output += dt / _ramp_time;

				if (output > 1.0f) {
					output = 1.0f;
					start = hrt_absolute_time();
					ramp_state = RAMP_WAIT;
					PX4_WARN("[motor_ramp] ramp finished, waiting");
				}

				set_out(fd, max_channels, output);
				break;
			}

		case RAMP_WAIT: {
				if (timer > 3.0f) {
					_thread_should_exit = true;
					break;
				}

				set_out(fd, max_channels, output);
				break;
			}
		}

		// rate limit
		usleep(2000);
	}

	if (fd >= 0) {
		/* disarm */
		ioctl(fd, PWM_SERVO_DISARM, 0);

		px4_close(fd);
	}

	PX4_WARN("[motor_ramp] exiting");

	_thread_running = false;

	return 0;
}
