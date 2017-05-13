/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @author Roman Bapst <bapstroman@gmail.com>
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

enum RampState {
	RAMP_INIT,
	RAMP_MIN,
	RAMP_RAMP,
	RAMP_WAIT
};

enum Mode {
	RAMP,
	SINE,
	SQUARE
};

static bool _thread_should_exit = false;		/**< motor_ramp exit flag */
static bool _thread_running = false;		/**< motor_ramp status flag */
static int _motor_ramp_task;				/**< Handle of motor_ramp task / thread */
static float _ramp_time;
static int _min_pwm;
static int _max_pwm;
static Mode _mode;
static char *_mode_c;

/**
 * motor_ramp management function.
 */
extern "C" __EXPORT int motor_ramp_main(int argc, char *argv[]);

/**
 * Mainloop of motor_ramp.
 */
int motor_ramp_thread_main(int argc, char *argv[]);

bool min_pwm_valid(unsigned pwm_value);

bool max_pwm_valid(unsigned pwm_value);

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
		PX4_ERR("%s", reason);
	}

	PX4_WARN("\n\nWARNING: motors will ramp up to full speed!\n\n"
		 "Usage: motor_ramp <mode> <min_pwm> <time> [<max_pwm>]\n"
		 "<mode> can be one of (ramp|sine|square)\n\n"
		 "Example:\n"
		 "sdlog2 on\n"
		 "mc_att_control stop\n"
		 "fw_att_control stop\n"
		 "motor_ramp sine 1100 0.5\n");
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
	if (argc < 4) {
		usage("missing parameters");
		return 1;
	}

	if (_thread_running) {
		PX4_WARN("motor_ramp already running\n");
		/* this is not an error */
		return 0;
	}

	if (!strcmp(argv[1], "ramp")) {
		_mode = RAMP;

	} else if (!strcmp(argv[1], "sine")) {
		_mode = SINE;

	} else if (!strcmp(argv[1], "square")) {
		_mode = SQUARE;

	} else {
		usage("selected mode not valid");
		return 1;
	}

	_mode_c = argv[1];

	_min_pwm = atoi(argv[2]);

	if (!min_pwm_valid(_min_pwm)) {
		usage("min PWM not in range");
		return 1;
	}

	_ramp_time = atof(argv[3]);

	if (argc > 4) {
		_max_pwm = atoi(argv[4]);

		if (!max_pwm_valid(_max_pwm)) {
			usage("max PWM not in range");
			return 1;
		}

	} else {
		_max_pwm = 2000;
	}

	if (!(_ramp_time > 0)) {
		usage("ramp time must be greater than 0");
		return 1;
	}

	_thread_should_exit = false;
	_motor_ramp_task = px4_task_spawn_cmd("motor_ramp",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_DEFAULT + 40,
					      2000,
					      motor_ramp_thread_main,
					      (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
	return 0;

	usage("unrecognized command");
	return 1;
}

bool min_pwm_valid(unsigned pwm_value)
{
	return pwm_value >= 900 && pwm_value <= 1500;
}

bool max_pwm_valid(unsigned pwm_value)
{
	return pwm_value <= 2100 && pwm_value > _min_pwm;
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
		PX4_ERR("failed setting min values");
		return 1;
	}

	return 0;
}

int set_out(int fd, unsigned long max_channels, float output)
{
	int ret;
	int pwm = (_max_pwm - _min_pwm) * output + _min_pwm;

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
	/* make sure no other source is publishing control values now */
	struct actuator_controls_s actuators;
	int act_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);

	/* clear changed flag */
	orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, act_sub, &actuators);

	/* wait 50 ms */
	usleep(50000);

	/* now expect nothing changed on that topic */
	bool orb_updated;
	orb_check(act_sub, &orb_updated);

	if (orb_updated) {
		PX4_ERR("ABORTING! Attitude control still active. Please ensure to shut down all controllers:\n"
			"\tmc_att_control stop\n"
			"\tfw_att_control stop\n");
		return 1;
	}

	/* get number of channels available on the device */
	if (px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)max_channels) != OK) {
		PX4_ERR("PWM_SERVO_GET_COUNT");
		return 1;
	}

	/* tell IO/FMU that its ok to disable its safety with the switch */
	if (px4_ioctl(fd, PWM_SERVO_SET_ARM_OK, 0) != OK) {
		PX4_ERR("PWM_SERVO_SET_ARM_OK");
		return 1;
	}

	/* tell IO/FMU that the system is armed (it will output values if safety is off) */
	if (px4_ioctl(fd, PWM_SERVO_ARM, 0) != OK) {
		PX4_ERR("PWM_SERVO_ARM");
		return 1;
	}

	/* tell IO to switch off safety without using the safety switch */
	if (px4_ioctl(fd, PWM_SERVO_SET_FORCE_SAFETY_OFF, 0) != OK) {
		PX4_ERR("PWM_SERVO_SET_FORCE_SAFETY_OFF");
		return 1;
	}

	return 0;
}

int motor_ramp_thread_main(int argc, char *argv[])
{
	_thread_running = true;

	char *dev = PWM_OUTPUT0_DEVICE_PATH;
	unsigned long max_channels = 0;

	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_ERR("can't open %s", dev);
	}

	if (prepare(fd, &max_channels) != OK) {
		_thread_should_exit = true;
	}

	set_out(fd, max_channels, 0.0f);

	float dt = 0.001f; // prevent division with 0
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
				PX4_WARN("setting pwm min: %d", _min_pwm);
				set_min_pwm(fd, max_channels, _min_pwm);
				ramp_state = RAMP_MIN;
				break;
			}

		case RAMP_MIN: {
				if (timer > 3.0f) {
					PX4_WARN("starting %s: %.2f sec", _mode_c, (double)_ramp_time);
					start = hrt_absolute_time();
					ramp_state = RAMP_RAMP;
				}

				set_out(fd, max_channels, output);
				break;
			}

		case RAMP_RAMP: {
				if (_mode == RAMP) {
					output += 1000.0f * dt / (_max_pwm - _min_pwm) / _ramp_time;

				} else if (_mode == SINE) {
					// sine outpout with period T = _ramp_time and magnitude between [0,1]
					// phase shift makes sure that it starts at zero when timer is zero
					output = 0.5f * (1.0f + sinf(M_TWOPI_F * timer / _ramp_time - M_PI_2_F));

				} else if (_mode == SQUARE) {
					output = fmodf(timer, _ramp_time) > (_ramp_time * 0.5f) ? 1.0f : 0.0f;
				}

				if ((output > 1.0f && _mode == RAMP) || (timer > 3.0f * _ramp_time)) {
					// for ramp mode we set output to 1, for others we just leave it as is
					output = _mode != RAMP ? output : 1.0f;
					start = hrt_absolute_time();
					ramp_state = RAMP_WAIT;
					PX4_WARN("%s finished, waiting", _mode_c);
				}

				set_out(fd, max_channels, output);
				break;
			}

		case RAMP_WAIT: {
				if (timer > 0.5f) {
					_thread_should_exit = true;
					PX4_WARN("stopping");
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

	_thread_running = false;

	return 0;
}
