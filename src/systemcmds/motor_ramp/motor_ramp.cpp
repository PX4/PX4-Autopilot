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
#include <uORB/uORB.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/control_state.h>
#include <platforms/px4_defines.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"

static bool thread_should_exit = false;		/**< motor_ramp exit flag */
static bool thread_running = false;		/**< motor_ramp status flag */
static int motor_ramp_task;				/**< Handle of motor_ramp task / thread */

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

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PX4_WARN("usage: motor_ramp {start|stop|status} [-p <additional params>]\n\n");
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
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			PX4_WARN("motor_ramp already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		motor_ramp_task = px4_task_spawn_cmd("motor_ramp",
						     SCHED_DEFAULT,
						     SCHED_PRIORITY_DEFAULT,
						     2000,
						     motor_ramp_thread_main,
						     (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			PX4_WARN("\trunning\n");

		} else {
			PX4_WARN("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

void set_min_pwm(unsigned pwm_value)
{
	int ret;
	unsigned servo_count;
	char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	if (fd < 0) {PX4_WARN("can't open %s", dev);}

	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	struct pwm_output_values pwm_values;
	memset(&pwm_values, 0, sizeof(pwm_values));

	pwm_values.channel_count = servo_count;

	for (int i = 0; i < servo_count; i++) {
		pwm_values.values[i] = pwm_value;
	}

	ret = px4_ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values);

	if (ret != OK) {PX4_WARN("failed setting min values");}

	px4_close(fd);
}

void publish(orb_advert_t actuator_outputs_pub, struct actuator_outputs_s &actuator_outputs, float output)
{
	//PX4_WARN("[motor_ramp] publish %f", (double)output);
	for (int i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++) {
		actuator_outputs.output[i] = output;
	}
	
	orb_publish(ORB_ID(actuator_outputs), actuator_outputs_pub, &actuator_outputs);
}

int motor_ramp_thread_main(int argc, char *argv[])
{
	PX4_WARN("[motor_ramp] starting");

	thread_running = true;

	struct actuator_outputs_s actuator_outputs = {};
	orb_advert_t actuator_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &actuator_outputs);
	int ctrl_state_sub = orb_subscribe(ORB_ID(control_state));

	/* wakeup source */
	px4_pollfd_struct_t fds[1];

	/* Setup of loop */
	fds[0].fd = ctrl_state_sub;
	fds[0].events = POLLIN;

	float dt = 0.01f; // prevent division with 0
	float timer = 0.0f;
	hrt_abstime start = 0;
	hrt_abstime last_run = 0;

	enum RampState ramp_state = RAMP_INIT;
	unsigned min_pwm = 1100;
	float ramp_time = 0.5f;
	float output = 0.0f;

	publish(actuator_outputs_pub, actuator_outputs, 0.0f);

	while (!thread_should_exit) {
		if (last_run > 0) {
			dt = hrt_elapsed_time(&last_run) * 1e-6;

		} else {
			start = hrt_absolute_time();
		}

		last_run = hrt_absolute_time();
		timer = hrt_elapsed_time(&start) * 1e-6;

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			PX4_WARN("[motor_ramp] poll error %d", pret);
			continue;
		}

		switch (ramp_state) {
		case RAMP_INIT: {
				if (min_pwm > 1350) {
					thread_should_exit = true;
					PX4_WARN("[motor_ramp] pwm min too high %d", min_pwm);
					break;
				}

				PX4_WARN("[motor_ramp] setting pwm min %d", min_pwm);
				set_min_pwm(min_pwm);
				ramp_state = RAMP_MIN;
				break;
			}

		case RAMP_MIN: {
				if (timer > 3.0f) {
					PX4_WARN("[motor_ramp] min set, starting ramp");
					start = hrt_absolute_time();
					ramp_state = RAMP_RAMP;
				}

				break;
			}

		case RAMP_RAMP: {
				output += dt / ramp_time;

				if (output > 1.0f) {
					output = 1.0f;
					start = hrt_absolute_time();
					ramp_state = RAMP_WAIT;
					PX4_WARN("[motor_ramp] ramp finished, waiting");
				}

				publish(actuator_outputs_pub, actuator_outputs, output);
				break;
			}

		case RAMP_WAIT: {
				if (timer > 3.0f) {
					thread_should_exit = true;
					break;
				}

				break;
			}
		}
	}

	publish(actuator_outputs_pub, actuator_outputs, 0.0f);

	PX4_WARN("[motor_ramp] exiting");

	thread_running = false;

	return 0;
}
