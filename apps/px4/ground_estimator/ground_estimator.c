/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
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
 * @file ground_estimator.c
 * ground_estimator application example for PX4 autopilot
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <stdlib.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/debug_key_value.h>


static bool thread_should_exit = false;		/**< ground_estimator exit flag */
static bool thread_running = false;		/**< ground_estimator status flag */
static int ground_estimator_task;				/**< Handle of ground_estimator task / thread */

/**
 * ground_estimator management function.
 */
__EXPORT int ground_estimator_main(int argc, char *argv[]);

/**
 * Mainloop of ground_estimator.
 */
int ground_estimator_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

int ground_estimator_thread_main(int argc, char *argv[]) {

	printf("[ground_estimator] starting\n");

	/* subscribe to raw data */
	int sub_raw = orb_subscribe(ORB_ID(sensor_combined));

	/* advertise debug value */
	struct debug_key_value_s dbg = { .key = "posx", .value = 0.0f };
	orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);

	float position[3] = {};
	float velocity[3] = {};

	uint64_t last_time = 0;

	struct pollfd fds[] = {
		{ .fd = sub_raw,   .events = POLLIN },
	};

	while (!thread_should_exit) {
		
		/* wait for sensor update */
		int ret = poll(fds, 1, 1000);

		if (ret < 0) {
			/* XXX this is seriously bad - should be an emergency */
		} else if (ret == 0) {
			/* XXX this means no sensor data - should be critical or emergency */
			printf("[ground estimator bm] WARNING: Not getting sensor data - sensor app running?\n");
		} else {
			struct sensor_combined_s s;
			orb_copy(ORB_ID(sensor_combined), sub_raw, &s);

			float dt = ((float)(s.timestamp - last_time)) / 1000000.0f;

			/* Integration */
			position[0] += velocity[0] * dt;
			position[1] += velocity[1] * dt;
			position[2] += velocity[2] * dt;

			velocity[0] += velocity[0] * 0.01f + 0.99f * s.accelerometer_m_s2[0] * dt;
			velocity[1] += velocity[1] * 0.01f + 0.99f * s.accelerometer_m_s2[1] * dt;
			velocity[2] += velocity[2] * 0.01f + 0.99f * s.accelerometer_m_s2[2] * dt;

			dbg.value = position[0];

			orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
		}

	}

	printf("[ground_estimator] exiting.\n");

	return 0;
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: ground_estimator {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The ground_estimator app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_create().
 */
int ground_estimator_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("ground_estimator already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		ground_estimator_task = task_create("ground_estimator", SCHED_PRIORITY_DEFAULT, 4096, ground_estimator_thread_main, (argv) ? (const char **)&argv[2] : (const char **)NULL);
		thread_running = true;
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tground_estimator is running\n");
		} else {
			printf("\tground_estimator not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}
