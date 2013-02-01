/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Tobias Naegeli <naegelit@student.ethz.ch>
 *			 Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
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
 * @file position_estimator_flow_main.c
 * Model-identification based position estimator for multirotors
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/optical_flow.h>
#include <poll.h>

__EXPORT int position_estimator_flow_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

int position_estimator_flow_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int position_estimator_flow_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("flow position estimator already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn("position_estimator_flow",
					 SCHED_RR,
					 SCHED_PRIORITY_DEFAULT,
					 4096,
					 position_estimator_flow_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tflow position estimator is running\n");
		} else {
			printf("\tflow position estimator not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int position_estimator_flow_thread_main(int argc, char *argv[])
{

	/* welcome user */
	thread_running = true;
	printf("[multirotor flow position estimator] starting\n");

	/* FIXME should be a parameter */
	static const int8_t rotM_flow_sensor[3][3] =   {{ 0,-1, 0 },
													{ 1, 0, 0 },
													{ 0, 0, 1 }};

	static float u[2] = {0.0f, 0.0f};
	static float speed[3] = {0.0f, 0.0f, 0.0f}; // x,y
	static float flow_speed[3] = {0.0f, 0.0f, 0.0f};
	static float global_speed[3] = {0.0f, 0.0f, 0.0f};
	static uint32_t counter = 0;
	static uint64_t time_last_flow = 0; // in ms
	static float dt = 0; // seconds
	static float time_scale = pow(10,-6);

	/* subscribe to vehicle status, attitude*/
	struct vehicle_status_s vstatus;
	struct vehicle_attitude_s att;
	struct optical_flow_s flow;

	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* subscribe to attitude at 100 Hz */
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	/* subscribe to attitude at 100 Hz */
	int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));

	/* publish global position messages */
	struct vehicle_local_position_s local_pos = {
		.x = 0,
		.y = 0,
		.z = 0
	};
	orb_advert_t local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos);

	printf("[multirotor flow position estimator] initialized\n");

	while (!thread_should_exit) {

		/*This runs at the rate of the flow sensors */
		struct pollfd fds[1] = { {.fd = optical_flow_sub, .events = POLLIN} };

		if (poll(fds, 1, 5000) <= 0) {
			printf("[multirotor flow position estimator] no flow");
		} else {

			orb_copy(ORB_ID(optical_flow), optical_flow_sub, &flow);
			/* got flow, updating attitude and status as well */
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
			orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);

			/*copy flow */
			flow_speed[0] = flow.flow_comp_x_m;
			flow_speed[1] = flow.flow_comp_y_m;
			flow_speed[2] = 0.0f;

			/* ignore first flow msg */
			if(time_last_flow == 0)
			{
				time_last_flow = flow.timestamp;
				continue;
			}

			/* calc dt */
			dt = (float)(flow.timestamp - time_last_flow) * time_scale ;
			time_last_flow = flow.timestamp;

			/* convert to global velocity */
			for(uint8_t i = 0; i < 3; i++) {
				float sum = 0.0f;
				for(uint8_t j = 0; j < 3; j++) {
					sum = sum + flow_speed[j] * rotM_flow_sensor[j][i];
				}
				speed[i] = sum;
			}

//			for(uint8_t i = 0; i < 3; i++) {
//				float sum = 0.0f;
//				for(uint8_t j = 0; j < 3; j++) {
//					sum = sum + speed[j] * att.R[j][i];
//				}
//				global_speed[i] = sum;
//			}

//			local_pos.x = local_pos.x + global_speed[0] * dt;
//			local_pos.y = local_pos.y + global_speed[1] * dt;
			local_pos.x = local_pos.x + speed[0] * dt;
			local_pos.y = local_pos.y + speed[1] * dt;

			local_pos.vx = speed[0];
			local_pos.vy = speed[1];

			orb_publish(ORB_ID(vehicle_local_position), local_pos_pub, &local_pos);

		}

		counter++;
	}

	printf("[multirotor flow position estimator] exiting.\n");
	thread_running = false;

	fflush(stdout);
	return 0;
}


