/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file multirotor_pos_control_flow.c
 *
 * Skeleton for multirotor position controller
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>
#include <time.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <systemlib/systemlib.h>
#include <poll.h>

#include "multirotor_pos_control_flow_params.h"


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

__EXPORT int multirotor_pos_control_flow_main(int argc, char *argv[]);

/**
 * Mainloop of position controller.
 */
static int multirotor_pos_control_flow_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: deamon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_spawn().
 */
int multirotor_pos_control_flow_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("multirotor flow position control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("multirotor_pos_control_flow",
					 SCHED_DEFAULT,
//					 SCHED_PRIORITY_MAX,
					 SCHED_PRIORITY_MAX - 60,
					 4096,
					 multirotor_pos_control_flow_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tmultirotor flow position control app is running\n");
		} else {
			printf("\tmultirotor flow position control app not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static int
multirotor_pos_control_flow_thread_main(int argc, char *argv[])
{

	/* welcome user */
	thread_running = true;
	printf("[multirotor flow position control] starting\n");

	int loopcounter = 0;
	uint32_t counter = 0;

	/* structures */
	struct vehicle_status_s vstatus;
	struct vehicle_attitude_s att;
	struct vehicle_local_position_setpoint_s local_pos_sp;
	struct manual_control_setpoint_s manual;
	struct vehicle_local_position_s local_pos;

	struct vehicle_attitude_setpoint_s att_sp;

	/* subscribe to attitude, motor setpoints and system state */
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	int vehicle_local_position_setpoint_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));

	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	struct multirotor_position_control_params p;
	struct multirotor_position_control_param_handles h;
	parameters_init(&h);
	parameters_update(&h, &p);

	printf("[multirotor flow position control] initialized\n");

	while (!thread_should_exit) {

		/*This runs at the rate of the flow sensors */
		struct pollfd fds[1] = { {.fd = vehicle_local_position_sub, .events = POLLIN} };

		if (poll(fds, 1, 5000) <= 0) {
			printf("[multirotor flow position estimator] no local position updates");
		} else {

			/* get a local copy of the vehicle state */
			orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);
			/* get a local copy of manual setpoint */
			orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual);
			/* get a local copy of attitude */
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
			/* get a local copy of local position */
			orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &local_pos);
			/* get a local copy of local position setpoint */
			orb_copy(ORB_ID(vehicle_local_position_setpoint), vehicle_local_position_setpoint_sub, &local_pos_sp);

			if (loopcounter == 500) {
				parameters_update(&h, &p);
				loopcounter = 0;
			}

			//if (state.state_machine == SYSTEM_STATE_AUTO) {

				float x_setpoint = 0.0f;
				float y_setpoint = 0.0f;

				// XXX enable switching between Vicon and local position estimate
				/* local pos is the Vicon position */

				// XXX just an example, lacks rotation around world-body transformation
//				att_sp.pitch_body = (local_pos.x - x_setpoint) * p.p - local_pos.vx * p.d;
//				att_sp.roll_body = (local_pos.y - y_setpoint) * p.p - local_pos.vy * p.d;
//				att_sp.pitch_body = local_pos.vx * 1;
//				att_sp.roll_body =  - local_pos.vy * 1;
				att_sp.pitch_body = (local_pos.x - x_setpoint) * 10 + local_pos.vx * 20;
				att_sp.roll_body = (local_pos.y - y_setpoint) * 10 - local_pos.vy * 20;

				if(counter % 1 == 0) {
					printf("pos setpoint: %.3f, %.3f", att_sp.pitch_body, att_sp.roll_body);
				}

				att_sp.yaw_body = 0.0f;
				att_sp.thrust = 0.3f;
				att_sp.timestamp = hrt_absolute_time();

				/* publish new attitude setpoint */
				orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);

			//}

		}

		/* run at approximately 50 Hz */
		usleep(20000);
		loopcounter++;
		counter++;
	}

	printf("[multirotor flow position control] ending now...\n");
	thread_running = false;

	fflush(stdout);
	return 0;


//	/* welcome user */
//	printf("[multirotor flow position control] Control started, taking over position control\n");
//
//	/* variables */
//	static const int8_t rotM_flow_sensor[3][3] =   {{ 0,-1, 0 },
//													{ 1, 0, 0 },
//													{ 0, 0, 1 }};
//	static float speed[3] = {0.0f, 0.0f, 0.0f}; // x,y
//	static float flow_speed[3] = {0.0f, 0.0f, 0.0f};
//	static uint64_t time_last_flow = 0; // in ms
//	static float dt = 0; // seconds
//	static float time_scale = pow(10,-6);
//
//	/* structures */
//	struct vehicle_status_s state;
//	struct vehicle_attitude_s att;
//	struct vehicle_local_position_setpoint_s local_pos_sp;
////	struct vehicle_vicon_position_s local_pos;
//	struct manual_control_setpoint_s manual;
//	struct vehicle_attitude_setpoint_s att_sp;
//	struct optical_flow_s flow;
//	struct vehicle_local_position_s local_pos = {
//		.x = 0,
//		.y = 0,
//		.z = 0
//	};
//
//	/* subscribe to attitude, motor setpoints and system state */
//	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
//	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
//	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
////	int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
//	int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
//	int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));
//
//	/* publish attitude setpoint */
//	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
//
//	thread_running = true;
//
//	int loopcounter = 0;
//	int counter = 0;
//
//	struct multirotor_position_control_params p;
//	struct multirotor_position_control_param_handles h;
//	parameters_init(&h);
//	parameters_update(&h, &p);
//
//	printf("[multirotor flow position control] initialized\n");
//
//	while (!thread_should_exit) {
//
//		/* get a local copy of the vehicle state */
//		orb_copy(ORB_ID(vehicle_status), state_sub, &state);
//		/* get a local copy of manual setpoint */
//		orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
//		/* get a local copy of attitude */
//		orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
//		/* get a local copy of local position */
////		orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);
//		/* get a local copy of local position setpoint */
//		orb_copy(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_sub, &local_pos_sp);
//		/* get a local copy of optical flow */
//		orb_copy(ORB_ID(vehicle_local_position_setpoint), optical_flow_sub, &flow);
//
////		if (loopcounter == 500) {
////			parameters_update(&h, &p);
////			loopcounter = 0;
////		}
//
//		/*copy flow */
//		flow_speed[0] = flow.flow_comp_x_m;
//		flow_speed[1] = flow.flow_comp_y_m;
//		flow_speed[2] = 0.0f;
//
//		/* ignore first flow msg */
//		if(time_last_flow == 0)
//		{
//			time_last_flow = flow.timestamp;
//			continue;
//		}
//
//		/* calc dt */
//		dt = (float)(flow.timestamp - time_last_flow) * time_scale ;
//		time_last_flow = flow.timestamp;
//
//		/* convert to global velocity */
//		for(uint8_t i = 0; i < 3; i++) {
//			float sum = 0.0f;
//			for(uint8_t j = 0; j < 3; j++) {
//				sum = sum + flow_speed[j] * rotM_flow_sensor[j][i];
//			}
//			speed[i] = sum;
//		}
//
//		local_pos.x = local_pos.x + speed[0] * dt;
//		local_pos.y = local_pos.y + speed[1] * dt;
//
//			//if (state.state_machine == SYSTEM_STATE_AUTO) {
//
//				// XXX IMPLEMENT POSITION CONTROL HERE
//
//				//float dT = 1.0f / 50.0f;
//
//				float x_setpoint = 0.0f;
//				float y_setpoint = 0.0f;
//
//				// XXX enable switching between Vicon and local position estimate
//				/* local pos is the Vicon position */
//
//				// XXX just an example, lacks rotation around world-body transformation
////				att_sp.pitch_body = (local_pos.x - x_setpoint) * p.p - speed[0] * p.d;
////				att_sp.roll_body = (local_pos.y - y_setpoint) * p.p - speed[1] * p.d;
//				att_sp.pitch_body = (local_pos.x - x_setpoint) * 1;// - speed[0] * p.d;
//				att_sp.roll_body = (local_pos.y - y_setpoint) * 1;// - speed[1] * p.d;
//
//				if(counter % 10000 == 0) {
//					printf("pos setpoint: %.3f, %.3f", flow.flow_comp_x_m, flow.flow_comp_y_m);
//				}
//
////				att_sp.yaw_body = 0.0f;
////				att_sp.thrust = 0.3f;
////				att_sp.timestamp = hrt_absolute_time();
//
//				/* publish new attitude setpoint */
////				orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
//			// } else if (state.state_machine == SYSTEM_STATE_STABILIZED) {
//				/* set setpoint to current position */
//				// XXX select pos reset channel on remote
//				/* reset setpoint to current position  (position hold) */
//				// if (1 == 2) {
//				// 	local_pos_sp.x = local_pos.x;
//				// 	local_pos_sp.y = local_pos.y;
//				// 	local_pos_sp.z = local_pos.z;
//				// 	local_pos_sp.yaw = att.yaw;
//				// }
//			//}
//
//			/* run at approximately 50 Hz */
//			usleep(2000000);
//			loopcounter++;
//			counter++;
//
//	}

	printf("[multirotor flow position control] ending now...\n");

	thread_running = false;

	fflush(stdout);
	return 0;
}

