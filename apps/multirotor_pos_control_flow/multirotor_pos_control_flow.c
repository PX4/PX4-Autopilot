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
#include <math.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
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
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	int vehicle_local_position_setpoint_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));

	/* polling */
	struct pollfd fds[2] = {
				{ .fd = vehicle_local_position_sub, .events = POLLIN }, // positions from estimator
				{ .fd = parameter_update_sub,   .events = POLLIN },
			};

	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	/* limits */
	float pitch_limit = 0.33f;
	float roll_limit = 0.33f;
	float thrust_limit_upper = 0.5f;
	float thrust_limit_lower = 0.2f;

	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "multirotor_att_control_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "multirotor_att_control_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "multirotor_att_control_err");

	struct multirotor_position_control_flow_params params;
	struct multirotor_position_control_flow_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	printf("[multirotor flow position control] initialized\n");

	while (!thread_should_exit) {

		/* wait for a position update, check for exit condition every 500 ms */
		int ret = poll(fds, 2, 500);

		if (ret < 0) {
			/* poll error, count it in perf */
			perf_count(mc_err_perf);

		} else if (ret == 0) {
			/* no return value, ignore */
			printf("[multirotor flow position estimator] no local position updates\n");
		} else {

			if (fds[1].revents & POLLIN){
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

				parameters_update(&param_handles, &params);
				printf("[multirotor flow position control] parameters updated.\n");
			}

			/* only run controller if position changed */
			if (fds[0].revents & POLLIN) {

				perf_begin(mc_loop_perf);

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

				//if (vstatus.state_machine == SYSTEM_STATE_AUTO) {

//					att_sp.pitch_body = (local_pos.x - x_setpoint) * p.p - local_pos.vx * p.d;
//					att_sp.roll_body = (local_pos.y - y_setpoint) * p.p - local_pos.vy * p.d;
//					att_sp.pitch_body = local_pos.vx * 1;
//					att_sp.roll_body =  - local_pos.vy * 1;
					/*
					 * setpoint is to be set as "how to change attitude"
					 *
					 *
					 */

//					if (loopcounter == 200){
//						if(isnan(local_pos.x))
//							printf("x is nan...\n");
//						if(isnan(params.pos_sp_x))
//							printf("pos_sp_x is nan...\n");
//						if(isnan(params.pos_p))
//							printf("pos_p is nan...\n");
//						if(isnan(local_pos.vx))
//							printf("vx is nan...\n");
//						if(isnan(params.pos_d))
//							printf("pos_d is nan...\n");
//						if(isnan(local_pos.y))
//							printf("y is nan...\n");
//						if(isnan(params.pos_sp_y))
//							printf("pos_sp_y is nan...\n");
//						if(isnan(local_pos.vy))
//							printf("local_pos.vy is nan...\n");
//						if(isnan(local_pos.z))
//							printf("z is nan...\n");
//						if(isnan(params.height_sp))
//							printf("height_sp is nan...\n");
//						if(isnan(params.height_p))
//							printf("height_p is nan...\n");
//
//						loopcounter = 0;
//					}


					float pitch_body = (local_pos.x - params.pos_sp_x) * params.pos_p + local_pos.vx * params.pos_d;
					float roll_body = - (local_pos.y - params.pos_sp_y) * params.pos_p - local_pos.vy * params.pos_d;
					float thrust = (local_pos.z - params.height_sp) * params.height_p;

					if((roll_body <= roll_limit) && (roll_body >= -roll_limit)){
						att_sp.roll_body = roll_body;
					} else {
						if(roll_body > roll_limit){
							att_sp.roll_body = roll_limit;
						}
						if(roll_body < -roll_limit){
							att_sp.roll_body = -roll_limit;
						}
					}
					if((pitch_body <= pitch_limit) && (pitch_body >= -pitch_limit)){
						att_sp.pitch_body = pitch_body;
					} else {
						if(pitch_body > pitch_limit){
							att_sp.pitch_body = pitch_limit;
						}
						if(pitch_body < -pitch_limit){
							att_sp.pitch_body = -pitch_limit;
						}
					}

					float height_ctrl_thrust_feedforward = params.thrust_offset; // FIXME
					float height_ctrl_thrust = height_ctrl_thrust_feedforward + thrust;

					/* the throttle stick on the rc control limits the maximum thrust */
					if(!isnan(manual.throttle))
						thrust_limit_upper = manual.throttle;
					if (height_ctrl_thrust >= thrust_limit_upper){
						att_sp.thrust = thrust_limit_upper;
					/* never go too low with the thrust that it becomes uncontrollable */
					}else if(height_ctrl_thrust < thrust_limit_lower){
						att_sp.thrust = thrust_limit_lower;
					}

					/* do not control yaw */
					att_sp.yaw_body = 0.0f;
					att_sp.timestamp = hrt_absolute_time();

					/* publish new attitude setpoint */
					if (loopcounter == 200){
						if(!isnan(att_sp.pitch_body) && !isnan(att_sp.roll_body) && !isnan(att_sp.yaw_body) && !isnan(att_sp.thrust))
						{
							printf("pitch:%.3f, roll:%.3f\n", att_sp.pitch_body, att_sp.roll_body);
							printf("yaw:%.3f, thrust:%.3f\n", att_sp.yaw_body, att_sp.thrust);
						} else {
							if(isnan(att_sp.pitch_body))
								printf("pitch_body is nan...\n");
							if(isnan(att_sp.roll_body))
								printf("roll_body is nan...\n");
							if(isnan(att_sp.yaw_body))
								printf("pitch is nan...\n");
							if(isnan(att_sp.thrust))
								printf("pitch is nan...\n");
						}

						loopcounter = 0;
					}

					orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);

					loopcounter++;
				//}

				/* measure in what intervals the controller runs */
				perf_count(mc_interval_perf);
				perf_end(mc_loop_perf);
			}
		}

		/* run at approximately 50 Hz */
		//usleep(20000);

		counter++;

	}

	printf("[multirotor flow position control] ending now...\n");

	thread_running = false;

	close(parameter_update_sub);
	close(vehicle_attitude_sub);
	close(vehicle_local_position_setpoint_sub);
	close(vehicle_local_position_sub);
	close(vehicle_status_sub);
	close(manual_control_setpoint_sub);
	close(att_sp_pub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	return 0;
}

