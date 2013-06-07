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
 * @file flow_position_control.c
 *
 * Skeleton for flow position controller
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
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_bodyframe_position.h>
#include <uORB/topics/vehicle_bodyframe_position_setpoint.h>
#include <uORB/topics/vehicle_bodyframe_speed_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <poll.h>

#include "flow_position_control_params.h"


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

__EXPORT int flow_position_control_main(int argc, char *argv[]);

/**
 * Mainloop of position controller.
 */
static int flow_position_control_thread_main(int argc, char *argv[]);

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
int flow_position_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("flow position control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("flow_position_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 6,
					 4096,
					 flow_position_control_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tflow position control app is running\n");
		} else {
			printf("\tflow position control app not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static int
flow_position_control_thread_main(int argc, char *argv[])
{
	/* welcome user */
	thread_running = true;
	printf("[flow position control] starting\n");

	uint32_t counter = 0;

	/* structures */
	struct vehicle_status_s vstatus;
	struct vehicle_attitude_s att;
	struct manual_control_setpoint_s manual;
	struct vehicle_bodyframe_position_s bodyframe_pos;
	struct vehicle_bodyframe_position_setpoint_s bodyframe_pos_sp;

	struct vehicle_bodyframe_speed_setpoint_s speed_sp;
	struct vehicle_local_position_setpoint_s debug_speed_sp;

	/* subscribe to attitude, motor setpoints and system state */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int vehicle_bodyframe_position_sub = orb_subscribe(ORB_ID(vehicle_bodyframe_position));
	int vehicle_bodyframe_position_setpoint_sub = orb_subscribe(ORB_ID(vehicle_bodyframe_position_setpoint));

	orb_advert_t speed_sp_pub;
	orb_advert_t debug_speed_sp_pub;
	bool speed_setpoint_adverted = false;

	/* parameters init*/
	struct flow_position_control_params params;
	struct flow_position_control_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	//TODO init
	bodyframe_pos_sp.x = 0.0f;
	bodyframe_pos_sp.y = 0.0f;
	bodyframe_pos_sp.yaw = 0.0f;

	/* states */
	float integrated_h_error = 0.0f;
	float last_height = 0.0f;
	float thrust_limit_upper = params.limit_thrust_lower; // it will be updated with manual input
	bool bodyframe_setpoint_valid = false;

	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "flow_position_control_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "flow_position_control_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "flow_position_control_err");

	static bool sensors_ready = false;

	while (!thread_should_exit) {

		/* wait for first attitude msg to be sure all data are available */
		if (sensors_ready) {

			/* polling */
			struct pollfd fds[2] = {
				{ .fd = vehicle_bodyframe_position_sub, .events = POLLIN }, // positions from estimator
				{ .fd = parameter_update_sub,   .events = POLLIN }

			};

			/* wait for a position update, check for exit condition every 500 ms */
			int ret = poll(fds, 2, 500);

			if (ret < 0) {
				/* poll error, count it in perf */
				perf_count(mc_err_perf);

			} else if (ret == 0) {
				/* no return value, ignore */
				printf("[flow position control] no bodyframe position updates\n"); // XXX wrong place

			} else {

				/* parameter update available? */
				if (fds[1].revents & POLLIN){
					/* read from param to clear updated flag */
					struct parameter_update_s update;
					orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

					parameters_update(&param_handles, &params);
					printf("[flow position control] parameters updated.\n");
				}

				/* only run controller if position/speed changed */
				if (fds[0].revents & POLLIN) {

					perf_begin(mc_loop_perf);

					/* get a local copy of the vehicle state */
					orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);
					/* get a local copy of manual setpoint */
					orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual);
					/* get a local copy of attitude */
					orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
					/* get a local copy of bodyframe position */
					orb_copy(ORB_ID(vehicle_bodyframe_position), vehicle_bodyframe_position_sub, &bodyframe_pos);

					if (vstatus.state_machine == SYSTEM_STATE_AUTO) {

//						/* be sure that we have a valid setpoint to the current position
//						 * can be a setpoint referring to the old position (wait one update)
//						 * */
//						if(!bodyframe_setpoint_valid) {
//							/* if setpoint is invalid, wait until setpoint changes... */
//							bool new_setpoint =  false;
//							orb_check(vehicle_bodyframe_position_setpoint_sub, &new_setpoint);
//							if (new_setpoint) {
//								bodyframe_setpoint_valid = true;
//							}
//						}
//
//						if (bodyframe_setpoint_valid) {
//							/* get a local copy of bodyframe position setpoint */
//							orb_copy(ORB_ID(vehicle_bodyframe_position_setpoint), vehicle_bodyframe_position_setpoint_sub, &bodyframe_pos_sp);

							/* calc new roll/pitch */
//							float pitch_body = (bodyframe_pos.x - bodyframe_pos_sp.x) * params.pos_p + bodyframe_pos.vx * params.pos_d;
//							float roll_body = - (bodyframe_pos.y - bodyframe_pos_sp.y) * params.pos_p - bodyframe_pos.vy * params.pos_d;
							float speed_body_x = (bodyframe_pos_sp.x - bodyframe_pos.x) * params.pos_p - bodyframe_pos.vx * params.pos_d;
							float speed_body_y = (bodyframe_pos_sp.y - bodyframe_pos.y) * params.pos_p - bodyframe_pos.vy * params.pos_d;


							/* limit speed setpoints */
							if((speed_body_x <= params.limit_speed_x) && (speed_body_x >= -params.limit_speed_x)){
								speed_sp.vx = speed_body_x;
							} else {
								if(speed_body_x > params.limit_speed_x){
									speed_sp.vx = params.limit_speed_x;
								}
								if(speed_body_x < -params.limit_speed_x){
									speed_sp.vx = -params.limit_speed_x;
								}
							}

							if((speed_body_y <= params.limit_speed_y) && (speed_body_y >= -params.limit_speed_y)){
								speed_sp.vy = speed_body_y;
							} else {
								if(speed_body_y > params.limit_speed_y){
									speed_sp.vy = params.limit_speed_y;
								}
								if(speed_body_y < -params.limit_speed_y){
									speed_sp.vy = -params.limit_speed_y;
								}
							}

							/* forward yaw setpoint */
							speed_sp.yaw_sp = bodyframe_pos_sp.yaw;

							/* calc new thrust */
							float height_error = (bodyframe_pos.z - params.height_sp);
							integrated_h_error = integrated_h_error + height_error;
							float integrated_thrust_addition = integrated_h_error * params.height_i;

							if(integrated_thrust_addition > params.limit_thrust_int){
								integrated_thrust_addition = params.limit_thrust_int;
							}
							if(integrated_thrust_addition < -params.limit_thrust_int){
								integrated_thrust_addition = -params.limit_thrust_int;
							}

							float height_speed = last_height - bodyframe_pos.z;
							last_height = bodyframe_pos.z;
							float thrust_diff = height_error * params.height_p - height_speed * params.height_d; // just PD controller
							float thrust = thrust_diff + integrated_thrust_addition;

							float height_ctrl_thrust = params.thrust_feedforward + thrust;

							/* reset integral if on ground */
							// TODO
							if(isfinite(manual.throttle)) {
								thrust_limit_upper = manual.throttle;
							}

							if (thrust_limit_upper < 0.2f) {
								integrated_h_error = 0.0f;
							}

							/* set thrust within controllable limits */
							if(height_ctrl_thrust < params.limit_thrust_lower)
								height_ctrl_thrust = params.limit_thrust_lower;
							if(height_ctrl_thrust > params.limit_thrust_upper)
								height_ctrl_thrust = params.limit_thrust_upper;

							speed_sp.thrust_sp = height_ctrl_thrust;
							speed_sp.timestamp = hrt_absolute_time();

							/* publish new attitude setpoint */
							if(isfinite(speed_sp.vx) && isfinite(speed_sp.vy) && isfinite(speed_sp.yaw_sp) && isfinite(speed_sp.thrust_sp))
							{
								debug_speed_sp.x = speed_sp.vx;
								debug_speed_sp.y = speed_sp.vy;
								debug_speed_sp.yaw = speed_sp.yaw_sp;
								debug_speed_sp.z = speed_sp.thrust_sp;

								if(speed_setpoint_adverted)
								{
//									orb_publish(ORB_ID(vehicle_local_position_setpoint), debug_speed_sp_pub, &debug_speed_sp);
									orb_publish(ORB_ID(vehicle_bodyframe_speed_setpoint), speed_sp_pub, &speed_sp);
								}
								else
								{
									debug_speed_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &debug_speed_sp);
									speed_sp_pub = orb_advertise(ORB_ID(vehicle_bodyframe_speed_setpoint), &speed_sp);
									speed_setpoint_adverted = true;
								}
							}
							else
							{
								warnx("NaN in flow position controller!");
							}
//						}

					} else {
						/* call orb copy for setpoint to recognize new one if mode changes */
//						orb_copy(ORB_ID(vehicle_bodyframe_position_setpoint), vehicle_bodyframe_position_setpoint_sub, &bodyframe_pos_sp);

						/* in manual or stabilized state just reset attitude setpoint */
						speed_sp.vx = 0.0f;
						speed_sp.vy = 0.0f;
						speed_sp.yaw_sp = att.yaw;
//						bodyframe_setpoint_valid = false;

						//TODO
						bodyframe_pos_sp.x = bodyframe_pos.x;
						bodyframe_pos_sp.y = bodyframe_pos.y;
						bodyframe_pos_sp.yaw = att.yaw;
					}

					/* measure in what intervals the controller runs */
					perf_count(mc_interval_perf);
					perf_end(mc_loop_perf);
				}

			}

			counter++;


		} else {
			/* sensors not ready waiting for first attitude msg */

			/* polling */
			struct pollfd fds[1] = {
				{ .fd = vehicle_attitude_sub, .events = POLLIN },
			};

			/* wait for a flow msg, check for exit condition every 5 s */
			int ret = poll(fds, 1, 5000);

			if (ret < 0) {
				/* poll error, count it in perf */
				perf_count(mc_err_perf);

			} else if (ret == 0) {
				/* no return value, ignore */
				printf("[flow position control] no attitude received.\n");
			} else {

				if (fds[0].revents & POLLIN){
					sensors_ready = true;
					printf("[flow position control] initialized.\n");
				}
			}
		}

	}

	printf("[flow position control] ending now...\n");

	thread_running = false;

	close(parameter_update_sub);
	close(vehicle_attitude_sub);
	close(vehicle_bodyframe_position_setpoint_sub);
	close(vehicle_bodyframe_position_sub);
	close(vehicle_status_sub);
	close(manual_control_setpoint_sub);
	close(speed_sp_pub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	return 0;
}

