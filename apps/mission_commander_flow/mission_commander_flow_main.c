/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: @author Samuel Zihlmann <samuezih@ee.ethz.ch>
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
 * @file mission_commander_flow_main.c
 */
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <sys/types.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_waypoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_bodyframe_position.h>
#include <uORB/topics/vehicle_bodyframe_position_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/omnidirectional_flow.h>
#include <uORB/topics/discrete_radar.h>

#include <uORB/topics/vehicle_global_position.h>
#include <poll.h>
#include <mavlink/mavlink_log.h>

#include "mission_commander_flow_params.h"
#include "mission_sounds.h"
#include "mission_helper.h"

#define sign(x) (( x > 0 ) - ( x < 0 ))

__EXPORT int mission_commander_flow_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
static int mavlink_fd;

int mission_commander_flow_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

/**
 * Navigation helpfunctions.
 */
void convert_setpoint_bodyframe2local(
		struct vehicle_local_position_s *local_pos,
		struct vehicle_bodyframe_position_s *bodyframe_pos,
		struct vehicle_attitude_s *att,
		struct vehicle_bodyframe_position_setpoint_s *bodyframe_pos_sp,
		struct vehicle_local_position_setpoint_s *local_pos_sp
		);
void convert_setpoint_local2bodyframe(
		struct vehicle_local_position_s *local_pos,
		struct vehicle_bodyframe_position_s *bodyframe_pos,
		struct vehicle_attitude_s *att,
		struct vehicle_local_position_setpoint_s *local_pos_sp,
		struct vehicle_bodyframe_position_setpoint_s *bodyframe_pos_sp
		);

float get_yaw(
		struct vehicle_local_position_s *local_pos,
		struct vehicle_local_position_setpoint_s *local_pos_sp
		);

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
int mission_commander_flow_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("mission commander already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn("mission_commander_flow",
					 SCHED_RR,
					 SCHED_PRIORITY_MAX - 60,
					 4096,
					 mission_commander_flow_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tmission commander is running\n");
		} else {
			printf("\tmission commander not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}


void convert_setpoint_bodyframe2local(
		struct vehicle_local_position_s *local_pos,
		struct vehicle_bodyframe_position_s *bodyframe_pos,
		struct vehicle_attitude_s *att,
		struct vehicle_bodyframe_position_setpoint_s *bodyframe_pos_sp,
		struct vehicle_local_position_setpoint_s *local_pos_sp
		)
{
	static float wp_bodyframe_offset[3] = {0.0f, 0.0f, 0.0f};
	static float wp_local_offset[3] = {0.0f, 0.0f, 0.0f};

	wp_bodyframe_offset[0] = bodyframe_pos_sp->x - bodyframe_pos->x;
	wp_bodyframe_offset[1] = bodyframe_pos_sp->y - bodyframe_pos->y;
	wp_bodyframe_offset[2] = 0; // no influence of z...

	/* calc current waypoint cooridnates in local */
	for(uint8_t i = 0; i < 3; i++) {
		float sum = 0.0f;
		for(uint8_t j = 0; j < 3; j++) {
			sum = sum + wp_bodyframe_offset[j] * att->R[i][j];
		}
		wp_local_offset[i] = sum;
	}

	local_pos_sp->x = local_pos->x + wp_local_offset[0];
	local_pos_sp->y = local_pos->y + wp_local_offset[1];
	local_pos_sp->z = bodyframe_pos_sp->z; // let z as it is...
	local_pos_sp->yaw = bodyframe_pos_sp->yaw;
}

void convert_setpoint_local2bodyframe(
		struct vehicle_local_position_s *local_pos,
		struct vehicle_bodyframe_position_s *bodyframe_pos,
		struct vehicle_attitude_s *att,
		struct vehicle_local_position_setpoint_s *local_pos_sp,
		struct vehicle_bodyframe_position_setpoint_s *bodyframe_pos_sp
		)
{
	static float wp_local_offset[3] = {0.0f, 0.0f, 0.0f}; // x,y
	static float wp_bodyframe_offset[3] = {0.0f, 0.0f, 0.0f};

	wp_local_offset[0] = local_pos_sp->x - local_pos->x;
	wp_local_offset[1] = local_pos_sp->y - local_pos->y;
	wp_local_offset[2] = 0; // no influence of z...

	/* calc current waypoint cooridnates in bodyframe */
	for(uint8_t i = 0; i < 3; i++) {
		float sum = 0.0f;
		for(uint8_t j = 0; j < 3; j++) {
			sum = sum + wp_local_offset[j] * att->R[j][i];
		}
		wp_bodyframe_offset[i] = sum;
	}

	bodyframe_pos_sp->x = bodyframe_pos->x + wp_bodyframe_offset[0];
	bodyframe_pos_sp->y = bodyframe_pos->y + wp_bodyframe_offset[1];
	bodyframe_pos_sp->z = local_pos_sp->z; // let z as it is...
	bodyframe_pos_sp->yaw = local_pos_sp->yaw;

}

float get_yaw(
		struct vehicle_local_position_s *local_pos,
		struct vehicle_local_position_setpoint_s *local_pos_sp
		)
{
	float dx = local_pos_sp->x - local_pos->x;
	float dy = local_pos_sp->y - local_pos->y;

	return atan2f(dy,dx);
}


int mission_commander_flow_thread_main(int argc, char *argv[])
{

	/* welcome user */
	thread_running = true;
	printf("[mission commander] starting\n");
	static uint32_t counter = 0;

	/* structures */
	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct omnidirectional_flow_s omni_flow;
	memset(&omni_flow, 0, sizeof(omni_flow));
	struct vehicle_local_position_s local_pos;
	memset(&local_pos, 0, sizeof(local_pos));
	struct vehicle_bodyframe_position_s bodyframe_pos;
	memset(&bodyframe_pos, 0, sizeof(bodyframe_pos));

	/* mission parameters */
	struct vehicle_local_position_setpoint_s final_dest_local;
	memset(&final_dest_local, 0, sizeof(final_dest_local));
	struct vehicle_bodyframe_position_setpoint_s final_dest_bodyframe;
	memset(&final_dest_bodyframe, 0, sizeof(final_dest_bodyframe));

	/* publishing parameters */
	struct vehicle_local_position_setpoint_s local_pos_sp;
	memset(&local_pos_sp, 0, sizeof(local_pos_sp));
	struct vehicle_bodyframe_position_setpoint_s bodyframe_pos_sp;
	memset(&bodyframe_pos_sp, 0, sizeof(bodyframe_pos_sp));
	struct discrete_radar_s discrete_radar;
	memset(&discrete_radar, 0, sizeof(discrete_radar));


	/* subscribe to attitude, motor setpoints and system state */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int omnidirectional_flow_sub = orb_subscribe(ORB_ID(omnidirectional_flow));
	int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	int vehicle_bodyframe_position_sub = orb_subscribe(ORB_ID(vehicle_bodyframe_position));
	int discrete_radar_sub = orb_subscribe(ORB_ID(discrete_radar));
	int vehicle_local_waypoint_sub = orb_subscribe(ORB_ID(vehicle_local_waypoint));

	/* advert local/bodyframe position setpoint */
	/* TODO fix advert bug */
	orb_advert_t vehicle_local_position_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &local_pos_sp);
	orb_advert_t vehicle_bodyframe_position_sp_pub = orb_advertise(ORB_ID(vehicle_bodyframe_position_setpoint), &bodyframe_pos_sp);

	/* for debugging */
	struct vehicle_global_position_s debug_pos;
	memset(&debug_pos, 0, sizeof(debug_pos));
	orb_advert_t debug_pos_pub = orb_advertise(ORB_ID(vehicle_global_position), &debug_pos);

	mission_sounds_init();
	bool sensors_ready = false;

	/* log init */
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	if (mavlink_fd < 0) {
		warnx("ERROR: Failed to open MAVLink log stream, start mavlink app first.\n");
	}

	/* mission states*/
	struct mission_state_s mission_state;
	init_state(&mission_state);

	/* mission */
	static bool new_waypoint = false;

	/* debug */
	float debug_value1 = 0.0f;
	float debug_value2 = 0.0f;
	int debug_value3 = 0;
	int debug_value4 = 0;

	/* parameters init*/
	struct mission_commander_flow_params params;
	struct mission_commander_flow_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "mission_commander_flow_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "mission_commander_flow_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "mission_commander_flow_err");

	while (!thread_should_exit) {

		if (sensors_ready) {
			/*This runs at the rate of the sensors */
			struct pollfd fds[5] = {
					{ .fd = vehicle_local_position_sub,		.events = POLLIN },
					{ .fd = discrete_radar_sub,   			.events = POLLIN },
					{ .fd = vehicle_local_waypoint_sub,		.events = POLLIN },
					{ .fd = manual_control_setpoint_sub, 	.events = POLLIN },
					{ .fd = parameter_update_sub,   		.events = POLLIN }
			};

			/* wait for a sensor update, check for exit condition every 500 ms */
			int ret = poll(fds, 5, 500);

			if (ret < 0) {
				/* poll error, count it in perf */
				perf_count(mc_err_perf);

			} else if (ret == 0) {
				/* no return value, ignore */
				printf("[mission commander] no updates.\n");

			} else {

				/* parameter update available? */
				if (fds[4].revents & POLLIN){
					/* read from param to clear updated flag */
					struct parameter_update_s update;
					orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

					parameters_update(&param_handles, &params);
					printf("[mission commander] parameters updated.\n");
				}

				/* new mission commands -> later from qgroundcontroll */
				if (fds[3].revents & POLLIN){
					/* NEW MANUAL CONTROLS------------------------------------------------------*/
					/* get a local copy of manual setpoint */
					orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual);
					/* get a local copy of the vehicle state */
					orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);

					if (vstatus.state_machine == SYSTEM_STATE_AUTO) {
						/* update mission flags */

						if (mission_state.state == MISSION_RESETED) {

							if (manual.aux2 < 0) {
								/* start mission planner */
								do_state_update(&mission_state, mavlink_fd, MISSION_READY);
							}

						} else {
							if (manual.aux2 > 0) {

								if (mission_state.state == MISSION_STARTED) {
									/* abort current mission */
									do_state_update(&mission_state, mavlink_fd, MISSION_ABORTED);
									/* set position setpoint to current position*/
									/* TODO do we need that? */
									bodyframe_pos_sp.x = bodyframe_pos.x;
									bodyframe_pos_sp.y = bodyframe_pos.y;
									bodyframe_pos_sp.yaw = att.yaw;

								} else {
									/* reset mission */
									do_state_update(&mission_state, mavlink_fd, MISSION_RESETED);

								}
							}
						}
					} else {

						if (mission_state.state != MISSION_RESETED) {
							do_state_update(&mission_state, mavlink_fd, MISSION_RESETED);
						}

					}
				}

				/* new waypoint available? */
				if (fds[2].revents & POLLIN){

					struct vehicle_local_waypoint_s local_wp;
					orb_copy(ORB_ID(vehicle_local_waypoint), vehicle_local_waypoint_sub, &local_wp);

					final_dest_local.x = local_wp.x;
					final_dest_local.y = local_wp.y;
					final_dest_local.z = local_wp.z;
					final_dest_local.yaw = local_wp.yaw;

					new_waypoint = true;

					mavlink_log_info(mavlink_fd, "[mission commander] got new local waypoint.");
					char buf[50] = {0};
					sprintf(buf, "[mission commander] WP x: %6.1f/y: %6.1f/yaw %6.1f",
							(double)final_dest_local.x, (double)final_dest_local.y, (double)final_dest_local.yaw);
					mavlink_log_info(mavlink_fd, buf);
					printf("[mission commander] got new local waypoint.\n");
				}


				/* only if radar changed */
				if (fds[1].revents & POLLIN){
					/* get a local copy of discrete radar */
					orb_copy(ORB_ID(discrete_radar), discrete_radar_sub, &discrete_radar);

					if (mission_state.state == MISSION_STARTED) {
						do_radar_update(&mission_state, &params, mavlink_fd, &discrete_radar);
					}
				}

				/* only if local position changed */
				if (fds[0].revents & POLLIN) {

					perf_begin(mc_loop_perf);

					/* get a local copy of manual setpoint */
					orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual);
					/* get a local copy of local position */
					orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &local_pos);
					/* get a local copy of bodyframe position */
					orb_copy(ORB_ID(vehicle_bodyframe_position), vehicle_bodyframe_position_sub, &bodyframe_pos);
					/* get a local copy of attitude */
					orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
					/* get a local copy of the vehicle state */
					orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);

					if (vstatus.state_machine == SYSTEM_STATE_AUTO) {

						/* calc new mission setpoint */
						if (mission_state.state == MISSION_READY) {

							/* do we have a destination ??? -> start mission */
							final_dest_bodyframe.x = bodyframe_pos_sp.x + params.mission_x_offset;
							final_dest_bodyframe.y = bodyframe_pos_sp.y + params.mission_y_offset;
							convert_setpoint_bodyframe2local(&local_pos,&bodyframe_pos,&att,&final_dest_bodyframe,&final_dest_local);
							final_dest_bodyframe.yaw = get_yaw(&local_pos, &final_dest_local); // changeable later
							do_state_update(&mission_state, mavlink_fd, MISSION_STARTED);

						} else if (mission_state.state == MISSION_STARTED) {

							/* no yaw correction in final sequence -> no need for waypoint update */
							if(!mission_state.final_sequence){
								/* calc final destination in bodyframe */
								convert_setpoint_local2bodyframe(&local_pos,&bodyframe_pos,&att,&final_dest_local,&final_dest_bodyframe);
							}

							/* ------------------------------------------------------------------------- *
							 *  NEED FOR REACTION														 *
							 * ------------------------------------------------------------------------- */
							if (mission_state.radar_current == RADAR_REACT_LEFT || mission_state.radar_current == RADAR_REACT_RIGHT) {

								/* react left */
								if (mission_state.radar_current == RADAR_REACT_LEFT) {

									if (mission_state.react == REACT_TURN) {
										if (!mission_state.wall_left) {
											/* do this update only if no wall on the left side */
											bodyframe_pos_sp.y = bodyframe_pos_sp.y - params.mission_update_step;
										}
										bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw - params.mission_update_step_yaw;

									} else if (mission_state.react == REACT_PASS_OBJECT) {
										/* go straight only if too near correct to left */
										if (discrete_radar.distances[23] < params.mission_min_side_dist) {
											bodyframe_pos_sp.y = bodyframe_pos_sp.y - params.mission_update_step;
										}

									} else if (mission_state.react == REACT_TEST) {


									}

								/* react right */
								} else if (mission_state.radar_current == RADAR_REACT_RIGHT) {

									if (mission_state.react == REACT_TURN) {
										if (!mission_state.wall_right) {
											/* do this update only if no wall on the right side */
											bodyframe_pos_sp.y = bodyframe_pos_sp.y + params.mission_update_step;
										}
										bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw + params.mission_update_step_yaw;

									} else if (mission_state.react == REACT_PASS_OBJECT) {
										/* go straight only if too near correct to right */
										if (discrete_radar.distances[23] < params.mission_min_side_dist) {
											bodyframe_pos_sp.y = bodyframe_pos_sp.y - params.mission_update_step;
										}

									} else if (mission_state.react == REACT_TEST) {

									}
								}

								bodyframe_pos_sp.x = bodyframe_pos_sp.x + params.mission_update_step; // we need flow... one step
								mission_state.reaction_counter++;



							/* ------------------------------------------------------------------------- *
							 *  GO CAREFULLY													`		 *
							 * ------------------------------------------------------------------------- */
							} else {

								/* ideal if we are parallel to wall... */

								float yaw_final = get_yaw(&local_pos, &final_dest_local);

								float yaw_error = yaw_final - att.yaw;

								if (yaw_error > M_PI_F) {
									yaw_error -= M_TWOPI_F;

								} else if (yaw_error < -M_PI_F) {
									yaw_error += M_TWOPI_F;
								}

								/* wait until yaw is approximately correct except if we are in final sequence */
								if (fabsf(yaw_error) < params.mission_yaw_thld || mission_state.final_sequence)
								{
									/* calc offsets */
									float wp_bodyframe_offset_x = final_dest_bodyframe.x - bodyframe_pos_sp.x;
									float wp_bodyframe_offset_y = final_dest_bodyframe.y - bodyframe_pos_sp.y;

									/* final mission sequence? */
									if(!mission_state.final_sequence){
										if (fabsf(wp_bodyframe_offset_x) < params.mission_wp_radius) {
											mission_state.final_sequence = true;
										}

									} else {
										if (fabsf(wp_bodyframe_offset_x) > params.mission_wp_radius) {
											mission_state.final_sequence = false;
										}
									}

									/* x */
									int x_steps = (int)(wp_bodyframe_offset_x / params.mission_update_step);


									if (x_steps == 0) {

										/* mission accomplished */
										do_state_update(&mission_state, mavlink_fd, MISSION_ACCOMPLISHED);

										bodyframe_pos_sp.x = bodyframe_pos_sp.x + wp_bodyframe_offset_x;
										bodyframe_pos_sp.y = bodyframe_pos_sp.y + wp_bodyframe_offset_y;

									} else {

										/* x and y update steps */
										bodyframe_pos_sp.x = bodyframe_pos_sp.x + sign(x_steps) * params.mission_update_step;
										float update_step_y = wp_bodyframe_offset_y / fabsf((float) x_steps);
										bodyframe_pos_sp.y = bodyframe_pos_sp.y + update_step_y;

										/* yaw set final only if not final sequence */
										if (!mission_state.final_sequence) {
											/* FIXME this makes problem */
											//bodyframe_pos_sp.yaw = yaw_final;
										}

										/* debug */
										debug_value1 = wp_bodyframe_offset_x;
										debug_value2 = wp_bodyframe_offset_y;
										debug_value3 = x_steps;

									}

									/* follow wall corrections if too near */
									if (!mission_state.final_sequence) {

										if (mission_state.radar_current == RADAR_FOLLOW_WALL_L) {
											/* go straight only if too near correct to right */
											if (discrete_radar.distances[9] < params.mission_min_side_dist) {
												bodyframe_pos_sp.y = bodyframe_pos_sp.y + params.mission_update_step;
											}

										} else if (mission_state.radar_current == RADAR_FOLLOW_WALL_R) {
											/* go straight only if too near correct to right */
											if (discrete_radar.distances[23] < params.mission_min_side_dist) {
												bodyframe_pos_sp.y = bodyframe_pos_sp.y - params.mission_update_step;
											}

										} else if (mission_state.radar_current == RADAR_FOLLOW_CORRIDOR) {
											/* TODO try also to get yaw in direction of corridor */
											if (discrete_radar.distances[9] < discrete_radar.distances[23]) {
												/* correct to right */
												bodyframe_pos_sp.y = bodyframe_pos_sp.y + params.mission_update_step;
											} else {
												/* correct to left */
												bodyframe_pos_sp.y = bodyframe_pos_sp.y - params.mission_update_step;
											}
										}

									}

								} else {

									/* turn to correct yaw position before starting to move */
									if (yaw_error > 0) {
										bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw + 2.0f * params.mission_update_step_yaw;
									} else {
										bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw - 2.0f * params.mission_update_step_yaw;
									}
								}
							}

						}

						/*
						 * manually update position setpoint -> e.g. overwrite commands
						 * from mission commander if something goes wrong
						 */
						if(manual.pitch < -0.2f) {
							bodyframe_pos_sp.x += 3.0f * params.mission_update_step;
						} else if (manual.pitch > 0.2f) {
							bodyframe_pos_sp.x -= 3.0f * params.mission_update_step;
						}

						if(manual.roll < -0.2f) {
							bodyframe_pos_sp.y -= 3.0f * params.mission_update_step;
						} else if (manual.roll > 0.2f) {
							bodyframe_pos_sp.y += 3.0f * params.mission_update_step;
						}

						if(manual.yaw < -1.0f) { // bigger threshold because of rc calibration for manual flight
							bodyframe_pos_sp.yaw -= 3.0f * params.mission_update_step_yaw;
						} else if (manual.yaw > 1.0f) {
							bodyframe_pos_sp.yaw += 3.0f * params.mission_update_step_yaw;
						}

						/* modulo for rotation -pi +pi */
						if(bodyframe_pos_sp.yaw < -M_PI_F) {
							bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw + M_TWOPI_F;
						} else if(bodyframe_pos_sp.yaw > M_PI_F) {
							bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw - M_TWOPI_F;
						}

					} else
					{
						/* if back in auto or stabilized mode reset setpoint to current position */
						bodyframe_pos_sp.x = bodyframe_pos.x;
						bodyframe_pos_sp.y = bodyframe_pos.y;
						bodyframe_pos_sp.yaw = att.yaw;
					}

					/* send new position setpoint */
					if(isfinite(bodyframe_pos_sp.x) && isfinite(bodyframe_pos_sp.y) && isfinite(bodyframe_pos_sp.yaw))
					{
						orb_publish(ORB_ID(vehicle_bodyframe_position_setpoint), vehicle_bodyframe_position_sp_pub, &bodyframe_pos_sp);
					} else
					{
						printf("[mission commander] invalid bodyframe position setpoint!!!\n");
					}

					convert_setpoint_bodyframe2local(&local_pos,&bodyframe_pos,&att,&bodyframe_pos_sp,&local_pos_sp);

					if(isfinite(local_pos_sp.x) && isfinite(local_pos_sp.y) && isfinite(local_pos_sp.yaw))
					{
						orb_publish(ORB_ID(vehicle_local_position_setpoint), vehicle_local_position_sp_pub, &local_pos_sp);
					} else
					{
						printf("[mission commander] invalid local position setpoint!!!\n");
					}

					/* TODO remove DEBUG */
					debug_pos.lon = (int)(debug_value1 * 100.0f);
					debug_pos.lat = (int)(debug_value2 * 100.0f);
					debug_pos.alt = debug_value3;
					debug_pos.relative_alt = debug_value4;
					orb_publish(ORB_ID(vehicle_global_position), debug_pos_pub, &debug_pos);

					/* measure in what intervals the mission commander runs */
					perf_count(mc_interval_perf);
					perf_end(mc_loop_perf);

				}

			}

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
				printf("[mission commander] no attitude received.\n");
			} else {

				if (fds[0].revents & POLLIN){
					sensors_ready = true;
					printf("[mission commander] initialized.\n");
				}
			}
		}

		counter++;
	}

	printf("[mission commander] exiting.\n");
	thread_running = false;

	close(parameter_update_sub);
	close(vehicle_attitude_sub);
	close(vehicle_local_position_sub);
	close(vehicle_bodyframe_position_sub);
	close(vehicle_status_sub);
	close(manual_control_setpoint_sub);
	close(omnidirectional_flow_sub);

	mission_sounds_deinit();

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	return 0;
}


