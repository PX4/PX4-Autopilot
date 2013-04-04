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
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_bodyframe_position.h>
#include <uORB/topics/vehicle_bodyframe_position_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/omnidirectional_flow.h>
#include <uORB/topics/discrete_radar.h>

#include <uORB/topics/vehicle_global_position.h>
#include <poll.h>

#include "mission_commander_flow_params.h"
#include "mission_sounds.h"

#define sign(x) (( x > 0 ) - ( x < 0 ))

__EXPORT int mission_commander_flow_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

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

	/* advert local/bodyframe position setpoint */
	orb_advert_t vehicle_local_position_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &local_pos_sp);
	orb_advert_t vehicle_bodyframe_position_sp_pub = orb_advertise(ORB_ID(vehicle_bodyframe_position_setpoint), &bodyframe_pos_sp);

	/* for debugging */
	struct vehicle_global_position_s debug_pos;
	memset(&debug_pos, 0, sizeof(debug_pos));
	orb_advert_t debug_pos_pub = orb_advertise(ORB_ID(vehicle_global_position), &debug_pos);

	mission_sounds_init();
	bool sensors_ready = false;

	/* mission states*/
	static bool mission_started = false;
	static bool mission_accomplished = false;
	static bool mission_aborted = false;
	mission_reaction_state_t react_state;
	int remaining_sp = 0;

	/* mission flags */
	static bool start_mission = false;
	static bool abort_mission = false;
	static bool reset_mission = false;
	static bool finish_mission = false;

	/* mission */
	static bool final_sequence = false;

	/* debug */
	float debug_value1 = 0.0f;
	float debug_value2 = 0.0f;
	int debug_value3 = 0;
	int debug_value4 = 0;

	/* manual control overwrite parameters */
	static float setpoint_update_step = 0.01f;
	static float setpoint_update_step_yaw = 0.05f;

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
			struct pollfd fds[3] = {
					{ .fd = discrete_radar_sub,   			.events = POLLIN },
					{ .fd = manual_control_setpoint_sub, 	.events = POLLIN },
					{ .fd = parameter_update_sub,   		.events = POLLIN }
			};

			/* wait for a sensor update, check for exit condition every 500 ms */
			int ret = poll(fds, 3, 500);

			if (ret < 0) {
				/* poll error, count it in perf */
				perf_count(mc_err_perf);

			} else if (ret == 0) {
				/* no return value, ignore */
				printf("[mission commander] no radar updates.\n");

			} else {

				/* parameter update available? */
				if (fds[2].revents & POLLIN){
					/* read from param to clear updated flag */
					struct parameter_update_s update;
					orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

					parameters_update(&param_handles, &params);
					printf("[mission commander] parameters updated.\n");
				}

				/* new mission commands -> later from qgroundcontroll */
				if (fds[1].revents & POLLIN){
					/* NEW MANUAL CONTROLS------------------------------------------------------*/
					/* get a local copy of manual setpoint */
					orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual);
					/* get a local copy of the vehicle state */
					orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);

					if (vstatus.state_machine == SYSTEM_STATE_AUTO) {
						/* update mission flags */
						if (!mission_started && !mission_accomplished && !mission_aborted) {
							if (manual.aux2 < 0) {
								/* start mission planner */
								start_mission = true;
							}
						} else {
							if (manual.aux2 > 0) {

								if (mission_started){
									/* abort current mission */
									abort_mission = true;
								}
							}
						}
					}
				}

				/* only if flow data changed */
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
					/* get a local copy of discrete radar */
					orb_copy(ORB_ID(discrete_radar), discrete_radar_sub, &discrete_radar);

					/* do mission flag commands */
					if (start_mission) {
						/* only one mission... 2m forward */
						/* TODO do it with current setpoint??? DONE*/
						final_dest_bodyframe.x = bodyframe_pos_sp.x + params.mission_x_offset;
						final_dest_bodyframe.y = bodyframe_pos_sp.y + params.mission_y_offset;
						convert_setpoint_bodyframe2local(&local_pos,&bodyframe_pos,&att,&final_dest_bodyframe,&final_dest_local);
						final_dest_bodyframe.yaw = get_yaw(&local_pos, &final_dest_local); // changeable later

						mission_accomplished = false;
						mission_aborted = false;
						mission_started = true;
						react_state = MISSION_CLEAR;

						bodyframe_pos_sp.x = bodyframe_pos.x;
						bodyframe_pos_sp.y = bodyframe_pos.y;
						bodyframe_pos_sp.yaw = get_yaw(&local_pos,&final_dest_local);

						printf("[mission commander] mission started.\n");
						tune_mission_started();
						start_mission = false;
					}

					if (finish_mission) {
						/* reset mission planner */
						mission_started = false;
						mission_aborted = false;
						mission_accomplished = true;
						react_state = MISSION_CLEAR;

						final_sequence = false;

						printf("[mission commander] mission accomplished.\n");
						tune_mission_accomplished();
						finish_mission = false;
					}

					if (abort_mission) {
						/* reset mission planner */
						mission_started = false;
						mission_aborted = true;
						mission_accomplished = false;
						react_state = MISSION_CLEAR;

						final_sequence = false;

						/* set position setpoint to current position*/
						bodyframe_pos_sp.x = bodyframe_pos.x;
						bodyframe_pos_sp.y = bodyframe_pos.y;
						bodyframe_pos_sp.yaw = att.yaw;

						printf("[mission commander] mission aborted.\n");
						tune_mission_aborted();
						abort_mission = false;
					}

					if (reset_mission) {
						/* reset mission planner */
						mission_started = false;
						mission_aborted = false;
						mission_accomplished = false;
						react_state = MISSION_CLEAR;

						final_sequence = false;

						/* set position setpoint to current position*/
						bodyframe_pos_sp.x = bodyframe_pos.x;
						bodyframe_pos_sp.y = bodyframe_pos.y;
						bodyframe_pos_sp.yaw = att.yaw;

						printf("[mission commander] mission reseted.\n");
						reset_mission = false;
					}

					if (vstatus.state_machine == SYSTEM_STATE_AUTO) {

						/* calc new mission setpoint */
						if (mission_started) {

							do {
								/* no yaw correction in final sequence -> no need for waypoint update */
								if(!final_sequence){
									/* calc final destination in bodyframe */
									convert_setpoint_local2bodyframe(&local_pos,&bodyframe_pos,&att,&final_dest_local,&final_dest_bodyframe);
								}

								/* looking for lokal situation */
								bool front_free = true;
								bool front_react = false;
								bool wall_left = false;
								bool wall_right = false;

								for(int i = 14; i<19; i++)
								{
									if (discrete_radar.distances[i] < params.mission_min_dist) {
										front_free = false;
									} else if(discrete_radar.distances[i] < params.mission_reac_dist) {
										front_react = true;
									}
								}

								/* calc side-freeness simple */
								int left = (discrete_radar.distances[9] + discrete_radar.distances[10] + discrete_radar.distances[11]) / 3;
								int right = (discrete_radar.distances[21] + discrete_radar.distances[22] + discrete_radar.distances[23]) / 3;

								if(react_state == MISSION_CLEAR) {
									/* TODO what if a wall appears? */
									if(!front_free || front_react ) {
										/* obstacle is detected */
										if (left > right) {
											react_state = MISSION_REACT_LEFT;
										} else {
											react_state = MISSION_REACT_RIGHT;
										}
									} else if (left < params.mission_reac_dist) {
										react_state = MISSION_FOLLOW_WALL_L;
									} else if (right < params.mission_reac_dist) {
										react_state = MISSION_FOLLOW_WALL_R;
									}

								}

								if(!front_free) {
									/* stand still and make nothing waiting for better weather */
									/* TODO add problem solving */
									printf("[mission commander] %i too close to wall.\n", params.mission_min_dist);
									abort_mission = true;
									break;
								}

								/* ------------------------------------------------------------------------- *
								 *  NEED FOR REACTION														 *
								 * ------------------------------------------------------------------------- */
								if (react_state != MISSION_CLEAR)
								{

									/* do a reaction */
									if (left < params.mission_reac_dist) {
										wall_left = true;
									} else {
										wall_left = false;
									}
									if (right < params.mission_reac_dist) {
										wall_right = true;
									} else {
										wall_right = false;
									}

									bodyframe_pos_sp.x = bodyframe_pos_sp.x + params.mission_update_step; // we need flow... one step

									if (react_state == MISSION_REACT_LEFT && !wall_left) {
										bodyframe_pos_sp.y = bodyframe_pos_sp.y - params.mission_update_step;
										bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw - params.mission_update_step_yaw;

									} else if (react_state == MISSION_REACT_RIGHT && !wall_right) {
										bodyframe_pos_sp.y = bodyframe_pos_sp.y + params.mission_update_step;
										bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw + params.mission_update_step_yaw;

									} else if (react_state == MISSION_FOLLOW_WALL_L) {
										if (discrete_radar.distances[9] < params.mission_min_dist) {
											bodyframe_pos_sp.y = bodyframe_pos_sp.y + params.mission_update_step;
										}

									} else if (react_state == MISSION_FOLLOW_WALL_R) {
										if (discrete_radar.distances[23] < params.mission_min_dist) {
											bodyframe_pos_sp.y = bodyframe_pos_sp.y - params.mission_update_step;
										}

									} else if (react_state == MISSION_FOLLOW_CORRIDOR) {
										/* TODO try also to get yaw in direction of corridor */
										if (discrete_radar.distances[9] < discrete_radar.distances[23]) {
											/* correct to right */
											bodyframe_pos_sp.y = bodyframe_pos_sp.y + params.mission_update_step;
										} else {
											/* correct to left */
											bodyframe_pos_sp.y = bodyframe_pos_sp.y - params.mission_update_step;
										}
									}

									/* do we need a state change */
									int front_dist = (discrete_radar.distances[15] + discrete_radar.distances[16] + discrete_radar.distances[17]) / 3;

									if (react_state == MISSION_FOLLOW_WALL_L || react_state == MISSION_FOLLOW_WALL_R) {
										if (wall_left && wall_right) {
											react_state = MISSION_FOLLOW_CORRIDOR;
										}
									} if (react_state == MISSION_FOLLOW_CORRIDOR) {
										if (wall_left && !wall_right) {
											react_state = MISSION_FOLLOW_WALL_L;
										} else if (!wall_left && !wall_right) {
											react_state = MISSION_FOLLOW_WALL_R;
										}
									}

									if (react_state == MISSION_REACT_LEFT || react_state == MISSION_REACT_RIGHT) {

										if (front_dist > params.mission_reac_dist) {
											if (react_state == MISSION_REACT_LEFT) {
												react_state = MISSION_FOLLOW_WALL_R;
											} else {
												react_state = MISSION_FOLLOW_WALL_L;
											}
										}

									} else if (react_state == MISSION_FOLLOW_WALL_L || react_state == MISSION_FOLLOW_WALL_R) {

										if (front_dist < params.mission_reac_dist) {

											if (react_state == MISSION_FOLLOW_WALL_L) {
												react_state = MISSION_REACT_RIGHT;
											} else {
												react_state = MISSION_REACT_LEFT;
											}
										}

									} else if (react_state == MISSION_FOLLOW_CORRIDOR) {
										if (front_dist < params.mission_reac_dist) {
											abort_mission = true;
										}
									}

									/* if way is free change mission state */
									bool free_radar = true;
									for(int i = 0; i<32; i++)
									{
										if(discrete_radar.distances[i] < params.mission_reac_dist) {
											free_radar = false;
										}
									}
									if (free_radar) {
										react_state = MISSION_CLEAR;
										/* recalculate direction to the final destination */
										bodyframe_pos_sp.yaw = get_yaw(&local_pos, &final_dest_local);
									}

								/* ------------------------------------------------------------------------- *
								 *  CLEAR TO GO																 *
								 * ------------------------------------------------------------------------- */
								} else
								{

									/* wait until yaw is approximately correct */
									if (fabsf(att.yaw - bodyframe_pos_sp.yaw) < params.mission_yaw_thld)
									{
										/* calc offsets */
										float wp_bodyframe_offset_x = final_dest_bodyframe.x - bodyframe_pos_sp.x;
										float wp_bodyframe_offset_y = final_dest_bodyframe.y - bodyframe_pos_sp.y;

										/* final mission sequence? */
										if(!final_sequence){
											if (fabsf(wp_bodyframe_offset_x) < params.mission_wp_radius) {
												final_sequence = true;
											}

										} else {
											if (fabsf(wp_bodyframe_offset_x) > params.mission_wp_radius) {
												final_sequence = false;
											}
										}

										/* x */
										int x_steps = (int)(wp_bodyframe_offset_x / params.mission_update_step);

										bodyframe_pos_sp.x = bodyframe_pos_sp.x + sign(x_steps) * params.mission_update_step;
										float update_step_y = wp_bodyframe_offset_y / fabsf((float) x_steps);
										bodyframe_pos_sp.y = bodyframe_pos_sp.y + update_step_y;

										debug_value1 = wp_bodyframe_offset_x;
										debug_value2 = wp_bodyframe_offset_y;
										debug_value3 = x_steps;

										if (x_steps == 0) {
											finish_mission = true;
										}

									} else {
										bodyframe_pos_sp.yaw = get_yaw(&local_pos, &final_dest_local);
									}
								}

							} while (false); // just to use break
						}

						/*
						 * manually update position setpoint -> e.g. overwrite commands
						 * from mission commander if something goes wrong
						 */
						if(manual.pitch < -0.2f) {
							bodyframe_pos_sp.x += setpoint_update_step;
						} else if (manual.pitch > 0.2f) {
							bodyframe_pos_sp.x -= setpoint_update_step;
						}

						if(manual.roll < -0.2f) {
							bodyframe_pos_sp.y -= setpoint_update_step;
						} else if (manual.roll > 0.2f) {
							bodyframe_pos_sp.y += setpoint_update_step;
						}

						if(manual.yaw < -1.0f) { // bigger threshold because of rc calibration for manual flight
							bodyframe_pos_sp.yaw -= setpoint_update_step_yaw;
						} else if (manual.yaw > 1.0f) {
							bodyframe_pos_sp.yaw += setpoint_update_step_yaw;
						}

						/* modulo for rotation -pi +pi */
						if(bodyframe_pos_sp.yaw < -M_PI_F) {
							bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw + M_TWOPI_F;
						} else if(bodyframe_pos_sp.yaw > M_PI_F) {
							bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw - M_TWOPI_F;
						}

					} else
					{
						if (mission_started || mission_accomplished || mission_aborted) {
							reset_mission = true;
						}

						/* if back in auto or stabilized mode reset setpoint to current position */
						bodyframe_pos_sp.x = bodyframe_pos.x;
						bodyframe_pos_sp.y = bodyframe_pos.y;
						bodyframe_pos_sp.yaw = att.yaw;
					}

					/* send new position setpoint */
					if(isfinite(bodyframe_pos_sp.x) && isfinite(bodyframe_pos_sp.y) && isfinite(bodyframe_pos_sp.yaw))
					{
						orb_publish(ORB_ID(vehicle_bodyframe_position_setpoint), vehicle_bodyframe_position_sp_pub, &bodyframe_pos_sp);
					}

					convert_setpoint_bodyframe2local(&local_pos,&bodyframe_pos,&att,&bodyframe_pos_sp,&local_pos_sp);

					if(isfinite(local_pos_sp.x) && isfinite(local_pos_sp.y) && isfinite(local_pos_sp.yaw))
					{
						orb_publish(ORB_ID(vehicle_local_position_setpoint), vehicle_local_position_sp_pub, &local_pos_sp);
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


