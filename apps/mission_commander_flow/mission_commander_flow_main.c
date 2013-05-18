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
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_vicon_position.h>
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
					 8192,
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
	struct discrete_radar_s discrete_radar;
	memset(&discrete_radar, 0, sizeof(discrete_radar));

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
//	struct vehicle_global_position_s debug_pos;
//	memset(&debug_pos, 0, sizeof(debug_pos));
//	orb_advert_t debug_pos_pub = orb_advertise(ORB_ID(vehicle_global_position), &debug_pos);
//	struct vehicle_vicon_position_s vicon_pos;
//	memset(&vicon_pos, 0, sizeof(vicon_pos));
//	int vehicle_vicon_bos_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
	struct vehicle_gps_position_s debug_pos;
	memset(&debug_pos, 0, sizeof(debug_pos));
	orb_advert_t debug_pos_pub = orb_advertise(ORB_ID(vehicle_gps_position), &debug_pos);

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

	/* parameters init*/
	struct mission_commander_flow_params params;
	struct mission_commander_flow_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	/* radar front situation update*/
	static float position_last[2] = { 0.0f };
	static float position_update[2] = { 0.0f };
	static float yaw_last = 0.0f;
	static float yaw_update = 0.0f;
	static bool update_initialized = false;

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

//					struct vehicle_local_waypoint_s local_wp;
//					orb_copy(ORB_ID(vehicle_local_waypoint), vehicle_local_waypoint_sub, &local_wp);
//
//					final_dest_local.x = local_wp.x;
//					final_dest_local.y = local_wp.y;
//					final_dest_local.z = local_wp.z;
//					final_dest_local.yaw = local_wp.yaw;
//
//					new_waypoint = true;
//
//					mavlink_log_info(mavlink_fd, "[mission commander] got new local waypoint.");
//					char buf[50] = {0};
//					sprintf(buf, "[mission commander] WP x: %6.1f/y: %6.1f/yaw %6.1f",
//							(double)final_dest_local.x, (double)final_dest_local.y, (double)final_dest_local.yaw);
//					mavlink_log_info(mavlink_fd, buf);
//					printf("[mission commander] got new local waypoint.\n");

				}


				/* only if radar changed */
				if (fds[1].revents & POLLIN){
					/* get a local copy of discrete radar */
					orb_copy(ORB_ID(discrete_radar), discrete_radar_sub, &discrete_radar);
					/* get a local copy of attitude */
					orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
					/* get a local copy of local position */
					orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &local_pos);
					/* get a local copy of bodyframe position */
					orb_copy(ORB_ID(vehicle_bodyframe_position), vehicle_bodyframe_position_sub, &bodyframe_pos);

					if (mission_state.state == MISSION_STARTED || params.debug) {
						/* test if enough space */
						if ((discrete_radar.sonar * 1000.0f) > params.mission_min_front_dist) {

							/* update sonar obstacle if valid */
							if (mission_state.sonar_obstacle.valid) {
								convert_setpoint_local2bodyframe(&local_pos, &bodyframe_pos, &att,
										&mission_state.sonar_obstacle.sonar_obstacle_local,
										&mission_state.sonar_obstacle.sonar_obstacle_bodyframe);

								/* convert to polar */
								float x_bodyframe_off = mission_state.sonar_obstacle.sonar_obstacle_bodyframe.x - bodyframe_pos.x;
								float y_bodyframe_off = mission_state.sonar_obstacle.sonar_obstacle_bodyframe.y - bodyframe_pos.y;

								if (x_bodyframe_off > 0.0f) {
									mission_state.sonar_obstacle.sonar_obst_polar_r =
											sqrtf(	x_bodyframe_off * x_bodyframe_off + y_bodyframe_off * y_bodyframe_off);

									mission_state.sonar_obstacle.sonar_obst_polar_alpha =
											atan2f( y_bodyframe_off, x_bodyframe_off);
								} else {
									mission_state.sonar_obstacle.valid = false;
								}
							}

//							/* DEBUG: LOG AS GPS */
//							if(mission_state.sonar_obstacle.valid) {
//
//								mission_state.debug_value1 = mission_state.sonar_obstacle.sonar_obst_polar_alpha;
//								mission_state.debug_value2 = mission_state.sonar_obstacle.sonar_obst_polar_r;
//								mission_state.debug_value3 = mission_state.sonar_obstacle.sonar_obst_pitch;
//
//							} else {
//								mission_state.debug_value1 = 0.0f;
//								mission_state.debug_value2 = 0.0f;
//								mission_state.debug_value3 = 0.0f;
//
//							}

							do_radar_update(&mission_state, &params, mavlink_fd, &discrete_radar);

							if(mission_state.sonar_obstacle.valid && mission_state.sonar_obstacle.updated) {
								/* create new sonar obstacle and convert to local frame */
								mission_state.sonar_obstacle.sonar_obstacle_bodyframe.x = bodyframe_pos.x +
										mission_state.sonar_obstacle.sonar_obst_polar_r;
								mission_state.sonar_obstacle.sonar_obstacle_bodyframe.y = bodyframe_pos.y;

								convert_setpoint_bodyframe2local(&local_pos, &bodyframe_pos, &att,
										&mission_state.sonar_obstacle.sonar_obstacle_bodyframe,
										&mission_state.sonar_obstacle.sonar_obstacle_local);
							}

						} else {

							if(mission_state.state == MISSION_STARTED)
							{
								/* abord mission */
								do_state_update(&mission_state, mavlink_fd, MISSION_ABORTED);
							}

						}
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
							convert_setpoint_bodyframe2local(&local_pos, &bodyframe_pos,&att,
									&final_dest_bodyframe, &final_dest_local);
							final_dest_bodyframe.yaw = get_yaw(&local_pos, &final_dest_local); // changeable later
							do_state_update(&mission_state, mavlink_fd, MISSION_STARTED);

						} else if (mission_state.state == MISSION_STARTED) {

							/* no yaw correction in final sequence -> no need for waypoint update */
							if(!mission_state.final_sequence){
								/* calc final destination in bodyframe */
								convert_setpoint_local2bodyframe(&local_pos, &bodyframe_pos, &att,
										&final_dest_local, &final_dest_bodyframe);
							}

							/* calc yaw to final destination */
							float yaw_final = get_yaw(&local_pos, &final_dest_local);

//							float yaw_error = yaw_final - att.yaw;
//
//							if (yaw_error > M_PI_F) {
//								yaw_error -= M_TWOPI_F;
//
//							} else if (yaw_error < -M_PI_F) {
//								yaw_error += M_TWOPI_F;
//							}

							float yaw_sp_error = yaw_final - bodyframe_pos_sp.yaw;

							if (yaw_sp_error > M_PI_F) {
								yaw_sp_error -= M_TWOPI_F;

							} else if (yaw_sp_error < -M_PI_F) {
								yaw_sp_error += M_TWOPI_F;
							}

							if (!mission_state.initialized) {
								/* correct yaw first */

								if (fabsf(yaw_sp_error) < params.mission_update_step_yaw) {
									mission_state.initialized = true;
									bodyframe_pos_sp.yaw = yaw_final;
								} else {
									if (yaw_sp_error > 0) {
										bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw + params.mission_update_step_yaw;
									} else {
										bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw - params.mission_update_step_yaw;
									}
								}

							} else {

								if (mission_state.free_to_go || mission_state.final_sequence) {
									/* there are no obstacles */

									/* correct yaw if needed */
									if (fabsf(yaw_sp_error) > params.mission_update_step_yaw && !mission_state.final_sequence) {
										if (yaw_sp_error > 0) {
											bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw + params.mission_update_step_yaw;
										} else {
											bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw - params.mission_update_step_yaw;
										}

										/* we need flow... step by step */
										bodyframe_pos_sp.x = bodyframe_pos_sp.x + params.mission_update_step_x;

//									if (fabsf(yaw_sp_error) > params.mission_yaw_thld && !mission_state.final_sequence) {
//
//										/* we need flow... step by step */
//										bodyframe_pos_sp.x = bodyframe_pos_sp.x + params.mission_update_step_x;
//
//										/* is setpoint already correct */
//
//
//										if (yaw_sp_error > 0 &&  ) {
//											bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw + params.mission_update_step_yaw;
//										} else {
//											bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw - params.mission_update_step_yaw;
//										}

									} else {

										/* calc offsets to final destination*/
										float wp_bodyframe_offset_x = final_dest_bodyframe.x - bodyframe_pos_sp.x;
										float wp_bodyframe_offset_y = final_dest_bodyframe.y - bodyframe_pos_sp.y;

										/* final mission sequence? */
										if(!mission_state.final_sequence){
											bodyframe_pos_sp.yaw = yaw_final;

											if (fabsf(wp_bodyframe_offset_x) < params.mission_wp_radius) {
												mission_state.final_sequence = true;
											}

										} else {

											if (fabsf(wp_bodyframe_offset_x) > params.mission_wp_radius) {
												mission_state.final_sequence = false;
											}
										}

										/* x */
										int x_steps = (int)(wp_bodyframe_offset_x / params.mission_update_step_x);

										if (x_steps == 0) {

											/* mission accomplished */
											do_state_update(&mission_state, mavlink_fd, MISSION_ACCOMPLISHED);

											bodyframe_pos_sp.x = bodyframe_pos_sp.x + wp_bodyframe_offset_x;
											bodyframe_pos_sp.y = bodyframe_pos_sp.y + wp_bodyframe_offset_y;

										} else {

											/* x and y update steps */
											bodyframe_pos_sp.x = bodyframe_pos_sp.x + sign(x_steps) * params.mission_update_step_x;
											float update_step_y = wp_bodyframe_offset_y / fabsf((float) x_steps);
											bodyframe_pos_sp.y = bodyframe_pos_sp.y + update_step_y;

										}
									}

								} else {

									/* let radar controller do his job */
									bodyframe_pos_sp.yaw = bodyframe_pos_sp.yaw + mission_state.step.yaw;
									bodyframe_pos_sp.x = bodyframe_pos_sp.x + mission_state.step.x;
									bodyframe_pos_sp.y = bodyframe_pos_sp.y + mission_state.step.y;
								}

							}

						}

						/*
						 * manually update position setpoint -> e.g. overwrite commands
						 * from mission commander if something goes wrong
						 */
						if(manual.pitch < -0.2f) {
							bodyframe_pos_sp.x += params.mission_update_step_x;
						} else if (manual.pitch > 0.2f) {
							bodyframe_pos_sp.x -= params.mission_update_step_x;
						}

						if(manual.roll < -0.2f) {
							bodyframe_pos_sp.y -= params.mission_update_step_x;
						} else if (manual.roll > 0.2f) {
							bodyframe_pos_sp.y += params.mission_update_step_x;
						}

						if(manual.yaw < -1.0f) { // bigger threshold because of rc calibration for manual flight
							bodyframe_pos_sp.yaw -= 2.0f * params.mission_update_step_yaw;
						} else if (manual.yaw > 1.0f) {
							bodyframe_pos_sp.yaw += 2.0f * params.mission_update_step_yaw;
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


					/* DEBUG: LOG AS GPS */

					/* TODO remove DEBUG */
//					debug_pos.alt = mission_state.debug_value1;
//					debug_pos.relative_alt = mission_state.debug_value2;
//					debug_pos.lon = mission_state.debug_value3;
//					debug_pos.lat = mission_state.debug_value4;

					debug_pos.lat = (int32_t) mission_state.state;
					debug_pos.lon = (int32_t) mission_state.sonar_obstacle.valid;
					debug_pos.alt = (int32_t) mission_state.free_to_go;

//					orb_publish(ORB_ID(vehicle_global_position), debug_pos_pub, &debug_pos);
					orb_publish(ORB_ID(vehicle_gps_position), debug_pos_pub, &debug_pos);

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


