/****************************************************************************
 *
 *   Copyright (C) 2008-2013 PX4 Development Team. All rights reserved.
 *   Author: Samuel Zihlmann <samuezih@ee.ethz.ch>
 *   		 Lorenz Meier <lm@inf.ethz.ch>
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
 * Optical flow position controller
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
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_bodyframe_speed_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <poll.h>
#include <mavlink/mavlink_log.h>

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
 * to task_spawn_cmd().
 */
int flow_position_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start"))
	{
		if (thread_running)
		{
			printf("flow position control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd("flow_position_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 6,
					 4096,
					 flow_position_control_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop"))
	{
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status"))
	{
		if (thread_running)
			printf("\tflow position control app is running\n");
		else
			printf("\tflow position control app not started\n");

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
	static int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[fpc] started");

	uint32_t counter = 0;
	const float time_scale = powf(10.0f,-6.0f);

	/* structures */
	struct actuator_armed_s armed;
	memset(&armed, 0, sizeof(armed));
	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct filtered_bottom_flow_s filtered_flow;
	memset(&filtered_flow, 0, sizeof(filtered_flow));
	struct vehicle_local_position_s local_pos;
	memset(&local_pos, 0, sizeof(local_pos));
	struct vehicle_bodyframe_speed_setpoint_s speed_sp;
	memset(&speed_sp, 0, sizeof(speed_sp));

	/* subscribe to attitude, motor setpoints and system state */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	int manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int filtered_bottom_flow_sub = orb_subscribe(ORB_ID(filtered_bottom_flow));
	int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	orb_advert_t speed_sp_pub;
	bool speed_setpoint_adverted = false;

	/* parameters init*/
	struct flow_position_control_params params;
	struct flow_position_control_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	/* init flow sum setpoint */
	float flow_sp_sumx = 0.0f;
	float flow_sp_sumy = 0.0f;

	/* init yaw setpoint */
	float yaw_sp = 0.0f;

	/* init height setpoint */
	float height_sp = params.height_min;

	/* height controller states */
	bool start_phase = true;
	bool landing_initialized = false;
	float landing_thrust_start = 0.0f;

	/* states */
	float integrated_h_error = 0.0f;
	float last_local_pos_z = 0.0f;
	bool update_flow_sp_sumx = false;
	bool update_flow_sp_sumy = false;
	uint64_t last_time = 0.0f;
	float dt = 0.0f; // s


	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "flow_position_control_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "flow_position_control_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "flow_position_control_err");

	static bool sensors_ready = false;
	static bool status_changed = false;

	while (!thread_should_exit)
	{
		/* wait for first attitude msg to be sure all data are available */
		if (sensors_ready)
		{
			/* polling */
			struct pollfd fds[2] = {
				{ .fd = filtered_bottom_flow_sub, .events = POLLIN }, // positions from estimator
				{ .fd = parameter_update_sub,   .events = POLLIN }

			};

			/* wait for a position update, check for exit condition every 500 ms */
			int ret = poll(fds, 2, 500);

			if (ret < 0)
			{
				/* poll error, count it in perf */
				perf_count(mc_err_perf);
			}
			else if (ret == 0)
			{
				/* no return value, ignore */
//				printf("[flow position control] no filtered flow updates\n");
			}
			else
			{
				/* parameter update available? */
				if (fds[1].revents & POLLIN)
				{
					/* read from param to clear updated flag */
					struct parameter_update_s update;
					orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

					parameters_update(&param_handles, &params);
					mavlink_log_info(mavlink_fd,"[fpc] parameters updated.");
				}

				/* only run controller if position/speed changed */
				if (fds[0].revents & POLLIN)
				{
					perf_begin(mc_loop_perf);

					/* get a local copy of the vehicle state */
					orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
					/* get a local copy of manual setpoint */
					orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual);
					/* get a local copy of attitude */
					orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
					/* get a local copy of filtered bottom flow */
					orb_copy(ORB_ID(filtered_bottom_flow), filtered_bottom_flow_sub, &filtered_flow);
					/* get a local copy of local position */
					orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &local_pos);
					/* get a local copy of control mode */
					orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);

					if (control_mode.flag_control_velocity_enabled)
					{
						float manual_pitch = manual.pitch / params.rc_scale_pitch; // 0 to 1
						float manual_roll = manual.roll / params.rc_scale_roll; // 0 to 1
						float manual_yaw = manual.yaw / params.rc_scale_yaw; // -1 to 1

						if(status_changed == false)
							mavlink_log_info(mavlink_fd,"[fpc] flow POSITION control engaged");

						status_changed = true;

						/* calc dt */
						if(last_time == 0)
						{
							last_time = hrt_absolute_time();
							continue;
						}
						dt = ((float) (hrt_absolute_time() - last_time)) * time_scale;
						last_time = hrt_absolute_time();

						/* update flow sum setpoint */
						if (update_flow_sp_sumx)
						{
							flow_sp_sumx = filtered_flow.sumx;
							update_flow_sp_sumx = false;
						}
						if (update_flow_sp_sumy)
						{
							flow_sp_sumy = filtered_flow.sumy;
							update_flow_sp_sumy = false;
						} 

						/* calc new bodyframe speed setpoints */
						float speed_body_x = (flow_sp_sumx - filtered_flow.sumx) * params.pos_p - filtered_flow.vx * params.pos_d;
						float speed_body_y = (flow_sp_sumy - filtered_flow.sumy) * params.pos_p - filtered_flow.vy * params.pos_d;
						float speed_limit_height_factor = height_sp; // the settings are for 1 meter

						/* overwrite with rc input if there is any */
						if(isfinite(manual_pitch) && isfinite(manual_roll))
						{
							if(fabsf(manual_pitch) > params.manual_threshold)
							{
								speed_body_x = -manual_pitch * params.limit_speed_x * speed_limit_height_factor;
								update_flow_sp_sumx = true;
							}

							if(fabsf(manual_roll) > params.manual_threshold)
							{
								speed_body_y = manual_roll * params.limit_speed_y * speed_limit_height_factor;
								update_flow_sp_sumy = true;
							}
						}

						/* limit speed setpoints */
						if((speed_body_x <= params.limit_speed_x * speed_limit_height_factor) &&
								(speed_body_x >= -params.limit_speed_x * speed_limit_height_factor))
						{
							speed_sp.vx = speed_body_x;
						}
						else
						{
							if(speed_body_x > params.limit_speed_x * speed_limit_height_factor)
								speed_sp.vx = params.limit_speed_x * speed_limit_height_factor;
							if(speed_body_x < -params.limit_speed_x * speed_limit_height_factor)
								speed_sp.vx = -params.limit_speed_x * speed_limit_height_factor;
						}

						if((speed_body_y <= params.limit_speed_y * speed_limit_height_factor) &&
								(speed_body_y >= -params.limit_speed_y * speed_limit_height_factor))
						{
							speed_sp.vy = speed_body_y;
						}
						else
						{
							if(speed_body_y > params.limit_speed_y * speed_limit_height_factor)
								speed_sp.vy = params.limit_speed_y * speed_limit_height_factor;
							if(speed_body_y < -params.limit_speed_y * speed_limit_height_factor)
								speed_sp.vy = -params.limit_speed_y * speed_limit_height_factor;
						}

						/* manual yaw change */
						if(isfinite(manual_yaw) && isfinite(manual.throttle))
						{
							if(fabsf(manual_yaw) > params.manual_threshold && manual.throttle > 0.2f)
							{
								yaw_sp += manual_yaw * params.limit_yaw_step;

								/* modulo for rotation -pi +pi */
								if(yaw_sp < -M_PI_F)
									yaw_sp = yaw_sp + M_TWOPI_F;
								else if(yaw_sp > M_PI_F)
									yaw_sp = yaw_sp - M_TWOPI_F;
							}
						}

						/* forward yaw setpoint */
						speed_sp.yaw_sp = yaw_sp;


						/* manual height control
						 * 0-20%: thrust linear down
						 * 20%-40%: down
						 * 40%-60%: stabilize altitude
						 * 60-100%: up
						 */
						float thrust_control = 0.0f;

						if (isfinite(manual.throttle))
						{
							if (start_phase)
							{
								/* control start thrust with stick input */
								if (manual.throttle < 0.4f)
								{
									/* first 40% for up to feedforward */
									thrust_control = manual.throttle / 0.4f * params.thrust_feedforward;
								}
								else
								{
									/* second 60% for up to feedforward + 10% */
									thrust_control = (manual.throttle - 0.4f) / 0.6f * 0.1f + params.thrust_feedforward;
								}

								/* exit start phase if setpoint is reached */
								if (height_sp < -local_pos.z && thrust_control > params.limit_thrust_lower)
								{
									start_phase = false;
									/* switch to stabilize */
									thrust_control = params.thrust_feedforward;
								}
							}
							else
							{
								if (manual.throttle < 0.2f)
								{
									/* landing initialization */
									if (!landing_initialized)
									{
										/* consider last thrust control to avoid steps */
										landing_thrust_start = speed_sp.thrust_sp;
										landing_initialized = true;
									}

									/* set current height as setpoint to avoid steps */
									if (-local_pos.z > params.height_min)
										height_sp = -local_pos.z;
									else
										height_sp = params.height_min;

									/* lower 20% stick range controls thrust down */
									thrust_control = manual.throttle / 0.2f * landing_thrust_start;

									/* assume ground position here */
									if (thrust_control < 0.1f)
									{
										/* reset integral if on ground */
										integrated_h_error = 0.0f;
										/* switch to start phase */
										start_phase = true;
										/* reset height setpoint */
										height_sp = params.height_min;
									}
								}
								else
								{
									/* stabilized mode */
									landing_initialized = false;

									/* calc new thrust with PID */
									float height_error = (local_pos.z - (-height_sp));

									/* update height setpoint if needed*/
									if (manual.throttle < 0.4f)
									{
										/* down */
										if (height_sp > params.height_min + params.height_rate &&
												fabsf(height_error) < params.limit_height_error)
											height_sp -= params.height_rate * dt;
									}

									if (manual.throttle > 0.6f)
									{
										/* up */
										if (height_sp < params.height_max &&
												fabsf(height_error) < params.limit_height_error)
											height_sp += params.height_rate * dt;
									}

									/* instead of speed limitation, limit height error (downwards) */
									if(height_error > params.limit_height_error)
										height_error = params.limit_height_error;
									else if(height_error < -params.limit_height_error)
										height_error = -params.limit_height_error;

									integrated_h_error = integrated_h_error + height_error;
									float integrated_thrust_addition = integrated_h_error * params.height_i;

									if(integrated_thrust_addition > params.limit_thrust_int)
										integrated_thrust_addition = params.limit_thrust_int;
									if(integrated_thrust_addition < -params.limit_thrust_int)
										integrated_thrust_addition = -params.limit_thrust_int;

									float height_speed = last_local_pos_z - local_pos.z;
									float thrust_diff = height_error * params.height_p - height_speed * params.height_d;

									thrust_control = params.thrust_feedforward + thrust_diff + integrated_thrust_addition;

									/* add attitude component
									 * F = Fz / (cos(pitch)*cos(roll)) -> can be found in rotM
									 */
//									// TODO problem with attitude
//									if (att.R_valid && att.R[2][2] > 0)
//										thrust_control = thrust_control / att.R[2][2];

									/* set thrust lower limit */
									if(thrust_control < params.limit_thrust_lower)
										thrust_control = params.limit_thrust_lower;
								}
							}

							/* set thrust upper limit */
							if(thrust_control > params.limit_thrust_upper)
								thrust_control = params.limit_thrust_upper;
						}
						/* store actual height for speed estimation */
						last_local_pos_z = local_pos.z;

						speed_sp.thrust_sp =  thrust_control; //manual.throttle;
						speed_sp.timestamp = hrt_absolute_time();

						/* publish new speed setpoint */
						if(isfinite(speed_sp.vx) && isfinite(speed_sp.vy) && isfinite(speed_sp.yaw_sp) && isfinite(speed_sp.thrust_sp))
						{

							if(speed_setpoint_adverted)
							{
								orb_publish(ORB_ID(vehicle_bodyframe_speed_setpoint), speed_sp_pub, &speed_sp);
							}
							else
							{
								speed_sp_pub = orb_advertise(ORB_ID(vehicle_bodyframe_speed_setpoint), &speed_sp);
								speed_setpoint_adverted = true;
							}
						}
						else
						{
							warnx("NaN in flow position controller!");
						}
					}
					else
					{
						/* in manual or stabilized state just reset speed and flow sum setpoint */
						//mavlink_log_info(mavlink_fd,"[fpc] reset speed sp, flow_sp_sumx,y (%f,%f)",filtered_flow.sumx, filtered_flow.sumy);
						if(status_changed == true)
							mavlink_log_info(mavlink_fd,"[fpc] flow POSITION controller disengaged.");

						status_changed = false;
						speed_sp.vx = 0.0f;
						speed_sp.vy = 0.0f;
						flow_sp_sumx = filtered_flow.sumx;
						flow_sp_sumy = filtered_flow.sumy;
						if(isfinite(att.yaw))
						{
							yaw_sp = att.yaw;
							speed_sp.yaw_sp = att.yaw;
						}
						if(isfinite(manual.throttle))
							speed_sp.thrust_sp = manual.throttle;
					}
					/* measure in what intervals the controller runs */
					perf_count(mc_interval_perf);
					perf_end(mc_loop_perf);
				}
			}

			counter++;
		}
		else
		{
			/* sensors not ready waiting for first attitude msg */

			/* polling */
			struct pollfd fds[1] = {
				{ .fd = vehicle_attitude_sub, .events = POLLIN },
			};

			/* wait for a flow msg, check for exit condition every 5 s */
			int ret = poll(fds, 1, 5000);

			if (ret < 0)
			{
				/* poll error, count it in perf */
				perf_count(mc_err_perf);
			}
			else if (ret == 0)
			{
				/* no return value, ignore */
				mavlink_log_info(mavlink_fd,"[fpc] no attitude received.\n");
			}
			else
			{
				if (fds[0].revents & POLLIN)
				{
					sensors_ready = true;
					mavlink_log_info(mavlink_fd,"[fpc] initialized.\n");
				}
			}
		}
	}

	mavlink_log_info(mavlink_fd,"[fpc] ending now...\n");

	thread_running = false;

	close(parameter_update_sub);
	close(vehicle_attitude_sub);
	close(vehicle_local_position_sub);
	close(armed_sub);
	close(control_mode_sub);
	close(manual_control_setpoint_sub);
	close(speed_sp_pub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	return 0;
}

