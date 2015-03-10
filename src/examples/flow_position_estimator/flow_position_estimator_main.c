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
 * @file flow_position_estimator_main.c
 *
 * Optical flow position estimator
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <poll.h>
#include <platforms/px4_defines.h>

#include "flow_position_estimator_params.h"

__EXPORT int flow_position_estimator_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

int flow_position_estimator_thread_main(int argc, char *argv[]);
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
 * to task_spawn_cmd().
 */
int flow_position_estimator_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start"))
	{
		if (thread_running)
		{
			printf("flow position estimator already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("flow_position_estimator",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 5,
					 4000,
					 flow_position_estimator_thread_main,
					 (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
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
			printf("\tflow position estimator is running\n");
		else
			printf("\tflow position estimator not started\n");

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int flow_position_estimator_thread_main(int argc, char *argv[])
{
	/* welcome user */
	thread_running = true;
	printf("[flow position estimator] starting\n");

	/* rotation matrix for transformation of optical flow speed vectors */
	static const int8_t rotM_flow_sensor[3][3] =   {{  0, -1, 0 },
													{ 1, 0, 0 },
													{  0, 0, 1 }}; // 90deg rotated
	const float time_scale = powf(10.0f,-6.0f);
	static float speed[3] = {0.0f, 0.0f, 0.0f};
	static float flow_speed[3] = {0.0f, 0.0f, 0.0f};
	static float global_speed[3] = {0.0f, 0.0f, 0.0f};
	static uint32_t counter = 0;
	static uint64_t time_last_flow = 0; // in ms
	static float dt = 0.0f; // seconds
	static float sonar_last = 0.0f;
	static bool sonar_valid = false;
	static float sonar_lp = 0.0f;

	/* subscribe to vehicle status, attitude, sensors and flow*/
	struct actuator_armed_s armed;
	memset(&armed, 0, sizeof(armed));
	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct optical_flow_s flow;
	memset(&flow, 0, sizeof(flow));

	/* subscribe to parameter changes */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	/* subscribe to armed topic */
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	/* subscribe to safety topic */
	int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));

	/* subscribe to attitude */
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	/* subscribe to attitude setpoint */
	int vehicle_attitude_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));

	/* subscribe to optical flow*/
	int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));

	/* init local position and filtered flow struct */
	struct vehicle_local_position_s local_pos = {
			.x = 0.0f,
			.y = 0.0f,
			.z = 0.0f,
			.vx = 0.0f,
			.vy = 0.0f,
			.vz = 0.0f
	};
	struct filtered_bottom_flow_s filtered_flow = {
			.sumx = 0.0f,
			.sumy = 0.0f,
			.vx = 0.0f,
			.vy = 0.0f
	};

	/* advert pub messages */
	orb_advert_t local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos);
	orb_advert_t filtered_flow_pub = orb_advertise(ORB_ID(filtered_bottom_flow), &filtered_flow);

	/* vehicle flying status parameters */
	bool vehicle_liftoff = false;
	bool sensors_ready = false;

	/* parameters init*/
	struct flow_position_estimator_params params;
	struct flow_position_estimator_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "flow_position_estimator_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "flow_position_estimator_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "flow_position_estimator_err");

	while (!thread_should_exit)
	{

		if (sensors_ready)
		{
			/*This runs at the rate of the sensors */
			struct pollfd fds[2] = {
					{ .fd = optical_flow_sub, .events = POLLIN },
					{ .fd = parameter_update_sub,   .events = POLLIN }
			};

			/* wait for a sensor update, check for exit condition every 500 ms */
			int ret = poll(fds, 2, 500);

			if (ret < 0)
			{
				/* poll error, count it in perf */
				perf_count(mc_err_perf);

			}
			else if (ret == 0)
			{
				/* no return value, ignore */
//				printf("[flow position estimator] no bottom flow.\n");
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
					printf("[flow position estimator] parameters updated.\n");
				}

				/* only if flow data changed */
				if (fds[0].revents & POLLIN)
				{
					perf_begin(mc_loop_perf);

					orb_copy(ORB_ID(optical_flow), optical_flow_sub, &flow);
					/* got flow, updating attitude and status as well */
					orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
					orb_copy(ORB_ID(vehicle_attitude_setpoint), vehicle_attitude_setpoint_sub, &att_sp);
					orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
					orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);

					/* vehicle state estimation */
					float sonar_new = flow.ground_distance_m;

					/* set liftoff boolean
					 * -> at bottom sonar sometimes does not work correctly, and has to be calibrated (distance higher than 0.3m)
					 * -> accept sonar measurements after reaching calibration distance (values between 0.3m and 1.0m for some time)
					 * -> minimum sonar value 0.3m
					 */

					if (!vehicle_liftoff)
					{
						if (armed.armed && att_sp.thrust > params.minimum_liftoff_thrust && sonar_new > 0.3f && sonar_new < 1.0f)
							vehicle_liftoff = true;
					}
					else
					{
						if (!armed.armed || (att_sp.thrust < params.minimum_liftoff_thrust && sonar_new <= 0.3f))
							vehicle_liftoff = false;
					}

					/* calc dt between flow timestamps */
					/* ignore first flow msg */
					if(time_last_flow == 0)
					{
						time_last_flow = flow.timestamp;
						continue;
					}
					dt = (float)(flow.timestamp - time_last_flow) * time_scale ;
					time_last_flow = flow.timestamp;

					/* only make position update if vehicle is lift off or DEBUG is activated*/
					if (vehicle_liftoff || params.debug)
					{
						/* copy flow */
						if (flow.integration_timespan > 0) {
							flow_speed[0] = flow.pixel_flow_x_integral / (flow.integration_timespan / 1e6f) * flow.ground_distance_m;
							flow_speed[1] = flow.pixel_flow_y_integral / (flow.integration_timespan / 1e6f) * flow.ground_distance_m;
						} else {
							flow_speed[0] = 0;
							flow_speed[1] = 0;
						}
						flow_speed[2] = 0.0f;

						/* convert to bodyframe velocity */
						for(uint8_t i = 0; i < 3; i++)
						{
							float sum = 0.0f;
							for(uint8_t j = 0; j < 3; j++)
								sum = sum + flow_speed[j] * rotM_flow_sensor[j][i];

							speed[i] = sum;
						}

						/* update filtered flow */
						filtered_flow.sumx = filtered_flow.sumx + speed[0] * dt;
						filtered_flow.sumy = filtered_flow.sumy + speed[1] * dt;
						filtered_flow.vx = speed[0];
						filtered_flow.vy = speed[1];

						// TODO add yaw rotation correction (with distance to vehicle zero)

						/* convert to globalframe velocity
						 * -> local position is currently not used for position control
						 */
						for(uint8_t i = 0; i < 3; i++)
						{
							float sum = 0.0f;
							for(uint8_t j = 0; j < 3; j++)
								sum = sum + speed[j] * PX4_R(att.R, i, j);

							global_speed[i] = sum;
						}

						local_pos.x = local_pos.x + global_speed[0] * dt;
						local_pos.y = local_pos.y + global_speed[1] * dt;
						local_pos.vx = global_speed[0];
						local_pos.vy = global_speed[1];
						local_pos.xy_valid = true;
						local_pos.v_xy_valid = true;
					}
					else
					{
						/* set speed to zero and let position as it is */
						filtered_flow.vx = 0;
						filtered_flow.vy = 0;
						local_pos.vx = 0;
						local_pos.vy = 0;
						local_pos.xy_valid = false;
						local_pos.v_xy_valid = false;
					}

					/* filtering ground distance */
					if (!vehicle_liftoff || !armed.armed)
					{
						/* not possible to fly */
						sonar_valid = false;
						local_pos.z = 0.0f;
						local_pos.z_valid = false;
					}
					else
					{
						sonar_valid = true;
					}

					if (sonar_valid || params.debug)
					{
						/* simple lowpass sonar filtering */
						/* if new value or with sonar update frequency */
						if (sonar_new != sonar_last || counter % 10 == 0)
						{
							sonar_lp = 0.05f * sonar_new + 0.95f * sonar_lp;
							sonar_last = sonar_new;
						}

						float height_diff = sonar_new - sonar_lp;

						/* if over 1/2m spike follow lowpass */
						if (height_diff < -params.sonar_lower_lp_threshold || height_diff > params.sonar_upper_lp_threshold)
						{
							local_pos.z = -sonar_lp;
						}
						else
						{
							local_pos.z = -sonar_new;
						}

						local_pos.z_valid = true;
					}

					filtered_flow.timestamp = hrt_absolute_time();
					local_pos.timestamp = hrt_absolute_time();

					/* publish local position */
					if(isfinite(local_pos.x) && isfinite(local_pos.y) && isfinite(local_pos.z)
							&& isfinite(local_pos.vx) && isfinite(local_pos.vy))
					{
						orb_publish(ORB_ID(vehicle_local_position), local_pos_pub, &local_pos);
					}

					/* publish filtered flow */
					if(isfinite(filtered_flow.sumx) && isfinite(filtered_flow.sumy) && isfinite(filtered_flow.vx) && isfinite(filtered_flow.vy))
					{
						orb_publish(ORB_ID(filtered_bottom_flow), filtered_flow_pub, &filtered_flow);
					}

					/* measure in what intervals the position estimator runs */
					perf_count(mc_interval_perf);
					perf_end(mc_loop_perf);

				}
			}

		}
		else
		{
			/* sensors not ready waiting for first attitude msg */

			/* polling */
			struct pollfd fds[1] = {
				{ .fd = vehicle_attitude_sub, .events = POLLIN },
			};

			/* wait for a attitude message, check for exit condition every 5 s */
			int ret = poll(fds, 1, 5000);

			if (ret < 0)
			{
				/* poll error, count it in perf */
				perf_count(mc_err_perf);
			}
			else if (ret == 0)
			{
				/* no return value, ignore */
				printf("[flow position estimator] no attitude received.\n");
			}
			else
			{
				if (fds[0].revents & POLLIN){
					sensors_ready = true;
					printf("[flow position estimator] initialized.\n");
				}
			}
		}

		counter++;
	}

	printf("[flow position estimator] exiting.\n");
	thread_running = false;

	close(vehicle_attitude_setpoint_sub);
	close(vehicle_attitude_sub);
	close(armed_sub);
	close(control_mode_sub);
	close(parameter_update_sub);
	close(optical_flow_sub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	return 0;
}


