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
 * @file flow_speed_control.c
 *
 * Optical flow speed controller
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
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_bodyframe_speed_setpoint.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <poll.h>
#include <mavlink/mavlink_log.h>

#include "flow_speed_control_params.h"


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

__EXPORT int flow_speed_control_main(int argc, char *argv[]);

/**
 * Mainloop of position controller.
 */
static int flow_speed_control_thread_main(int argc, char *argv[]);

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
int flow_speed_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start"))
	{
		if (thread_running)
		{
			printf("flow speed control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd("flow_speed_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 6,
					 4096,
					 flow_speed_control_thread_main,
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
			printf("\tflow speed control app is running\n");
		else
			printf("\tflow speed control app not started\n");

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static int
flow_speed_control_thread_main(int argc, char *argv[])
{
	/* welcome user */
	thread_running = true;
	static int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd,"[fsc] started");

	uint32_t counter = 0;

	/* structures */
	struct actuator_armed_s armed;
	memset(&armed, 0, sizeof(armed));
	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));
	struct filtered_bottom_flow_s filtered_flow;
	memset(&filtered_flow, 0, sizeof(filtered_flow));
	struct vehicle_bodyframe_speed_setpoint_s speed_sp;
	memset(&speed_sp, 0, sizeof(speed_sp));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));

	/* subscribe to attitude, motor setpoints and system state */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	int filtered_bottom_flow_sub = orb_subscribe(ORB_ID(filtered_bottom_flow));
	int vehicle_bodyframe_speed_setpoint_sub = orb_subscribe(ORB_ID(vehicle_bodyframe_speed_setpoint));

	orb_advert_t att_sp_pub;
	bool attitude_setpoint_adverted = false;

	/* parameters init*/
	struct flow_speed_control_params params;
	struct flow_speed_control_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "flow_speed_control_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "flow_speed_control_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "flow_speed_control_err");

	static bool sensors_ready = false;
	static bool status_changed = false;

	while (!thread_should_exit)
	{
		/* wait for first attitude msg to be sure all data are available */
		if (sensors_ready)
		{
			/* polling */
			struct pollfd fds[2] = {
				{ .fd = vehicle_bodyframe_speed_setpoint_sub, .events = POLLIN }, // speed setpoint from pos controller
				{ .fd = parameter_update_sub,   .events = POLLIN }
			};

			/* wait for a position update, check for exit condition every 5000 ms */
			int ret = poll(fds, 2, 500);

			if (ret < 0)
			{
				/* poll error, count it in perf */
				perf_count(mc_err_perf);
			}
			else if (ret == 0)
			{
				/* no return value, ignore */
//				printf("[flow speed control] no bodyframe speed setpoints updates\n");
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
					mavlink_log_info(mavlink_fd,"[fsp] parameters updated.");
				}

				/* only run controller if position/speed changed */
				if (fds[0].revents & POLLIN)
				{
					perf_begin(mc_loop_perf);

					/* get a local copy of the armed topic */
					orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
					/* get a local copy of the control mode */
					orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);
					/* get a local copy of filtered bottom flow */
					orb_copy(ORB_ID(filtered_bottom_flow), filtered_bottom_flow_sub, &filtered_flow);
					/* get a local copy of bodyframe speed setpoint */
					orb_copy(ORB_ID(vehicle_bodyframe_speed_setpoint), vehicle_bodyframe_speed_setpoint_sub, &speed_sp);
					/* get a local copy of control mode */
					orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);

					if (control_mode.flag_control_velocity_enabled)
					{
						/* calc new roll/pitch */
						float pitch_body = -(speed_sp.vx - filtered_flow.vx) * params.speed_p;
						float roll_body  =  (speed_sp.vy - filtered_flow.vy) * params.speed_p;

						if(status_changed == false)
							mavlink_log_info(mavlink_fd,"[fsc] flow SPEED control engaged");

						status_changed = true;

						/* limit roll and pitch corrections */
						if((pitch_body <= params.limit_pitch) && (pitch_body >= -params.limit_pitch))
						{
							att_sp.pitch_body = pitch_body;
						}
						else
						{
							if(pitch_body > params.limit_pitch)
								att_sp.pitch_body = params.limit_pitch;
							if(pitch_body < -params.limit_pitch)
								att_sp.pitch_body = -params.limit_pitch;
						}

						if((roll_body <= params.limit_roll) && (roll_body >= -params.limit_roll))
						{
							att_sp.roll_body = roll_body;
						}
						else
						{
							if(roll_body > params.limit_roll)
								att_sp.roll_body = params.limit_roll;
							if(roll_body < -params.limit_roll)
								att_sp.roll_body = -params.limit_roll;
						}

						/* set yaw setpoint forward*/
						att_sp.yaw_body = speed_sp.yaw_sp;

						/* add trim from parameters */
						att_sp.roll_body = att_sp.roll_body + params.trim_roll;
						att_sp.pitch_body = att_sp.pitch_body + params.trim_pitch;

						att_sp.thrust = speed_sp.thrust_sp;
						att_sp.timestamp = hrt_absolute_time();

						/* publish new attitude setpoint */
						if(isfinite(att_sp.pitch_body) && isfinite(att_sp.roll_body) && isfinite(att_sp.yaw_body) && isfinite(att_sp.thrust))
						{
							if (attitude_setpoint_adverted)
							{
								orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
							}
							else
							{
								att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
								attitude_setpoint_adverted = true;
							}
						}
						else
						{
							warnx("NaN in flow speed controller!");
						}
					}
					else
					{
						if(status_changed == true)
							mavlink_log_info(mavlink_fd,"[fsc] flow SPEED controller disengaged.");

						status_changed = false;

						/* reset attitude setpoint */
						att_sp.roll_body = 0.0f;
						att_sp.pitch_body = 0.0f;
						att_sp.thrust = 0.0f;
						att_sp.yaw_body = 0.0f;
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
				mavlink_log_info(mavlink_fd,"[fsc] no attitude received.");
			}
			else
			{
				if (fds[0].revents & POLLIN)
				{
					sensors_ready = true;
					mavlink_log_info(mavlink_fd,"[fsp] initialized.");
				}
			}
		}
	}

	mavlink_log_info(mavlink_fd,"[fsc] ending now...");

	thread_running = false;

	close(parameter_update_sub);
	close(vehicle_attitude_sub);
	close(vehicle_bodyframe_speed_setpoint_sub);
	close(filtered_bottom_flow_sub);
	close(armed_sub);
	close(control_mode_sub);
	close(att_sp_pub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	return 0;
}
