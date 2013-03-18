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
#include <uORB/topics/sensor_combined.h>
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
	struct sensor_combined_s sensor;
	struct vehicle_attitude_s att;
	struct vehicle_local_position_setpoint_s local_pos_sp;
	struct manual_control_setpoint_s manual;
	struct vehicle_local_position_s local_pos;



	struct vehicle_attitude_setpoint_s att_sp;

	/* subscribe to attitude, motor setpoints and system state */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	int vehicle_local_position_setpoint_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));



	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	/* parameters init*/
	struct multirotor_position_control_flow_params params;
	struct multirotor_position_control_flow_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	/* limits */
	float pitch_limit = 0.33f;
	float roll_limit = 0.33f;
	float thrust_limit_upper = 0.5f;
	float thrust_limit_lower = 0.5f;
	float thrust_limit_integrated = 0.05f;
	float integrated_h_error = 0.0f;
	float last_height = 0.0f;
	float last_thrust = 0.0f;
	float setpoint_x = 0.0f;
	float setpoint_y = 0.0f;
	float setpoint_update_step = 0.005f;
	float initial_yaw = 0.0f;
	float yaw_sum = 0.0f;
	int yaw_init_counter = 0;

	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "multirotor_att_control_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "multirotor_att_control_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "multirotor_att_control_err");

	static bool sensors_ready = false;

	while (!thread_should_exit) {

		/* wait for first attitude msg to be sure all data are available */
		if (sensors_ready) {

			/* polling */
			struct pollfd fds[2] = {
				{ .fd = vehicle_local_position_sub, .events = POLLIN }, // positions from estimator
				{ .fd = parameter_update_sub,   .events = POLLIN },
			};

			/* wait for a position update, check for exit condition every 500 ms */
			int ret = poll(fds, 2, 500);

			if (ret < 0) {
				/* poll error, count it in perf */
				perf_count(mc_err_perf);

			} else if (ret == 0) {
				/* no return value, ignore */
				printf("[multirotor flow position control] no local position updates\n"); // XXX wrong place
			} else {

				if (fds[1].revents & POLLIN){
					/* read from param to clear updated flag */
					struct parameter_update_s update;
					orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

					parameters_update(&param_handles, &params);
					printf("[multirotor flow position control] parameters updated.\n");
				}

	//			if (fds[0].revents & POLLIN){
	//
	//				/* get a local copy of the sensor data */
	//				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);
	//
	//				acc_lp = acc_lp * 0.9f + 0.1f * (sensor.accelerometer_m_s2[2] + 9.81f);
	//				if (acc_counter != sensor.accelerometer_counter)
	//				{
	//					if (last_sensor_timestamp != 0)
	//					{
	//						float acc_time = (float)(sensor.timestamp - last_sensor_timestamp) / 1000000.0f;
	//						acc_speed_z = acc_speed_z + (sensor.accelerometer_m_s2[2] + const_earth_gravity) * acc_time;
	//					}
	//
	//					last_sensor_timestamp = sensor.timestamp;
	//				}
	//
	//				acc_counter = sensor.accelerometer_counter;
	//			}

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

					if (vstatus.state_machine == SYSTEM_STATE_AUTO) {

						if (yaw_init_counter < 10) {
							yaw_sum += att.yaw;
							yaw_init_counter++;
						} else if (yaw_init_counter == 10){
							initial_yaw = yaw_sum / 10.0f;
						}

						/*
						 * attitude setpoint is to be set as "how to change attitude"
						 *
						 *
						 */

						/* update position setpoint? */
						if(manual.pitch < -0.2f) {
							setpoint_x += setpoint_update_step;
						} else if (manual.pitch > 0.2f) {
							setpoint_x -= setpoint_update_step;
						}
						if(manual.roll < -0.2f) {
							setpoint_y -= setpoint_update_step;
						} else if (manual.roll > 0.2f) {
							setpoint_y += setpoint_update_step;
						}

						/* calc new roll/pitch */
						float pitch_body = (local_pos.x - setpoint_x) * params.pos_p + local_pos.vx * params.pos_d;
						float roll_body = - (local_pos.y - setpoint_y) * params.pos_p - local_pos.vy * params.pos_d;

						/* limit roll and pitch corrections */
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

						/* calc new thrust */
						float height_error = (local_pos.z - params.height_sp);
						integrated_h_error = integrated_h_error + height_error;
						float integrated_thrust_addition = integrated_h_error * params.height_i;
						if(integrated_thrust_addition > thrust_limit_integrated){
							integrated_thrust_addition = thrust_limit_integrated;
						}
						if(integrated_thrust_addition < -thrust_limit_integrated){
							integrated_thrust_addition = -thrust_limit_integrated;
						}

						float height_speed = last_height - local_pos.z;
						last_height = local_pos.z;
						float thrust_diff = height_error * params.height_p - height_speed * params.height_d; // just PD controller
						float thrust = thrust_diff + integrated_thrust_addition;

						float height_ctrl_thrust = params.thrust_feedforward + thrust;

						/* the throttle stick on the rc control limits the maximum thrust */
						if(isfinite(manual.throttle))
							thrust_limit_upper = manual.throttle;

						/* never go too low with the thrust that it becomes uncontrollable */
						if(height_ctrl_thrust < thrust_limit_lower){
							height_ctrl_thrust = thrust_limit_lower;
						}

						if (height_ctrl_thrust >= thrust_limit_upper){
							att_sp.thrust = thrust_limit_upper;
						} else {
							att_sp.thrust = height_ctrl_thrust;
						}
						last_thrust = att_sp.thrust;


						/* do not control yaw */
						if(yaw_init_counter < 10)
							att_sp.yaw_body = att.yaw;
						else
							att_sp.yaw_body = initial_yaw;

						att_sp.timestamp = hrt_absolute_time();

						/* publish new attitude setpoint */
						if (loopcounter == 200){
							if(isfinite(att_sp.pitch_body) && isfinite(att_sp.roll_body) && isfinite(att_sp.yaw_body) && isfinite(att_sp.thrust))
							{
	//							printf("pitch:%.3f, roll:%.3f\n", att_sp.pitch_body, att_sp.roll_body);
	//							printf("yaw:%.3f, thrust:%.3f\n", att_sp.yaw_body, att_sp.thrust);
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
					} else
					{
						setpoint_x = 0.0f;
						setpoint_y = 0.0f;
						initial_yaw = 0.0f;
						yaw_init_counter = 0;
					}

					/* measure in what intervals the controller runs */
					perf_count(mc_interval_perf);
					perf_end(mc_loop_perf);
				}

			}

			/* run at approximately 50 Hz */
			//usleep(20000);

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
				printf("[multirotor flow position control] no attitude received.\n");
			} else {

				if (fds[0].revents & POLLIN){
					sensors_ready = true;
					printf("[multirotor flow position control] initialized.\n");
				}
			}
		}

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

