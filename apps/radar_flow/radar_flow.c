/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file radar_flow.c
 * Handling omnidirectional flow msg and provide radar information
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <poll.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_bodyframe_position.h>
#include <uORB/topics/omnidirectional_flow.h>
#include <uORB/topics/discrete_radar.h>

#include "radar_flow_params.h"
#include "radar_sounds.h"
#include "codegen/frontFlowKalmanFilter.h"
#include "codegen/wallEstimationFilter.h"

static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/**
 * Daemon management function.
 */
__EXPORT int radar_flow_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int radar_flow_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
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
int radar_flow_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("radar already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn("radar_flow",
					 SCHED_RR,
					 SCHED_PRIORITY_MAX - 50,
					 8162,
					 radar_flow_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tradar is running\n");
		} else {
			printf("\tradar not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}


int radar_flow_thread_main(int argc, char *argv[]) {

	printf("[radar] starting\n");
	thread_running = true;

	uint32_t counter = 0;

	/* structures */
	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct omnidirectional_flow_s omni_flow;
	memset(&omni_flow, 0, sizeof(omni_flow));
	struct vehicle_bodyframe_position_s bodyframe_pos;
	memset(&bodyframe_pos, 0, sizeof(bodyframe_pos));

	/* publishing parameters */
	struct discrete_radar_s discrete_radar;
	memset(&discrete_radar, 0, sizeof(discrete_radar));

	/* subscribe to attitude, motor setpoints and system state */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int omnidirectional_flow_sub = orb_subscribe(ORB_ID(omnidirectional_flow));
	int vehicle_bodyframe_position_sub = orb_subscribe(ORB_ID(vehicle_bodyframe_position));

	orb_advert_t discrete_radar_pub = orb_advertise(ORB_ID(discrete_radar), &discrete_radar);

	/* parameters init*/
	struct radar_flow_params params;
	struct radar_flow_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "radar_flow_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "radar_flow_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "radar_flow_err");

	radar_sounds_init();

	printf("[radar] initialized\n");

	int tone_frequence = 100;
	int tone_counter = 0;
	uint64_t time_last_flow = 0;

	/* flow filter parameters */
	static float flow_aposteriori_k[40] = { 0.0f };
	static float speed_aposteriori_k[4] = { 0.0f };
	static float flow_aposteriori[40] = { 0.0f };
	static float speed_aposteriori[4] = { 0.0f };
	static float omni_left_filtered[10] = { 0.0f };
	static float omni_right_filtered[10] = { 0.0f };
	static uint8_t omni_left_invalid_counter[5] = { 0 };
	static uint8_t omni_right_invalid_counter[5] = { 0 };
	static float speed[2] = { 0.0f };
	static float speed_filtered[2] = { 0.0f };
	static float front_distance_filtered = 5.0f;

	/* wall estimation parameters */
	static float filter_settings[8] = { 0.0f };
	static float position_last[2] = { 0.0f };
	static float position_update[2] = { 0.0f };
	static float yaw_last = 0.0f;
	static float yaw_update = 0.0f;
	static float radar[32] = { 0.0f };
	static float radar_filtered[32] = { 0.0f };
	static float radar_weights[32] = { 0.0f };
	static float radar_filtered_k[32] = { 0.0f };
	static float radar_weights_k[32] = { 0.0f };
	static float distance_left = 5.0f;
	static float distance_right = 5.0f;
	static float distance_front = 5.0f;
	static int sonar_counter = 0;
	static int sonar_gradient = 0;

	static bool sensors_ready = false;
	static bool update_initialized = false;

	filter_settings[0] = params.s0;
	filter_settings[1] = params.s1;
	filter_settings[2] = params.s2;
	filter_settings[3] = params.s3;
	filter_settings[4] = params.s4;
	filter_settings[5] = params.s5;
	filter_settings[6] = params.s6;
	filter_settings[7] = params.s7;

	while (!thread_should_exit) {

		/* wait for first attitude msg to be sure all data are available */
		if (sensors_ready) {

			/* polling */
			struct pollfd fds[2] = {
				{ .fd = omnidirectional_flow_sub, .events = POLLIN },
				{ .fd = parameter_update_sub,   .events = POLLIN },
			};

			/* wait for a flow msg, check for exit condition every 500 ms */
			int ret = poll(fds, 2, 500);

			if (ret < 0) {
				/* poll error, count it in perf */
				perf_count(mc_err_perf);

			} else if (ret == 0) {
				/* no return value, ignore */
				printf("[radar] no omnidirectional flow msgs.\n");
			} else {

				if (fds[1].revents & POLLIN){
					/* NEW PARAMETERS ----------------------------------------------------------*/

					/* read from param to clear updated flag */
					struct parameter_update_s update;
					orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

					parameters_update(&param_handles, &params);

					filter_settings[0] = params.s0;
					filter_settings[1] = params.s1;
					filter_settings[2] = params.s2;
					filter_settings[3] = params.s3;
					filter_settings[4] = params.s4;
					filter_settings[5] = params.s5;
					filter_settings[6] = params.s6;
					filter_settings[7] = params.s7;

					printf("[radar] parameters updated.\n");
				}

				if (fds[0].revents & POLLIN) {
					/* NEW OMNIDIRECTIONAL FLOW VALUES-------------------------------------------*/

					perf_begin(mc_loop_perf);

					/* get a local copy of the vehicle state */
					orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);
					/* get a local copy of attitude */
					orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
					/* get a local copy of bodyframe position */
					orb_copy(ORB_ID(vehicle_bodyframe_position), vehicle_bodyframe_position_sub, &bodyframe_pos);
					/* get a local copy of omnidirectional flow */
					orb_copy(ORB_ID(omnidirectional_flow), omnidirectional_flow_sub, &omni_flow);

					if (time_last_flow == 0){
						time_last_flow = omni_flow.timestamp;
						continue;
					}

					/* count invalid flow values */
					for (int i = 0; i<5; i++){
						if (omni_flow.left[i+5] == 0) {
							omni_left_invalid_counter[i]++;
						} else {
							omni_left_invalid_counter[i] = 0;
						}
						if (omni_flow.right[i] == 0) {
							omni_right_invalid_counter[i]++;
						} else {
							omni_right_invalid_counter[i] = 0;
						}

						/* set flow to zero if no new value arrived */
						if (omni_left_invalid_counter[i] > 3) {
							omni_flow.left[i+5] = 1;
							omni_flow.left[i] = 0;
						}
						if (omni_right_invalid_counter[i] > 3) {
							omni_flow.right[i] = 1;
							omni_flow.right[i+5] = 0;
						}
					}

					/* filtering data */
					front_distance_filtered = (1 - params.front_lp_alpha) * front_distance_filtered + params.front_lp_alpha * omni_flow.front_distance_m;
					float dt = ((float)(omni_flow.timestamp - time_last_flow)) / 1000000.0f; // seconds
					time_last_flow = omni_flow.timestamp;
					speed[0] = bodyframe_pos.vx;
					speed[1] = bodyframe_pos.vy;
					frontFlowKalmanFilter(dt, params.kalman_k1, params.kalman_k2, flow_aposteriori_k, speed_aposteriori_k, omni_flow.left, omni_flow.right, speed, 1, flow_aposteriori, speed_aposteriori);
					memcpy(flow_aposteriori_k, flow_aposteriori, sizeof(flow_aposteriori));
					memcpy(speed_aposteriori_k, speed_aposteriori, sizeof(speed_aposteriori));

					/* sensor is 180 degree inverted*/
					for (int i = 0; i<10; i++){
						omni_left_filtered[i] = -flow_aposteriori[20 + 18 - i*2];
						omni_right_filtered[i] = -flow_aposteriori[18 - i*2];
					}

					/* send filtered flow values ------------------------------------------------------------------ */
					//debug
					if (params.debug) {
						discrete_radar.distances[4] = omni_left_filtered[0];
						discrete_radar.distances[5] = omni_left_filtered[1];
						discrete_radar.distances[6] = omni_left_filtered[2];
						discrete_radar.distances[7] = omni_left_filtered[3];
						discrete_radar.distances[8] = omni_left_filtered[4];
						discrete_radar.distances[9] = omni_left_filtered[5];
						discrete_radar.distances[10] = omni_left_filtered[6];
						discrete_radar.distances[11] = omni_left_filtered[7];
						discrete_radar.distances[12] = omni_left_filtered[8];
						discrete_radar.distances[13] = omni_left_filtered[9];

						discrete_radar.distances[19] = omni_right_filtered[0];
						discrete_radar.distances[20] = omni_right_filtered[1];
						discrete_radar.distances[21] = omni_right_filtered[2];
						discrete_radar.distances[22] = omni_right_filtered[3];
						discrete_radar.distances[23] = omni_right_filtered[4];
						discrete_radar.distances[24] = omni_right_filtered[5];
						discrete_radar.distances[25] = omni_right_filtered[6];
						discrete_radar.distances[26] = omni_right_filtered[7];
						discrete_radar.distances[27] = omni_right_filtered[8];
						discrete_radar.distances[28] = omni_right_filtered[9];
						discrete_radar.timestamp = hrt_absolute_time();
						orb_publish(ORB_ID(discrete_radar), discrete_radar_pub, &discrete_radar);
						continue;
					}

					/* send radar values ------------------------------------------------------------------ */

					speed_filtered[0] = speed_aposteriori[0];
					speed_filtered[1] = speed_aposteriori[2];

					if (update_initialized) {
						position_update[0] = bodyframe_pos.x - position_last[0];
						position_update[1] = bodyframe_pos.y - position_last[1];
						yaw_update = att.yaw - yaw_last;
					} else {
						/* at first round */
						update_initialized = true;
					}

					position_last[0] = bodyframe_pos.x;
					position_last[1] = bodyframe_pos.y;
					yaw_last = att.yaw;

					wallEstimationFilter(radar_filtered_k, radar_weights_k, omni_left_filtered, omni_right_filtered, front_distance_filtered, 1, speed_filtered, position_update, yaw_update, filter_settings, radar, radar_filtered, radar_weights);
					memcpy(radar_filtered_k, radar_filtered, sizeof(radar_filtered));
					memcpy(radar_weights_k, radar_weights, sizeof(radar_weights));

					for (int i = 0; i<32; i++) {
						discrete_radar.distances[i] = (int16_t)(radar_filtered[i] * 1000);
//						discrete_radar.distances[i] = (int16_t)(radar_weights[i] * 1000);
					}
					distance_left = radar_filtered[8];
					distance_right = radar_filtered[24];
					distance_front = front_distance_filtered;


					int gradient_left = 0;
					int gradient_right = 0;
					int gradient_front = 0;

					if (distance_left < 10.0f || distance_left > 0.0f) {

						if (distance_left < 2.0f) {
							gradient_left = (int)(100.0f - (100.0f * distance_left) / 2.0f);
						}
					}

					if (distance_right < 10.0f || distance_right > 0.0f) {

						if (distance_right < 2.0f) {
							gradient_right = (int)(100.0f - (100.0f * distance_right) / 2.0f);
						}
					}

					if (distance_front < 10.0f || distance_front > 0.0f) {

						if (distance_front < 2.0f) {
							gradient_front = (int)(100.0f - (100.0f * distance_left) / 2.0f);
						}
					}

					if (gradient_front > gradient_left && gradient_front > gradient_right) {
						sonar_gradient = gradient_front;
					} else {
						if (gradient_left > gradient_right) {
							sonar_gradient = gradient_left;
						} else {
							sonar_gradient = gradient_right;
						}
					}

					sonar_counter++;
					if (sonar_counter > (220 - 2 * sonar_gradient)){
						if (params.beep_front_sonar)
						{
							tune_sonar();
						}
						sonar_counter = 0;
					}

					discrete_radar.timestamp = hrt_absolute_time();
					orb_publish(ORB_ID(discrete_radar), discrete_radar_pub, &discrete_radar);

//					if(params.pos_sp_x)
//					{
//						if (!played)
//						{
//							tune_tetris();
//							played = true;
//							printf("tetris tuned...\n");
//						}
//					} else {
//						played = false;
//					}

					/* reset parameters */
					position_update[0] = 0.0f;
					position_update[1] = 0.0f;
					yaw_update = 0.0f;

					/* measure in what intervals the controller runs */
					perf_count(mc_interval_perf);
					perf_end(mc_loop_perf);
				}


			}

			tone_frequence =  (int)(-bodyframe_pos.z * 100);
			if(tone_counter > tone_frequence)
			{
				if (params.beep_bottom_sonar)
				{
					tune_sonar();
				}
				tone_counter = 0;
			}

			tone_counter++;
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
				printf("[radar] no attitude received.\n");
			} else {

				if (fds[0].revents & POLLIN){
					sensors_ready = true;
					tune_ready();
				}
			}
		}
	}

	printf("[radar] ending now...\n");

	thread_running = false;

	close(parameter_update_sub);
	close(vehicle_attitude_sub);
	close(vehicle_bodyframe_position_sub);
	close(vehicle_status_sub);
	close(omnidirectional_flow_sub);

	radar_sounds_deinit();

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	return 0;
}
