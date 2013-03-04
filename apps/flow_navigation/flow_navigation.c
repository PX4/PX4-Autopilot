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
 * @file flow_navigation.c
 * Handling flow msg and provide navigation commands
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <poll.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/omnidirectional_flow.h>
#include <uORB/topics/wall_estimation.h>

#include "flow_navigation_params.h"
#include "sounds.h"
#include "codegen/flowNavigation.h";
#include "codegen/frontFlowKalmanFilter.h"
#include "codegen/wallEstimator.h"

static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/**
 * Daemon management function.
 */
__EXPORT int flow_navigation_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int flow_navigation_thread_main(int argc, char *argv[]);

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
int flow_navigation_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("flow_navigation already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn("flow_navigation",
					 SCHED_RR,
					 SCHED_PRIORITY_DEFAULT,
					 4096,
					 flow_navigation_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tflow_navigation is running\n");
		} else {
			printf("\tflow_navigation not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int flow_navigation_thread_main(int argc, char *argv[]) {

	printf("[flow_navigation] starting\n");
	thread_running = true;

	uint32_t counter = 0;

	/* structures */
	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct optical_flow_s optical_flow;
	memset(&optical_flow, 0, sizeof(optical_flow));
	struct omnidirectional_flow_s omni_flow;
	memset(&omni_flow, 0, sizeof(omni_flow));
	struct vehicle_local_position_s local_pos;
	memset(&local_pos, 0, sizeof(local_pos));

	struct vehicle_local_position_setpoint_s local_pos_sp;
	memset(&local_pos_sp, 0, sizeof(local_pos_sp));
	struct wall_estimation_s wall_estimation;
	memset(&wall_estimation, 0, sizeof(wall_estimation));

	/* subscribe to attitude, motor setpoints and system state */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));
	int omnidirectional_flow_sub = orb_subscribe(ORB_ID(omnidirectional_flow));
	int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	orb_advert_t vehicle_local_position_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &local_pos_sp);
	orb_advert_t wall_estimation_pub = orb_advertise(ORB_ID(wall_estimation), &wall_estimation);

	/* parameters init*/
	struct flow_navigation_params params;
	struct flow_navigation_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	/* limits */

	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "flow_navigation_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "flow_navigation_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "flow_navigation_err");

	sounds_init();

	printf("[flow_navigation] initialized\n");

	int tone_frequence = 100;
	int tone_counter = 0;
	uint64_t time_last_flow = 0;
	static float flow_aposteriori_k[40] = { 0.0f };
	static float speed_aposteriori_k[4] = { 0.0f };
	static float flow_aposteriori[40] = { 0.0f };
	static float speed_aposteriori[4] = { 0.0f };
	static float omni_left_filtered[10] = { 0.0f };
	static float omni_right_filtered[10] = { 0.0f };
	static float speed[2] = { 0.0f };
	static float speed_filtered[2] = { 0.0f };
	static float thresholds[3] = { 0.05f, 0.05f, 2.0f };
	static float distance_left = 0.0f;
	static float distance_right = 0.0f;
	int sonar_counter = 0;
	int sonar_gradient = 0;

	while (!thread_should_exit) {

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
			printf("[flow_navigation] no omnidirectional flow msgs.\n");
		} else {

			if (fds[1].revents & POLLIN){
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

				parameters_update(&param_handles, &params);
				printf("[flow_navigation] parameters updated.\n");
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
				/* get a local copy of optical flow */
				orb_copy(ORB_ID(optical_flow), optical_flow_sub, &optical_flow);
				/* get a local copy of omnidirectional flow */
				orb_copy(ORB_ID(omnidirectional_flow), omnidirectional_flow_sub, &omni_flow);

				if (vstatus.state_machine == SYSTEM_STATE_AUTO) {

				} else
				{


				}

				if (time_last_flow == 0){
					time_last_flow = omni_flow.timestamp;
					continue;
				}

				float dt = ((float)(omni_flow.timestamp - time_last_flow)) / 1000000.0f; // seconds
				time_last_flow = omni_flow.timestamp;
				speed[0] = - optical_flow.flow_comp_y_m; // XXX change with rot matrix...
				speed[1] = optical_flow.flow_comp_x_m;
				frontFlowKalmanFilter(dt, params.kalman_k1, params.kalman_k2, flow_aposteriori_k, speed_aposteriori_k, omni_flow.left, omni_flow.right, speed, 1, flow_aposteriori, speed_aposteriori);
				memcpy(flow_aposteriori_k, flow_aposteriori, sizeof(flow_aposteriori));
				memcpy(speed_aposteriori_k, speed_aposteriori, sizeof(speed_aposteriori));

				/* sensor is 180 degree inverted*/
				for (int i = 0; i<10; i++){
					omni_left_filtered[i] = -flow_aposteriori[20 + 18 - i*2];
					omni_right_filtered[i] = -flow_aposteriori[18 - i*2];
				}
				speed_filtered[0] = speed_aposteriori[0];
				speed_filtered[1] = speed_aposteriori[2];

				/* debug */
				local_pos_sp.x = speed_filtered[0];
				local_pos_sp.y = speed_filtered[1];

//				wallEstimator(omni_left_filtered, omni_right_filtered, 1.0f, 1, 0.5, 0, &distance_left, &distance_right);
				wallEstimator(omni_left_filtered, omni_right_filtered, 1.0f, 1, speed_filtered, thresholds, &distance_left, &distance_right);

				int gradient_left = 0;
				int gradient_right = 0;

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

				if (gradient_left > gradient_right) {
					sonar_gradient = gradient_left;
				} else {
					sonar_gradient = gradient_right;
				}

				sonar_counter++;
				if (sonar_counter > (220 - 2 * sonar_gradient)){
					if (params.beep_front_sonar)
					{
						tune_sonar();
					}
					sonar_counter = 0;
				}

				/* debug */
				local_pos_sp.z = distance_left;
				local_pos_sp.yaw = distance_right;

				wall_estimation.left[0] = distance_left;
				wall_estimation.right[0] = distance_right;
				orb_publish(ORB_ID(vehicle_local_position_setpoint), vehicle_local_position_sp_pub, &local_pos_sp);
				orb_publish(ORB_ID(wall_estimation), wall_estimation_pub, &wall_estimation);

//				if(params.pos_sp_x)
//				{
//					if (!played)
//					{
//						tune_tetris();
//						played = true;
//						printf("tetris tuned...\n");
//					}
//				} else {
//					played = false;
//				}

				/* measure in what intervals the controller runs */
				perf_count(mc_interval_perf);
				perf_end(mc_loop_perf);
			}


		}



		tone_frequence =  (int)(-local_pos.z * 100);
		if(tone_counter > tone_frequence)
		{
			if (params.beep_bottom_sonar)
			{
				tune_sonar();
			}
			tone_counter = 0;
		}

		/* run at approximately 50 Hz */
		//usleep(20000);

		tone_counter++;
		counter++;

	}

	printf("[flow_navigation] ending now...\n");

	thread_running = false;

	close(parameter_update_sub);
	close(vehicle_attitude_sub);
	close(vehicle_local_position_sub);
	close(vehicle_status_sub);
	close(manual_control_setpoint_sub);
	close(optical_flow_sub);
	close(omnidirectional_flow_sub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	return 0;
}
