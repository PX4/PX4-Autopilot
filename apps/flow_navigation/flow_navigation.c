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

#include "flow_navigation_params.h"

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

	/* subscribe to attitude, motor setpoints and system state */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));
	int omnidirectional_flow_sub = orb_subscribe(ORB_ID(omnidirectional_flow));
	int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	/* polling */
	struct pollfd fds[2] = {
				{ .fd = omnidirectional_flow_sub, .events = POLLIN },
				{ .fd = parameter_update_sub,   .events = POLLIN },
			};

	orb_advert_t vehicle_local_position_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos);

	/* parameters init*/
	struct flow_navigation_params params;
	struct flow_navigation_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	/* limits */

	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "multirotor_att_control_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "multirotor_att_control_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "multirotor_att_control_err");

	printf("[multirotor flow position control] initialized\n");

	while (!thread_should_exit) {

		/* wait for a flow msg, check for exit condition every 500 ms */
		int ret = poll(fds, 2, 500);

		if (ret < 0) {
			/* poll error, count it in perf */
			perf_count(mc_err_perf);

		} else if (ret == 0) {
			/* no return value, ignore */
			printf("[multirotor flow position estimator] no omnidirectional flow msgs.\n");
		} else {

			if (fds[1].revents & POLLIN){
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

				parameters_update(&param_handles, &params);
				printf("[multirotor flow position control] parameters updated.\n");
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

				/* measure in what intervals the controller runs */
				perf_count(mc_interval_perf);
				perf_end(mc_loop_perf);
			}
		}

		/* run at approximately 50 Hz */
		//usleep(20000);

		counter++;

	}

	printf("[multirotor flow position control] ending now...\n");

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
