/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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

/*
 * @file multirotor_att_control_main.c
 *
 * Implementation of multirotor attitude control main loop.
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
#include <getopt.h>
#include <time.h>
#include <math.h>
#include <poll.h>
#include <sys/prctl.h>
#include <arch/board/up_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/ardrone_control.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/ardrone_motors_setpoint.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_controls.h>

#include <systemlib/perf_counter.h>

#include "multirotor_attitude_control.h"

__EXPORT int multirotor_att_control_main(int argc, char *argv[]);


static enum {
	CONTROL_MODE_RATES = 0,
	CONTROL_MODE_ATTITUDE = 1,
} control_mode;


static bool thread_should_exit;
static int mc_task;

static int
mc_thread_main(int argc, char *argv[])
{
	bool motor_test_mode = false;

	/* structures */
	/* declare and safely initialize all structs */
	struct vehicle_status_s state;
	memset(&state, 0, sizeof(state));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));
	struct ardrone_motors_setpoint_s setpoint;
	memset(&setpoint, 0, sizeof(setpoint));
	struct actuator_controls_s actuator_controls;
	memset(&actuator_controls, 0, sizeof(actuator_controls));

	struct actuator_controls_s actuators;
	struct actuator_armed_s armed;

	/* subscribe to attitude, motor setpoints and system state */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int att_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	// int setpoint_sub = orb_subscribe(ORB_ID(ardrone_motors_setpoint));

	/* rate-limit the attitude subscription to 200Hz to pace our loop */
	orb_set_interval(att_sub, 5);
	struct pollfd fds = { .fd = att_sub, .events = POLLIN };

	/* publish actuator controls */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;
	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	armed.armed = true;
	orb_advert_t armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);

	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "multirotor_att_control");

	/* welcome user */
	printf("[multirotor_att_control] starting\n");

	while (!thread_should_exit) {

		/* wait for a sensor update, check for exit condition every 500 ms */
		poll(&fds, 1, 500);

		perf_begin(mc_loop_perf);

		/* get a local copy of manual setpoint */
		orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
		/* get a local copy of attitude */
		orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
		/* get a local copy of attitude setpoint */
		orb_copy(ORB_ID(vehicle_attitude_setpoint), att_setpoint_sub, &att_sp);

		att_sp.roll_body = -manual.roll * M_PI_F / 8.0f;
		att_sp.pitch_body = -manual.pitch * M_PI_F / 8.0f;
		att_sp.yaw_body = -manual.yaw * M_PI_F;
		if (motor_test_mode) {
			att_sp.roll_body = 0.0f;
			att_sp.pitch_body = 0.0f;
			att_sp.yaw_body = 0.0f;
			att_sp.thrust = 0.3f;
		} else {
			if (state.state_machine == SYSTEM_STATE_MANUAL ||
				state.state_machine == SYSTEM_STATE_GROUND_READY ||
				state.state_machine == SYSTEM_STATE_STABILIZED ||
				state.state_machine == SYSTEM_STATE_AUTO ||
				state.state_machine == SYSTEM_STATE_MISSION_ABORT ||
				state.state_machine == SYSTEM_STATE_EMCY_LANDING) {
				att_sp.thrust = manual.throttle;

			} else if (state.state_machine == SYSTEM_STATE_EMCY_CUTOFF) {
				/* immediately cut off motors */
				att_sp.thrust = 0.0f;

			} else {
				/* limit motor throttle to zero for an unknown mode */
				att_sp.thrust = 0.0f;
			}
			
		}

		multirotor_control_attitude(&att_sp, &att, &state, &actuator_controls, motor_test_mode);
		//ardrone_mixing_and_output(ardrone_write, &actuator_controls, motor_test_mode);

		/* publish the result */
		orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

		perf_end(mc_loop_perf);
	}

	printf("[multirotor att control] stopping.\n");

	/* kill all outputs */
	armed.armed = false;
	orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;
	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);


	close(att_sub);
	close(state_sub);
	close(manual_sub);
	close(actuator_pub);
	close(armed_pub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	exit(0);
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: multirotor_att_control [-m <mode>] {start|stop}\n");
	fprintf(stderr, "    <mode> is 'rates' or 'attitude'\n");
	exit(1);
}

int multirotor_att_control_main(int argc, char *argv[])
{
	int	ch;

	control_mode = CONTROL_MODE_RATES;
	unsigned int optioncount = 0;

	while ((ch = getopt(argc, argv, "m:")) != EOF) {
		switch (ch) {
		case 'm':
			if (!strcmp(optarg, "rates")) {
				control_mode = CONTROL_MODE_RATES;
			} else if (!strcmp(optarg, "attitude")) {
				control_mode = CONTROL_MODE_RATES;
			} else {
				usage("unrecognized -m value");
			}
			optioncount += 2;
		default:
			usage("unrecognized option");
		}
	}
	argc -= optioncount;
	argv += optioncount;

	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		thread_should_exit = false;
		mc_task = task_create("multirotor_att_control", SCHED_PRIORITY_MAX - 15, 2048, mc_thread_main, NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}
