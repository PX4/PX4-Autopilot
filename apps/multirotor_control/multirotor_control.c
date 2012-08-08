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

/*
 * @file multirotor_control.c
 *
 * Implementation of multirotor controllers
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
#include <poll.h>
#include <sys/prctl.h>
#include <arch/board/up_hrt.h>
#include "multirotor_control.h"
#include "multirotor_attitude_control.h"
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/rc_channels.h>

#include <systemlib/perf_counter.h>

__EXPORT int multirotor_control_main(int argc, char *argv[]);


static enum {
	CONTROL_MODE_RATES = 0,
	CONTROL_MODE_ATTITUDE = 1,
} control_mode;


static bool thread_should_exit;
static int mc_task;

static int
mc_thread_main(int argc, char *argv[])
{
	/* structures */
	struct vehicle_status_s state;
	struct vehicle_attitude_s att;
	struct rc_channels_s rc;
	struct actuator_controls_s actuators;
	struct actuator_armed_s armed;

	/* subscribe to attitude, motor setpoints and system state */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int rc_sub = orb_subscribe(ORB_ID(rc_channels));

	/* rate-limit the attitude subscription to 200Hz to pace our loop */
	orb_set_interval(att_sub, 5);
	struct pollfd fds = { .fd = att_sub, .events = POLLIN };

	/* publish actuator controls */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;
	int actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	armed.armed = true;
	int armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);

	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "multirotor_control");

	/* welcome user */
	printf("[multirotor_control] starting\n");

	while (!thread_should_exit) {

		/* wait for a sensor update */
		poll(&fds, 1, -1);

		perf_begin(mc_loop_perf);

		/* get a local copy of the vehicle state */
		orb_copy(ORB_ID(vehicle_status), state_sub, &state);

		/* get a local copy of rc inputs */
		orb_copy(ORB_ID(rc_channels), rc_sub, &rc);

		/* get a local copy of attitude */
		orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

		/* run the attitude controller */
		multirotor_control_attitude(&rc, &att, &state, &actuators);

		/* publish the result */
		orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

		perf_end(mc_loop_perf);
	}

	printf("[multirotor_control] stopping\r\n");

	/* kill all outputs */
	armed.armed = false;
	orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;
	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);


	close(att_sub);
	close(state_sub);
	close(rc_sub);
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
	fprintf(stderr, "usage: multirotor_control [-m <mode>] {start|stop}\n");
	fprintf(stderr, "    <mode> is 'rates' or 'attitude'\n");
	exit(1);
}

int multirotor_control_main(int argc, char *argv[])
{
	int	ch;

	control_mode = CONTROL_MODE_RATES;

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
		default:
			usage("unrecognized option");
		}
	}
	argc -= optind;
	argv += optind;

	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		thread_should_exit = false;
		mc_task = task_create("multirotor_attitude", SCHED_PRIORITY_MAX - 15, 2048, mc_thread_main, NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	usage("unrecognised command");
	exit(1);
}
