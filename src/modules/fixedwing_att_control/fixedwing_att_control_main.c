/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: 	@author Thomas Gubler <thomasgubler@student.ethz.ch>
 *   			@author Doug Weibel <douglas.weibel@colorado.edu>
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
 * @file fixedwing_att_control.c
 * Implementation of a fixed wing attitude controller.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/debug_key_value.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>

#include "fixedwing_att_control_rate.h"
#include "fixedwing_att_control_att.h"

/* Prototypes */
/**
 * Deamon management function.
 */
__EXPORT int fixedwing_att_control_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int fixedwing_att_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/* Variables */
static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

/* Main Thread */
int fixedwing_att_control_thread_main(int argc, char *argv[])
{
	/* read arguments */
	bool verbose = false;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}

	/* welcome user */
	printf("[fixedwing att control] started\n");

	/* declare and safely initialize all structs */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct vehicle_rates_setpoint_s rates_sp;
	memset(&rates_sp, 0, sizeof(rates_sp));
	struct vehicle_global_position_s global_pos;
	memset(&global_pos, 0, sizeof(global_pos));
	struct manual_control_setpoint_s manual_sp;
	memset(&manual_sp, 0, sizeof(manual_sp));
	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));
	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));

	/* output structs */
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));


	/* publish actuator controls */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}

	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	orb_advert_t rates_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);

	/* subscribe */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* Setup of loop */
	float gyro[3] = {0.0f, 0.0f, 0.0f};
	float speed_body[3] = {0.0f, 0.0f, 0.0f};
	struct pollfd fds = { .fd = att_sub, .events = POLLIN };

	while (!thread_should_exit) {
		/* wait for a sensor update, check for exit condition every 500 ms */
		poll(&fds, 1, 500);

		/* Check if there is a new position measurement or  attitude setpoint */
		bool pos_updated;
		orb_check(global_pos_sub, &pos_updated);
		bool att_sp_updated;
		orb_check(att_sp_sub, &att_sp_updated);

		/* get a local copy of attitude */
		orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

		if (att_sp_updated)
			orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &att_sp);

		if (pos_updated) {
			orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &global_pos);

			if (att.R_valid) {
				speed_body[0] = att.R[0][0] * global_pos.vx + att.R[0][1] * global_pos.vy + att.R[0][2] * global_pos.vz;
				speed_body[1] = att.R[1][0] * global_pos.vx + att.R[1][1] * global_pos.vy + att.R[1][2] * global_pos.vz;
				speed_body[2] = att.R[2][0] * global_pos.vx + att.R[2][1] * global_pos.vy + att.R[2][2] * global_pos.vz;

			} else {
				speed_body[0] = 0;
				speed_body[1] = 0;
				speed_body[2] = 0;

				printf("FW ATT CONTROL: Did not get a valid R\n");
			}
		}

		orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
		orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);
		orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);

		gyro[0] = att.rollspeed;
		gyro[1] = att.pitchspeed;
		gyro[2] = att.yawspeed;

		/* set manual setpoints if required */
		if (control_mode.flag_control_manual_enabled) {
			if (control_mode.flag_control_attitude_enabled) {

				/* if the RC signal is lost, try to stay level and go slowly back down to ground */
				if (vstatus.rc_signal_lost) {

					/* put plane into loiter */
					att_sp.roll_body = 0.3f;
					att_sp.pitch_body = 0.0f;

					/* limit throttle to 60 % of last value if sane */
					if (isfinite(manual_sp.throttle) &&
					    (manual_sp.throttle >= 0.0f) &&
					    (manual_sp.throttle <= 1.0f)) {
						att_sp.thrust = 0.6f * manual_sp.throttle;

					} else {
						att_sp.thrust = 0.0f;
					}

					att_sp.yaw_body = 0;

					// XXX disable yaw control, loiter

				} else {

					att_sp.roll_body = manual_sp.roll;
					att_sp.pitch_body = manual_sp.pitch;
					att_sp.yaw_body = 0;
					att_sp.thrust = manual_sp.throttle;
				}

				att_sp.timestamp = hrt_absolute_time();

				/* pass through flaps */
				if (isfinite(manual_sp.flaps)) {
					actuators.control[4] = manual_sp.flaps;

				} else {
					actuators.control[4] = 0.0f;
				}

			} else {
				/* directly pass through values */
				actuators.control[0] = manual_sp.roll;
				/* positive pitch means negative actuator -> pull up */
				actuators.control[1] = manual_sp.pitch;
				actuators.control[2] = manual_sp.yaw;
				actuators.control[3] = manual_sp.throttle;

				if (isfinite(manual_sp.flaps)) {
					actuators.control[4] = manual_sp.flaps;

				} else {
					actuators.control[4] = 0.0f;
				}
			}
		}
		
		/* execute attitude control if requested */
		if (control_mode.flag_control_attitude_enabled) {
			/* attitude control */
			fixedwing_att_control_attitude(&att_sp, &att, speed_body, &rates_sp);

			/* angular rate control */
			fixedwing_att_control_rates(&rates_sp, gyro, &actuators);

			/* pass through throttle */
			actuators.control[3] = att_sp.thrust;

			/* set flaps to zero */
			actuators.control[4] = 0.0f;

		}

		/* publish rates */
		orb_publish(ORB_ID(vehicle_rates_setpoint), rates_pub, &rates_sp);

		/* sanity check and publish actuator outputs */
		if (isfinite(actuators.control[0]) &&
		    isfinite(actuators.control[1]) &&
		    isfinite(actuators.control[2]) &&
		    isfinite(actuators.control[3])) {
			orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
		}
	}

	printf("[fixedwing_att_control] exiting, stopping all motors.\n");
	thread_running = false;

	/* kill all outputs */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;

	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);



	close(att_sub);
	close(actuator_pub);
	close(rates_pub);

	fflush(stdout);
	exit(0);

	return 0;

}

/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: fixedwing_att_control {start|stop|status}\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int fixedwing_att_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("fixedwing_att_control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd("fixedwing_att_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 20,
					 2048,
					 fixedwing_att_control_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		thread_running = true;
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tfixedwing_att_control is running\n");

		} else {
			printf("\tfixedwing_att_control not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}



