/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: 	Lorenz Meier <lm@inf.ethz.ch>
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
 * @file main.c
 * Implementation of a fixed wing attitude controller. This file is a complete
 * fixed wing controller flying manual attitude control or auto waypoint control.
 * There is no need to touch any other system components to extend / modify the
 * complete control architecture.
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
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

/* process-specific header files */
#include "params.h"

/* Prototypes */
/**
 * Daemon management function.
 */
__EXPORT int ex_fixedwing_control_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int fixedwing_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

void control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att,
	float speed_body[], float gyro[], struct vehicle_rates_setpoint_s *rates_sp, 
	struct actuator_controls_s *actuators);

void control_heading(const struct vehicle_global_position_s *pos, const struct vehicle_global_position_setpoint_s *sp,
	const struct vehicle_attitude_s *att, struct vehicle_attitude_setpoint_s *att_sp);

/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static struct params p;
static struct param_handles ph;

void control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att,
	float speed_body[], float gyro[], struct vehicle_rates_setpoint_s *rates_sp, 
	struct actuator_controls_s *actuators)
{

	/* 
	 * The PX4 architecture provides a mixer outside of the controller.
	 * The mixer is fed with a default vector of actuator controls, representing
	 * moments applied to the vehicle frame. This vector
	 * is structured as:
	 *
	 * Control Group 0 (attitude):
	 * 
	 *    0  -  roll   (-1..+1)
	 *    1  -  pitch  (-1..+1)
	 *    2  -  yaw    (-1..+1)
	 *    3  -  thrust ( 0..+1)
	 *    4  -  flaps  (-1..+1)
	 *    ...
	 *
	 * Control Group 1 (payloads / special):
	 *
	 *    ...
	 */

	/*
	 * Calculate roll error and apply P gain
	 */
	float roll_err = att->roll - att_sp->roll_body;
	actuators->control[0] = roll_err * p.roll_p;

	/*
	 * Calculate pitch error and apply P gain
	 */
	float pitch_err = att->pitch - att_sp->pitch_body;
	actuators->control[1] = pitch_err * p.pitch_p;
}

void control_heading(const struct vehicle_global_position_s *pos, const struct vehicle_global_position_setpoint_s *sp,
	const struct vehicle_attitude_s *att, struct vehicle_attitude_setpoint_s *att_sp)
{

	/*
	 * Calculate heading error of current position to desired position
	 */

	/* PX4 uses 1e7 scaled integers to represent global coordinates for max resolution */
	float bearing = get_bearing_to_next_waypoint(pos->lat/1e7d, pos->lon/1e7d, sp->lat/1e7d, sp->lon/1e7d);

	/* calculate heading error */
	float yaw_err = att->yaw - bearing;
	/* apply control gain */
	att_sp->roll_body = yaw_err * p.hdng_p;
}

/* Main Thread */
int fixedwing_control_thread_main(int argc, char *argv[])
{
	/* read arguments */
	bool verbose = false;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}

	/* welcome user (warnx prints a line, including an appended\n, with variable arguments */
	warnx("[example fixedwing control] started");

	/* initialize parameters, first the handles, then the values */
	parameters_init(&ph);
	parameters_update(&ph, &p);

	/* declare and safely initialize all structs to zero */
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
	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));
	struct vehicle_global_position_setpoint_s global_sp;
	memset(&global_sp, 0, sizeof(global_sp));

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
	int vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	int global_sp_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));
	int param_sub = orb_subscribe(ORB_ID(parameter_update));

	/* Setup of loop */
	float gyro[3] = {0.0f, 0.0f, 0.0f};
	float speed_body[3] = {0.0f, 0.0f, 0.0f};
	struct pollfd fds[2] = {{ .fd = param_sub, .events = POLLIN },
				{ .fd = att_sub, .events = POLLIN }};

	while (!thread_should_exit) {

		/*
		 * Wait for a sensor or param update, check for exit condition every 500 ms.
		 * This means that the execution will block here without consuming any resources,
		 * but will continue to execute the very moment a new attitude measurement or
		 * a param update is published. So no latency in contrast to the polling
		 * design pattern (do not confuse the poll() system call with polling).
		 *
		 * This design pattern makes the controller also agnostic of the attitude
		 * update speed - it runs as fast as the attitude updates with minimal latency.
		 */
		int ret = poll(fds, 2, 500);

		if (ret < 0) {
			/* poll error, this will not really happen in practice */
			warnx("poll error");

		} else if (ret == 0) {
			/* no return value = nothing changed for 500 ms, ignore */
		} else {

			/* only update parameters if they changed */
			if (fds[0].revents & POLLIN) {
				/* read from param to clear updated flag (uORB API requirement) */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), param_sub, &update);

				/* if a param update occured, re-read our parameters */
				parameters_update(&ph, &p);
			}

			/* only run controller if attitude changed */
			if (fds[1].revents & POLLIN) {


				/* Check if there is a new position measurement or position setpoint */
				bool pos_updated;
				orb_check(global_pos_sub, &pos_updated);
				bool global_sp_updated;
				orb_check(global_sp_sub, &global_sp_updated);

				/* get a local copy of attitude */
				orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

				if (global_sp_updated)
					orb_copy(ORB_ID(vehicle_global_position_setpoint), global_sp_sub, &global_sp);

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

						warnx("Did not get a valid R\n");
					}
				}

				orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
				orb_copy(ORB_ID(vehicle_status), vstatus_sub, &vstatus);

				gyro[0] = att.rollspeed;
				gyro[1] = att.pitchspeed;
				gyro[2] = att.yawspeed;

				/* control */

				if (vstatus.state_machine == SYSTEM_STATE_AUTO ||
				    vstatus.state_machine == SYSTEM_STATE_STABILIZED) {

				    	/* simple heading control */
				    	control_heading(&global_pos, &global_sp, &att, &att_sp);

				    	/* nail pitch and yaw (rudder) to zero. This example only controls roll (index 0) */
				    	actuators.control[1] = 0.0f;
				    	actuators.control[2] = 0.0f;
					
					/* simple attitude control */
					control_attitude(&att_sp, &att, speed_body, gyro, &rates_sp, &actuators);

					/* pass through throttle */
					actuators.control[3] = att_sp.thrust;

					/* set flaps to zero */
					actuators.control[4] = 0.0f;

				} else if (vstatus.state_machine == SYSTEM_STATE_MANUAL) {
					if (vstatus.manual_control_mode == VEHICLE_MANUAL_CONTROL_MODE_SAS) {

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

						/* attitude control */
						control_attitude(&att_sp, &att, speed_body, gyro, &rates_sp, &actuators);

						/* pass through throttle */
						actuators.control[3] = att_sp.thrust;

						/* pass through flaps */
						if (isfinite(manual_sp.flaps)) {
							actuators.control[4] = manual_sp.flaps;

						} else {
							actuators.control[4] = 0.0f;
						}

					} else if (vstatus.manual_control_mode == VEHICLE_MANUAL_CONTROL_MODE_DIRECT) {
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
		}
	}

	printf("[ex_fixedwing_control] exiting, stopping all motors.\n");
	thread_running = false;

	/* kill all outputs */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;

	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

	fflush(stdout);

	return 0;
}

/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: ex_fixedwing_control {start|stop|status}\n\n");
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
int ex_fixedwing_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("ex_fixedwing_control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("ex_fixedwing_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 20,
					 2048,
					 fixedwing_control_thread_main,
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
			printf("\tex_fixedwing_control is running\n");

		} else {
			printf("\tex_fixedwing_control not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}



