/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Ivan Ovinnikov <oivan@ethz.ch>
 *   Modifications: Doug Weibel <douglas.weibel@colorado.edu>
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
 * @file fixedwing_control.c
 * Implementation of a fixed wing attitude and position controller.
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
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/geo/geo.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/debug_key_value.h>

static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

/**
 * Deamon management function.
 */
__EXPORT int fixedwing_control_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int fixedwing_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/*
 * Controller parameters, accessible via MAVLink
 *
 */
// Roll control parameters
PARAM_DEFINE_FLOAT(FW_ROLLRATE_P, 0.3f);
// Need to add functionality to suppress integrator windup while on the ground
// Suggested value of FW_ROLLRATE_I is 0.0 till this is in place
PARAM_DEFINE_FLOAT(FW_ROLLRATE_I, 0.0f);
PARAM_DEFINE_FLOAT(FW_ROLLRATE_AWU, 0.0f);
PARAM_DEFINE_FLOAT(FW_ROLLRATE_LIM, 0.7f);   // Roll rate limit in radians/sec
PARAM_DEFINE_FLOAT(FW_ROLL_P, 0.3f);
PARAM_DEFINE_FLOAT(FW_ROLL_LIM, 0.7f);	// Roll angle limit in radians

//Pitch control parameters
PARAM_DEFINE_FLOAT(FW_PITCHRATE_P, 0.3f);
// Need to add functionality to suppress integrator windup while on the ground
// Suggested value of FW_PITCHRATE_I is 0.0 till this is in place
PARAM_DEFINE_FLOAT(FW_PITCHRATE_I, 0.0f);
PARAM_DEFINE_FLOAT(FW_PITCHRATE_AWU, 0.0f);
PARAM_DEFINE_FLOAT(FW_PITCHRATE_LIM, 0.35f);   // Pitch rate limit in radians/sec
PARAM_DEFINE_FLOAT(FW_PITCH_P, 0.3f);
PARAM_DEFINE_FLOAT(FW_PITCH_LIM, 0.35f);	// Pitch angle limit in radians

struct fw_att_control_params {
	float rollrate_p;
	float rollrate_i;
	float rollrate_awu;
	float rollrate_lim;
	float roll_p;
	float roll_lim;
	float pitchrate_p;
	float pitchrate_i;
	float pitchrate_awu;
	float pitchrate_lim;
	float pitch_p;
	float pitch_lim;
};

struct fw_att_control_param_handles {
	param_t rollrate_p;
	param_t rollrate_i;
	param_t rollrate_awu;
	param_t rollrate_lim;
	param_t roll_p;
	param_t roll_lim;
	param_t pitchrate_p;
	param_t pitchrate_i;
	param_t pitchrate_awu;
	param_t pitchrate_lim;
	param_t pitch_p;
	param_t pitch_lim;
};


// TO_DO - Navigation control will be moved to a separate app
// Attitude control will just handle the inner angle and rate loops
// to control pitch and roll, and turn coordination via rudder and 
// possibly throttle compensation for battery voltage sag.

PARAM_DEFINE_FLOAT(FW_HEADING_P, 0.1f);
PARAM_DEFINE_FLOAT(FW_HEADING_LIM, 0.15f);

struct fw_pos_control_params {
	float heading_p;
	float heading_lim;
};

struct fw_pos_control_param_handles {
	param_t heading_p;
	param_t heading_lim;
};

/**
 * Initialize all parameter handles and values
 *
 */
static int att_parameters_init(struct fw_att_control_param_handles *h);

/**
 * Update all parameters
 *
 */
static int att_parameters_update(const struct fw_att_control_param_handles *h, struct fw_att_control_params *p);

/**
 * Initialize all parameter handles and values
 *
 */
static int pos_parameters_init(struct fw_pos_control_param_handles *h);

/**
 * Update all parameters
 *
 */
static int pos_parameters_update(const struct fw_pos_control_param_handles *h, struct fw_pos_control_params *p);


/**
 * The fixed wing control main thread.
 *
 * The main loop executes continously and calculates the control
 * response.
 *
 * @param argc number of arguments
 * @param argv argument array
 *
 * @return 0
 *
 */
int fixedwing_control_thread_main(int argc, char *argv[])
{
	/* read arguments */
	bool verbose = false;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}

	/* welcome user */
	printf("[fixedwing control] started\n");

	/* output structs */
	struct actuator_controls_s actuators;
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));

	/* publish actuator controls */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;
	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	/* Subscribe to global position, attitude and rc */
	/* declare and safely initialize all structs */
	struct vehicle_status_s state;
	memset(&state, 0, sizeof(state));
	struct vehicle_attitude_s att;
	memset(&att_sp, 0, sizeof(att_sp));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));

	/* subscribe to attitude, motor setpoints and system state */
	struct vehicle_global_position_s global_pos;
	int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	struct vehicle_global_position_setpoint_s global_setpoint;
	int global_setpoint_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int att_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	/* Mainloop setup */
	unsigned int loopcounter = 0;

	uint64_t last_run = 0;
	uint64_t last_run_pos = 0;

	bool global_sp_updated_set_once = false;

	struct fw_att_control_params p;
	struct fw_att_control_param_handles h;

	struct fw_pos_control_params ppos;
	struct fw_pos_control_param_handles hpos;

	/* initialize the pid controllers */
	att_parameters_init(&h);
	att_parameters_update(&h, &p);

	pos_parameters_init(&hpos);
	pos_parameters_update(&hpos, &ppos);

// TO_DO  Fix output limit functionallity of PID controller or add that function elsewhere
	PID_t roll_rate_controller;
	pid_init(&roll_rate_controller, p.rollrate_p, p.rollrate_i, 0.0f, p.rollrate_awu,
			p.rollrate_lim,PID_MODE_DERIVATIV_NONE);
	PID_t roll_angle_controller;
	pid_init(&roll_angle_controller, p.roll_p, 0.0f, 0.0f, 0.0f,
			p.roll_lim,PID_MODE_DERIVATIV_NONE);
			
	PID_t pitch_rate_controller;
	pid_init(&pitch_rate_controller, p.pitchrate_p, p.pitchrate_i, 0.0f, p.pitchrate_awu,
			p.pitchrate_lim,PID_MODE_DERIVATIV_NONE);
	PID_t pitch_angle_controller;
	pid_init(&pitch_angle_controller, p.pitch_p, 0.0f, 0.0f, 0.0f,
			p.pitch_lim,PID_MODE_DERIVATIV_NONE);

	PID_t heading_controller;
	pid_init(&heading_controller, ppos.heading_p, 0.0f, 0.0f, 0.0f,
			100.0f,PID_MODE_DERIVATIV_SET);		// Temporary arbitrarily large limit

	// XXX remove in production
	/* advertise debug value */
	struct debug_key_value_s dbg = { .key = "", .value = 0.0f };
	orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);

	// This is the top of the main loop
	while(!thread_should_exit) {

		struct pollfd fds[1] = {
			{ .fd = att_sub,   .events = POLLIN },
		};
		int ret = poll(fds, 1, 1000);

		if (ret < 0) {
			/* XXX this is seriously bad - should be an emergency */
		} else if (ret == 0) {
			/* XXX this means no sensor data - should be critical or emergency */
			printf("[fixedwing control] WARNING: Not getting attitude - estimator running?\n");
		} else {

			// FIXME SUBSCRIBE
			if (loopcounter % 100 == 0) {
				att_parameters_update(&h, &p);
				pos_parameters_update(&hpos, &ppos);
				pid_set_parameters(&roll_rate_controller, p.rollrate_p, p.rollrate_i, 0.0f, 
									p.rollrate_awu, p.rollrate_lim);
				pid_set_parameters(&roll_angle_controller, p.roll_p, 0.0f, 0.0f, 
									0.0f, p.roll_lim);
				pid_set_parameters(&pitch_rate_controller, p.pitchrate_p, p.pitchrate_i, 0.0f, 
									p.pitchrate_awu, p.pitchrate_lim);
				pid_set_parameters(&pitch_angle_controller, p.pitch_p, 0.0f, 0.0f, 
									0.0f, p.pitch_lim);
				pid_set_parameters(&heading_controller, ppos.heading_p, 0.0f, 0.0f, 0.0f, 90.0f);
//printf("[fixedwing control debug] p: %8.4f, i: %8.4f, limit: %8.4f  \n",
//p.rollrate_p, p.rollrate_i, p.rollrate_lim);
			}

			/* if position updated, run position control loop */
			bool pos_updated;
			orb_check(global_pos_sub, &pos_updated);
			bool global_sp_updated;
			orb_check(global_setpoint_sub, &global_sp_updated);
			if (global_sp_updated) {
				global_sp_updated_set_once = true;
			}
			/* checking has to happen before the read, as the read clears the changed flag */

			/* get a local copy of system state */
			orb_copy(ORB_ID(vehicle_status), state_sub, &state);
			/* get a local copy of manual setpoint */
			orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
			/* get a local copy of attitude */
			orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
			/* get a local copy of attitude setpoint */
			//orb_copy(ORB_ID(vehicle_attitude_setpoint), att_setpoint_sub, &att_sp);
			// XXX update to switch between external attitude reference and the
			// attitude calculated here

			char name[10];

			if (pos_updated) {

				/* get position */
				orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &global_pos);
				
				if (global_sp_updated_set_once) {
					orb_copy(ORB_ID(vehicle_global_position_setpoint), global_setpoint_sub, &global_setpoint);
				
			
					/* calculate delta T */
					const float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
					last_run = hrt_absolute_time();

					/* calculate bearing error */
					float target_bearing = get_bearing_to_next_waypoint(global_pos.lat / (double)1e7d, global_pos.lon / (double)1e7d,
						global_setpoint.lat / (double)1e7d, global_setpoint.lon / (double)1e7d);

					/* shift error to prevent wrapping issues */
					float bearing_error = target_bearing - att.yaw;

					if (loopcounter % 2 == 0) {
						sprintf(name, "hdng err1");
						memcpy(dbg.key, name, sizeof(name));
						dbg.value = bearing_error;
						orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
					}

					if (bearing_error < M_PI_F) {
						bearing_error += 2.0f * M_PI_F;
					}

					if (bearing_error > M_PI_F) {
						bearing_error -= 2.0f * M_PI_F;
					}

					if (loopcounter % 2 != 0) {
						sprintf(name, "hdng err2");
						memcpy(dbg.key, name, sizeof(name));
						dbg.value = bearing_error;
						orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
					}

					/* calculate roll setpoint, do this artificially around zero */
					att_sp.roll_body = pid_calculate(&heading_controller, bearing_error,
					0.0f, att.yawspeed, deltaT);

					/* limit roll angle output */
					if (att_sp.roll_body > ppos.heading_lim) {
						att_sp.roll_body = ppos.heading_lim;
						heading_controller.saturated = 1;
					}

					if (att_sp.roll_body < -ppos.heading_lim) {
						att_sp.roll_body = -ppos.heading_lim;
						heading_controller.saturated = 1;
					}

					att_sp.pitch_body = 0.0f;
					att_sp.yaw_body = 0.0f;

				} else {
					/* no setpoint, maintain level flight */
					att_sp.roll_body = 0.0f;
					att_sp.pitch_body = 0.0f;
					att_sp.yaw_body = 0.0f;
				}

				att_sp.thrust = 0.7f;
			}

			/* calculate delta T */
			const float deltaTpos = (hrt_absolute_time() - last_run_pos) / 1000000.0f;
			last_run_pos = hrt_absolute_time();

			if (verbose && (loopcounter % 20 == 0)) {
				printf("[fixedwing control] roll sp: %8.4f, \n", att_sp.roll_body);
			}

			// actuator control[0] is aileron (or elevon roll control)
			// Commanded roll rate from P controller on roll angle
			float roll_rate_command = pid_calculate(&roll_angle_controller, att_sp.roll_body,
					att.roll, 0.0f, deltaTpos);
			// actuator control from PI controller on roll rate
			actuators.control[0] = pid_calculate(&roll_rate_controller, roll_rate_command,
					att.rollspeed, 0.0f, deltaTpos);
					
			// actuator control[1] is elevator (or elevon pitch control)
			// Commanded pitch rate from P controller on pitch angle
			float pitch_rate_command = pid_calculate(&pitch_angle_controller, att_sp.pitch_body,
					att.pitch, 0.0f, deltaTpos);
			// actuator control from PI controller on pitch rate
			actuators.control[1] = pid_calculate(&pitch_rate_controller, pitch_rate_command,
					att.pitchspeed, 0.0f, deltaTpos);
					
			// actuator control[3] is throttle
			actuators.control[3] = att_sp.thrust;
					
			/* publish attitude setpoint (for MAVLink) */
			orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);

			/* publish actuator setpoints (for mixer) */
			orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

			loopcounter++;

		}
	}

	printf("[fixedwing_control] exiting.\n");
	thread_running = false;

	return 0;
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: fixedwing_control {start|stop|status}\n\n");
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
int fixedwing_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("fixedwing_control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("fixedwing_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 20,
					 4096,
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
			printf("\tfixedwing_control is running\n");
		} else {
			printf("\tfixedwing_control not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static int att_parameters_init(struct fw_att_control_param_handles *h)
{
	/* PID parameters */
	
	h->rollrate_p 			=	param_find("FW_ROLLRATE_P");
	h->rollrate_i 			=	param_find("FW_ROLLRATE_I");
	h->rollrate_awu 		=	param_find("FW_ROLLRATE_AWU");
	h->rollrate_lim 		=	param_find("FW_ROLLRATE_LIM");
	h->roll_p 				=	param_find("FW_ROLL_P");
	h->roll_lim 			=	param_find("FW_ROLL_LIM");
	h->pitchrate_p 			=	param_find("FW_PITCHRATE_P");
	h->pitchrate_i 			=	param_find("FW_PITCHRATE_I");
	h->pitchrate_awu 		=	param_find("FW_PITCHRATE_AWU");
	h->pitchrate_lim 		=	param_find("FW_PITCHRATE_LIM");
	h->pitch_p 				=	param_find("FW_PITCH_P");
	h->pitch_lim 			=	param_find("FW_PITCH_LIM");
	
	return OK;
}

static int att_parameters_update(const struct fw_att_control_param_handles *h, struct fw_att_control_params *p)
{
	param_get(h->rollrate_p, &(p->rollrate_p));
	param_get(h->rollrate_i, &(p->rollrate_i));
	param_get(h->rollrate_awu, &(p->rollrate_awu));
	param_get(h->rollrate_lim, &(p->rollrate_lim));
	param_get(h->roll_p, &(p->roll_p));
	param_get(h->roll_lim, &(p->roll_lim));
	param_get(h->pitchrate_p, &(p->pitchrate_p));
	param_get(h->pitchrate_i, &(p->pitchrate_i));
	param_get(h->pitchrate_awu, &(p->pitchrate_awu));
	param_get(h->pitchrate_lim, &(p->pitchrate_lim));
	param_get(h->pitch_p, &(p->pitch_p));
	param_get(h->pitch_lim, &(p->pitch_lim));
	
	return OK;
}

static int pos_parameters_init(struct fw_pos_control_param_handles *h)
{
	/* PID parameters */
	h->heading_p 	=	param_find("FW_HEADING_P");
	h->heading_lim 	=	param_find("FW_HEADING_LIM");

	return OK;
}

static int pos_parameters_update(const struct fw_pos_control_param_handles *h, struct fw_pos_control_params *p)
{
	param_get(h->heading_p, &(p->heading_p));
	param_get(h->heading_lim, &(p->heading_lim));

	return OK;
}
