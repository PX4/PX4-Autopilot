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
 * @file fixedwing_pos_control.c
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
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/geo/geo.h>
#include <systemlib/systemlib.h>

/*
 * Controller parameters, accessible via MAVLink
 *
 */
PARAM_DEFINE_FLOAT(FW_HEADING_P, 0.1f);
PARAM_DEFINE_FLOAT(FW_ALT_P, 0.1f);
PARAM_DEFINE_FLOAT(FW_ROLL_LIM, 0.7f);	// Roll angle limit in radians
PARAM_DEFINE_FLOAT(FW_PITCH_LIM, 0.35f);	// Pitch angle limit in radians


struct fw_pos_control_params {
	float heading_p;
	float altitude_p;
	float roll_lim;
	float pitch_lim;
};

struct fw_pos_control_param_handles {
	param_t heading_p;
	param_t altitude_p;
	param_t roll_lim;
	param_t pitch_lim;

};


/* Prototypes */
/* Internal Prototypes */
static int parameters_init(struct fw_pos_control_param_handles *h);
static int parameters_update(const struct fw_pos_control_param_handles *h, struct fw_pos_control_params *p);

/**
 * Deamon management function.
 */
__EXPORT int fixedwing_pos_control_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int fixedwing_pos_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/* Variables */
static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */


/**
 * Parameter management
 */
static int parameters_init(struct fw_pos_control_param_handles *h)
{
	/* PID parameters */
	h->heading_p 	=	param_find("FW_HEADING_P");
	h->altitude_p 	=	param_find("FW_ALT_P");
	h->roll_lim 	=	param_find("FW_ROLL_LIM");
	h->pitch_lim 	=	param_find("FW_PITCH_LIM");


	return OK;
}

static int parameters_update(const struct fw_pos_control_param_handles *h, struct fw_pos_control_params *p)
{
	param_get(h->heading_p, &(p->heading_p));
	param_get(h->altitude_p, &(p->altitude_p));
	param_get(h->roll_lim, &(p->roll_lim));
	param_get(h->pitch_lim, &(p->pitch_lim));

	return OK;
}


/* Main Thread */
int fixedwing_pos_control_thread_main(int argc, char *argv[])
{
	/* read arguments */
		bool verbose = false;

		for (int i = 1; i < argc; i++) {
			if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
				verbose = true;
			}
		}

		/* welcome user */
		printf("[fixedwing att_control] started\n");

		/* declare and safely initialize all structs */
		struct vehicle_global_position_s global_pos;
		memset(&global_pos, 0, sizeof(global_pos));
		struct vehicle_global_position_setpoint_s global_setpoint;
		memset(&global_setpoint, 0, sizeof(global_setpoint));
		struct vehicle_attitude_s att;
		memset(&att, 0, sizeof(att));

		/* output structs */
		struct vehicle_attitude_setpoint_s attitude_setpoint;
		memset(&attitude_setpoint, 0, sizeof(attitude_setpoint));

		/* publish attitude setpoint */
		attitude_setpoint.roll_tait_bryan = 0.0f;
		attitude_setpoint.pitch_tait_bryan = 0.0f;
		attitude_setpoint.yaw_tait_bryan = 0.0f;
		orb_advert_t attitude_setpoint_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &attitude_setpoint);

		/* subscribe */
		int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
		int global_setpoint_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));
		int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

		/* Setup of loop */
		struct pollfd fds = { .fd = att_sub, .events = POLLIN };
		bool global_sp_updated_set_once = false;

		while(!thread_should_exit)
		{
			/* wait for a sensor update, check for exit condition every 500 ms */
			poll(&fds, 1, 500);

			static int counter = 0;
			static bool initialized = false;

			static struct fw_pos_control_params p;
			static struct fw_pos_control_param_handles h;

			PID_t heading_controller;
			PID_t altitude_controller;

			if(!initialized)
			{
				parameters_init(&h);
				parameters_update(&h, &p);
				pid_init(&heading_controller, p.heading_p, 0.0f, 0.0f, 0.0f,p.roll_lim,PID_MODE_DERIVATIV_NONE);
				pid_init(&altitude_controller, p.altitude_p, 0.0f, 0.0f, 0.0f,p.pitch_lim,PID_MODE_DERIVATIV_NONE);
				initialized = true;
			}

			/* load new parameters with lower rate */
			if (counter % 100 == 0) {
				/* update parameters from storage */
				parameters_update(&h, &p);
				pid_set_parameters(&heading_controller, p.heading_p, 0, 0, 0, p.roll_lim);
				pid_set_parameters(&altitude_controller, p.altitude_p, 0, 0, 0, p.pitch_lim);

			}

			/* Check if there is a new position or setpoint */
			bool pos_updated;
			orb_check(global_pos_sub, &pos_updated);
			bool global_sp_updated;
			orb_check(global_setpoint_sub, &global_sp_updated);
			if(global_sp_updated)
				global_sp_updated_set_once = true;


			/* Load local copies */
			orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
			if(pos_updated)
				orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &global_pos);
			if (global_sp_updated)
				orb_copy(ORB_ID(vehicle_global_position_setpoint), global_setpoint_sub, &global_setpoint);

			/* Control */


			/* Simple Horizontal Control */
			if(global_sp_updated_set_once)
			{
				/* calculate bearing error */
				float target_bearing = get_bearing_to_next_waypoint(global_pos.lat / (double)1e7d, global_pos.lon / (double)1e7d,
					global_setpoint.lat / (double)1e7d, global_setpoint.lon / (double)1e7d);

				/* shift error to prevent wrapping issues */
				float bearing_error = target_bearing - att.yaw;

				if (bearing_error < M_PI_F) {
					bearing_error += 2.0f * M_PI_F;
				}

				if (bearing_error > M_PI_F) {
					bearing_error -= 2.0f * M_PI_F;
				}

				/* calculate roll setpoint, do this artificially around zero */
				attitude_setpoint.roll_tait_bryan = pid_calculate(&heading_controller, bearing_error, 0.0f, 0.0f, 0.0f);
			}

			/* Very simple Altitude Control */
			if(global_sp_updated_set_once && pos_updated)
			{

				//TODO: take care of relatie vs. ab. altitude
				attitude_setpoint.pitch_tait_bryan = pid_calculate(&altitude_controller, global_setpoint.altitude, global_pos.alt, 0.0f, 0.0f);

			}
			/*Publish the attitude setpoint */
			orb_publish(ORB_ID(vehicle_attitude_setpoint), attitude_setpoint_pub, &attitude_setpoint);

			counter++;
		}

		printf("[fixedwing_pos_control] exiting.\n");
		thread_running = false;


		close(attitude_setpoint_pub);

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
	fprintf(stderr, "usage: fixedwing_pos_control {start|stop|status}\n\n");
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
int fixedwing_pos_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("fixedwing_pos_control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("fixedwing_pos_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 20,
					 4096,
					 fixedwing_pos_control_thread_main,
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
			printf("\tfixedwing_pos_control is running\n");
		} else {
			printf("\tfixedwing_pos_control not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}



