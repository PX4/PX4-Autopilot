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
#include <termios.h>
#include <time.h>
#include <sys/prctl.h>
#include <arch/board/up_hrt.h>
#include "multirotor_control.h"
#include "multirotor_attitude_control.h"
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/ardrone_control.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/sensor_combined.h>

__EXPORT int ardrone_control_main(int argc, char *argv[]);

/****************************************************************************
 * Internal Definitions
 ****************************************************************************/


enum {
	CONTROL_MODE_RATES = 0,
	CONTROL_MODE_ATTITUDE = 1,
} control_mode;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/*File descriptors */
int ardrone_write;
int gpios;

bool position_control_thread_started;

/****************************************************************************
 * pthread loops
 ****************************************************************************/
// static void *position_control_loop(void *arg)
// {
// 	struct vehicle_status_s *state = (struct vehicle_status_s *)arg;
// 	// Set thread name
// 	prctl(PR_SET_NAME, "ardrone pos ctrl", getpid());

// 	while (1) {
// 		if (state->state_machine == SYSTEM_STATE_AUTO) {
// //			control_position();   //FIXME TODO XXX
// 			/* temporary 50 Hz execution */
// 			usleep(20000);

// 		} else {
// 			position_control_thread_started = false;
// 			break;
// 		}
// 	}

// 	return NULL;
// }

/****************************************************************************
 * main
 ****************************************************************************/

int ardrone_control_main(int argc, char *argv[])
{
	/* welcome user */
	printf("[ardrone_control] Control started, taking over motors\n");

	/* default values for arguments */
	char *ardrone_uart_name = "/dev/ttyS1";
	control_mode = CONTROL_MODE_RATES;

	char *commandline_usage = "\tusage: ardrone_control -d ardrone-devicename -m mode\n\tmodes are:\n\t\trates\n\t\tattitude\n";

	/* read commandline arguments */
	int i;

	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) { //ardrone set
			if (argc > i + 1) {
				ardrone_uart_name = argv[i + 1];

			} else {
				printf(commandline_usage);
				return 0;
			}

		} else if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--mode") == 0) {
			if (argc > i + 1) {
				if (strcmp(argv[i + 1], "rates") == 0) {
					control_mode = CONTROL_MODE_RATES;

				} else if (strcmp(argv[i + 1], "attitude") == 0) {
					control_mode = CONTROL_MODE_ATTITUDE;

				} else {
					printf(commandline_usage);
					return 0;
				}

			} else {
				printf(commandline_usage);
				return 0;
			}
		}
	}

	/* initialize motors */
	
	int counter = 0;

	/* pthread for position control */
	// pthread_t position_control_thread;
	// position_control_thread_started = false;

	/* structures */
	struct vehicle_status_s state;
	struct vehicle_attitude_s att;
	struct rc_channels_s rc;
	struct sensor_combined_s raw;

	/* subscribe to attitude, motor setpoints and system state */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int rc_sub = orb_subscribe(ORB_ID(rc_channels));
	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));

	/* publish AR.Drone motor control state */
	// int ardrone_pub = orb_advertise(ORB_ID(ardrone_control), &ar_control);

	while (1) {
		/* get a local copy of the vehicle state */
		orb_copy(ORB_ID(vehicle_status), state_sub, &state);
		/* get a local copy of rc */
		orb_copy(ORB_ID(rc_channels), rc_sub, &rc);
		/* get a local copy of attitude */
		orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

		multirotor_control_attitude(&rc, &att, &state);

		/* run at approximately 200 Hz */
		usleep(5000);
		counter++;
	}

	printf("[ardrone_control] ending now...\r\n");
	fflush(stdout);
	return 0;
}

