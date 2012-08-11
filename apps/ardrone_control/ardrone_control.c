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
 * @file Implementation of AR.Drone 1.0 / 2.0 control interface
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
#include "ardrone_control.h"
#include "attitude_control.h"
#include "rate_control.h"
#include "ardrone_motor_control.h"
#include "position_control.h"
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/ardrone_control.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/ardrone_motors_setpoint.h>
#include <uORB/topics/sensor_combined.h>

#include "ardrone_control_helper.h"

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
static void *position_control_loop(void *arg)
{
	struct vehicle_status_s *state = (struct vehicle_status_s *)arg;
	// Set thread name
	prctl(PR_SET_NAME, "ardrone pos ctrl", getpid());

	while (1) {
		if (state->state_machine == SYSTEM_STATE_AUTO) {
//			control_position();   //FIXME TODO XXX
			/* temporary 50 Hz execution */
			usleep(20000);

		} else {
			position_control_thread_started = false;
			break;
		}
	}

	return NULL;
}

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

	/* open uarts */
	printf("[ardrone_control] AR.Drone UART is %s\n", ardrone_uart_name);
	ardrone_write = open(ardrone_uart_name, O_RDWR | O_NOCTTY | O_NDELAY);

	/* initialize motors */
	ar_init_motors(ardrone_write, &gpios);
	int counter = 0;

	/* Led animation */
	int led_counter = 0;

	/* pthread for position control */
	pthread_t position_control_thread;
	position_control_thread_started = false;

	/* structures */
	struct vehicle_status_s state;
	struct vehicle_attitude_s att;
	struct ardrone_control_s ar_control;
	struct manual_control_setpoint_s manual;
	struct sensor_combined_s raw;
	struct ardrone_motors_setpoint_s setpoint;

	/* subscribe to attitude, motor setpoints and system state */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	int setpoint_sub = orb_subscribe(ORB_ID(ardrone_motors_setpoint));

	/* publish AR.Drone motor control state */
	int ardrone_pub = orb_advertise(ORB_ID(ardrone_control), &ar_control);

	while (1) {
		/* get a local copy of the vehicle state */
		orb_copy(ORB_ID(vehicle_status), state_sub, &state);
		/* get a local copy of manual setpoint */
		orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
		/* get a local copy of attitude */
		orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

		if (state.state_machine == SYSTEM_STATE_AUTO) {
			if (false == position_control_thread_started) {
				pthread_attr_t position_control_thread_attr;
				pthread_attr_init(&position_control_thread_attr);
				pthread_attr_setstacksize(&position_control_thread_attr, 2048);
				pthread_create(&position_control_thread, &position_control_thread_attr, position_control_loop, &state);
				position_control_thread_started = true;
			}

			control_attitude(0, 0, 0, 0, &att, &state, ardrone_pub, &ar_control);

			//No check for remote sticks to disarm in auto mode, land/disarm with ground station

		} else if (state.state_machine == SYSTEM_STATE_MANUAL) {
			if (control_mode == CONTROL_MODE_RATES) {
				orb_copy(ORB_ID(sensor_combined), sensor_sub, &raw);
				orb_copy(ORB_ID(ardrone_motors_setpoint), setpoint_sub, &setpoint);
				control_rates(&raw, &setpoint);

			} else if (control_mode == CONTROL_MODE_ATTITUDE) {
				control_attitude(manual.roll, manual.pitch, manual.yaw,
					manual.throttle, &att, &state, ardrone_pub, &ar_control);
			}

		} else {

		}

		if (counter % 30 == 0) {
			if (led_counter == 0) ar_set_leds(ardrone_write, 0, 1, 0, 0, 0, 0, 0 , 0);

			if (led_counter == 1) ar_set_leds(ardrone_write, 1, 1, 0, 0, 0, 0, 0 , 0);

			if (led_counter == 2) ar_set_leds(ardrone_write, 1, 0, 0, 0, 0, 0, 0 , 0);

			if (led_counter == 3) ar_set_leds(ardrone_write, 0, 0, 0, 1, 0, 0, 0 , 0);

			if (led_counter == 4) ar_set_leds(ardrone_write, 0, 0, 1, 1, 0, 0, 0 , 0);

			if (led_counter == 5) ar_set_leds(ardrone_write, 0, 0, 1, 0, 0, 0, 0 , 0);

			if (led_counter == 6) ar_set_leds(ardrone_write, 0, 0, 0, 0, 0, 1, 0 , 0);

			if (led_counter == 7) ar_set_leds(ardrone_write, 0, 0, 0, 0, 1, 1, 0 , 0);

			if (led_counter == 8) ar_set_leds(ardrone_write, 0, 0, 0, 0, 1, 0, 0 , 0);

			if (led_counter == 9) ar_set_leds(ardrone_write, 0, 0, 0, 0, 0, 0, 0 , 1);

			if (led_counter == 10) ar_set_leds(ardrone_write, 0, 0, 0, 0, 0, 0, 1 , 1);

			if (led_counter == 11) ar_set_leds(ardrone_write, 0, 0, 0, 0, 0, 0, 1 , 0);

			led_counter++;

			if (led_counter == 12) led_counter = 0;
		}

		/* run at approximately 200 Hz */
		usleep(5000);

		// This is a hardcore debug code piece to validate
		// the motor interface
		// uint8_t motorSpeedBuf[5] = {1, 2, 3, 4, 5};
		// ar_get_motor_packet(motorSpeedBuf, 20, 20, 20, 20);
		// write(ardrone_write, motorSpeedBuf, 5);
		// usleep(15000);

		counter++;
	}

	/* close uarts */
	close(ardrone_write);
	ar_multiplexing_deinit(gpios);

	printf("[ardrone_control] ending now...\r\n");
	fflush(stdout);
	return 0;
}

