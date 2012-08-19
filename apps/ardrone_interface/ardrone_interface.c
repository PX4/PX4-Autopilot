/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

/**
 * @file ardrone_interface.c
 * Implementation of AR.Drone 1.0 / 2.0 motor control interface.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>
#include <time.h>
#include <sys/prctl.h>
#include <arch/board/up_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>

#include "ardrone_motor_control.h"

__EXPORT int ardrone_interface_main(int argc, char *argv[]);


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int ardrone_interface_task;		/**< Handle of deamon task / thread */

/**
 * Mainloop of ardrone_interface.
 */
int ardrone_interface_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: deamon {start|stop|status} [-p <additional params>]\n\n");
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
int ardrone_interface_main(int argc, char *argv[])
{
		if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("ardrone_interface already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		ardrone_interface_task = task_create("ardrone_interface", SCHED_PRIORITY_MAX - 15, 4096, ardrone_interface_thread_main, (argv) ? (const char **)&argv[2] : (const char **)NULL);
		thread_running = true;
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tardrone_interface is running\n");
		} else {
			printf("\tardrone_interface not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int ardrone_interface_thread_main(int argc, char *argv[])
{
	/* welcome user */
	printf("[ardrone_interface] Control started, taking over motors\n");

	/* default values for arguments */
	char *ardrone_uart_name = "/dev/ttyS1";

	/* File descriptors */
	int ardrone_write;
	int gpios;

	enum {
		CONTROL_MODE_RATES = 0,
		CONTROL_MODE_ATTITUDE = 1,
	} control_mode = CONTROL_MODE_ATTITUDE;

	char *commandline_usage = "\tusage: ardrone_interface -d ardrone-devicename -m mode\n\tmodes are:\n\t\trates\n\t\tattitude\n";

	bool motor_test_mode = false;

	/* read commandline arguments */
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) { //ardrone set
			if (argc > i + 1) {
				ardrone_uart_name = argv[i + 1];
			} else {
				printf(commandline_usage);
				return ERROR;
			}

		} else if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--mode") == 0) {
			if (argc > i + 1) {
				if (strcmp(argv[i + 1], "rates") == 0) {
					control_mode = CONTROL_MODE_RATES;

				} else if (strcmp(argv[i + 1], "attitude") == 0) {
					control_mode = CONTROL_MODE_ATTITUDE;

				} else {
					printf(commandline_usage);
					return ERROR;
				}

			} else {
				printf(commandline_usage);
				return ERROR;
			}

		} else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--test") == 0) {
			motor_test_mode = true;
		}
	}

	/* open uarts */
	printf("[ardrone_interface] AR.Drone UART is %s\n", ardrone_uart_name);
	ardrone_write = open(ardrone_uart_name, O_RDWR | O_NOCTTY | O_NDELAY);
	if (ardrone_write < 0) {
		fprintf(stderr, "[ardrone_interface] Failed opening AR.Drone UART, exiting.\n");
		exit(ERROR);
	}

	/* initialize motors */
	if (OK != ar_init_motors(ardrone_write, &gpios)) {
		close(ardrone_write);
		fprintf(stderr, "[ardrone_interface] Failed initializing AR.Drone motors, exiting.\n");
		exit(ERROR);
	}

	/* Led animation */
	int counter = 0;
	int led_counter = 0;

	/* declare and safely initialize all structs */
	struct vehicle_status_s state;
	memset(&state, 0, sizeof(state));
	struct actuator_controls_s actuator_controls;
	memset(&actuator_controls, 0, sizeof(actuator_controls));

	/* subscribe to attitude, motor setpoints and system state */
	int actuator_controls_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));

	while (!thread_should_exit) {

		/* get a local copy of the vehicle state */
		orb_copy(ORB_ID(vehicle_status), state_sub, &state);
		/* get a local copy of the actuator controls */
		orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_controls_sub, &actuator_controls);
		// if ..
			ardrone_mixing_and_output(ardrone_write, &actuator_controls, motor_test_mode);
		// } else {
		// 	/* Silently lock down motor speeds to zero */
		// 	ardrone_write_motor_commands(ardrone_write, 0, 0, 0, 0);
		// }

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

		/* run at approximately 50 Hz */
		usleep(20000);

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

	printf("[ardrone_interface] ending now...\n");
	fflush(stdout);

	thread_running = false;

	return OK;
}

