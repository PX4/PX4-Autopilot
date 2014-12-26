/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
#include <systemlib/err.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>

#include <systemlib/systemlib.h>

#include "ardrone_motor_control.h"

__EXPORT int ardrone_interface_main(int argc, char *argv[]);


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int ardrone_interface_task;		/**< Handle of deamon task / thread */
static int ardrone_write;			/**< UART to write AR.Drone commands to */

/**
 * Mainloop of ardrone_interface.
 */
int ardrone_interface_thread_main(int argc, char *argv[]);

/**
 * Open the UART connected to the motor controllers
 */
static int ardrone_open_uart(char *uart_name, struct termios *uart_config_original);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
	warnx("usage: {start|stop|status} [-d <UART>]\n\n");
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
			warnx("already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		ardrone_interface_task = task_spawn_cmd("ardrone_interface",
						    SCHED_DEFAULT,
						    SCHED_PRIORITY_MAX - 15,
						    1100,
						    ardrone_interface_thread_main,
						    (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("running");
		} else {
			warnx("not started");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static int ardrone_open_uart(char *uart_name, struct termios *uart_config_original)
{
	/* baud rate */
	int speed = B115200;
	int uart;

	/* open uart */
	uart = open(uart_name, O_RDWR | O_NOCTTY);

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
		warnx("ERR: TCGETATTR %s: %d", uart_name, termios_state);
		close(uart);
		return -1;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(uart, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		warnx("ERR: cfsetispeed %s: %d", uart_name, termios_state);
		close(uart);
		return -1;
	}


	if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
		warnx("ERR: tcsetattr: %s", uart_name);
		close(uart);
		return -1;
	}

	return uart;
}

int ardrone_interface_thread_main(int argc, char *argv[])
{
	thread_running = true;

	char *device = "/dev/ttyS1";

	/* File descriptors */
	int gpios;

	char *commandline_usage = "\tusage: ardrone_interface start|status|stop [-t for motor test (10%% thrust)]\n";

	bool motor_test_mode = false;
	int test_motor = -1;

	/* read commandline arguments */
	for (int i = 0; i < argc && argv[i]; i++) {
		if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--test") == 0) {
			motor_test_mode = true;
		}

		if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--motor") == 0) {
			if (i+1 < argc) {
				int motor = atoi(argv[i+1]);
				if (motor > 0 && motor < 5) {
					test_motor = motor;
				} else {
					thread_running = false;
					errx(1, "supply a motor # between 1 and 4. Example: -m 1\n %s", commandline_usage);
				}
			} else {
				thread_running = false;
				errx(1, "missing parameter to -m 1..4\n %s", commandline_usage);
			}
		}
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) { //device set
			if (argc > i + 1) {
				device = argv[i + 1];

			} else {
				thread_running = false;
				errx(1, "missing parameter to -m 1..4\n %s", commandline_usage);
			}
		}
	}

	struct termios uart_config_original;

	if (motor_test_mode) {
		warnx("setting 10 %% thrust.\n");
	}

	/* Led animation */
	int counter = 0;
	int led_counter = 0;

	/* declare and safely initialize all structs */
	struct actuator_controls_s actuator_controls;
	memset(&actuator_controls, 0, sizeof(actuator_controls));
	struct actuator_armed_s armed;
	//XXX is this necessairy?
	armed.armed = false;

	/* subscribe to attitude, motor setpoints and system state */
	int actuator_controls_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	/* enable UART, writes potentially an empty buffer, but multiplexing is disabled */
	ardrone_write = ardrone_open_uart(device, &uart_config_original);

	/* initialize multiplexing, deactivate all outputs - must happen after UART open to claim GPIOs on PX4FMU */
	gpios = ar_multiplexing_init();

	if (ardrone_write < 0) {
		warnx("No UART, exiting.");
		thread_running = false;
		exit(ERROR);
	}

	/* initialize motors */
	if (OK != ar_init_motors(ardrone_write, gpios)) {
		close(ardrone_write);
		warnx("motor init fail");
		thread_running = false;
		exit(ERROR);
	}

	ardrone_write_motor_commands(ardrone_write, 0, 0, 0, 0);


	// XXX Re-done initialization to make sure it is accepted by the motors
	// XXX should be removed after more testing, but no harm

	/* close uarts */
	close(ardrone_write);

	/* enable UART, writes potentially an empty buffer, but multiplexing is disabled */
	ardrone_write = ardrone_open_uart(device, &uart_config_original);

	/* initialize multiplexing, deactivate all outputs - must happen after UART open to claim GPIOs on PX4FMU */
	gpios = ar_multiplexing_init();

	if (ardrone_write < 0) {
		warnx("write fail");
		thread_running = false;
		exit(ERROR);
	}

	/* initialize motors */
	if (OK != ar_init_motors(ardrone_write, gpios)) {
		close(ardrone_write);
		warnx("motor init fail");
		thread_running = false;
		exit(ERROR);
	}

	while (!thread_should_exit) {

		if (motor_test_mode) {
			/* set motors to idle speed */
			if (test_motor > 0 && test_motor < 5) {
				int motors[4] = {0, 0, 0, 0};
				motors[test_motor - 1] = 10;
				ardrone_write_motor_commands(ardrone_write, motors[0], motors[1], motors[2], motors[3]);
			} else {
				ardrone_write_motor_commands(ardrone_write, 10, 10, 10, 10);
			}

		} else {
			/* MAIN OPERATION MODE */

			/* get a local copy of the actuator controls */
			orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_controls_sub, &actuator_controls);
			orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
			
			/* for now only spin if armed and immediately shut down
			 * if in failsafe
			 */
			if (armed.armed && !armed.lockdown) {
				ardrone_mixing_and_output(ardrone_write, &actuator_controls);

			} else {
				/* Silently lock down motor speeds to zero */
				ardrone_write_motor_commands(ardrone_write, 0, 0, 0, 0);
			}
		}

		if (counter % 24 == 0) {
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
		usleep(4500);

		counter++;
	}

	/* restore old UART config */
	int termios_state;

	if ((termios_state = tcsetattr(ardrone_write, TCSANOW, &uart_config_original)) < 0) {
		warnx("ERR: tcsetattr");
	}

	/* close uarts */
	close(ardrone_write);
	ar_multiplexing_deinit(gpios);

	fflush(stdout);

	thread_running = false;

	return OK;
}

