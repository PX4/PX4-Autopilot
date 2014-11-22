/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
 *   Author: Stefan Rado <px4@sradonia.net>
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
 * @file frsky_telemetry.c
 * @author Stefan Rado <px4@sradonia.net>
 *
 * FrSky telemetry implementation.
 *
 * This daemon emulates an FrSky sensor hub by periodically sending data
 * packets to an attached FrSky receiver.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <termios.h>

#include "frsky_data.h"


/* thread state */
static volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
static int frsky_task;

/* functions */
static int frsky_open_uart(const char *uart_name, struct termios *uart_config_original);
static void usage(void);
static int frsky_telemetry_thread_main(int argc, char *argv[]);
__EXPORT int frsky_telemetry_main(int argc, char *argv[]);


/**
 * Opens the UART device and sets all required serial parameters.
 */
static int frsky_open_uart(const char *uart_name, struct termios *uart_config_original)
{
	/* Open UART */
	const int uart = open(uart_name, O_WRONLY | O_NOCTTY);

	if (uart < 0) {
		err(1, "Error opening port: %s", uart_name);
	}

	/* Back up the original UART configuration to restore it after exit */
	int termios_state;
	if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
		warnx("ERR: tcgetattr%s: %d\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	/* Fill the struct for the new configuration */
	struct termios uart_config;
	tcgetattr(uart, &uart_config);

	/* Disable output post-processing */
	uart_config.c_oflag &= ~OPOST;

	/* Set baud rate */
	static const speed_t speed = B9600;

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		warnx("ERR: %s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
		warnx("ERR: %s (tcsetattr)\n", uart_name);
		close(uart);
		return -1;
	}

	return uart;
}

/**
 * Print command usage information
 */
static void usage()
{
	fprintf(stderr,
			"usage: frsky_telemetry start [-d <devicename>]\n"
			"       frsky_telemetry stop\n"
			"       frsky_telemetry status\n");
	exit(1);
}

/**
 * The daemon thread.
 */
static int frsky_telemetry_thread_main(int argc, char *argv[])
{
	/* Default values for arguments */
	char *device_name = "/dev/ttyS1"; /* USART2 */

	/* Work around some stupidity in task_create's argv handling */
	argc -= 2;
	argv += 2;

	int ch;
	while ((ch = getopt(argc, argv, "d:")) != EOF) {
		switch (ch) {
		case 'd':
			device_name = optarg;
			break;

		default:
			usage();
			break;
		}
	}

	/* Open UART */
	struct termios uart_config_original;
	const int uart = frsky_open_uart(device_name, &uart_config_original);

	if (uart < 0)
		err(1, "could not open %s", device_name);

	/* Subscribe to topics */
	frsky_init();

	thread_running = true;

	/* Main thread loop */
	unsigned int iteration = 0;
	while (!thread_should_exit) {

		/* Sleep 200 ms */
		usleep(200000);

		/* Send frame 1 (every 200ms): acceleration values, altitude (vario), temperatures, current & voltages, RPM */
		frsky_send_frame1(uart);

		/* Send frame 2 (every 1000ms): course, latitude, longitude, speed, altitude (GPS), fuel level */
		if (iteration % 5 == 0)
		{
			frsky_send_frame2(uart);
		}

		/* Send frame 3 (every 5000ms): date, time */
		if (iteration % 25 == 0)
		{
			frsky_send_frame3(uart);

			iteration = 0;
		}

		iteration++;
	}

	/* Reset the UART flags to original state */
	tcsetattr(uart, TCSANOW, &uart_config_original);
	close(uart);

	thread_running = false;
	return 0;
}

/**
 * The main command function.
 * Processes command line arguments and starts the daemon.
 */
int frsky_telemetry_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("missing command");
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		/* this is not an error */
		if (thread_running)
			errx(0, "frsky_telemetry already running");

		thread_should_exit = false;
		frsky_task = task_spawn_cmd("frsky_telemetry",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  frsky_telemetry_thread_main,
					  (const char **)argv);

		while (!thread_running) {
			usleep(200);
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {

		/* this is not an error */
		if (!thread_running)
			errx(0, "frsky_telemetry already stopped");

		thread_should_exit = true;

		while (thread_running) {
			usleep(200000);
			warnx(".");
		}

		warnx("terminated.");
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	usage();
	/* not getting here */
	return 0;
}
