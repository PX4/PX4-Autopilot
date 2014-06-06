/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.		 
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
 * @file matlab_csv_serial_main.c
 *
 * Matlab CSV / ASCII format interface at 921600 baud, 8 data bits,
 * 1 stop bit, no parity
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <poll.h>

__EXPORT int matlab_csv_serial_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

int matlab_csv_serial_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_spawn_cmd().
 */
int matlab_csv_serial_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start"))
	{
		if (thread_running)
		{
			printf("flow position estimator already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("matlab_csv_serial",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 5,
					 2000,
					 matlab_csv_serial_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop"))
	{
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status"))
	{
		if (thread_running) {
			warnx("running");
		} else {
			warnx("stopped");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int matlab_csv_serial_thread_main(int argc, char *argv[])
{
	/* welcome user */
	thread_running = true;

	if (argc < 2) {
		errx(1, "need a serial port name as argument");
	}

	int serial_fd = open(argv[1], O_RDWR | O_NOCTTY);

	unsigned speed = 921600;

	if (serial_fd < 0) {
		err(1, "failed to open port: %s", argv[1]);
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;
	bool is_usb = false;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(_uart_fd, uart_config_original)) < 0) {
		warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
		close(_uart_fd);
		return -1;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(_uart_fd, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* USB serial is indicated by /dev/ttyACM0*/
	if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			warnx("ERR SET BAUD %s: %d\n", argv[1], termios_state);
			close(serial_fd);
			return -1;
		}

	}

	if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR SET CONF %s\n", argv[1]);
		close(serial_fd);
		return -1;
	}

	/* subscribe to vehicle status, attitude, sensors and flow*/
	struct accel_report accel0;
	struct accel_report accel1;
	struct gyro_report gyro0;
	struct gyro_report gyro1;

	/* subscribe to parameter changes */
	int accel0_sub = orb_subscribe(ORB_ID(sensor_accel));
	int accel1_sub;
	int gyro0_sub;
	int gyro1_sub;

	while (!thread_should_exit)
	{

		/*This runs at the rate of the sensors */
		struct pollfd fds[] = {
				{ .fd = accel0_sub, .events = POLLIN },
				{ .fd = accel1_sub,   .events = POLLIN },
				{ .fd = gyro0_sub,   .events = POLLIN },
				{ .fd = gyro1_sub,   .events = POLLIN }
		};

		/* wait for a sensor update, check for exit condition every 500 ms */
		int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), 500);

		if (ret < 0)
		{
			/* poll error, count it in perf */
			perf_count(mc_err_perf);

		}
		else if (ret == 0)
		{
			/* no return value, ignore */
			warnx("no sensor data");
		}
		else
		{

			/* parameter update available? */
			if (fds[0].revents & POLLIN)
			{
				orb_copy(ORB_ID(sensor_accel), accel0_sub, &accel0);

				// write out on accel 0, but collect for all other sensors as they have updates
				vdprintf(serial_fd, "%llu,%d\n", accel0.timestamp, (int)accel0.x_raw);
			}

		}
	}

	warnx("exiting");
	thread_running = false;

	fflush(stdout);
	return 0;
}


