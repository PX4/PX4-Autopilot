/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: 	James Goppert
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
 * @file roboclaw.cpp
 * RoboClaw Driver
 *
 * @author James Goppert
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h> // strcmp

#include <systemlib/systemlib.h> // SCHED_DEFAULT
#include <systemlib/err.h> // errx

#include "RoboClawDevice.h"
#include "RoboClaw.h"

volatile static bool thread_should_exit = false; // Daemon exit flag
volatile static bool thread_running = false; // Daemon status flag
volatile static bool test_thread_running = false; // Daemon status flag
volatile static int daemon_task; // Handle of deamon task / thread

/**
 * Daemon management function.
 */
extern "C" __EXPORT int roboclaw_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int roboclaw_thread_main(int argc, char *argv[]);

/**
 * Test
 */
int roboclaw_test_main(int argc, char *argv[]);

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int roboclaw_main(int argc, char *argv[])
{
	const char *command = nullptr;

	// parse
	if (argc == 1) {
		command = "help";

	} else if (argc > 1) {
		command = argv[1];

	} else {
		errx(1, "wrong number of args");
	}

	// handle
	if (!strcmp(command, "help")) {
		warnx("usage: (start|stop|status|test|reset)");
		return OK;

	} else if (!strcmp(command, "start")) {
		if (thread_running) {
			warnx("already running");
			return 0;
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("roboclaw",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_MAX - 30,
					     4096,
					     roboclaw_thread_main,
					     (argv) ? (const char **)&argv[2] : (const char **)NULL);
		return 0;

	} else if (!strcmp(command, "stop")) {
		thread_should_exit = true;
		int loop_count = 0;

		while (thread_running == true) {
			usleep(1000000);

			if (loop_count++ > 5) {
				warnx("forcing deletion");
				task_delete(daemon_task);
			}

			warnx("waiting for process to exit");
		}

		return 0;

	} else if (!strcmp(command, "status")) {
		if (thread_running) {
			warnx("is running");
			return 0;

		} else {
			warnx("not started");
			return -1;
		}

	} else if (!strcmp(command, "test")) {
		if (thread_running) {
			warnx("must stop first");
			return -1;

		} else if (test_thread_running) {
			warnx("test already running");

		} else {
			daemon_task = task_spawn_cmd("roboclaw_test",
						     SCHED_DEFAULT,
						     SCHED_PRIORITY_MAX - 30,
						     4096,
						     roboclaw_test_main,
						     (argv) ? (const char **)&argv[2] :
						     (const char **)NULL);
			return 0;
		}

	} else if (!strcmp(command, "reset")) {
		if (thread_running) {
			warnx("not implemented");
			return 0;

		} else {
			warnx("not started");
			return -1;
		}

	} else {
		errx(1, "unknown command: %s", command);
	}

	return OK;
}

int roboclaw_thread_main(int argc, char *argv[])
{
	// defaults
	const char *port = "/dev/ttyS6";
	uint8_t address = 128;
	uint32_t timeout = 1000; // 1 second
	bool doack = false; // do ack for writes

	// parse
	if (argc == 3) {
		port = argv[1];
		address = strtoul(argv[2], nullptr, 0);

	} else if (argc != 1) {
		errx(1, "wrong number of args");
	}

	warnx("starting");
	thread_running = true;
	RoboClawDevice roboclaw(port, address, timeout, doack);

	while (!thread_should_exit) {
		roboclaw.update();
	}

	warnx("exiting.");
	thread_running = false;
	return 0;
}

int roboclaw_test_main(int argc, char *argv[])
{
	test_thread_running = true;
	// defaults
	const char *port = "/dev/ttyS6";
	uint8_t address = 128;
	//bool doack = false; // do ack for writes

	// parse
	if (argc == 3) {
		port = argv[1];
		address = strtoul(argv[2], nullptr, 0);

	} else if (argc != 1) {
		errx(1, "wrong number of args");
	}

	printf("starting new test.\n");

	// open port
	int uart = open(port, O_RDWR | O_NONBLOCK | O_NOCTTY);

	if (uart < 0) {
		errx(1, "failed to open port: %s", port);
		return 0;
	}

	// setup uart
	warnx("setting up uart");
	struct termios uart_config;
	int ret = tcgetattr(uart, &uart_config);

	if (ret < 0) { errx(1, "failed to get attr"); }

	uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	ret = cfsetispeed(&uart_config, B38400);

	if (ret < 0) { errx(1, "failed to set input speed"); }

	ret = cfsetospeed(&uart_config, B38400);

	if (ret < 0) { errx(1, "failed to set output speed"); }

	ret = tcsetattr(uart, TCSANOW, &uart_config);

	if (ret < 0) { errx(1, "failed to set attr"); }

	// clear old data
	tcflush(uart, TCIOFLUSH);

	// message data
	uint8_t get_version = 21;
	char msg[200];
	char buf[10];

	// request version
	printf("requesting version\n");
	write(uart, &address, 1);
	write(uart, &get_version, 1);

	// read version with poll
	printf("reading version with poll\n");
	struct pollfd uartPoll;
	uint32_t tout = 1000;
	uartPoll.fd = uart;
	uartPoll.events = POLLIN;
	msg[0] = '\0';

	while (true) {
		int pollrc = poll(&uartPoll, 1, tout);

		if (pollrc < 1) { break; }

		ret = ::read(uart, buf, sizeof(buf));

		if (ret < 1) { break; }

		strncat(msg, (const char *)buf, ret);
	}

	printf("poll msg: %s\n", msg);

	// request version
	printf("requesting version\n");
	write(uart, &address, 1);
	write(uart, &get_version, 1);

	// read version w/o poll
	printf("reading version w/o poll\n");
	msg[0] = '\0';

	for (int i = 0; i < 10000; i++) {
		ret = ::read(uart, buf, sizeof(buf));

		if (ret < 1) {
			continue;
		}

		strncat(msg, (const char *)buf, ret);
	}

	printf("no poll msg: %s\n", msg);

	// close uart
	close(uart);
	test_thread_running = false;
	return 0;
}

// vi:noet:smarttab:autoindent:ts=4:sw=4:tw=78
