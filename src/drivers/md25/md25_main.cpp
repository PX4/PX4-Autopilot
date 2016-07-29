/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @ file md25.cpp
 *
 * Driver for MD25 I2C Motor Driver
 *
 * references:
 * http://www.robot-electronics.co.uk/htm/md25tech.htm
 * http://www.robot-electronics.co.uk/files/rpi_md25.c
 *
 */

#include <px4_config.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>

#include <arch/board/board.h>
#include "md25.hpp"

static bool thread_should_exit = false;     /**< Deamon exit flag */
static bool thread_running = false;     /**< Deamon status flag */
static int deamon_task;             /**< Handle of deamon task / thread */

/**
 * Deamon management function.
 */
extern "C" __EXPORT int md25_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int md25_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: md25 {start|stop|read|status|search|test|change_address}\n\n");
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
int md25_main(int argc, char *argv[])
{

	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("md25 already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("md25",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 10,
						 2048,
						 md25_thread_main,
						 (const char **)argv);
		exit(0);
	}

	if (!strcmp(argv[1], "test")) {

		if (argc < 4) {
			printf("usage: md25 test bus address\n");
			exit(0);
		}

		const char *deviceName = "/dev/md25";

		uint8_t bus = strtoul(argv[2], nullptr, 0);

		uint8_t address = strtoul(argv[3], nullptr, 0);

		md25Test(deviceName, bus, address);

		exit(0);
	}

	if (!strcmp(argv[1], "sine")) {

		if (argc < 6) {
			printf("usage: md25 sine bus address amp freq\n");
			exit(0);
		}

		const char *deviceName = "/dev/md25";

		uint8_t bus = strtoul(argv[2], nullptr, 0);

		uint8_t address = strtoul(argv[3], nullptr, 0);

		float amplitude = atof(argv[4]);

		float frequency = atof(argv[5]);

		md25Sine(deviceName, bus, address, amplitude, frequency);

		exit(0);
	}

	if (!strcmp(argv[1], "probe")) {
		if (argc < 4) {
			printf("usage: md25 probe bus address\n");
			exit(0);
		}

		const char *deviceName = "/dev/md25";

		uint8_t bus = strtoul(argv[2], nullptr, 0);

		uint8_t address = strtoul(argv[3], nullptr, 0);

		MD25 md25(deviceName, bus, address);

		int ret = md25.probe();

		if (ret == OK) {
			printf("MD25 found on bus %d at address 0x%X\n", bus, md25.get_address());

		} else {
			printf("MD25 not found on bus %d\n", bus);
		}

		exit(0);
	}

	if (!strcmp(argv[1], "read")) {
		if (argc < 4) {
			printf("usage: md25 read bus address\n");
			exit(0);
		}

		const char *deviceName = "/dev/md25";

		uint8_t bus = strtoul(argv[2], nullptr, 0);

		uint8_t address = strtoul(argv[3], nullptr, 0);

		MD25 md25(deviceName, bus, address);

		// print status
		char buf[400];
		md25.status(buf, sizeof(buf));
		printf("%s\n", buf);

		exit(0);
	}


	if (!strcmp(argv[1], "search")) {
		if (argc < 3) {
			printf("usage: md25 search bus\n");
			exit(0);
		}

		const char *deviceName = "/dev/md25";

		uint8_t bus = strtoul(argv[2], nullptr, 0);

		uint8_t address = strtoul(argv[3], nullptr, 0);

		MD25 md25(deviceName, bus, address);

		md25.search();

		exit(0);
	}

	if (!strcmp(argv[1], "change_address")) {
		if (argc < 5) {
			printf("usage: md25 change_address bus old_address new_address\n");
			exit(0);
		}

		const char *deviceName = "/dev/md25";

		uint8_t bus = strtoul(argv[2], nullptr, 0);

		uint8_t old_address = strtoul(argv[3], nullptr, 0);

		uint8_t new_address = strtoul(argv[4], nullptr, 0);

		MD25 md25(deviceName, bus, old_address);

		int ret = md25.setDeviceAddress(new_address);

		if (ret == OK)  {
			printf("MD25 new address set to 0x%X\n", new_address);

		} else {
			printf("MD25 failed to set address to 0x%X\n", new_address);
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tmd25 app is running\n");

		} else {
			printf("\tmd25 app not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int md25_thread_main(int argc, char *argv[])
{
	printf("[MD25] starting\n");

	if (argc < 5) {
		// extra md25 in arg list since this is a thread
		printf("usage: md25 start bus address\n");
		exit(0);
	}

	const char *deviceName = "/dev/md25";

	uint8_t bus = strtoul(argv[3], nullptr, 0);

	uint8_t address = strtoul(argv[4], nullptr, 0);

	// start
	MD25 md25(deviceName, bus, address);

	thread_running = true;

	// loop
	while (!thread_should_exit) {
		md25.update();
	}

	// exit
	printf("[MD25] exiting.\n");
	thread_running = false;
	return 0;
}

// vi:noet:smarttab:autoindent:ts=4:sw=4:tw=78
