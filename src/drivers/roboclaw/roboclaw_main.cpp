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
 * @ file roboclaw_main.cpp
 *
 * RoboClaw Motor Driver
 *
 * references:
 * http://downloads.orionrobotics.com/downloads/datasheets/motor_controller_robo_claw_R0401.pdf
 *
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>

#include <arch/board/board.h>
#include "RoboClaw.hpp"

static bool thread_should_exit = false;     /**< Deamon exit flag */
static bool thread_running = false;     /**< Deamon status flag */
static int deamon_task;             /**< Handle of deamon task / thread */

/**
 * Deamon management function.
 */
extern "C" __EXPORT int roboclaw_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int roboclaw_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage();

static void usage()
{
	fprintf(stderr, "usage: roboclaw "
			"{start|stop|status|test}\n\n");
}

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

	if (argc < 1)
		usage();

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("roboclaw already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd("roboclaw",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 10,
					 2048,
					 roboclaw_thread_main,
					 (char * const *)argv);
		exit(0);

	} else if (!strcmp(argv[1], "test")) {

		const char * deviceName = "/dev/ttyS2";
		uint8_t address = 128;
		uint16_t pulsesPerRev = 1200;

		if (argc == 2) {
			printf("testing with default settings\n");
		} else if (argc != 4) {
			printf("usage: roboclaw test device address pulses_per_rev\n");
			exit(-1);
		} else {
			deviceName = argv[2];
			address = strtoul(argv[3], nullptr, 0);
			pulsesPerRev = strtoul(argv[4], nullptr, 0);
		}

		printf("device:\t%s\taddress:\t%d\tpulses per rev:\t%ld\n",
			deviceName, address, pulsesPerRev);

		roboclawTest(deviceName, address, pulsesPerRev);
		thread_should_exit = true;
		exit(0);

	} else if (!strcmp(argv[1], "stop")) {

		thread_should_exit = true;
		exit(0);

	} else if (!strcmp(argv[1], "status")) {

		if (thread_running) {
			printf("\troboclaw app is running\n");

		} else {
			printf("\troboclaw app not started\n");
		}
		exit(0);
	}

	usage();
	exit(1);
}

int roboclaw_thread_main(int argc, char *argv[])
{
	printf("[roboclaw] starting\n");

	// skip parent process args
	argc -=2;
	argv +=2;

	if (argc < 3) {
		printf("usage: roboclaw start device address\n");
		return -1;
	}

	const char *deviceName = argv[1];
	uint8_t address = strtoul(argv[2], nullptr, 0);
	uint16_t pulsesPerRev = strtoul(argv[3], nullptr, 0);

	printf("device:\t%s\taddress:\t%d\tpulses per rev:\t%ld\n",
			deviceName, address, pulsesPerRev);

	// start
	RoboClaw roboclaw(deviceName, address, pulsesPerRev);

	thread_running = true;

	// loop
	while (!thread_should_exit) {
		roboclaw.update();
	}

	// exit
	printf("[roboclaw] exiting.\n");
	thread_running = false;
	return 0;
}

// vi:noet:smarttab:autoindent:ts=4:sw=4:tw=78
