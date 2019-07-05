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
 * @file roboclaw_main.cpp
 *
 * RoboClaw Motor Driver
 *
 * references:
 * http://downloads.ionmc.com/docs/roboclaw_user_manual.pdf
 *
 */

#include <px4_config.h>
#include <px4_log.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <parameters/param.h>

#include <arch/board/board.h>
#include "RoboClaw.hpp"

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
	PX4_INFO("usage: roboclaw {start|stop|status|test}");
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

	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("roboclaw already running\n");
			/* this is not an error */
			return 0;
		}

		RoboClaw::taskShouldExit = false;
		deamon_task = px4_task_spawn_cmd("roboclaw",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 10,
						 1500,
						 roboclaw_thread_main,
						 (char *const *)argv);
		return 0;

	} else if (!strcmp(argv[1], "stop")) {

		RoboClaw::taskShouldExit = true;
		return 0;

	} else if (!strcmp(argv[1], "status")) {

		if (thread_running) {
			printf("\troboclaw app is running\n");

		} else {
			printf("\troboclaw app not started\n");
		}

		return 0;
	}

	usage();
	return 1;
}

int roboclaw_thread_main(int argc, char *argv[])
{
	printf("[roboclaw] starting\n");

	// skip parent process args
	argc -= 2;
	argv += 2;

	if (argc < 2) {
		printf("usage: roboclaw start <device>\n");
		return -1;
	}

	const char *deviceName = argv[1];

	// start
	RoboClaw roboclaw(deviceName);

	thread_running = true;

	roboclaw.taskMain();

	// exit
	printf("[roboclaw] exiting.\n");
	thread_running = false;
	return 0;
}
