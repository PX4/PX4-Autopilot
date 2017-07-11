/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file iekf_main.cpp
 *
 * Invariant Extended Kalman Filter
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <fcntl.h>
#include <px4_posix.h>

#include "IEKF.hpp"

static IEKF *est = nullptr;
static int deamon_task = 0;             /**< Handle of deamon task / thread */
volatile bool running = false;

/**
 * Deamon management function.
 */
extern "C" __EXPORT int iekf_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int iekf_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static int usage(const char *reason);

static int
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: iekf {start|stop|status} [-p <additional params>]\n\n");
	return 1;
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int iekf_main(int argc, char *argv[])
{

	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (est != nullptr) {
			PX4_INFO("already running");
			/* this is not an error */
			return 0;
		}

		deamon_task = px4_task_spawn_cmd("iekf",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 10000,
						 iekf_thread_main,
						 (argv && argc > 2) ? (char *const *) &argv[2] : (char *const *) nullptr);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (est != nullptr) {
			PX4_INFO("stop requested");
			running = false;

		} else {
			PX4_INFO("not started");
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (est != nullptr) {
			PX4_INFO("is running");

		} else {
			PX4_INFO("not started");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int iekf_thread_main(int argc, char *argv[])
{
	PX4_INFO("started");

	if (est == nullptr) {
		est = new IEKF();

	} else {

		PX4_INFO("already running");
		return -1;
	}

	running = true;

	while (running) {
		// uses polling
		est->update();
	}

	delete est;
	est = nullptr;
	PX4_INFO("stopped");
	return 0;
}
