/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file attitude_estimator_ekf2_main.cpp
 *
 * Extended Kalman Filter for Attitude Estimation version 2.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#include <stdio.h>
#include <string.h>
#include <systemlib/err.h>

#include "BlockAttEkf.hpp"

extern "C" __EXPORT int attitude_estimator_ekf2_main(int argc, char *argv[]);

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int attitude_estimator_ekf2_task; /**< Handle of deamon task / thread */

/**
 * Mainloop of attitude_estimator_ekf2.
 */
int attitude_estimator_ekf2_thread_main(int argc, char *argv[]);

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

	fprintf(stderr, "usage: attitude_estimator_ekf2 {start|stop|status} [-p <additional params>]\n\n");
}

int attitude_estimator_ekf2_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		attitude_estimator_ekf2_task = px4_task_spawn_cmd("attitude_estimator_ekf2",
					       SCHED_DEFAULT,
					       SCHED_PRIORITY_MAX - 5,
					       7700,
					       attitude_estimator_ekf2_thread_main,
					       (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("running");
			return 0;

		} else {
			warnx("not started");
			return 1;
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int attitude_estimator_ekf2_thread_main(int argc, char *argv[])
{
	thread_running = true;

	BlockAttEkf est;

	while (!thread_should_exit) {
		est.update();
	};

	thread_running = false;

	return 0;
}
