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
 * @file publisher_main.cpp
 * Example publisher for ros and px4
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */
#include <string.h>
#include <cstdlib>
#include "publisher_example.h"

static bool thread_running = false;     /**< Deamon status flag */
static int daemon_task;             /**< Handle of deamon task / thread */
namespace px4
{
bool task_should_exit = false;
}
using namespace px4;

PX4_MAIN_FUNCTION(publisher);

extern "C" __EXPORT int publisher_main(int argc, char *argv[])
{
	px4::init(argc, argv, "publisher");

	if (argc < 1) {
		errx(1, "usage: publisher {start|stop|status}");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			exit(0);
		}

		task_should_exit = false;

		daemon_task = task_spawn_cmd("publisher",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2000,
				       publisher_task_main,
					(argv) ? (const char **)&argv[2] : (const char **)NULL);

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		task_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("is running");

		} else {
			warnx("not started");
		}

		exit(0);
	}

	warnx("unrecognized command");
	return 1;
}

PX4_MAIN_FUNCTION(publisher)
{
	warnx("starting");
	PublisherExample p;
	thread_running = true;
	p.main();

	warnx("exiting.");
	thread_running = false;
	return 0;
}
