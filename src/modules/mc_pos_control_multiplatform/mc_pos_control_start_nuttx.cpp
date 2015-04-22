/****************************************************************************
 *
 *   Copyright (C) 2013 - 2015 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_m_start_nuttx.cpp
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */
#include <string.h>
#include <cstdlib>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

extern bool thread_running;
int daemon_task;             /**< Handle of deamon task / thread */
namespace px4
{
bool task_should_exit = false;
}
using namespace px4;

extern int main(int argc, char **argv);

extern "C" __EXPORT int mc_pos_control_m_main(int argc, char *argv[]);
int mc_pos_control_m_main(int argc, char *argv[])
{
	if (argc < 2) {
		errx(1, "usage: mc_pos_control_m {start|stop|status}");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			exit(0);
		}

		task_should_exit = false;

		daemon_task = task_spawn_cmd("mc_pos_control_m",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2500,
				       main,
					(argv) ? (char* const*)&argv[2] : (char* const*)NULL);

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
