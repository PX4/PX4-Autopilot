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
 * @file qshell_start_qurt.cpp
 * Listener for shell commands from posix
 *
 * @author Nicolas de Palezieux <ndepal@gmail.com>
 */

#include "qshell.h"
#include <px4_log.h>
#include <px4_app.h>
#include <px4_tasks.h>
#include <stdio.h>
#include <string.h>
#include <sched.h>

static int daemon_task;             /* Handle of deamon task / thread */

//using namespace px4;

extern "C" __EXPORT int qshell_main(int argc, char *argv[]);

int qshell_entry(int argc, char **argv)
{
	//px4::init(argc, argv, "qshell");

	PX4_DEBUG("qshell entry.....");
	QShell qshell;
	qshell.main();

	PX4_DEBUG("goodbye");
	return 0;
}

static void usage()
{
	PX4_INFO("usage: qshell {start|stop|status}");
}
int qshell_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (QShell::appState.isRunning()) {
			PX4_INFO("already running");
			/* this is not an error */
			return 0;
		}

		PX4_DEBUG("before starting the qshell_entry task");

		daemon_task = px4_task_spawn_cmd("qshell",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 8192,
						 qshell_entry,
						 (char *const *)argv);

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		QShell::appState.requestExit();
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (QShell::appState.isRunning()) {
			PX4_INFO("is running");

		} else {
			PX4_INFO("not started");
		}

		return 0;
	}

	usage();
	return 1;
}
