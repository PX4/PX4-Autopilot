/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file load_mon.c
 *
 * @author Jonathan Challinger <jonathan@3drobotics.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <poll.h>

#include <px4_config.h>

#include <drivers/drv_hrt.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/cpuload.h>

#include <uORB/uORB.h>
#include <uORB/topics/cpuload.h>

#define LOAD_MON_INTERVAL_SEC 1.0f
#define LOAD_MON_INTERVAL_US (LOAD_MON_INTERVAL_SEC*1.0e6f)

extern "C" __EXPORT int load_mon_main(int argc, char *argv[]);
int load_mon_thread(int argc, char *argv[]);

static const char *module_name = "load_mon";
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */


extern struct system_load_s system_load;
struct cpuload_s cpuload;
static orb_advert_t cpuload_publish_handle = nullptr;

/**
 * Mainloop of load_mon.
 */
int load_mon_thread(int argc, char *argv[])
{
	warnx("[%s] starting\n", module_name);
	thread_running = true;

	hrt_abstime last_idle_time = system_load.tasks[0].total_runtime;

	while (!thread_should_exit) {
		usleep(LOAD_MON_INTERVAL_US);

		/* compute system load */
		uint64_t interval_idletime = system_load.tasks[0].total_runtime - last_idle_time;
		last_idle_time = system_load.tasks[0].total_runtime;

		cpuload.timestamp = hrt_absolute_time();
		cpuload.load = 1.0f - (float)interval_idletime / LOAD_MON_INTERVAL_US;

		if (cpuload_publish_handle == nullptr) {
			cpuload_publish_handle = orb_advertise(ORB_ID(cpuload), &cpuload);
		} else {
			orb_publish(ORB_ID(cpuload), cpuload_publish_handle, &cpuload);
		}
	}

	warnx("[%s] exiting.\n", module_name);

	thread_running = false;

	return 0;
}

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: %s {start|stop|status} [-p <additional params>]\n\n", module_name);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int load_mon_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd(module_name,
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 load_mon_thread,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

