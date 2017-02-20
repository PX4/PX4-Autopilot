/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file precland_beacon_est_main.cpp
 * Precision land beacon position estimator
 *
 * @author Nicolas de Palezieux <ndepal@gmail.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>					//usleep
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/systemlib.h>	//Scheduler
#include <systemlib/err.h>			//print to console

#include "PreclandBeaconEst.h"


namespace precland_beacon_est
{

//Function prototypes
static int precland_beacon_est_start();
static void precland_beacon_est_stop();

/**
 * land detector app start / stop handling function
 * This makes the land detector module accessible from the nuttx shell
 * @ingroup apps
 */
extern "C" __EXPORT int precland_beacon_est_main(int argc, char *argv[]);

//Private variables
static PreclandBeaconEst *precland_beacon_est_task = nullptr;

/**
* Stop the task, force killing it if it doesn't stop by itself
**/
static void precland_beacon_est_stop()
{
	if (precland_beacon_est_task == nullptr) {
		PX4_WARN("not running");
		return;
	}

	precland_beacon_est_task->stop();

	// Wait for task to die
	int i = 0;

	do {
		/* wait 20ms */
		usleep(20000);

	} while (precland_beacon_est_task->is_running() && ++i < 50);


	delete precland_beacon_est_task;
	precland_beacon_est_task = nullptr;
	PX4_WARN("precland_beacon_est has been stopped");
}

/**
* Start new task, fails if it is already running. Returns OK if successful
**/
static int precland_beacon_est_start()
{
	if (precland_beacon_est_task != nullptr) {
		PX4_WARN("already running");
		return -1;
	}

	precland_beacon_est_task = new PreclandBeaconEst();

	//Check if alloc worked
	if (precland_beacon_est_task == nullptr) {
		PX4_WARN("alloc failed");
		return -1;
	}

	//Start new thread task
	int ret = precland_beacon_est_task->start();

	if (ret) {
		PX4_WARN("task start failed: %d", -errno);
		return -1;
	}

	/* avoid memory fragmentation by not exiting start handler until the task has fully started */
	const uint64_t timeout = hrt_absolute_time() + 5000000; //5 second timeout

	/* avoid printing dots just yet and do one sleep before the first check */
	usleep(10000);

	/* check if the waiting involving dots and a newline are still needed */
	if (!precland_beacon_est_task->is_running()) {
		while (!precland_beacon_est_task->is_running()) {
			usleep(50000);

			if (hrt_absolute_time() > timeout) {
				PX4_WARN("start failed - timeout");
				precland_beacon_est_stop();
				return 1;
			}
		}
	}

	return 0;
}

/**
* Main entry point for this module
**/
int precland_beacon_est_main(int argc, char *argv[])
{

	if (argc < 2) {
		goto exiterr;
	}

	if (argc >= 2 && !strcmp(argv[1], "start")) {
		if (precland_beacon_est_start() != 0) {
			PX4_WARN("precland_beacon_est start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		precland_beacon_est_stop();
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (precland_beacon_est_task) {

			if (precland_beacon_est_task->is_running()) {
				PX4_INFO("running");

			} else {
				PX4_WARN("exists, but not running");
			}

			return 0;

		} else {
			PX4_WARN("not running");
			return 1;
		}
	}

exiterr:
	PX4_WARN("usage: precland_beacon_est {start|stop|status}");
	return 1;
}

}
