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
 * @file fw_land_detector_main.cpp
 * Land detection algorithm for fixed wings
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 */

#include <unistd.h>                 //usleep
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/systemlib.h>    //Scheduler
#include <systemlib/err.h>          //print to console

#include "FixedwingLandDetector.h"

//Function prototypes
static int fw_land_detector_start();
static void fw_land_detector_stop();

/**
 * land detector app start / stop handling function
 * This makes the land detector module accessible from the nuttx shell
 * @ingroup apps
 */
extern "C" __EXPORT int fw_land_detector_main(int argc, char *argv[]);

//Private variables
static FixedwingLandDetector *fw_land_detector_task = nullptr;
static int _landDetectorTaskID = -1;

/**
* Deamon thread function
**/
static void fw_land_detector_deamon_thread(int argc, char *argv[])
{
	fw_land_detector_task->landDetectorLoop();
}

/**
* Stop the task, force killing it if it doesn't stop by itself
**/
static void fw_land_detector_stop()
{
	if (fw_land_detector_task == nullptr || _landDetectorTaskID == -1) {
		errx(1, "not running");
		return;
	}

	fw_land_detector_task->shutdown();

	//Wait for task to die
	int i = 0;

	do {
		/* wait 20ms */
		usleep(20000);

		/* if we have given up, kill it */
		if (++i > 50) {
			task_delete(_landDetectorTaskID);
			break;
		}
	} while (fw_land_detector_task->isRunning());


	delete fw_land_detector_task;
	fw_land_detector_task = nullptr;
	_landDetectorTaskID = -1;
	warn("fw_land_detector has been stopped");
}

/**
* Start new task, fails if it is already running. Returns OK if successful
**/
static int fw_land_detector_start()
{
	if (fw_land_detector_task != nullptr || _landDetectorTaskID != -1) {
		errx(1, "already running");
		return -1;
	}

	//Allocate memory
	fw_land_detector_task = new FixedwingLandDetector();

	if (fw_land_detector_task == nullptr) {
		errx(1, "alloc failed");
		return -1;
	}

	//Start new thread task
	_landDetectorTaskID = task_spawn_cmd("fw_land_detector",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_DEFAULT,
					     1024,
					     (main_t)&fw_land_detector_deamon_thread,
					     nullptr);

	if (_landDetectorTaskID < 0) {
		errx(1, "task start failed: %d", -errno);
		return -1;
	}

	/* avoid memory fragmentation by not exiting start handler until the task has fully started */
	const uint32_t timeout = hrt_absolute_time() + 5000000; //5 second timeout

	while (!fw_land_detector_task->isRunning()) {
		usleep(50000);
		printf(".");
		fflush(stdout);

		if (hrt_absolute_time() > timeout) {
			err(1, "start failed - timeout");
			fw_land_detector_stop();
			exit(1);
		}
	}

	printf("\n");

	exit(0);
	return 0;
}

/**
* Main entry point for this module
**/
int fw_land_detector_main(int argc, char *argv[])
{

	if (argc < 1) {
		warnx("usage: fw_land_detector {start|stop|status}");
		exit(0);
	}

	if (!strcmp(argv[1], "start")) {
		fw_land_detector_start();
	}

	if (!strcmp(argv[1], "stop")) {
		fw_land_detector_stop();
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (fw_land_detector_task) {

			if (fw_land_detector_task->isRunning()) {
				warnx("running");

			} else {
				errx(1, "exists, but not running");
			}

			exit(0);

		} else {
			errx(1, "not running");
		}
	}

	warn("usage: fw_land_detector {start|stop|status}");
	return 1;
}
