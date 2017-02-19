/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file land_detector_main.cpp
 * Land detection algorithm
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Lorenz Meier <lorenz@px4.io>
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

#include "FixedwingLandDetector.h"
#include "MulticopterLandDetector.h"
#include "VtolLandDetector.h"
#include "RoverLandDetector.h"


namespace land_detector
{

// Function prototypes
static int land_detector_start(const char *mode);
static void land_detector_stop();

/**
 * land detector app start / stop handling function
 * This makes the land detector module accessible from the nuttx shell
 * @ingroup apps
 */
extern "C" __EXPORT int land_detector_main(int argc, char *argv[]);

// Private variables
static LandDetector *land_detector_task = nullptr;
static char _currentMode[12];

/**
 * Stop the task, force killing it if it doesn't stop by itself
 */
static void land_detector_stop()
{
	if (land_detector_task == nullptr) {
		PX4_WARN("not running");
		return;
	}

	land_detector_task->stop();

	// Wait for task to die
	int i = 0;

	do {
		// wait 20ms at a time
		usleep(20000);

	} while (land_detector_task->is_running() && ++i < 50);


	delete land_detector_task;
	land_detector_task = nullptr;
	PX4_WARN("land_detector has been stopped");
}

/**
 * Start new task, fails if it is already running. Returns OK if successful
 */
static int land_detector_start(const char *mode)
{
	if (land_detector_task != nullptr) {
		PX4_WARN("already running");
		return -1;
	}

	//Allocate memory
	if (!strcmp(mode, "fixedwing")) {
		land_detector_task = new FixedwingLandDetector();

	} else if (!strcmp(mode, "multicopter")) {
		land_detector_task = new MulticopterLandDetector();

	} else if (!strcmp(mode, "vtol")) {
		land_detector_task = new VtolLandDetector();

	} else if (!strcmp(mode, "rover")) {
		land_detector_task = new RoverLandDetector();

	} else {
		PX4_WARN("[mode] must be either 'fixedwing', 'multicopter', or 'vtol'");
		return -1;
	}

	// Check if alloc worked
	if (land_detector_task == nullptr) {
		PX4_WARN("alloc failed");
		return -1;
	}

	// Start new thread task
	int ret = land_detector_task->start();

	if (ret) {
		PX4_WARN("task start failed: %d", -errno);
		return -1;
	}

	// Avoid memory fragmentation by not exiting start handler until the task has fully started
	const uint64_t timeout = hrt_absolute_time() + 5000000; // 5 second timeout

	// Do one sleep before the first check
	usleep(10000);

	if (!land_detector_task->is_running()) {
		while (!land_detector_task->is_running()) {
			usleep(50000);

			if (hrt_absolute_time() > timeout) {
				PX4_WARN("start failed - timeout");
				land_detector_stop();
				return 1;
			}
		}
	}

	// Remember current active mode
	strncpy(_currentMode, mode, sizeof(_currentMode) - 1);
	_currentMode[sizeof(_currentMode) - 1] = '\0';

	return 0;
}

/**
 * Main entry point for this module
 */
int land_detector_main(int argc, char *argv[])
{

	if (argc < 2) {
		goto exiterr;
	}

	if (argc >= 2 && !strcmp(argv[1], "start")) {
		if (land_detector_start(argv[2]) != 0) {
			PX4_WARN("land_detector start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		land_detector_stop();
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (land_detector_task) {

			if (land_detector_task->is_running()) {
				PX4_INFO("running (%s)", _currentMode);
				LandDetector::LandDetectionState state = land_detector_task->get_state();

				switch (state) {
				case LandDetector::LandDetectionState::FLYING:
					PX4_INFO("State: Flying");
					break;

				case LandDetector::LandDetectionState::LANDED:
					PX4_INFO("State: Landed");
					break;

				case LandDetector::LandDetectionState::FREEFALL:
					PX4_INFO("State: Freefall");
					break;

				default:
					PX4_ERR("State: unknown");
					break;
				}

			} else {
				PX4_WARN("exists, but not running (%s)", _currentMode);
			}

			return 0;

		} else {
			PX4_WARN("not running");
			return 1;
		}
	}

exiterr:
	PX4_WARN("usage: land_detector {start|stop|status} [mode]");
	PX4_WARN("mode can either be 'fixedwing' or 'multicopter'");
	return 1;
}

}
