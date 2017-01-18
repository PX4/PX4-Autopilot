/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#include "FixedwingStickMapper.hpp"
#include "MulticopterStickMapper.hpp"

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

static volatile bool thread_should_exit = false;     /**< Deamon exit flag */
static volatile bool thread_running = false;     /**< Deamon status flag */

/**
 * Stick Mapper app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int stick_mapper_main(int argc, char *argv[]);

static StickMapper *stick_mapper_task = nullptr;
static char _currentMode[12];

static int stick_mapper_start(const char *mode);
static void stick_mapper_stop();
static int usage(const char *reason);

int stick_mapper_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_WARN("usage: stick_mapper {start|stop|status} [mode]");
		PX4_WARN("mode can either be 'fixedwing', 'multicopter', or 'vtol'");
		return 1;
	}

	if (argc >= 2 && !strcmp(argv[1], "start")) {
		if (stick_mapper_start(argv[2]) != 0) {
			PX4_WARN("stick_mapper start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		stick_mapper_stop();
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (stick_mapper_task) {
			if (stick_mapper_task->is_running()) {
				PX4_INFO("running (%s)", _currentMode);

			} else {
				PX4_WARN("exists, but not running (%s)", _currentMode);
			}

			return 0;

		} else {
			PX4_WARN("not running");
			return 1;
		}
	}

	usage("unrecognized command");
	return 1;
}

// start new task, fails if it is already running. Returns OK if successful
static int stick_mapper_start(const char *mode)
{
	if (stick_mapper_task != nullptr) {
		PX4_WARN("already running");
		return -1;
	}

	// allocate memory
	if (!strcmp(mode, "fixedwing")) {
		stick_mapper_task = new FixedwingStickMapper();

	} else if (!strcmp(mode, "multicopter")) {
		stick_mapper_task = new MulticopterStickMapper();

	} else if (!strcmp(mode, "vtol")) {
		//stick_mapper_task = new VtolStickMapper();

	} else {
		PX4_WARN("[mode] must be either 'fixedwing', 'multicopter', or 'vtol'");
		return -1;
	}

	// check if alloc worked
	if (stick_mapper_task == nullptr) {
		PX4_WARN("alloc failed");
		return -1;
	}

	// start new thread task
	stick_mapper_task->start();

	/* avoid memory fragmentation by not exiting start handler until the task has fully started */
	const uint64_t timeout = hrt_absolute_time() + 5000000; // 5 second timeout

	/* avoid printing dots just yet and do one sleep before the first check */
	usleep(10000);

	/* check if the waiting involving dots and a newline are still needed */
	if (!stick_mapper_task->is_running()) {
		while (!stick_mapper_task->is_running()) {
			usleep(50000);

			if (hrt_absolute_time() > timeout) {
				PX4_WARN("start failed - timeout");
				stick_mapper_stop();
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
* Stop the task, force killing it if it doesn't stop by itself
**/
static void stick_mapper_stop()
{
	if (stick_mapper_task == nullptr) {
		PX4_WARN("not running");
		return;
	}

	stick_mapper_task->stop();

	// Wait for task to die
	int i = 0;

	do {
		/* wait 20ms */
		usleep(20000);

	} while (stick_mapper_task->is_running() && ++i < 50);


	delete stick_mapper_task;
	stick_mapper_task = nullptr;
	PX4_WARN("stick_mapper_task has been stopped");
}

static int
usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PX4_WARN("usage: stick_mapper {start|stop|status} [mode]");
	PX4_WARN("mode can either be 'fixedwing', 'multicopter', or 'vtol'");
	return 1;
}
