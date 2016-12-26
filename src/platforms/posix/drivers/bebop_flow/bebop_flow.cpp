
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
 * @file bebop_flow.cpp
 *
 * This is a wrapper around the Parrot Bebop's downward-facing camera and integrates
 * an optical flow computation.
 */

#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>

#include "video_device.h"

extern "C" { __EXPORT int bebop_flow_main(int argc, char *argv[]); }

namespace bebop_flow
{
VideoDevice *g_dev = nullptr;            // interface to the video device
volatile bool _task_should_exit = false; // flag indicating if bebop flow task should exit
static bool _is_running = false;         // flag indicating if bebop flow  app is running
static px4_task_t _task_handle = -1;     // handle to the task main thread

static char *dev_name = "/dev/video0";

int start();
int stop();
int info();
int clear_errors();
void usage();
void task_main(int argc, char *argv[]);


void task_main(int argc, char *argv[])
{
	_is_running = true;

	// Main loop
	while (!_task_should_exit) {

	}

	_is_running = false;

}

int start()
{
	g_dev = new VideoDevice();

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating video device object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("Video device start failed");
		return ret;
	}

	/* start the task */
	_task_handle = px4_task_spawn_cmd("bebop_flow",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  (px4_main_t)&task_main,
					  nullptr);

	if (_task_handle < 0) {
		warn("task start failed");
		return -1;
	}

	return 0;
}

int stop()
{
	// Stop bebop flow task
	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}

	_task_handle = -1;

	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	// Stop DF device
	int ret = g_dev->stop();

	if (ret != 0) {
		PX4_ERR("driver could not be stopped");
		return ret;
	}

	delete g_dev;
	g_dev = nullptr;
	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_DEBUG("state @ %p", g_dev);

	int ret = g_dev->print_info();

	if (ret != 0) {
		PX4_ERR("Unable to print info");
		return ret;
	}

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: bebop_flow 'start', 'info', 'stop'");
}

} /* bebop flow namespace*/

int
bebop_flow_main(int argc, char *argv[])
{
	int ret = 0;
	int myoptind = 1;

	if (argc <= 1) {
		bebop_flow::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = bebop_flow::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = bebop_flow::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = bebop_flow::info();
	}

	else {
		bebop_flow::usage();
		return 1;
	}

	return ret;
}
