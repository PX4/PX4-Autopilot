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
#include "dump_pgm.h"
#include <mt9v117/MT9V117.hpp>

extern "C" { __EXPORT int bebop_flow_main(int argc, char *argv[]); }

using namespace DriverFramework;

namespace bebop_flow
{
MT9V117 *image_sensor = nullptr;				 // I2C image sensor
VideoDevice *g_dev = nullptr;            // interface to the video device
volatile bool _task_should_exit = false; // flag indicating if bebop flow task should exit
static bool _is_running = false;         // flag indicating if bebop flow  app is running
static px4_task_t _task_handle = -1;     // handle to the task main thread
volatile unsigned int _trigger = 0;			 // Number of images to write as pgm

const char *dev_name = "/dev/video0";		 // V4L video device

int start();
int stop();
int info();
int trigger(int count);
int clear_errors();
void usage();
void task_main(int argc, char *argv[]);

void task_main(int argc, char *argv[])
{
	_is_running = true;
	int ret = 0;
	struct frame_data frame;
	memset(&frame, 0, sizeof(frame));
	uint32_t timeout_cnt = 0;

	// Main loop
	while (!_task_should_exit) {

		ret = g_dev->get_frame(frame);

		if (ret < 0) {
			PX4_ERR("Get Frame failed");
			continue;

		} else if (ret == 1) {
			// No image in buffer
			usleep(1000);
			++timeout_cnt;

			if (timeout_cnt > 1000) {
				PX4_WARN("No frames received for 1 sec");
				timeout_cnt = 0;
			}

			continue;
		}

		timeout_cnt = 0;

		// Write images into a file
		if (_trigger > 0) {
			PX4_INFO("Trigger camera");

			dump_pgm(frame.data, frame.bytes, frame.seq, frame.timestamp);
			--_trigger;
		}

		/***************************************************************
		 *
		 * Optical Flow computation
		 *
		 **************************************************************/

		ret = g_dev->put_frame(frame);

		if (ret < 0) {
			PX4_ERR("Put Frame failed");
		}
	}

	_is_running = false;
}

int start()
{
	if (_is_running) {
		PX4_WARN("bebop_flow already running");
		return -1;
	}

	// Prepare the I2C device
	image_sensor = new MT9V117(IMAGE_DEVICE_PATH);

	if (image_sensor == nullptr) {
		PX4_ERR("failed instantiating image sensor object");
		return -1;
	}

	int ret = image_sensor->start();

	if (ret != 0) {
		PX4_ERR("Image sensor start failed");
		return ret;
	}

	// Start the video device
	g_dev = new VideoDevice(dev_name, 6);

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating video device object");
		return -1;
	}

	ret = g_dev->start();

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
		PX4_WARN("task start failed");
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
	_task_should_exit = false;
	_trigger = 0;

	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return -1;
	}

	int ret = g_dev->stop();

	if (ret != 0) {
		PX4_ERR("driver could not be stopped");
		return ret;
	}

	if (image_sensor == nullptr) {
		PX4_ERR("Image sensor not running");
		return -1;
	}

	ret = image_sensor->stop();

	if (ret != 0) {
		PX4_ERR("Image sensor driver  could not be stopped");
		return ret;
	}

	delete g_dev;
	delete image_sensor;
	g_dev = nullptr;
	image_sensor = nullptr;
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

int trigger(int count)
{
	if (_is_running) {
		_trigger = count;

	} else {
		PX4_WARN("bebop_flow is not running");
	}

	return OK;
}

void
usage()
{
	PX4_INFO("Usage: bebop_flow 'start', 'info', 'stop', 'trigger [-n #]'");
}

} /* bebop flow namespace*/

int
bebop_flow_main(int argc, char *argv[])
{
	int ch;
	int ret = 0;
	int myoptind = 1;
	const char *myoptarg = NULL;
	unsigned int trigger_count = 1;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'n':
			trigger_count = atoi(myoptarg);
			break;

		default:
			bebop_flow::usage();
			return 0;
		}
	}

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

	else if (!strcmp(verb, "trigger")) {
		ret = bebop_flow::trigger(trigger_count);
	}

	else {
		bebop_flow::usage();
		return 1;
	}

	return ret;
}
