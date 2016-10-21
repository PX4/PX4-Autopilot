/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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

#include <px4_getopt.h>
#include <px4_tasks.h>
#include <px4_log.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_rc_input.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <semaphore.h>
#include "rc_receiver_api.h"
#ifdef __cplusplus
}
#endif

/** driver 'main' command */
extern "C" { __EXPORT int rc_receiver_main(int argc, char *argv[]); }

#define MAX_LEN_DEV_PATH 32

namespace rc_receiver
{

/** Threshold value to detect rc receiver signal lost in millisecond */
#define SIGNAL_LOST_THRESHOLD_MS   500

/** serial device path that rc receiver is connected to */
static char _device[MAX_LEN_DEV_PATH];

/** flag indicating if rc_receiver module is running */
static bool _is_running = false;

/** flag indicating if task thread should exit */
static bool _task_should_exit = false;

/** handle to the task main thread */
static px4_task_t _task_handle = -1;

/** RC receiver type */
static enum RC_RECEIVER_TYPES _rc_receiver_type;

/** RC input channel value array accomondating up to RC_INPUT_MAX_CHANNELS */
static uint16_t rc_inputs[input_rc_s::RC_INPUT_MAX_CHANNELS];

/** rc_input uorb topic publication handle */
static orb_advert_t _input_rc_pub = nullptr;

/** rc_input uorb topic data */
static struct input_rc_s _rc_in;

/**< parameter update subscription */
static int _params_sub;

struct {
	param_t rc_receiver_type;
} _params_handles;  /**< parameter handles */

/** Print out the usage information */
static void usage();

/** mpu9x50 start measurement */
static void start();

/** mpu9x50 stop measurement */
static void stop();

/** task main trampoline function */
static void	task_main_trampoline(int argc, char *argv[]);

/** mpu9x50 measurement thread primary entry point */
static void task_main(int argc, char *argv[]);

/** update all parameters */
static void parameters_update();

/** poll parameter update */
static void parameter_update_poll();

void parameters_update()
{
	PX4_DEBUG("rc_receiver_parameters_update");
	float v;

	// accel params
	if (param_get(_params_handles.rc_receiver_type, &v) == 0) {
		_rc_receiver_type = (enum RC_RECEIVER_TYPES)v;
		PX4_DEBUG("rc_receiver_parameters_update rc_receiver_type %f", v);
	}
}

void parameters_init()
{
	_params_handles.rc_receiver_type	=	param_find("RC_RECEIVER_TYPE");

	parameters_update();
}

void parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void start()
{
	ASSERT(_task_handle == -1);

	/* start the task */
	_task_handle = px4_task_spawn_cmd("rc_receiver_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX,
					  1500,
					  (px4_main_t)&task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		warn("task start failed");
		return;
	}

	_is_running = true;
}

void stop()
{
	// TODO - set thread exit signal to terminate the task main thread

	_is_running = false;
	_task_handle = -1;
}

void task_main_trampoline(int argc, char *argv[])
{
	PX4_WARN("task_main_trampoline");
	task_main(argc, argv);
}

void task_main(int argc, char *argv[])
{
	PX4_WARN("enter rc_receiver task_main");
	uint32_t fd;

	// clear the rc_input report for topic advertise
	memset(&_rc_in, 0, sizeof(struct input_rc_s));

	_input_rc_pub = orb_advertise(ORB_ID(input_rc), &_rc_in);

	if (_input_rc_pub == nullptr) {
		PX4_WARN("failed to advertise input_rc uorb topic. Quit!");
		return ;
	}

	// subscribe to parameter_update topic
	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	// Open the RC receiver device on the specified serial port
	fd = rc_receiver_open(_rc_receiver_type, _device);

	if (fd <= 0) {
		PX4_WARN("failed to open rc receiver type %d dev %s. Quit!",
			 _rc_receiver_type, _device);
		return ;
	}

	// Continuously receive RC packet from serial device, until task is signaled
	// to exit
	uint32_t num_channels;
	uint64_t ts = hrt_absolute_time();
	int ret;
	int counter = 0;

	_rc_in.timestamp_last_signal = ts;

	while (!_task_should_exit) {
		// poll parameter update
		parameter_update_poll();

		// read RC packet from serial device in blocking mode.
		num_channels = input_rc_s::RC_INPUT_MAX_CHANNELS;

		ret = rc_receiver_get_packet(fd, rc_inputs, &num_channels);
		ts = hrt_absolute_time();

		_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_QURT;

		if (ret < 0) {
			// enum RC_RECEIVER_ERRORS error_code = rc_receiver_get_last_error(fd);
			// PX4_WARN("RC packet receiving timed out. error code %d", error_code);

			uint64_t time_diff_us = ts - _rc_in.timestamp_last_signal;

			if (time_diff_us > SIGNAL_LOST_THRESHOLD_MS * 1000) {
				_rc_in.rc_lost = true;

				if (++counter == 500) {
					PX4_WARN("RC signal lost for %u ms", time_diff_us / 1000);
					counter = 0;
				}

			} else {
				continue;
			}
		}

		// populate the input_rc_s structure
		if (ret == 0) {
			_rc_in.timestamp = ts;
			_rc_in.timestamp_last_signal = _rc_in.timestamp;
			_rc_in.channel_count = num_channels;

			// TODO - need to add support for RSSI, failsafe mode
			_rc_in.rssi = RC_INPUT_RSSI_MAX;
			_rc_in.rc_failsafe = false;
			_rc_in.rc_lost = false;
			_rc_in.rc_lost_frame_count = 0;
			_rc_in.rc_total_frame_count = 1;
		}

		for (uint32_t i = 0; i < num_channels; i++) {
			// Scale the Spektrum DSM value to ppm encoding. This is for the
			// consistency with PX4 which internally translates DSM to PPM.
			// See modules/px4iofirmware/dsm.c for details
			// NOTE: rc_receiver spektrum driver outputs the data in 10bit DSM
			// format, so we need to double the channel value before the scaling
			_rc_in.values[i] = ((((int)(rc_inputs[i] * 2) - 1024) * 1000) / 1700) + 1500;
		}

		orb_publish(ORB_ID(input_rc), _input_rc_pub, &_rc_in);
	}

	rc_receiver_close(fd);
}

void usage()
{
	PX4_WARN("missing command: try 'start', 'stop', 'status'");
	PX4_WARN("options:");
	PX4_WARN("    -D device");
}

} // namespace rc_receiver

int rc_receiver_main(int argc, char *argv[])
{
	const char *device = NULL;
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "D:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'D':
			device = myoptarg;
			break;

		default:
			rc_receiver::usage();
			return 1;
		}
	}

	// Check on required arguments
	if (device == NULL || strlen(device) == 0) {
		rc_receiver::usage();
		return 1;
	}

	memset(rc_receiver::_device, 0, MAX_LEN_DEV_PATH);
	strncpy(rc_receiver::_device, device, strlen(device));

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (rc_receiver::_is_running) {
			PX4_WARN("rc_receiver already running");
			return 1;
		}

		rc_receiver::start();

	} else if (!strcmp(verb, "stop")) {
		if (rc_receiver::_is_running) {
			PX4_WARN("rc_receiver is not running");
			return 1;
		}

		rc_receiver::stop();

	} else if (!strcmp(verb, "status")) {
		PX4_WARN("rc_receiver is %s", rc_receiver::_is_running ? "running" : "stopped");
		return 0;

	} else {
		rc_receiver::usage();
		return 1;
	}

	return 0;
}
