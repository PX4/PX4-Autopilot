/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <drivers/drv_hrt.h>

#include <px4_platform_common/getopt.h>
#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>

#include "rc_controller.hpp"

int RC_ControllerModule::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int RC_ControllerModule::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "throttle")) {
		uint16_t val = atoi(argv[1]);
		get_instance()->set_throttle(val);
		PX4_INFO("Setting throttle to %u", val);
		return 0;
	}

	if (!strcmp(argv[0], "yaw")) {
		uint16_t val = atoi(argv[1]);
		get_instance()->set_yaw(val);
		PX4_INFO("Setting yaw to %u", val);
		return 0;
	}

	if (!strcmp(argv[0], "pitch")) {
		uint16_t val = atoi(argv[1]);
		get_instance()->set_pitch(val);
		PX4_INFO("Setting pitch to %u", val);
		return 0;
	}

	if (!strcmp(argv[0], "roll")) {
		uint16_t val = atoi(argv[1]);
		get_instance()->set_roll(val);
		PX4_INFO("Setting roll to %u", val);
		return 0;
	}

	return print_usage("unknown command");
}


int RC_ControllerModule::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("RC_ControllerModule",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_MAX,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

RC_ControllerModule *RC_ControllerModule::instantiate(int argc, char *argv[])
{
	// int example_param = 0;
	// bool example_flag = false;
	// bool error_flag = false;
	//
	// int myoptind = 1;
	// int ch;
	// const char *myoptarg = nullptr;
	//
	// // parse CLI arguments
	// while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
	// 	switch (ch) {
	// 	case 'p':
	// 		example_param = (int)strtol(myoptarg, nullptr, 10);
	// 		break;
	//
	// 	case 'f':
	// 		example_flag = true;
	// 		break;
	//
	// 	case '?':
	// 		error_flag = true;
	// 		break;
	//
	// 	default:
	// 		PX4_WARN("unrecognized flag");
	// 		error_flag = true;
	// 		break;
	// 	}
	// }
	//
	// if (error_flag) {
	// 	return nullptr;
	// }

	// RC_ControllerModule *instance = new RC_ControllerModule(example_param, example_flag);
	RC_ControllerModule *instance = new RC_ControllerModule();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

// RC_ControllerModule::RC_ControllerModule(int example_param, bool example_flag)
RC_ControllerModule::RC_ControllerModule()
	: ModuleParams(nullptr)
{
	_throttle = 800;
	// _yaw      = 1500;
	// _pitch    = 1500;
	// _roll     = 1500;
	_yaw      = 800;
	_pitch    = 800;
	_roll     = 800;
}

void RC_ControllerModule::run()
{
	PX4_INFO("ModalAI RC_ControllerModule starting");

	input_rc_s rc_data;
	memset(&rc_data, 0, sizeof(input_rc_s));
	orb_advert_t input_rc_pub_fd = orb_advertise(ORB_ID(input_rc), &rc_data);

	while (!should_exit()) {

		usleep(10000);

		rc_data.input_source = input_rc_s::RC_INPUT_SOURCE_QURT;
		rc_data.channel_count = 4;
		// for (unsigned i = 0; i < rc_data.channel_count; ++i) {
		// 	rc_data.values[i] = 1024;
		// }
		rc_data.values[0] = _throttle;
		rc_data.values[1] = _yaw;
		rc_data.values[2] = _pitch;
		rc_data.values[3] = _roll;

		rc_data.timestamp = hrt_absolute_time();
		rc_data.timestamp_last_signal = rc_data.timestamp;
		rc_data.rc_ppm_frame_length = 0;
		rc_data.rssi = 100;
		rc_data.rc_failsafe = false;
		rc_data.rc_lost = 0;
		rc_data.rc_lost_frame_count = 0;
		rc_data.rc_total_frame_count = 0;

		orb_publish(ORB_ID(input_rc), input_rc_pub_fd, &rc_data);

		parameters_update();
	}

	PX4_INFO("RC_ControllerModule Exiting");
}

void RC_ControllerModule::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int RC_ControllerModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	// TODO: PRINT_MODULE_DESCRIPTION

	PRINT_MODULE_USAGE_NAME("rc_controller", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("rc");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rc_controller_main(int argc, char *argv[])
{
	return RC_ControllerModule::main(argc, argv);
}
