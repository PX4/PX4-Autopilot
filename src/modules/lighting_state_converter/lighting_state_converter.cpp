/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "lighting_state_converter.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>


int LightingStateConverter::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int LightingStateConverter::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int LightingStateConverter::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

LightingStateConverter *LightingStateConverter::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	LightingStateConverter *instance = new LightingStateConverter(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

LightingStateConverter::LightingStateConverter(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void LightingStateConverter::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	int lighting_states_sub = orb_subscribe(ORB_ID(lighting_states));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = lighting_states_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	parameters_update(true);

	_lighting_id_no = _param_lighting_id_no.get();

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here
			//PX4_ERR("Timeout on poll");

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			struct lighting_states_s lighting_states;
			orb_copy(ORB_ID(lighting_states), lighting_states_sub, &lighting_states);
			// TODO: do something with the data...

			printf("Received state data. Contents: \n");
			for(int i = 0; i < 10; i++) {
				printf("ID: %d State: %d\n", i, lighting_states.state[i]);
			}

			for (int i = (_lighting_id_no - 1)*2; i < (_lighting_id_no*2); i++) {
				printf("_lighting_id_no: %d\n", i);
				if (lighting_states.state[i] != 255) {
					led_control_s led_control;
					if(i == (_lighting_id_no - 1)*2) led_control.led_mask = 0b0000011110000010;
					if(i == (_lighting_id_no*2)-1) led_control.led_mask = 0b0000000001111100;

					if (lighting_states.state[i] == 0) {
						led_control.color = led_control_s::COLOR_DIM_RED;
						led_control.mode = led_control_s::MODE_ON;
					}

					if (lighting_states.state[i] == 1) {
						led_control.color = led_control_s::COLOR_RED;
						led_control.mode = led_control_s::MODE_ON;
					}

					if (lighting_states.state[i] == 2) {
						led_control.color = led_control_s::COLOR_WHITE;
						led_control.mode = led_control_s::MODE_ON;
					}

					if (lighting_states.state[i] == 3) {
						led_control.color = led_control_s::COLOR_YELLOW;
						led_control.mode = led_control_s::MODE_BLINK_NORMAL;
						led_control.num_blinks = 0;
					}

					if (lighting_states.state[i] == 4) {
						led_control.color = led_control_s::COLOR_YELLOW;
						led_control.mode = led_control_s::MODE_BREATHE;
						led_control.num_blinks = 0;
					}

					led_control.timestamp = hrt_absolute_time();
					_led_control_pub.publish(led_control);
				}
			}
		}

		parameters_update();
	}

	orb_unsubscribe(lighting_states_sub);
}

void LightingStateConverter::parameters_update(bool force)
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

int LightingStateConverter::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int lighting_state_converter_main(int argc, char *argv[])
{
	return LightingStateConverter::main(argc, argv);
}
