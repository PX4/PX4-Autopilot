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

#include "sc_thruster_controller.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

int ScThrusterController::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int ScThrusterController::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	/*
	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
	        get_instance()->do_something();
	        return 0;
	}
	 */

	return print_usage("unknown command");
}

int ScThrusterController::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 1024, (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

ScThrusterController *ScThrusterController::instantiate(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	bool error_flag = false;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	_debug_thruster_print = false;

	while ((ch = px4_getopt(argc, argv, "d", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			_debug_thruster_print = true;
			PX4_INFO("Debugging enabled");
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

	ScThrusterController *instance = new ScThrusterController();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

ScThrusterController::ScThrusterController() : ModuleParams(nullptr) {}

void ScThrusterController::run()
{
	// Subscribe to thruster command
	int thrustercmd_sub = orb_subscribe(ORB_ID(thruster_command));
	px4_pollfd_struct_t fds[] = {
		{.fd = thrustercmd_sub, .events = POLLIN},
	};

	// Create publisher for PWM values
	struct actuator_motors_s actuator_motors_msg;
	memset(&actuator_motors_msg, 0, sizeof(actuator_motors_msg));
	orb_advert_t _actuator_motors_pub = orb_advertise(ORB_ID(actuator_motors), &actuator_motors_msg);

	// For publishing debug messages
	struct mavlink_log_s my_msg;
	memset(&my_msg, 0, sizeof(my_msg));
	orb_advert_t _my_message_pub = orb_advertise(ORB_ID(mavlink_log), &my_msg);

	// Thruster command structure
	struct thruster_command_s thruster_cmd;

	// initialize parameters
	parameters_update(true);
	bool debugPrint = ScThrusterController::_debug_thruster_print;
	PX4_INFO("Debugging enabled: %d", debugPrint);

	while (!should_exit()) {
		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here
			if (debugPrint) { PX4_WARN("Timeout..."); }

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {
			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(thruster_command), thrustercmd_sub, &thruster_cmd);

			if (debugPrint)
				PX4_INFO("x1: %8.4f\t x2: %8.4f\t y1: %8.4f\t y2: %8.4f", (double)thruster_cmd.x1, (double)thruster_cmd.x2,
					 (double)thruster_cmd.y1, (double)thruster_cmd.y2);

			actuator_motors_msg.control[0] = thruster_cmd.x1 > 0 ? thruster_cmd.x1 : 0;
			actuator_motors_msg.control[1] = thruster_cmd.x1 < 0 ? -thruster_cmd.x1 : 0;
			actuator_motors_msg.control[2] = thruster_cmd.x2 > 0 ? thruster_cmd.x2 : 0;
			actuator_motors_msg.control[3] = thruster_cmd.x2 < 0 ? -thruster_cmd.x2 : 0;
			actuator_motors_msg.control[4] = thruster_cmd.y1 > 0 ? thruster_cmd.y1 : 0;
			actuator_motors_msg.control[5] = thruster_cmd.y1 < 0 ? -thruster_cmd.y1 : 0;
			actuator_motors_msg.control[6] = thruster_cmd.y2 > 0 ? thruster_cmd.y2 : 0;
			actuator_motors_msg.control[7] = thruster_cmd.y2 < 0 ? -thruster_cmd.y2 : 0;

			orb_publish(ORB_ID(actuator_motors), _actuator_motors_pub, &actuator_motors_msg);

			if (debugPrint) {
				strcpy(my_msg.text, "Thruster command received!");
				orb_publish(ORB_ID(mavlink_log), _my_message_pub, &my_msg);
			}
		}

		parameters_update();
	}

	orb_unsubscribe(thrustercmd_sub);
}

void ScThrusterController::parameters_update(bool force)
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

int ScThrusterController::print_usage(const char *reason)
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

int sc_thruster_controller_main(int argc, char* argv[]) { return ScThrusterController::main(argc, argv); }
