/****************************************************************************
*
*   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
*
* This module is a first test for the robosub motor control.
*
* @author Daan Smienk <daansmienk10@gmail.com>
*/

#include "rs_pos_control.hpp"
#include "px4_platform_common/defines.h"
#include "px4_platform_common/log.h"


/**
* Robosub motor control app start / stop handling function
*
* @ingroup apps
*/
extern "C" __EXPORT int rs_pos_control_main(int argc, char *argv[]);


RobosubPosControl::RobosubPosControl():
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

RobosubPosControl::~RobosubPosControl()
{
	perf_free(_loop_perf);
}

bool RobosubPosControl::init()
{
	ScheduleNow();
	PX4_DEBUG("RobosubPosControl::init()");
	return true;
}

void RobosubPosControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

void RobosubPosControl::Run()
{
	PX4_INFO("RobosubPosControl::Run()");

	if (should_exit()) {
		//  _vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check vehicle control mode for changes to publication state */
	//  _vcontrol_mode_sub.update(&_vcontrol_mode);
	_vcontrol_mode_sub.update(&_vcontrol_mode);

	/* update parameters from storage */
	parameters_update();

	perf_end(_loop_perf);
}

int RobosubPosControl::task_spawn(int argc, char *argv[])
{
	RobosubPosControl *instance = new RobosubPosControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int RobosubPosControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int RobosubPosControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the attitude of an unmanned underwater vehicle (UUV).

Publishes `vehicle_thrust_setpont` and `vehicle_torque_setpoint` messages at a constant 250Hz.

### Implementation
Currently, this implementation supports only a few modes:

* Full manual: Roll, pitch, yaw, and throttle controls are passed directly through to the actuators
* Auto mission: The uuv runs missions

### Examples
CLI usage example:
$ rs_pos_control start
$ rs_pos_control status
$ rs_pos_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("_robosub_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rs_pos_control_main(int argc, char *argv[])
{
	return RobosubPosControl::main(argc, argv);
}
