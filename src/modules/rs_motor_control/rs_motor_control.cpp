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

#include "rs_motor_control.hpp"
#include "px4_platform_common/defines.h"
#include "px4_platform_common/log.h"


/**
* Robosub motor control app start / stop handling function
*
* @ingroup apps
*/
extern "C" __EXPORT int rs_motor_control_main(int argc, char *argv[]);


RobosubMotorControl::RobosubMotorControl():
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

RobosubMotorControl::~RobosubMotorControl()
{
	perf_free(_loop_perf);
}

bool RobosubMotorControl::init()
{
	// Execute the Run() function everytime an input_rc is publiced
	if (!_rs_input_rc_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	PX4_DEBUG("RobosubMotorControl::init()");
	return true;
}

void RobosubMotorControl::parameters_update(bool force)
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

void RobosubMotorControl::Run()
{
	PX4_INFO("RobosubMotorControl::Run()");

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

	PX4_INFO("rc data");

	// Check if the input_rc topic has been updated
	// and copy the data to the local variable
	if (_input_rc_sub.update(&_input_rc)) {
		input_rc_s rc_data {};
		_input_rc_sub.copy(&rc_data);

		// Debug print the rc data
		for (unsigned i = 0; i < rc_data.channel_count; ++i) {
			PX4_INFO("rc_data[%u]: %d", i, rc_data.values[i]);
		}

		// Normalize the rc data to a value between -1 and 1
		float normalized_0 = (rc_data.values[0] - 1500) / 400.0f;
		float normalized_1 = (rc_data.values[1] - 1500) / 400.0f;
		float normalized_2 = (rc_data.values[2] - 1500) / 400.0f;
		float normalized_3 = (rc_data.values[3] - 1500) / 400.0f;
		float normalized_4 = (rc_data.values[4] - 1500) / 400.0f;
		float normalized_5 = (rc_data.values[5] - 1500) / 400.0f;
		// float normalized_6 = (rc_data.values[6] - 1500) / 400.0f;


		// actuator_motors_s actuator_motors{};
		// actuator_motors.timestamp = hrt_absolute_time();
		// actuator_motors.reversible_flags = 0;
		// actuator_motors.control[0] = normalized;
		// _actuator_motors_pub.publish(actuator_motors);
		if (_vcontrol_mode.flag_control_manual_enabled ||
		    _vcontrol_mode.flag_control_attitude_enabled) {

			_vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint.timestamp_sample = hrt_absolute_time();
			normalized_0 = math::constrain(normalized_0, -1.0f, 1.0f);
			normalized_1 = math::constrain(normalized_1, -1.0f, 1.0f);
			normalized_2 = math::constrain(normalized_2, -1.0f, 1.0f);
			normalized_3 = math::constrain(normalized_3, -1.0f, 1.0f);
			normalized_4 = math::constrain(normalized_4, -1.0f, 1.0f);
			normalized_5 = math::constrain(normalized_5, -1.0f, 1.0f);
			_vehicle_thrust_setpoint.xyz[0] = normalized_1;
			_vehicle_thrust_setpoint.xyz[1] = normalized_2;
			_vehicle_thrust_setpoint.xyz[2] = normalized_4;

			_vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint.timestamp_sample = hrt_absolute_time();
			// _vehicle_torque_setpoint.xyz[0] = normalized_3;
			// _vehicle_torque_setpoint.xyz[1] = normalized_4;
			// _vehicle_torque_setpoint.xyz[2] = normalized_5;

			_vehicle_thrust_setpoint_pub.publish(_vehicle_thrust_setpoint);
			_vehicle_torque_setpoint_pub.publish(_vehicle_torque_setpoint);
			PX4_INFO("Published setpoints at %llu", hrt_absolute_time());
			PX4_INFO("_vehicle_thruster_setpoint: %f, %f, %f", (double)_vehicle_thrust_setpoint.xyz[0],
				 (double)_vehicle_thrust_setpoint.xyz[1], (double)_vehicle_thrust_setpoint.xyz[2]);
			PX4_INFO("Control mode flags: manual=%d, attitude=%d",
				 (int)_vcontrol_mode.flag_control_manual_enabled,
				 (int)_vcontrol_mode.flag_control_attitude_enabled);
			// if(_vehicle_thrust_setpoint_sub.update(&_vehicle_thrust_setpoint_sub)) {
			// 	PX4_INFO("_vehicle_thrust_setpoint_sub updated");
			// }
		}

		// TODO_RS: Why function 101?
		// int function = 101;
		// float value = normalized;
		// TODO_RS: Figure out if its possible to do this without a test function
		// actuator_test(function, value, 0, false);

	}

	perf_end(_loop_perf);
}

void RobosubMotorControl::actuator_test(int function, float value, int timeout_ms, bool release_control)
{
	actuator_test_s actuator_test{};
	actuator_test.timestamp = hrt_absolute_time();
	actuator_test.function = function;
	actuator_test.value = value;
	actuator_test.action = release_control ? actuator_test_s::ACTION_RELEASE_CONTROL : actuator_test_s::ACTION_DO_CONTROL;
	actuator_test.timeout_ms = timeout_ms;

	uORB::Publication<actuator_test_s> actuator_test_pub{ORB_ID(actuator_test)};
	actuator_test_pub.publish(actuator_test);
}

int RobosubMotorControl::task_spawn(int argc, char *argv[])
{
	RobosubMotorControl *instance = new RobosubMotorControl();

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

int RobosubMotorControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int RobosubMotorControl::print_usage(const char *reason)
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
$ rs_motor_control start
$ rs_motor_control status
$ rs_motor_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("_robosub_motor_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rs_motor_control_main(int argc, char *argv[])
{
	return RobosubMotorControl::main(argc, argv);
}
