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
#include "../rs_motor_control/rs_motor_control.hpp"
#include "px4_platform_common/defines.h"
#include "px4_platform_common/log.h"

RobosubMotorControl RobosubPosControl::robosub_motor_control;



/**
* Robosub motor control app start / stop handling function
*
* @ingroup apps
*/
extern "C" __EXPORT int rs_pos_control_main(int argc, char *argv[]);


RobosubPosControl::RobosubPosControl():
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
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
	ScheduleOnInterval(100_ms);
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


	/* update parameters from storage */
	parameters_update();
	posControl();

	perf_end(_loop_perf);
}

void RobosubPosControl::posControl()
{
	if (_drone_task_sub.update(&_drone_task)) {
		drone_task_s drone_task{};
		_drone_task_sub.copy(&drone_task);

		if (drone_task.task == drone_task_s::TASK_AUTONOMOUS) {
			AltitudeControl();
			HorizontalControl();
		}
	}
}


void RobosubPosControl::AltitudeControl() {
	// Constants
	const float setpointPressure = SURFACE_PRESSURE + (SETPOINT / METER_PER_BAR);

	// Simulated or subscribed current pressure
	// Replace with actual sensor input when available
	// float currentPressure = _drone_task.current_pressure; // e.g., 1.02 bar DOES NOT COMPILE
	float currentPressure = 1;

	// PID calculation
	float error = setpointPressure - currentPressure;

	// PX4 delta time calculation
	const hrt_abstime now = hrt_absolute_time();
	static hrt_abstime last_time = now;
	const float dt = math::constrain((now - last_time) / 1e6f, 0.001f, 1.0f); // dt in seconds
	last_time = now;

	_integral += error * dt;
	float derivative = (error - _previous_error) / dt;
	_previous_error = error;

	float thrust = _Kp * error + _Ki * _integral + _Kd * derivative;

	// Clamp thrust between -1.0 and 1.0
	thrust = math::constrain(thrust, -1.0f, 1.0f);

	// Output thrust to the motors
	robosub_motor_control.actuator_test(MOTOR_UP1, 		thrust, 0, false);
	robosub_motor_control.actuator_test(MOTOR_UP2, 		(thrust * 0.5f), 0, false);
	robosub_motor_control.actuator_test(MOTOR_UP3, 		(thrust * 0.5f), 0, false);
}

void RobosubPosControl::HorizontalControl() {
	vehicle_local_position_s local_pos{};
	if (_vehicle_local_position_sub.update(&local_pos)) {
		matrix::Vector3f current_pos(local_pos.x, local_pos.y, local_pos.z);

		switch (_auto_move_state) {
		case AutoMoveState::INIT:
		_origin_position = current_pos;
		_state_entry_time = hrt_absolute_time();
		_auto_move_state = AutoMoveState::MOVE_FORWARD;
		break;

		case AutoMoveState::MOVE_FORWARD:
		send_position_setpoint(_origin_position + matrix::Vector3f(2.f, 0.f, 0.f));
		if (distance_to(current_pos, _origin_position + matrix::Vector3f(2.f, 0.f, 0.f)) < 0.2f) {
			_state_entry_time = hrt_absolute_time();
			_auto_move_state = AutoMoveState::WAIT_FORWARD;
		}
		break;

		case AutoMoveState::WAIT_FORWARD:
		if (hrt_elapsed_time(&_state_entry_time) > 1000000) { // 1s
			_auto_move_state = AutoMoveState::MOVE_ORIGIN;
		}
		break;

		case AutoMoveState::MOVE_ORIGIN:
		send_position_setpoint(_origin_position);
		if (distance_to(current_pos, _origin_position) < 0.2f) {
			_state_entry_time = hrt_absolute_time();
			_auto_move_state = AutoMoveState::WAIT_ORIGIN;
		}
		break;

		case AutoMoveState::WAIT_ORIGIN:
		if (hrt_elapsed_time(&_state_entry_time) > 1000000) {
			_auto_move_state = AutoMoveState::MOVE_BACKWARD;
		}
		break;

		case AutoMoveState::MOVE_BACKWARD:
		send_position_setpoint(_origin_position + matrix::Vector3f(-2.f, 0.f, 0.f));
		if (distance_to(current_pos, _origin_position + matrix::Vector3f(-2.f, 0.f, 0.f)) < 0.2f) {
			_state_entry_time = hrt_absolute_time();
			_auto_move_state = AutoMoveState::WAIT_BACKWARD;
		}
		break;

		case AutoMoveState::WAIT_BACKWARD:
		if (hrt_elapsed_time(&_state_entry_time) > 1000000) {
			_auto_move_state = AutoMoveState::DONE;
		}
		break;

		case AutoMoveState::DONE:
		// Hold position or stop
		send_position_setpoint(current_pos);
		break;
		}
	}
}

void RobosubPosControl::send_position_setpoint(const matrix::Vector3f &pos) {
	vehicle_local_position_setpoint_s setpoint{};
	setpoint.timestamp = hrt_absolute_time();
	setpoint.x = pos(0);
	setpoint.y = pos(1);
	setpoint.z = pos(2);

	local_pos_setpoint_pub.publish(setpoint);
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
