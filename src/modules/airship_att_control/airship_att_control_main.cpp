/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file airship_att_control_main.cpp
 * Airship attitude controller.
 *
 * @author Anton Erasmus	<anton@flycloudline.com>
 */

#include "airship_att_control.hpp"

using namespace matrix;

AirshipAttitudeControl::AirshipAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_actuators_0_pub(ORB_ID(actuator_controls_0)),
	_loop_perf(perf_alloc(PC_ELAPSED, "airship_att_control"))
{
}

AirshipAttitudeControl::~AirshipAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
AirshipAttitudeControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
AirshipAttitudeControl::parameter_update_poll()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

void
AirshipAttitudeControl::publish_actuator_controls()
{
	// zero actuators if not armed
	if (_vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		for (uint8_t i = 0 ; i < 4 ; i++) {
			_actuators.control[i] = 0.0f;
		}

	} else {
		_actuators.control[0] = 0.0f;
		_actuators.control[1] = _manual_control_sp.x;
		_actuators.control[2] = _manual_control_sp.r;
		_actuators.control[3] = _manual_control_sp.z;
	}

	// note: _actuators.timestamp_sample is set in AirshipAttitudeControl::Run()
	_actuators.timestamp = hrt_absolute_time();

	_actuators_0_pub.publish(_actuators);
}

void AirshipAttitudeControl::publishTorqueSetpoint(const hrt_abstime &timestamp_sample)
{
	vehicle_torque_setpoint_s v_torque_sp = {};
	v_torque_sp.timestamp = hrt_absolute_time();
	v_torque_sp.timestamp_sample = timestamp_sample;
	v_torque_sp.xyz[0] = _actuators.control[0];
	v_torque_sp.xyz[1] = _actuators.control[1];
	v_torque_sp.xyz[2] = _actuators.control[2];

	_vehicle_torque_setpoint_pub.publish(v_torque_sp);
}

void AirshipAttitudeControl::publishThrustSetpoint(const hrt_abstime &timestamp_sample)
{
	vehicle_thrust_setpoint_s v_thrust_sp = {};
	v_thrust_sp.timestamp = hrt_absolute_time();
	v_thrust_sp.timestamp_sample = timestamp_sample;
	v_thrust_sp.xyz[0] = _actuators.control[3];
	v_thrust_sp.xyz[1] = 0.0f;
	v_thrust_sp.xyz[2] = 0.0f;

	_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
}

void
AirshipAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		_actuators.timestamp_sample = angular_velocity.timestamp_sample;

		/* run the rate controller immediately after a gyro update */
		publish_actuator_controls();
		publishTorqueSetpoint(angular_velocity.timestamp_sample);
		publishThrustSetpoint(angular_velocity.timestamp_sample);

		/* check for updates in manual control topic */
		_manual_control_sp_sub.update(&_manual_control_sp);

		/* check for updates in vehicle status topic */
		_vehicle_status_sub.update(&_vehicle_status);

		parameter_update_poll();
	}

	perf_end(_loop_perf);
}

int AirshipAttitudeControl::task_spawn(int argc, char *argv[])
{
	AirshipAttitudeControl *instance = new AirshipAttitudeControl();

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

int AirshipAttitudeControl::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_loop_perf);

	print_message(ORB_ID(actuator_controls), _actuators);

	return 0;
}

int AirshipAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AirshipAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the airship attitude and rate controller. Ideally it would
take attitude setpoints (`vehicle_attitude_setpoint`) or rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

Currently it is feeding the `manual_control_setpoint` topic directly to the actuators.

### Implementation
To reduce control latency, the module directly polls on the gyro topic published by the IMU driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("airship_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int airship_att_control_main(int argc, char *argv[])
{
	return AirshipAttitudeControl::main(argc, argv);
}
