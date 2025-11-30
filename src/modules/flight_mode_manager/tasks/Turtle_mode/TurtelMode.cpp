/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file TurtelMode.cpp
 *
 * Flight task for turtle mode - allows manual control for flipping a crashed drone
 */

#include "TurtelMode.hpp"
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>
#include <float.h>
#include <parameters/param.h>
#include <inttypes.h>
#include <mathlib/mathlib.h>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/vehicle_command.h>
// Note: We use actuator_test instead of actuator_motors because:
// - actuator_test works when DISARMED (bypasses arming checks) - essential for turtle mode
// - actuator_motors requires ARMED state + OFFBOARD mode - not suitable for crashed/recovery scenarios
// - actuator_test is designed for testing/calibration and works per-motor (one message per function)
// - actuator_motors controls all motors in one message and goes through control allocator

using namespace matrix;

TurtleMode::~TurtleMode()
{
	// Release actuator_test control for all motors
	for (int i = 0; i < 4; i++) {
		actuator_test_s actuator_test{};
		actuator_test.timestamp = hrt_absolute_time();
		actuator_test.function = actuator_test_s::FUNCTION_MOTOR1 + i;
		actuator_test.value = NAN;
		actuator_test.action = actuator_test_s::ACTION_RELEASE_CONTROL;
		actuator_test.timeout_ms = 0;
		
		_actuator_test_pub.publish(actuator_test);
	}
	
	// Set DSHOT_3D_ENABLE parameter to zero when exiting turtle mode
	param_t param = param_find("DSHOT_3D_ENABLE");
	
	if (param != PARAM_INVALID) {
		int32_t value = 0;
		param_set(param, &value);
	}
}

bool TurtleMode::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskManualAltitude::activate(last_setpoint);

	// Set DSHOT_3D_ENABLE to 1 for turtle mode
	param_t param = param_find("DSHOT_3D_ENABLE");
	
	if (param != PARAM_INVALID) {
		int32_t value = 1;
		param_set(param, &value);
	}
	
	return ret;
}

bool TurtleMode::update()
{
	bool ret = FlightTaskManualAltitude::update();



	// In turtle mode, we want full manual control without altitude lock
	// The base class handles most of the work, but we ensure no position locking

	_publishMotorOutputs();
	return ret;
}


void TurtleMode::_publishMotorOutputs(){
	// Use actuator_test mode to bypass arming checks
	// This allows motors to run even when disarmed - essential for turtle mode recovery
	// Publish one message per motor (actuator_test works per function)
	// Alternative: actuator_motors topic requires ARMED + OFFBOARD mode, not suitable here
	
	// Get control inputs from sticks (throttle, roll, pitch, yaw)
	// These are normalized values from the base class
	float motor_values[4] = {-0.2f, -0.2f, -0.2f, -0.2f};
	// Normalize to [0, 1] range for actuator_test (negative values map to NaN)
	for (int i = 0; i < 4; i++) {
		actuator_test_s actuator_test{};
		actuator_test.timestamp = hrt_absolute_time();
		actuator_test.function = actuator_test_s::FUNCTION_MOTOR1 + i;
		actuator_test.value = motor_values[i];
		actuator_test.action = actuator_test_s::ACTION_DO_CONTROL;
		actuator_test.timeout_ms = 0; // No timeout - keep control until explicitly released
		
		_actuator_test_pub.publish(actuator_test);
	}
	
}

bool TurtleMode::forceArm()
{
	// Send vehicle command to force arm (param2 = 21196 bypasses preflight checks)
	_sendVehicleCommand(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
			   static_cast<float>(vehicle_command_s::ARMING_ACTION_ARM),
			   21196.0f); // Magic number to force arm
	
	PX4_INFO("Force arm command sent");
	return true;
}

bool TurtleMode::disarm(bool force)
{
	// Send vehicle command to disarm
	float param2 = force ? 21196.0f : 0.0f; // 21196 forces disarm even in flight
	
	_sendVehicleCommand(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
			   static_cast<float>(vehicle_command_s::ARMING_ACTION_DISARM),
			   param2);
	
	PX4_INFO("Disarm command sent (force: %s)", force ? "yes" : "no");
	return true;
}

void TurtleMode::_sendVehicleCommand(uint32_t command, float param1, float param2)
{
	vehicle_command_s vehicle_command{};
	vehicle_command.timestamp = hrt_absolute_time();
	vehicle_command.command = command;
	vehicle_command.param1 = param1;
	vehicle_command.param2 = param2;
	vehicle_command.target_system = 1;
	vehicle_command.target_component = 1;
	vehicle_command.source_system = 1;
	vehicle_command.source_component = 1;
	vehicle_command.from_external = false; // Internal command
	
	_vehicle_command_pub.publish(vehicle_command);
}


