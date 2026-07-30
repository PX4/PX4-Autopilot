/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

#include <uORB/topics/setpoint_config.h>
#include <uORB/topics/setpoint_config_reply.h>
#include <uORB/topics/vehicle_control_mode.h>

namespace mode_util
{
enum class SetpointType : uint16_t {
	Invalid = setpoint_config_s::TYPE_INVALID,
	DirectActuators = setpoint_config_s::TYPE_DIRECT_ACTUATORS,
	Goto = setpoint_config_s::TYPE_MULTICOPTER_GOTO,
	FixedwingLateralLongitudinal = setpoint_config_s::TYPE_FIXEDWING_LATERAL_LONGITUDINAL,
	Trajectory = setpoint_config_s::TYPE_TRAJECTORY,
	Rates = setpoint_config_s::TYPE_RATES,
	Attitude = setpoint_config_s::TYPE_ATTITUDE,
	RoverPosition = setpoint_config_s::TYPE_ROVER_POSITION,
	RoverSpeedAttitude = setpoint_config_s::TYPE_ROVER_SPEED_ATTITUDE,
	RoverSpeedRate = setpoint_config_s::TYPE_ROVER_SPEED_RATE,
	RoverSpeedSteering = setpoint_config_s::TYPE_ROVER_SPEED_STEERING,
	RoverThrottleAttitude = setpoint_config_s::TYPE_ROVER_THROTTLE_ATTITUDE,
	RoverThrottleRate = setpoint_config_s::TYPE_ROVER_THROTTLE_RATE,
	RoverThrottleSteering = setpoint_config_s::TYPE_ROVER_THROTTLE_STEERING,
	Trajectory_6dof = setpoint_config_s::TYPE_TRAJECTORY_6DOF,
	ThrustAndTorque = setpoint_config_s::TYPE_THRUST_AND_TORQUE,
	PositionTriplet = setpoint_config_s::TYPE_POSITION_TRIPLET,
};

/**
 * Fill in the required control mode flags based on a setpoint type.
 * Note that flags are not cleared, only set
 *
 * If a setpoint type has optional flags (e.g. position), it will be set here too.
 */
void getControlMode(SetpointType setpoint_type, vehicle_control_mode_s &control_mode);

enum class SetpointTypeResult : uint8_t {
	Success = setpoint_config_reply_s::RESULT_SUCCESS,
	FailureOther = setpoint_config_reply_s::RESULT_FAILURE_OTHER,
	Unknown = setpoint_config_reply_s::RESULT_UNKNOWN_SETPOINT_TYPE,
	Unsupported = setpoint_config_reply_s::RESULT_UNSUPPORTED,
};

/**
 * Check if a setpoint type is valid for a given vehicle type
 */
SetpointTypeResult isSetpointTypeValid(SetpointType setpoint_type, uint8_t vehicle_type);

} // namespace mode_util
