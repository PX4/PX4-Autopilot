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

#ifndef MODULE_NAME
#define MODULE_NAME "ModeUtil"
#endif

#include "setpoint_types.hpp"
#include <uORB/topics/vehicle_status.h>
#include <px4_platform_common/log.h>

namespace mode_util
{
void getControlMode(SetpointType setpoint_type, vehicle_control_mode_s &control_mode)
{
	switch (setpoint_type) {
	case SetpointType::Invalid:
		PX4_ERR("Invalid setpoint type");
		break;

	case SetpointType::DirectActuators:
		// Nothing enabled
		break;

	case SetpointType::Goto:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_acceleration_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		control_mode.flag_control_position_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		break;

	case SetpointType::FixedwingLateralLongitudinal:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_acceleration_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		control_mode.flag_control_position_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		break;

	case SetpointType::Trajectory:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_acceleration_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		control_mode.flag_control_position_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		break;

	case SetpointType::Rates:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		break;

	case SetpointType::Attitude:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		break;

	case SetpointType::RoverPosition:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_acceleration_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		control_mode.flag_control_position_enabled = true;
		break;

	case SetpointType::RoverSpeedAttitude:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_acceleration_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		break;

	case SetpointType::RoverSpeedRate:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_acceleration_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		break;

	case SetpointType::RoverSpeedSteering:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_acceleration_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		break;

	case SetpointType::RoverThrottleAttitude:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_acceleration_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		break;

	case SetpointType::RoverThrottleRate:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_acceleration_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		break;

	case SetpointType::RoverThrottleSteering:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_acceleration_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		break;

	case SetpointType::Trajectory_6dof:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_acceleration_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		control_mode.flag_control_position_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		break;

	case SetpointType::ThrustAndTorque:
		control_mode.flag_control_allocation_enabled = true;
		break;

	case SetpointType::PositionTriplet:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_acceleration_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		control_mode.flag_control_position_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_auto_enabled = true;
		break;
	}
}

SetpointTypeResult isSetpointTypeValid(SetpointType setpoint_type, uint8_t vehicle_type)
{
	switch (setpoint_type) {
	case SetpointType::Invalid:
		return SetpointTypeResult::Unknown;

	case SetpointType::DirectActuators:
		return SetpointTypeResult::Success;

	case SetpointType::Goto:
		if (vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			return SetpointTypeResult::Success;
		}

		return SetpointTypeResult::Unsupported;

	case SetpointType::FixedwingLateralLongitudinal:
		if (vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
			return SetpointTypeResult::Success;
		}

		return SetpointTypeResult::Unsupported;

	case SetpointType::Trajectory:
	case SetpointType::Rates:
	case SetpointType::Attitude:
		return SetpointTypeResult::Success;

	case SetpointType::RoverPosition:
	case SetpointType::RoverSpeedAttitude:
	case SetpointType::RoverSpeedRate:
	case SetpointType::RoverSpeedSteering:
	case SetpointType::RoverThrottleAttitude:
	case SetpointType::RoverThrottleRate:
	case SetpointType::RoverThrottleSteering:
		if (vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER) {
			return SetpointTypeResult::Success;
		}

		return SetpointTypeResult::Unsupported;

	case SetpointType::Trajectory_6dof:

		// Used by spacecraft but that does not seem to set the vehicle_type
		if (vehicle_type == vehicle_status_s::VEHICLE_TYPE_UNSPECIFIED) {
			return SetpointTypeResult::Success;
		}

		return SetpointTypeResult::Unsupported;

	case SetpointType::ThrustAndTorque:
		return SetpointTypeResult::Success;

	case SetpointType::PositionTriplet:
		return SetpointTypeResult::Success;
	}

	return SetpointTypeResult::Unknown;
}
} // namespace mode_util
