/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "control_mode.hpp"
#include "setpoint_types.hpp"
#include <uORB/topics/vehicle_status.h>

namespace mode_util
{

static bool stabilization_required(uint8_t vehicle_type)
{
	return vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
}

void getVehicleControlMode(uint8_t nav_state, uint8_t vehicle_type,
			   const offboard_control_mode_s &offboard_control_mode,
			   vehicle_control_mode_s &vehicle_control_mode, SetpointType external_mode_setpoint_type)
{

	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		vehicle_control_mode.flag_control_manual_enabled = true;

		if (stabilization_required(vehicle_type)) {
			getControlMode(SetpointType::Attitude, vehicle_control_mode);

		} else {
			getControlMode(SetpointType::ThrustAndTorque, vehicle_control_mode);
		}

		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		vehicle_control_mode.flag_control_manual_enabled = true;
		getControlMode(SetpointType::Attitude, vehicle_control_mode);
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
	case vehicle_status_s::NAVIGATION_STATE_ALTITUDE_CRUISE:
		vehicle_control_mode.flag_control_manual_enabled = true;
		getControlMode(SetpointType::Trajectory, vehicle_control_mode);
		vehicle_control_mode.flag_control_velocity_enabled = false;
		vehicle_control_mode.flag_control_position_enabled = false;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
	case vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW:
		vehicle_control_mode.flag_control_manual_enabled = true;
		getControlMode(SetpointType::Trajectory, vehicle_control_mode);
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_GUIDED_COURSE:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
		getControlMode(SetpointType::PositionTriplet, vehicle_control_mode);
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		vehicle_control_mode.flag_control_manual_enabled = true;
		getControlMode(SetpointType::Rates, vehicle_control_mode);
		break;

	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		vehicle_control_mode.flag_control_auto_enabled = true;
		vehicle_control_mode.flag_control_climb_rate_enabled = true;
		vehicle_control_mode.flag_control_attitude_enabled = true;
		vehicle_control_mode.flag_control_rates_enabled = true;
		vehicle_control_mode.flag_control_allocation_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		/* disable all controllers on termination */
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		vehicle_control_mode.flag_control_offboard_enabled = true;

		if (offboard_control_mode.position) {
			getControlMode(SetpointType::Trajectory, vehicle_control_mode);

		} else if (offboard_control_mode.velocity) {
			getControlMode(SetpointType::Trajectory, vehicle_control_mode);
			vehicle_control_mode.flag_control_position_enabled = false;

		} else if (offboard_control_mode.acceleration) {
			getControlMode(SetpointType::Trajectory, vehicle_control_mode);
			// There is no dedicated acceleration flag. To make sure the right controllers run, we set the
			// velocity flag.
			vehicle_control_mode.flag_control_velocity_enabled = true;
			vehicle_control_mode.flag_control_position_enabled = false;
			vehicle_control_mode.flag_control_altitude_enabled = false;
			vehicle_control_mode.flag_control_climb_rate_enabled = false;

		} else if (offboard_control_mode.attitude) {
			getControlMode(SetpointType::Attitude, vehicle_control_mode);

		} else if (offboard_control_mode.body_rate) {
			getControlMode(SetpointType::Rates, vehicle_control_mode);

		} else if (offboard_control_mode.thrust_and_torque) {
			getControlMode(SetpointType::ThrustAndTorque, vehicle_control_mode);

		} else if (offboard_control_mode.direct_actuator) {
			getControlMode(SetpointType::DirectActuators, vehicle_control_mode);
		}

		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:

	// Follow Target supports RC adjustment, so disable auto control mode to disable
	// the Flight Task from exiting itself when RC stick movement is detected.
	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		vehicle_control_mode.flag_control_manual_enabled = false;
		getControlMode(SetpointType::Trajectory, vehicle_control_mode);
		break;

	case vehicle_status_s::NAVIGATION_STATE_EXTERNAL1 ... vehicle_status_s::NAVIGATION_STATE_EXTERNAL8:
		getControlMode(external_mode_setpoint_type, vehicle_control_mode);
		break;

	default:
		break;
	}

}

} // namespace mode_util
