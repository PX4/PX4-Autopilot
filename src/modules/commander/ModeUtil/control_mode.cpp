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
#include <uORB/topics/vehicle_status.h>

namespace mode_util
{

static bool stabilization_required(uint8_t vehicle_type)
{
	return vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
}

void getVehicleControlMode(bool armed, uint8_t nav_state, uint8_t vehicle_type,
			   const offboard_control_mode_s &offboard_control_mode,
			   vehicle_control_mode_s &vehicle_control_mode)
{
	vehicle_control_mode.flag_armed = armed;

	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		vehicle_control_mode.flag_control_manual_enabled = true;
		vehicle_control_mode.flag_control_rates_enabled = stabilization_required(vehicle_type);
		vehicle_control_mode.flag_control_attitude_enabled = stabilization_required(vehicle_type);
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		vehicle_control_mode.flag_control_manual_enabled = true;
		vehicle_control_mode.flag_control_rates_enabled = true;
		vehicle_control_mode.flag_control_attitude_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		vehicle_control_mode.flag_control_manual_enabled = true;
		vehicle_control_mode.flag_control_rates_enabled = true;
		vehicle_control_mode.flag_control_attitude_enabled = true;
		vehicle_control_mode.flag_control_altitude_enabled = true;
		vehicle_control_mode.flag_control_climb_rate_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		vehicle_control_mode.flag_control_manual_enabled = true;
		vehicle_control_mode.flag_control_rates_enabled = true;
		vehicle_control_mode.flag_control_attitude_enabled = true;
		vehicle_control_mode.flag_control_altitude_enabled = true;
		vehicle_control_mode.flag_control_climb_rate_enabled = true;
		vehicle_control_mode.flag_control_position_enabled = true;
		vehicle_control_mode.flag_control_velocity_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
		vehicle_control_mode.flag_control_auto_enabled = true;
		vehicle_control_mode.flag_control_rates_enabled = true;
		vehicle_control_mode.flag_control_attitude_enabled = true;
		vehicle_control_mode.flag_control_altitude_enabled = true;
		vehicle_control_mode.flag_control_climb_rate_enabled = true;
		vehicle_control_mode.flag_control_position_enabled = true;
		vehicle_control_mode.flag_control_velocity_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		vehicle_control_mode.flag_control_manual_enabled = true;
		vehicle_control_mode.flag_control_rates_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		vehicle_control_mode.flag_control_auto_enabled = true;
		vehicle_control_mode.flag_control_rates_enabled = true;
		vehicle_control_mode.flag_control_attitude_enabled = true;
		vehicle_control_mode.flag_control_climb_rate_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		/* disable all controllers on termination */
		vehicle_control_mode.flag_control_termination_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		vehicle_control_mode.flag_control_offboard_enabled = true;

		if (offboard_control_mode.position) {
			vehicle_control_mode.flag_control_position_enabled = true;
			vehicle_control_mode.flag_control_velocity_enabled = true;
			vehicle_control_mode.flag_control_altitude_enabled = true;
			vehicle_control_mode.flag_control_climb_rate_enabled = true;
			vehicle_control_mode.flag_control_acceleration_enabled = true;
			vehicle_control_mode.flag_control_rates_enabled = true;
			vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (offboard_control_mode.velocity) {
			vehicle_control_mode.flag_control_velocity_enabled = true;
			vehicle_control_mode.flag_control_altitude_enabled = true;
			vehicle_control_mode.flag_control_climb_rate_enabled = true;
			vehicle_control_mode.flag_control_acceleration_enabled = true;
			vehicle_control_mode.flag_control_rates_enabled = true;
			vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (offboard_control_mode.acceleration) {
			vehicle_control_mode.flag_control_acceleration_enabled = true;
			vehicle_control_mode.flag_control_rates_enabled = true;
			vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (offboard_control_mode.attitude) {
			vehicle_control_mode.flag_control_rates_enabled = true;
			vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (offboard_control_mode.body_rate) {
			vehicle_control_mode.flag_control_rates_enabled = true;
		}

		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:

	// Follow Target supports RC adjustment, so disable auto control mode to disable
	// the Flight Task from exiting itself when RC stick movement is detected.
	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		vehicle_control_mode.flag_control_manual_enabled = false;
		vehicle_control_mode.flag_control_auto_enabled = false;
		vehicle_control_mode.flag_control_rates_enabled = true;
		vehicle_control_mode.flag_control_attitude_enabled = true;
		vehicle_control_mode.flag_control_altitude_enabled = true;
		vehicle_control_mode.flag_control_climb_rate_enabled = true;
		vehicle_control_mode.flag_control_position_enabled = true;
		vehicle_control_mode.flag_control_velocity_enabled = true;
		break;

	default:
		break;
	}

	vehicle_control_mode.flag_multicopter_position_control_enabled =
		(vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)
		&& (vehicle_control_mode.flag_control_altitude_enabled
		    || vehicle_control_mode.flag_control_climb_rate_enabled
		    || vehicle_control_mode.flag_control_position_enabled
		    || vehicle_control_mode.flag_control_velocity_enabled
		    || vehicle_control_mode.flag_control_acceleration_enabled);
}

} // namespace mode_util
