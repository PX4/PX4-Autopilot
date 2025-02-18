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

#include "mode_requirements.hpp"
#include <uORB/topics/vehicle_status.h>

namespace mode_util
{

static inline void setRequirement(uint8_t nav_state, uint32_t &mode_requirement)
{
	mode_requirement |= 1u << nav_state;
}


void getModeRequirements(uint8_t vehicle_type, failsafe_flags_s &flags)
{
	flags.mode_req_angular_velocity = 0;
	flags.mode_req_attitude = 0;
	flags.mode_req_local_position = 0;
	flags.mode_req_local_position_relaxed = 0;
	flags.mode_req_global_position = 0;
	flags.mode_req_local_alt = 0;
	flags.mode_req_mission = 0;
	flags.mode_req_offboard_signal = 0;
	flags.mode_req_home_position = 0;
	flags.mode_req_wind_and_flight_time_compliance = 0;
	flags.mode_req_prevent_arming = 0;
	flags.mode_req_manual_control = 0;
	flags.mode_req_other = 0;

	// NAVIGATION_STATE_MANUAL
	setRequirement(vehicle_status_s::NAVIGATION_STATE_MANUAL, flags.mode_req_manual_control);

	// NAVIGATION_STATE_ALTCTL
	setRequirement(vehicle_status_s::NAVIGATION_STATE_ALTCTL, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_ALTCTL, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_ALTCTL, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_ALTCTL, flags.mode_req_manual_control);

	// NAVIGATION_STATE_POSCTL
	setRequirement(vehicle_status_s::NAVIGATION_STATE_POSCTL, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_POSCTL, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_POSCTL, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_POSCTL, flags.mode_req_local_position_relaxed);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_POSCTL, flags.mode_req_manual_control);

	// NAVIGATION_STATE_POSITION_SLOW
	setRequirement(vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW, flags.mode_req_local_position_relaxed);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW, flags.mode_req_manual_control);

	// NAVIGATION_STATE_AUTO_MISSION
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, flags.mode_req_global_position);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, flags.mode_req_mission);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, flags.mode_req_wind_and_flight_time_compliance);

	// NAVIGATION_STATE_AUTO_LOITER
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER, flags.mode_req_global_position);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER, flags.mode_req_wind_and_flight_time_compliance);

	// NAVIGATION_STATE_AUTO_RTL
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_RTL, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_RTL, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_RTL, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_RTL, flags.mode_req_global_position);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_RTL, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_RTL, flags.mode_req_home_position);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_RTL, flags.mode_req_prevent_arming);

	// NAVIGATION_STATE_ACRO
	setRequirement(vehicle_status_s::NAVIGATION_STATE_ACRO, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_ACRO, flags.mode_req_manual_control);

	// NAVIGATION_STATE_DESCEND
	setRequirement(vehicle_status_s::NAVIGATION_STATE_DESCEND, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_DESCEND, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_DESCEND, flags.mode_req_prevent_arming);

	// NAVIGATION_STATE_TERMINATION
	setRequirement(vehicle_status_s::NAVIGATION_STATE_TERMINATION, flags.mode_req_prevent_arming);

	// NAVIGATION_STATE_OFFBOARD
	setRequirement(vehicle_status_s::NAVIGATION_STATE_OFFBOARD, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_OFFBOARD, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_OFFBOARD, flags.mode_req_offboard_signal);

	// NAVIGATION_STATE_STAB
	setRequirement(vehicle_status_s::NAVIGATION_STATE_STAB, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_STAB, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_STAB, flags.mode_req_manual_control);

	// NAVIGATION_STATE_AUTO_TAKEOFF
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF, flags.mode_req_local_alt);

	// NAVIGATION_STATE_AUTO_LAND
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_LAND, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_LAND, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_LAND, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_LAND, flags.mode_req_local_position_relaxed);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_LAND, flags.mode_req_prevent_arming);

	// NAVIGATION_STATE_AUTO_FOLLOW_TARGET
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET, flags.mode_req_prevent_arming);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET, flags.mode_req_wind_and_flight_time_compliance);

	// NAVIGATION_STATE_AUTO_PRECLAND
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND, flags.mode_req_prevent_arming);

	// NAVIGATION_STATE_ORBIT
	setRequirement(vehicle_status_s::NAVIGATION_STATE_ORBIT, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_ORBIT, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_ORBIT, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_ORBIT, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_ORBIT, flags.mode_req_prevent_arming);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_ORBIT, flags.mode_req_wind_and_flight_time_compliance);

	// NAVIGATION_STATE_AUTO_VTOL_TAKEOFF
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF, flags.mode_req_local_alt);

	// NAVIGATION_STATE_EXTERNALx: handled outside

	static_assert(vehicle_status_s::NAVIGATION_STATE_MAX == 31, "update mode requirements");
}

} // namespace mode_util
