/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file takeoff_no_nav.cpp
 *
 * Helper class to do a takeoff without lateral navigation
 *
 */

#include "takeoff_no_nav.h"
#include "navigator.h" //TODO remove?

TakeoffNoNav::TakeoffNoNav(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF_NO_NAV)
{
}

void TakeoffNoNav::on_activation()
{
	_navigator->reset_cruising_speed();
	_navigator->set_cruising_throttle();
	set_takeoff_position();
}

void TakeoffNoNav::on_active()
{
	if (_local_altitude_at_takeoff - _navigator->get_local_position()->z > _navigator->get_param_mis_takeoff_alt()) {
		// Takeoff done - reset any potentially valid reposition triplet which was not handled.
		// We do this to avoid random loiter locations after switching to loiter mode after this.
		position_setpoint_triplet_s *reposition_triplet = _navigator->get_reposition_triplet();
		_navigator->reset_position_setpoint(reposition_triplet->previous);
		_navigator->reset_position_setpoint(reposition_triplet->current);
		_navigator->reset_position_setpoint(reposition_triplet->next);

		_navigator->get_mission_result()->finished = true;
		_navigator->set_mission_result_updated();
		_navigator->mode_completed(getNavigatorStateId());
	}
}

void TakeoffNoNav::set_takeoff_position()
{
	// for now only support takeoff to MIS_TAKEOFF_ALT //TODO add mavlink interface

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	_local_altitude_at_takeoff = _navigator->get_local_position()->z;

	const float takeoff_altitude_amsl = _navigator->get_global_position()->alt + _navigator->get_param_mis_takeoff_alt();
	mavlink_log_info(_navigator->get_mavlink_log_pub(),
			 "Using default takeoff altitude: %.1f m\t", (double)_navigator->get_param_mis_takeoff_alt());

	events::send<float>(events::ID("navigator_takeoff_no_nav_default_alt"), {events::Log::Info, events::LogInternal::Info},
			    "Using default takeoff altitude: {1:.2m}",
			    _navigator->get_param_mis_takeoff_alt());

	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();

	pos_sp_triplet->current.lat = NAN;
	pos_sp_triplet->current.lon = NAN;
	pos_sp_triplet->current.alt =
		takeoff_altitude_amsl; // need to think about how to interface with the controller without global position
	pos_sp_triplet->current.yaw = NAN;
	pos_sp_triplet->current.loiter_radius = NAN;
	pos_sp_triplet->current.loiter_direction_counter_clockwise = NAN;
	pos_sp_triplet->current.acceptance_radius = NAN;
	pos_sp_triplet->current.alt_acceptance_radius = NAN;
	pos_sp_triplet->current.cruising_speed = NAN;
	pos_sp_triplet->current.cruising_throttle = NAN;
	pos_sp_triplet->current.gliding_enabled = false;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;
	pos_sp_triplet->current.valid = true;
	pos_sp_triplet->current.timestamp = hrt_absolute_time();

	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}
