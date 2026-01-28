/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file loiter.cpp
 *
 * Helper class to loiter
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "loiter.h"
#include "navigator.h"

Loiter::Loiter(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER),
	ModuleParams(navigator)
{
}

void
Loiter::on_activation()
{
	if (_navigator->get_reposition_triplet()->current.valid
	    && hrt_elapsed_time(&_navigator->get_reposition_triplet()->current.timestamp) < 500_ms) {
		reposition();

	} else {
		// this is executed when the flight mode is switched to Hold manually, not through a reposition
		set_loiter_position();
	}

	// reset cruising speed to default
	_navigator->reset_cruising_speed();
}

void
Loiter::on_active()
{
	if (_navigator->get_reposition_triplet()->current.valid
	    && hrt_elapsed_time(&_navigator->get_reposition_triplet()->current.timestamp) < 500_ms) {
		reposition();
	}
}

void
Loiter::set_loiter_position()
{
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED &&
	    _navigator->get_land_detected()->landed) {

		// Not setting loiter position if disarmed and landed, instead mark the current
		// setpoint as invalid and idle (both, just to be sure).

		_navigator->get_position_setpoint_triplet()->current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
		_navigator->set_position_setpoint_triplet_updated();
		return;

	}

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	if (_navigator->get_land_detected()->landed) {
		_mission_item.nav_cmd = NAV_CMD_IDLE;

	} else {
		// Check if we already loiter on a circle and are on the loiter pattern.
		bool on_loiter{false};

		if (pos_sp_triplet->current.valid && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER
		    && pos_sp_triplet->current.loiter_pattern == position_setpoint_s::LOITER_TYPE_ORBIT) {
			const float d_current = get_distance_to_next_waypoint(pos_sp_triplet->current.lat, pos_sp_triplet->current.lon,
						_navigator->get_global_position()->lat, _navigator->get_global_position()->lon);
			on_loiter = d_current <= (_navigator->get_acceptance_radius() + pos_sp_triplet->current.loiter_radius);

		}

		if (on_loiter) {
			setLoiterItemFromCurrentPositionSetpoint(&_mission_item);

		} else if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			setLoiterItemFromCurrentPositionWithBraking(&_mission_item);

		} else {
			setLoiterItemFromCurrentPosition(&_mission_item);
		}

	}

	// convert mission item to current setpoint
	pos_sp_triplet->previous.valid = false;
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

void
Loiter::reposition()
{
	// we can't reposition if we are not armed yet
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		return;
	}

	struct position_setpoint_triplet_s *rep = _navigator->get_reposition_triplet();

	if (rep->current.valid) {
		// set loiter position based on reposition command

		// convert mission item to current setpoint
		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		pos_sp_triplet->previous.yaw = _navigator->get_local_position()->heading;
		pos_sp_triplet->previous.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->previous.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->previous.alt = _navigator->get_global_position()->alt;
		memcpy(&pos_sp_triplet->current, &rep->current, sizeof(rep->current));
		pos_sp_triplet->next.valid = false;

		_navigator->set_position_setpoint_triplet_updated();

		// mark this as done
		memset(rep, 0, sizeof(*rep));
	}
}
