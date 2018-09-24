/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file ams.cpp
 * Helper class for AMS (Auto Maneuver System)
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 */

#include "ams.h"
#include "navigator.h"


AMS::AMS(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
AMS::on_inactive()
{
	// Reset AMS state.
	_ams_state = AMSState::NONE;
}

void
AMS::on_activation()
{
	if (_navigator->get_land_detected()->landed) {
		// For safety reasons don't go into AMS if landed.
		_ams_state = AMSState::LANDED;

	} else {
		// Otherwise descend now
		_ams_state = AMSState::DESCEND;
	}

	set_ams_item();
}

void
AMS::on_active()
{
	bool fast_forward(false);

	// Stop the descend if we suddenly get altitude above ground data and we reached the AMS descend altitude target
	if (_ams_state == AMSState::DESCEND) {
		const vehicle_local_position_s &local_pos = *_navigator->get_local_position();
		if (local_pos.dist_bottom_valid) {
			if (local_pos.dist_bottom <= _param_ams_descend_alt.get()) {
				fast_forward = true;
			}
		}
	}

	if ((_ams_state != AMSState::LANDED && is_mission_item_reached()) || fast_forward) {
		advance_ams();
		set_ams_item();
	}

}

void
AMS::set_ams_item()
{
	_navigator->set_can_loiter_at_sp(false);

	const home_position_s &home = *_navigator->get_home_position();
	const vehicle_global_position_s &gpos = *_navigator->get_global_position();
	const vehicle_local_position_s &local_pos = *_navigator->get_local_position();

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// Compute the loiter altitude. Above ground if data is valid; above home otherwise
	const float altitude_above_ground = local_pos.dist_bottom_valid ? local_pos.dist_bottom : 0.0f;
	const float loiter_altitude = math::min(home.alt + altitude_above_ground + _param_ams_descend_alt.get(), gpos.alt);

	switch (_ams_state) {
	case AMSState::TRANSITION_TO_MC: {
			set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
			break;
		}

	case AMSState::DESCEND: {
			_latitude = gpos.lat;
			_longitude = gpos.lon;
			_altitude = loiter_altitude;

			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = _latitude;
			_mission_item.lon = _longitude;
			_mission_item.altitude = _altitude;
			_mission_item.altitude_is_relative = false;

			_mission_item.yaw = NAN;

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			// Disable previous setpoint to prevent drift.
			pos_sp_triplet->previous.valid = false;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "AMS: descend to %d m (%d m above home)",
						     (int)ceilf(_mission_item.altitude), (int)ceilf(_mission_item.altitude - home.alt));
			break;
		}

	case AMSState::LOITER: {
			_mission_item.lat = _latitude;
			_mission_item.lon = _longitude;
			_mission_item.altitude = _altitude;
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = false;
			_mission_item.origin = ORIGIN_ONBOARD;

			_navigator->set_can_loiter_at_sp(true);

			_mission_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "AMS: completed, loitering");

			break;
		}

	case AMSState::LAND: {
			// Land at home position.
			_mission_item.nav_cmd = NAV_CMD_LAND;
			_mission_item.lat = _latitude;
			_mission_item.lon = _longitude;
			_mission_item.yaw = NAN;
			_mission_item.altitude = _altitude;
			_mission_item.altitude_is_relative = false;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "AMS: landing");
			break;
		}

	case AMSState::LANDED: {
			set_idle_item(&_mission_item);
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	// Execute command if set. This is required for commands like VTOL transition.
	if (!item_contains_position(_mission_item)) {
		issue_command(_mission_item);
	}

	// Convert mission item to current position setpoint and make it valid.
	mission_apply_limitation(_mission_item);

	if (mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current)) {
		_navigator->set_position_setpoint_triplet_updated();
	}
}

void
AMS::advance_ams()
{
	switch (_ams_state) {

	case AMSState::TRANSITION_TO_MC:
		_ams_state = AMSState::DESCEND;
		break;

	case AMSState::DESCEND:

		if (ams_type() == AMSType::LOITER) {
			_ams_state = AMSState::LOITER;

		} else if (ams_type() == AMSType::LAND) {
			_ams_state = AMSState::LAND;

		} else if (ams_type() == AMSType::RALLY) {
			_ams_state = AMSState::RALLY;

		} else { // unknown, just loiter
			_ams_state = AMSState::LOITER;
		}

		break;

	case AMSState::RALLY:
		_ams_state = AMSState::LAND;

	case AMSState::LAND:
		_ams_state = AMSState::LANDED;
		break;

	default:
		break;
	}
}
