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
/**
 * @file vtol_takeoff.cpp
 *
 * Helper class to do a VTOL takeoff and transition into a loiter.
 *
 */

#include "vtol_takeoff.h"
#include "navigator.h"

using matrix::wrap_pi;

VtolTakeoff::VtolTakeoff(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
VtolTakeoff::on_activation()
{
	if (hrt_elapsed_time(&_navigator->get_global_position()->timestamp) < 1_s) {
		set_takeoff_position();
		_takeoff_state = vtol_takeoff_state::TAKEOFF_HOVER;
		_navigator->reset_cruising_speed();
		_navigator->set_cruising_throttle();
	}
}

void
VtolTakeoff::on_active()
{
	if (is_mission_item_reached_or_completed()) {
		reset_mission_item_reached();

		switch	(_takeoff_state) {
		case vtol_takeoff_state::TAKEOFF_HOVER: {

				position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

				_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
				_mission_item.yaw = wrap_pi(get_bearing_to_next_waypoint(_mission_item.lat,
							    _mission_item.lon, _loiter_location(0), _loiter_location(1)));
				_mission_item.force_heading = true;
				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
				pos_sp_triplet->current.cruising_speed = -1.f;
				_navigator->set_position_setpoint_triplet_updated();

				_takeoff_state = vtol_takeoff_state::ALIGN_HEADING;

				break;
			}

		case vtol_takeoff_state::ALIGN_HEADING: {

				set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
				_mission_item.lat = _loiter_location(0);
				_mission_item.lon = _loiter_location(1);
				position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
				pos_sp_triplet->previous = pos_sp_triplet->current;

				_navigator->set_position_setpoint_triplet_updated();

				issue_command(_mission_item);

				_takeoff_state = vtol_takeoff_state::TRANSITION;

				break;
			}

		case vtol_takeoff_state::TRANSITION: {
				position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

				if (pos_sp_triplet->current.valid && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
					setLoiterItemFromCurrentPositionSetpoint(&_mission_item);

				} else {
					setLoiterItemFromCurrentPosition(&_mission_item);
				}

				_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;

				// we need the vehicle to loiter indefinitely but also we want this mission item to be reached as soon
				// as the loiter is established. therefore, set a small loiter time so that the mission item will be reached quickly,
				// however it will just continue loitering as there is no next mission item
				_mission_item.time_inside = 1.f;
				_mission_item.loiter_radius = _navigator->get_loiter_radius();
				_mission_item.acceptance_radius  = _navigator->get_acceptance_radius();
				_mission_item.altitude = _navigator->get_home_position()->alt + _param_loiter_alt.get();

				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
				pos_sp_triplet->current.lat = _loiter_location(0);
				pos_sp_triplet->current.lon = _loiter_location(1);
				pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
				pos_sp_triplet->current.cruising_speed = -1.f;
				pos_sp_triplet->current.cruising_throttle = -1.f;

				_mission_item.lat = pos_sp_triplet->current.lat;
				_mission_item.lon = pos_sp_triplet->current.lon;

				_navigator->set_position_setpoint_triplet_updated();

				reset_mission_item_reached();

				_takeoff_state = vtol_takeoff_state::CLIMB;

				break;
			}

		case vtol_takeoff_state::CLIMB: {

				// reset any potentially valid reposition triplet which was not handled
				// we do this to avoid random loiter locations after switching to loiter mode after this
				position_setpoint_triplet_s *reposition_triplet = _navigator->get_reposition_triplet();
				_navigator->reset_position_setpoint(reposition_triplet->previous);
				_navigator->reset_position_setpoint(reposition_triplet->current);
				_navigator->reset_position_setpoint(reposition_triplet->next);

				// the VTOL takeoff is done
				_navigator->get_mission_result()->finished = true;
				_navigator->set_mission_result_updated();
				_navigator->mode_completed(vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF);

				break;
			}

		default: {

				break;
			}
		}
	}
}

void
VtolTakeoff::set_takeoff_position()
{
	// set current mission item to takeoff
	set_takeoff_item(&_mission_item, _transition_alt_amsl);

	_mission_item.lat = _navigator->get_global_position()->lat;
	_mission_item.lon = _navigator->get_global_position()->lon;

	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();

	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}
