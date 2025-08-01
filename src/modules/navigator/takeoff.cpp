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
 * @file takeoff.cpp
 *
 * Helper class to Takeoff
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "takeoff.h"
#include "navigator.h"
#include <px4_platform_common/events.h>

Takeoff::Takeoff(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF)
{
}

void
Takeoff::on_activation()
{
	set_takeoff_position();

	// reset cruising speed to default
	_navigator->reset_cruising_speed();

	_fw_takeoff_state = fw_takeoff_state::CLIMBOUT; // only used for fixed-wing takeoff
}

void
Takeoff::on_active()
{
	if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {

		switch (_fw_takeoff_state) {
		case fw_takeoff_state::CLIMBOUT: {
				if (_navigator->get_global_position()->alt >= _loiter_altitude_msl) {

					setLoiterItemCommonFields(&_mission_item);

					_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;

					position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

					// we need the vehicle to loiter indefinitely but also we want this mission item to be reached as soon
					// as the loiter is established. therefore, set a small loiter time so that the mission item will be reached quickly,
					// however it will just continue loitering as there is no next mission item
					_mission_item.time_inside = 1.f;
					_mission_item.loiter_radius = _navigator->get_default_loiter_rad();
					_mission_item.acceptance_radius  = _navigator->get_acceptance_radius();
					_mission_item.altitude = _loiter_altitude_msl;

					mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
					pos_sp_triplet->current.lat = _loiter_position_lat_lon(0) > DBL_EPSILON ?
								      _loiter_position_lat_lon(0) : _navigator->get_global_position()->lat;
					pos_sp_triplet->current.lon = _loiter_position_lat_lon(1) > DBL_EPSILON ?
								      _loiter_position_lat_lon(1) : _navigator->get_global_position()->lon;
					pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
					pos_sp_triplet->current.cruising_speed = -1.f;
					pos_sp_triplet->current.cruising_throttle = -1.f;

					_mission_item.lat = pos_sp_triplet->current.lat;
					_mission_item.lon = pos_sp_triplet->current.lon;

					_navigator->set_position_setpoint_triplet_updated();

					reset_mission_item_reached();

					_fw_takeoff_state = fw_takeoff_state::GO_TO_LOITER;
				}

				break;
			}

		case fw_takeoff_state::GO_TO_LOITER: {
				const bool navigation_valid = _navigator->get_local_position()->xy_valid;
				bool lateral_acceptance_reached = true;

				if (navigation_valid) {
					// only consider lateral acceptance if position estimation is valid
					const float distance_to_loiter = get_distance_to_next_waypoint(_navigator->get_global_position()->lat,
									 _navigator->get_global_position()->lon, _mission_item.lat, _mission_item.lon);

					const float mission_item_loiter_radius_abs = (PX4_ISFINITE(_mission_item.loiter_radius)
							&& fabsf(_mission_item.loiter_radius) > FLT_EPSILON) ? fabsf(_mission_item.loiter_radius) :
							_navigator->get_default_loiter_rad();
					lateral_acceptance_reached = distance_to_loiter < _navigator->get_acceptance_radius() + mission_item_loiter_radius_abs;
				}

				const bool vertical_acceptance_reached = _navigator->get_global_position()->alt >= _loiter_altitude_msl -
						_navigator->get_altitude_acceptance_radius();

				if (lateral_acceptance_reached && vertical_acceptance_reached) {

					position_setpoint_triplet_s *reposition_triplet = _navigator->get_reposition_triplet();
					_navigator->reset_position_setpoint(reposition_triplet->previous);
					_navigator->reset_position_setpoint(reposition_triplet->current);
					_navigator->reset_position_setpoint(reposition_triplet->next);

					// the FW takeoff mode is completed, exit to Hold (handled by Commander)
					_navigator->get_mission_result()->finished = true;
					_navigator->set_mission_result_updated();
					_navigator->mode_completed(getNavigatorStateId());

					_loiter_altitude_msl = NAN; // reset for next takeoff command
				}

				break;
			}

		default:
			break;
		}

	} else { // rotary-wing takeoff
		struct position_setpoint_triplet_s *rep = _navigator->get_takeoff_triplet();

		if (rep->current.valid) {
			// reset the position
			set_takeoff_position();

		} else if (is_mission_item_reached_or_completed() && !_navigator->get_mission_result()->finished) {
			_navigator->get_mission_result()->finished = true;
			_navigator->set_mission_result_updated();
			_navigator->mode_completed(getNavigatorStateId());

			position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

			// set loiter item so position controllers stop doing takeoff logic
			if (_navigator->get_land_detected()->landed) {
				_mission_item.nav_cmd = NAV_CMD_IDLE;

			} else {
				if (pos_sp_triplet->current.valid
				    && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
					setLoiterItemFromCurrentPositionSetpoint(&_mission_item);
				}
			}

			mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
			_navigator->set_position_setpoint_triplet_updated();
		}
	}
}

void
Takeoff::set_takeoff_position()
{
	struct position_setpoint_triplet_s *rep = _navigator->get_takeoff_triplet();

	float takeoff_altitude_amsl = 0.f;

	if (rep->current.valid && PX4_ISFINITE(rep->current.alt)) {
		takeoff_altitude_amsl = rep->current.alt;

	} else {
		takeoff_altitude_amsl = _navigator->get_global_position()->alt + _navigator->get_param_mis_takeoff_alt();
		mavlink_log_info(_navigator->get_mavlink_log_pub(),
				 "Using default takeoff altitude: %.1f m\t", (double)_navigator->get_param_mis_takeoff_alt());

		events::send<float>(events::ID("navigator_takeoff_default_alt"), {events::Log::Info, events::LogInternal::Info},
				    "Using default takeoff altitude: {1:.2m}",
				    _navigator->get_param_mis_takeoff_alt());
	}

	if (takeoff_altitude_amsl < _navigator->get_global_position()->alt) {
		// If the suggestion is lower than our current alt, let's not go down.
		takeoff_altitude_amsl = _navigator->get_global_position()->alt;
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Already higher than takeoff altitude\t");
		events::send(events::ID("navigator_takeoff_already_higher"), {events::Log::Error, events::LogInternal::Info},
			     "Already higher than takeoff altitude (not descending)");
	}

	if (!PX4_ISFINITE(_loiter_altitude_msl)) {
		if (_navigator->get_loiter_min_alt() > FLT_EPSILON) {
			_loiter_altitude_msl = math::max(_loiter_altitude_msl, takeoff_altitude_amsl + _navigator->get_loiter_min_alt());

		}
	}

	// set current mission item to takeoff
	set_takeoff_item(&_mission_item, takeoff_altitude_amsl);
	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();
	reset_mission_item_reached();

	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->next.valid = false;

	if (rep->current.valid) {

		// Go on and check which changes had been requested
		if (PX4_ISFINITE(rep->current.yaw)) {
			pos_sp_triplet->current.yaw = rep->current.yaw;
		}

		// Set the current latitude and longitude even if they are NAN
		// NANs are handled in FlightTaskAuto.cpp
		pos_sp_triplet->current.lat = rep->current.lat;
		pos_sp_triplet->current.lon = rep->current.lon;

		// mark this as done
		memset(rep, 0, sizeof(*rep));
	}

	_navigator->set_position_setpoint_triplet_updated();
}
