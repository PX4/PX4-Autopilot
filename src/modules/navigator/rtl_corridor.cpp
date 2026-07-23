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
/**
 * @file rtl_corridor.cpp
 */

#include <float.h>

#include "rtl_corridor.h"
#include "navigator.h"

#include <lib/geo/geo.h>
#include <matrix/Vector2.hpp>

RtlCorridor::RtlCorridor(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL),
	ModuleParams(navigator)
{
}

void RtlCorridor::on_activation()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();

	_current_index = 0;
	_reached_final_waypoint = (_num_waypoints == 0);

	_navigator->reset_cruising_speed();
	_navigator->set_cruising_throttle();

	if (!_reached_final_waypoint) {
		setCurrentWaypointItem();
	}
}

void RtlCorridor::on_active()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();

	if (_reached_final_waypoint) {
		return;
	}

	if (is_mission_item_reached_or_completed()) {
		_current_index++;

		if (_current_index >= _num_waypoints) {
			// Reached the final node (the standoff point above the landing pad).
			_reached_final_waypoint = true;

		} else {
			setCurrentWaypointItem();
		}
	}
}

void RtlCorridor::on_inactivation()
{
}

void RtlCorridor::on_inactive()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();
}

void RtlCorridor::setCurrentWaypointItem()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	const mission_corridor_node_s &node = _waypoints[_current_index];

	PositionYawSetpoint pos_yaw_sp {
		.lat = node.lat,
		.lon = node.lon,
		.alt = node.alt,
		.yaw = NAN,
	};

	setMoveToPositionMissionItem(_mission_item, pos_yaw_sp);

	reset_mission_item_reached();

	if (mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current)) {
		_navigator->set_position_setpoint_triplet_updated();
	}
}

void RtlCorridor::setPath(const mission_corridor_node_s *waypoints, uint8_t num_waypoints)
{
	// Only allow setting a new path if not activated yet, matching
	// RtlDirect::setRtlPosition()'s contract.
	if (isActive()) {
		return;
	}

	static constexpr uint8_t kMaxWaypoints = static_cast<uint8_t>(DM_KEY_CORRIDOR_NODES_MAX);
	_num_waypoints = (num_waypoints < kMaxWaypoints) ? num_waypoints : kMaxWaypoints;

	for (uint8_t i = 0; i < _num_waypoints; i++) {
		_waypoints[i] = waypoints[i];
	}

	// Reset traversal state so a fresh path starts from the beginning. This matters for the
	// time estimate consumed by the battery failsafe while RTL is inactive: without it, a
	// _current_index / _reached_final_waypoint left over from a previous activation would make
	// calc_rtl_time_estimate() sum only a tail of the path and under-estimate the return time.
	_current_index = 0;
	_reached_final_waypoint = false;
}

rtl_time_estimate_s RtlCorridor::calc_rtl_time_estimate()
{
	_global_pos_sub.update();
	_rtl_time_estimator.update();
	_rtl_time_estimator.setVehicleType(_vehicle_status_sub.get().vehicle_type);
	_rtl_time_estimator.reset();

	if (_num_waypoints == 0) {
		return _rtl_time_estimator.getEstimate();
	}

	// Sum the remaining corridor legs from the current position through to the
	// final node (the nest or the chosen rally point).
	double from_lat = _global_pos_sub.get().lat;
	double from_lon = _global_pos_sub.get().lon;
	float from_alt = _global_pos_sub.get().alt;

	const uint8_t start_index = _reached_final_waypoint ? static_cast<uint8_t>(_num_waypoints - 1) : _current_index;

	for (uint8_t i = start_index; i < _num_waypoints; i++) {
		const mission_corridor_node_s &node = _waypoints[i];

		matrix::Vector2f direction{};
		get_vector_to_next_waypoint(from_lat, from_lon, node.lat, node.lon, &direction(0), &direction(1));
		const float hor_dist = get_distance_to_next_waypoint(from_lat, from_lon, node.lat, node.lon);

		_rtl_time_estimator.addDistance(hor_dist, direction, 0.f);
		_rtl_time_estimator.addVertDistance(node.alt - from_alt);

		from_lat = node.lat;
		from_lon = node.lon;
		from_alt = node.alt;
	}

	// Approximate the terminal descent + landing at the final node. The node is a standoff point
	// above the landing pad; the vehicle descends from there to the ground (touchdown is governed
	// by the land detector). The pad's true ground elevation is unknown, so approximate the
	// descent by the node's height above home -- the reference the rest of RTL uses -- when home
	// is valid, else fall back to RTL_DESCEND_ALT. This replaces a previous model that added a
	// fictitious climb of RTL_DESCEND_ALT above the node and omitted the real descent, which
	// under-estimated (fired the battery failsafe late) for standoff nodes well above the pad.
	// It stays a rough estimate -- the failsafe carries its own RTL_TIME_FACTOR / RTL_TIME_MARGIN
	// padding -- and does not replicate RtlDirect's VTOL-transition / precision-land branching.
	float terminal_descent_m = _param_rtl_descend_alt.get();

	if (_navigator->home_global_position_valid()) {
		terminal_descent_m = fmaxf(0.f, from_alt - _navigator->get_home_position()->alt);
	}

	_rtl_time_estimator.addWait(_param_rtl_land_delay.get());
	_rtl_time_estimator.addVertDistance(-terminal_descent_m); // negative -> descent

	return _rtl_time_estimator.getEstimate();
}
