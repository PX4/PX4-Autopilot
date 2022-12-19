/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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


#include "terrain_follower.hpp"
#include <lib/geo/geo.h>

using Vector2d = matrix::Vector2<double>;
using Vector2f = matrix::Vector2<float>;

TerrainFollower::TerrainFollower() :
	ModuleParams(nullptr)
{
	param_handle_fw_climbrate_max = param_find("FW_T_CLMB_MAX");
}

TerrainFollower::~TerrainFollower()
{

}

void TerrainFollower::updateParams()
{
	ModuleParams::updateParams();
	param_get(param_handle_fw_climbrate_max, &param_fw_climbrate_max);
}

void TerrainFollower::reset()
{
	_current_mission_item_valid = false;
	_has_intermediate_mission_item = false;
	_dist_to_current_mission_item = 0.0f;
	_pos_lat_lon.setZero();
	_alt_amsl_m = 0.0f;
	_alt_home_amsl_m = 0.0f;
	_ground_speed_m_s = 0.0f;
	_had_first_mission_item = false;
}

void TerrainFollower::setCurrentMissionItem(const mission_item_s &current_mission_item)
{

	_dist_to_current_mission_item = get_distance_to_next_waypoint(_pos_lat_lon(0), _pos_lat_lon(1),
					current_mission_item.lat, current_mission_item.lon);

	// if mission item is too close, then don't set it
	if (_dist_to_current_mission_item > _param_check_dist.get()) {
		if (_had_first_mission_item) {
			_previous_mission_item = _current_mission_item;
			_previous_mission_item.nav_cmd = NAV_CMD_WAYPOINT;

		} else {
			_previous_mission_item = current_mission_item;
			_previous_mission_item.lat = _pos_lat_lon(0);
			_previous_mission_item.lon = _pos_lat_lon(1);
			_previous_mission_item.altitude = _alt_amsl_m;
			_previous_mission_item.altitude_is_relative = false;
		}

		_had_first_mission_item = true;
		_current_mission_item = current_mission_item;
		_current_mission_item_valid = true;
		_has_intermediate_mission_item = false;
	}
}

Vector2d TerrainFollower::getClosestPointOnMissionPath()
{
	float x, y;
	get_vector_to_next_waypoint(_previous_mission_item.lat, _previous_mission_item.lon, _current_mission_item.lat,
				    _current_mission_item.lon, &x, &y);
	Vector2f u_prev_to_target = Vector2f(x, y).unit_or_zero();
	get_vector_to_next_waypoint(_previous_mission_item.lat, _previous_mission_item.lon, _pos_lat_lon(0), _pos_lat_lon(1),
				    &x, &y);
	Vector2f prev_to_pos(x, y);

	const Vector2f prev_to_closest(u_prev_to_target * (prev_to_pos * u_prev_to_target));

	double lat, lon;
	create_waypoint_from_line_and_dist(_previous_mission_item.lat, _previous_mission_item.lon, _current_mission_item.lat,
					   _current_mission_item.lon, prev_to_closest.length(), &lat, &lon);

	return Vector2d(lat, lon);

}

bool TerrainFollower::updateIntermediateMissionItem(bool mission_item_reached)
{
	if (!_current_mission_item_valid) {
		return false;
	}

	bool should_reload_mission_items = false;

	bool target_item_reached = false;

	if (mission_item_reached) {

		if (_has_intermediate_mission_item) {
			// we reached an intermediate and need to re-load mission items
			_has_intermediate_mission_item = false;
			should_reload_mission_items = true;

		} else {
			// this was the target, don't reload the misson items but advance the mission
			target_item_reached = true;
			should_reload_mission_items = false;
			_current_mission_item_valid = false;
		}
	}

	if (!target_item_reached) {
		_dist_to_current_mission_item = get_distance_to_next_waypoint(_pos_lat_lon(0), _pos_lat_lon(1),
						_current_mission_item.lat, _current_mission_item.lon);

		if (_dist_to_current_mission_item > math::max(_param_check_dist.get(), 1.5f * _loiter_radius)) {
			Vector2d check_point_lat_lon = calculateCheckpoint();

			float new_altitude = 0.0f;
			const bool need_update = needAltitudeUpdate(check_point_lat_lon, new_altitude);

			if (need_update) {
				if (!_is_hovercraft) {
					// check if the climb is too steep for a fixed wing
					const float alt_gradient_max = param_fw_climbrate_max / math::max(_ground_speed_m_s, 0.1f);

					const float alt_grad = getAltitudeGradient(_pos_lat_lon, check_point_lat_lon, _alt_amsl_m,
							       new_altitude);

					// if the altitude change is too steep, then loiter to that altitude at the closest point of the planned mission path
					if (alt_grad > alt_gradient_max) {
						Vector2d closest_point_on_mission_path = getClosestPointOnMissionPath();
						check_point_lat_lon(0) = closest_point_on_mission_path(0);
						check_point_lat_lon(1) = closest_point_on_mission_path(1);
					}
				}

				// by default we use the current mission item the vehicle is supposed to go to
				_intermediate_mission_item = _current_mission_item;
				_intermediate_mission_item.lat = check_point_lat_lon(0);
				_intermediate_mission_item.lon = check_point_lat_lon(1);
				_intermediate_mission_item.altitude = new_altitude;
				_intermediate_mission_item.altitude_is_relative = false;
				_intermediate_mission_item.autocontinue = true;
				_intermediate_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
				_intermediate_mission_item.loiter_radius = _loiter_radius;

				should_reload_mission_items = true;
				_has_intermediate_mission_item = true;

			}
		}
	}

	return should_reload_mission_items;
}

bool TerrainFollower::needAltitudeUpdate(const Vector2d &check_point_lat_lon, float &new_altitude)
{
	float dist_to_terrain_at_checkpoint = 0;
	const bool terrain_valid = getDistanceToTerrainAtLocation(check_point_lat_lon,
				   dist_to_terrain_at_checkpoint);

	const bool min_alt_violation = _param_terrain_dist_min.get() > 0
				       && (dist_to_terrain_at_checkpoint < _param_terrain_dist_min.get());
	const bool max_alt_violation = _param_terrain_dist_max.get() > 0
				       && (dist_to_terrain_at_checkpoint > _param_terrain_dist_max.get());

	if (terrain_valid && (min_alt_violation || max_alt_violation)) {

		float alt_rel_target;

		if (_param_terrain_dist_min.get() > 0 && _param_terrain_dist_max.get() > 0) {
			alt_rel_target = 0.5f * (_param_terrain_dist_min.get() + _param_terrain_dist_max.get());

		} else if (min_alt_violation) {
			alt_rel_target = _param_terrain_dist_min.get();

		} else {
			alt_rel_target = _param_terrain_dist_max.get();
		}

		const float alt_at_checkpoint_amsl = getAltitudeAtCheckPoint(check_point_lat_lon);
		new_altitude = alt_at_checkpoint_amsl + alt_rel_target - dist_to_terrain_at_checkpoint;
		return true;
	}

	return false;
}

Vector2d TerrainFollower::calculateCheckpoint()
{
	double check_point_lat, check_point_lon;

	const float dist_prev_current = get_distance_to_next_waypoint(_previous_mission_item.lat, _previous_mission_item.lon,
					_current_mission_item.lat, _current_mission_item.lon);

	const float look_ahead_dist = math::min(_dist_to_current_mission_item, _param_check_dist.get());

	create_waypoint_from_line_and_dist(_previous_mission_item.lat, _previous_mission_item.lon, _current_mission_item.lat,
					   _current_mission_item.lon, math::max(dist_prev_current - _dist_to_current_mission_item + look_ahead_dist, 1.0f),
					   &check_point_lat, &check_point_lon);

	return Vector2d(check_point_lat, check_point_lon);
}

float TerrainFollower::getAltitudeGradient(const matrix::Vector2<double> &A, const matrix::Vector2<double> &B,
		const float alt_A,
		const float altB)
{
	const float dist_between_waypoints = get_distance_to_next_waypoint(A(0), A(1), B(0), B(1));

	return (altB - alt_A) / math::max(dist_between_waypoints, 0.1f);
}

float TerrainFollower::getAltitudeAtCheckPoint(const matrix::Vector2<double> &check_point_lat_lon)
{
	const float dist_to_checkpoint_m = get_distance_to_next_waypoint(_pos_lat_lon(0), _pos_lat_lon(1),
					   check_point_lat_lon(0), check_point_lat_lon(1));
	const float mission_item_alt_amsl_m = _current_mission_item.altitude_is_relative ? _current_mission_item.altitude +
					      _alt_home_amsl_m : _current_mission_item.altitude;

	const float alt_gradient = getAltitudeGradient(_pos_lat_lon, Vector2d(_current_mission_item.lat,
				   _current_mission_item.lon), _alt_amsl_m, mission_item_alt_amsl_m);

	return _alt_amsl_m + alt_gradient * dist_to_checkpoint_m;
}

bool TerrainFollower::getDistanceToTerrainAtLocation(const matrix::Vector2<double> &check_point_lat_lon,
		float &dist_to_terrain)
{
	bool ret = false;
	float terrain_alt_amsl_m;

	if (_terrain_provider->lookup(check_point_lat_lon(0), check_point_lat_lon(1), terrain_alt_amsl_m)) {
		dist_to_terrain = getAltitudeAtCheckPoint(check_point_lat_lon) - terrain_alt_amsl_m;
		ret = true;
	}

	return ret;
}
