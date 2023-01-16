/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file vtol_land.cpp
 *
 * Helper class to do a VTOL landing using a loiter down to altitude landing pattern.
 *
 */

#include "vtol_land.h"
#include "navigator.h"
#include "navigation.h"

#include "modules/dataman/dataman.h"

using matrix::wrap_pi;

VtolLand::VtolLand(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
VtolLand::on_activation()
{
	_global_pos_sub.update();
	_wind_sub.update();

	readVtolLandApproachesFromStorage();
	set_loiter_position();
	_land_state = vtol_land_state::MOVE_TO_LOITER;

}

void VtolLand::on_inactive()
{
	_global_pos_sub.update();
	_wind_sub.update();
}

void
VtolLand::on_active()
{
	_global_pos_sub.update();
	_wind_sub.update();

	if (is_mission_item_reached_or_completed()) {
		switch	(_land_state) {
		case vtol_land_state::MOVE_TO_LOITER: {
				_mission_item.altitude = _destination.alt + _land_approach.height_m;
				_mission_item.nav_cmd = NAV_CMD_LOITER_TO_ALT;
				_mission_item.loiter_radius = _param_rtl_loiter_rad.get();

				_navigator->get_mission_result()->finished = false;
				_navigator->set_mission_result_updated();
				reset_mission_item_reached();

				// convert mission item to current setpoint
				struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
				mission_apply_limitation(_mission_item);
				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

				pos_sp_triplet->next.valid = true;
				pos_sp_triplet->next.lat = _land_pos_lat_lon(0);
				pos_sp_triplet->next.lon = _land_pos_lat_lon(1);
				pos_sp_triplet->next.type = position_setpoint_s::SETPOINT_TYPE_LAND;

				pos_sp_triplet->previous.valid = false;
				pos_sp_triplet->current.yaw_valid = true;

				_land_state = vtol_land_state::LOITER_DOWN;
				break;
			}

		case vtol_land_state::LOITER_DOWN: {
				_mission_item.lat = _land_pos_lat_lon(0);
				_mission_item.lon = _land_pos_lat_lon(1);

				_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
				_mission_item.vtol_back_transition = true;

				struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

				// set previous item location to loiter location such that vehicle tracks line between loiter
				// location and land location after exiting the loiter circle
				pos_sp_triplet->previous.lat = _loiter_pos_lat_lon(0);
				pos_sp_triplet->previous.lon = _loiter_pos_lat_lon(1);
				pos_sp_triplet->previous.alt = _mission_item.altitude;
				pos_sp_triplet->previous.valid = true;

				//publish_navigator_mission_item(); // for logging
				_navigator->set_position_setpoint_triplet_updated();

				// issue_command(_mission_item);
				reset_mission_item_reached();

				_land_state = vtol_land_state::TRANSITION_TO_MC;

				break;
			}

		case vtol_land_state::TRANSITION_TO_MC: {
				set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
				_mission_item.vtol_back_transition = true;

				issue_command(_mission_item);

				struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

				//publish_navigator_mission_item(); // for logging
				_navigator->set_position_setpoint_triplet_updated();

				// issue_command(_mission_item);
				reset_mission_item_reached();

				_land_state = vtol_land_state::LAND;

				break;
			}

		case vtol_land_state::LAND: {
				_mission_item.nav_cmd = NAV_CMD_LAND;
				struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

				//publish_navigator_mission_item(); // for logging
				_navigator->set_position_setpoint_triplet_updated();

				// issue_command(_mission_item);
				reset_mission_item_reached();

				_land_state = vtol_land_state::IDLE;

				break;
			}

		default: {

				break;
			}
		}
	}
}

bool VtolLand::hasVtolLandApproach()
{
	readVtolLandApproachesFromStorage();
	return _vtol_home_land_approaches.isAnyApproachValid();
}

void
VtolLand::set_loiter_position()
{

	_land_approach = chooseBestLandingApproach();

	if (PX4_ISFINITE(_land_approach.lat) && PX4_ISFINITE(_land_approach.lon)) {
		_loiter_pos_lat_lon(0) = _land_approach.lat;
		_loiter_pos_lat_lon(1) = _land_approach.lon;

	} else {
		_loiter_pos_lat_lon(0) = _destination.lat;
		_loiter_pos_lat_lon(1) = _destination.lon;
	}


	_mission_item.lat  = _loiter_pos_lat_lon(0);
	_mission_item.lon = _loiter_pos_lat_lon(1);
	_mission_item.altitude = math::max(_destination.alt + _param_return_alt_rel_m.get(),
					   _global_pos_sub.get().alt);
	_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
	_mission_item.force_heading = true;
	_mission_item.autocontinue = false;
	_mission_item.time_inside = _min_loiter_time_before_land;

	_mission_item.altitude_is_relative = false;

	_mission_item.loiter_radius = _param_rtl_loiter_rad.get();
	_mission_item.origin = ORIGIN_ONBOARD;

	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();
	reset_mission_item_reached();

	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

	_navigator->set_can_loiter_at_sp(true);

	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

loiter_point_s VtolLand::chooseBestLandingApproach()
{
	const float wind_direction = atan2f(_wind_sub.get().windspeed_east, _wind_sub.get().windspeed_north);
	int8_t min_index = -1;
	float wind_angle_prev = INFINITY;

	for (int i = 0; i < _vtol_home_land_approaches.num_approaches_max; i++) {

		if (_vtol_home_land_approaches.approaches[i].isValid()) {
			const float wind_angle = wrap_pi(get_bearing_to_next_waypoint(_destination.lat,
							 _destination.lon, _vtol_home_land_approaches.approaches[i].lat,
							 _vtol_home_land_approaches.approaches[i].lon) - wind_direction);

			if (fabsf(wind_angle) < wind_angle_prev) {
				min_index = i;
				wind_angle_prev = fabsf(wind_angle);
			}

		}
	}

	if (min_index >= 0) {
		return _vtol_home_land_approaches.approaches[min_index];

	} else {

		return loiter_point_s();
	}
}

void VtolLand::readVtolLandApproachesFromStorage()
{

	// go through all mission items in the rally point storage. If we find a mission item of type NAV_CMD_RALLY_POINT
	// which is within MAX_DIST_FROM_LAND_FOR_APPROACHES of our current pestination position then treat ALL following mission items of type NAV_CMD_LOITER_TO_ALT which come
	// BEFORE the next mission item of type NAV_CMD_RALLY_POINT as land approaches for the destination
	_vtol_home_land_approaches.resetAllApproaches();

	mission_stats_entry_s stats;
	int ret = dm_read(DM_KEY_SAFE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));
	int num_mission_items = 0;
	bool foundLandApproaches = false;
	uint8_t sector_counter = 0;

	if (ret == sizeof(mission_stats_entry_s)) {
		num_mission_items = stats.num_items;
	}

	for (int current_seq = 1; current_seq <= num_mission_items; ++current_seq) {
		mission_item_s mission_item{};

		if (dm_read(DM_KEY_SAFE_POINTS, current_seq, &mission_item, sizeof(mission_item_s)) !=
		    sizeof(mission_item_s)) {
			PX4_ERR("dm_read failed");
			break;
		}

		if (mission_item.nav_cmd == NAV_CMD_RALLY_POINT) {

			if (foundLandApproaches) {
				break;
			}

			const float dist_to_land = get_distance_to_next_waypoint(mission_item.lat, mission_item.lon, _destination.lat,
						   _destination.lon);

			if (!mission_item.is_mission_rally_point && dist_to_land < MAX_DIST_FROM_LAND_FOR_APPROACHES) {
				foundLandApproaches = true;
				_vtol_home_land_approaches.land_location_lat_lon = matrix::Vector2d(mission_item.lat, mission_item.lon);
			}

			sector_counter = 0;
		}

		if (foundLandApproaches && mission_item.nav_cmd == NAV_CMD_LOITER_TO_ALT) {
			_vtol_home_land_approaches.approaches[sector_counter].lat = mission_item.lat;
			_vtol_home_land_approaches.approaches[sector_counter].lon = mission_item.lon;
			_vtol_home_land_approaches.approaches[sector_counter].height_m = mission_item.altitude;
			_vtol_home_land_approaches.approaches[sector_counter].loiter_radius_m = mission_item.loiter_radius;
			sector_counter++;
		}
	}
}

rtl_time_estimate_s VtolLand::calc_rtl_time_estimate()
{
	rtl_time_estimate_s time_estimate;
	time_estimate.valid = false;
	time_estimate.timestamp = hrt_absolute_time();

	return time_estimate;
}
