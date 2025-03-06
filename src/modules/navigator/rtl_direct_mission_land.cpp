/***************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file rtl_direct_mission_land.cpp
 *
 * Helper class for RTL
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "rtl_direct_mission_land.h"
#include "navigator.h"

#include <drivers/drv_hrt.h>

static constexpr int32_t DEFAULT_DIRECT_MISSION_LAND_CACHE_SIZE = 5;

RtlDirectMissionLand::RtlDirectMissionLand(Navigator *navigator) :
	RtlBase(navigator, DEFAULT_DIRECT_MISSION_LAND_CACHE_SIZE)
{

}

void
RtlDirectMissionLand::updateDatamanCache()
{
	int32_t start_index;

	start_index = math::min(_mission.land_start_index, static_cast<int32_t>(_mission.count));

	if ((start_index >= 0) && (_mission.count > 0) && hasMissionLandStart() && (start_index != _load_mission_index)) {

		int32_t end_index = static_cast<int32_t>(_mission.count);

		// Check that we load all data into the cache
		if (end_index - start_index > _dataman_cache_size_signed) {
			_dataman_cache.invalidate();
			_dataman_cache_size_signed = end_index - start_index;
			_dataman_cache.resize(_dataman_cache_size_signed);
		}

		for (int32_t index = start_index; index != end_index; index += math::signNoZero(_dataman_cache_size_signed)) {

			_dataman_cache.load(static_cast<dm_item_t>(_mission.mission_dataman_id), index);
		}

		_load_mission_index = start_index;
	}

	_dataman_cache.update();
}

void RtlDirectMissionLand::on_inactive()
{
	MissionBase::on_inactive();

	updateDatamanCache();
}

void RtlDirectMissionLand::on_activation()
{
	_land_detected_sub.update();
	_global_pos_sub.update();

	_needs_climbing = false;

	if (hasMissionLandStart()) {
		_is_current_planned_mission_item_valid = (goToItem(_mission.land_start_index, false) == PX4_OK);

		_needs_climbing = checkNeedsToClimb();

	} else {
		_is_current_planned_mission_item_valid = false;
	}


	if (_land_detected_sub.get().landed) {
		// already landed, no need to do anything, invalidad the position mission item.
		_is_current_planned_mission_item_valid = false;
	}

	MissionBase::on_activation();
}

bool RtlDirectMissionLand::setNextMissionItem()
{
	return (goToNextPositionItem(true) == PX4_OK);
}

void RtlDirectMissionLand::setActiveMissionItems()
{
	WorkItemType new_work_item_type{WorkItemType::WORK_ITEM_TYPE_DEFAULT};
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// Climb to altitude
	if (_needs_climbing && _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) {
		// TODO: check if we also should use NAV_CMD_LOITER_TO_ALT for rotary wing
		if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;

		} else {
			_mission_item.nav_cmd = NAV_CMD_LOITER_TO_ALT;
		}

		_mission_item.lat = _global_pos_sub.get().lat;
		_mission_item.lon = _global_pos_sub.get().lon;
		_mission_item.altitude = _rtl_alt;
		_mission_item.altitude_is_relative = false;

		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();

		mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL Mission land: climb to %d m\t",
				 (int)ceilf(_rtl_alt));
		events::send<int32_t>(events::ID("rtl_mission_land_climb"), events::Log::Info,
				      "RTL Mission Land: climb to {1m_v}",
				      (int32_t)ceilf(_rtl_alt));

		_needs_climbing = false;
		mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_CLIMB;

	} else if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
		   _vehicle_status_sub.get().is_vtol &&
		   !_land_detected_sub.get().landed && _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) {
		// Transition to fixed wing if necessary.
		set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
		_mission_item.yaw = _navigator->get_local_position()->heading;

		// keep current setpoints (FW position controller generates wp to track during transition)
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF;

	} else if (item_contains_position(_mission_item)) {

		static constexpr size_t max_num_next_items{1u};
		int32_t next_mission_items_index[max_num_next_items];
		size_t num_found_items = 0;
		getNextPositionItems(_mission.current_seq + 1, next_mission_items_index, num_found_items, max_num_next_items);

		mission_item_s next_mission_items[max_num_next_items];
		const dm_item_t mission_dataman_id = static_cast<dm_item_t>(_mission.mission_dataman_id);

		for (size_t i = 0U; i < num_found_items; i++) {
			mission_item_s next_mission_item;
			bool success = _dataman_cache.loadWait(mission_dataman_id, next_mission_items_index[i],
							       reinterpret_cast<uint8_t *>(&next_mission_item), sizeof(next_mission_item), MAX_DATAMAN_LOAD_WAIT);

			if (success) {
				next_mission_items[i] = next_mission_item;

			} else {
				num_found_items = i;
				break;
			}
		}

		if (_mission_item.nav_cmd == NAV_CMD_LAND ||
		    _mission_item.nav_cmd == NAV_CMD_VTOL_LAND) {
			handleLanding(new_work_item_type, next_mission_items, num_found_items);

		} else {
			// convert mission item to a simple waypoint, keep loiter to alt
			if (_mission_item.nav_cmd != NAV_CMD_LOITER_TO_ALT) {
				_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			}

			_mission_item.autocontinue = true;
			_mission_item.time_inside = 0.0f;

			pos_sp_triplet->previous = pos_sp_triplet->current;
		}

		if (num_found_items > 0) {
			mission_item_to_position_setpoint(next_mission_items[0u], &pos_sp_triplet->next);
		}

		mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

		// prevent lateral guidance from loitering at a waypoint as part of a mission landing if the altitude
		// is not achieved.
		const bool fw_on_mission_landing = _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
						   && isLanding() &&
						   _mission_item.nav_cmd == NAV_CMD_WAYPOINT;
		const bool mc_landing_after_transition = _vehicle_status_sub.get().vehicle_type ==
				vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && _vehicle_status_sub.get().is_vtol &&
				new_work_item_type == WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND;

		if (fw_on_mission_landing || mc_landing_after_transition) {
			pos_sp_triplet->current.alt_acceptance_radius = FLT_MAX;
		}
	}

	issue_command(_mission_item);

	/* set current work item type */
	_work_item_type = new_work_item_type;

	reset_mission_item_reached();

	if (_mission_type == MissionType::MISSION_TYPE_MISSION) {
		set_mission_result();
	}

	publish_navigator_mission_item(); // for logging
	_navigator->set_position_setpoint_triplet_updated();
}

rtl_time_estimate_s RtlDirectMissionLand::calc_rtl_time_estimate()
{
	_rtl_time_estimator.update();
	_rtl_time_estimator.setVehicleType(_vehicle_status_sub.get().vehicle_type);
	_rtl_time_estimator.reset();

	if (_mission.count > 0 && hasMissionLandStart()) {
		int32_t start_item_index{-1};
		bool is_in_climbing_submode{false};

		if (isActive()) {
			start_item_index = math::max(_mission.current_seq, _mission.land_start_index);
			is_in_climbing_submode = _work_item_type == WorkItemType::WORK_ITEM_TYPE_CLIMB;

		} else {
			start_item_index = _mission.land_start_index;
			is_in_climbing_submode = checkNeedsToClimb();
		}

		if (start_item_index >= 0 && start_item_index < static_cast<int32_t>(_mission.count)) {
			float altitude_at_calculation_point;
			matrix::Vector2d hor_position_at_calculation_point{_global_pos_sub.get().lat, _global_pos_sub.get().lon};

			if (is_in_climbing_submode) {
				if (_enforce_rtl_alt) {
					_rtl_time_estimator.addVertDistance(_rtl_alt - _global_pos_sub.get().alt);
					altitude_at_calculation_point = _rtl_alt;

				} else {
					if (_global_pos_sub.get().alt < _rtl_alt) {
						_rtl_time_estimator.addVertDistance(_rtl_alt - _global_pos_sub.get().alt);
					}

					altitude_at_calculation_point = math::max(_rtl_alt, _global_pos_sub.get().alt);
				}

			} else {
				altitude_at_calculation_point = _global_pos_sub.get().alt;
			}

			while (start_item_index < _mission.count && start_item_index >= 0) {
				int32_t next_mission_item_index;
				size_t num_found_items{0U};
				getNextPositionItems(start_item_index, &next_mission_item_index, num_found_items, 1U);

				if (num_found_items > 0U) {
					mission_item_s next_position_mission_item;
					const dm_item_t dataman_id = static_cast<dm_item_t>(_mission.mission_dataman_id);
					bool success = _dataman_cache.loadWait(dataman_id, next_mission_item_index,
									       reinterpret_cast<uint8_t *>(&next_position_mission_item), sizeof(next_position_mission_item), MAX_DATAMAN_LOAD_WAIT);

					if (!success) {
						// Could not load the mission item, mark time estimate as invalid.
						_rtl_time_estimator.reset();
						break;
					}

					switch (next_position_mission_item.nav_cmd) {
					case NAV_CMD_LOITER_UNLIMITED: {
							_rtl_time_estimator.reset();
							break;
						}

					case NAV_CMD_LOITER_TIME_LIMIT: {
							// Go to loiter
							matrix::Vector2f direction{};
							get_vector_to_next_waypoint(hor_position_at_calculation_point(0), hor_position_at_calculation_point(1),
										    next_position_mission_item.lat, next_position_mission_item.lon, &direction(0), &direction(1));

							float hor_dist = get_distance_to_next_waypoint(hor_position_at_calculation_point(0),
									 hor_position_at_calculation_point(1), next_position_mission_item.lat,
									 next_position_mission_item.lon);

							if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
								hor_dist = math::max(0.f, hor_dist - next_position_mission_item.loiter_radius);
							}

							_rtl_time_estimator.addDistance(hor_dist, direction, 0.f);

							// add time
							_rtl_time_estimator.addWait(next_position_mission_item.time_inside);
							break;
						}

					case NAV_CMD_LOITER_TO_ALT: {
							// Go to point horizontally
							matrix::Vector2f direction{};
							get_vector_to_next_waypoint(hor_position_at_calculation_point(0), hor_position_at_calculation_point(1),
										    next_position_mission_item.lat, next_position_mission_item.lon, &direction(0), &direction(1));

							float hor_dist = get_distance_to_next_waypoint(hor_position_at_calculation_point(0),
									 hor_position_at_calculation_point(1), next_position_mission_item.lat,
									 next_position_mission_item.lon);

							if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
								hor_dist = math::max(0.f, hor_dist - next_position_mission_item.loiter_radius);
							}

							_rtl_time_estimator.addDistance(hor_dist, direction, 0.f);

							// Add the vertical loiter
							_rtl_time_estimator.addVertDistance(get_absolute_altitude_for_item(next_position_mission_item) -
											    altitude_at_calculation_point);

							break;
						}

					case NAV_CMD_LAND: // Fallthrough
					case NAV_CMD_VTOL_LAND: {

							matrix::Vector2f direction{};
							get_vector_to_next_waypoint(hor_position_at_calculation_point(0), hor_position_at_calculation_point(1),
										    next_position_mission_item.lat, next_position_mission_item.lon, &direction(0), &direction(1));

							const float hor_dist = get_distance_to_next_waypoint(hor_position_at_calculation_point(0),
									       hor_position_at_calculation_point(1), next_position_mission_item.lat, next_position_mission_item.lon);

							// For fixed wing, add diagonal line
							if ((_vehicle_status_sub.get().vehicle_type != vehicle_status_s::VEHICLE_TYPE_FIXED_WING)
							    && (!_vehicle_status_sub.get().is_vtol)) {

								_rtl_time_estimator.addDistance(hor_dist, direction,
												get_absolute_altitude_for_item(next_position_mission_item) - altitude_at_calculation_point);

							} else {
								// For VTOL, Rotary, go there horizontally first, then land
								_rtl_time_estimator.addDistance(hor_dist, direction, 0.f);

								if (_vehicle_status_sub.get().is_vtol) {
									_rtl_time_estimator.setVehicleType(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
								}

								_rtl_time_estimator.addVertDistance(get_absolute_altitude_for_item(next_position_mission_item) -
												    altitude_at_calculation_point);
							}

							break;
						}

					default: {
							// Default assume can go to the location directly
							matrix::Vector2f direction{};
							get_vector_to_next_waypoint(hor_position_at_calculation_point(0), hor_position_at_calculation_point(1),
										    next_position_mission_item.lat, next_position_mission_item.lon, &direction(0), &direction(1));

							const float hor_dist = get_distance_to_next_waypoint(hor_position_at_calculation_point(0),
									       hor_position_at_calculation_point(1), next_position_mission_item.lat, next_position_mission_item.lon);

							_rtl_time_estimator.addDistance(hor_dist, direction,
											get_absolute_altitude_for_item(next_position_mission_item) - altitude_at_calculation_point);
							break;
						}
					}

					start_item_index = next_mission_item_index + 1;
					hor_position_at_calculation_point(0) = next_position_mission_item.lat;
					hor_position_at_calculation_point(1) = next_position_mission_item.lon;
					altitude_at_calculation_point = get_absolute_altitude_for_item(next_position_mission_item);


				} else {
					start_item_index = -1;
				}
			}
		}
	}

	return _rtl_time_estimator.getEstimate();
}

bool RtlDirectMissionLand::checkNeedsToClimb()
{
	bool needs_climbing{false};

	if ((_global_pos_sub.get().alt < _rtl_alt) || _enforce_rtl_alt) {

		// If lower than return altitude, climb up first.
		// If enforce_rtl_alt is true then forcing altitude change even if above.
		needs_climbing = true;

	}

	return needs_climbing;
}
