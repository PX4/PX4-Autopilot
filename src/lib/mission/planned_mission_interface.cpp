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
 * @file planned_mission_interface.cpp
 *
 */

#include "planned_mission_interface.h"

#include <stdlib.h>

#include "dataman/dataman.h"
#include "lib/geo/geo.h"
#include "modules/navigator/mission_block.h"

void PlannedMissionInterface::update()
{
	_home_pos_sub.update();

	if (!_is_land_start_item_searched && _home_pos_sub.get().valid_alt) {
		findLandStartItem();
		_is_land_start_item_searched = true;
	}

	if (_mission_sub.updated()) {
		mission_s new_mission;
		_mission_sub.update(&new_mission);

		if (isMissionValid(new_mission)) {
			/* Check if it was updated externally*/
			if (new_mission.timestamp > _mission.timestamp) {
				bool mission_waypoints_changed{checkMissionWaypointsChanged(_mission, new_mission)};
				_mission = new_mission;

				if (goToItem(new_mission.current_seq, true) == EXIT_SUCCESS) {
					if (_home_pos_sub.get().valid_alt) {
						findLandStartItem();

					} else {
						_is_land_start_item_searched = false;
					}

					onMissionUpdate(mission_waypoints_changed);
				}
			}
		}
	}
}

void PlannedMissionInterface::getPreviousPositionItems(int32_t start_index, struct mission_item_s items[],
		size_t &num_found_items, uint8_t max_num_items) const
{
	num_found_items = 0u;

	int32_t next_mission_index{start_index};

	for (size_t item_idx = 0u; item_idx < max_num_items; item_idx++) {
		if (next_mission_index < 0) {
			break;
		}

		mission_item_s next_mission_item;
		bool found_next_item{false};

		do {
			found_next_item = getNonJumpItem(next_mission_index, next_mission_item, true, false, true) == EXIT_SUCCESS;
			next_mission_index--;
		} while (!MissionBlock::item_contains_position(next_mission_item) && found_next_item);

		if (found_next_item) {
			items[item_idx] = next_mission_item;
			num_found_items = item_idx + 1;

		} else {
			break;
		}
	}
}

void PlannedMissionInterface::getNextPositionItems(int32_t start_index, struct mission_item_s items[],
		size_t &num_found_items, uint8_t max_num_items) const
{
	// Make sure vector does not contain any preexisting elements.
	num_found_items = 0u;

	int32_t next_mission_index{start_index};

	for (size_t item_idx = 0u; item_idx < max_num_items; item_idx++) {
		if (next_mission_index >= _mission.count) {
			break;
		}

		mission_item_s next_mission_item;
		bool found_next_item{false};

		do {
			found_next_item = getNonJumpItem(next_mission_index, next_mission_item, true, false, false) == EXIT_SUCCESS;
			next_mission_index++;
		} while (!MissionBlock::item_contains_position(next_mission_item) && found_next_item);

		if (found_next_item) {
			items[item_idx] = next_mission_item;
			num_found_items = item_idx + 1;

		} else {
			break;
		}
	}
}

int PlannedMissionInterface::getNonJumpItem(int32_t &mission_index, mission_item_s &mission, bool execute_jump,
		bool write_jumps, bool mission_direction_backward) const
{
	if (mission_index >= _mission.count || mission_index < 0) {
		return EXIT_FAILURE;
	}

	int32_t new_mission_index{mission_index};
	mission_item_s new_mission;

	for (uint16_t jump_count = 0u; jump_count < _max_jump_iteraion; jump_count++) {
		if (readMissionItem(new_mission, new_mission_index) != EXIT_SUCCESS) {
			PX4_ERR("Could not read next item.");
			return EXIT_FAILURE;
		}

		if (new_mission.nav_cmd == NAV_CMD_DO_JUMP) {
			if (new_mission.do_jump_mission_index >= _mission.count || new_mission.do_jump_mission_index < 0) {
				PX4_ERR("Do Jump mission index is out of bounds.");
				return EXIT_FAILURE;
			}

			if ((new_mission.do_jump_current_count < new_mission.do_jump_repeat_count) && execute_jump) {
				if (write_jumps) {
					new_mission.do_jump_current_count++;

					if (writeMissionItem(new_mission, new_mission_index) != EXIT_SUCCESS) {
						PX4_ERR("Could not update jump count on mission item.");
						// Still continue searching for next non jump item.
					}
				}

				new_mission_index = new_mission.do_jump_mission_index;

			} else {
				if (mission_direction_backward) {
					new_mission_index--;

				} else {
					new_mission_index++;
				}
			}

		} else {
			break;
		}
	}

	mission_index = new_mission_index;
	mission = new_mission;

	return EXIT_SUCCESS;
}

bool PlannedMissionInterface::hasMissionLandStart() const
{
	return (_land_start_index != _invalid_index) && (_land_start_index < _mission.count);
}

int PlannedMissionInterface::goToNextItem(bool execute_jump)
{
	if (_mission.current_seq + 1 >= (_mission.count)) {
		return EXIT_FAILURE;
	}

	return goToItem(_mission.current_seq + 1, execute_jump);
}

int PlannedMissionInterface::goToPreviousItem(bool execute_jump)
{
	if (_mission.current_seq <= 0) {
		return EXIT_FAILURE;
	}

	return goToItem(_mission.current_seq - 1, execute_jump, true);
}

int PlannedMissionInterface::goToItem(int32_t index, bool execute_jump, bool mission_direction_backward)
{
	mission_item_s mission_item;

	if (getNonJumpItem(index, mission_item, execute_jump, true, mission_direction_backward) == EXIT_SUCCESS) {
		if (setMissionIndex(index) == EXIT_SUCCESS) {
			_current_planned_mission_item = mission_item;

		} else {
			return EXIT_FAILURE;
		}

	} else {
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

int PlannedMissionInterface::goToPreviousPositionItem(bool execute_jump)
{
	do {
		if (goToPreviousItem(execute_jump) != EXIT_SUCCESS) {
			return EXIT_FAILURE;
		}
	} while (!MissionBlock::item_contains_position(_current_planned_mission_item));

	return EXIT_SUCCESS;
}

int PlannedMissionInterface::goToNextPositionItem(bool execute_jump)
{
	do {
		if (goToNextItem(execute_jump) != EXIT_SUCCESS) {
			return EXIT_FAILURE;
		}
	} while (!MissionBlock::item_contains_position(_current_planned_mission_item));

	return EXIT_SUCCESS;
}

int PlannedMissionInterface::setMissionIndex(int32_t index)
{
	// Nothing to do if it is already at the current index.
	if (index == _mission.current_seq) {
		return EXIT_SUCCESS;
	}

	mission_s mission{_mission};
	mission.current_seq = index;
	mission.timestamp = hrt_absolute_time();

	if (writeMission(mission) == EXIT_SUCCESS) {
		_mission = mission;
		return EXIT_SUCCESS;

	} else {
		return EXIT_FAILURE;
	}
}

int32_t PlannedMissionInterface::getClosestMissionItemIndex(double lat, double lon, float alt, float home_alt,
		const vehicle_status_s &vehicle_status)
{
	int32_t min_dist_index(-1);
	float min_dist(FLT_MAX), dist_xy(FLT_MAX), dist_z(FLT_MAX);

	for (int32_t mission_item_index = 0; mission_item_index < _mission.count; mission_item_index++) {
		mission_item_s mission;

		if (readMissionItem(mission, mission_item_index) != EXIT_SUCCESS) {
			PX4_ERR("Could not set mission closest to position.");
			return EXIT_FAILURE;
		}

		if (MissionBlock::item_contains_position(mission)) {
			// do not consider land waypoints for a fw
			if (!((mission.nav_cmd == NAV_CMD_LAND) &&
			      (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) &&
			      (!vehicle_status.is_vtol))) {
				float dist = get_distance_to_point_global_wgs84(mission.lat, mission.lon,
						MissionBlock::get_absolute_altitude_for_item(mission, home_alt),
						lat,
						lon,
						alt,
						&dist_xy, &dist_z);

				if (dist < min_dist) {
					min_dist = dist;
					min_dist_index = mission_item_index;
				}
			}
		}
	}

	return min_dist_index;
}

int PlannedMissionInterface::setMissionToClosestItem(double lat, double lon, float alt, float home_alt,
		const vehicle_status_s &vehicle_status)
{
	int32_t min_dist_index(getClosestMissionItemIndex(lat, lon, alt, home_alt, vehicle_status));

	return goToItem(min_dist_index, false);
}

int PlannedMissionInterface::goToMissionLandStart()
{
	if (!hasMissionLandStart()) {
		return EXIT_FAILURE;
	}

	return goToItem(_land_start_index, false);
}

int PlannedMissionInterface::initMission()
{
	mission_s mission;
	bool ret_val{EXIT_FAILURE};

	if (readMission(mission) == EXIT_SUCCESS) {
		_mission = mission;

		if (goToItem(mission.current_seq, true) == EXIT_SUCCESS) {
			findLandStartItem();
			ret_val = EXIT_SUCCESS;
		}

	} else {
		resetMission();
	}

	_mission_pub.advertise();

	return ret_val;
}

void PlannedMissionInterface::resetMission()
{
	/* we do not need to reset mission if there is none.*/
	if (_mission.count == 0u) {
		return;
	}

	/* Set a new mission*/
	mission_s new_mission{.timestamp = hrt_absolute_time(),
			      .current_seq = 0,
			      .count = 0u,
			      .dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0};

	if (writeMission(new_mission) == EXIT_SUCCESS) {
		_mission = new_mission;

	} else {
		PX4_ERR("Mission Initialization failed.");
	}
}

void PlannedMissionInterface::resetMissionJumpCounter()
{
	for (size_t mission_index = 0u; mission_index < _mission.count; mission_index++) {
		mission_item_s mission_item;

		if (readMissionItem(mission_item, mission_index) != EXIT_SUCCESS) {
			PX4_ERR("Could not read mission item for jump count reset.");
			break;
		}

		if (mission_item.nav_cmd == NAV_CMD_DO_JUMP) {
			mission_item.do_jump_current_count = 0u;

			if (writeMissionItem(mission_item, mission_index) != EXIT_SUCCESS) {
				PX4_ERR("Could not write mission item for jump count reset.");
				break;
			}
		}
	}
}

int PlannedMissionInterface::writeMission(mission_s &mission)
{
	int ret_val{EXIT_SUCCESS};

	if (!isMissionValid(mission)) {
		return EXIT_FAILURE;
	}

	/* lock MISSION_STATE item */
	int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);

	if (dm_lock_ret != 0) {
		PX4_ERR("DM_KEY_MISSION_STATE lock failed");
		return EXIT_FAILURE;
	}

	if (dm_write(DM_KEY_MISSION_STATE, 0, &mission, sizeof(mission_s)) != sizeof(mission_s)) {
		PX4_ERR("Can't save mission state");
		ret_val = EXIT_FAILURE;

	} else {
		_mission_pub.publish(mission);
	}

	dm_unlock(DM_KEY_MISSION_STATE);

	return ret_val;
}

int PlannedMissionInterface::readMission(mission_s &read_mission) const
{
	int ret_val{EXIT_SUCCESS};

	/* lock MISSION_STATE item */
	int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);

	if (dm_lock_ret != 0) {
		PX4_ERR("DM lock failed.");
		return EXIT_FAILURE;
	}

	mission_s mission;

	if (dm_read(DM_KEY_MISSION_STATE, 0, &mission, sizeof(mission_s)) != sizeof(mission_s)) {
		PX4_ERR("Can't read mission state.");
		ret_val = EXIT_FAILURE;

	} else {
		if (isMissionValid(mission)) {
			read_mission = mission;

		} else {
			ret_val = EXIT_FAILURE;
		}
	}

	dm_unlock(DM_KEY_MISSION_STATE);

	return ret_val;
}

int PlannedMissionInterface::readMissionItem(mission_item_s &read_mission_item, size_t index) const
{
	int ret_val{EXIT_SUCCESS};

	if (index >= _mission.count) {
		return EXIT_FAILURE;
	}

	dm_item_t current_dm_item{static_cast<dm_item_t>(_mission.dataman_id)};

	/* lock current mission item */
	/*int dm_lock_ret = dm_lock(current_dm_item);

	if (dm_lock_ret != 0) {
		PX4_ERR("DM lock failed.");
		return EXIT_FAILURE;
	}*/

	mission_item_s mission_item;

	if (dm_read(current_dm_item, index, &mission_item, sizeof(mission_item_s)) != sizeof(mission_item_s)) {
		PX4_ERR("Can't read mission item from DM.");
		ret_val = EXIT_FAILURE;

	} else {
		read_mission_item = mission_item;
	}

	//dm_unlock(current_dm_item);

	return ret_val;
}

int PlannedMissionInterface::writeMissionItem(const mission_item_s &mission_item, size_t index) const
{
	int ret_val{EXIT_SUCCESS};

	if (index >= _mission.count) {
		return EXIT_FAILURE;
	}

	dm_item_t current_dm_item{static_cast<dm_item_t>(_mission.dataman_id)};

	/* lock current mission item */
	/*int dm_lock_ret = dm_lock(current_dm_item);

	if (dm_lock_ret != 0) {
		PX4_ERR("DM lock failed.");
		return EXIT_FAILURE;
	}*/

	if (dm_write(current_dm_item, index, &mission_item, sizeof(mission_item_s)) != sizeof(mission_item_s)) {
		PX4_ERR("Can't write mission item to DM.");
		ret_val = EXIT_FAILURE;
	}

	//dm_unlock(current_dm_item);

	return ret_val;
}

bool PlannedMissionInterface::isMissionValid(mission_s &mission) const
{
	bool ret_val{false};

	if ((mission.current_seq < mission.count) &&
	    (mission.current_seq >= 0) &&
	    (mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 || mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_1) &&
	    (mission.timestamp != 0u)) {
		ret_val = true;

	}

	return ret_val;
}

void PlannedMissionInterface::findLandStartItem()
{
	_land_start_index = _invalid_index;
	_land_index = _invalid_index;

	for (size_t mission_item_index = 1u; mission_item_index < _mission.count; mission_item_index++) {
		mission_item_s mission;

		if (readMissionItem(mission, mission_item_index) == EXIT_SUCCESS) {
			if (mission.nav_cmd == NAV_CMD_DO_LAND_START) {
				_land_start_index = mission_item_index;
			}

			if ((mission.nav_cmd == NAV_CMD_VTOL_LAND) || (mission.nav_cmd == NAV_CMD_LAND)) {
				_land_index = mission_item_index;
				break;
			}

		} else {
			break;
		}
	}

	if (_land_start_index != _invalid_index) {
		mission_item_s mission;
		size_t num_found_item{0u};
		getNextPositionItems(_land_start_index, &mission, num_found_item, 1u);

		if (num_found_item == 1u) {
			_land_start_pos.lat = mission.lat;
			_land_start_pos.lon = mission.lon;
			_land_start_pos.alt = mission.altitude_is_relative ?	mission.altitude +
					      _home_pos_sub.get().alt : mission.altitude;

		} else {
			PX4_ERR("Could not read land start coordinates.");
			_land_start_pos.lat = 0.0;
			_land_start_pos.lon = 0.0;
		}

		_land_pos.lat = _land_start_pos.lat;
		_land_pos.lon = _land_start_pos.lon;
		_land_pos.alt = _land_start_pos.alt;
	}

	if (_land_index != _invalid_index) {
		mission_item_s mission;

		if (readMissionItem(mission, _land_index) == EXIT_SUCCESS) {
			_land_pos.lat = mission.lat;
			_land_pos.lon = mission.lon;
			_land_pos.alt = mission.altitude_is_relative ?	mission.altitude +
					_home_pos_sub.get().alt : mission.altitude;
		}

		if (_land_start_index == _invalid_index) {
			_land_start_index = _land_index;
			_land_start_pos.lat = _land_pos.lat;
			_land_start_pos.lon = _land_pos.lon;
			_land_start_pos.alt = _land_pos.alt;
		}
	}
}

bool PlannedMissionInterface::checkMissionWaypointsChanged(const mission_s &old_mission,
		const mission_s &new_mission) const
{
	return (new_mission.count != old_mission.count) || (new_mission.dataman_id != old_mission.dataman_id);
}
