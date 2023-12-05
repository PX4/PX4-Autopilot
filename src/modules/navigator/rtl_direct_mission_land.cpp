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

void RtlDirectMissionLand::on_activation()
{
	_land_detected_sub.update();
	_global_pos_sub.update();

	_needs_climbing = false;

	if (hasMissionLandStart()) {
		_is_current_planned_mission_item_valid = (goToItem(_mission.land_start_index, false) == PX4_OK);

		if ((_global_pos_sub.get().alt < _rtl_alt) || _enforce_rtl_alt) {

			// If lower than return altitude, climb up first.
			// If enforce_rtl_alt is true then forcing altitude change even if above.
			_needs_climbing = true;

		}

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
		// do not use LOITER_TO_ALT for rotary wing mode as it would then always climb to at least MIS_LTRMIN_ALT,
		// even if current climb altitude is below (e.g. RTL immediately after take off)
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
	rtl_time_estimate_s time_estimate;
	time_estimate.valid = false;
	time_estimate.timestamp = hrt_absolute_time();

	return time_estimate;
}
