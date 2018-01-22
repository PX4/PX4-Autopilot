/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file mission_reverse.cpp
 *
 * Helper class to fly a offboard mission in reverse.
 * Only the positions of the waypoints are used, the type is converted to
 * MAV_CMD_NAV_WAYPOINT.
 *
 * @author Florian Achermann <florian.achermann@mavt.ethz.ch>
 */


#include "mission_reverse.h"
#include "navigator.h"

#include <geo/geo.h>
#include <dataman/dataman.h>
#include <lib/mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>

MissionReverse::MissionReverse(Navigator *navigator, Mission *mission) :
	MissionBlock(navigator),
	_mission(mission)
{
}

MissionReverse::~MissionReverse()
{
}

void
MissionReverse::on_inactive()
{
	if (!_navigator->in_rtl_state() && _mission_reverse_finished) {
		_mission_reverse_finished = false;
	}
}

void
MissionReverse::on_inactivation()
{
	if (_mission->_current_offboard_mission_index < 0) {
		_mission->_current_offboard_mission_index = 0;
	}
}

void
MissionReverse::on_activation()
{
	_mission_reverse_finished = false;
	_previous_mission = _mission->_offboard_mission;

	if (!(_mission->_offboard_mission.count == 0)) {
		if (_navigator->get_vstatus()->is_rotary_wing &&
		    _navigator->get_vstatus()->is_vtol &&
		    !_navigator->get_land_detected()->landed) {
			command_vtol_transition();

		} else {
			advance_mission();
			set_mission_items();
		}
	}

	mavlink_log_info(_navigator->get_mavlink_log_pub(), "Mission Reverse activated");
}

void
MissionReverse::on_active()
{
	// don't do anything if the plane is landed
	if (_navigator->get_land_detected()->landed) {
		return;
	}

	if (mission_changed()) {
		_mission->_current_offboard_mission_index = index_closest_mission_item();
		set_mission_items();
	}

	if (_mission->_mission_type != Mission::MISSION_TYPE_NONE && _mission->is_mission_item_reached()) {
		/* If we just completed a takeoff which was inserted before the right waypoint,
		   there is no need to report that we reached it because we didn't. */
		if (_mission->_work_item_type != Mission::WORK_ITEM_TYPE_TAKEOFF) {
			_mission->set_mission_item_reached();
		}

		// autocontinue to the next waypoint
		advance_mission();
		set_mission_items();

	} else if ((_mission->_mission_type != Mission::MISSION_TYPE_NONE)
		   && _mission->_param_altmode.get() == Mission::MISSION_ALTMODE_FOH) {
		_mission->altitude_sp_foh_update();

	} else {
		/* if waypoint position reached allow loiter on the setpoint */
		if (_mission->_waypoint_position_reached && _mission->_mission_item.nav_cmd != NAV_CMD_IDLE) {
			_navigator->set_can_loiter_at_sp(true);
		}
	}

	/* check if a cruise speed change has been commanded */
	if (_mission->_mission_type != Mission::MISSION_TYPE_NONE) {
		_mission->cruising_speed_sp_update();
	}

	/* see if we need to update the current yaw heading */
	if ((_mission->_param_yawmode.get() != Mission::MISSION_YAWMODE_NONE
	     && _mission->_param_yawmode.get() < Mission::MISSION_YAWMODE_MAX
	     && _mission->_mission_type != Mission::MISSION_TYPE_NONE)
	    || _navigator->get_vstatus()->is_vtol) {

		_mission->heading_sp_update();
	}

	/* check if landing needs to be aborted */
	if ((_mission_item.nav_cmd == NAV_CMD_LAND)
	    && (_navigator->abort_landing())) {

		_mission->do_abort_landing();
	}
}

void
MissionReverse::advance_mission()
{
	dm_item_t dm_current = (dm_item_t)(_mission->_offboard_mission.dataman_id);

	for (int32_t i = _mission->_current_offboard_mission_index - 1; i >= 0; i--) {
		struct mission_item_s missionitem = {};
		const ssize_t len = sizeof(missionitem);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			break;
		}

		if (item_contains_position(missionitem)) {
			_mission->_current_offboard_mission_index = i;
			return;
		}
	}

	// finished flying back the mission
	_mission->_current_offboard_mission_index = -1;
}

void
MissionReverse::set_mission_items()
{
	/* reset the altitude foh (first order hold) logic, if altitude foh is enabled (param) a new foh element starts now */
	_mission->_min_current_sp_distance_xy = FLT_MAX;

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* the home dist check provides user feedback, so we initialize it to this */
	bool user_feedback_done = false;

	/* mission item that comes after current if available */
	struct mission_item_s mission_item_next_position;
	bool has_next_position_item = false;

	Mission::work_item_type new_work_item_type = Mission::WORK_ITEM_TYPE_DEFAULT;

	if (_mission->prepare_mission_items(true, &_mission->_mission_item, &mission_item_next_position,
					    &has_next_position_item)) {
		/* if mission type changed, notify */
		if (_mission->_mission_type != Mission::MISSION_TYPE_OFFBOARD) {
			mavlink_log_info(_navigator->get_mavlink_log_pub(), "Executing mission in reverse.");
			user_feedback_done = true;
		}

		_mission->_mission_type = Mission::MISSION_TYPE_OFFBOARD;

	} else {
		/* no mission available or mission finished, switch to loiter */
		if (_mission->_mission_type != Mission::MISSION_TYPE_NONE) {

			/* https://en.wikipedia.org/wiki/Loiter_(aeronautics) */
			mavlink_log_info(_navigator->get_mavlink_log_pub(), "Mission reverse finished, loitering.");

			/* use last setpoint for loiter */
			_navigator->set_can_loiter_at_sp(true);

			user_feedback_done = true;

			// reset the mission index so that the mission could be restarted.
			_mission->_current_offboard_mission_index = 0;
		}

		_mission->_mission_type = Mission::MISSION_TYPE_NONE;

		/* set loiter mission item and ensure that there is a minimum clearance from home */
		_mission->set_loiter_item(&_mission->_mission_item, _navigator->get_takeoff_min_alt());

		/* update position setpoint triplet  */
		pos_sp_triplet->previous.valid = false;
		mission_apply_limitation(_mission->_mission_item);
		_mission->mission_item_to_position_setpoint(_mission->_mission_item, &pos_sp_triplet->current);
		pos_sp_triplet->next.valid = false;

		/* reuse setpoint for LOITER only if it's not IDLE */
		_navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER);

		// set mission finished
		_navigator->get_mission_result()->finished = true;
		_navigator->set_mission_result_updated();
		_mission_reverse_finished = true;

		if (!user_feedback_done) {
			/* only tell users that we got no mission if there has not been any
			 * better, more specific feedback yet
			 * https://en.wikipedia.org/wiki/Loiter_(aeronautics)
			 */

			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "No valid mission available (reverse), loitering.");

			user_feedback_done = true;
		}

		_navigator->set_position_setpoint_triplet_updated();
		return;
	}

	/*********************************** handle mission item *********************************************/

	/* handle position mission items */
	if (_mission->item_contains_position(_mission->_mission_item)) {
		// convert mission item to a simple waypoint
		_mission->_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		_mission->_mission_item.autocontinue = true;
		_mission->_mission_item.time_inside = 0.0f;

		/* we have a new position item so set previous position setpoint to current */
		pos_sp_triplet->previous = pos_sp_triplet->current;

	} else {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "MissionReverse: Got a non-position mission item, ignoring it");
	}

	/* set current position setpoint from mission item (is protected against non-position items) */
	_mission->mission_apply_limitation(_mission->_mission_item);
	_mission->mission_item_to_position_setpoint(_mission->_mission_item, &pos_sp_triplet->current);

	/* set current work item type */
	_mission->_work_item_type = new_work_item_type;

	_mission->_need_takeoff = false;

	_navigator->set_can_loiter_at_sp(false);
	_mission->reset_mission_item_reached();

	_mission->set_current_offboard_mission_item();

	if (_mission_item.autocontinue && get_time_inside(_mission_item) < FLT_EPSILON) {
		/* try to process next mission item */
		if (has_next_position_item) {
			/* got next mission item, update setpoint triplet */
			_mission->mission_apply_limitation(mission_item_next_position);
			_mission->mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->next);

		} else {
			/* next mission item is not available */
			pos_sp_triplet->next.valid = false;
		}

	} else {
		/* vehicle will be paused on current waypoint, don't set next item */
		pos_sp_triplet->next.valid = false;
	}

	/* Save the distance between the current sp and the previous one */
	if (pos_sp_triplet->current.valid && pos_sp_triplet->previous.valid) {

		_mission->_distance_current_previous = get_distance_to_next_waypoint(
				pos_sp_triplet->current.lat,
				pos_sp_triplet->current.lon,
				pos_sp_triplet->previous.lat,
				pos_sp_triplet->previous.lon);
	}

	_navigator->set_position_setpoint_triplet_updated();
}


uint16_t
MissionReverse::index_closest_mission_item() const
{
	uint16_t min_dist_index(0);
	float min_dist(FLT_MAX), dist_xy(FLT_MAX), dist_z(FLT_MAX);

	dm_item_t dm_current = (dm_item_t)(_mission->_offboard_mission.dataman_id);

	for (size_t i = 0; i < _mission->_offboard_mission.count; i++) {
		struct mission_item_s missionitem = {};
		const ssize_t len = sizeof(missionitem);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			break;
		}


		if (item_contains_position(missionitem)) {
			float altitude_amsl = get_absolute_altitude_for_item(missionitem);

			float dist = get_distance_to_point_global_wgs84(missionitem.lat, missionitem.lon, altitude_amsl,
					_navigator->get_global_position()->lat,
					_navigator->get_global_position()->lon,
					_navigator->get_global_position()->alt,
					&dist_xy, &dist_z);

			if (dist < min_dist) {
				min_dist = dist;
				min_dist_index = i;
			}
		}
	}

	// also consider the home position
	float dist = get_distance_to_point_global_wgs84(
			     _navigator->get_home_position()->lat,
			     _navigator->get_home_position()->lon,
			     _navigator->get_home_position()->alt,
			     _navigator->get_global_position()->lat,
			     _navigator->get_global_position()->lon,
			     _navigator->get_global_position()->alt,
			     &dist_xy, &dist_z);

	if (dist < min_dist) {
		min_dist = dist;
		min_dist_index = -1;
	}

	return min_dist_index;
}

bool
MissionReverse::mission_changed()
{
	if ((_previous_mission.timestamp == _mission->_offboard_mission.timestamp) &&
	    (_previous_mission.current_seq == _mission->_offboard_mission.current_seq) &&
	    (_previous_mission.count == _mission->_offboard_mission.count) &&
	    (_previous_mission.dataman_id == _mission->_offboard_mission.dataman_id)) {
		return false;

	} else {
		_previous_mission = _mission->_offboard_mission;
		return true;
	}
}

void
MissionReverse::command_vtol_transition()
{
	_mission->_mission_item.nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
	_mission->_mission_item.params[0] = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;
	_mission->_mission_item.yaw = _navigator->get_global_position()->yaw;

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->previous = pos_sp_triplet->current;
	_mission->generate_waypoint_from_heading(&pos_sp_triplet->current, _mission_item.yaw);
	_navigator->set_position_setpoint_triplet_updated();
	_mission->issue_command(_mission->_mission_item);
}
