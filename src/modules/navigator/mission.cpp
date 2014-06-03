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
 * @file navigator_mission.cpp
 *
 * Helper class to access missions
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>

#include <dataman/dataman.h>
#include <systemlib/err.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission_result.h>

#include "navigator.h"
#include "mission.h"


Mission::Mission(Navigator *navigator) :
	SuperBlock(NULL, "MIS"),
	_navigator(navigator),
	_first_run(true),
	_param_onboard_enabled(this, "ONBOARD_EN"),
	_onboard_mission_sub(-1),
	_offboard_mission_sub(-1),
	_onboard_mission({0}),
	_offboard_mission({0}),
	_mission_item({0}),
	_mission_result_pub(-1),
	_mission_result({0}),
	_mission_type(MISSION_TYPE_NONE)
{
	/* load initial params */
	updateParams();
	/* set initial mission items */
	reset();
}

Mission::~Mission()
{
}

void
Mission::reset()
{
	_first_run = true;
}

bool
Mission::update(struct position_setpoint_triplet_s *pos_sp_triplet)
{
	bool updated = false;
	/* check if anything has changed, and reset mission items if needed */
	if (is_onboard_mission_updated() || is_offboard_mission_updated() || _first_run) {
		set_mission_items(pos_sp_triplet);
		updated = true;
		_first_run = false;
	}

	if (_mission_type != MISSION_TYPE_NONE && is_mission_item_reached()) {
		advance_mission();
		set_mission_items(pos_sp_triplet);
		updated = true;
	}
	return updated;
}

bool
Mission::is_mission_item_reached()
{
	/* TODO: count turns */
#if 0
	if ((_mission_item.nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||
	     _mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
	     _mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED) &&
	    _mission_item.loiter_radius > 0.01f) {

		return false;
	}
#endif

	hrt_abstime now = hrt_absolute_time();

	if (!_waypoint_position_reached) {

		float dist = -1.0f;
		float dist_xy = -1.0f;
		float dist_z = -1.0f;

		float altitude_amsl = _mission_item.altitude_is_relative
				      ? _mission_item.altitude + _navigator->get_home_position()->alt
			              : _mission_item.altitude;

		dist = get_distance_to_point_global_wgs84(_mission_item.lat, _mission_item.lon, altitude_amsl,
				                          _navigator->get_global_position()->lat,
							  _navigator->get_global_position()->lon,
							  _navigator->get_global_position()->alt,
				&dist_xy, &dist_z);

		if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF) {

			/* require only altitude for takeoff */
			if (_navigator->get_global_position()->alt > altitude_amsl - _mission_item.acceptance_radius) {
				_waypoint_position_reached = true;
			}
		} else {
			if (dist >= 0.0f && dist <= _mission_item.acceptance_radius) {
				_waypoint_position_reached = true;
			}
		}
	}

	if (_waypoint_position_reached && !_waypoint_yaw_reached) {

		/* TODO: removed takeoff, why? */
		if (_navigator->get_vstatus()->is_rotary_wing && isfinite(_mission_item.yaw)) {

			/* check yaw if defined only for rotary wing except takeoff */
			float yaw_err = _wrap_pi(_mission_item.yaw - _navigator->get_global_position()->yaw);

			if (fabsf(yaw_err) < 0.2f) { /* TODO: get rid of magic number */
				_waypoint_yaw_reached = true;
			}

		} else {
			_waypoint_yaw_reached = true;
		}
	}

	/* check if the current waypoint was reached */
	if (_waypoint_position_reached && _waypoint_yaw_reached) {

		if (_time_first_inside_orbit == 0) {
			_time_first_inside_orbit = now;

			// if (_mission_item.time_inside > 0.01f) {
			// 	mavlink_log_info(_mavlink_fd, "#audio: waypoint reached, wait for %.1fs",
			// 		(double)_mission_item.time_inside);
			// }
		}

		/* check if the MAV was long enough inside the waypoint orbit */
		if (now - _time_first_inside_orbit >= (hrt_abstime)_mission_item.time_inside * 1e6f) {
			return true;
		}
	}
	return false;
}

void
Mission::reset_mission_item_reached() {
	_waypoint_position_reached = false;
	_waypoint_yaw_reached = false;
	_time_first_inside_orbit = 0;
}

void
Mission::mission_item_to_position_setpoint(const mission_item_s *item, position_setpoint_s *sp)
{
	sp->valid = true;

	sp->lat = item->lat;
	sp->lon = item->lon;
	sp->alt = item->altitude_is_relative ? item->altitude + _navigator->get_home_position()->alt : item->altitude;
	sp->yaw = item->yaw;
	sp->loiter_radius = item->loiter_radius;
	sp->loiter_direction = item->loiter_direction;
	sp->pitch_min = item->pitch_min;

	if (item->nav_cmd == NAV_CMD_TAKEOFF) {
		sp->type = SETPOINT_TYPE_TAKEOFF;

	} else if (item->nav_cmd == NAV_CMD_LAND) {
		sp->type = SETPOINT_TYPE_LAND;

	} else if (item->nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
		   item->nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||
		   item->nav_cmd == NAV_CMD_LOITER_UNLIMITED) {
		sp->type = SETPOINT_TYPE_LOITER;

	} else {
		sp->type = SETPOINT_TYPE_POSITION;
	}
}

bool
Mission::is_onboard_mission_updated()
{
	bool updated;
	orb_check(_onboard_mission_sub, &updated);

	if (!updated) {
		return false;
	}

	if (orb_copy(ORB_ID(onboard_mission), _onboard_mission_sub, &_onboard_mission) != OK) {
		_onboard_mission.count = 0;
		_onboard_mission.current_index = 0;
	}
	return true;
}

bool
Mission::is_offboard_mission_updated()
{
	bool updated;
	orb_check(_offboard_mission_sub, &updated);
	if (!updated) {
		return false;
	}

	if (orb_copy(ORB_ID(offboard_mission), _offboard_mission_sub, &_offboard_mission) == OK) {

		/* Check mission feasibility, for now do not handle the return value,
		 * however warnings are issued to the gcs via mavlink from inside the MissionFeasiblityChecker */
		dm_item_t dm_current;

		if (_offboard_mission.dataman_id == 0) {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_0;
		} else {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_1;
		}

		missionFeasiblityChecker.checkMissionFeasible(_navigator->get_vstatus()->is_rotary_wing, dm_current,
			                                      (size_t)_offboard_mission.count,
							      _navigator->get_geofence());
	} else {
		_offboard_mission.count = 0;
		_offboard_mission.current_index = 0;
	}
	return true;
}


void
Mission::advance_mission()
{
	switch (_mission_type) {
	case MISSION_TYPE_ONBOARD:
		_onboard_mission.current_index++;
		break;

	case MISSION_TYPE_OFFBOARD:
		_offboard_mission.current_index++;
		break;

	case MISSION_TYPE_NONE:
	default:
		break;
	}
}

void
Mission::set_mission_items(struct position_setpoint_triplet_s *pos_sp_triplet)
{
	warnx("set mission items");

	set_previous_pos_setpoint(&pos_sp_triplet->current, &pos_sp_triplet->previous);

	if (is_current_onboard_mission_item_set(&pos_sp_triplet->current)) {
		/* try setting onboard mission item */
		_mission_type = MISSION_TYPE_ONBOARD;

	} else if (is_current_offboard_mission_item_set(&pos_sp_triplet->current)) {
		/* try setting offboard mission item */
		_mission_type = MISSION_TYPE_OFFBOARD;
	} else {
		_mission_type = MISSION_TYPE_NONE;
	}
}

void
Mission::set_previous_pos_setpoint(const struct position_setpoint_s *current_pos_sp,
		                   struct position_setpoint_s *previous_pos_sp)
{
	/* reuse current setpoint as previous setpoint */
	if (current_pos_sp->valid) {
		memcpy(previous_pos_sp, current_pos_sp, sizeof(struct position_setpoint_s));
	}
}

bool
Mission::is_current_onboard_mission_item_set(struct position_setpoint_s *current_pos_sp)
{
	if (_param_onboard_enabled.get() > 0
	    && _onboard_mission.current_index < (int)_onboard_mission.count) {
		struct mission_item_s new_mission_item;
		if (read_mission_item(DM_KEY_WAYPOINTS_ONBOARD, true, &_onboard_mission.current_index,
					&new_mission_item)) {
			/* convert the current mission item and set it valid */
			mission_item_to_position_setpoint(&_mission_item, current_pos_sp);
			current_pos_sp->valid = true;

			/* TODO: report this somehow */
			return true;
		}
	}
	return false;
}

bool
Mission::is_current_offboard_mission_item_set(struct position_setpoint_s *current_pos_sp)
{
	if (_offboard_mission.current_index < (int)_offboard_mission.count) {
		dm_item_t dm_current;
		if (_offboard_mission.dataman_id == 0) {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_0;
		} else {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_1;
		}
		struct mission_item_s new_mission_item;
		if (read_mission_item(dm_current, true, &_offboard_mission.current_index, &new_mission_item)) {
			/* convert the current mission item and set it valid */
			mission_item_to_position_setpoint(&_mission_item, current_pos_sp);
			current_pos_sp->valid = true;

			report_current_offboard_mission_item();
			return true;
		}
	}
	return false;
}

void
Mission::get_next_onboard_mission_item(struct position_setpoint_s *next_pos_sp)
{
	int next_temp_mission_index = _onboard_mission.current_index + 1;

	/* try if there is a next onboard mission */
	if (next_temp_mission_index < (int)_onboard_mission.count) {
		struct mission_item_s new_mission_item;
		if (read_mission_item(DM_KEY_WAYPOINTS_ONBOARD, false, &next_temp_mission_index, &new_mission_item)) {
			/* convert next mission item to position setpoint */
			mission_item_to_position_setpoint(&new_mission_item, next_pos_sp);
			next_pos_sp->valid = true;
			return;
		}
	}

	/* give up */
	next_pos_sp->valid = false;
	return;
}

void
Mission::get_next_offboard_mission_item(struct position_setpoint_s *next_pos_sp)
{
	/* try if there is a next offboard mission */
	int next_temp_mission_index = _offboard_mission.current_index + 1;
	if (next_temp_mission_index < (int)_offboard_mission.count) {
		dm_item_t dm_current;
		if (_offboard_mission.dataman_id == 0) {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_0;
		} else {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_1;
		}
		struct mission_item_s new_mission_item;
		if (read_mission_item(dm_current, false, &next_temp_mission_index, &new_mission_item)) {
			/* convert next mission item to position setpoint */
			mission_item_to_position_setpoint(&new_mission_item, next_pos_sp);
			next_pos_sp->valid = true;
			return;
		}
	}
	/* give up */
	next_pos_sp->valid = false;
	return;
}

bool
Mission::read_mission_item(const dm_item_t dm_item, bool is_current, int *mission_index,
		           struct mission_item_s *new_mission_item)
{
	/* repeat several to get the mission item because we might have to follow multiple DO_JUMPS */
	for (int i=0; i<10; i++) {
		const ssize_t len = sizeof(struct mission_item_s);

		/* read mission item from datamanager */
		if (dm_read(dm_item, *mission_index, new_mission_item, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return false;
		}

		/* check for DO_JUMP item, and whether it hasn't not already been repeated enough times */
		if (new_mission_item->nav_cmd == NAV_CMD_DO_JUMP) {

			if (new_mission_item->do_jump_current_count >= new_mission_item->do_jump_repeat_count) {
				return false;
			}

			/* only raise the repeat count if this is for the current mission item
			 * but not for the next mission item */
			if (is_current) {
				(new_mission_item->do_jump_current_count)++;

				/* save repeat count */
				if (dm_write(dm_item, *mission_index, DM_PERSIST_IN_FLIGHT_RESET,
					     new_mission_item, len) != len) {
					/* not supposed to happen unless the datamanager can't access the dataman */
					return false;
				}
			}
			/* set new mission item index and repeat
			 * we don't have to validate here, if it's invalid, we should realize this later .*/
			*mission_index = new_mission_item->do_jump_mission_index;

		} else {
			/* if it's not a DO_JUMP, then we were successful */
			return true;
		}
	}

	/* we have given up, we don't want to cycle forever */
	warnx("ERROR: cycling through mission items without success");
	return false;
}

void
Mission::report_mission_item_reached()
{
	if (_mission_type == MISSION_TYPE_OFFBOARD) {
		_mission_result.mission_reached = true;
		_mission_result.mission_index_reached = _offboard_mission.current_index;
	}
	publish_mission_result();
}

void
Mission::report_current_offboard_mission_item()
{
	_mission_result.index_current_mission = _offboard_mission.current_index;
	publish_mission_result();
}

void
Mission::publish_mission_result()
{
	/* lazily publish the mission result only once available */
	if (_mission_result_pub > 0) {
		/* publish mission result */
		orb_publish(ORB_ID(mission_result), _mission_result_pub, &_mission_result);

	} else {
		/* advertise and publish */
		_mission_result_pub = orb_advertise(ORB_ID(mission_result), &_mission_result);
	}
	/* reset reached bool */
	_mission_result.mission_reached = false;
}
