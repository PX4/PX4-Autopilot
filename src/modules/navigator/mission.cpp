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

#include <string.h>
#include <stdlib.h>

#include <dataman/dataman.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission_result.h>

#include "mission.h"


Mission::Mission() :

	_offboard_dataman_id(-1),
	_current_offboard_mission_index(0),
	_current_onboard_mission_index(0),
	_offboard_mission_item_count(0),
	_onboard_mission_item_count(0),
	_onboard_mission_allowed(false),
	_current_mission_type(MISSION_TYPE_NONE),
	_mission_result_pub(-1),
	_mission_result({})
{
}

Mission::~Mission()
{
}

void
Mission::set_offboard_dataman_id(const int new_id)
{
	_offboard_dataman_id = new_id;
}

void
Mission::set_offboard_mission_count(int new_count)
{
	_offboard_mission_item_count = new_count;
}

void
Mission::set_onboard_mission_count(int new_count)
{
	_onboard_mission_item_count = new_count;
}

void
Mission::set_onboard_mission_allowed(bool allowed)
{
	_onboard_mission_allowed = allowed;
}

bool
Mission::command_current_offboard_mission_index(const int new_index)
{
	if (new_index >= 0) {
		_current_offboard_mission_index = (unsigned)new_index;
	} else {

		/* if less WPs available, reset to first WP */
		if (_current_offboard_mission_index >= _offboard_mission_item_count) {
			_current_offboard_mission_index = 0;
		}
	}
	report_current_offboard_mission_item();
}

bool
Mission::command_current_onboard_mission_index(int new_index)
{
	if (new_index != -1) {
		_current_onboard_mission_index = (unsigned)new_index;
	} else {

		/* if less WPs available, reset to first WP */
		if (_current_onboard_mission_index >= _onboard_mission_item_count) {
			_current_onboard_mission_index = 0;
		}
	}
	// TODO: implement this for onboard missions as well
	// report_current_mission_item();
}

bool
Mission::get_current_mission_item(struct mission_item_s *new_mission_item, bool *onboard, int *index)
{
	*onboard = false;
	*index = -1;

	/* try onboard mission first */
	if (_current_onboard_mission_index < _onboard_mission_item_count && _onboard_mission_allowed) {
		if (read_mission_item(DM_KEY_WAYPOINTS_ONBOARD, true, &_current_onboard_mission_index, new_mission_item)) {
			_current_mission_type = MISSION_TYPE_ONBOARD;
			*onboard = true;
			*index = _current_onboard_mission_index;

			return true;
		}
	}

	/* otherwise fallback to offboard */
	if (_current_offboard_mission_index < _offboard_mission_item_count) {

		dm_item_t dm_current;
		if (_offboard_dataman_id == 0) {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_0;
		} else {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_1;
		}
		if (read_mission_item(dm_current, true, &_current_offboard_mission_index, new_mission_item)) {

			_current_mission_type = MISSION_TYPE_OFFBOARD;
			*onboard = false;
			*index = _current_offboard_mission_index;

			return true;
		}
	}

	/* happens when no more mission items can be added as a next item */
	_current_mission_type = MISSION_TYPE_NONE;

	return false;
}

bool
Mission::get_next_mission_item(struct mission_item_s *new_mission_item)
{
	int next_temp_mission_index = _current_onboard_mission_index + 1;

	/* try onboard mission first */
	if (next_temp_mission_index < _onboard_mission_item_count && _onboard_mission_allowed) {
		if (read_mission_item(DM_KEY_WAYPOINTS_ONBOARD, false, &next_temp_mission_index, new_mission_item)) {
			return true;
		}
	}

	/* then try offboard mission */
	dm_item_t dm_current;
	if (_offboard_dataman_id == 0) {
		dm_current = DM_KEY_WAYPOINTS_OFFBOARD_0;
	} else {
		dm_current = DM_KEY_WAYPOINTS_OFFBOARD_1;
	}
	next_temp_mission_index = _current_offboard_mission_index + 1;
	if (next_temp_mission_index < _offboard_mission_item_count) {
		if (read_mission_item(dm_current, false, &next_temp_mission_index, new_mission_item)) {
			return true;
		}
	}

	/* both failed, bail out */
	return false;
}

bool
Mission::read_mission_item(const dm_item_t dm_item, bool is_current, int *mission_index, struct mission_item_s *new_mission_item)
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

			if (new_mission_item->do_jump_current_count < new_mission_item->do_jump_repeat_count) {

				/* only raise the repeat count if this is for the current mission item
				 * but not for the next mission item */
				if (is_current) {
					(new_mission_item->do_jump_current_count)++;

					/* save repeat count */
					if (dm_write(dm_item, *mission_index, DM_PERSIST_IN_FLIGHT_RESET, new_mission_item, len) != len) {
						/* not supposed to happen unless the datamanager can't access the SD card, etc. */
						return false;
					}
				}
				/* set new mission item index and repeat
				 * we don't have to validate here, if it's invalid, we should realize this later .*/
				*mission_index = new_mission_item->do_jump_mission_index;
			} else {
				return false;
			}

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
Mission::move_to_next()
{
	report_mission_item_reached();

	switch (_current_mission_type) {
	case MISSION_TYPE_ONBOARD:
		_current_onboard_mission_index++;
		break;

	case MISSION_TYPE_OFFBOARD:
		_current_offboard_mission_index++;
		break;

	case MISSION_TYPE_NONE:
	default:
		break;
	}
}

void
Mission::report_mission_item_reached()
{
	if (_current_mission_type == MISSION_TYPE_OFFBOARD) {
		_mission_result.mission_reached = true;
		_mission_result.mission_index_reached = _current_offboard_mission_index;
	}
	publish_mission_result();
}

void
Mission::report_current_offboard_mission_item()
{
	_mission_result.index_current_mission = _current_offboard_mission_index;
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

