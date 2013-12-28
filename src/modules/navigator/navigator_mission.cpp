/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Julian Oes <joes@student.ethz.ch>
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
 * Helper class to access missions
 */

// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <unistd.h>

#include <stdlib.h>
#include <dataman/dataman.h>
#include "navigator_mission.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;


Mission::Mission() : 
	
	_offboard_dataman_id(-1),
	_current_offboard_mission_index(0),
	_current_onboard_mission_index(0),
	_offboard_mission_item_count(0),
	_onboard_mission_item_count(0),
	_onboard_mission_allowed(false),
	_current_mission_type(MISSION_TYPE_NONE)
{}

Mission::~Mission()
{
	
}

void
Mission::set_offboard_dataman_id(int new_id)
{
	_offboard_dataman_id = new_id;
}

void
Mission::set_current_offboard_mission_index(int new_index)
{
	if (new_index != -1) {
		_current_offboard_mission_index = (unsigned)new_index;
	}
}

void
Mission::set_current_onboard_mission_index(int new_index)
{
	if (new_index != -1) {
		_current_onboard_mission_index = (unsigned)new_index;
	}
}

void
Mission::set_offboard_mission_count(unsigned new_count)
{
	_offboard_mission_item_count = new_count;
}

void
Mission::set_onboard_mission_count(unsigned new_count)
{
	_onboard_mission_item_count = new_count;
}

void
Mission::set_onboard_mission_allowed(bool allowed)
{
	_onboard_mission_allowed = allowed;
}

bool
Mission::current_mission_available()
{
	return (current_onboard_mission_available() || current_offboard_mission_available());

}

bool
Mission::next_mission_available()
{
	return (next_onboard_mission_available() || next_offboard_mission_available());
}

int
Mission::get_current_mission_item(struct mission_item_s *new_mission_item, bool *onboard, unsigned *index)
{
	/* try onboard mission first */
	if (current_onboard_mission_available()) {
		
		const ssize_t len = sizeof(struct mission_item_s);
		if (dm_read(DM_KEY_WAYPOINTS_ONBOARD, _current_onboard_mission_index, new_mission_item, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return ERROR;
		}
		_current_mission_type = MISSION_TYPE_ONBOARD;
		*onboard = true;
		*index = _current_onboard_mission_index;

	/* otherwise fallback to offboard */
	} else if (current_offboard_mission_available()) {

		dm_item_t dm_current;

		if (_offboard_dataman_id == 0) {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_0;
		} else {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_1;
		}

		const ssize_t len = sizeof(struct mission_item_s);
		if (dm_read(dm_current, _current_offboard_mission_index, new_mission_item, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			_current_mission_type = MISSION_TYPE_NONE;
			return ERROR;
		}
		_current_mission_type = MISSION_TYPE_OFFBOARD;
		*onboard = false;
		*index = _current_offboard_mission_index;

	} else {
		/* happens when no more mission items can be added as a next item */
		_current_mission_type = MISSION_TYPE_NONE;
		return ERROR;
	}

	return OK;
}

int
Mission::get_next_mission_item(struct mission_item_s *new_mission_item)
{
	/* try onboard mission first */
	if (next_onboard_mission_available()) {
		
		const ssize_t len = sizeof(struct mission_item_s);
		if (dm_read(DM_KEY_WAYPOINTS_ONBOARD, _current_onboard_mission_index + 1, new_mission_item, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return ERROR;
		}

	/* otherwise fallback to offboard */
	} else if (next_offboard_mission_available()) {

		dm_item_t dm_current;

		if (_offboard_dataman_id == 0) {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_0;
		} else {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_1;
		}

		const ssize_t len = sizeof(struct mission_item_s);
		if (dm_read(dm_current, _current_offboard_mission_index + 1, new_mission_item, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return ERROR;
		}

	} else {
		/* happens when no more mission items can be added as a next item */
		return ERROR;
	}

	return OK;
}


bool
Mission::current_onboard_mission_available()
{
	return _onboard_mission_item_count > _current_onboard_mission_index && _onboard_mission_allowed;
}

bool
Mission::current_offboard_mission_available()
{
	return _offboard_mission_item_count > _current_offboard_mission_index;
}

bool
Mission::next_onboard_mission_available()
{
	unsigned next = 0;

	if (_current_mission_type != MISSION_TYPE_ONBOARD) {
		next = 1;
	}

	return _onboard_mission_item_count > (_current_onboard_mission_index + next) && _onboard_mission_allowed;
}

bool
Mission::next_offboard_mission_available()
{
	unsigned next = 0;

	if (_current_mission_type != MISSION_TYPE_OFFBOARD) {
		next = 1;
	}

	return _offboard_mission_item_count > (_current_offboard_mission_index + next);
}

void
Mission::move_to_next()
{
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