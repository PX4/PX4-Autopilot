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
#include <mavlink/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>

#include "navigator.h"
#include "mission.h"

Mission::Mission(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_param_onboard_enabled(this, "ONBOARD_EN"),
	_onboard_mission({0}),
	_offboard_mission({0}),
	_current_onboard_mission_index(-1),
	_current_offboard_mission_index(-1),
	_mission_result_pub(-1),
	_mission_result({0}),
	_mission_type(MISSION_TYPE_NONE)
{
	/* load initial params */
	updateParams();
	/* set initial mission items */
	on_inactive();

}

Mission::~Mission()
{
}

void
Mission::on_inactive()
{
	_first_run = true;

	/* check anyway if missions have changed so that feedback to groundstation is given */
	bool onboard_updated;
	orb_check(_navigator->get_onboard_mission_sub(), &onboard_updated);
	if (onboard_updated) {
		update_onboard_mission();
	}

	bool offboard_updated;
	orb_check(_navigator->get_offboard_mission_sub(), &offboard_updated);
	if (offboard_updated) {
		update_offboard_mission();
	}
}

bool
Mission::on_active(struct position_setpoint_triplet_s *pos_sp_triplet)
{
	bool updated = false;

	/* check if anything has changed */
	bool onboard_updated;
	orb_check(_navigator->get_onboard_mission_sub(), &onboard_updated);
	if (onboard_updated) {
		update_onboard_mission();
	}

	bool offboard_updated;
	orb_check(_navigator->get_offboard_mission_sub(), &offboard_updated);
	if (offboard_updated) {
		update_offboard_mission();
	}

	/* reset mission items if needed */
	if (onboard_updated || offboard_updated || _first_run) {
		set_mission_items(pos_sp_triplet);
		updated = true;
		_first_run = false;
	}

	/* lets check if we reached the current mission item */
	if (_mission_type != MISSION_TYPE_NONE && is_mission_item_reached()) {
		advance_mission();
		set_mission_items(pos_sp_triplet);
		updated = true;
	}

	return updated;
}

void
Mission::update_onboard_mission()
{
	if (orb_copy(ORB_ID(onboard_mission), _navigator->get_onboard_mission_sub(), &_onboard_mission) == OK) {
		/* accept the current index set by the onboard mission if it is within bounds */
		if (_onboard_mission.current_index >=0
		&& _onboard_mission.current_index < (int)_onboard_mission.count) {
			_current_onboard_mission_index = _onboard_mission.current_index;
		} else {
			/* if less WPs available, reset to first WP */
			if (_current_onboard_mission_index >= (int)_onboard_mission.count) {
				_current_onboard_mission_index = 0;
			/* if not initialized, set it to 0 */
			} else if (_current_onboard_mission_index < 0) {
				_current_onboard_mission_index = 0;
			}
			/* otherwise, just leave it */
		}
	} else {
		_onboard_mission.count = 0;
		_onboard_mission.current_index = 0;
		_current_onboard_mission_index = 0;
	}
}

void
Mission::update_offboard_mission()
{
	if (orb_copy(ORB_ID(offboard_mission), _navigator->get_offboard_mission_sub(), &_offboard_mission) == OK) {

		/* determine current index */
		if (_offboard_mission.current_index >= 0
		    && _offboard_mission.current_index < (int)_offboard_mission.count) {
			_current_offboard_mission_index = _offboard_mission.current_index;
		} else {
			/* if less WPs available, reset to first WP */
			if (_current_offboard_mission_index >= (int)_offboard_mission.count) {
				_current_offboard_mission_index = 0;
			/* if not initialized, set it to 0 */
			} else if (_current_offboard_mission_index < 0) {
				_current_offboard_mission_index = 0;
			}
			/* otherwise, just leave it */
		}

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
							      _navigator->get_geofence(),
							      _navigator->get_home_position()->alt);
	} else {
		_offboard_mission.count = 0;
		_offboard_mission.current_index = 0;
		_current_offboard_mission_index = 0;
	}
	report_current_offboard_mission_item();
}


void
Mission::advance_mission()
{
	switch (_mission_type) {
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
Mission::set_mission_items(struct position_setpoint_triplet_s *pos_sp_triplet)
{
	set_previous_pos_setpoint(pos_sp_triplet);

	/* try setting onboard mission item */
	if (is_current_onboard_mission_item_set(&pos_sp_triplet->current)) {
		/* if mission type changed, notify */
		if (_mission_type != MISSION_TYPE_ONBOARD) {
			mavlink_log_info(_navigator->get_mavlink_fd(),
					"#audio: onboard mission running");
		}
		_mission_type = MISSION_TYPE_ONBOARD;
		_navigator->set_can_loiter_at_sp(false);

	/* try setting offboard mission item */
	} else if (is_current_offboard_mission_item_set(&pos_sp_triplet->current)) {
		/* if mission type changed, notify */
		if (_mission_type != MISSION_TYPE_OFFBOARD) {
			mavlink_log_info(_navigator->get_mavlink_fd(),
					"#audio: offboard mission running");
		}
		_mission_type = MISSION_TYPE_OFFBOARD;
		_navigator->set_can_loiter_at_sp(false);
	} else {
		if (_mission_type != MISSION_TYPE_NONE) {
			mavlink_log_info(_navigator->get_mavlink_fd(),
					"#audio: mission finished");
		} else {
			mavlink_log_info(_navigator->get_mavlink_fd(),
					"#audio: no mission available");
		}
		_mission_type = MISSION_TYPE_NONE;
        _navigator->set_can_loiter_at_sp(pos_sp_triplet->current.valid && _waypoint_position_reached);

		set_loiter_item(pos_sp_triplet);
		reset_mission_item_reached();
		report_mission_finished();
	}
}

bool
Mission::is_current_onboard_mission_item_set(struct position_setpoint_s *current_pos_sp)
{
	/* make sure param is up to date */
	updateParams();
	if (_param_onboard_enabled.get() > 0 &&
	    _current_onboard_mission_index >= 0&&
	    _current_onboard_mission_index < (int)_onboard_mission.count) {
		struct mission_item_s new_mission_item;
		if (read_mission_item(DM_KEY_WAYPOINTS_ONBOARD, true, &_current_onboard_mission_index,
					&new_mission_item)) {
			/* convert the current mission item and set it valid */
			mission_item_to_position_setpoint(&new_mission_item, current_pos_sp);
			current_pos_sp->valid = true;

			reset_mission_item_reached();

			/* TODO: report this somehow */
			memcpy(&_mission_item, &new_mission_item, sizeof(struct mission_item_s));
			return true;
		}
	}
	return false;
}

bool
Mission::is_current_offboard_mission_item_set(struct position_setpoint_s *current_pos_sp)
{
	if (_current_offboard_mission_index >= 0 &&
	    _current_offboard_mission_index < (int)_offboard_mission.count) {
		dm_item_t dm_current;
		if (_offboard_mission.dataman_id == 0) {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_0;
		} else {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_1;
		}
		struct mission_item_s new_mission_item;
		if (read_mission_item(dm_current, true, &_current_offboard_mission_index, &new_mission_item)) {
			/* convert the current mission item and set it valid */
			mission_item_to_position_setpoint(&new_mission_item, current_pos_sp);
			current_pos_sp->valid = true;

			reset_mission_item_reached();

			report_current_offboard_mission_item();
			memcpy(&_mission_item, &new_mission_item, sizeof(struct mission_item_s));
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
	if (_onboard_mission.current_index >= 0 &&
	    next_temp_mission_index < (int)_onboard_mission.count) {
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
	warnx("next index: %d, count; %d", next_temp_mission_index, _offboard_mission.count);
	if (_offboard_mission.current_index >= 0 &&
	    next_temp_mission_index < (int)_offboard_mission.count) {
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
			mavlink_log_critical(_navigator->get_mavlink_fd(),
			                     "#audio: ERROR waypoint could not be read");
			return false;
		}

		/* check for DO_JUMP item, and whether it hasn't not already been repeated enough times */
		if (new_mission_item->nav_cmd == NAV_CMD_DO_JUMP) {

			/* do DO_JUMP as many times as requested */
			if (new_mission_item->do_jump_current_count < new_mission_item->do_jump_repeat_count) {

				/* only raise the repeat count if this is for the current mission item
				* but not for the next mission item */
				if (is_current) {
					(new_mission_item->do_jump_current_count)++;
					/* save repeat count */
					if (dm_write(dm_item, *mission_index, DM_PERSIST_IN_FLIGHT_RESET,
						new_mission_item, len) != len) {
						/* not supposed to happen unless the datamanager can't access the
						 * dataman */
						mavlink_log_critical(_navigator->get_mavlink_fd(),
								"#audio: ERROR DO JUMP waypoint could not be written");
						return false;
					}
				}
				/* set new mission item index and repeat
				* we don't have to validate here, if it's invalid, we should realize this later .*/
				*mission_index = new_mission_item->do_jump_mission_index;
			} else {
				mavlink_log_info(_navigator->get_mavlink_fd(),
						 "#audio: DO JUMP repetitions completed");
				/* no more DO_JUMPS, therefore just try to continue with next mission item */
				(*mission_index)++;
			}

		} else {
			/* if it's not a DO_JUMP, then we were successful */
			return true;
		}
	}

	/* we have given up, we don't want to cycle forever */
	mavlink_log_critical(_navigator->get_mavlink_fd(),
			     "#audio: ERROR DO JUMP is cycling, giving up");
	return false;
}

void
Mission::report_mission_item_reached()
{
	if (_mission_type == MISSION_TYPE_OFFBOARD) {
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
Mission::report_mission_finished()
{
	_mission_result.mission_finished = true;
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
	_mission_result.mission_finished = false;
}
