/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author Simon Wilks <simon@uaventure.com>
 * @author Andreas Antener <andreas@uaventure.com>
 * @author Sander Smeets <sander@droneslab.com>
 */

#include "mission.h"
#include "navigator.h"

#include <drivers/drv_hrt.h>
#include <dataman/dataman.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <navigator/navigation.h>
#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>

Mission::Mission(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_param_onboard_enabled(this, "MIS_ONBOARD_EN", false),
	_param_takeoff_alt(this, "MIS_TAKEOFF_ALT", false),
	_param_dist_1wp(this, "MIS_DIST_1WP", false),
	_param_altmode(this, "MIS_ALTMODE", false),
	_param_yawmode(this, "MIS_YAWMODE", false),
	_param_force_vtol(this, "NAV_FORCE_VT", false),
	_param_fw_climbout_diff(this, "FW_CLMBOUT_DIFF", false),
	_missionFeasibilityChecker(navigator)
{
	updateParams();
}

void
Mission::on_inactive()
{
	/* We need to reset the mission cruising speed, otherwise the
	 * mission velocity which might have been set using mission items
	 * is used for missions such as RTL. */
	_navigator->set_cruising_speed();

	/* Without home a mission can't be valid yet anyway, let's wait. */
	if (!_navigator->home_position_valid()) {
		return;
	}

	if (_inited) {
		/* check if missions have changed so that feedback to ground station is given */
		bool onboard_updated = false;
		orb_check(_navigator->get_onboard_mission_sub(), &onboard_updated);

		if (onboard_updated) {
			update_onboard_mission();
		}

		bool offboard_updated = false;
		orb_check(_navigator->get_offboard_mission_sub(), &offboard_updated);

		if (offboard_updated) {
			update_offboard_mission();
		}

		/* reset the current offboard mission if needed */
		if (need_to_reset_mission(false)) {
			reset_offboard_mission(_offboard_mission);
			update_offboard_mission();
			_navigator->reset_cruising_speed();
		}

	} else {

		/* load missions from storage */
		mission_s mission_state = {};

		dm_lock(DM_KEY_MISSION_STATE);

		/* read current state */
		int read_res = dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s));

		dm_unlock(DM_KEY_MISSION_STATE);

		if (read_res == sizeof(mission_s)) {
			_offboard_mission.dataman_id = mission_state.dataman_id;
			_offboard_mission.count = mission_state.count;
			_current_offboard_mission_index = mission_state.current_seq;
		}

		/* On init let's check the mission, maybe there is already one available. */
		check_mission_valid(false);

		_inited = true;
	}

	/* require takeoff after non-loiter or landing */
	if (!_navigator->get_can_loiter_at_sp() || _navigator->get_land_detected()->landed) {
		_need_takeoff = true;
	}

	/* reset so current mission item gets restarted if mission was paused */
	_work_item_type = WORK_ITEM_TYPE_DEFAULT;
}

void
Mission::on_inactivation()
{
	// Disable camera trigger
	vehicle_command_s cmd = {};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL;
	// Pause trigger
	cmd.param1 = -1.0f;
	cmd.param3 = 1.0f;
	_navigator->publish_vehicle_cmd(&cmd);
}

void
Mission::on_activation()
{
	set_mission_items();

	// unpause triggering if it was paused
	vehicle_command_s cmd = {};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL;
	// unpause trigger
	cmd.param1 = -1.0f;
	cmd.param3 = 0.0f;
	_navigator->publish_vehicle_cmd(&cmd);
}

void
Mission::on_active()
{
	check_mission_valid(false);

	/* check if anything has changed */
	bool onboard_updated = false;
	orb_check(_navigator->get_onboard_mission_sub(), &onboard_updated);

	if (onboard_updated) {
		update_onboard_mission();
	}

	bool offboard_updated = false;
	orb_check(_navigator->get_offboard_mission_sub(), &offboard_updated);

	if (offboard_updated) {
		update_offboard_mission();
	}

	/* reset the current offboard mission if needed */
	if (need_to_reset_mission(true)) {
		reset_offboard_mission(_offboard_mission);
		update_offboard_mission();
		_navigator->reset_cruising_speed();
		offboard_updated = true;
	}

	/* reset mission items if needed */
	if (onboard_updated || offboard_updated) {
		set_mission_items();
	}

	/* lets check if we reached the current mission item */
	if (_mission_type != MISSION_TYPE_NONE && is_navigator_item_reached()) {

		/* If we just completed a takeoff which was inserted before the right waypoint,
		   there is no need to report that we reached it because we didn't. */
		if (_work_item_type != WORK_ITEM_TYPE_TAKEOFF) {
			set_mission_item_reached();
		}

		if (_navigator_item.autocontinue) {
			/* switch to next waypoint if 'autocontinue' flag set */
			advance_mission();
			set_mission_items();
		}

	} else if (_mission_type != MISSION_TYPE_NONE && _param_altmode.get() == MISSION_ALTMODE_FOH) {

		altitude_sp_foh_update();

	} else {
		/* if waypoint position reached allow loiter on the setpoint */
		if (_waypoint_position_reached && _navigator_item.nav_cmd != NAV_CMD_IDLE) {
			_navigator->set_can_loiter_at_sp(true);
		}
	}

	/* check if a cruise speed change has been commanded */
	if (_mission_type != MISSION_TYPE_NONE) {
		cruising_speed_sp_update();
	}

	/* see if we need to update the current yaw heading */
	if ((_param_yawmode.get() != MISSION_YAWMODE_NONE
	     && _param_yawmode.get() < MISSION_YAWMODE_MAX
	     && _mission_type != MISSION_TYPE_NONE)
	    || _navigator->get_vstatus()->is_vtol) {

		heading_sp_update();
	}

	/* check if landing needs to be aborted */
	if ((_navigator_item.nav_cmd == NAV_CMD_LAND)
	    && (_navigator->abort_landing())) {

		do_abort_landing();
	}
}

bool
Mission::set_current_offboard_mission_index(unsigned index)
{
	if (index < _offboard_mission.count) {

		_current_offboard_mission_index = index;
		set_current_offboard_mission_item();

		// update mission items if already in active mission
		if (_navigator->is_planned_mission()) {
			// prevent following "previous - current" line
			_navigator->get_position_setpoint_triplet()->previous.valid = false;
			_navigator->get_position_setpoint_triplet()->current.valid = false;
			set_mission_items();
		}

		return true;
	}

	return false;
}

int
Mission::find_offboard_land_start()
{
	/* find the first MAV_CMD_DO_LAND_START and return the index
	 *  return -1 if not found
	 *
	 * TODO: implement full spec and find closest landing point geographically
	 */

	dm_item_t dm_current = DM_KEY_WAYPOINTS_OFFBOARD(_offboard_mission.dataman_id);

	for (size_t i = 0; i < _offboard_mission.count; i++) {
		struct mission_item_s missionitem = {};
		const ssize_t len = sizeof(missionitem);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return -1;
		}

		if (missionitem.nav_cmd == NAV_CMD_DO_LAND_START) {
			return i;
		}
	}

	return -1;
}

void
Mission::update_onboard_mission()
{
	/* reset triplets */
	_navigator->reset_triplets();

	if (orb_copy(ORB_ID(onboard_mission), _navigator->get_onboard_mission_sub(), &_onboard_mission) == OK) {
		/* accept the current index set by the onboard mission if it is within bounds */
		if (_onboard_mission.current_seq >= 0
		    && _onboard_mission.current_seq < (int)_onboard_mission.count) {

			_current_onboard_mission_index = _onboard_mission.current_seq;

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

		// XXX check validity here as well
		_navigator->get_mission_result()->valid = true;
		/* reset mission failure if we have an updated valid mission */
		_navigator->get_mission_result()->failure = false;

		/* reset reached info as well */
		_navigator->get_mission_result()->reached = false;
		_navigator->get_mission_result()->seq_reached = 0;
		_navigator->get_mission_result()->seq_total = _onboard_mission.count;

		/* reset work item if new mission has been accepted */
		_work_item_type = WORK_ITEM_TYPE_DEFAULT;
		_navigator->increment_mission_instance_count();
		_navigator->set_mission_result_updated();

	} else {
		_onboard_mission.count = 0;
		_onboard_mission.current_seq = 0;
		_current_onboard_mission_index = 0;
	}
}

void
Mission::update_offboard_mission()
{
	bool failed = true;

	/* reset triplets */
	_navigator->reset_triplets();

	if (orb_copy(ORB_ID(offboard_mission), _navigator->get_offboard_mission_sub(), &_offboard_mission) == OK) {
		// The following is not really a warning, but it can be useful to have this message in the log file
		PX4_WARN("offboard mission updated: dataman_id=%d, count=%d, current_seq=%d", _offboard_mission.dataman_id,
			 _offboard_mission.count, _offboard_mission.current_seq);

		/* determine current index */
		if (_offboard_mission.current_seq >= 0 && _offboard_mission.current_seq < (int)_offboard_mission.count) {
			_current_offboard_mission_index = _offboard_mission.current_seq;

		} else {
			/* if less items available, reset to first item */
			if (_current_offboard_mission_index >= (int)_offboard_mission.count) {
				_current_offboard_mission_index = 0;

			} else if (_current_offboard_mission_index < 0) {
				/* if not initialized, set it to 0 */
				_current_offboard_mission_index = 0;
			}

			/* otherwise, just leave it */
		}

		check_mission_valid(true);

		failed = !_navigator->get_mission_result()->valid;

		if (!failed) {
			/* reset mission failure if we have an updated valid mission */
			_navigator->get_mission_result()->failure = false;

			/* reset reached info as well */
			_navigator->get_mission_result()->reached = false;
			_navigator->get_mission_result()->seq_reached = 0;
			_navigator->get_mission_result()->seq_total = _offboard_mission.count;

			/* reset work item if new mission has been accepted */
			_work_item_type = WORK_ITEM_TYPE_DEFAULT;
		}

	} else {
		PX4_ERR("offboard mission update failed, handle: %d", _navigator->get_offboard_mission_sub());
	}

	if (failed) {
		_offboard_mission.count = 0;
		_offboard_mission.current_seq = 0;
		_current_offboard_mission_index = 0;

		PX4_ERR("mission check failed");
	}

	set_current_offboard_mission_item();
}


void
Mission::advance_mission()
{
	/* do not advance mission item if we're processing sub mission work items */
	if (_work_item_type != WORK_ITEM_TYPE_DEFAULT) {
		return;
	}

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
Mission::set_mission_items()
{
	/* reset the altitude foh (first order hold) logic, if altitude foh is enabled (param) a new foh element starts now */
	altitude_sp_foh_reset();

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* the home dist check provides user feedback, so we initialize it to this */
	bool user_feedback_done = false;

	/* mission item that comes after current if available */
	struct mission_item_s mission_item_next_position;
	bool has_next_position_item = false;

	work_item_type new_work_item_type = WORK_ITEM_TYPE_DEFAULT;

	/* try setting onboard mission item */
	if (_param_onboard_enabled.get()
	    && prepare_mission_items(true, &_mission_item, &mission_item_next_position, &has_next_position_item)) {

		/* if mission type changed, notify */
		if (_mission_type != MISSION_TYPE_ONBOARD) {
			mavlink_log_info(_navigator->get_mavlink_log_pub(), "Executing internal mission.");
			user_feedback_done = true;
		}

		_mission_type = MISSION_TYPE_ONBOARD;

		/* try setting offboard mission item */

	} else if (prepare_mission_items(false, &_mission_item, &mission_item_next_position, &has_next_position_item)) {
		/* if mission type changed, notify */
		if (_mission_type != MISSION_TYPE_OFFBOARD) {
			mavlink_log_info(_navigator->get_mavlink_log_pub(), "Executing mission.");
			user_feedback_done = true;
		}

		_mission_type = MISSION_TYPE_OFFBOARD;

	} else {
		/* no mission available or mission finished, switch to loiter */
		if (_mission_type != MISSION_TYPE_NONE) {

			if (_navigator->get_land_detected()->landed) {
				mavlink_log_info(_navigator->get_mavlink_log_pub(), "Mission finished, landed.");

			} else {
				/* https://en.wikipedia.org/wiki/Loiter_(aeronautics) */
				mavlink_log_info(_navigator->get_mavlink_log_pub(), "Mission finished, loitering.");

				/* use last setpoint for loiter */
				_navigator->set_can_loiter_at_sp(true);
			}

			user_feedback_done = true;
		}

		_mission_type = MISSION_TYPE_NONE;

		/* set loiter mission item and ensure that there is a minimum clearance from home */
		set_loiter_item(&_navigator_item, _param_takeoff_alt.get());

		/* update position setpoint triplet  */
		pos_sp_triplet->previous.valid = false;
		navigator_apply_limitation(_navigator_item);
		navigator_item_to_position_setpoint(_navigator_item, &pos_sp_triplet->current);
		pos_sp_triplet->next.valid = false;

		/* reuse setpoint for LOITER only if it's not IDLE */
		_navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER);

		set_mission_finished();

		if (!user_feedback_done) {
			/* only tell users that we got no mission if there has not been any
			 * better, more specific feedback yet
			 * https://en.wikipedia.org/wiki/Loiter_(aeronautics)
			 */

			if (_navigator->get_land_detected()->landed) {
				/* landed, refusing to take off without a mission */
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "No valid mission available, refusing takeoff.");

			} else {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "No valid mission available, loitering.");
			}

			user_feedback_done = true;
		}

		_navigator->set_position_setpoint_triplet_updated();
		return;
	}

	/*********************************** handle mission item *********************************************/
	// convert mission_item to local navigator_item
	//TODO use _navigator_item instead of _mission_item
	_navigator->mission_item_to_navigator_item(&_navigator_item, &_mission_item);
	struct navigator_item_s navigator_item_next_position;
	_navigator->mission_item_to_navigator_item(&navigator_item_next_position, &mission_item_next_position);

	/* handle position navigator items */
	if (item_contains_position(_navigator_item)) {

		/* force vtol land */
		if (_navigator_item.nav_cmd == NAV_CMD_LAND && _param_force_vtol.get() == 1
		    && !_navigator->get_vstatus()->is_rotary_wing
		    && _navigator->get_vstatus()->is_vtol) {

			_navigator_item.nav_cmd = NAV_CMD_VTOL_LAND;
		}

		/* we have a new position item so set previous position setpoint to current */
		set_previous_pos_setpoint();

		/* do takeoff before going to setpoint if needed and not already in takeoff */
		/* in fixed-wing this whole block will be ignored and a takeoff item is always propagated */
		if (do_need_vertical_takeoff() &&
		    _work_item_type == WORK_ITEM_TYPE_DEFAULT &&
		    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

			new_work_item_type = WORK_ITEM_TYPE_TAKEOFF;

			/* use current navigator item as next position item */
			memcpy(&navigator_item_next_position, &_navigator_item, sizeof(struct navigator_item_s));
			navigator_item_next_position.nav_cmd = NAV_CMD_WAYPOINT;
			has_next_position_item = true;

			check_for_takeoff_altitude(&_navigator_item);

			_navigator_item.nav_cmd = NAV_CMD_TAKEOFF;

			/* hold heading for takeoff items */
			_navigator_item.yaw = _navigator->get_local_position()->yaw;
			_navigator_item.autocontinue = true;
			_navigator_item.time_inside = 0.0f;

			// temporary: use both, local and global -> afterwards only local
			_navigator_item.x = _navigator->get_local_position()->x;
			_navigator_item.y = _navigator->get_local_position()->y;

		} else if (_navigator_item.nav_cmd == NAV_CMD_TAKEOFF
			   && _work_item_type == WORK_ITEM_TYPE_DEFAULT
			   && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
			   && _navigator->get_vstatus()->is_rotary_wing) {

			/* if there is no need to do a takeoff but we have a takeoff item, treat is as waypoint */
			_navigator_item.nav_cmd = NAV_CMD_WAYPOINT;
			/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
			_navigator_item.yaw = NAN;
			/* since _navigator_item.time_inside and and _navigator_item.pitch_min build a union, we need to set time_inside to zero
			 * since in NAV_CMD_TAKEOFF mode there is currently no time_inside.
			 * Note also that resetting time_inside to zero will cause pitch_min to be zero as well.
			 */
			_navigator_item.time_inside = 0.0f;

		} else if (_navigator_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
			   && _work_item_type == WORK_ITEM_TYPE_DEFAULT
			   && new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

			if (_navigator->get_vstatus()->is_rotary_wing) {
				/* haven't transitioned yet, trigger vtol takeoff logic below */
				_work_item_type = WORK_ITEM_TYPE_TAKEOFF;

			} else {
				/* already in fixed-wing, go to waypoint */
				_navigator_item.nav_cmd = NAV_CMD_WAYPOINT;
			}

			/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
			_navigator_item.yaw = NAN;
		}

		/* if we just did a normal takeoff navigate to the actual waypoint now */
		if (_navigator_item.nav_cmd == NAV_CMD_TAKEOFF &&
		    _work_item_type == WORK_ITEM_TYPE_TAKEOFF &&
		    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

			_navigator_item.nav_cmd = NAV_CMD_WAYPOINT;
			/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
			_navigator_item.yaw = NAN;
			/* since _navigator_item.time_inside and and _navigator_item.pitch_min build a union, we need to set time_inside to zero
			 * since in NAV_CMD_TAKEOFF mode there is currently no time_inside.
			 */
			_navigator_item.time_inside = 0.0f;
		}

		/* if we just did a VTOL takeoff, prepare transition */
		if (_navigator_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF &&
		    _work_item_type == WORK_ITEM_TYPE_TAKEOFF &&
		    new_work_item_type == WORK_ITEM_TYPE_DEFAULT &&
		    _navigator->get_vstatus()->is_rotary_wing &&
		    !_navigator->get_land_detected()->landed) {

			/* check if the vtol_takeoff waypoint is on top of us */
			if (do_need_move_to_takeoff()) {
				new_work_item_type = WORK_ITEM_TYPE_TRANSITON_AFTER_TAKEOFF;
			}

			_navigator_item.nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
			_navigator_item.params[0] = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;
			_navigator_item.yaw = _navigator->get_local_position()->yaw;

			/* set position setpoint to target during the transition */
			// TODO: if has_next_position_item and use_next set next, or if use_heading set generated
			generate_waypoint_from_heading(&pos_sp_triplet->current, _navigator_item.yaw);
		}

		/* takeoff completed and transitioned, move to takeoff wp as fixed wing */
		if (_navigator_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
		    && _work_item_type == WORK_ITEM_TYPE_TRANSITON_AFTER_TAKEOFF
		    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

			new_work_item_type = WORK_ITEM_TYPE_DEFAULT;
			_navigator_item.nav_cmd = NAV_CMD_WAYPOINT;
			_navigator_item.autocontinue = true;
			_navigator_item.time_inside = 0.0f;
		}

		/* move to land wp as fixed wing */
		if (_navigator_item.nav_cmd == NAV_CMD_VTOL_LAND
		    && _work_item_type == WORK_ITEM_TYPE_DEFAULT
		    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
		    && !_navigator->get_land_detected()->landed) {

			new_work_item_type = WORK_ITEM_TYPE_MOVE_TO_LAND;

			/* use current navigator item as next position item */
			memcpy(&navigator_item_next_position, &_navigator_item, sizeof(struct navigator_item_s));
			has_next_position_item = true;

			float lpos_z = _navigator->get_local_position()->z;

			if (pos_sp_triplet->current.valid
			    && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
				lpos_z = pos_sp_triplet->current.z;
			}

			_navigator_item.z = lpos_z;
			_navigator_item.nav_cmd = NAV_CMD_WAYPOINT;
			_navigator_item.autocontinue = true;
			_navigator_item.time_inside = 0.0f;
			_navigator_item.vtol_back_transition = true;
		}

		/* transition to MC */
		if (_navigator_item.nav_cmd == NAV_CMD_VTOL_LAND
		    && _work_item_type == WORK_ITEM_TYPE_MOVE_TO_LAND
		    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
		    && !_navigator->get_vstatus()->is_rotary_wing
		    && !_navigator->get_land_detected()->landed) {

			_navigator_item.nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
			_navigator_item.params[0] = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
			_navigator_item.autocontinue = true;
			new_work_item_type = WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION;
		}

		/* move to landing waypoint before descent if necessary */
		if (do_need_move_to_land() &&
		    (_work_item_type == WORK_ITEM_TYPE_DEFAULT ||
		     _work_item_type == WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION) &&
		    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

			new_work_item_type = WORK_ITEM_TYPE_MOVE_TO_LAND;

			/* use current navigator item as next position item */
			memcpy(&navigator_item_next_position, &_navigator_item, sizeof(struct navigator_item_s));
			has_next_position_item = true;

			/*
			 * Ignoring waypoint z position:
			 * Set z to the same as we have now to prevent descending too fast into
			 * the ground. Actual landing will descend anyway until it touches down.
			 * XXX: We might want to change that at some point if it is clear to the user
			 * what the altitude means on this waypoint type.
			 */

			float lpos_z = _navigator->get_local_position()->z;

			if (pos_sp_triplet->current.valid
			    && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
				lpos_z = pos_sp_triplet->current.z;
			}

			_navigator_item.z = lpos_z;
			_navigator_item.nav_cmd = NAV_CMD_WAYPOINT;
			_navigator_item.autocontinue = true;
			_navigator_item.time_inside = 0.0f;
			_navigator_item.disable_mc_yaw = true;
		}

		/* we just moved to the landing waypoint, now descend */
		if (_work_item_type == WORK_ITEM_TYPE_MOVE_TO_LAND &&
		    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

			new_work_item_type = WORK_ITEM_TYPE_DEFAULT;
			/* XXX: noop */
		}

		/* ignore yaw for landing items */
		/* XXX: if specified heading for landing is desired we could add another step before the descent
		 * that aligns the vehicle first */
		if (_navigator_item.nav_cmd == NAV_CMD_LAND || _navigator_item.nav_cmd == NAV_CMD_VTOL_LAND) {
			_navigator_item.yaw = NAN;
		}

	} else {
		/* handle non-position navigator items such as commands */

		/* turn towards next waypoint before MC to FW transition */
		if (_navigator_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
		    && _work_item_type == WORK_ITEM_TYPE_DEFAULT
		    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
		    && _navigator->get_vstatus()->is_rotary_wing
		    && !_navigator->get_land_detected()->landed
		    && has_next_position_item) {

			new_work_item_type = WORK_ITEM_TYPE_ALIGN;

			set_align_navigator_item(&_navigator_item, &navigator_item_next_position);

			/* set position setpoint to target during the transition */
			navigator_apply_limitation(_navigator_item);
			navigator_item_to_position_setpoint(navigator_item_next_position, &pos_sp_triplet->current);
		}

		/* yaw is aligned now */
		if (_work_item_type == WORK_ITEM_TYPE_ALIGN &&
		    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

			new_work_item_type = WORK_ITEM_TYPE_DEFAULT;

			/* set position setpoint to target during the transition */
			generate_waypoint_from_heading(&pos_sp_triplet->current, pos_sp_triplet->current.yaw);
		}

		/* don't advance mission after FW to MC command */
		if (_navigator_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
		    && _work_item_type == WORK_ITEM_TYPE_DEFAULT
		    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
		    && !_navigator->get_vstatus()->is_rotary_wing
		    && !_navigator->get_land_detected()->landed
		    && pos_sp_triplet->current.valid) {

			new_work_item_type = WORK_ITEM_TYPE_CMD_BEFORE_MOVE;
		}

		/* after FW to MC transition finish moving to the waypoint */
		if (_work_item_type == WORK_ITEM_TYPE_CMD_BEFORE_MOVE &&
		    new_work_item_type == WORK_ITEM_TYPE_DEFAULT
		    && pos_sp_triplet->current.valid) {

			new_work_item_type = WORK_ITEM_TYPE_DEFAULT;

			_navigator_item.nav_cmd = NAV_CMD_WAYPOINT;
			copy_positon_if_valid(&_navigator_item, &pos_sp_triplet->current);
			_navigator_item.autocontinue = true;
			_navigator_item.time_inside = 0;
		}
	}

	/*********************************** set setpoints and check next *********************************************/
	/* set current position setpoint from mission item (is protected against non-position items) */
	navigator_apply_limitation(_navigator_item);
	navigator_item_to_position_setpoint(_navigator_item, &pos_sp_triplet->current);

	/* issue command if ready (will do nothing for position mission items) */
	issue_command(_navigator_item);

	/* set current work item type */
	_work_item_type = new_work_item_type;

	/* require takeoff after landing or idle */
	if (pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LAND
	    || pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {

		_need_takeoff = true;
	}

	_navigator->set_can_loiter_at_sp(false);
	reset_navigator_item_reached();

	if (_mission_type == MISSION_TYPE_OFFBOARD) {
		set_current_offboard_mission_item();
	}

	if (_navigator_item.autocontinue && get_time_inside(_navigator_item) < FLT_EPSILON) {
		/* try to process next mission item */
		if (has_next_position_item) {
			/* got next mission item, update setpoint triplet */
			navigator_apply_limitation(_navigator_item);
			navigator_item_to_position_setpoint(navigator_item_next_position, &pos_sp_triplet->next);

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

		_distance_current_previous = matrix::Vector2f(pos_sp_triplet->current.x - pos_sp_triplet->previous.x,
					     pos_sp_triplet->current.y - pos_sp_triplet->previous.y).length();
	}

	_navigator->set_position_setpoint_triplet_updated();
}

bool
Mission::do_need_vertical_takeoff()
{
	if (_navigator->get_vstatus()->is_rotary_wing) {

		check_for_takeoff_altitude(&_navigator_item);

		if (_navigator->get_land_detected()->landed) {
			/* force takeoff if landed (additional protection) */
			_need_takeoff = true;

		} else if (_navigator->get_local_position()->z < _navigator_item.z - _navigator->get_acceptance_radius()) {
			/* if in-air and already above takeoff height, don't do takeoff */
			_need_takeoff = false;

		} else if (_navigator->get_local_position()->z >= _navigator_item.z + _navigator->get_acceptance_radius()
			   && (_navigator_item.nav_cmd == NAV_CMD_TAKEOFF
			       || _navigator_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF)) {
			/* if in-air but below takeoff height and we have a takeoff item */
			_need_takeoff = true;
		}

		/* check if current mission item is one that requires takeoff before */
		if (_need_takeoff && (
			    _navigator_item.nav_cmd == NAV_CMD_TAKEOFF ||
			    _navigator_item.nav_cmd == NAV_CMD_WAYPOINT ||
			    _navigator_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF ||
			    _navigator_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
			    _navigator_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED)) {

			_need_takeoff = false;
			return true;
		}
	}

	return false;
}

bool
Mission::do_need_move_to_land()
{
	if (_navigator->get_vstatus()->is_rotary_wing
	    && (_navigator_item.nav_cmd == NAV_CMD_LAND || _navigator_item.nav_cmd == NAV_CMD_VTOL_LAND)) {

		return get_horizontal_distance_to_target(_navigator_item) > _navigator->get_acceptance_radius();
	}

	return false;
}

bool
Mission::do_need_move_to_takeoff()
{
	if (_navigator->get_vstatus()->is_rotary_wing && _navigator_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF) {

		return get_horizontal_distance_to_target(_navigator_item) > _navigator->get_acceptance_radius();
	}

	return false;
}

void
Mission::copy_positon_if_valid(struct navigator_item_s *item, struct position_setpoint_s *setpoint)
{
	if (setpoint->valid && setpoint->type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
		item->x = setpoint->x;
		item->y = setpoint->y;
		item->z = setpoint->z;

	} else {
		item->x = _navigator->get_local_position()->x;
		item->y = _navigator->get_local_position()->y;
		item->z = _navigator->get_local_position()->z;
	}

}

void
Mission::set_align_navigator_item(struct navigator_item_s *item, struct navigator_item_s *item_next)
{
	item->nav_cmd = NAV_CMD_WAYPOINT;
	copy_positon_if_valid(item, &(_navigator->get_position_setpoint_triplet()->current));
	item->autocontinue = true;
	item->time_inside = 0.0f;
	item->yaw = _navigator->get_heading_to_target(matrix::Vector2f(_navigator_item.x, _navigator_item.y));
	item->force_heading = true;
}

void
Mission::check_for_takeoff_altitude(struct navigator_item_s *item)
{
	/* at this point the alitude should always be either absolute for global
	 * and relative for local
	 */

	/* takeoff to at least MIS_TAKEOFF_ALT above home/ground, even if first waypoint is lower */
	if (_navigator->get_land_detected()->landed) {
		item->z = fminf(item->z, _navigator->get_local_position()->z - _param_takeoff_alt.get());

	} else {
		item->z = fminf(item->z, _navigator->get_home_position()->z - _param_takeoff_alt.get());

	}

	mavlink_log_info(_navigator->get_mavlink_log_pub(), "Takeoff to %.1f meters above home",
			 (double)(-(item->z - _navigator->get_home_position()->z)));
}

void
Mission::heading_sp_update()
{
	/* we don't want to be yawing during takeoff, landing or aligning for a transition */
	if (_navigator_item.nav_cmd == NAV_CMD_TAKEOFF
	    || _navigator_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
	    || _navigator_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
	    || _navigator_item.nav_cmd == NAV_CMD_LAND
	    || _navigator_item.nav_cmd == NAV_CMD_VTOL_LAND
	    || _work_item_type == WORK_ITEM_TYPE_ALIGN) {

		return;
	}

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* Don't change setpoint if last and current waypoint are not valid */
	if (!pos_sp_triplet->current.valid) {
		return;
	}

	/* set yaw angle for the waypoint if a loiter time has been specified */
	if (_waypoint_position_reached && get_time_inside(_navigator_item) > FLT_EPSILON) {
		// XXX: should actually be param4 from mission item
		// at the moment it will just keep the heading it has
		//_mission_item.yaw = _on_arrival_yaw;
		//pos_sp_triplet->current.yaw = _mission_item.yaw;

	} else {
		/* Calculate direction the vehicle should point to. */
		matrix::Vector2f destination;
		matrix::Vector2f currentpose(_navigator->get_local_position()->x, _navigator->get_local_position()->y);

		/* target location is home */
		if ((_param_yawmode.get() == MISSION_YAWMODE_FRONT_TO_HOME
		     || _param_yawmode.get() == MISSION_YAWMODE_BACK_TO_HOME)
		    // need to be rotary wing for this but not in a transition
		    // in VTOL mode this will prevent updating yaw during FW flight
		    // (which would result in a wrong yaw setpoint spike during back transition)
		    && _navigator->get_vstatus()->is_rotary_wing
		    && !(_navigator_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION || _navigator->get_vstatus()->in_transition_mode)) {
			destination(0) = _navigator->get_home_position()->x;
			destination(1) = _navigator->get_home_position()->y;

		} else if (_param_yawmode.get() == MISSION_YAWMODE_TO_ROI
			   && _navigator->get_vroi().mode == vehicle_roi_s::ROI_LOCATION) {
			/* target location is ROI */

			/* TODO : switch ROI to local as well
			 * point_to_latlon[0] = _navigator->get_vroi()->lat;
			 * point_to_latlon[1] = _navigator->get_vroi()->lon;
			 *
			 */

		} else {
			/* target location is next (current) waypoint */
			destination(0) = pos_sp_triplet->current.x;
			destination(1) = pos_sp_triplet->current.y;
		}

		const float d_current = matrix::Vector2f(destination - currentpose).length();

		if (d_current > _navigator->get_acceptance_radius()) {

			const float yaw = _navigator->get_heading_to_target(currentpose, destination);

			/* always keep the back of the rotary wing pointing towards home */
			if (_param_yawmode.get() == MISSION_YAWMODE_BACK_TO_HOME) {
				_navigator_item.yaw = _wrap_pi(yaw + M_PI_F);
				pos_sp_triplet->current.yaw = _navigator_item.yaw;

			} else {
				_navigator_item.yaw = yaw;
				pos_sp_triplet->current.yaw = _navigator_item.yaw;
			}
		}
	}

	// we set yaw directly so we can run this in parallel to the FOH update
	_navigator->set_position_setpoint_triplet_updated();
}

void
Mission::altitude_sp_foh_update()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* Don't change setpoint if last and current waypoint are not valid
	 * or if the previous altitude isn't from a position or loiter setpoint or
	 * if rotary wing since that is handled in the mc_pos_control
	 */


	if (!pos_sp_triplet->previous.valid || !pos_sp_triplet->current.valid || !PX4_ISFINITE(pos_sp_triplet->previous.z)
	    || !(pos_sp_triplet->previous.type == position_setpoint_s::SETPOINT_TYPE_POSITION ||
		 pos_sp_triplet->previous.type == position_setpoint_s::SETPOINT_TYPE_LOITER) ||
	    _navigator->get_vstatus()->is_rotary_wing) {

		return;
	}

	/* Do not try to find a solution if the last waypoint is inside the acceptance radius of the current one */
	if (_distance_current_previous - _navigator->get_acceptance_radius(_navigator_item.acceptance_radius) < FLT_EPSILON) {
		return;
	}

	/* Don't do FOH for non-missions, landing and takeoff waypoints, the ground may be near
	 * and the FW controller has a custom landing logic
	 *
	 * NAV_CMD_LOITER_TO_ALT doesn't change altitude until reaching desired lat/lon
	 * */
	if (_navigator_item.nav_cmd == NAV_CMD_LAND
	    || _navigator_item.nav_cmd == NAV_CMD_VTOL_LAND
	    || _navigator_item.nav_cmd == NAV_CMD_TAKEOFF
	    || _navigator_item.nav_cmd == NAV_CMD_LOITER_TO_ALT
	    || !_navigator->is_planned_mission()) {

		return;
	}

	/* Calculate distance to current waypoint */
	float d_current = get_horizontal_distance_to_target(_navigator_item);

	/* Save distance to waypoint if it is the smallest ever achieved, however make sure that
	 * _min_current_sp_distance_xy is never larger than the distance between the current and the previous wp */
	_min_current_sp_distance_xy = math::min(math::min(d_current, _min_current_sp_distance_xy),
						_distance_current_previous);

	/* if the minimal distance is smaller then the acceptance radius, we should be at waypoint alt
	 * navigator will soon switch to the next waypoint item (if there is one) as soon as we reach this altitude */
	if (_min_current_sp_distance_xy < _navigator->get_acceptance_radius(_navigator_item.acceptance_radius)) {
		pos_sp_triplet->current.z = _navigator_item.z;

	} else {
		/* update the altitude sp of the 'current' item in the sp triplet, but do not update the altitude sp
		 * of the mission item as it is used to check if the mission item is reached
		 * The setpoint is set linearly and such that the system reaches the current altitude at the acceptance
		 * radius around the current waypoint
		 **/

		float delta_z = _navigator_item.z - pos_sp_triplet->previous.z;
		float grad_z = -delta_z / (_distance_current_previous - _navigator->get_acceptance_radius(
						   _navigator_item.acceptance_radius));
		float a_z = pos_sp_triplet->previous.z - grad_z * _distance_current_previous;
		pos_sp_triplet->current.z = a_z + grad_z * _min_current_sp_distance_xy;
	}


	// we set altitude directly so we can run this in parallel to the heading update
	_navigator->set_position_setpoint_triplet_updated();

}

void
Mission::altitude_sp_foh_reset()
{
	_min_current_sp_distance_xy = FLT_MAX;
}

void
Mission::cruising_speed_sp_update()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	const float cruising_speed = _navigator->get_cruising_speed();

	/* Don't change setpoint if the current waypoint is not valid */
	if (!pos_sp_triplet->current.valid ||
	    fabsf(pos_sp_triplet->current.cruising_speed - cruising_speed) < FLT_EPSILON) {
		return;
	}

	pos_sp_triplet->current.cruising_speed = cruising_speed;

	_navigator->set_position_setpoint_triplet_updated();
}

void
Mission::do_abort_landing()
{
	// Abort FW landing
	//  first climb out then loiter over intended landing location

	if (_navigator_item.nav_cmd != NAV_CMD_LAND) {
		return;
	}

	// loiter at the larger of MIS_LTRMIN_ALT above the landing point
	//  or 2 * FW_CLMBOUT_DIFF above the current altitude
	float z_landing = _navigator_item.z;
	float z_sp = math::min(z_landing - _param_loiter_min_alt.get(),
			       _navigator->get_local_position()->z - (2 * _param_fw_climbout_diff.get()));


	_navigator_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
	_navigator_item.z = z_sp;
	_navigator_item.yaw = NAN;
	_navigator_item.loiter_radius = _navigator->get_loiter_radius();
	_navigator_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
	_navigator_item.acceptance_radius = _navigator->get_acceptance_radius();
	_navigator_item.autocontinue = false;
	_navigator_item.origin = ORIGIN_ONBOARD;

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	navigator_apply_limitation(_navigator_item);
	navigator_item_to_position_setpoint(_navigator_item, &pos_sp_triplet->current);

	_navigator->set_position_setpoint_triplet_updated();

	mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "Holding at %dm above landing.",
				     (int)(-1 * (z_sp - z_landing)));

	// reset mission index to start of landing
	int land_start_index = find_offboard_land_start();

	if (land_start_index != -1) {
		_current_offboard_mission_index = land_start_index;

	} else {
		// move mission index back (landing approach point)
		_current_offboard_mission_index -= 1;
	}

	// send reposition cmd to get out of mission
	vehicle_command_s vcmd = {};
	vcmd.timestamp = hrt_absolute_time();
	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
	vcmd.param1 = -1;
	vcmd.param2 = 1;
	vcmd.param5 = _navigator_item.x; //TODO: check if that makes sense to set param5/6/7 to local
	vcmd.param6 = _navigator_item.z;
	vcmd.param7 = z_sp;

	_navigator->publish_vehicle_cmd(&vcmd);
}

bool
Mission::prepare_mission_items(bool onboard, struct mission_item_s *mission_item,
			       struct mission_item_s *next_position_mission_item, bool *has_next_position_item)
{
	bool first_res = false;
	int offset = 1;

	if (read_mission_item(onboard, 0, mission_item)) {

		first_res = true;

		/* trying to find next position mission item */
		while (read_mission_item(onboard, offset, next_position_mission_item)) {

			if (item_contains_position(*next_position_mission_item)) {
				*has_next_position_item = true;
				break;
			}

			offset++;
		}
	}

	return first_res;
}

bool
Mission::read_mission_item(bool onboard, int offset, struct mission_item_s *mission_item)
{
	/* select onboard/offboard mission */
	int *mission_index_ptr;
	dm_item_t dm_item;

	struct mission_s *mission = (onboard) ? &_onboard_mission : &_offboard_mission;
	int current_index = (onboard) ? _current_onboard_mission_index : _current_offboard_mission_index;
	int index_to_read = current_index + offset;

	/* do not work on empty missions */
	if (mission->count == 0) {
		return false;
	}

	if (onboard) {
		/* onboard mission */
		mission_index_ptr = (offset == 0) ? &_current_onboard_mission_index : &index_to_read;
		dm_item = DM_KEY_WAYPOINTS_ONBOARD;

	} else {
		/* offboard mission */
		mission_index_ptr = (offset == 0) ? &_current_offboard_mission_index : &index_to_read;
		dm_item = DM_KEY_WAYPOINTS_OFFBOARD(_offboard_mission.dataman_id);
	}

	/* Repeat this several times in case there are several DO JUMPS that we need to follow along, however, after
	 * 10 iterations we have to assume that the DO JUMPS are probably cycling and give up. */
	for (int i = 0; i < 10; i++) {

		if (*mission_index_ptr < 0 || *mission_index_ptr >= (int)mission->count) {
			/* mission item index out of bounds - if they are equal, we just reached the end */
			if (*mission_index_ptr != (int)mission->count) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission item index out of bound, index: %d, max: %d.",
						     *mission_index_ptr,
						     (int)mission->count);
			}

			return false;
		}

		const ssize_t len = sizeof(struct mission_item_s);

		/* read mission item to temp storage first to not overwrite current mission item if data damaged */
		struct mission_item_s mission_item_tmp;

		/* read mission item from datamanager */
		if (dm_read(dm_item, *mission_index_ptr, &mission_item_tmp, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Waypoint could not be read.");
			return false;
		}

		/* check for DO_JUMP item, and whether it hasn't not already been repeated enough times */
		if (mission_item_tmp.nav_cmd == NAV_CMD_DO_JUMP) {

			/* do DO_JUMP as many times as requested */
			if (mission_item_tmp.do_jump_current_count < mission_item_tmp.do_jump_repeat_count) {

				/* only raise the repeat count if this is for the current mission item
				 * but not for the read ahead mission item */
				if (offset == 0) {
					(mission_item_tmp.do_jump_current_count)++;

					/* save repeat count */
					if (dm_write(dm_item, *mission_index_ptr, DM_PERSIST_POWER_ON_RESET,
						     &mission_item_tmp, len) != len) {
						/* not supposed to happen unless the datamanager can't access the
						 * dataman */
						mavlink_log_critical(_navigator->get_mavlink_log_pub(), "DO JUMP waypoint could not be written.");
						return false;
					}

					report_do_jump_mission_changed(*mission_index_ptr, mission_item_tmp.do_jump_repeat_count);
				}

				/* set new mission item index and repeat
				 * we don't have to validate here, if it's invalid, we should realize this later .*/
				*mission_index_ptr = mission_item_tmp.do_jump_mission_index;

			} else {
				if (offset == 0) {
					mavlink_log_info(_navigator->get_mavlink_log_pub(), "DO JUMP repetitions completed.");
				}

				/* no more DO_JUMPS, therefore just try to continue with next mission item */
				(*mission_index_ptr)++;
			}

		} else {
			/* if it's not a DO_JUMP, then we were successful */
			memcpy(mission_item, &mission_item_tmp, sizeof(struct mission_item_s));
			return true;
		}
	}

	/* we have given up, we don't want to cycle forever */
	mavlink_log_critical(_navigator->get_mavlink_log_pub(), "DO JUMP is cycling, giving up.");
	return false;
}

void
Mission::save_offboard_mission_state()
{
	mission_s mission_state = {};

	/* lock MISSION_STATE item */
	int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);

	if (dm_lock_ret != 0) {
		PX4_ERR("lock failed");
	}

	/* read current state */
	int read_res = dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s));

	if (read_res == sizeof(mission_s)) {
		/* data read successfully, check dataman ID and items count */
		if (mission_state.dataman_id == _offboard_mission.dataman_id && mission_state.count == _offboard_mission.count) {
			/* navigator may modify only sequence, write modified state only if it changed */
			if (mission_state.current_seq != _current_offboard_mission_index) {
				if (dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission_state,
					     sizeof(mission_s)) != sizeof(mission_s)) {

					mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Can't save mission state.");
				}
			}
		}

	} else {
		/* invalid data, this must not happen and indicates error in offboard_mission publisher */
		mission_state.dataman_id = _offboard_mission.dataman_id;
		mission_state.count = _offboard_mission.count;
		mission_state.current_seq = _current_offboard_mission_index;

		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Invalid mission state.");

		/* write modified state only if changed */
		if (dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission_state,
			     sizeof(mission_s)) != sizeof(mission_s)) {

			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Can't save mission state.");
		}
	}

	/* unlock MISSION_STATE item */
	if (dm_lock_ret == 0) {
		dm_unlock(DM_KEY_MISSION_STATE);
	}
}

void
Mission::report_do_jump_mission_changed(int index, int do_jumps_remaining)
{
	/* inform about the change */
	_navigator->get_mission_result()->item_do_jump_changed = true;
	_navigator->get_mission_result()->item_changed_index = index;
	_navigator->get_mission_result()->item_do_jump_remaining = do_jumps_remaining;
	_navigator->set_mission_result_updated();
}

void
Mission::set_mission_item_reached()
{
	_navigator->get_mission_result()->reached = true;
	_navigator->get_mission_result()->seq_reached = _current_offboard_mission_index;
	_navigator->set_mission_result_updated();
	reset_navigator_item_reached();
}

void
Mission::set_current_offboard_mission_item()
{
	_navigator->get_mission_result()->reached = false;
	_navigator->get_mission_result()->finished = false;
	_navigator->get_mission_result()->seq_current = _current_offboard_mission_index;
	_navigator->set_mission_result_updated();

	save_offboard_mission_state();
}

void
Mission::set_mission_finished()
{
	_navigator->get_mission_result()->finished = true;
	_navigator->set_mission_result_updated();
}

void
Mission::check_mission_valid(bool force)
{
	if ((!_home_inited && _navigator->home_position_valid()) || force) {

		_navigator->get_mission_result()->valid =
			_missionFeasibilityChecker.checkMissionFeasible(_offboard_mission, _param_dist_1wp.get(), false);

		_navigator->get_mission_result()->seq_total = _offboard_mission.count;
		_navigator->increment_mission_instance_count();
		_navigator->set_mission_result_updated();
		_home_inited = _navigator->home_position_valid();
	}
}

void
Mission::reset_offboard_mission(struct mission_s &mission)
{
	dm_lock(DM_KEY_MISSION_STATE);

	if (dm_read(DM_KEY_MISSION_STATE, 0, &mission, sizeof(mission_s)) == sizeof(mission_s)) {
		if (mission.dataman_id >= 0 && mission.dataman_id <= 1) {
			/* set current item to 0 */
			mission.current_seq = 0;

			/* reset jump counters */
			if (mission.count > 0) {
				dm_item_t dm_current = DM_KEY_WAYPOINTS_OFFBOARD(mission.dataman_id);

				for (unsigned index = 0; index < mission.count; index++) {
					struct mission_item_s item;
					const ssize_t len = sizeof(struct mission_item_s);

					if (dm_read(dm_current, index, &item, len) != len) {
						PX4_WARN("could not read mission item during reset");
						break;
					}

					if (item.nav_cmd == NAV_CMD_DO_JUMP) {
						item.do_jump_current_count = 0;

						if (dm_write(dm_current, index, DM_PERSIST_POWER_ON_RESET, &item, len) != len) {
							PX4_WARN("could not save mission item during reset");
							break;
						}
					}
				}
			}

		} else {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Could not read mission.");

			/* initialize mission state in dataman */
			mission.dataman_id = 0;
			mission.count = 0;
			mission.current_seq = 0;
		}

		dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission, sizeof(mission_s));
	}

	dm_unlock(DM_KEY_MISSION_STATE);
}

bool
Mission::need_to_reset_mission(bool active)
{
	/* reset mission state when disarmed */
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED && _need_mission_reset) {
		_need_mission_reset = false;
		return true;

	} else if (_navigator->get_vstatus()->arming_state == vehicle_status_s::ARMING_STATE_ARMED && active) {
		/* mission is running, need reset after disarm */
		_need_mission_reset = true;
	}

	return false;
}

void
Mission::generate_waypoint_from_heading(struct position_setpoint_s *setpoint, float yaw)
{
	// TODO: check if yaw corresponds to NED
	matrix::Quatf q_rot = matrix::AxisAnglef(matrix::Vector3f(0.0f, 0.0f, -1.0f), yaw);
	matrix::Vector3f destination = 1000000.0f * q_rot.conjugate(matrix::Vector3f(1.0f, 0.0f, 0.0f));

	setpoint->x = destination(0);
	setpoint->y = destination(1);
	setpoint->type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	setpoint->yaw = yaw;
}
