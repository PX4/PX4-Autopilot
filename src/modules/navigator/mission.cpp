/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
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
 * @file mission.cpp
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
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "mission.h"
#include "mission_item_utils.h"
#include "navigator.h"

#include <string.h>
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <lib/geo/geo.h>
#include <navigator/navigation.h>
#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <px4_platform_common/events.h>

using namespace time_literals;

static constexpr int32_t DEFAULT_MISSION_CACHE_SIZE = 10;

Mission::Mission(Navigator *navigator) :
	MissionBase(navigator, DEFAULT_MISSION_CACHE_SIZE, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)
{
}

void
Mission::on_inactive()
{
	_vehicle_status_sub.update();
	_land_detected_sub.update();
	recheckAfterHomeUpdateOnGround();

	if (_need_mission_save && _vehicle_status_sub.get().arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		save_mission_state();
	}

	MissionBase::on_inactive();
}

void
Mission::on_activation()
{
	_need_mission_save = true;

	check_mission_valid(true);

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	_global_pos_sub.update();
	_vehicle_status_sub.update();
	_land_detected_sub.update();

	// Restart before planning the join. MissionBase::on_activation() must not restart again once the
	// join has selected its target, which can be the last item.
	const bool restarted = checkMissionRestart();

	if (restarted) {
		_active_jump_anchor = {};
	}

	// Only a mission flown along its route joins. After the first start, a landing (disarm, also before a restart)
	// or a selection, fly to the current item, also across later pauses, until a position item is reached.
	const bool keep_mission_index = !_mission_has_been_activated || _system_disarmed_while_inactive
					|| _keep_index_until_reached;
	_system_disarmed_while_inactive = false;
	trySetRouteJoinOnActivation(keep_mission_index);
	_keep_index_until_reached = keep_mission_index;
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE

	MissionBase::on_activation();
}

void
Mission::on_active()
{
	_vehicle_status_sub.update();
	_land_detected_sub.update();
	recheckAfterHomeUpdateOnGround();

	MissionBase::on_active();
}

void
Mission::recheckAfterHomeUpdateOnGround()
{
	// takeoff_time is cleared only at disarm, and the land detector reports the takeoff before Commander
	// sets Home for it, so Home updates in flight keep the verdict.
	const vehicle_status_s &status = _vehicle_status_sub.get();

	if ((status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) && (status.takeoff_time == 0)
	    && _land_detected_sub.get().landed && (_mission_check_state == MissionCheckState::Complete)
	    && (_navigator->get_mission_result()->home_position_counter != _navigator->get_home_position()->update_count)) {
		_mission_check_state = MissionCheckState::Unchecked;
	}
}


bool
Mission::set_current_mission_index(int32_t index, bool reset_jump_counters)
{
	if (!_navigator->get_mission_result()->valid || (_mission.count == 0)) {
		return false;
	}

	if ((index != -1) && ((index < 0) || (index >= _mission.count))) {
		return false;
	}

	if ((index == -1) || (index == _mission.current_seq)) {
		// Keep the current mission item unchanged.
		if (reset_jump_counters) {
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
			resetJoinRouteState();
			_active_jump_anchor = {};
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
			resetVtolTransition();
			resetMissionJumpCounter();

			// A reset can bring a finished mission back to a resumable state (mirrors
			// checkMissionRestart()'s use of the same assignment).
			_is_current_planned_mission_item_valid = isMissionValid();

			if (isActive()) {
				update_mission();
				set_mission_items();
			}
		}

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0

		// Selecting the current item keeps it until it is reached, as any selection (-1 selects no item).
		if (index != -1) {
			_keep_index_until_reached = true;
		}

#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE

		return true;
	}

	if (goToItem(index, MissionTraversalType::FollowMissionControlFlow) != PX4_OK) {
		// Keep the old mission index (it was not updated by the interface) and report back.
		return false;
	}

	if (reset_jump_counters) {
		resetMissionJumpCounter();
	}

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	// The user actively moved the mission index: any armed route join or loop anchor is stale.
	resetJoinRouteState();
	_active_jump_anchor = {};
	// Fly to the selected item, also across pauses, until a position item is reached.
	_keep_index_until_reached = true;
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
	resetVtolTransition();

	_is_current_planned_mission_item_valid = true;

	// we start from the first item so can reset the cache
	if (_mission.current_seq == 0) {
		resetItemCache();
	}

	// update mission items if already in active mission
	if (isActive()) {
		// prevent following "previous - current" line
		_navigator->reset_triplets();
		update_mission();
		set_mission_items();
	}

	// User has actively set new index, reset.
	_inactivation_index = -1;

	return true;
}

bool Mission::setNextMissionItem()
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	updateActiveJumpAnchorForNominalAdvance();
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE

	return (goToNextItem() == PX4_OK);
}

bool
Mission::do_need_move_to_takeoff(const mission_item_s &takeoff_item)
{
	if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && takeoff_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF) {

		float d_current = get_distance_to_next_waypoint(takeoff_item.lat, takeoff_item.lon,
				  _global_pos_sub.get().lat, _global_pos_sub.get().lon);

		return d_current > _navigator->get_acceptance_radius();
	}

	return false;
}

void Mission::setActiveMissionItems()
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0

	// The join flies a synthetic waypoint and transition; loading the target item would replace them.
	if (handleJoinRouteWorkItems()) {
		return;
	}

#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE

	/* Get mission item that comes after current if available */
	static constexpr size_t max_num_next_items{2u};
	int32_t next_mission_items_index[max_num_next_items];
	size_t num_found_items;

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

	/*********************************** handle mission item *********************************************/
	WorkItemType new_work_item_type = WorkItemType::WORK_ITEM_TYPE_DEFAULT;

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	const position_setpoint_s current_setpoint_copy = pos_sp_triplet->current;

	/* Skip VTOL/FW Takeoff item if in air, fixed-wing and didn't start the takeoff already*/
	if ((_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF || _mission_item.nav_cmd == NAV_CMD_TAKEOFF) &&
	    (_work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) &&
	    (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) &&
	    !_land_detected_sub.get().landed) {
		if (setNextMissionItem()) {
			if (!loadCurrentMissionItem()) {
				setEndOfMissionItems();
				return;
			}

		} else {
			setEndOfMissionItems();
			return;
		}
	}

	if (mission_item_contains_position(_mission_item)) {

		handleTakeoff(new_work_item_type, next_mission_items, num_found_items);

		if (vtolTransitionActive()) {
			return;
		}

		handleLanding(new_work_item_type, next_mission_items, num_found_items);

		// TODO Precision land needs to be refactored: https://github.com/PX4/Firmware/issues/14320
		if (new_work_item_type != WorkItemType::WORK_ITEM_TYPE_PRECISION_LAND) {
			mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
		}

		// prevent fixed wing lateral guidance from loitering at a waypoint as part of a mission landing if the altitude
		// is not achieved.
		const bool fw_on_mission_landing = _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
						   && isLanding() &&
						   _mission_item.nav_cmd == NAV_CMD_WAYPOINT;
		const bool mc_landing_after_transition = _vehicle_status_sub.get().vehicle_type ==
				vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && _vehicle_status_sub.get().is_vtol &&
				(new_work_item_type == WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND ||
				 new_work_item_type == WorkItemType::WORK_ITEM_TYPE_ALIGN_TO_LAND);

		if (fw_on_mission_landing || mc_landing_after_transition) {
			pos_sp_triplet->current.alt_acceptance_radius = FLT_MAX;
		}

		// Allow a rotary wing vehicle to decelerate before reaching a wp with a hold time or a timeout
		// This is done by setting the position triplet's next position's valid flag to false,
		// which makes the FlightTask disregard the next position
		// TODO: Setting the next waypoint's validity flag to handle braking / correct waypoint behavior
		// seems hacky, handle this more properly.
		const bool brake_for_hold = _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					    && (get_time_inside(_mission_item) > FLT_EPSILON || item_has_timeout(_mission_item));

		if (_mission_item.autocontinue && !brake_for_hold) {
			/* try to process next mission item */
			if (num_found_items >= 1u) {
				/* got next mission item, update setpoint triplet */
				mission_item_to_position_setpoint(next_mission_items[0u], &pos_sp_triplet->next);

			} else {
				/* next mission item is not available */
				pos_sp_triplet->next.valid = false;
			}

		} else {
			/* vehicle will be paused on current waypoint, don't set next item */
			pos_sp_triplet->next.valid = false;
		}

	} else if (item_contains_gate(_mission_item)) {
		// The mission item is a gate, let's check if the next item in the list provides
		// a position to go towards.

		if (num_found_items > 0u) {
			// We have a position, convert it to the setpoint and update setpoint triplet
			mission_item_to_position_setpoint(next_mission_items[0u], &pos_sp_triplet->current);
		}

		if (num_found_items >= 2u) {
			/* got next mission item, update setpoint triplet */
			mission_item_to_position_setpoint(next_mission_items[1u], &pos_sp_triplet->next);

		} else {
			pos_sp_triplet->next.valid = false;
		}

	} else if (_mission_item.nav_cmd == NAV_CMD_DELAY) {
		// Invalidate next waypoint to ensure vehicle holds position and doesn't try to track ahead
		pos_sp_triplet->next.valid = false;

	} else {
		if (handleVtolTransition(next_mission_items, num_found_items)) {
			return;
		}
	}

	// Only set the previous position item if the current one really changed
	if ((_work_item_type != WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND) &&
	    (_work_item_type != WorkItemType::WORK_ITEM_TYPE_ALIGN_TO_LAND) &&
	    !position_setpoint_equal(&pos_sp_triplet->current, &current_setpoint_copy)) {
		pos_sp_triplet->previous = current_setpoint_copy;
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

void Mission::handleTakeoff(WorkItemType &new_work_item_type, mission_item_s next_mission_items[],
			    size_t &num_found_items)
{
	/* do climb before going to setpoint if needed and not already executing climb */
	/* in fixed-wing this whole block will be ignored and a takeoff item is always propagated */
	if (PX4_ISFINITE(_mission_init_climb_altitude_amsl) &&
	    _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) {

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_CLIMB;

		/* use current mission item as next position item */
		num_found_items = 1u;
		next_mission_items[0u] = _mission_item;
		next_mission_items[0u].nav_cmd = NAV_CMD_WAYPOINT;

		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Climb to %.1f meters above home\t",
				 (double)(_mission_init_climb_altitude_amsl - _navigator->get_home_position()->alt));
		events::send<float>(events::ID("mission_climb_before_start"), events::Log::Info,
				    "Climb to {1:.1m_v} above home", _mission_init_climb_altitude_amsl - _navigator->get_home_position()->alt);

		if (_land_detected_sub.get().landed) {
			_mission_item.nav_cmd = NAV_CMD_TAKEOFF;

		} else {
			_mission_item.nav_cmd = NAV_CMD_LOITER_TO_ALT;
		}

		_mission_item.lat = _global_pos_sub.get().lat;
		_mission_item.lon = _global_pos_sub.get().lon;
		_mission_item.yaw = NAN; // FlightTaskAuto handles yaw directly
		_mission_item.altitude = _mission_init_climb_altitude_amsl;
		_mission_item.altitude_is_relative = false;
		_mission_item.autocontinue = true;
		_mission_item.time_inside = 0.0f;

		_mission_init_climb_altitude_amsl = NAN;

	} else if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF
		   && _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT
		   && _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

		/* if there is no need to do a takeoff but we have a takeoff item, treat is as waypoint */
		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
		_mission_item.yaw = NAN;

	} else if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
		   && _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) {
		// if the vehicle is already in fixed wing mode then the current mission item
		// will be accepted immediately and the work items will be skipped
		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_CLIMB;


		/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
		_mission_item.yaw = NAN;
	}

	/* if we just did a normal takeoff navigate to the actual waypoint now */
	if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF &&
	    _work_item_type == WorkItemType::WORK_ITEM_TYPE_CLIMB) {

		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
		_mission_item.yaw = NAN;
	}

	/* VTOL takeoff climb done: align towards the takeoff waypoint, then transition to fixed-wing */
	if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
	    && _work_item_type == WorkItemType::WORK_ITEM_TYPE_CLIMB
	    && _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && !_land_detected_sub.get().landed) {
		mission_item_s command{};
		set_vtol_transition_item(&command, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
		auto &next = _navigator->get_position_setpoint_triplet()->next;

		if (_mission_item.autocontinue && num_found_items > 0) {
			mission_item_to_position_setpoint(next_mission_items[0], &next);

		} else {
			next.valid = false;
		}

		// Publish the changed next waypoint even when a back transition delays alignment.
		_navigator->set_position_setpoint_triplet_updated();

		// startVtolTransition() copies the takeoff item before changing _mission_item.
		startVtolTransition(command, &_mission_item, WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF);
		return;
	}

	/* takeoff completed and transitioned, move to takeoff wp as fixed wing */
	if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
	    && _work_item_type == WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF) {

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_DEFAULT;
		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		_mission_item.autocontinue = true;
		_mission_item.time_inside = 0.0f;
	}
}

bool Mission::handleVtolTransition(const mission_item_s next_mission_items[], size_t num_found_items)
{
	if (_mission_item.nav_cmd != NAV_CMD_DO_VTOL_TRANSITION) {
		return false;
	}

	// Invalid requests keep the existing command/reach behaviour.
	const float mode = _mission_item.params[0];

	if (!PX4_ISFINITE(mode) || mode < vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC
	    || mode >= vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW + 1) {
		return false;
	}

	startVtolTransition(_mission_item, num_found_items > 0 ? &next_mission_items[0] : nullptr,
			    WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	return true;
}

void Mission::prepareVtolTransitionItem(bool aligning)
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0

	if (_vtol_transition.completion_work == WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN) {
		prepareRouteTransitionItem(aligning);
		return;
	}

#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE

	auto &triplet = *_navigator->get_position_setpoint_triplet();
	const position_setpoint_s current_setpoint_copy = triplet.current;
	const bool takeoff = _vtol_transition.completion_work == WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF;
	const bool front_transition = int(_vtol_transition.command.params[0]) == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;

	if (aligning) {
		if (takeoff) {
			// Retain VTOL_TAKEOFF's altitude-only acceptance while turning at the current position.
			_mission_item = _vtol_transition.target;
			_mission_item.yaw = get_bearing_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon,
					    _mission_item.lat, _mission_item.lon);
			_mission_item.force_heading = true;
			_mission_item.lat = _global_pos_sub.get().lat;
			_mission_item.lon = _global_pos_sub.get().lon;
			mission_item_to_position_setpoint(_mission_item, &triplet.current);

		} else {
			_mission_item = _vtol_transition.command;
			set_align_mission_item(&_mission_item, &_vtol_transition.target);
			// The command's param 2 (immediate transition) shares its slot with the acceptance radius.
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			// Check alignment where the vehicle is: a back transition can end past the reached waypoint.
			_mission_item.lat = _global_pos_sub.get().lat;
			_mission_item.lon = _global_pos_sub.get().lon;
			_mission_item.altitude = _global_pos_sub.get().alt;
			// Nominal missions publish the next target while checking the alignment item.
			mission_item_to_position_setpoint(_vtol_transition.target, &triplet.current);
		}

	} else {
		// Without an alignment during a front transition, publish the target as the alignment would.
		const bool publish_target = front_transition && _vehicle_status_sub.get().in_transition_to_fw
					    && mission_item_contains_position(_vtol_transition.target);

		if (takeoff) {
			if (!do_need_move_to_takeoff(_vtol_transition.target)) {
				_vtol_transition.completion_work = WorkItemType::WORK_ITEM_TYPE_DEFAULT;
			}

		} else if (_work_item_type == WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING) {
			triplet.previous = triplet.current;

		} else if (publish_target) {
			mission_item_to_position_setpoint(_vtol_transition.target, &triplet.current);
		}

		_mission_item = _vtol_transition.command;

		if (takeoff || publish_target || _work_item_type == WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING) {
			triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		}
	}

	if (!position_setpoint_equal(&triplet.current, &current_setpoint_copy)) {
		triplet.previous = current_setpoint_copy;
	}
}

void
Mission::save_mission_state()
{
	if (_vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		// Save only while disarmed, as this is a blocking operation
		_need_mission_save = true;
		return;
	}

	_need_mission_save = false;
	mission_s mission_state = {};

	/* read current state */
	bool success = _dataman_client.readSync(DM_KEY_MISSION_STATE, 0, reinterpret_cast<uint8_t *>(&mission_state),
						sizeof(mission_s));

	if (success) {
		/* data read successfully, check dataman ID and items count */
		if (mission_state.mission_dataman_id == _mission.mission_dataman_id && mission_state.count == _mission.count
		    && mission_state.mission_id == _mission.mission_id) {
			/* navigator may modify only sequence, write modified state only if it changed */
			if (mission_state.current_seq != _mission.current_seq) {
				mission_state = _mission;

				success = _dataman_client.writeSync(DM_KEY_MISSION_STATE, 0, reinterpret_cast<uint8_t *>(&mission_state),
								    sizeof(mission_s));

				if (!success) {

					PX4_ERR("Can't save mission state");
				}
			}
		}

	} else {
		/* invalid data, this must not happen and indicates error in mission publisher */
		mission_state = _mission;

		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Invalid mission state.\t");
		/* EVENT
		 * @description No mission or storage failure
		 */
		events::send(events::ID("mission_invalid_mission_state"), events::Log::Error, "Invalid mission state");

		/* write modified state only if changed */
		success = _dataman_client.writeSync(DM_KEY_MISSION_STATE, 0, reinterpret_cast<uint8_t *>(&mission_state),
						    sizeof(mission_s));

		if (!success) {

			PX4_ERR("Can't save mission state");
		}
	}
}
