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

void Mission::updateMissionRouteCache()
{
	_navigator->get_mission_route_cache()->update(_mission);
}

void Mission::syncMissionRouteState()
{
	const bool mission_changed = (_route_state_mission_id != _mission.mission_id)
				     || (_route_state_mission_count != _mission.count)
				     || (_route_state_mission_dataman_id != _mission.mission_dataman_id);

	if (mission_changed) {
		resetJoinRouteState();
		_last_flown_loop_segment = {};
		_route_state_mission_id = _mission.mission_id;
		_route_state_mission_count = _mission.count;
		_route_state_mission_dataman_id = _mission.mission_dataman_id;
	}

	updateMissionRouteCache();
}

void
Mission::on_inactive()
{
	_vehicle_status_sub.update();

	if (_need_mission_save && _vehicle_status_sub.get().arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		save_mission_state();
	}

	MissionBase::on_inactive();
	syncMissionRouteState();
}

void
Mission::on_activation()
{
	_need_mission_save = true;
	const bool resume_mission_on_previous = (_inactivation_index > 0) && cameraWasTriggering();

	check_mission_valid(true);
	_global_pos_sub.update();
	_vehicle_status_sub.update();
	_land_detected_sub.update();
	syncMissionRouteState();

	// Mission rejoin planning below needs the restarted current_seq immediately. MissionBase::on_activation()
	// performs the same guard again, but after this reset the base-class call is inert.
	checkMissionRestart();

	if (_mission.current_seq == 0) {
		_last_flown_loop_segment = {};
	}

	trySetRouteJoinOnActivation(resume_mission_on_previous);
	MissionBase::on_activation();
}

void Mission::on_active()
{
	MissionBase::on_active();
	syncMissionRouteState();
}

bool Mission::trySetRouteJoinOnActivation(bool resume_mission_on_previous)
{
	resetJoinRouteState();

	if (resume_mission_on_previous
	    || (_param_mis_route_join.get() == 0)
	    || _work_item_type != WorkItemType::WORK_ITEM_TYPE_DEFAULT
	    || _land_detected_sub.get().landed
	    || _mission.count < 2
	    || _mission.current_seq < 0
	    || _mission.current_seq >= _mission.count) {
		return false;
	}

	MissionRouteCache *mission_route_cache = _navigator->get_mission_route_cache();

	if ((mission_route_cache == nullptr) || (mission_route_cache->isReady(_mission) == false)) {
		return false;
	}

	const auto *global_position = _navigator->get_global_position();

	if ((global_position == nullptr)
	    || (PX4_ISFINITE(global_position->lat) == false)
	    || (PX4_ISFINITE(global_position->lon) == false)
	    || (PX4_ISFINITE(global_position->alt) == false)) {
		return false;
	}

	MissionRoutePlanner planner(*mission_route_cache);
	const auto &vehicle_status = _vehicle_status_sub.get();
	const auto *local_position = _navigator->get_local_position();

	MissionRoutePlanner::Config config{};
	config.parameters.vehicle_projection_search_dist = (vehicle_status.vehicle_type
			== vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)
			? _param_mis_mc_seg_dist.get() : _param_mis_fw_seg_dist.get();
	config.parameters.acceptance_radius = _navigator->get_acceptance_radius();
	config.parameters.home_altitude_amsl = _navigator->home_global_position_valid()
					       ? _navigator->get_home_position()->alt : global_position->alt;

	config.state.velocity_valid = (local_position != nullptr) && PX4_ISFINITE(local_position->vx) && PX4_ISFINITE(local_position->vy);
	config.state.velocity_ne(0) = (local_position != nullptr) ? local_position->vx : NAN;
	config.state.velocity_ne(1) = (local_position != nullptr) ? local_position->vy : NAN;
	config.state.is_fixed_wing = vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	config.state.in_transition_to_fw = vehicle_status.in_transition_to_fw;
	config.state.is_flying_reverse = false;

	config.last_flown_loop_segment = _last_flown_loop_segment;

	MissionRoutePlanner::JoinPlan join_plan{};
	MissionRoutePlanner::FailureReason failure_reason{MissionRoutePlanner::FailureReason::Unknown};
	MissionRoutePlanner::Position vehicle_position{global_position->lat, global_position->lon, global_position->alt};

	if (!planner.planMissionResumeJoin(vehicle_position, _mission.current_seq,
					   config, join_plan, failure_reason)) {
		PX4_ERR("Mission route rejoin unavailable: %s", MissionRoutePlanner::failureReasonString(failure_reason));
		return false;
	}

	if (!join_plan.valid()
	    || join_plan.path.first_item_index < 0
	    || join_plan.path.first_item_index >= _mission.count) {
		PX4_ERR("Mission route rejoin failed to build a valid join plan");
		return false;
	}

	// Keep this cache loop-only. Nominal segments are already implied by current_seq,
	// but an active DO_JUMP edge must survive the jump so later replans can stay anchored on it.
	_last_flown_loop_segment = join_plan.projection_context.seg_candidate.segment.validLoop()
				   ? join_plan.projection_context.seg_candidate.segment
				   : MissionRoutePlanner::Segment{};
	setMissionIndex(join_plan.path.first_item_index);
	_is_current_planned_mission_item_valid = isMissionValid();

	setupJoinRoute(join_plan.join_context, join_plan.path);

	PX4_INFO("Mission route join: target=%d rev=%u vtol=%u skip_alt=%u",
		 static_cast<int>(join_plan.path.first_item_index),
		 static_cast<unsigned>(join_plan.path.direction_reversed),
		 static_cast<unsigned>(join_plan.join_context.transition_action),
		 static_cast<unsigned>(join_plan.join_context.skip_altitude_requirement));

	mavlink_log_info(_navigator->get_mavlink_log_pub(), "Rejoining mission route\t");
	return true;
}

bool
Mission::set_current_mission_index(uint16_t index)
{
	if (index == _mission.current_seq) {
		return true;
	}

	if (_navigator->get_mission_result()->valid && (index < _mission.count)) {
		if (goToItem(index, true) != PX4_OK) {
			// Keep the old mission index (it was not updated by the interface) and report back.
			return false;
		}

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
		resetJoinRouteState();
		_last_flown_loop_segment = {};

		return true;
	}

	return false;
}

bool Mission::setNextMissionItem()
{
	updateLastFlownLoopSegmentForNominalAdvance(_last_flown_loop_segment);
	return (goToNextItem(true) == PX4_OK);
}

bool
Mission::do_need_move_to_takeoff()
{
	if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF) {

		float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
				  _global_pos_sub.get().lat, _global_pos_sub.get().lon);

		return d_current > _navigator->get_acceptance_radius();
	}

	return false;
}

void Mission::setActiveMissionItems()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	const position_setpoint_s current_setpoint_copy = pos_sp_triplet->current;


	// The join-route pipeline uses synthetic virtual waypoints (branch-in projection and
	// post-join transitions) that do not exist in the dataman cache.
	// If handleJoinRouteWorkItems returns true, it has fully generated, issued, and
	// published the virtual setpoint for this control cycle.
	if (handleJoinRouteWorkItems(pos_sp_triplet, current_setpoint_copy)) {
		// Early return to avoid evaluate look-aheads (e.g., getNextPositionItems)
		// against _mission.current_seq, which is invalid while navigating
		// to a virtual projection, and would overwrite the virtual mission item.
		return;
	}

	/* Get mission item that comes after current if available */
	static constexpr size_t max_num_next_items{2u};
	int32_t next_mission_items_index[max_num_next_items];
	size_t num_found_items;

	getNextPositionItems(_mission.current_seq + 1, next_mission_items_index, num_found_items, max_num_next_items);

	mission_item_s next_mission_items[max_num_next_items];

	for (size_t i = 0U; i < num_found_items; i++) {
		mission_item_s next_mission_item;
		bool success = loadMissionItemFromCache(next_mission_items_index[i], next_mission_item);

		if (success) {
			next_mission_items[i] = next_mission_item;

		} else {
			num_found_items = i;
			break;
		}
	}

	/*********************************** handle mission item *********************************************/
	WorkItemType new_work_item_type = WorkItemType::WORK_ITEM_TYPE_DEFAULT;

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

	if (item_contains_position(_mission_item)) {

		handleTakeoff(new_work_item_type, next_mission_items, num_found_items);

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
				new_work_item_type == WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND;

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
		handleVtolTransition(new_work_item_type, next_mission_items, num_found_items);
	}

	// Only set the previous position item if the current one really changed
	if ((_work_item_type != WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND) &&
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
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

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

	/* if we just did a VTOL takeoff, prepare transition */
	if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF &&
	    _work_item_type == WorkItemType::WORK_ITEM_TYPE_CLIMB &&
	    _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
	    !_land_detected_sub.get().landed) {

		/* set yaw setpoint to heading of VTOL_TAKEOFF wp against current position */
		_mission_item.yaw = get_bearing_to_next_waypoint(
					    _global_pos_sub.get().lat, _global_pos_sub.get().lon,
					    _mission_item.lat, _mission_item.lon);

		_mission_item.force_heading = true;

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING;

		/* set position setpoint to current while aligning */
		_mission_item.lat = _global_pos_sub.get().lat;
		_mission_item.lon = _global_pos_sub.get().lon;
	}

	/* heading is aligned now, prepare transition */
	if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF &&
	    _work_item_type == WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING &&
	    _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
	    !_land_detected_sub.get().landed) {

		/* check if the vtol_takeoff waypoint is on top of us */
		if (do_need_move_to_takeoff()) {
			new_work_item_type = WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF;
		}

		set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
		_mission_item.yaw = NAN;

		// keep current setpoints (FW position controller generates wp to track during transition)
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
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

void Mission::handleVtolTransition(WorkItemType &new_work_item_type, mission_item_s next_mission_items[],
				   size_t &num_found_items)
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* turn towards next waypoint before MC to FW transition */
	if (_mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
	    && _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT
	    && new_work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT
	    && _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && !_land_detected_sub.get().landed
	    && (num_found_items > 0u)) {

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING;

		set_align_mission_item(&_mission_item, &next_mission_items[0u]);

		/* set position setpoint to target during the transition */
		mission_item_to_position_setpoint(next_mission_items[0u], &pos_sp_triplet->current);
	}

	/* yaw is aligned now */
	if (_work_item_type == WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING &&
	    new_work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) {

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_DEFAULT;

		pos_sp_triplet->previous = pos_sp_triplet->current;
		// keep current setpoints (FW position controller generates wp to track during transition)
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
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
