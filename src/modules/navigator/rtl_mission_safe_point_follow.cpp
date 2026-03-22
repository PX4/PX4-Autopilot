/***************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file rtl_mission_safe_point_follow.cpp
 *
 * RTL executor that follows the uploaded mission route toward a safe-point
 * branch-off, then leaves the route to land at the chosen safe point.
 * Falls back to the closest mission endpoint when no safe point is available.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "rtl_mission_safe_point_follow.h"
#include "navigator.h"

#include <dataman_client/DatamanClient.hpp>
#include <drivers/drv_hrt.h>

static constexpr int32_t DEFAULT_MISSION_SAFE_POINT_FOLLOW_CACHE_SIZE = 5;

namespace
{

bool isLandingCommand(const mission_item_s &mission_item)
{
	return mission_item.nav_cmd == NAV_CMD_LAND || mission_item.nav_cmd == NAV_CMD_VTOL_LAND;
}

} // namespace

RtlMissionSafePointFollow::RtlMissionSafePointFollow(Navigator *navigator, mission_s mission,
		DatamanCache &full_mission_cache) :
	RtlBase(navigator, DEFAULT_MISSION_SAFE_POINT_FOLLOW_CACHE_SIZE),
	_full_mission_cache(full_mission_cache)
{
	_mission = mission;
}

bool RtlMissionSafePointFollow::loadMissionItemFromCache(int32_t index, mission_item_s &mission_item)
{
	return index >= 0
	       && index < _mission.count
	       && _full_mission_cache.loadWait(static_cast<dm_item_t>(_mission.mission_dataman_id), index,
					       reinterpret_cast<uint8_t *>(&mission_item), sizeof(mission_item),
					       MAX_DATAMAN_LOAD_WAIT);
}

void RtlMissionSafePointFollow::setRoutePlan(const RtlRoutePlanner::Plan &plan)
{
	_plan = plan;
	_branch_off_index = _plan.selection.branchOffIndex();
	updateLastFlownLoopSegmentFromPlan();

	if (_plan.valid() && !_plan.join_context.valid()) {
		_plan.join_context.projection = _plan.projection_context.seg_candidate.projection;
	}

	if (_plan.valid()) {
		_join_requires_back_transition =
			vtolTransitionActionForTarget(_plan.selection.path.first_item_index,
						      _plan.selection.path.direction_reversed) == VtolTransitionAction::BackTransition;
	}
}

void RtlMissionSafePointFollow::on_inactivation()
{
	_should_go_straight_to_goal = _should_go_straight_to_goal
				      || _plan.selection.direct_to_safe_point
				      || _stage == Stage::BranchOff
				      || _stage == Stage::LandAtGoal;

	MissionBase::on_inactivation();
}

void RtlMissionSafePointFollow::on_activation()
{
	_stage = Stage::Idle;
	_branch_off_index = _plan.selection.branchOffIndex();
	updateLastFlownLoopSegmentFromPlan();

	if (_plan.valid()) {
		setMissionIndex(_plan.selection.path.first_item_index);
		_is_current_planned_mission_item_valid = isMissionValid();

		const bool reverse_land_from_takeoff = _plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionTakeoff
						       && _plan.selection.path.direction_reversed
						       && _plan.selection.path.in_first_item_acc_rad;

		_stage = (_should_go_straight_to_goal || _plan.selection.direct_to_safe_point || reverse_land_from_takeoff)
			 ? Stage::LandAtGoal : Stage::JoinRoute;

		PX4_INFO("RTL SRP activate: goal=%u target=%d rev=%u direct=%u stage=%u branch_off=%d",
			 static_cast<unsigned>(_plan.selection.goal_type),
			 static_cast<int>(_plan.selection.path.first_item_index),
			 static_cast<unsigned>(_plan.selection.path.direction_reversed),
			 static_cast<unsigned>(_should_go_straight_to_goal || _plan.selection.direct_to_safe_point),
			 static_cast<unsigned>(_stage),
			 static_cast<int>(_branch_off_index));

	} else {
		_is_current_planned_mission_item_valid = false;
	}

	if (_land_detected_sub.get().landed) {
		_is_current_planned_mission_item_valid = false;
	}

	// Reset the triplet when SRP retargets the mission so the controller does not keep following a stale line.
	_navigator->get_position_setpoint_triplet()->previous.valid = false;
	_navigator->get_position_setpoint_triplet()->current.valid = false;
	_navigator->get_position_setpoint_triplet()->next.valid = false;

	MissionBase::on_activation();
}

bool RtlMissionSafePointFollow::advanceRouteTarget()
{
	bool advanced = false;

	if (_plan.selection.path.direction_reversed) {
		int32_t previous_index = _mission.current_seq;

		if (findPreviousPositionIndexNoJump(previous_index, previous_index)) {
			setMissionIndex(previous_index);
			advanced = true;

		} else {
			PX4_WARN("RTL SRP reverse traversal exhausted, holding position");
			return false;
		}

	} else {
		updateLastFlownLoopSegmentForNominalAdvance();

		// SRP treats the route as geometry only: DO_JUMP items must not be executed as
		// mission control flow (the planner already modelled them as loop edges).  Walk
		// forward through the mission items and skip any DO_JUMP entries instead of
		// following them, mirroring what findPreviousPositionIndexNoJump does for
		// the reverse direction.
		int32_t next_index = _mission.current_seq + 1;

		if (findNextPositionIndexNoJump(next_index, next_index)) {
			setMissionIndex(next_index);
			advanced = true;

		} else {
			PX4_WARN("RTL SRP nominal traversal exhausted, holding position");
			return false;
		}
	}

	if (advanced && currentTargetIsBranchOff()) {
		_stage = Stage::BranchOff;
		PX4_DEBUG("RTL SRP next target is branch-off, leaving route for goal");
	}

	return advanced;
}

bool RtlMissionSafePointFollow::setNextMissionItem()
{
	switch (_stage) {
	case Stage::JoinRoute:
		if (_join_requires_back_transition) {
			_stage = Stage::TransitionAfterJoin;
			PX4_DEBUG("RTL SRP join route reached, applying BT before following route");

		} else if (joinProjectionNearBranchOff()) {
			_should_go_straight_to_goal = true;
			_stage = Stage::LandAtGoal;
			PX4_DEBUG("RTL SRP join projection is already near branch-off, flying straight to goal");

		} else if (currentTargetIsBranchOff()) {
			_stage = Stage::BranchOff;
			PX4_DEBUG("RTL SRP current target is branch-off, leaving route for goal");

		} else {
			_stage = Stage::FollowRoute;
			PX4_DEBUG("RTL SRP join route reached, following route");
		}

		return true;

	case Stage::TransitionAfterJoin:
		if (joinProjectionNearBranchOff()) {
			_should_go_straight_to_goal = true;
			_stage = Stage::LandAtGoal;
			PX4_DEBUG("RTL SRP join transition complete, flying straight to goal");

		} else if (currentTargetIsBranchOff()) {
			_stage = Stage::BranchOff;
			PX4_DEBUG("RTL SRP current target is branch-off after join transition, leaving route for goal");

		} else {
			_stage = Stage::FollowRoute;
			PX4_DEBUG("RTL SRP join transition complete, following route");
		}

		return true;

	case Stage::FollowRoute:
		return advanceRouteTarget();

	case Stage::TransitionDuringRoute:
		// The in-flight transition completed; resume route following from the same target.
		_stage = Stage::FollowRoute;
		_transition_target_index = -1;
		PX4_DEBUG("RTL SRP route transition complete, resuming route following");
		return true;

	case Stage::BranchOff:
		_stage = Stage::LandAtGoal;
		_should_go_straight_to_goal = true;
		PX4_DEBUG("RTL SRP branch-off waypoint reached, landing at goal");
		return true;

	case Stage::LandAtGoal:
	case Stage::Idle:
	default:
		return false;
	}
}

void RtlMissionSafePointFollow::setWaypointMissionItem(mission_item_s &mission_item,
		const RtlRoutePlanner::Position &position, bool autocontinue, bool vtol_back_transition_required) const
{
	mission_item = {};
	mission_item.nav_cmd = NAV_CMD_WAYPOINT;
	mission_item.lat = position.lat;
	mission_item.lon = position.lon;
	mission_item.altitude = position.alt;
	mission_item.altitude_is_relative = false;
	mission_item.acceptance_radius = _navigator->get_acceptance_radius();

	if (vtol_back_transition_required) {
		mission_item.vtol_back_transition = true;

	} else if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
		   || (_vehicle_status_sub.get().in_transition_mode && !_vehicle_status_sub.get().in_transition_to_fw)) {
		// Match the legacy join behavior: use a wider acceptance radius when the vehicle is already in FW-like flow.
		mission_item.acceptance_radius = 2.f * _navigator->get_acceptance_radius();
	}

	mission_item.yaw = NAN;
	mission_item.time_inside = 0.f;
	mission_item.autocontinue = autocontinue;
	mission_item.origin = ORIGIN_ONBOARD;
}

void RtlMissionSafePointFollow::setLandMissionItem(mission_item_s &mission_item) const
{
	mission_item = {};
	mission_item.nav_cmd = _navigator->force_vtol() ? NAV_CMD_VTOL_LAND : NAV_CMD_LAND;
	mission_item.lat = _plan.selection.goal_position.lat;
	mission_item.lon = _plan.selection.goal_position.lon;

	if (_plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionTakeoff && _plan.selection.path.direction_reversed) {
		// Reverse landing at the takeoff point: land at ground level, not at the takeoff altitude.
		mission_item.altitude = _navigator->home_global_position_valid()
					? _navigator->get_home_position()->alt
					: _navigator->get_global_position()->alt;

	} else {
		mission_item.altitude = _plan.selection.goal_position.alt;
	}

	mission_item.altitude_is_relative = false;
	mission_item.land_precision = 2;
	mission_item.autocontinue = false;
	mission_item.origin = ORIGIN_ONBOARD;
}

// During route following, the mission is treated as geometry only.  Position-bearing items such as
// NAV_CMD_LOITER_UNLIMITED, NAV_CMD_LOITER_TIME_LIMIT, and NAV_CMD_LOITER_TO_ALT are converted
// to plain waypoints with autocontinue enabled and zero hold time so the vehicle keeps moving.
// NAV_CMD_DELAY items are non-position and are naturally skipped by findNextPositionIndexNoJump(),
// but if one were encountered it would also be clamped here.
void RtlMissionSafePointFollow::normalizeRouteMissionItem(mission_item_s &mission_item) const
{
	if (!item_contains_position(mission_item)) {
		// Clamp non-position delay commands so the vehicle does not stall on the route.
		if (mission_item.nav_cmd == NAV_CMD_DELAY) {
			mission_item.autocontinue = true;
			mission_item.time_inside = 0.f;
		}

		return;
	}

	mission_item.nav_cmd = NAV_CMD_WAYPOINT;
	mission_item.autocontinue = true;
	mission_item.time_inside = 0.f;
}


bool RtlMissionSafePointFollow::currentTargetIsBranchOff() const
{
	return _plan.selection.safe_point_found
	       && _branch_off_index >= 0
	       && _mission.current_seq == _branch_off_index;
}

bool RtlMissionSafePointFollow::joinProjectionNearBranchOff() const
{
	if (!_plan.selection.safe_point_found || !_plan.join_context.valid() || !_plan.selection.branch_off_projection.valid()) {
		return false;
	}

	const float distance = get_distance_to_next_waypoint(_plan.selection.branch_off_projection.lat,
			       _plan.selection.branch_off_projection.lon,
			       _plan.join_context.projection.lat,
			       _plan.join_context.projection.lon);

	return PX4_ISFINITE(distance) && distance < _navigator->get_acceptance_radius();
}

void RtlMissionSafePointFollow::updateLastFlownLoopSegmentFromPlan()
{
	_last_flown_loop_segment = _plan.projection_context.seg_candidate.segment;
}

void RtlMissionSafePointFollow::updateLastFlownLoopSegmentForNominalAdvance()
{
	_last_flown_loop_segment = {};

	for (int32_t index = _mission.current_seq + 1; index < _mission.count; ++index) {
		mission_item_s mission_item{};

		if (!loadMissionItemFromCache(index, mission_item)) {
			return;
		}

		if (mission_item.nav_cmd == NAV_CMD_DO_JUMP
		    && mission_item.do_jump_current_count < mission_item.do_jump_repeat_count) {
			int32_t loop_start_index = -1;
			int32_t loop_end_index = -1;

			if (!findAttachedPositionIndex(index, loop_start_index)
			    || !findNextPositionIndexNoJump(mission_item.do_jump_mission_index, loop_end_index)) {
				return;
			}

			mission_item_s loop_start_item{};
			mission_item_s loop_end_item{};

			if (!loadMissionItemFromCache(loop_start_index, loop_start_item)
			    || !loadMissionItemFromCache(loop_end_index, loop_end_item)) {
				return;
			}

			_last_flown_loop_segment.start.idx = loop_start_index;
			_last_flown_loop_segment.start.nav_cmd = loop_start_item.nav_cmd;
			_last_flown_loop_segment.end.idx = loop_end_index;
			_last_flown_loop_segment.end.nav_cmd = loop_end_item.nav_cmd;
			_last_flown_loop_segment.is_loop = true;
			const int remaining_loops = static_cast<int>(mission_item.do_jump_repeat_count)
						    - static_cast<int>(mission_item.do_jump_current_count);
			_last_flown_loop_segment.loops_remaining = static_cast<uint8_t>(remaining_loops > 0 ? remaining_loops : 0);

			PX4_DEBUG("RTL SRP on loop jump seg [%u,%u], jump idx: %u",
				  static_cast<unsigned>(_last_flown_loop_segment.start.idx),
				  static_cast<unsigned>(_last_flown_loop_segment.end.idx),
				  static_cast<unsigned>(mission_item.do_jump_mission_index));
			return;
		}

		if (MissionBlock::item_contains_position(mission_item)) {
			break;
		}
	}
}

bool RtlMissionSafePointFollow::loadAdjacentRouteItem(mission_item_s &mission_item, int32_t *adjacent_index)
{
	if (_plan.selection.path.direction_reversed) {
		int32_t adjacent_route_index = _mission.current_seq;

		if (!findPreviousPositionIndexNoJump(adjacent_route_index, adjacent_route_index)) {
			return false;
		}

		if (adjacent_index != nullptr) {
			*adjacent_index = adjacent_route_index;
		}

		return loadMissionItemFromCache(adjacent_route_index, mission_item);

	} else {
		// Walk forward without following DO_JUMP control flow, matching the planner's
		// geometry-only treatment of loop edges.
		int32_t adjacent_route_index = _mission.current_seq + 1;

		if (!findNextPositionIndexNoJump(adjacent_route_index, adjacent_route_index)) {
			return false;
		}

		if (adjacent_index != nullptr) {
			*adjacent_index = adjacent_route_index;
		}

		return loadMissionItemFromCache(adjacent_route_index, mission_item);
	}
}


void RtlMissionSafePointFollow::publishRouteItems(position_setpoint_triplet_s *pos_sp_triplet,
		const position_setpoint_s &current_setpoint_copy, const mission_item_s &current_mission_item,
		const mission_item_s *next_mission_item)
{
	mission_item_to_position_setpoint(current_mission_item, &pos_sp_triplet->current);

	if (next_mission_item != nullptr) {
		mission_item_to_position_setpoint(*next_mission_item, &pos_sp_triplet->next);

	} else {
		_navigator->reset_position_setpoint(pos_sp_triplet->next);
	}

	if (!position_setpoint_equal(&pos_sp_triplet->current, &current_setpoint_copy)) {
		pos_sp_triplet->previous = current_setpoint_copy;
	}

	issue_command(current_mission_item);

	// Synchronize _mission_item with the published waypoint so that
	// is_mission_item_reached_or_completed() checks the correct position.
	// Without this, the reach check uses the raw dataman item (e.g. WP2) while
	// the drone is actually flying to a virtual waypoint (e.g. branch-off projection
	// on segment 1-2), causing the stage machine to stall or misbehave.
	_mission_item = current_mission_item;

	_work_item_type = WorkItemType::WORK_ITEM_TYPE_DEFAULT;
	reset_mission_item_reached();

	if (_mission_type == MissionType::MISSION_TYPE_MISSION) {
		set_mission_result();
	}

	publish_navigator_mission_item();
	_navigator->set_position_setpoint_triplet_updated();
}

void RtlMissionSafePointFollow::publishLandingItems(position_setpoint_triplet_s *pos_sp_triplet,
		const position_setpoint_s &current_setpoint_copy, const mission_item_s &landing_mission_item)
{
	static constexpr size_t max_num_next_items{1U};
	mission_item_s next_mission_items[max_num_next_items] {};
	size_t num_found_items = 0U;
	WorkItemType new_work_item_type{WorkItemType::WORK_ITEM_TYPE_DEFAULT};

	_mission_item = landing_mission_item;
	handleLanding(new_work_item_type, next_mission_items, num_found_items);

	if (num_found_items > 0U) {
		mission_item_to_position_setpoint(next_mission_items[0], &pos_sp_triplet->next);

	} else {
		_navigator->reset_position_setpoint(pos_sp_triplet->next);
	}

	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

	if ((_work_item_type != WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND)
	    && !position_setpoint_equal(&pos_sp_triplet->current, &current_setpoint_copy)) {
		pos_sp_triplet->previous = current_setpoint_copy;
	}

	const bool fw_on_goal_landing = _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
					&& _stage == Stage::LandAtGoal
					&& _mission_item.nav_cmd == NAV_CMD_WAYPOINT;
	const bool mc_landing_after_transition = _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
			&& _vehicle_status_sub.get().is_vtol
			&& new_work_item_type == WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND;

	if (fw_on_goal_landing || mc_landing_after_transition) {
		pos_sp_triplet->current.alt_acceptance_radius = FLT_MAX;
	}

	issue_command(_mission_item);
	_work_item_type = new_work_item_type;
	reset_mission_item_reached();

	if (_mission_type == MissionType::MISSION_TYPE_MISSION) {
		set_mission_result();
	}

	publish_navigator_mission_item();
	_navigator->set_position_setpoint_triplet_updated();
}

void RtlMissionSafePointFollow::setActiveMissionItems()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	const position_setpoint_s current_setpoint_copy = pos_sp_triplet->current;

	switch (_stage) {
	case Stage::JoinRoute: {
			mission_item_s join_item{};
			mission_item_s next_route_item = _mission_item;
			normalizeRouteMissionItem(next_route_item);
			setWaypointMissionItem(join_item, _plan.join_context.projection, true,
					       _join_requires_back_transition);

			if (_plan.join_context.skip_altitude_requirement) {
				join_item.altitude = _navigator->get_global_position()->alt;
			}

			publishRouteItems(pos_sp_triplet, current_setpoint_copy, join_item, &next_route_item);
			break;
		}

	case Stage::TransitionAfterJoin: {
			if (_join_requires_back_transition
			    && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				|| _vehicle_status_sub.get().in_transition_to_fw)
			    && !_land_detected_sub.get().landed) {

				PX4_DEBUG("RTL SRP join route applying BT");
				set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
				_mission_item.yaw = NAN;
				pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
				pos_sp_triplet->previous.valid = false;
				issue_command(_mission_item);
				reset_mission_item_reached();

				if (_mission_type == MissionType::MISSION_TYPE_MISSION) {
					set_mission_result();
				}

				publish_navigator_mission_item();
				_navigator->set_position_setpoint_triplet_updated();
				break;
			}

			_stage = Stage::FollowRoute;
			[[fallthrough]];
		}

	case Stage::FollowRoute: {
			const bool current_item_is_mission_landing = _plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionLand
					&& isLandingCommand(_mission_item);

			// Assume takeoff is always at index 0. Reversing exhausts the route here.
			const bool current_item_is_mission_takeoff = _plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionTakeoff
					&& _mission.current_seq == 0;

			if (current_item_is_mission_landing || current_item_is_mission_takeoff) {
				_stage = Stage::LandAtGoal;
				PX4_DEBUG("RTL SRP endpoint target active, handing over to landing stage");

				mission_item_s landing_item{};

				if (current_item_is_mission_landing) {
					landing_item = _mission_item;

				} else {
					setLandMissionItem(landing_item);
				}

				publishLandingItems(pos_sp_triplet, current_setpoint_copy, landing_item);
				break;
			}

			// Check for a VTOL transition only once per target to prevent re-issuing the same command.
			const VtolTransitionAction transition_action = (_transition_target_index != _mission.current_seq)
					? vtolTransitionActionForTarget(_mission.current_seq,
							_plan.selection.path.direction_reversed)
					: VtolTransitionAction::None; // Already issued for this target.

			mission_item_s current_route_item = _mission_item;
			const bool current_item_is_endpoint = _plan.selection.goal_type != RtlRoutePlanner::GoalType::SafePoint
							      && isLandingCommand(_mission_item);

			if (!current_item_is_endpoint) {
				normalizeRouteMissionItem(current_route_item);
			}

			mission_item_s next_route_item{};
			mission_item_s *next_route_item_ptr = nullptr;
			int32_t adjacent_index = -1;

			if (loadAdjacentRouteItem(next_route_item, &adjacent_index)) {
				const bool next_item_is_branch_off = _plan.selection.safe_point_found
								     && adjacent_index == _branch_off_index;

				if (next_item_is_branch_off) {
					// Mirror the legacy triplet handling so the controller sees the projected branch-off
					// before it becomes the active current mission item.
					setWaypointMissionItem(next_route_item, _plan.selection.branch_off_projection, true);

				} else if (!(_plan.selection.goal_type != RtlRoutePlanner::GoalType::SafePoint
					     && (next_route_item.nav_cmd == NAV_CMD_TAKEOFF
						 || next_route_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
						 || next_route_item.nav_cmd == NAV_CMD_LAND
						 || next_route_item.nav_cmd == NAV_CMD_VTOL_LAND))) {
					normalizeRouteMissionItem(next_route_item);
				}

				next_route_item_ptr = &next_route_item;
			}

			publishRouteItems(pos_sp_triplet, current_setpoint_copy, current_route_item, next_route_item_ptr);

			if (transition_action != VtolTransitionAction::None
			    && _vehicle_status_sub.get().is_vtol
			    && !_land_detected_sub.get().landed) {
				_transition_target_index = _mission.current_seq;
				_stage = Stage::TransitionDuringRoute;

				mission_item_s transition_item{};
				set_vtol_transition_item(&transition_item,
							 transition_action == VtolTransitionAction::BackTransition
							 ? vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC
							 : vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);

				if (transition_action == VtolTransitionAction::FrontTransition) {
					PX4_INFO("RTL SRP route applying FT");
					transition_item.yaw = _navigator->get_local_position()->heading;

				} else {
					PX4_INFO("RTL SRP route applying BT");
				}

				issue_command(transition_item);
				pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

				if (transition_action == VtolTransitionAction::BackTransition) {
					pos_sp_triplet->previous.valid = false;
				}

				publish_navigator_mission_item();
				_navigator->set_position_setpoint_triplet_updated();
			}

			break;
		}

	case Stage::TransitionDuringRoute: {
			// Transition is in progress — keep the current position setpoint and wait for
			// the transition to complete (detected by setNextMissionItem via is_mission_item_reached).
			break;
		}

	case Stage::BranchOff: {
			mission_item_s branch_off_item{};
			mission_item_s land_item{};
			setWaypointMissionItem(branch_off_item, _plan.selection.branch_off_projection, true);
			setLandMissionItem(land_item);
			publishRouteItems(pos_sp_triplet, current_setpoint_copy, branch_off_item, &land_item);
			break;
		}

	case Stage::LandAtGoal: {
			mission_item_s land_item{};

			if (_plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionLand && isLandingCommand(_mission_item)) {
				land_item = _mission_item;

			} else {
				setLandMissionItem(land_item);
			}

			publishLandingItems(pos_sp_triplet, current_setpoint_copy, land_item);
			break;
		}

	case Stage::Idle:
	default:
		setEndOfMissionItems();
		break;
	}
}

rtl_time_estimate_s RtlMissionSafePointFollow::calc_rtl_time_estimate()
{
	_rtl_time_estimator.update();
	_rtl_time_estimator.setVehicleType(_vehicle_status_sub.get().vehicle_type);
	_rtl_time_estimator.reset();

	if (!_plan.valid() || !_plan.selection.found || _stage == Stage::Idle || _stage == Stage::LandAtGoal) {
		return _rtl_time_estimator.getEstimate();
	}

	const auto *global_pos = _navigator->get_global_position();

	if (global_pos == nullptr || !PX4_ISFINITE(global_pos->lat) || !PX4_ISFINITE(global_pos->lon)) {
		return _rtl_time_estimator.getEstimate();
	}

	matrix::Vector2d hor_pos{global_pos->lat, global_pos->lon};
	float altitude = global_pos->alt;

	// Helper lambda to add a leg from current tracking position to a target lat/lon/alt.
	auto add_leg = [&](double target_lat, double target_lon, float target_alt) {
		matrix::Vector2f direction{};
		get_vector_to_next_waypoint(hor_pos(0), hor_pos(1), target_lat, target_lon, &direction(0), &direction(1));

		const float hor_dist = get_distance_to_next_waypoint(hor_pos(0), hor_pos(1), target_lat, target_lon);
		const float vert_dist = target_alt - altitude;

		_rtl_time_estimator.addDistance(hor_dist, direction, vert_dist);

		hor_pos(0) = target_lat;
		hor_pos(1) = target_lon;
		altitude = target_alt;
	};

	switch (_stage) {
	case Stage::JoinRoute:
	case Stage::TransitionAfterJoin:

		// Add the join leg: vehicle → join projection.
		if (_plan.join_context.valid()) {
			add_leg(_plan.join_context.projection.lat,
				_plan.join_context.projection.lon,
				_plan.join_context.projection.alt);
		}

		[[fallthrough]];

	case Stage::FollowRoute:
	case Stage::TransitionDuringRoute: {
			// Walk the remaining route items from the current target to the branch-off (or endpoint).
			int32_t walk_index = _mission.current_seq;

			// For JoinRoute/TransitionAfterJoin the walk starts at the first route item.
			if (_stage == Stage::JoinRoute || _stage == Stage::TransitionAfterJoin) {
				walk_index = _plan.selection.path.first_item_index;
			}

			// Walk up to a reasonable limit to avoid infinite loops on corrupted data.
			for (int steps = 0; steps < _mission.count && walk_index >= 0 && walk_index < _mission.count; ++steps) {
				mission_item_s mi{};

				if (!loadMissionItemFromCache(walk_index, mi)) {
					break;
				}

				if (item_contains_position(mi)) {
					const float item_alt = get_absolute_altitude_for_item(mi);
					add_leg(mi.lat, mi.lon, item_alt);

					// Stop if we've reached the branch-off index or the endpoint.
					if (_plan.selection.safe_point_found && walk_index == _branch_off_index) {
						break;
					}

					if (!_plan.selection.safe_point_found) {
						if (_plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionLand && isLandingCommand(mi)) {
							break;
						}

						if (_plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionTakeoff && walk_index == 0) {
							break;
						}
					}
				}

				// Advance in the appropriate direction.
				if (_plan.selection.path.direction_reversed) {
					int32_t prev = walk_index;

					if (!findPreviousPositionIndexNoJump(walk_index, prev)) {
						break;
					}

					walk_index = prev;

				} else {
					int32_t next = walk_index + 1;

					if (!findNextPositionIndexNoJump(next, next)) {
						break;
					}

					walk_index = next;
				}
			}

			// Add the branch-off → goal leg if applicable.
			if (_plan.selection.safe_point_found && _plan.selection.goal_position.valid()) {
				// VTOL final descent is always in MC mode.
				if (_vehicle_status_sub.get().is_vtol) {
					_rtl_time_estimator.setVehicleType(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
				}

				add_leg(_plan.selection.goal_position.lat,
					_plan.selection.goal_position.lon,
					_plan.selection.goal_position.alt);
			}

			break;
		}

	case Stage::BranchOff:

		// Already at the branch-off; add only the goal leg.
		if (_plan.selection.goal_position.valid()) {
			// VTOL final descent is always in MC mode.
			if (_vehicle_status_sub.get().is_vtol) {
				_rtl_time_estimator.setVehicleType(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
			}

			add_leg(_plan.selection.goal_position.lat,
				_plan.selection.goal_position.lon,
				_plan.selection.goal_position.alt);
		}

		break;

	default:
		break;
	}

	return _rtl_time_estimator.getEstimate();
}
