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

#include <drivers/drv_hrt.h>

static constexpr int32_t DEFAULT_MISSION_SAFE_POINT_FOLLOW_CACHE_SIZE = 5;

namespace
{

bool isLandingCommand(const mission_item_s &mission_item)
{
	return mission_item.nav_cmd == NAV_CMD_LAND || mission_item.nav_cmd == NAV_CMD_VTOL_LAND;
}

} // namespace

RtlMissionSafePointFollow::RtlMissionSafePointFollow(Navigator *navigator, mission_s mission) :
	RtlBase(navigator, DEFAULT_MISSION_SAFE_POINT_FOLLOW_CACHE_SIZE)
{
	_mission = mission;
}

/**
 * @brief Store the current route plan and cache the branch-off index used by the follow-stage state machine.
 */
void RtlMissionSafePointFollow::setRoutePlan(const RtlRoutePlanner::Plan &plan)
{
	_plan = plan;
	_branch_off_index = _plan.selection.branchOffIndex();
	updateLastFlownLoopSegmentFromPlan();

	if (_plan.valid() && !_plan.join_context.valid()) {
		_plan.join_context.projection = _plan.projection_context.projection.projection;
	}

	if (_plan.valid()) {
		_plan.join_context.vtol_back_transition_required =
			requiresBacktransitionForTarget(_plan.selection.path.first_item_index,
							_plan.selection.path.direction_reversed);
	}
}

/**
 * @brief Cache whether we were already branched off before MissionBase resets the active work item state.
 */
void RtlMissionSafePointFollow::on_inactivation()
{
	_should_go_straight_to_goal = _should_go_straight_to_goal
				      || _plan.selection.direct_to_safe_point
				      || _stage == Stage::BranchOff
				      || _stage == Stage::LandAtGoal;

	MissionBase::on_inactivation();
}

/**
 * @brief Initialize the SRP stage machine from the cached route plan.
 */
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

/**
 * @brief Advance the SRP work state when the current mission item is reached.
 */
bool RtlMissionSafePointFollow::setNextMissionItem()
{
	switch (_stage) {
	case Stage::JoinRoute:
		if (_plan.join_context.vtol_back_transition_required) {
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
		if (currentTargetIsBranchOff()) {
			// Legacy SRP branches off as soon as that branch-off index becomes the active target.
			// It does not wait until the original mission waypoint at that index has been flown.
			_stage = Stage::BranchOff;
			PX4_DEBUG("RTL SRP current target is branch-off, leaving route for goal");
			return true;
		}

		if (_plan.selection.goal_type != RtlRoutePlanner::GoalType::SafePoint
		    && _mission.current_seq == _plan.selection.path.first_item_index) {
			_stage = Stage::LandAtGoal;
			PX4_DEBUG("RTL SRP fallback endpoint reached, landing at goal");
			return true;
		}

		if (_plan.selection.path.direction_reversed) {
			int32_t previous_index = _mission.current_seq;

			if (loadPreviousRoutePositionItemNoJump(previous_index, previous_index)) {
				setMissionIndex(previous_index);

				if (currentTargetIsBranchOff()) {
					_stage = Stage::BranchOff;
					PX4_DEBUG("RTL SRP next reverse target is branch-off, leaving route for goal");
				}

				return true;
			}

			return false;
		}

		updateLastFlownLoopSegmentForNominalAdvance();

		// SRP treats the route as geometry only: DO_JUMP items must not be executed as
		// mission control flow (the planner already modelled them as loop edges).  Walk
		// forward through the mission items and skip any DO_JUMP entries instead of
		// following them, mirroring what loadPreviousRoutePositionItemNoJump does for
		// the reverse direction.
		{
			int32_t next_index = _mission.current_seq + 1;

			if (findNextRoutePositionIndex(next_index, next_index)) {
				setMissionIndex(next_index);

				if (currentTargetIsBranchOff()) {
					_stage = Stage::BranchOff;
					PX4_DEBUG("RTL SRP next nominal target is branch-off, leaving route for goal");
				}

				return true;
			}
		}

		return false;

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

/**
 * @brief Build a waypoint mission item with RTL-friendly acceptance settings.
 */
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

/**
 * @brief Build the final landing item for either the safe point or the mission endpoint fallback.
 */
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

/**
 * @brief Normalize a mission item so it behaves like a pure route waypoint.
 *
 * During route following, the mission is treated as geometry only.  Position-bearing items such as
 * NAV_CMD_LOITER_UNLIMITED, NAV_CMD_LOITER_TIME_LIMIT, and NAV_CMD_LOITER_TO_ALT are converted
 * to plain waypoints with autocontinue enabled and zero hold time so the vehicle keeps moving.
 * NAV_CMD_DELAY items are non-position and are naturally skipped by findNextRoutePositionIndex(),
 * but if one were encountered it would also be clamped here.
 */
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

/**
 * @brief Load the previous position item without executing DO_JUMP entries while walking the mission backwards.
 */
bool RtlMissionSafePointFollow::loadPreviousRoutePositionItemNoJump(int32_t start_index, int32_t &previous_index)
{
	for (int32_t index = start_index - 1; index >= 0; --index) {
		mission_item_s mission_item{};

		if (!_dataman_cache.loadWait(static_cast<dm_item_t>(_mission.mission_dataman_id), index,
					     reinterpret_cast<uint8_t *>(&mission_item), sizeof(mission_item),
					     MAX_DATAMAN_LOAD_WAIT)) {
			continue;
		}

		if (mission_item.nav_cmd == NAV_CMD_DO_JUMP) {
			continue;
		}

		if (MissionBlock::item_contains_position(mission_item)) {
			previous_index = index;
			return true;
		}
	}

	return false;
}

/**
 * @brief Load a mission item from the active mission dataman stream.
 */
bool RtlMissionSafePointFollow::loadMissionItemAtIndex(int32_t index, mission_item_s &mission_item)
{
	return index >= 0
	       && index < _mission.count
	       && _dataman_cache.loadWait(static_cast<dm_item_t>(_mission.mission_dataman_id), index,
					  reinterpret_cast<uint8_t *>(&mission_item), sizeof(mission_item),
					  MAX_DATAMAN_LOAD_WAIT);
}

/**
 * @brief Find the nearest position item attached to or preceding the supplied mission index.
 */
bool RtlMissionSafePointFollow::findAttachedRoutePositionIndex(int32_t start_index, int32_t &attached_index)
{
	for (int32_t index = start_index; index >= 0; --index) {
		mission_item_s mission_item{};

		if (!loadMissionItemAtIndex(index, mission_item)) {
			return false;
		}

		if (MissionBlock::item_contains_position(mission_item)) {
			attached_index = index;
			return true;
		}
	}

	return false;
}

/**
 * @brief Find the next position item at or after the supplied mission index.
 */
bool RtlMissionSafePointFollow::findNextRoutePositionIndex(int32_t start_index, int32_t &next_index)
{
	for (int32_t index = start_index; index < _mission.count; ++index) {
		mission_item_s mission_item{};

		if (!loadMissionItemAtIndex(index, mission_item)) {
			return false;
		}

		if (MissionBlock::item_contains_position(mission_item)) {
			next_index = index;
			return true;
		}
	}

	return false;
}

/**
 * @brief Return whether the current mission target should be replaced by the virtual branch-off waypoint.
 */
bool RtlMissionSafePointFollow::currentTargetIsBranchOff() const
{
	return _plan.selection.safe_point_found
	       && _branch_off_index >= 0
	       && _mission.current_seq == _branch_off_index;
}

/**
 * @brief Return whether the join projection already sits close enough to the stored branch-off projection.
 */
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

/**
 * @brief Seed the loop-anchor memory from the current route plan projection.
 */
void RtlMissionSafePointFollow::updateLastFlownLoopSegmentFromPlan()
{
	_last_flown_loop_segment = _plan.projection_context.projection.segment;
}

/**
 * @brief Mirror the legacy `_last_loop_jump_flown` bookkeeping before executing a nominal DO_JUMP.
 */
void RtlMissionSafePointFollow::updateLastFlownLoopSegmentForNominalAdvance()
{
	_last_flown_loop_segment = {};

	for (int32_t index = _mission.current_seq + 1; index < _mission.count; ++index) {
		mission_item_s mission_item{};

		if (!loadMissionItemAtIndex(index, mission_item)) {
			return;
		}

		if (mission_item.nav_cmd == NAV_CMD_DO_JUMP
		    && mission_item.do_jump_current_count < mission_item.do_jump_repeat_count) {
			int32_t loop_start_index = -1;
			int32_t loop_end_index = -1;

			if (!findAttachedRoutePositionIndex(index, loop_start_index)
			    || !findNextRoutePositionIndex(mission_item.do_jump_mission_index, loop_end_index)) {
				return;
			}

			mission_item_s loop_start_item{};
			mission_item_s loop_end_item{};

			if (!loadMissionItemAtIndex(loop_start_index, loop_start_item)
			    || !loadMissionItemAtIndex(loop_end_index, loop_end_item)) {
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

/**
 * @brief Load the adjacent route item for the next trajectory segment.
 */
bool RtlMissionSafePointFollow::loadAdjacentRouteItem(mission_item_s &mission_item, int32_t *adjacent_index)
{
	if (_plan.selection.path.direction_reversed) {
		int32_t adjacent_route_index = _mission.current_seq;

		if (!loadPreviousRoutePositionItemNoJump(adjacent_route_index, adjacent_route_index)) {
			return false;
		}

		if (adjacent_index != nullptr) {
			*adjacent_index = adjacent_route_index;
		}

		return _dataman_cache.loadWait(static_cast<dm_item_t>(_mission.mission_dataman_id), adjacent_route_index,
					       reinterpret_cast<uint8_t *>(&mission_item), sizeof(mission_item),
					       MAX_DATAMAN_LOAD_WAIT);

	} else {
		// Walk forward without following DO_JUMP control flow, matching the planner's
		// geometry-only treatment of loop edges.
		int32_t adjacent_route_index = _mission.current_seq + 1;

		if (!findNextRoutePositionIndex(adjacent_route_index, adjacent_route_index)) {
			return false;
		}

		if (adjacent_index != nullptr) {
			*adjacent_index = adjacent_route_index;
		}

		return _dataman_cache.loadWait(static_cast<dm_item_t>(_mission.mission_dataman_id), adjacent_route_index,
					       reinterpret_cast<uint8_t *>(&mission_item), sizeof(mission_item),
					       MAX_DATAMAN_LOAD_WAIT);
	}
}

/**
 * @brief Check whether entering the segment at the given target requires a VTOL back-transition.
 *
 * Only inspects the entry state for the single target_index, not the entire path between two indices.
 */
bool RtlMissionSafePointFollow::requiresBacktransitionForTarget(int32_t target_index, bool reversed)
{
	if (!_navigator->get_vstatus()->is_vtol || target_index < 0) {
		return false;
	}

	const auto action = transitionActionForTargetIndex(target_index, reversed);
	return action == RtlRoutePlanner::TransitionAction::BackTransition;
}

/**
 * @brief Determine whether the current target requires a front or back transition before continuing the route.
 *
 * This mirrors the planner's transitionActionForTargetIndex() logic but reads mission items directly
 * through the executor's dataman cache, consistent with how other mission-based RTL modes handle
 * VTOL transitions without an intermediate Provider abstraction.
 */
RtlRoutePlanner::TransitionAction RtlMissionSafePointFollow::transitionActionForTargetIndex(int32_t target_index,
		bool direction_reversed)
{
	const auto &vehicle_status = _vehicle_status_sub.get();

	if (!vehicle_status.is_vtol || target_index < 0 || target_index >= _mission.count) {
		return RtlRoutePlanner::TransitionAction::None;
	}

	// Step 1: Find the segment-end anchor for the target index.
	// When flying forward, the anchor is the first position item at or after the target;
	// when reversed, the anchor starts one index past the target.
	int32_t anchor_search_start = direction_reversed ? target_index + 1 : target_index;

	if (anchor_search_start >= _mission.count) {
		return RtlRoutePlanner::TransitionAction::None;
	}

	int32_t segment_anchor_index = -1;

	if (!findNextRoutePositionIndex(anchor_search_start, segment_anchor_index)) {
		return RtlRoutePlanner::TransitionAction::None;
	}

	// Step 2: Walk backward from the anchor to find the most recent DO_VTOL_TRANSITION.
	// This tells us the expected VTOL state when entering this segment.
	uint8_t target_segment_state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;

	for (int32_t index = segment_anchor_index; index >= 0; --index) {
		mission_item_s mission_item{};

		if (!loadMissionItemAtIndex(index, mission_item)) {
			break;
		}

		if (mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION) {
			const int transition_mode = static_cast<int>(roundf(mission_item.params[0]));

			if (transition_mode == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC) {
				target_segment_state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;

			} else if (transition_mode == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW) {
				target_segment_state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;
			}

			break;
		}
	}

	// Step 3: Compare the expected VTOL state with the current vehicle state.
	const bool currently_fw = vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				  || vehicle_status.in_transition_to_fw;

	if (target_segment_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC && currently_fw) {
		return RtlRoutePlanner::TransitionAction::BackTransition;
	}

	if (target_segment_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW && !currently_fw) {
		return RtlRoutePlanner::TransitionAction::FrontTransition;
	}

	return RtlRoutePlanner::TransitionAction::None;
}

/**
 * @brief Publish a synthetic work item and keep the current/next route items aligned with the mission setpoint triplet.
 */
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

/**
 * @brief Publish the landing stage through MissionBase::handleLanding() so SRP keeps VTOL and precision-land semantics.
 */
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

/**
 * @brief Emit the SRP setpoints for the current stage, mirroring the legacy join/branch-off sequencing.
 */
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
					       _plan.join_context.vtol_back_transition_required);

			if (_plan.join_context.skip_altitude_requirement) {
				join_item.altitude = _navigator->get_global_position()->alt;
			}

			publishRouteItems(pos_sp_triplet, current_setpoint_copy, join_item, &next_route_item);
			break;
		}

	case Stage::TransitionAfterJoin: {
			if (_plan.join_context.vtol_back_transition_required
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
			if (currentTargetIsBranchOff()) {
				_stage = Stage::BranchOff;
				mission_item_s branch_off_item{};
				mission_item_s land_item{};
				setWaypointMissionItem(branch_off_item, _plan.selection.branch_off_projection, true);
				setLandMissionItem(land_item);
				publishRouteItems(pos_sp_triplet, current_setpoint_copy, branch_off_item, &land_item);
				break;
			}

			const bool selected_endpoint_active = _plan.selection.goal_type != RtlRoutePlanner::GoalType::SafePoint
							      && _mission.current_seq == _plan.selection.path.first_item_index;
			const bool current_item_is_mission_landing = _plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionLand
					&& isLandingCommand(_mission_item);

			if (selected_endpoint_active
			    && (current_item_is_mission_landing
				|| _plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionTakeoff)) {
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

			const auto transition_action = transitionActionForTargetIndex(_mission.current_seq,
						       _plan.selection.path.direction_reversed);

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

			if (transition_action != RtlRoutePlanner::TransitionAction::None
			    && _vehicle_status_sub.get().is_vtol
			    && !_land_detected_sub.get().landed) {
				mission_item_s transition_item{};
				set_vtol_transition_item(&transition_item,
							 transition_action == RtlRoutePlanner::TransitionAction::BackTransition
							 ? vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC
							 : vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);

				if (transition_action == RtlRoutePlanner::TransitionAction::FrontTransition) {
					PX4_INFO("RTL SRP route applying FT");
					transition_item.yaw = _navigator->get_local_position()->heading;

				} else {
					PX4_INFO("RTL SRP route applying BT");
				}

				issue_command(transition_item);
				pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

				if (transition_action == RtlRoutePlanner::TransitionAction::BackTransition) {
					pos_sp_triplet->previous.valid = false;
				}

				publish_navigator_mission_item();
				_navigator->set_position_setpoint_triplet_updated();
			}

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

/**
 * @brief Estimate the remaining flight time to the SRP goal.
 *
 * Route Safe Point Return follows a known geometric path, so we can produce a useful estimate by
 * summing the remaining route distance and the branch-off-to-goal distance, then dividing by the
 * active cruising speed.  This is an approximation that ignores altitude changes, wind, and VTOL
 * transitions, but it is far more useful than returning valid=false.
 */
rtl_time_estimate_s RtlMissionSafePointFollow::calc_rtl_time_estimate()
{
	rtl_time_estimate_s time_estimate{};
	time_estimate.valid = false;
	time_estimate.timestamp = hrt_absolute_time();

	if (!_plan.valid() || !_plan.selection.found) {
		return time_estimate;
	}

	const float cruising_speed = _navigator->get_cruising_speed();

	if (!PX4_ISFINITE(cruising_speed) || cruising_speed < FLT_EPSILON) {
		return time_estimate;
	}

	float remaining_dist = 0.f;

	switch (_stage) {
	case Stage::Idle:
		return time_estimate;

	case Stage::JoinRoute:

	// Fall through: include join + full route distance.
	case Stage::TransitionAfterJoin:

	// Fall through: include remaining route distance.
	case Stage::FollowRoute: {
			// During FollowRoute, approximate the remaining distance as the straight-line
			// distance from the vehicle to the current waypoint target plus the plan's
			// route distance reduced by the distance already covered.  For JoinRoute and
			// TransitionAfterJoin stages we fall through and use the full plan distance
			// since we haven't started following the route yet.
			if (_stage == Stage::FollowRoute) {
				const auto *global_pos = _navigator->get_global_position();

				if (global_pos != nullptr && PX4_ISFINITE(global_pos->lat) && PX4_ISFINITE(global_pos->lon)
				    && _plan.projection_context.projection.projection.valid()) {
					// Distance from vehicle's original projection to now, approximated as
					// straight-line from current position to the plan's join projection.
					const float dist_already_covered = get_distance_to_next_waypoint(
							_plan.projection_context.projection.projection.lat,
							_plan.projection_context.projection.projection.lon,
							global_pos->lat, global_pos->lon);

					// Use max(plan_dist - covered, 0) to avoid going negative if the vehicle
					// has drifted past the branch-off.
					remaining_dist += fmaxf(_plan.selection.path.dist - dist_already_covered, 0.f);

				} else {
					remaining_dist += _plan.selection.path.dist;
				}

			} else {
				remaining_dist += _plan.selection.path.dist;
			}

			// Add the straight-line distance from the branch-off projection to the goal.
			if (_plan.selection.branch_off_projection.valid() && _plan.selection.goal_position.valid()) {
				remaining_dist += get_distance_to_next_waypoint(
							  _plan.selection.branch_off_projection.lat,
							  _plan.selection.branch_off_projection.lon,
							  _plan.selection.goal_position.lat,
							  _plan.selection.goal_position.lon);
			}

			break;
		}

	case Stage::BranchOff:

		// Already past the branch-off waypoint; only the goal leg remains.
		if (_plan.selection.branch_off_projection.valid() && _plan.selection.goal_position.valid()) {
			remaining_dist += get_distance_to_next_waypoint(
						  _plan.selection.branch_off_projection.lat,
						  _plan.selection.branch_off_projection.lon,
						  _plan.selection.goal_position.lat,
						  _plan.selection.goal_position.lon);
		}

		break;

	case Stage::LandAtGoal:
		// Landing is in progress; we cannot estimate the remaining descent time from here.
		return time_estimate;
	}

	time_estimate.time_estimate = remaining_dist / cruising_speed;
	time_estimate.safe_time_estimate = time_estimate.time_estimate;
	time_estimate.valid = PX4_ISFINITE(time_estimate.time_estimate);
	return time_estimate;
}
