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
#include <mathlib/mathlib.h>

RtlMissionSafePointFollow::RtlMissionSafePointFollow(Navigator *navigator, mission_s mission) :
	// Route-safe-point RTL already reads mission geometry from MissionRouteCache, so
	// MissionBase's sliding-window DatamanCache would only duplicate mission RAM.
	RtlBase(navigator, 0)
{
	_mission = mission;
}

RtlMissionSafePointFollow::~RtlMissionSafePointFollow()
{
	perf_free(_calc_rtl_time_estimate_perf);
}

bool RtlMissionSafePointFollow::loadMissionItemFromCache(int32_t index, mission_item_s &mission_item)
{
	MissionRouteCache *mission_route_cache = _navigator->get_mission_route_cache();

	return mission_route_cache != nullptr
	       && mission_route_cache->isReady(_mission)
	       && index >= 0
	       && index < _mission.count
	       && mission_route_cache->loadMissionItem(index, mission_item);
}

void RtlMissionSafePointFollow::configureRouteSafePoint(const RouteSafePointConfig &config)
{
	_plan = config.plan;
	_goal_land_approach = config.goal_land_approach;
	_rtl_alt = config.rtl_alt;
	updateLastFlownLoopSegmentFromPlan();
}

bool RtlMissionSafePointFollow::missionIndexInBounds(int32_t index) const
{
	return index >= 0 && index < _mission.count;
}

void RtlMissionSafePointFollow::resetExecutorProgress()
{
	_state.resetProgress();
	resetJoinRouteState();
}

bool RtlMissionSafePointFollow::useGoalLandApproach() const
{
	return _plan.selection.safe_point_found && _goal_land_approach.isValid();
}

RtlMissionSafePointFollow::Stage RtlMissionSafePointFollow::finalGoalStage() const
{
	return useGoalLandApproach() ? Stage::ApproachAtGoal : Stage::LandAtGoal;
}

bool RtlMissionSafePointFollow::goalIsMissionLanding() const
{
	return !_plan.selection.safe_point_found
	       && _plan.selection.goal_type == MissionRoutePlanner::GoalType::MissionLand;
}

bool RtlMissionSafePointFollow::goalIsMissionTakeoff() const
{
	return !_plan.selection.safe_point_found
	       && _plan.selection.goal_type == MissionRoutePlanner::GoalType::MissionTakeoff;
}

bool RtlMissionSafePointFollow::missionItemMatchesSelectedEndpoint(const mission_item_s &mission_item) const
{
	if (goalIsMissionLanding()) {
		return isLandingCommand(mission_item);
	}

	if (goalIsMissionTakeoff()) {
		return isTakeoffCommand(mission_item);
	}

	return false;
}

void RtlMissionSafePointFollow::on_inactivation()
{
	resetExecutorProgress();

	MissionBase::on_inactivation();
}

void RtlMissionSafePointFollow::on_activation()
{
	_vehicle_status_sub.update();
	_land_detected_sub.update();

	resetExecutorProgress();
	updateLastFlownLoopSegmentFromPlan();

	if (_plan.valid()) {
		const int32_t first_item_index = _plan.selection.path.first_item_index;

		if (missionIndexInBounds(first_item_index)) {
			setMissionIndex(first_item_index);
			_is_current_planned_mission_item_valid = isMissionValid();
			_state.stage = _plan.selection.skip_route_to_safe_point ? finalGoalStage() : Stage::FollowRoute;

		} else {
			PX4_ERR("RTL plan start index out of bounds: %ld (count=%ld)",
				static_cast<long>(first_item_index), static_cast<long>(_mission.count));
			_is_current_planned_mission_item_valid = false;
			_state.stage = Stage::Idle;
		}

		PX4_INFO("RTL to %s target=%d rev=%u skip_route=%u vtol=%u stage=%u",
			 MissionRoutePlanner::goalTypeString(_plan.selection.goal_type),
			 static_cast<int>(_plan.selection.path.first_item_index),
			 static_cast<unsigned>(_plan.selection.path.direction_reversed),
			 static_cast<unsigned>(_plan.selection.skip_route_to_safe_point),
			 static_cast<unsigned>(_plan.join_context.transition_action),
			 static_cast<unsigned>(_state.stage));

	} else {
		_is_current_planned_mission_item_valid = false;
	}

	if (_land_detected_sub.get().landed) {
		_state.stage = Stage::Idle;
		_is_current_planned_mission_item_valid = false;
	}

	// Reset the triplet when retargeting the mission so the controller does not keep following a stale line.
	_navigator->get_position_setpoint_triplet()->previous.valid = false;
	_navigator->get_position_setpoint_triplet()->current.valid = false;
	_navigator->get_position_setpoint_triplet()->next.valid = false;

	if (_is_current_planned_mission_item_valid && !_plan.selection.skip_route_to_safe_point) {
		// Route Safe Point Return reuses MissionBase's shared JOIN_ROUTE ->
		// TRANSITION_AFTER_JOIN executor path through RtlBase.
		setupJoinRoute(_plan.join_context, _plan.selection.path);

		PX4_INFO("RTL safe point %d selected: branch off after idx %u",
			 static_cast<int>(_plan.selection.safe_point_index),
			 static_cast<unsigned>(_plan.selection.branchOffIndex()));
	}

	MissionBase::on_activation();
}

bool RtlMissionSafePointFollow::advanceRouteTarget()
{
	bool advanced = false;
	const auto continueToGoal = [this](const char *reason) {
		_state.stage = finalGoalStage();
		clearRouteTransitionState();
		PX4_INFO("%s, straight to goal", reason);
		return true;
	};

	if (_plan.selection.path.direction_reversed) {
		const int32_t search_start_index = _mission.current_seq;
		int32_t previous_index = -1;

		if (findPreviousPositionIndex(search_start_index, previous_index, PositionTraversalType::IgnoreDoJump)) {
			setMissionIndex(previous_index);
			advanced = true;

		} else {
			return continueToGoal("RTL reverse route complete");
		}

	} else {
		updateLastFlownLoopSegmentForNominalAdvance();

		// Treats the route as geometry only: DO_JUMP items must not be executed as
		// mission control flow (the planner already modelled them as loop segments).
		// Walk forward through the mission items and skip any DO_JUMP entries.
		const int32_t search_start_index = _mission.current_seq + 1;
		int32_t next_index = -1;

		if (findNextPositionIndex(search_start_index, next_index, PositionTraversalType::IgnoreDoJump)) {
			setMissionIndex(next_index);
			advanced = true;

		} else {
			return continueToGoal("RTL route complete");
		}
	}

	return advanced;
}

bool RtlMissionSafePointFollow::setNextMissionItem()
{
	switch (_state.stage) {
	case Stage::FollowRoute: {
			const VtolTransitionAction reverse_transition_action = _plan.selection.path.direction_reversed
					? vtolTransitionActionAfterReachingReverseTarget(_mission.current_seq)
					: VtolTransitionAction::None;
			const bool wait_for_reverse_transition = reverse_transition_action != VtolTransitionAction::None
					&& _vehicle_status_sub.get().is_vtol && !_land_detected_sub.get().landed;

			if (wait_for_reverse_transition) {
				armRouteTransition(reverse_transition_action, true);
				return true;
			}

			return advanceRouteTarget();
		}

	case Stage::TransitionDuringRoute: {
			const bool advance_after_transition = _state.advance_route_after_transition;
			const bool branch_off_after_transition = currentTargetIsBranchOff();
			clearRouteTransitionState();

			if (advance_after_transition) {
				_state.stage = Stage::FollowRoute;
				PX4_INFO("RTL route transition complete, advancing reverse route");
				return advanceRouteTarget();
			}

			_state.stage = branch_off_after_transition ? Stage::BranchOff : Stage::FollowRoute;
			PX4_INFO(branch_off_after_transition ? "RTL route transition complete, branching off"
				 : "RTL route transition complete");
			return true;
		}

	case Stage::BranchOff:
		_state.stage = finalGoalStage();
		PX4_INFO("RTL branch-off reached, straight to goal");
		return true;

	case Stage::ApproachAtGoal:
		_state.stage = Stage::LandAtGoal;
		PX4_INFO("RTL goal approach reached, landing");
		return true;

	case Stage::LandAtGoal:
	case Stage::Idle:
	default:
		return false;
	}
}

void RtlMissionSafePointFollow::setWaypointMissionItem(mission_item_s &mission_item,
		const MissionRoutePlanner::Position &position, bool autocontinue, bool vtol_back_transition_required) const
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

	} else if (vehicleInFwLikeState(_vehicle_status_sub.get())) {
		// Match the legacy join behavior: use a wider acceptance radius when the vehicle is already in FW-like flow.
		mission_item.acceptance_radius = kJoinRouteFlyByAcceptanceRadiusScale * _navigator->get_acceptance_radius();
	}

	mission_item.yaw = NAN;
	mission_item.time_inside = 0.f;
	mission_item.autocontinue = autocontinue;
	mission_item.origin = ORIGIN_ONBOARD;
}

void RtlMissionSafePointFollow::setLandMissionItem(mission_item_s &mission_item) const
{
	mission_item = {};
	mission_item.nav_cmd = (_vehicle_status_sub.get().is_vtol || _navigator->force_vtol()) ? NAV_CMD_VTOL_LAND : NAV_CMD_LAND;
	mission_item.lat = _plan.selection.goal_position.lat;
	mission_item.lon = _plan.selection.goal_position.lon;

	mission_item.altitude = _plan.selection.goal_position.alt;

	mission_item.altitude_is_relative = false;
	mission_item.yaw = NAN;
	mission_item.time_inside = 0.f;
	mission_item.land_precision = _param_rtl_pld_md.get();
	mission_item.autocontinue = false;
	mission_item.origin = ORIGIN_ONBOARD;
}

void RtlMissionSafePointFollow::setGoalApproachMissionItem(mission_item_s &mission_item) const
{
	// TODO: this is the same behavior as rtl_direct set_rtl_item but is it really what we want?
	const float loiter_altitude = PX4_ISFINITE(_rtl_alt) ? math::min(_goal_land_approach.height_m, _rtl_alt)
				      : _goal_land_approach.height_m;
	const float loiter_radius = (PX4_ISFINITE(_goal_land_approach.loiter_radius_m)
				     && fabsf(_goal_land_approach.loiter_radius_m) > FLT_EPSILON)
				    ? _goal_land_approach.loiter_radius_m
				    : _navigator->get_default_loiter_rad();

	const PositionYawSetpoint goal_approach{
		.lat = _goal_land_approach.lat,
		.lon = _goal_land_approach.lon,
		.alt = loiter_altitude,
		.yaw = NAN
	};

	setLoiterToAltMissionItem(mission_item, goal_approach, loiter_radius);
}

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

	switch (mission_item.nav_cmd) {
	case NAV_CMD_TAKEOFF:
	case NAV_CMD_VTOL_TAKEOFF:
	case NAV_CMD_LAND:
	case NAV_CMD_VTOL_LAND:
		// Keep endpoint commands intact: takeoff and landing retain their dedicated execution semantics.
		break;

	case NAV_CMD_WAYPOINT:
	case NAV_CMD_LOITER_UNLIMITED:
	case NAV_CMD_LOITER_TIME_LIMIT:
	case NAV_CMD_LOITER_TO_ALT:
		mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		mission_item.autocontinue = true;
		mission_item.time_inside = 0.f;
		break;

	default:
		break;
	}
}

bool RtlMissionSafePointFollow::currentTargetIsBranchOff() const
{
	return _plan.selection.safe_point_found
	       && _mission.current_seq == _plan.selection.branchOffIndex();
}

void RtlMissionSafePointFollow::updateLastFlownLoopSegmentFromPlan()
{
	// Keep only an actual DO_JUMP anchor. Non-loop segments are reconstructed from current_seq.
	_last_flown_loop_segment = _plan.projection_context.seg_candidate.segment.validLoop()
				   ? _plan.projection_context.seg_candidate.segment
				   : MissionRoutePlanner::Segment{};
}

void RtlMissionSafePointFollow::updateLastFlownLoopSegmentForNominalAdvance()
{
	MissionBase::updateLastFlownLoopSegmentForNominalAdvance(_last_flown_loop_segment);
}

bool RtlMissionSafePointFollow::loadAdjacentRouteItem(mission_item_s &mission_item, int32_t &adjacent_index)
{
	int32_t adjacent_route_index = -1;

	if (_plan.selection.path.direction_reversed) {
		// findPreviousPositionIndex() scans from start_index - 1, so reverse traversal passes
		// current_seq to get the immediately preceding route target.
		const int32_t search_start_index = _mission.current_seq;

		if (!findPreviousPositionIndex(search_start_index, adjacent_route_index,
					       PositionTraversalType::IgnoreDoJump)) {
			return false;
		}

		adjacent_index = adjacent_route_index;

	} else {
		// findNextPositionIndex() includes start_index itself, so nominal traversal starts at
		// current_seq + 1 to avoid returning the current route target again.
		// Walk forward without following DO_JUMP control flow, matching the planner's
		// geometry-only treatment of loop edges.
		const int32_t search_start_index = _mission.current_seq + 1;

		if (!findNextPositionIndex(search_start_index, adjacent_route_index,
					   PositionTraversalType::IgnoreDoJump)) {
			return false;
		}

		adjacent_index = adjacent_route_index;
	}

	return loadMissionItemFromCache(adjacent_route_index, mission_item);
}

void RtlMissionSafePointFollow::armRouteTransition(VtolTransitionAction action, bool advance_route_after_transition)
{
	_state.stage = Stage::TransitionDuringRoute;
	_state.transition_target_index = _mission.current_seq;
	_state.transition_action = action;
	_state.transition_command_sent = false;
	_state.advance_route_after_transition = advance_route_after_transition;
}

void RtlMissionSafePointFollow::clearRouteTransitionState()
{
	_state.clearRouteTransition();
}

void RtlMissionSafePointFollow::handleRouteTransitionStage(position_setpoint_triplet_s *pos_sp_triplet,
		const position_setpoint_s &current_setpoint_copy)
{
	if (_state.transition_command_sent) {
		return;
	}

	if (_state.transition_action == VtolTransitionAction::None || !missionIndexInBounds(_state.transition_target_index)) {
		PX4_ERR("RTL route transition stage is missing a valid target/action");
		clearRouteTransitionState();
		_state.stage = currentTargetIsBranchOff() ? Stage::BranchOff : Stage::FollowRoute;
		return;
	}

	const bool branch_off_target_active = currentTargetIsBranchOff() && !_state.advance_route_after_transition;
	mission_item_s current_route_item{};
	mission_item_s next_route_item{};
	mission_item_s *next_route_item_ptr = nullptr;
	int32_t adjacent_index = -1;

	if (branch_off_target_active) {
		setWaypointMissionItem(current_route_item, _plan.selection.branch_off_projection, true);

		if (useGoalLandApproach()) {
			setGoalApproachMissionItem(next_route_item);

		} else {
			setLandMissionItem(next_route_item);
		}

		next_route_item_ptr = &next_route_item;

	} else {
		current_route_item = _mission_item;
		normalizeRouteMissionItem(current_route_item);

		if (loadAdjacentRouteItem(next_route_item, adjacent_index)) {
			const bool next_item_is_branch_off = _plan.selection.safe_point_found
							     && adjacent_index == _plan.selection.branchOffIndex();
			const bool next_item_is_endpoint_command = missionItemMatchesSelectedEndpoint(next_route_item);

			if (next_item_is_branch_off) {
				setWaypointMissionItem(next_route_item, _plan.selection.branch_off_projection, true);

			} else if (!next_item_is_endpoint_command) {
				normalizeRouteMissionItem(next_route_item);
			}

			next_route_item_ptr = &next_route_item;
		}
	}

	if (_state.advance_route_after_transition
	    && _plan.selection.path.direction_reversed
	    && next_route_item_ptr != nullptr) {
		// Reverse waypoint-attached semantics trigger the transition only after the current
		// waypoint is reached. During that transition the vehicle must already track the next
		// reverse target, not fly back toward the waypoint it just completed.
		current_route_item = *next_route_item_ptr;
		next_route_item_ptr = nullptr;
	}

	publishRouteItems(pos_sp_triplet, current_setpoint_copy, current_route_item, next_route_item_ptr, false);

	mission_item_s transition_item{};
	const bool front_transition = _state.transition_action == VtolTransitionAction::FrontTransition;
	set_vtol_transition_item(&transition_item,
				 front_transition ? vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW
				 : vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);

	if (front_transition) {
		const auto *global_position = _navigator->get_global_position();

		if (branch_off_target_active
		    && global_position != nullptr
		    && PX4_ISFINITE(global_position->lat)
		    && PX4_ISFINITE(global_position->lon)) {
			transition_item.yaw = get_bearing_to_next_waypoint(global_position->lat, global_position->lon,
					      _plan.selection.branch_off_projection.lat,
					      _plan.selection.branch_off_projection.lon);

		} else {
			int32_t alignment_index = _mission.current_seq;

			if (_state.advance_route_after_transition
			    && _plan.selection.path.direction_reversed
			    && missionIndexInBounds(adjacent_index)) {
				alignment_index = adjacent_index;
			}

			transition_item.yaw = computeFrontTransitionAlignmentYaw(alignment_index);
		}

		PX4_INFO("RTL route front transition");

	} else {
		PX4_INFO("RTL route back transition");
	}

	_mission_item = transition_item;
	issue_command(_mission_item);
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

	if (_state.transition_action == VtolTransitionAction::BackTransition) {
		pos_sp_triplet->previous.valid = false;
	}

	reset_mission_item_reached();
	publish_navigator_mission_item();
	_navigator->set_position_setpoint_triplet_updated();
	_state.transition_command_sent = true;
}

void RtlMissionSafePointFollow::publishRouteItems(position_setpoint_triplet_s *pos_sp_triplet,
		const position_setpoint_s &current_setpoint_copy, const mission_item_s &current_mission_item,
		const mission_item_s *next_mission_item, bool sync_active_mission_item)
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

	if (sync_active_mission_item) {
		// Synchronize _mission_item with the published waypoint so that
		// is_mission_item_reached_or_completed() checks the correct position.
		// Without this, the reach check uses the raw dataman item (e.g. WP2) while
		// the drone is actually flying to a virtual waypoint (e.g. branch-off projection
		// on segment 1-2), causing the stage machine to stall or misbehave.
		_mission_item = current_mission_item;
	}

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

	// MissionBase::handleLanding() mutates _mission_item in-place when it needs to insert
	// synthetic move-to-land or VTOL-transition helpers before the final descent.
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
					&& _state.stage == Stage::LandAtGoal
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

void RtlMissionSafePointFollow::handleFollowRouteStage(position_setpoint_triplet_s *pos_sp_triplet,
		const position_setpoint_s &current_setpoint_copy)
{
	const bool current_item_is_endpoint = missionItemMatchesSelectedEndpoint(_mission_item);

	if (current_item_is_endpoint) {
		_state.stage = Stage::LandAtGoal;
		PX4_DEBUG("RTL endpoint target active, handing over to landing stage");

		mission_item_s landing_item{};

		if (goalIsMissionLanding()) {
			landing_item = _mission_item;

		} else {
			setLandMissionItem(landing_item);
		}

		publishLandingItems(pos_sp_triplet, current_setpoint_copy, landing_item);
		return;
	}

	const bool branch_off_target_active = currentTargetIsBranchOff();
	const bool segment_entry_transition_context = branch_off_target_active || !_plan.selection.path.direction_reversed;
	const VtolTransitionAction transition_action = segment_entry_transition_context
			? vtolTransitionActionForTarget(_mission.current_seq, _plan.selection.path.direction_reversed)
			: VtolTransitionAction::None;
	const bool wait_for_route_transition = transition_action != VtolTransitionAction::None
					       && _vehicle_status_sub.get().is_vtol
					       && !_land_detected_sub.get().landed;

	if (wait_for_route_transition) {
		armRouteTransition(transition_action, false);
		handleRouteTransitionStage(pos_sp_triplet, current_setpoint_copy);
		return;
	}

	if (branch_off_target_active) {
		_state.stage = Stage::BranchOff;
		PX4_INFO("RTL leaving route at branch-off");
		return;
	}

	mission_item_s current_route_item = _mission_item;
	normalizeRouteMissionItem(current_route_item);

	mission_item_s next_route_item{};
	mission_item_s *next_route_item_ptr = nullptr;
	int32_t adjacent_index = -1;

	if (loadAdjacentRouteItem(next_route_item, adjacent_index)) {
		const bool next_item_is_branch_off = _plan.selection.safe_point_found
						     && adjacent_index == _plan.selection.branchOffIndex();
		const bool next_item_is_endpoint_command = missionItemMatchesSelectedEndpoint(next_route_item);

		if (next_item_is_branch_off) {
			// Mirror the legacy triplet handling so the controller sees the projected branch-off
			// before it becomes the active current mission item.
			setWaypointMissionItem(next_route_item, _plan.selection.branch_off_projection, true);

		} else if (!next_item_is_endpoint_command) {
			normalizeRouteMissionItem(next_route_item);
		}

		next_route_item_ptr = &next_route_item;
	}

	publishRouteItems(pos_sp_triplet, current_setpoint_copy, current_route_item, next_route_item_ptr);
}

void RtlMissionSafePointFollow::setActiveMissionItems()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	const position_setpoint_s current_setpoint_copy = pos_sp_triplet->current;

	if (handleJoinRouteWorkItems(pos_sp_triplet, current_setpoint_copy)) {
		return;
	}

	switch (_state.stage) {
	case Stage::FollowRoute:
		handleFollowRouteStage(pos_sp_triplet, current_setpoint_copy);
		break;

	case Stage::TransitionDuringRoute:
		handleRouteTransitionStage(pos_sp_triplet, current_setpoint_copy);
		break;

	case Stage::BranchOff: {
			mission_item_s branch_off_item{};
			mission_item_s next_goal_item{};
			setWaypointMissionItem(branch_off_item, _plan.selection.branch_off_projection, true);

			if (useGoalLandApproach()) {
				setGoalApproachMissionItem(next_goal_item);

			} else {
				setLandMissionItem(next_goal_item);
			}

			publishRouteItems(pos_sp_triplet, current_setpoint_copy, branch_off_item, &next_goal_item);
			break;
		}

	case Stage::ApproachAtGoal: {
			mission_item_s goal_approach_item{};
			mission_item_s landing_item{};
			setGoalApproachMissionItem(goal_approach_item);
			setLandMissionItem(landing_item);
			publishRouteItems(pos_sp_triplet, current_setpoint_copy, goal_approach_item, &landing_item);
			break;
		}

	case Stage::LandAtGoal: {
			mission_item_s landing_item{};

			if (goalIsMissionLanding() && isLandingCommand(_mission_item)) {
				landing_item = _mission_item;

			} else {
				setLandMissionItem(landing_item);
			}

			publishLandingItems(pos_sp_triplet, current_setpoint_copy, landing_item);
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
	perf_begin(_calc_rtl_time_estimate_perf);

	_rtl_time_estimator.update();
	_rtl_time_estimator.setVehicleType(_vehicle_status_sub.get().vehicle_type);
	_rtl_time_estimator.reset();

	if (!_plan.valid() || !_plan.selection.found || _state.stage == Stage::Idle || _state.stage == Stage::LandAtGoal) {
		perf_end(_calc_rtl_time_estimate_perf);
		return _rtl_time_estimator.getEstimate();
	}

	const auto *global_pos = _navigator->get_global_position();

	if (global_pos == nullptr || !PX4_ISFINITE(global_pos->lat) || !PX4_ISFINITE(global_pos->lon)) {
		perf_end(_calc_rtl_time_estimate_perf);
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

	const auto add_goal_legs = [this, &add_leg]() {
		if (!_plan.selection.goal_position.valid()) {
			return;
		}

		if (useGoalLandApproach()) {
			const float approach_altitude = PX4_ISFINITE(_rtl_alt) ? math::min(_goal_land_approach.height_m, _rtl_alt)
							: _goal_land_approach.height_m;
			add_leg(_goal_land_approach.lat, _goal_land_approach.lon, approach_altitude);
		}

		// VTOL final descent is always in MC mode.
		if (_vehicle_status_sub.get().is_vtol) {
			_rtl_time_estimator.setVehicleType(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
		}

		add_leg(_plan.selection.goal_position.lat,
			_plan.selection.goal_position.lon,
			_plan.selection.goal_position.alt);
	};

	if ((_work_item_type == WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE
	     || _work_item_type == WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN)
	    && _plan.join_context.valid()) {
		add_leg(_plan.join_context.projection.lat,
			_plan.join_context.projection.lon,
			_plan.join_context.projection.alt);
	}

	switch (_state.stage) {
	case Stage::FollowRoute:
	case Stage::TransitionDuringRoute: {
			// Walk the remaining route items from the current target to the branch-off (or endpoint).
			int32_t walk_index = _mission.current_seq;

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
					if (_plan.selection.safe_point_found && walk_index == _plan.selection.branchOffIndex()) {
						break;
					}

					if (missionItemMatchesSelectedEndpoint(mi)) {
						break;
					}
				}

				// Advance in the appropriate direction.
				if (_plan.selection.path.direction_reversed) {
					int32_t prev = walk_index;

					if (!findPreviousPositionIndex(walk_index, prev, PositionTraversalType::IgnoreDoJump)) {
						break;
					}

					if (prev >= walk_index) {
						break;
					}

					walk_index = prev;

				} else {
					int32_t next = walk_index + 1;

					if (!findNextPositionIndex(next, next, PositionTraversalType::IgnoreDoJump)) {
						break;
					}

					if (next <= walk_index) {
						break;
					}

					walk_index = next;
				}
			}

			// Add the branch-off → goal leg(s) if applicable.
			if (_plan.selection.safe_point_found) {
				add_goal_legs();
			}

			break;
		}

	case Stage::BranchOff:
		// Already at the branch-off; add only the goal leg(s).
		add_goal_legs();
		break;

	case Stage::ApproachAtGoal:
		add_goal_legs();

		break;

	default:
		break;
	}

	const rtl_time_estimate_s estimate = _rtl_time_estimator.getEstimate();
	perf_end(_calc_rtl_time_estimate_perf);
	return estimate;
}
