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
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0

#include "mission_item_utils.h"
#include "navigator.h"

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

bool RtlMissionSafePointFollow::isMissionValid() const
{
	if (_state.stage == Stage::ApproachAtGoal || _state.stage == Stage::LandAtGoal) {
		// Feasibility results may already describe a replacement mission. The
		// committed landing still belongs to the previously validated plan.
		return _plan.valid() && missionIndexInBounds(_mission.current_seq);
	}

	return MissionBase::isMissionValid();
}

bool RtlMissionSafePointFollow::loadMissionItemFromCache(int32_t index, mission_item_s &mission_item)
{
	// Once committed to the goal, all remaining work belongs to the frozen plan.
	// MissionBase reloads the current item between approach/landing helper stages;
	// a replacement mission must not make those reloads abort the landing.
	if (missionIndexInBounds(index) && index == _mission.current_seq) {
		if (_state.stage == Stage::ApproachAtGoal) {
			setGoalApproachMissionItem(mission_item);
			return true;
		}

		if (_state.stage == Stage::LandAtGoal) {
			setLandMissionItem(mission_item);
			return true;
		}
	}

	if (_navigator == nullptr) {
		return false;
	}

	const MissionRouteCache &mission_route_cache = _navigator->get_mission_route_cache();

	return missionIndexInBounds(index)
	       && mission_route_cache.missionItemsReady(_mission)
	       && mission_route_cache.loadMissionItem(_mission, index, mission_item);
}

void RtlMissionSafePointFollow::configureRouteSafePoint(const RouteSafePointConfig &config)
{
	_plan = config.plan;
	_goal_land_approach = config.goal_land_approach;
	_rtl_alt = config.rtl_alt;
	_vtol_state_on_mission_upload = config.vtol_state_on_mission_upload;
	// Keep the endpoint command while landing helpers temporarily replace _mission_item,
	// and after the route source may be replaced during the committed landing stage.
	_goal_mission_land_item_valid = goalIsMissionLanding()
					&& loadMissionItemFromCache(_mission.land_index, _goal_mission_land_item)
					&& isLandingCommand(_goal_mission_land_item);
	_active_jump_anchor = _plan.active_jump_anchor;
}

bool RtlMissionSafePointFollow::missionIndexInBounds(int32_t index) const
{
	return index >= 0 && index < _mission.count;
}

void RtlMissionSafePointFollow::resetExecutorProgress()
{
	_state = {};
	resetJoinRouteState();
}

bool RtlMissionSafePointFollow::useGoalLandApproach() const
{
	return goalIsSafePoint() && _goal_land_approach.isValid();
}

RtlMissionSafePointFollow::Stage RtlMissionSafePointFollow::finalGoalStage() const
{
	return useGoalLandApproach() ? Stage::ApproachAtGoal : Stage::LandAtGoal;
}

bool RtlMissionSafePointFollow::goalIsMissionLanding() const
{
	return _plan.goal_type == mission_route::GoalType::kMissionLand;
}

bool RtlMissionSafePointFollow::goalIsMissionTakeoff() const
{
	return _plan.goal_type == mission_route::GoalType::kMissionTakeoff;
}

bool RtlMissionSafePointFollow::goalIsSafePoint() const
{
	return _plan.goal_type == mission_route::GoalType::kSafePoint;
}

bool RtlMissionSafePointFollow::isBranchOffIndex(int32_t index) const
{
	return goalIsSafePoint() && index == _plan.branch_off_mission_item_index;
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
	_active_jump_anchor = _plan.active_jump_anchor;

	if (_plan.valid()) {
		const int32_t first_item_index = _plan.first_mission_item_index;

		if (missionIndexInBounds(first_item_index)) {
			setMissionIndex(first_item_index);
			_is_current_planned_mission_item_valid = isMissionValid();
			_state.stage = _plan.fly_direct_to_goal ? finalGoalStage() : Stage::FollowRoute;

		} else {
			PX4_ERR("RTL plan start index out of bounds: %ld (count=%ld)",
				static_cast<long>(first_item_index), static_cast<long>(_mission.count));
			_is_current_planned_mission_item_valid = false;
			_state.stage = Stage::Idle;
		}

		PX4_INFO("RTL to %s target=%d rev=%u skip_route=%u vtol=%u stage=%u",
			 mission_route::goalTypeString(_plan.goal_type),
			 static_cast<int>(_plan.first_mission_item_index),
			 static_cast<unsigned>(_plan.direction_reversed),
			 static_cast<unsigned>(_plan.fly_direct_to_goal),
			 static_cast<unsigned>(_plan.vtol_transition_action),
			 static_cast<unsigned>(_state.stage));

	} else {
		_is_current_planned_mission_item_valid = false;
	}

	if (_land_detected_sub.get().landed) {
		_state.stage = Stage::Idle;
		_is_current_planned_mission_item_valid = false;
	}

	// Reset the triplet when retargeting the mission so the controller does not keep following a stale line.
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->current.valid = false;
	pos_sp_triplet->next.valid = false;

	if (_is_current_planned_mission_item_valid && !_plan.fly_direct_to_goal) {
		// Route Safe Point Return reuses MissionBase's shared JOIN_ROUTE ->
		// TRANSITION_AFTER_JOIN executor path through RtlBase.
		setupJoinRoute(_plan.join_position, _plan.use_current_altitude, _plan.vtol_transition_action);

		PX4_INFO("RTL route join: alt=%.1f AMSL use_current_alt=%u",
			 static_cast<double>(_plan.join_position.alt),
			 static_cast<unsigned>(_plan.use_current_altitude));

		if (goalIsSafePoint()) {
			PX4_INFO("RTL safe point %d: branch-off item=%d",
				 static_cast<int>(_plan.safe_point_index),
				 static_cast<int>(_plan.branch_off_mission_item_index));
		}
	}

	MissionBase::on_activation();
}

void RtlMissionSafePointFollow::advanceRouteTarget()
{
	// The planned join may follow a jump edge. All later RTL advances ignore
	// DO_JUMP control flow, so they cannot create another flown jump anchor.
	_active_jump_anchor = {};

	int32_t next_index = -1;

	if (findAdjacentRouteIndex(_mission.current_seq, next_index)) {
		setMissionIndex(next_index);
		return;
	}

	_state.stage = finalGoalStage();
	_state.clearRouteTransition();
	PX4_INFO("RTL %sroute complete, straight to goal", _plan.direction_reversed ? "reverse " : "");
}

bool RtlMissionSafePointFollow::setNextMissionItem()
{
	switch (_state.stage) {
	case Stage::FollowRoute: {
			const mission_route::VtolTransitionAction reverse_transition_action =
				_plan.direction_reversed
				? vtolTransitionActionAfterReachingReverseTarget(_mission.current_seq)
				: mission_route::VtolTransitionAction::kNone;
			const bool wait_for_reverse_transition = reverse_transition_action != mission_route::VtolTransitionAction::kNone
					&& _vehicle_status_sub.get().is_vtol && !_land_detected_sub.get().landed;

			if (wait_for_reverse_transition) {
				armRouteTransition(reverse_transition_action, true);
				return true;
			}

			advanceRouteTarget();
			return true;
		}

	case Stage::TransitionDuringRoute: {
			const bool advance_after_transition = _state.advance_route_after_transition;
			const bool branch_off_after_transition = currentTargetIsBranchOff();
			_state.clearRouteTransition();

			if (advance_after_transition) {
				_state.stage = Stage::FollowRoute;
				PX4_INFO("RTL route transition complete, advancing reverse route");
				advanceRouteTarget();
				return true;
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
		const mission_route::Position &position) const
{
	mission_item = {};
	mission_item.nav_cmd = NAV_CMD_WAYPOINT;
	mission_item.lat = position.lat;
	mission_item.lon = position.lon;
	mission_item.altitude = position.alt;
	mission_item.altitude_is_relative = false;
	mission_item.acceptance_radius = _navigator->get_acceptance_radius();

	if (vehicleInFwLikeState(_vehicle_status_sub.get())) {
		// Match the legacy join behavior: use a wider acceptance radius when the vehicle is already in FW-like flow.
		mission_item.acceptance_radius = kJoinRouteFlyByAcceptanceRadiusScale * _navigator->get_acceptance_radius();
	}

	mission_item.yaw = NAN;
	mission_item.time_inside = 0.f;
	mission_item.autocontinue = true;
	mission_item.origin = ORIGIN_ONBOARD;
}

void RtlMissionSafePointFollow::setLandMissionItem(mission_item_s &mission_item) const
{
	if (goalIsMissionLanding() && _goal_mission_land_item_valid) {
		mission_item = _goal_mission_land_item;
		return;
	}

	mission_item = {};
	mission_item.nav_cmd = (_vehicle_status_sub.get().is_vtol || _navigator->force_vtol()) ? NAV_CMD_VTOL_LAND : NAV_CMD_LAND;
	mission_item.lat = _plan.goal_position.lat;
	mission_item.lon = _plan.goal_position.lon;

	mission_item.altitude = _plan.goal_position.alt;

	mission_item.altitude_is_relative = false;
	mission_item.yaw = NAN;
	mission_item.time_inside = 0.f;
	mission_item.land_precision = _param_rtl_pld_md.get();
	mission_item.autocontinue = false;
	mission_item.origin = ORIGIN_ONBOARD;
}

void RtlMissionSafePointFollow::setGoalApproachMissionItem(mission_item_s &mission_item) const
{
	// Match direct RTL: do not climb above the already computed RTL altitude when entering
	// the selected VTOL approach loiter.
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

void RtlMissionSafePointFollow::setGoalMissionItem(mission_item_s &mission_item) const
{
	if (useGoalLandApproach()) {
		setGoalApproachMissionItem(mission_item);

	} else {
		setLandMissionItem(mission_item);
	}
}

void RtlMissionSafePointFollow::normalizeRouteMissionItem(mission_item_s &mission_item) const
{
	if (!mission_item_contains_position(mission_item)) {
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
	return isBranchOffIndex(_mission.current_seq);
}

bool RtlMissionSafePointFollow::findAdjacentRouteIndex(int32_t from_index, int32_t &adjacent_index)
{
	// The route is geometry only, so DO_JUMP items are never followed as control flow.
	// findPreviousPositionIndex() scans from from_index - 1; findNextPositionIndex() includes its start index.
	return _plan.direction_reversed
	       ? findPreviousPositionIndex(from_index, adjacent_index, MissionTraversalType::IgnoreDoJump)
	       : findNextPositionIndex(from_index + 1, adjacent_index, MissionTraversalType::IgnoreDoJump);
}

bool RtlMissionSafePointFollow::loadNextRouteItem(mission_item_s &next_route_item, int32_t &next_index)
{
	if (!findAdjacentRouteIndex(_mission.current_seq, next_index)
	    || !loadMissionItemFromCache(next_index, next_route_item)) {
		return false;
	}

	if (isBranchOffIndex(next_index)) {
		// Show the controller the projected branch-off before it becomes the current target.
		setWaypointMissionItem(next_route_item, _plan.branch_off_position);

	} else {
		normalizeRouteMissionItem(next_route_item);
	}

	return true;
}

void RtlMissionSafePointFollow::armRouteTransition(mission_route::VtolTransitionAction action,
		bool advance_route_after_transition)
{
	_state.stage = Stage::TransitionDuringRoute;
	_state.transition_target_index = _mission.current_seq;
	_state.transition_action = action;
	_state.transition_command_sent = false;
	_state.advance_route_after_transition = advance_route_after_transition;
}

void RtlMissionSafePointFollow::handleRouteTransitionStage(position_setpoint_triplet_s *pos_sp_triplet,
		const position_setpoint_s &current_setpoint_copy)
{
	if (_state.transition_command_sent) {
		return;
	}

	if (_state.transition_action == mission_route::VtolTransitionAction::kNone
	    || !missionIndexInBounds(_state.transition_target_index)) {
		PX4_ERR("RTL route transition stage is missing a valid target/action");
		_state.clearRouteTransition();
		_state.stage = currentTargetIsBranchOff() ? Stage::BranchOff : Stage::FollowRoute;
		return;
	}

	const bool branch_off_target_active = currentTargetIsBranchOff() && !_state.advance_route_after_transition;
	mission_item_s current_route_item{};
	mission_item_s next_route_item{};
	mission_item_s *next_route_item_ptr = nullptr;
	int32_t adjacent_index = -1;

	if (branch_off_target_active) {
		setWaypointMissionItem(current_route_item, _plan.branch_off_position);
		setGoalMissionItem(next_route_item);
		next_route_item_ptr = &next_route_item;

	} else {
		current_route_item = _mission_item;
		normalizeRouteMissionItem(current_route_item);

		if (loadNextRouteItem(next_route_item, adjacent_index)) {
			next_route_item_ptr = &next_route_item;
		}
	}

	if (_state.advance_route_after_transition
	    && _plan.direction_reversed
	    && next_route_item_ptr != nullptr) {
		// Reverse waypoint-attached semantics trigger the transition only after the current
		// waypoint is reached. During that transition the vehicle must already track the next
		// reverse target, not fly back toward the waypoint it just completed.
		current_route_item = *next_route_item_ptr;
		next_route_item_ptr = nullptr;
	}

	publishRouteItems(pos_sp_triplet, current_setpoint_copy, current_route_item, next_route_item_ptr, false);

	mission_item_s transition_item{};
	const bool front_transition = _state.transition_action == mission_route::VtolTransitionAction::kFrontTransition;
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
					      _plan.branch_off_position.lat,
					      _plan.branch_off_position.lon);

		} else {
			int32_t alignment_index = _mission.current_seq;

			if (_state.advance_route_after_transition
			    && _plan.direction_reversed
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

	if (_state.transition_action == mission_route::VtolTransitionAction::kBackTransition) {
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
		// Reach checks must use the published target, which may be a virtual branch-off
		// instead of the uploaded mission waypoint.
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

void RtlMissionSafePointFollow::publishBranchOffItems(position_setpoint_triplet_s *pos_sp_triplet,
		const position_setpoint_s &current_setpoint_copy)
{
	mission_item_s branch_off_item{};
	mission_item_s next_goal_item{};
	setWaypointMissionItem(branch_off_item, _plan.branch_off_position);
	setGoalMissionItem(next_goal_item);

	publishRouteItems(pos_sp_triplet, current_setpoint_copy, branch_off_item, &next_goal_item);
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
	// The planner resolved the first segment using its actual jump source and
	// upload state. Its join action is authoritative until that target is reached.
	const bool segment_entry_transition_context = _mission.current_seq != _plan.first_mission_item_index
			&& (branch_off_target_active || !_plan.direction_reversed);
	const mission_route::VtolTransitionAction transition_action = segment_entry_transition_context
			? vtolTransitionActionForTarget(_mission.current_seq, _plan.direction_reversed)
			: mission_route::VtolTransitionAction::kNone;
	const bool wait_for_route_transition = transition_action != mission_route::VtolTransitionAction::kNone
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
		// Publish the branch-off now so the next reach check uses the projected point.
		publishBranchOffItems(pos_sp_triplet, current_setpoint_copy);
		return;
	}

	mission_item_s current_route_item = _mission_item;
	normalizeRouteMissionItem(current_route_item);

	mission_item_s next_route_item{};
	int32_t next_index = -1;
	const bool has_next_route_item = loadNextRouteItem(next_route_item, next_index);

	publishRouteItems(pos_sp_triplet, current_setpoint_copy, current_route_item,
			  has_next_route_item ? &next_route_item : nullptr);
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

	case Stage::BranchOff:
		publishBranchOffItems(pos_sp_triplet, current_setpoint_copy);
		break;

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
			setLandMissionItem(landing_item);
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

	const vehicle_global_position_s *global_pos = _navigator->get_global_position();
	const bool can_estimate = _plan.valid()
				  && _state.stage != Stage::Idle
				  && _state.stage != Stage::LandAtGoal
				  && global_pos != nullptr
				  && PX4_ISFINITE(global_pos->lat)
				  && PX4_ISFINITE(global_pos->lon);

	if (can_estimate) {
		addRemainingLegsToTimeEstimate(*global_pos);
	}

	const rtl_time_estimate_s estimate = _rtl_time_estimator.getEstimate();
	perf_end(_calc_rtl_time_estimate_perf);
	return estimate;
}

void RtlMissionSafePointFollow::addRemainingLegsToTimeEstimate(const vehicle_global_position_s &global_pos)
{
	matrix::Vector2d hor_pos{global_pos.lat, global_pos.lon};
	float altitude = global_pos.alt;

	// Add a leg from the tracked position to a target, then track from the target.
	const auto add_leg = [&](double target_lat, double target_lon, float target_alt) {
		matrix::Vector2f direction{};
		get_vector_to_next_waypoint(hor_pos(0), hor_pos(1), target_lat, target_lon, &direction(0), &direction(1));

		const float hor_dist = get_distance_to_next_waypoint(hor_pos(0), hor_pos(1), target_lat, target_lon);
		_rtl_time_estimator.addDistance(hor_dist, direction, target_alt - altitude);

		hor_pos(0) = target_lat;
		hor_pos(1) = target_lon;
		altitude = target_alt;
	};

	const auto add_position_leg = [&](const mission_route::Position & position) {
		add_leg(position.lat, position.lon, position.alt);
	};

	const auto add_goal_legs = [&]() {
		if (!_plan.goal_position.valid()) {
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

		add_position_leg(_plan.goal_position);
	};

	if ((_work_item_type == WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE
	     || _work_item_type == WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN)
	    && _plan.join_position.valid()) {
		add_position_leg(_plan.join_position);
	}

	switch (_state.stage) {
	case Stage::FollowRoute:
	case Stage::TransitionDuringRoute: {
			// Walk the route from the current target to the branch-off or endpoint.
			// The step limit guards against corrupted data.
			int32_t walk_index = _mission.current_seq;

			for (int steps = 0; steps < _mission.count && missionIndexInBounds(walk_index); ++steps) {
				mission_item_s item{};

				if (!loadMissionItemFromCache(walk_index, item)) {
					break;
				}

				if (mission_item_contains_position(item)) {
					if (isBranchOffIndex(walk_index)) {
						add_position_leg(_plan.branch_off_position);
						break;
					}

					add_leg(item.lat, item.lon, get_absolute_altitude_for_item(item));

					if (missionItemMatchesSelectedEndpoint(item)) {
						break;
					}
				}

				int32_t adjacent_index = -1;

				if (!findAdjacentRouteIndex(walk_index, adjacent_index)) {
					break;
				}

				walk_index = adjacent_index;
			}

			if (goalIsSafePoint()) {
				add_goal_legs();
			}

			break;
		}

	case Stage::BranchOff:
		add_position_leg(_plan.branch_off_position);
		add_goal_legs();
		break;

	case Stage::ApproachAtGoal:
		add_goal_legs();
		break;

	default:
		break;
	}
}

#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
