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
	if (isExecutingGoalStage()) {
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
		if (_state.stage == Stage::Arrival) {
			setGoalArrivalMissionItem(mission_item, _arrival);
			return true;
		}

		if (_state.stage == Stage::LandAtGoal) {
			setGoalLandMissionItem(mission_item);
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

void RtlMissionSafePointFollow::configureRoute(const mission_route::RtlRoutePlan &plan,
		const loiter_point_s &goal_land_approach)
{
	_plan = plan;
	_goal_land_approach = goal_land_approach;
	// Keep the endpoint command while landing helpers temporarily replace _mission_item,
	// and after the route source may be replaced during the committed landing stage.
	_goal_mission_land_item_valid = goalIsMissionLanding()
					&& loadMissionItemFromCache(_mission.land_index, _goal_mission_land_item)
					&& mission_route::isLandingCmd(_goal_mission_land_item.nav_cmd);
	_active_jump_anchor = _plan.active_jump_anchor;
}

bool RtlMissionSafePointFollow::missionIndexInBounds(int32_t index) const
{
	return index >= 0 && index < _mission.count;
}

bool RtlMissionSafePointFollow::shouldReportMissionItemReached() const
{
	// Only uploaded route targets and an uploaded landing are reached mission items.
	return (_state.stage == Stage::FollowRoute || (_state.stage == Stage::LandAtGoal && goalIsMissionLanding()))
	       && MissionBase::shouldReportMissionItemReached();
}

void RtlMissionSafePointFollow::resetExecutorProgress()
{
	resetVtolTransition();
	_state = {};
	_arrival.reset();
	_goal_arrival_alt = NAN;
	resetJoinRouteState();
}

bool RtlMissionSafePointFollow::useGoalLandApproach() const
{
	return goalIsSafePoint() && _goal_land_approach.isValid()
	       && (vehicleInFwLikeState(_vehicle_status_sub.get()) || isExecutingGoalStage());
}

RtlMissionSafePointFollow::Stage RtlMissionSafePointFollow::finalGoalStage() const
{
	if (goalIsMissionLanding()) {
		return Stage::LandAtGoal;
	}

	return Stage::Arrival;
}

void RtlMissionSafePointFollow::enterGoalStage()
{
	_goal_arrival_alt = _navigator != nullptr ? _navigator->get_global_position()->alt : NAN;

	if (!vehicleInFwLikeState(_vehicle_status_sub.get())) {
		_goal_land_approach = {};
	}

	_arrival.start(useGoalLandApproach() ? RtlArrival::Phase::Descend : RtlArrival::Phase::Move);
	_state.stage = finalGoalStage();
}

bool RtlMissionSafePointFollow::isExecutingGoalStage() const
{
	return _state.stage == Stage::Arrival || _state.stage == Stage::LandAtGoal;
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
		return mission_route::isLandingCmd(mission_item.nav_cmd);
	}

	if (goalIsMissionTakeoff()) {
		return mission_route::isTakeoffCmd(mission_item.nav_cmd);
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
	_vtol_status_sub.update();
	_land_detected_sub.update();

	resetExecutorProgress();
	_active_jump_anchor = _plan.active_jump_anchor;

	if (_plan.valid()) {
		const int32_t first_item_index = _plan.first_mission_item_index;

		if (missionIndexInBounds(first_item_index)) {
			setMissionIndex(first_item_index);
			_is_current_planned_mission_item_valid = isMissionValid();

			if (_plan.fly_direct_to_goal) {
				enterGoalStage();

			} else {
				_state.stage = Stage::FollowRoute;
			}

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

	if (_is_current_planned_mission_item_valid && !_plan.fly_direct_to_goal) {
		if (_plan.geofence_prevents_shortcut) {
			PX4_WARN("RTL direct segment breaches geofence; following mission route");
		}

		// Route Safe Point Return reuses MissionBase's shared JOIN_ROUTE ->
		// TRANSITION_AFTER_JOIN executor path through RtlBase.
		setupJoinRoute(_plan.join_position, _plan.use_current_altitude, allowedRouteTransition(_plan.vtol_transition_action));

		PX4_INFO("RTL route join: alt=%.1f AMSL use_current_alt=%u",
			 static_cast<double>(_plan.join_position.alt),
			 static_cast<unsigned>(_plan.use_current_altitude));

		if (goalIsSafePoint()) {
			PX4_INFO("RTL safe point %d: branch-off item=%d",
				 static_cast<int>(_plan.safe_point_index),
				 static_cast<int>(_plan.branch_off_mission_item_index));
		}
	}

	// Return never restarts the uploaded mission after a disarm.
	_system_disarmed_while_inactive = false;

	MissionBase::on_activation();
}

bool RtlMissionSafePointFollow::frontTransitionInhibited() const
{
	return _vehicle_status_sub.get().is_vtol && _vtol_status_sub.get().fixed_wing_system_failure;
}

mission_route::VtolTransitionAction RtlMissionSafePointFollow::allowedRouteTransition(
	mission_route::VtolTransitionAction action) const
{
	return frontTransitionInhibited() && action == mission_route::VtolTransitionAction::kFrontTransition
	       ? mission_route::VtolTransitionAction::kNone : action;
}

void RtlMissionSafePointFollow::on_active()
{
	_vehicle_status_sub.update();
	_vtol_status_sub.update();

	MissionBase::on_active();

	// A synthetic goal landing does not end the uploaded mission; report Return complete as direct RTL does.
	if (_state.stage == Stage::LandAtGoal && !_mission_item.autocontinue && _land_detected_sub.get().landed) {
		_navigator->mode_completed(getNavigatorStateId());
	}
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

	enterGoalStage();
	_state.advance_route_after_transition = false;
	PX4_DEBUG("RTL %sroute complete, straight to goal", _plan.direction_reversed ? "reverse " : "");
}

bool RtlMissionSafePointFollow::setNextMissionItem()
{
	switch (_state.stage) {
	case Stage::FollowRoute: {
			const mission_route::VtolTransitionAction reverse_transition_action =
				_plan.direction_reversed
				? allowedRouteTransition(vtolTransitionActionAfterReachingReverseTarget(_mission.current_seq))
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
			const bool branch_off_after_transition = isBranchOffIndex(_mission.current_seq);
			_state.advance_route_after_transition = false;

			if (advance_after_transition) {
				_state.stage = Stage::FollowRoute;
				PX4_DEBUG("RTL route transition complete, advancing reverse route");
				advanceRouteTarget();
				return true;
			}

			_state.stage = branch_off_after_transition ? Stage::BranchOff : Stage::FollowRoute;
			PX4_DEBUG(branch_off_after_transition ? "RTL route transition complete, branching off"
				  : "RTL route transition complete");
			return true;
		}

	case Stage::BranchOff:
		enterGoalStage();
		PX4_DEBUG("RTL branch-off reached, straight to goal");
		return true;

	case Stage::Arrival:
		_arrival.advance(_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
				 && !_vehicle_status_sub.get().in_transition_mode
				 && fabsf(_param_rtl_land_delay.get()) <= FLT_EPSILON);

		if (_arrival.phase() == RtlArrival::Phase::Land) {
			_state.stage = Stage::LandAtGoal;
			PX4_DEBUG("RTL goal arrival complete, landing");
		}

		return true;

	case Stage::LandAtGoal:
	case Stage::Idle:
	default:
		return false;
	}
}

void RtlMissionSafePointFollow::setGoalLandMissionItem(mission_item_s &mission_item) const
{
	if (goalIsMissionLanding() && _goal_mission_land_item_valid) {
		mission_item = _goal_mission_land_item;

		// As in MissionBase::set_mission_items(), NAV_FORCE_VT lands a fixed-wing VTOL vertically.
		if (_navigator->force_vtol() && mission_item.nav_cmd == NAV_CMD_LAND) {
			mission_item.nav_cmd = NAV_CMD_VTOL_LAND;
		}

		return;
	}

	mission_item = {};
	mission_item.nav_cmd = _vehicle_status_sub.get().is_vtol ? NAV_CMD_VTOL_LAND : NAV_CMD_LAND;
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

float RtlMissionSafePointFollow::goalArrivalAltitude() const
{
	return PX4_ISFINITE(_goal_arrival_alt) ? _goal_arrival_alt : _navigator->get_global_position()->alt;
}

loiter_point_s RtlMissionSafePointFollow::goalLandApproach(float arrival_altitude, bool use_approach) const
{
	loiter_point_s approach = _goal_land_approach;

	if (!use_approach) {
		approach.lat = _plan.goal_position.lat;
		approach.lon = _plan.goal_position.lon;
		approach.height_m = _plan.goal_position.alt + _param_rtl_descend_alt.get();
	}

	if (!PX4_ISFINITE(approach.loiter_radius_m) || fabsf(approach.loiter_radius_m) <= FLT_EPSILON) {
		approach.loiter_radius_m = _param_rtl_loiter_rad.get();
	}

	// Route flight has its own altitude profile. Arrival must not restart the RTL climb.
	approach.height_m = math::min(approach.height_m, arrival_altitude);
	return approach;
}

void RtlMissionSafePointFollow::setGoalArrivalMissionItem(mission_item_s &mission_item,
		const RtlArrival &arrival) const
{
	const RtlArrival::ItemConfig config{
		.approach = goalLandApproach(goalArrivalAltitude(), useGoalLandApproach()),
		.transit_altitude = goalArrivalAltitude(),
		.land_delay = _param_rtl_land_delay.get()
	};
	mission_item = {};
	arrival.buildMissionItem(*this, mission_item, config, vehicleInFwLikeState(_vehicle_status_sub.get()));
}

void RtlMissionSafePointFollow::setGoalMissionItem(mission_item_s &mission_item) const
{
	if (goalIsMissionLanding()) {
		setGoalLandMissionItem(mission_item);

	} else {
		RtlArrival arrival;
		arrival.start(useGoalLandApproach() ? RtlArrival::Phase::Descend : RtlArrival::Phase::Move);
		setGoalArrivalMissionItem(mission_item, arrival);
	}
}

void RtlMissionSafePointFollow::normalizeRouteMissionItem(mission_item_s &mission_item) const
{
	// Takeoff and landing keep their own execution.
	if (mission_item_contains_position(mission_item) && !mission_route::isTakeoffCmd(mission_item.nav_cmd)
	    && !mission_route::isLandingCmd(mission_item.nav_cmd)) {
		makeRtlPositionItem(mission_item);
	}
}

bool RtlMissionSafePointFollow::findAdjacentRouteIndex(int32_t from_index, int32_t &adjacent_index)
{
	// The route is geometry only, so DO_JUMP items are never followed as control flow.
	// findPreviousPositionIndex() scans from from_index - 1; findNextPositionIndex() includes its start index.
	return _plan.direction_reversed
	       ? findPreviousPositionIndex(from_index, adjacent_index, MissionTraversalType::IgnoreDoJump)
	       : findNextPositionIndex(from_index + 1, adjacent_index, MissionTraversalType::IgnoreDoJump);
}

bool RtlMissionSafePointFollow::resolveRouteTarget(int32_t index, mission_item_s &mission_item) const
{
	if (isBranchOffIndex(index)) {
		setWaypointMissionItem(mission_item, _plan.branch_off_position);
		return true;
	}

	const bool selected_endpoint = missionItemMatchesSelectedEndpoint(mission_item);

	if (selected_endpoint && goalIsMissionTakeoff()) {
		setGoalMissionItem(mission_item);

	} else {
		normalizeRouteMissionItem(mission_item);
	}

	return selected_endpoint;
}

bool RtlMissionSafePointFollow::loadNextRouteItem(mission_item_s &next_route_item, int32_t &next_index)
{
	if (!findAdjacentRouteIndex(_mission.current_seq, next_index)
	    || !loadMissionItemFromCache(next_index, next_route_item)) {
		return false;
	}

	resolveRouteTarget(next_index, next_route_item);
	return true;
}

void RtlMissionSafePointFollow::armRouteTransition(mission_route::VtolTransitionAction action,
		bool advance_route_after_transition)
{
	_state.stage = Stage::TransitionDuringRoute;
	_state.advance_route_after_transition = advance_route_after_transition;
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	const position_setpoint_s current_setpoint_copy = pos_sp_triplet->current;

	const bool branch_off_target_active = isBranchOffIndex(_mission.current_seq) && !_state.advance_route_after_transition;
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

	// Track the resolved outgoing leg while an existing back transition finishes.
	publishRouteItems(pos_sp_triplet, current_setpoint_copy, current_route_item, next_route_item_ptr, false);
	mission_item_s command{};
	set_vtol_transition_item(&command, action == mission_route::VtolTransitionAction::kFrontTransition
				 ? vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW : vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	startVtolTransition(command, &current_route_item, WorkItemType::WORK_ITEM_TYPE_DEFAULT, true);
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

	const bool fw_on_goal_landing = vehicleInFwLikeState(_vehicle_status_sub.get())
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
	const bool branch_off_target_active = isBranchOffIndex(_mission.current_seq);
	// The planner resolved the first segment from its actual jump source.
	// Its join action is authoritative until that target is reached.
	const bool segment_entry_transition_context = _mission.current_seq != _plan.first_mission_item_index
			&& (branch_off_target_active || !_plan.direction_reversed);
	const mission_route::VtolTransitionAction transition_action = segment_entry_transition_context
			? allowedRouteTransition(vtolTransitionActionForTarget(_mission.current_seq, _plan.direction_reversed))
			: mission_route::VtolTransitionAction::kNone;
	const bool wait_for_route_transition = transition_action != mission_route::VtolTransitionAction::kNone
					       && _vehicle_status_sub.get().is_vtol
					       && !_land_detected_sub.get().landed;

	if (wait_for_route_transition) {
		armRouteTransition(transition_action, false);
		return;
	}

	if (branch_off_target_active) {
		_state.stage = Stage::BranchOff;
		PX4_DEBUG("RTL leaving route at branch-off");
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

	if (handleJoinRouteWorkItems()) {
		return;
	}

	if (_state.stage == Stage::FollowRoute && missionItemMatchesSelectedEndpoint(_mission_item)) {
		enterGoalStage();
		PX4_DEBUG("RTL endpoint target active, starting destination arrival");
	}

	switch (_state.stage) {
	case Stage::FollowRoute:
		handleFollowRouteStage(pos_sp_triplet, current_setpoint_copy);
		break;

	case Stage::TransitionDuringRoute:
		break;

	case Stage::BranchOff:
		publishBranchOffItems(pos_sp_triplet, current_setpoint_copy);
		break;

	case Stage::Arrival: {
			mission_item_s goal_item{};
			setGoalArrivalMissionItem(goal_item, _arrival);

			if (_arrival.phase() == RtlArrival::Phase::Move) {
				publishRouteItems(pos_sp_triplet, current_setpoint_copy, goal_item, nullptr);
				break;
			}

			// An offset approach must exit towards the landing point, as in direct RTL.
			goal_item.force_heading = goal_item.autocontinue
						  && get_distance_to_next_waypoint(goal_item.lat, goal_item.lon,
								  _plan.goal_position.lat, _plan.goal_position.lon) > _navigator->get_acceptance_radius();
			mission_item_s landing_item{};
			setGoalLandMissionItem(landing_item);
			publishRouteItems(pos_sp_triplet, current_setpoint_copy, goal_item,
					  goal_item.autocontinue ? &landing_item : nullptr);
			pos_sp_triplet->previous.valid = false;

			if (_arrival.phase() == RtlArrival::Phase::Hold && goal_item.autocontinue) {
				// The descent stage already enforced altitude before the landing delay began.
				pos_sp_triplet->current.alt_acceptance_radius = FLT_MAX;
			}

			break;
		}

	case Stage::LandAtGoal: {
			mission_item_s landing_item{};
			setGoalLandMissionItem(landing_item);
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

	// A newly configured executor may be queried before its first inactive cycle.
	_vehicle_status_sub.update();
	_vtol_status_sub.update();
	_rtl_time_estimator.update();
	_rtl_time_estimator.setVehicleType(_vehicle_status_sub.get().vehicle_type);
	_rtl_time_estimator.reset();

	const vehicle_global_position_s *global_pos = _navigator->get_global_position();
	const bool can_estimate = _plan.valid()
				  && global_pos != nullptr
				  && PX4_ISFINITE(global_pos->lat)
				  && PX4_ISFINITE(global_pos->lon)
				  && PX4_ISFINITE(global_pos->alt);

	if (can_estimate) {
		addRemainingLegsToTimeEstimate(*global_pos);
	}

	const rtl_time_estimate_s estimate = _rtl_time_estimator.getEstimate();
	perf_end(_calc_rtl_time_estimate_perf);
	return estimate;
}

void RtlMissionSafePointFollow::addRemainingLegsToTimeEstimate(const vehicle_global_position_s &global_pos)
{
	// RTL refreshes the plan from the current Mission index and vehicle position before
	// an inactive estimate. Forecast from that plan without changing execution progress.
	const bool inactive = _state.stage == Stage::Idle;
	const Stage stage = inactive ? (_plan.fly_direct_to_goal ? finalGoalStage() : Stage::FollowRoute) : _state.stage;
	const bool join_remaining = inactive ? !_plan.fly_direct_to_goal
				    : (_work_item_type == WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE
				       || _work_item_type == WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN
				       || _vtol_transition.completion_work == WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN);
	const position_setpoint_s &current_setpoint = _navigator->get_position_setpoint_triplet()->current;
	matrix::Vector2d hor_pos{global_pos.lat, global_pos.lon};
	float altitude = global_pos.alt;
	uint8_t current_vehicle_type = vehicleInFwLikeState(_vehicle_status_sub.get())
				       ? vehicle_status_s::VEHICLE_TYPE_FIXED_WING : _vehicle_status_sub.get().vehicle_type;

	if (frontTransitionInhibited()) {
		current_vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	}

	uint8_t estimated_vehicle_type = current_vehicle_type;
	_rtl_time_estimator.setVehicleType(estimated_vehicle_type);

	const auto set_route_mode = [&](mission_route::VtolTransitionAction action) {
		if (_vehicle_status_sub.get().is_vtol) {
			estimated_vehicle_type = current_vehicle_type;

			if (allowedRouteTransition(action) == mission_route::VtolTransitionAction::kFrontTransition) {
				estimated_vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

			} else if (action == mission_route::VtolTransitionAction::kBackTransition) {
				estimated_vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
			}

			_rtl_time_estimator.setVehicleType(estimated_vehicle_type);
		}
	};

	// Add a leg from the tracked position to a target, then track from the target.
	const auto add_leg = [&](double target_lat, double target_lon, float target_alt) {
		const float hor_dist = get_distance_to_next_waypoint(hor_pos(0), hor_pos(1), target_lat, target_lon);
		matrix::Vector2f direction{};

		// Only the fixed-wing wind correction reads the direction of a nonzero leg.
		if (estimated_vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING && hor_dist > FLT_EPSILON) {
			get_vector_to_next_waypoint(hor_pos(0), hor_pos(1), target_lat, target_lon, &direction(0), &direction(1));
		}

		_rtl_time_estimator.addDistance(hor_dist, direction, target_alt - altitude);

		hor_pos(0) = target_lat;
		hor_pos(1) = target_lon;
		altitude = target_alt;
	};

	const auto add_position_leg = [&](const mission_route::Position & position) {
		add_leg(position.lat, position.lon, position.alt);
	};

	const auto add_mission_item_legs = [&](const mission_item_s & item, bool current_target) {
		const bool loiter_to_alt = item.nav_cmd == NAV_CMD_LOITER_TO_ALT;
		// As in execution, NAV_FORCE_VT lands a fixed-wing VTOL vertically at a plain LAND.
		const bool vtol_landing = _vehicle_status_sub.get().is_vtol
					  && (item.nav_cmd == NAV_CMD_VTOL_LAND
					      || (item.nav_cmd == NAV_CMD_LAND && _param_nav_force_vt.get()
						  && estimated_vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING));
		const bool mc_landing = mission_route::isLandingCmd(item.nav_cmd)
					&& estimated_vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

		if (loiter_to_alt || mc_landing || vtol_landing) {
			// MissionBlock approaches the loiter before changing altitude. MissionBase
			// likewise moves MC horizontally to LAND before starting the descent.
			float approach_altitude = altitude;

			if (current_target && current_setpoint.valid
			    && (loiter_to_alt || _work_item_type == WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND)) {
				approach_altitude = current_setpoint.alt;
			}

			add_leg(item.lat, item.lon, approach_altitude);
		}

		if (vtol_landing) {
			set_route_mode(mission_route::VtolTransitionAction::kBackTransition);
		}

		add_leg(item.lat, item.lon, get_absolute_altitude_for_item(item));
	};

	const auto add_goal_legs = [&]() {
		if (!_plan.goal_position.valid()) {
			return;
		}

		const float arrival_altitude = isExecutingGoalStage() ? goalArrivalAltitude() : altitude;
		const bool use_approach = goalIsSafePoint() && _goal_land_approach.isValid()
					  && (estimated_vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING || isExecutingGoalStage());
		const loiter_point_s approach = goalLandApproach(arrival_altitude, use_approach);
		const bool hold_requested = use_approach
					    || estimated_vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					    || fabsf(_param_rtl_land_delay.get()) > FLT_EPSILON;

		const bool holding = !inactive && stage == Stage::Arrival && _arrival.phase() == RtlArrival::Phase::Hold;

		if (!holding) {
			// Approach horizontally before descending, as the destination stages do.
			const float approach_altitude = !inactive && stage == Stage::Arrival && _arrival.phase() == RtlArrival::Phase::Descend
							&& current_setpoint.valid
							? current_setpoint.alt : arrival_altitude;
			add_leg(approach.lat, approach.lon, approach_altitude);

			if (hold_requested) {
				add_leg(approach.lat, approach.lon, approach.height_m);
			}
		}

		if (hold_requested) {
			float remaining_wait = _param_rtl_land_delay.get();

			if (holding && remaining_wait > 0.f && _time_wp_reached != 0) {
				remaining_wait = math::max(0.f, remaining_wait - hrt_elapsed_time(&_time_wp_reached) * 1e-6f);
			}

			_rtl_time_estimator.addWait(remaining_wait);

			if (_param_rtl_land_delay.get() < -FLT_EPSILON) {
				// Like direct RTL, estimate time to the indefinite hold rather than a landing.
				return;
			}
		}

		// Fly the landing approach in the arrival mode, then descend as MC for VTOL.
		add_leg(_plan.goal_position.lat, _plan.goal_position.lon, altitude);

		if (_vehicle_status_sub.get().is_vtol) {
			set_route_mode(mission_route::VtolTransitionAction::kBackTransition);
		}

		_rtl_time_estimator.addVertDistance(_plan.goal_position.alt - altitude);
	};

	if (join_remaining && _plan.join_position.valid()) {
		add_leg(_plan.join_position.lat, _plan.join_position.lon,
			_plan.use_current_altitude ? altitude : _plan.join_position.alt);
	}

	if (join_remaining) {
		set_route_mode(_plan.vtol_transition_action);
	}

	switch (stage) {
	case Stage::FollowRoute:
	case Stage::TransitionDuringRoute: {
			// Walk the route from the current target to the branch-off or endpoint.
			// The step limit guards against corrupted data.
			int32_t walk_index = inactive ? _plan.first_mission_item_index : _mission.current_seq;

			if (!inactive && stage != Stage::FollowRoute && _state.advance_route_after_transition) {
				// The reverse waypoint was reached before this transition was armed.
				findAdjacentRouteIndex(_mission.current_seq, walk_index);
			}

			// One pass over the mission instead of a lookback per VTOL route target.
			static uint8_t vtol_states[MissionRouteCache::kMaxFullMissionCacheSize];
			const bool vtol_states_valid = _vehicle_status_sub.get().is_vtol
						       && getVtolStatesForMission(vtol_states, MissionRouteCache::kMaxFullMissionCacheSize);
			const uint8_t *route_vtol_states = vtol_states_valid ? vtol_states : nullptr;

			for (int steps = 0; steps < _mission.count && missionIndexInBounds(walk_index); ++steps) {
				mission_item_s item{};

				if (!loadMissionItemFromCache(walk_index, item)) {
					break;
				}

				if (mission_item_contains_position(item)) {
					if (walk_index != _plan.first_mission_item_index) {
						set_route_mode(vtolTransitionActionForTarget(walk_index, _plan.direction_reversed, route_vtol_states));
					}

					const bool last_route_item = resolveRouteTarget(walk_index, item);

					if (last_route_item && goalIsMissionTakeoff()) {
						// The takeoff altitude is replaced by the synthetic destination approach.
						break;
					}

					add_mission_item_legs(item, !inactive && !join_remaining && walk_index == _mission.current_seq);

					if (last_route_item) {
						break;
					}
				}

				int32_t adjacent_index = -1;

				if (!findAdjacentRouteIndex(walk_index, adjacent_index)) {
					break;
				}

				walk_index = adjacent_index;
			}

			if (goalIsSafePoint() || goalIsMissionTakeoff()) {
				add_goal_legs();
			}

			break;
		}

	case Stage::BranchOff:
		add_position_leg(_plan.branch_off_position);
		add_goal_legs();
		break;

	case Stage::Arrival:
		add_goal_legs();
		break;

	case Stage::LandAtGoal: {
			mission_item_s landing_item{};
			setGoalLandMissionItem(landing_item);
			add_mission_item_legs(landing_item, !inactive);
			break;
		}

	default:
		break;
	}
}

#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
