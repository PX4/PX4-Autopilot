/****************************************************************************
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
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "rtl_mission_safe_point_follow.h"

#include "mission_item_utils.h"
#include "mission_route_cache.h"
#include "mission_route_geofence.h"
#include "mission_route_planner.h"
#include "navigator.h"

#include <mathlib/mathlib.h>
#include <px4_platform_common/events.h>
#include <systemlib/mavlink_log.h>

RtlMissionSafePointFollow::RtlMissionSafePointFollow(Navigator *navigator) :
	// The items come from the route cache, so no Dataman window.
	RtlBase(navigator, 0)
{
}

void RtlMissionSafePointFollow::planRoute(const mission_s &mission)
{
	_mission = mission;
	_plan = {};
	_plan_failure = mission_route::FailureReason::kNone;

	const MissionRouteCache &route_cache = _navigator->get_mission_route_cache();

	// Plan only on what Mission checked and the route cache holds. Example: new rally points are published
	// before the route cache has loaded them.
	if (!isMissionValid() || (_navigator->get_mission_result()->geofence_id != mission.geofence_id)
	    || !route_cache.missionItemsReady(mission) || !route_cache.safePointsReady()
	    || (route_cache.safePointsId() != mission.safe_points_id)) {
		return;
	}

	const vehicle_global_position_s &global_position = *_navigator->get_global_position();
	const home_position_s &home_position = *_navigator->get_home_position();

	mission_route::RtlRouteRequest request{};
	request.vehicle_position = {global_position.lat, global_position.lon, global_position.alt};
	request.mission_index = math::max(mission.current_seq, int32_t{0});
	request.projection_search_distance_m = _param_mis_mc_seg_dist.get();
	request.safe_point_projection_search_distance_m = _param_rtl_rp_seg_dist.get();
	request.safe_point_arrival_height_m = _param_rtl_descend_alt.get();
	request.acceptance_radius_m = _navigator->get_acceptance_radius();
	request.direct_goal_acceptance_radius_m = _navigator->get_default_acceptance_radius();
	request.altitude_acceptance_radius_m = _navigator->get_altitude_acceptance_radius();
	request.home_altitude_amsl = home_position.valid_alt ? home_position.alt : NAN;

	// A rally point the vehicle cannot wait above inside the fence is not used.
	const mission_route::GeofencePathChecker path_checker{_navigator->get_geofence(), request.safe_point_arrival_height_m};
	const MissionRoutePlanner planner{route_cache, &path_checker};
	_plan_failure = planner.planRtlRoute(request, _plan);
}

bool RtlMissionSafePointFollow::plannedOn(const mission_s &mission) const
{
	// The ids are checksums of the uploads. The same mission uploaded again moves to the other Dataman slot.
	return (mission.mission_id == _mission.mission_id) && (mission.mission_dataman_id == _mission.mission_dataman_id)
	       && (mission.safe_points_id == _mission.safe_points_id) && (mission.geofence_id == _mission.geofence_id);
}

void RtlMissionSafePointFollow::on_activation()
{
	// Not MissionBase::on_activation(): Return never restarts the mission and replays none of its speed, camera or
	// gimbal items. Example: after a survey at 2 m/s it returns at MPC_XY_CRUISE, from its first setpoint on.
	_navigator->reset_triplets();
	_navigator->reset_cruising_speed();

	const bool landed = _land_detected_sub.get().landed;

	if (landed) {
		// Idle on the ground: Return is done at once, as the other Return types report it.
		setEndOfMissionItems();
		_navigator->mode_completed(getNavigatorStateId());
		return;
	}

	if (!_plan.valid()) {
		// Hold where the vehicle stops until RTL finds a route, then go on. Not the end of the mission or of Return.
		setLoiterItemFromCurrentPositionWithBraking(&_mission_item);
		_mission_item.altitude = _navigator->get_global_position()->alt;
		mission_item_to_position_setpoint(_mission_item, &_navigator->get_position_setpoint_triplet()->current);
		_navigator->set_position_setpoint_triplet_updated();

		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Return: no mission route, holding\t");
		events::send(events::ID("rtl_route_unavailable"), {events::Log::Critical, events::LogInternal::Warning},
			     "Return: no mission route, holding");
		// A FailureReason number: the names would cost flash for one console line.
		PX4_WARN("Return route unavailable: planner failure %d (0: inputs not ready)", static_cast<int>(_plan_failure));
		return;
	}

	setMissionIndex(_plan.first_mission_item_index);
	_is_current_planned_mission_item_valid = true;

	if (!_plan.fly_direct_to_goal) {
		_route_join_position = _plan.join_position;
	}

	update_mission();
	checkClimbRequired(_mission.current_seq);
	set_mission_items();
}

bool RtlMissionSafePointFollow::isDescending() const
{
	return (_mission_type == MissionType::MISSION_TYPE_MISSION) && mission_route::isLandingCmd(_mission_item.nav_cmd);
}

bool RtlMissionSafePointFollow::isLanding()
{
	if (_mission_type != MissionType::MISSION_TYPE_MISSION) {
		return false;
	}

	// The landing item itself, also during a precision landing, or a step before it.
	switch (_work_item_type) {
	case WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND:
	case WorkItemType::WORK_ITEM_TYPE_ALIGN_TO_LAND:
	case WorkItemType::WORK_ITEM_TYPE_LAND_DELAY:
		return true;

	default:
		return mission_route::isLandingCmd(_mission_item.nav_cmd);
	}
}

bool RtlMissionSafePointFollow::loadMissionItemFromCache(int32_t index, mission_item_s &mission_item)
{
	return _navigator->get_mission_route_cache().loadMissionItem(_mission, index, mission_item);
}

bool RtlMissionSafePointFollow::isGoal(int32_t index, const mission_item_s &mission_item) const
{
	if (_plan.fly_direct_to_goal) {
		return true;
	}

	switch (_plan.goal_type) {
	case mission_route::GoalType::kSafePoint:
		return index == _plan.branch_off_mission_item_index;

	case mission_route::GoalType::kMissionLand:
		return mission_route::isLandingCmd(mission_item.nav_cmd);

	case mission_route::GoalType::kMissionTakeoff:
		return mission_route::isTakeoffCmd(mission_item.nav_cmd);

	default:
		return false;
	}
}

bool RtlMissionSafePointFollow::findAdjacentRouteIndex(int32_t from_index, int32_t &adjacent_index)
{
	// findPreviousPositionIndex() starts one before from_index, findNextPositionIndex() at its start index.
	return _plan.direction_reversed
	       ? findPreviousPositionIndex(from_index, adjacent_index, MissionTraversalType::IgnoreDoJump)
	       : findNextPositionIndex(from_index + 1, adjacent_index, MissionTraversalType::IgnoreDoJump);
}

bool RtlMissionSafePointFollow::loadNextRouteItem(mission_item_s &mission_item)
{
	int32_t next_index{-1};

	if (!findAdjacentRouteIndex(_mission.current_seq, next_index) || !loadMissionItemFromCache(next_index, mission_item)) {
		return false;
	}

	useBranchOffPoint(next_index, mission_item);
	return true;
}

void RtlMissionSafePointFollow::useBranchOffPoint(int32_t index, mission_item_s &mission_item) const
{
	if ((_plan.goal_type == mission_route::GoalType::kSafePoint) && (index == _plan.branch_off_mission_item_index)) {
		setWaypointMissionItem(mission_item, _plan.branch_off_position);
	}
}

bool RtlMissionSafePointFollow::setNextMissionItem()
{
	int32_t next_index{-1};

	// Touchdown at the goal ends the Return, as the end of a mission.
	if (_land_detected_sub.get().landed || !findAdjacentRouteIndex(_mission.current_seq, next_index)) {
		return false;
	}

	setMissionIndex(next_index);
	return true;
}

void RtlMissionSafePointFollow::setActiveMissionItems()
{
	WorkItemType new_work_item_type{WorkItemType::WORK_ITEM_TYPE_DEFAULT};
	mission_item_s next_mission_items[1] {};
	size_t num_found_items{0u};
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	const position_setpoint_s current_setpoint_copy = pos_sp_triplet->current;

	// A pending join flies the branch-in point first, toward the loaded item.
	handleJoinRoute(new_work_item_type, next_mission_items, num_found_items);

	if (new_work_item_type == WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE) {
		// Joined on the leg where Return leaves the route, it flies on toward the branch-off point.
		useBranchOffPoint(_mission.current_seq, next_mission_items[0u]);
		handleJoinClimb(new_work_item_type, next_mission_items, num_found_items);

	} else if (isGoal(_mission.current_seq, _mission_item)) {
		handleGoal(new_work_item_type, next_mission_items, num_found_items);

	} else {
		// A route item is flown through, without its hold time, toward the next one.
		makeRtlPositionItem(_mission_item);
		num_found_items = loadNextRouteItem(next_mission_items[0u]) ? 1u : 0u;
	}

	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

	if (num_found_items > 0u) {
		mission_item_to_position_setpoint(next_mission_items[0u], &pos_sp_triplet->next);

	} else {
		_navigator->reset_position_setpoint(pos_sp_triplet->next);
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

void RtlMissionSafePointFollow::handleJoinClimb(WorkItemType &new_work_item_type, mission_item_s next_mission_items[],
		size_t &num_found_items)
{
	// MissionBase::checkClimbRequired() sets the altitude when the vehicle is below the branch-in point.
	if (!PX4_ISFINITE(_mission_init_climb_altitude_amsl)) {
		return;
	}

	new_work_item_type = WorkItemType::WORK_ITEM_TYPE_CLIMB;
	num_found_items = 1u;
	next_mission_items[0u] = _mission_item;

	const vehicle_global_position_s &position = *_navigator->get_global_position();
	setLoiterToAltMissionItem(_mission_item, {position.lat, position.lon, _mission_init_climb_altitude_amsl, NAN},
				  _navigator->get_default_loiter_rad());
	_mission_init_climb_altitude_amsl = NAN;
}

void RtlMissionSafePointFollow::handleGoal(WorkItemType &new_work_item_type, mission_item_s next_mission_items[],
		size_t &num_found_items)
{
	// A rally point is left from the branch-off point on the route, with the landing at the rally point next.
	if ((_plan.goal_type == mission_route::GoalType::kSafePoint) && !_plan.fly_direct_to_goal
	    && (_work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT)) {
		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_BRANCH_OFF;
		setWaypointMissionItem(_mission_item, _plan.branch_off_position);
		setGoalLandItem(next_mission_items[0u]);
		num_found_items = 1u;
		return;
	}

	// The mission landing lands as in Mission. Above a rally point or the takeoff point, the vehicle first
	// waits RTL_LAND_DELAY, at most RTL_DESCEND_ALT above it. Example: arriving 20 m above it with 30 m, it waits there.
	if (_plan.goal_type != mission_route::GoalType::kMissionLand) {
		setGoalLandItem(_mission_item);

		const bool wait = (fabsf(_param_rtl_land_delay.get()) > FLT_EPSILON)
				  && (_work_item_type != WorkItemType::WORK_ITEM_TYPE_LAND_DELAY)
				  && !do_need_move_to_item() && !_land_detected_sub.get().landed;

		if (wait) {
			const float wait_alt = math::min(_plan.goal_position.alt + _param_rtl_descend_alt.get(),
							 _navigator->get_global_position()->alt);

			new_work_item_type = WorkItemType::WORK_ITEM_TYPE_LAND_DELAY;
			setLoiterHoldMissionItem(_mission_item, {_plan.goal_position.lat, _plan.goal_position.lon, wait_alt, NAN},
						 _param_rtl_land_delay.get(), _navigator->get_default_loiter_rad());
			return;
		}
	}

	handleLanding(new_work_item_type, next_mission_items, num_found_items);
}

void RtlMissionSafePointFollow::setGoalLandItem(mission_item_s &mission_item) const
{
	mission_item = {};
	setLandMissionItem(mission_item, {_plan.goal_position.lat, _plan.goal_position.lon, _plan.goal_position.alt, NAN});
	mission_item.land_precision = _param_rtl_pld_md.get();
}

rtl_time_estimate_s RtlMissionSafePointFollow::calc_rtl_time_estimate()
{
	_rtl_time_estimator.update();
	_rtl_time_estimator.reset();

	// Without a route, or once stopped or landed, there is nothing left to fly.
	if (_plan.valid() && !(isActive() && (_mission_type == MissionType::MISSION_TYPE_NONE))) {
		addRemainingLegsToTimeEstimate();
	}

	return _rtl_time_estimator.getEstimate();
}

void RtlMissionSafePointFollow::addRemainingLegsToTimeEstimate()
{
	const vehicle_global_position_s &position = *_navigator->get_global_position();
	double lat = position.lat;
	double lon = position.lon;
	float alt = position.alt;

	// A leg from the end of the last one, at the cruise speed and the climb or descent rate.
	const auto add_leg = [&](double to_lat, double to_lon, float to_alt) {
		_rtl_time_estimator.addDistance(get_distance_to_next_waypoint(lat, lon, to_lat, to_lon), {}, to_alt - alt);
		lat = to_lat;
		lon = to_lon;
		alt = to_alt;
	};

	// Before Return starts, the plan is flown from its first item.
	const bool active = isActive();
	const WorkItemType step = active ? _work_item_type : WorkItemType::WORK_ITEM_TYPE_DEFAULT;
	const bool landing = active && mission_route::isLandingCmd(_mission_item.nav_cmd);
	const bool waiting = step == WorkItemType::WORK_ITEM_TYPE_LAND_DELAY;
	const bool goal_steps = landing || waiting || (step == WorkItemType::WORK_ITEM_TYPE_BRANCH_OFF)
				|| (step == WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND) || (step == WorkItemType::WORK_ITEM_TYPE_ALIGN_TO_LAND);
	int32_t index = active ? _mission.current_seq : _plan.first_mission_item_index;
	mission_item_s item{};

	if (!loadMissionItemFromCache(index, item)) {
		_rtl_time_estimator.reset();
		return;
	}

	if (!goal_steps) {
		const mission_route::Position &join = _plan.join_position;

		// The climb in place below the branch-in point, then the leg to it.
		if (active ? _route_join_position.valid() : !_plan.fly_direct_to_goal) {
			if (join.alt > alt) {
				_rtl_time_estimator.addVertDistance(join.alt - alt);
				alt = join.alt;
			}

			add_leg(join.lat, join.lon, join.alt);
		}

		for (int steps = 0; !isGoal(index, item); ++steps) {
			add_leg(item.lat, item.lon, get_absolute_altitude_for_item(item));

			if ((steps >= _mission.count) || !findAdjacentRouteIndex(index, index) || !loadMissionItemFromCache(index, item)) {
				_rtl_time_estimator.reset();
				return;
			}
		}
	}

	if (_plan.goal_type == mission_route::GoalType::kMissionLand) {
		// The move above the mission landing at the current altitude, then the descent.
		add_leg(item.lat, item.lon, alt);
		add_leg(item.lat, item.lon, get_absolute_altitude_for_item(item));
		return;
	}

	const mission_route::Position &goal = _plan.goal_position;
	const bool to_branch_off = !_plan.fly_direct_to_goal
				   && ((step == WorkItemType::WORK_ITEM_TYPE_BRANCH_OFF) || !goal_steps)
				   && (_plan.goal_type == mission_route::GoalType::kSafePoint);

	if (to_branch_off) {
		add_leg(_plan.branch_off_position.lat, _plan.branch_off_position.lon, _plan.branch_off_position.alt);
	}

	add_leg(goal.lat, goal.lon, alt);

	const float land_delay = _param_rtl_land_delay.get();

	if (!landing && (fabsf(land_delay) > FLT_EPSILON)) {
		const float wait_alt = math::min(goal.alt + _param_rtl_descend_alt.get(), alt);
		float wait_s = land_delay;

		if (waiting && (_time_wp_reached != 0)) {
			wait_s = math::max(land_delay - hrt_elapsed_time(&_time_wp_reached) * 1e-6f, 0.f);
		}

		add_leg(goal.lat, goal.lon, wait_alt);
		_rtl_time_estimator.addWait(wait_s);

		// A negative delay holds above the goal: the estimate ends there.
		if (land_delay < 0.f) {
			return;
		}
	}

	// The landing only descends. Example: below a rally point that is not at ground level it adds no climb.
	add_leg(goal.lat, goal.lon, math::min(goal.alt, alt));
}
