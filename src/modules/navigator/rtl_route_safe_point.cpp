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
 * @file rtl_route_safe_point.cpp
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "rtl_route_safe_point.h"

#include "navigator.h"
#include "rtl_base.h"

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
#include "mission_route_cache.h"
#include "mission_route_planner.h"
#include "rtl_mission_safe_point_follow.h"

#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>

namespace
{
class MissionViewProvider final : public mission_route::Provider
{
public:
	MissionViewProvider(const MissionRouteCache &cache, const MissionRouteCache::MissionView &view,
			    int32_t land_index) :
		_cache(cache), _view(view), _land_index(land_index)
	{}

	int missionCount() const override { return _view.count; }

	bool loadMissionItem(int index, mission_item_s &mission_item) const override
	{
		if (index < 0 || index >= _view.count || _view.items == nullptr) {
			return false;
		}

		mission_item = _view.items[index];
		return true;
	}

	int safePointCount() const override { return _cache.safePointCount(); }

	bool loadSafePointItem(int index, mission_item_s &safe_point_item) const override
	{
		return _cache.loadSafePointItem(index, safe_point_item);
	}

	bool hasVtolLandApproachesAtSafePointIndex(int safe_point_index, float home_altitude_amsl) const override
	{
		return _cache.hasVtolLandApproachesAtSafePointIndex(safe_point_index, home_altitude_amsl);
	}

	bool getMissionLandItem(int32_t &index, mission_item_s &land_item) const override
	{
		mission_item_s item{};

		if (!loadMissionItem(_land_index, item) || !mission_route::isLandingCmd(item.nav_cmd)) {
			return false;
		}

		index = _land_index;
		land_item = item;
		return true;
	}

private:
	const MissionRouteCache &_cache;
	MissionRouteCache::MissionView _view;
	int32_t _land_index;
};
} // namespace
#endif

RtlRouteSafePoint::RtlRouteSafePoint(ModuleParams *parent, Navigator *navigator)
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	: ModuleParams(parent),
	  _navigator(navigator)
#endif
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE == 0
	(void)parent;
	(void)navigator;
#endif
}

void RtlRouteSafePoint::reset()
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	_plan = {};
	_goal_land_approach = {};
	_last_flown_loop_segment = {};
	_mission_id = 0;
	_mission_generation = 0;
	_safe_points_id = 0;
	_mission_count = 0;
	_safe_point_count = 0;
	_mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	_safe_points_dataman_id = DM_KEY_SAFE_POINTS_0;
	_source_valid = false;
	_direction_reversed = false;
	_waiting_for_inputs = false;
#endif
}

bool RtlRouteSafePoint::supportsVehicle(const vehicle_status_s &vehicle_status) const
{
	return available() && !vehicle_status.is_vtol;
}

bool RtlRouteSafePoint::inputsReady(const mission_s &mission) const
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	const MissionRouteCache &cache = _navigator->get_mission_route_cache();
	return cache.missionItemsReady(mission) && cache.safePointsReady();
#else
	(void)mission;
	return false;
#endif
}

bool RtlRouteSafePoint::missionMatches(const mission_s &mission) const
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	return _source_valid
	       && mission.mission_id == _mission_id
	       && mission.count == _mission_count
	       && mission.mission_dataman_id == _mission_dataman_id;
#else
	(void)mission;
	return false;
#endif
}

bool RtlRouteSafePoint::sourceStillValid(const mission_s &mission) const
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0

	if (!sourceMatches(mission)) {
		return false;
	}

	const MissionRouteCache &cache = _navigator->get_mission_route_cache();

	if (!cache.safePointsReady()
	    || cache.safePointsId() != _safe_points_id
	    || cache.safePointCount() != _safe_point_count) {
		return false;
	}

	MissionRouteCache::MissionView view{};
	return cache.getMissionView(mission, view) && view.generation == _mission_generation;
#else
	(void)mission;
	return false;
#endif
}

bool RtlRouteSafePoint::evaluationPending(const mission_s &mission) const
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	return mission.count <= MissionRouteCache::kMaxFullMissionCacheSize
	       && !_navigator->get_mission_route_cache().missionItemsReady(mission);
#else
	(void)mission;
	return false;
#endif
}

bool RtlRouteSafePoint::retryReady(const mission_s &mission) const
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	return _waiting_for_inputs && inputsReady(mission);
#else
	(void)mission;
	return false;
#endif
}

RtlRouteSafePoint::Evaluation RtlRouteSafePoint::evaluate(const mission_s &mission,
		const vehicle_status_s &vehicle_status,
		const vehicle_global_position_s &global_position,
		const home_position_s &home_position,
		const wind_s &wind,
		bool mission_valid,
		bool rtl_active,
		bool require_vtol_approach)
{
	Evaluation evaluation{};

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	MissionRouteCache &cache = _navigator->get_mission_route_cache();
	const bool same_mission = missionMatches(mission);
	evaluation.executor_source_changed = !sourceMatches(mission);
	_waiting_for_inputs = false;

	if (!same_mission) {
		_direction_reversed = false;
		_last_flown_loop_segment = {};
	}

	const PositionYawSetpoint home_destination{home_position.lat, home_position.lon, home_position.alt, home_position.yaw};
	evaluation.home_has_land_approach = cache.hasVtolLandApproachesNearLocation(home_destination,
					    home_position.alt);

	// AUTO_RTL front transitions remain disabled in VTOL attitude control.
	if (!supportsVehicle(vehicle_status) || !mission_valid) {
		return evaluation;
	}

	if (!inputsReady(mission)) {
		_waiting_for_inputs = true;
		return evaluation;
	}

	MissionRouteCache::MissionView mission_view{};

	if (!cache.getMissionView(mission, mission_view)) {
		return evaluation;
	}

	const uint32_t safe_points_id = cache.safePointsId();
	const int safe_point_count = cache.safePointCount();
	MissionViewProvider provider{cache, mission_view, mission.land_index};
	MissionRoutePlanner planner{provider};
	const mission_route::PlannerConfig config = buildPlannerConfig(vehicle_status, home_position,
			require_vtol_approach);
	const mission_route::Position vehicle_position{global_position.lat, global_position.lon, global_position.alt};
	const int32_t mission_index = mission.current_seq >= 0 ? mission.current_seq : 0;
	const mission_route::RoutePlanResult result = planner.planRouteToGoal(vehicle_position, mission_index, config);

	if (!result.success || !result.plan.valid()) {
		PX4_ERR("RTL type 6 plan failed: %s", mission_route::failureReasonString(result.failure_reason));
		return evaluation;
	}

	if (!cache.missionViewStillValid(mission_view)
	    || !cache.safePointsReady()
	    || cache.safePointsId() != safe_points_id
	    || cache.safePointCount() != safe_point_count) {
		return evaluation;
	}

	loiter_point_s goal_land_approach{};

	if (result.plan.selection.safe_point_found) {
		const int32_t safe_point_index = result.plan.selection.safe_point_index;

		if (safe_point_index < 0 || safe_point_index >= UINT8_MAX) {
			PX4_ERR("RTL type 6 safe-point index out of range");
			return evaluation;
		}

		if (vehicle_status.is_vtol
		    && vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
			const land_approaches_s approaches =
				cache.getVtolLandApproachesAtSafePointIndex(safe_point_index, home_position.alt);

			if (approaches.isAnyApproachValid()) {
				goal_land_approach = chooseBestLandingApproach(approaches, wind);
			}
		}
	}

	const bool direction_will_change = _direction_reversed != result.plan.selection.path.direction_reversed;

	if (rtl_active && direction_will_change && vehicle_status.in_transition_to_fw) {
		vehicle_command_s command{};
		command.command = vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
		command.param1 = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
		_navigator->publish_vehicle_command(command);
	}

	_plan = result.plan;
	_goal_land_approach = goal_land_approach;
	_direction_reversed = result.plan.selection.path.direction_reversed;
	_last_flown_loop_segment = result.plan.projection_context.route_projection.segment.validLoop()
				   ? result.plan.projection_context.route_projection.segment
				   : mission_route::Segment{};
	_mission_id = mission.mission_id;
	_mission_count = mission.count;
	_mission_dataman_id = mission.mission_dataman_id;
	_mission_generation = mission_view.generation;
	_safe_points_id = mission.safe_points_id;
	_safe_point_count = static_cast<uint16_t>(safe_point_count);
	_safe_points_dataman_id = mission.safepoint_dataman_id;
	_source_valid = true;

	evaluation.success = true;
	evaluation.goal = convertGoal(result.plan.selection.goal_type);
	mission_route::copyPositionToYawSetpoint(result.plan.selection.goal_position, evaluation.destination);
	evaluation.safe_point_index = result.plan.selection.safe_point_found
				      ? static_cast<uint8_t>(result.plan.selection.safe_point_index)
				      : UINT8_MAX;
	evaluation.any_safe_point_has_land_approach = cache.anySafePointHasVtolLandApproach(home_position.alt);
#else
	(void)mission;
	(void)vehicle_status;
	(void)global_position;
	(void)home_position;
	(void)wind;
	(void)mission_valid;
	(void)rtl_active;
	(void)require_vtol_approach;
#endif

	return evaluation;
}

RtlBase *RtlRouteSafePoint::createExecutor(const mission_s &mission) const
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	RtlBase *executor = new RtlMissionSafePointFollow(_navigator, mission);

	if (executor != nullptr) {
		executor->initialize();
	}

	return executor;
#else
	(void)mission;
	return nullptr;
#endif
}

void RtlRouteSafePoint::configureExecutor(RtlBase &executor, float rtl_alt) const
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	RtlBase::RouteSafePointConfig config {};
	config.plan = _plan;
	config.goal_land_approach = _goal_land_approach;
	config.rtl_alt = rtl_alt;
	executor.configureRouteSafePoint(config);
#else
	(void)executor;
	(void)rtl_alt;
#endif
}

void RtlRouteSafePoint::recordExecutorProgress(const RtlBase &executor, bool preserve)
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0

	if (preserve) {
		_last_flown_loop_segment = executor.lastFlownLoopSegment();
	}

#else
	(void)executor;
	(void)preserve;
#endif
}

void RtlRouteSafePoint::clearExecutorProgress()
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	_last_flown_loop_segment = {};
#endif
}

uint32_t RtlRouteSafePoint::missionGeneration() const
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	return _mission_generation;
#else
	return 0;
#endif
}

mission_route::Segment RtlRouteSafePoint::lastFlownLoopSegment() const
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	return _last_flown_loop_segment;
#else
	return {};
#endif
}

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
mission_route::PlannerConfig RtlRouteSafePoint::buildPlannerConfig(const vehicle_status_s &vehicle_status,
		const home_position_s &home_position,
		bool require_vtol_approach) const
{
	mission_route::PlannerConfig config{};
	const vehicle_local_position_s *local_position = _navigator->get_local_position();

	config.parameters.vehicle_projection_search_dist =
		vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		? _param_mis_mc_seg_dist.get() : _param_mis_fw_seg_dist.get();
	config.parameters.safe_point_projection_search_dist = _param_rtl_rp_seg_dist.get();
	config.parameters.acceptance_radius = _navigator->get_acceptance_radius();
	config.parameters.direct_acceptance_radius = _navigator->get_default_acceptance_radius();
	config.parameters.altitude_acceptance_radius = _navigator->get_altitude_acceptance_radius();
	config.parameters.home_altitude_amsl = home_position.alt;
	config.parameters.u_turn_penalty_m = _param_rtl_fw_uturn_pen.get();
	config.state.is_flying_reverse = _direction_reversed;
	config.state.velocity_valid = local_position != nullptr
				      && PX4_ISFINITE(local_position->vx)
				      && PX4_ISFINITE(local_position->vy);
	config.state.velocity_ne(0) = local_position != nullptr ? local_position->vx : NAN;
	config.state.velocity_ne(1) = local_position != nullptr ? local_position->vy : NAN;
	config.state.is_fixed_wing = vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	config.state.in_transition_to_fw = vehicle_status.in_transition_to_fw;
	config.state.require_vtol_approach = vehicle_status.is_vtol
					     && config.state.is_fixed_wing
					     && require_vtol_approach;
	config.last_flown_loop_segment = _last_flown_loop_segment;
	return config;
}

RtlRouteSafePoint::Goal RtlRouteSafePoint::convertGoal(mission_route::GoalType goal)
{
	switch (goal) {
	case mission_route::GoalType::kSafePoint:
		return Goal::SafePoint;

	case mission_route::GoalType::kMissionLand:
		return Goal::MissionLand;

	case mission_route::GoalType::kMissionTakeoff:
		return Goal::MissionTakeoff;

	case mission_route::GoalType::kNone:
	default:
		return Goal::None;
	}
}

loiter_point_s RtlRouteSafePoint::chooseBestLandingApproach(const land_approaches_s &approaches,
		const wind_s &wind)
{
	if (!approaches.land_location_lat_lon.isAllFinite()) {
		return {};
	}

	const float wind_direction = atan2f(wind.windspeed_east, wind.windspeed_north);
	int8_t best_index{-1};
	float best_angle{INFINITY};

	for (int i = 0; i < approaches.num_approaches_max; ++i) {
		if (!approaches.approaches[i].isValid()) {
			continue;
		}

		const float angle = fabsf(matrix::wrap_pi(
						  get_bearing_to_next_waypoint(approaches.land_location_lat_lon(0),
								  approaches.land_location_lat_lon(1), approaches.approaches[i].lat,
								  approaches.approaches[i].lon) - wind_direction));

		if (angle < best_angle) {
			best_index = i;
			best_angle = angle;
		}
	}

	return best_index >= 0 ? approaches.approaches[best_index] : loiter_point_s{};
}

bool RtlRouteSafePoint::sourceMatches(const mission_s &mission) const
{
	return missionMatches(mission)
	       && mission.safe_points_id == _safe_points_id
	       && mission.safepoint_dataman_id == _safe_points_dataman_id;
}
#endif
