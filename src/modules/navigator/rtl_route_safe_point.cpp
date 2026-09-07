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

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0

#include "mission_route_cache.h"
#include "mission_route_land_approaches.h"
#include "mission_route_planner.h"
#include "navigator.h"
#include "rtl_base.h"
#include "rtl_mission_safe_point_follow.h"

#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>

namespace
{
class MissionViewProvider final : public mission_route::Provider
{
public:
	MissionViewProvider(const MissionRouteCache &cache, const MissionRouteCache::MissionView &view) :
		_cache(cache), _view(view)
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

private:
	const MissionRouteCache &_cache;
	MissionRouteCache::MissionView _view;
};
} // namespace

RtlRouteSafePoint::RtlRouteSafePoint(ModuleParams *parent, Navigator *navigator) :
	ModuleParams(parent),
	_navigator(navigator)
{
}

void RtlRouteSafePoint::reset()
{
	_plan = {};
	_goal_land_approach = {};
	_active_jump_anchor = {};
	_source = {};
	_vtol_state_on_mission_upload = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED;
	_direction_reversed = false;
	_waiting_for_inputs = false;
}

bool RtlRouteSafePoint::supportsVehicle(const vehicle_status_s &vehicle_status) const
{
	// AUTO_RTL front transitions remain disabled in VTOL attitude control.
	return !vehicle_status.is_vtol;
}

bool RtlRouteSafePoint::inputsReady(const mission_s &mission) const
{
	const MissionRouteCache &cache = _navigator->get_mission_route_cache();
	return cache.missionItemsReady(mission) && cache.safePointsReady()
	       && cache.safePointsId() == mission.safe_points_id;
}

bool RtlRouteSafePoint::missionMatches(const mission_s &mission) const
{
	return _source.valid
	       && mission.mission_id == _source.mission_id
	       && mission.count == _source.mission_count
	       && mission.mission_dataman_id == _source.mission_dataman_id
	       && mission.land_index == _source.mission_land_index;
}

bool RtlRouteSafePoint::sourceMatches(const mission_s &mission) const
{
	return missionMatches(mission)
	       && mission.safe_points_id == _source.safe_points_id
	       && mission.safepoint_dataman_id == _source.safe_points_dataman_id;
}

bool RtlRouteSafePoint::sourceStillValid(const mission_s &mission) const
{
	if (!sourceMatches(mission)) {
		return false;
	}

	const MissionRouteCache &cache = _navigator->get_mission_route_cache();

	if (!cache.safePointsReady()
	    || cache.safePointsId() != _source.safe_points_id
	    || cache.safePointCount() != _source.safe_point_count) {
		return false;
	}

	MissionRouteCache::MissionView view{};
	return cache.getMissionView(mission, view) && view.generation == _source.mission_generation;
}

bool RtlRouteSafePoint::evaluationPending(const mission_s &mission) const
{
	return mission.count <= MissionRouteCache::kMaxFullMissionCacheSize
	       && !_navigator->get_mission_route_cache().missionItemsReady(mission);
}

bool RtlRouteSafePoint::retryReady(const mission_s &mission) const
{
	return _waiting_for_inputs && inputsReady(mission);
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
	const MissionRouteCache &cache = _navigator->get_mission_route_cache();
	evaluation.executor_source_changed = !sourceMatches(mission);
	_waiting_for_inputs = false;

	// Inactive estimates are hypothetical: discard their direction and jump
	// anchor before the next estimate or initial activation, even if inputs
	// are pending. Preserve actual RTL progress through temporary fallback.
	if (!missionMatches(mission) || !rtl_active) {
		_direction_reversed = false;
		_active_jump_anchor = {};
	}

	const PositionYawSetpoint home_destination{home_position.lat, home_position.lon, home_position.alt, home_position.yaw};
	evaluation.home_has_land_approach = mission_route::hasVtolLandApproachesNearLocation(cache, home_destination,
					    home_position.alt);

	if (!supportsVehicle(vehicle_status)) {
		return evaluation;
	}

	if (!mission_valid) {
		// A replacement mission can reach the cache before feasibility validation
		// completes. RTL retries once both validation and the cache are ready.
		_waiting_for_inputs = mission.count > 0 && mission.count <= MissionRouteCache::kMaxFullMissionCacheSize;
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
	const MissionViewProvider provider{cache, mission_view};
	const MissionRoutePlanner planner{provider};
	const mission_route::RtlRouteRequest request = buildPlannerRequest(mission, vehicle_status, global_position,
			home_position, rtl_active, require_vtol_approach);
	mission_route::RtlRoutePlan plan{};
	const mission_route::FailureReason failure = planner.planRtlRoute(request, plan);

	if (failure != mission_route::FailureReason::kNone || !plan.valid()) {
		PX4_ERR("RTL type 7 plan failed: %s", mission_route::failureReasonString(failure));
		return evaluation;
	}

	// Only accept a plan whose cache inputs did not change while planning.
	if (!cache.missionViewStillValid(mission_view)
	    || !cache.safePointsReady()
	    || cache.safePointsId() != safe_points_id
	    || cache.safePointCount() != safe_point_count) {
		return evaluation;
	}

	const bool safe_point_goal = plan.goal_type == mission_route::GoalType::kSafePoint;

	if (safe_point_goal && (plan.safe_point_index < 0 || plan.safe_point_index >= UINT8_MAX)) {
		PX4_ERR("RTL type 7 safe-point index out of range");
		return evaluation;
	}

	if (rtl_active && _direction_reversed != plan.direction_reversed && vehicle_status.in_transition_to_fw) {
		// The route direction flips mid front transition: go back to MC first.
		vehicle_command_s command{};
		command.command = vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
		command.param1 = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
		_navigator->publish_vehicle_command(command);
	}

	_plan = plan;
	_goal_land_approach = safe_point_goal
			      ? selectGoalLandApproach(provider, plan, vehicle_status, home_position, wind)
			      : loiter_point_s{};
	_direction_reversed = plan.direction_reversed;
	_active_jump_anchor = plan.active_jump_anchor;
	_vtol_state_on_mission_upload = request.vtol_state_on_mission_upload;
	_source = {
		.mission_id = mission.mission_id,
		.mission_generation = mission_view.generation,
		.safe_points_id = safe_points_id,
		.mission_land_index = mission.land_index,
		.mission_count = mission.count,
		.safe_point_count = static_cast<uint16_t>(safe_point_count),
		.mission_dataman_id = mission.mission_dataman_id,
		.safe_points_dataman_id = mission.safepoint_dataman_id,
		.valid = true
	};

	evaluation.success = true;
	evaluation.goal = convertGoal(plan.goal_type);
	mission_route::copyPositionToYawSetpoint(plan.goal_position, evaluation.destination);
	evaluation.safe_point_index = safe_point_goal ? static_cast<uint8_t>(plan.safe_point_index) : UINT8_MAX;
	evaluation.any_safe_point_has_land_approach = mission_route::anySafePointHasVtolLandApproach(provider, home_position.alt);
	return evaluation;
}

RtlBase *RtlRouteSafePoint::createExecutor(const mission_s &mission) const
{
	RtlBase *executor = new RtlMissionSafePointFollow(_navigator, mission);

	if (executor != nullptr) {
		executor->initialize();
	}

	return executor;
}

void RtlRouteSafePoint::configureExecutor(RtlBase &executor, float rtl_alt) const
{
	RtlBase::RouteSafePointConfig config {};
	config.plan = _plan;
	config.goal_land_approach = _goal_land_approach;
	config.rtl_alt = rtl_alt;
	config.vtol_state_on_mission_upload = _vtol_state_on_mission_upload;
	executor.configureRouteSafePoint(config);
}

void RtlRouteSafePoint::recordExecutorProgress(const RtlBase &executor, bool preserve)
{
	if (preserve) {
		_active_jump_anchor = executor.activeJumpAnchor();
	}
}

void RtlRouteSafePoint::clearExecutorProgress()
{
	_active_jump_anchor = {};
}

uint32_t RtlRouteSafePoint::missionGeneration() const
{
	return _source.mission_generation;
}

mission_route::ActiveJumpAnchor RtlRouteSafePoint::activeJumpAnchor() const
{
	return _active_jump_anchor;
}

mission_route::RtlRouteRequest RtlRouteSafePoint::buildPlannerRequest(const mission_s &mission,
		const vehicle_status_s &vehicle_status,
		const vehicle_global_position_s &global_position,
		const home_position_s &home_position,
		bool rtl_active,
		bool require_vtol_approach) const
{
	mission_route::RtlRouteRequest request{};
	request.vehicle_position = {global_position.lat, global_position.lon, global_position.alt};
	request.mission_index = mission.current_seq >= 0 ? mission.current_seq : 0;
	request.mission_land_index = mission.land_index;
	// Only an active return carries flown state; inactive estimates start from the nominal mission.
	request.current_route_direction_reversed = rtl_active && _direction_reversed;
	request.active_jump_anchor = rtl_active ? _active_jump_anchor : mission_route::ActiveJumpAnchor{};
	request.projection_search_distance_m =
		vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		? _param_mis_mc_seg_dist.get() : _param_mis_fw_seg_dist.get();
	request.safe_point_projection_search_distance_m = _param_rtl_rp_seg_dist.get();
	request.acceptance_radius_m = _navigator->get_acceptance_radius();
	request.direct_goal_acceptance_radius_m = _navigator->get_default_acceptance_radius();
	request.altitude_acceptance_radius_m = _navigator->get_altitude_acceptance_radius();
	request.home_altitude_amsl = home_position.valid_alt ? home_position.alt : NAN;
	request.fw_u_turn_penalty_m = _param_rtl_fw_uturn_pen.get();
	request.is_fixed_wing = vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	request.in_transition_to_fw = vehicle_status.in_transition_to_fw;
	request.is_vtol = vehicle_status.is_vtol;
	request.vtol_state_on_mission_upload = _navigator->getMissionVtolStateOnUpload();
	request.require_vtol_approach = vehicle_status.is_vtol && request.is_fixed_wing && require_vtol_approach;

	const vehicle_local_position_s *local_position = _navigator->get_local_position();

	if (local_position != nullptr && local_position->v_xy_valid) {
		request.velocity_north_m_s = local_position->vx;
		request.velocity_east_m_s = local_position->vy;
	}

	return request;
}

loiter_point_s RtlRouteSafePoint::selectGoalLandApproach(const mission_route::Provider &provider,
		const mission_route::RtlRoutePlan &plan,
		const vehicle_status_s &vehicle_status,
		const home_position_s &home_position,
		const wind_s &wind)
{
	// Only a VTOL in fixed-wing flight lands through an approach loiter.
	if (!vehicle_status.is_vtol || vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		return {};
	}

	const land_approaches_s approaches = mission_route::getVtolLandApproachesAtSafePointIndex(provider,
					     plan.safe_point_index, home_position.alt);

	return approaches.isAnyApproachValid() ? chooseBestLandingApproach(approaches, wind) : loiter_point_s{};
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
		const loiter_point_s &approach = approaches.approaches[i];

		if (!approach.isValid()) {
			continue;
		}

		const float bearing = get_bearing_to_next_waypoint(approaches.land_location_lat_lon(0),
				      approaches.land_location_lat_lon(1), approach.lat, approach.lon);
		const float angle = fabsf(matrix::wrap_pi(bearing - wind_direction));

		if (angle < best_angle) {
			best_index = i;
			best_angle = angle;
		}
	}

	return best_index >= 0 ? approaches.approaches[best_index] : loiter_point_s{};
}

#else // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE == 0

// Route-following RTL is compiled out: every query reports the feature as unavailable.
RtlRouteSafePoint::RtlRouteSafePoint(ModuleParams *, Navigator *) {}
void RtlRouteSafePoint::reset() {}
bool RtlRouteSafePoint::supportsVehicle(const vehicle_status_s &) const { return false; }
bool RtlRouteSafePoint::inputsReady(const mission_s &) const { return false; }
bool RtlRouteSafePoint::missionMatches(const mission_s &) const { return false; }
bool RtlRouteSafePoint::sourceStillValid(const mission_s &) const { return false; }
bool RtlRouteSafePoint::evaluationPending(const mission_s &) const { return false; }
bool RtlRouteSafePoint::retryReady(const mission_s &) const { return false; }
RtlRouteSafePoint::Evaluation RtlRouteSafePoint::evaluate(const mission_s &, const vehicle_status_s &,
		const vehicle_global_position_s &, const home_position_s &, const wind_s &, bool, bool, bool) { return {}; }
RtlBase *RtlRouteSafePoint::createExecutor(const mission_s &) const { return nullptr; }
void RtlRouteSafePoint::configureExecutor(RtlBase &, float) const {}
void RtlRouteSafePoint::recordExecutorProgress(const RtlBase &, bool) {}
void RtlRouteSafePoint::clearExecutorProgress() {}
uint32_t RtlRouteSafePoint::missionGeneration() const { return 0; }
mission_route::ActiveJumpAnchor RtlRouteSafePoint::activeJumpAnchor() const { return {}; }

#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
