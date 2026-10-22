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
#include "mission_route_cache.h"
#include "mission_route_geofence.h"
#include "mission_route_land_approaches.h"
#include "mission_route_planner.h"
#include "navigator.h"

#include <cmath>
#include <px4_platform_common/log.h>

RtlRouteSafePoint::RtlRouteSafePoint(ModuleParams *parent, Navigator *navigator) :
	ModuleParams(parent),
	_navigator(navigator)
{
}

void RtlRouteSafePoint::reset()
{
	_active_jump_anchor = {};
	_source = {};
	_direction_reversed = false;
	_last_failure = mission_route::FailureReason::kNone;
}

bool RtlRouteSafePoint::supportsVehicle(const vehicle_status_s &vehicle_status) const
{
	return vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	       || vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
}

bool RtlRouteSafePoint::fenceReady() const
{
	const Geofence &geofence = _navigator->get_geofence();
	return geofence.isReadyForPathChecks() && !geofence.isFenceUpdatePending();
}

bool RtlRouteSafePoint::inputsReady(const mission_s &mission) const
{
	const MissionRouteCache &cache = _navigator->get_mission_route_cache();
	return cache.missionItemsReady(mission) && cache.safePointsReady()
	       && cache.safePointsId() == mission.safe_points_id && fenceReady();
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
	       && mission.safepoint_dataman_id == _source.safe_points_dataman_id
	       && mission.geofence_id == _source.geofence_id
	       && mission.fence_dataman_id == _source.fence_dataman_id;
}

bool RtlRouteSafePoint::sourceStillValid(const mission_s &mission) const
{
	// The count and generation also catch in-place edits of the same sources.
	const MissionRouteCache &cache = _navigator->get_mission_route_cache();
	MissionRouteCache::MissionView view{};
	return sourceMatches(mission) && inputsReady(mission)
	       && cache.safePointCount() == _source.safe_point_count
	       && cache.getMissionView(mission, view) && view.generation == _source.mission_generation;
}

bool RtlRouteSafePoint::evaluationPending(const mission_s &mission) const
{
	return mission.count <= MissionRouteCache::kMaxFullMissionCacheSize
	       && (!_navigator->get_mission_route_cache().missionItemsReady(mission) || !fenceReady());
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

	// Only an active Return keeps its flown direction and jump anchor, also while it has no route.
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

	// A replacement mission can reach the cache before its validation completes; RTL plans again later.
	if (!mission_valid || !inputsReady(mission)) {
		return evaluation;
	}

	MissionRouteCache::MissionView mission_view{};

	if (!cache.getMissionView(mission, mission_view)) {
		return evaluation;
	}

	const mission_route::RtlRouteRequest request = buildPlannerRequest(mission, vehicle_status, global_position,
			home_position, require_vtol_approach);
	const mission_route::GeofencePathChecker path_checker{_navigator->get_geofence(), request.vehicle_position};
	const MissionRoutePlanner planner{cache, &path_checker};
	const mission_route::FailureReason failure = planner.planRtlRoute(request, _plan);

	if (failure != mission_route::FailureReason::kNone) {
		// Plans repeat every 2 s, so only log a new reason.
		if (failure != _last_failure) {
			PX4_WARN("RTL type 7 plan failed: %s", mission_route::failureReasonString(failure));
			_last_failure = failure;
		}

		return evaluation;
	}

	_last_failure = mission_route::FailureReason::kNone;

	const bool safe_point_goal = _plan.goal_type == mission_route::GoalType::kSafePoint;
	_goal_land_approach = safe_point_goal
			      ? selectGoalLandApproach(cache, _plan.safe_point_index, vehicle_status, home_position, wind)
			      : loiter_point_s{};
	_direction_reversed = _plan.direction_reversed;
	_active_jump_anchor = _plan.active_jump_anchor;
	_source = {
		.mission_id = mission.mission_id,
		.mission_generation = mission_view.generation,
		.safe_points_id = cache.safePointsId(),
		.geofence_id = mission.geofence_id,
		.mission_land_index = mission.land_index,
		.mission_count = mission.count,
		.safe_point_count = static_cast<uint16_t>(cache.safePointCount()),
		.mission_dataman_id = mission.mission_dataman_id,
		.safe_points_dataman_id = mission.safepoint_dataman_id,
		.fence_dataman_id = mission.fence_dataman_id,
		.valid = true
	};

	evaluation.success = true;
	evaluation.any_safe_point_has_land_approach = mission_route::anySafePointHasVtolLandApproach(cache, home_position.alt);
	return evaluation;
}

void RtlRouteSafePoint::recordExecutorProgress(const mission_route::ActiveJumpAnchor &active_jump_anchor)
{
	_active_jump_anchor = active_jump_anchor;
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
		bool require_vtol_approach) const
{
	mission_route::RtlRouteRequest request{};
	request.vehicle_position = {global_position.lat, global_position.lon, global_position.alt};
	request.mission_index = mission.current_seq >= 0 ? mission.current_seq : 0;
	request.mission_land_index = mission.land_index;
	request.current_route_direction_reversed = _direction_reversed;
	request.active_jump_anchor = _active_jump_anchor;
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
	request.require_vtol_approach = vehicle_status.is_vtol && request.is_fixed_wing && require_vtol_approach;

	const vehicle_local_position_s *local_position = _navigator->get_local_position();

	if (local_position->v_xy_valid) {
		request.velocity_north_m_s = local_position->vx;
		request.velocity_east_m_s = local_position->vy;
	}

	return request;
}

loiter_point_s RtlRouteSafePoint::selectGoalLandApproach(const mission_route::Provider &provider,
		int safe_point_index,
		const vehicle_status_s &vehicle_status,
		const home_position_s &home_position,
		const wind_s &wind)
{
	// Keep the approach even if a later route leg will transition from MC to FW.
	if (!vehicle_status.is_vtol) {
		return {};
	}

	const land_approaches_s approaches = mission_route::getVtolLandApproachesAtSafePointIndex(provider,
					     safe_point_index, home_position.alt);
	return mission_route::chooseBestLandingApproach(approaches, atan2f(wind.windspeed_east, wind.windspeed_north));
}
