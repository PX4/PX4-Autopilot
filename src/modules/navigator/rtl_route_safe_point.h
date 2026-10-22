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
 * @file rtl_route_safe_point.h
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "mission_route_types.h"
#include "navigation.h"

#include <dataman/dataman.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>

class Navigator;

namespace mission_route
{
class Provider;
} // namespace mission_route

/**
 * Route-safe-point planning and source-coherency state for RTL type 7.
 * Only built with the full mission cache.
 */
class RtlRouteSafePoint : public ModuleParams
{
public:
	struct Evaluation {
		bool success{false};
		bool executor_source_changed{false};
		bool home_has_land_approach{false};
		bool any_safe_point_has_land_approach{false};
	};

	RtlRouteSafePoint(ModuleParams *parent, Navigator *navigator);

	void reset();
	bool supportsVehicle(const vehicle_status_s &vehicle_status) const;
	/** Match the uploaded mission independently of safe-point changes, preserving route progress through reloads. */
	bool missionMatches(const mission_s &mission) const;
	/** Check full source identity, cache and fence readiness, mission generation, and safe-point count. */
	bool sourceStillValid(const mission_s &mission) const;
	bool evaluationPending(const mission_s &mission) const;
	/** Whether the cached mission and rally points and the fence describe @p mission. */
	bool inputsReady(const mission_s &mission) const;

	Evaluation evaluate(const mission_s &mission,
			    const vehicle_status_s &vehicle_status,
			    const vehicle_global_position_s &global_position,
			    const home_position_s &home_position,
			    const wind_s &wind,
			    bool mission_valid,
			    bool rtl_active,
			    bool require_vtol_approach);

	/** Keep the follower's flown jump anchor for the replan. */
	void recordExecutorProgress(const mission_route::ActiveJumpAnchor &active_jump_anchor);

	uint32_t missionGeneration() const;
	mission_route::ActiveJumpAnchor activeJumpAnchor() const;

	/** Read only after evaluate() returns success; a failed evaluation may invalidate the plan. */
	const mission_route::RtlRoutePlan &plan() const { return _plan; }
	const loiter_point_s &goalLandApproach() const { return _goal_land_approach; }

private:
	/** Identity of the mission, safe points, and fence used by the last successful plan. */
	struct SourceSnapshot {
		uint32_t mission_id{0};
		uint32_t mission_generation{0};
		uint32_t safe_points_id{0};
		uint32_t geofence_id{0};
		int32_t mission_land_index{-1};
		uint16_t mission_count{0};
		uint16_t safe_point_count{0};
		uint8_t mission_dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_0};
		uint8_t safe_points_dataman_id{DM_KEY_SAFE_POINTS_0};
		uint8_t fence_dataman_id{DM_KEY_FENCE_POINTS_0};
		bool valid{false};
	};

	bool fenceReady() const;
	/** Gather the planner inputs; evaluate() has already cleared the flown state of an inactive estimate. */
	mission_route::RtlRouteRequest buildPlannerRequest(const mission_s &mission,
			const vehicle_status_s &vehicle_status,
			const vehicle_global_position_s &global_position,
			const home_position_s &home_position,
			bool require_vtol_approach) const;
	/** Retain the selected safe point's VTOL approach; the follower uses it if arriving in fixed-wing mode. */
	static loiter_point_s selectGoalLandApproach(const mission_route::Provider &provider,
			int safe_point_index,
			const vehicle_status_s &vehicle_status,
			const home_position_s &home_position,
			const wind_s &wind);
	/** Match uploaded mission, safe-point, and fence sources without requiring readiness. */
	bool sourceMatches(const mission_s &mission) const;

	Navigator *_navigator{nullptr};
	mission_route::ActiveJumpAnchor _active_jump_anchor{};
	SourceSnapshot _source{};
	bool _direction_reversed{false};
	mission_route::FailureReason _last_failure{mission_route::FailureReason::kNone};

	// Kept off the stack: evaluate() runs on the deepest navigator call chain.
	mission_route::RtlRoutePlan _plan{};
	loiter_point_s _goal_land_approach{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MIS_MC_SEG_DIST>) _param_mis_mc_seg_dist,
		(ParamFloat<px4::params::MIS_FW_SEG_DIST>) _param_mis_fw_seg_dist,
		(ParamFloat<px4::params::RTL_RP_SEG_DIST>) _param_rtl_rp_seg_dist,
		(ParamFloat<px4::params::RTL_FW_UTURN_PEN>) _param_rtl_fw_uturn_pen
	)
};
