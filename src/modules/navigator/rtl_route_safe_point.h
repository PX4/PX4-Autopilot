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
class RtlBase;

namespace mission_route
{
class Provider;
} // namespace mission_route

/**
 * Optional route-safe-point planning and source-coherency state for RTL.
 * With the full mission cache compiled out, every method reports the feature as unavailable.
 */
class RtlRouteSafePoint
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	: public ModuleParams
#endif
{
public:
	enum class Goal {
		None,
		SafePoint,
		MissionLand,
		MissionTakeoff,
	};

	struct Evaluation {
		bool success{false};
		bool executor_source_changed{false};
		bool home_has_land_approach{false};
		bool any_safe_point_has_land_approach{false};
		Goal goal{Goal::None};
		PositionYawSetpoint destination{static_cast<double>(NAN), static_cast<double>(NAN), NAN, NAN};
		uint8_t safe_point_index{UINT8_MAX};
	};

	RtlRouteSafePoint(ModuleParams *parent, Navigator *navigator);

	void reset();
	bool supportsVehicle(const vehicle_status_s &vehicle_status) const;
	bool inputsReady(const mission_s &mission) const;
	/** Match the uploaded mission independently of safe-point changes, preserving route progress through reloads. */
	bool missionMatches(const mission_s &mission) const;
	/** Check the full source identity, cache readiness, mission generation, and safe-point count. */
	bool sourceStillValid(const mission_s &mission) const;
	bool evaluationPending(const mission_s &mission) const;
	/** Retry a deferred evaluation once cache inputs are ready; the caller still checks mission validity. */
	bool retryReady(const mission_s &mission) const;

	Evaluation evaluate(const mission_s &mission,
			    const vehicle_status_s &vehicle_status,
			    const vehicle_global_position_s &global_position,
			    const home_position_s &home_position,
			    const wind_s &wind,
			    bool mission_valid,
			    bool rtl_active,
			    bool require_vtol_approach);

	RtlBase *createExecutor(const mission_s &mission) const;
	void configureExecutor(RtlBase &executor, float rtl_alt) const;
	/** Capture flown progress when requested; otherwise leave the newly planned anchor intact. */
	void recordExecutorProgress(const RtlBase &executor, bool preserve);
	void clearExecutorProgress();

	uint32_t missionGeneration() const;
	mission_route::ActiveJumpAnchor activeJumpAnchor() const;

private:
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	/** Identity of the cache contents used by the last successful plan. */
	struct SourceSnapshot {
		uint32_t mission_id{0};
		uint32_t mission_generation{0};
		uint32_t safe_points_id{0};
		int32_t mission_land_index{-1};
		uint16_t mission_count{0};
		uint16_t safe_point_count{0};
		uint8_t mission_dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_0};
		uint8_t safe_points_dataman_id{DM_KEY_SAFE_POINTS_0};
		bool valid{false};
	};

	/** Gather every planner input; only an active return contributes its flown direction and jump anchor. */
	mission_route::RtlRouteRequest buildPlannerRequest(const mission_s &mission,
			const vehicle_status_s &vehicle_status,
			const vehicle_global_position_s &global_position,
			const home_position_s &home_position,
			bool rtl_active,
			bool require_vtol_approach) const;
	/** Pick the wind-aligned VTOL approach loiter at the selected safe point; empty unless a fixed-wing VTOL. */
	static loiter_point_s selectGoalLandApproach(const mission_route::Provider &provider,
			const mission_route::RtlRoutePlan &plan,
			const vehicle_status_s &vehicle_status,
			const home_position_s &home_position,
			const wind_s &wind);
	static Goal convertGoal(mission_route::GoalType goal);
	static loiter_point_s chooseBestLandingApproach(const land_approaches_s &approaches,
			const wind_s &wind);
	/** Match both uploaded sources without requiring their caches to be ready. */
	bool sourceMatches(const mission_s &mission) const;

	Navigator *_navigator{nullptr};
	mission_route::RtlRoutePlan _plan{};
	loiter_point_s _goal_land_approach{};
	mission_route::ActiveJumpAnchor _active_jump_anchor{};
	SourceSnapshot _source{};
	uint8_t _vtol_state_on_mission_upload{vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED};
	bool _direction_reversed{false};
	bool _waiting_for_inputs{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MIS_MC_SEG_DIST>) _param_mis_mc_seg_dist,
		(ParamFloat<px4::params::MIS_FW_SEG_DIST>) _param_mis_fw_seg_dist,
		(ParamFloat<px4::params::RTL_RP_SEG_DIST>) _param_rtl_rp_seg_dist,
		(ParamFloat<px4::params::RTL_FW_UTURN_PEN>) _param_rtl_fw_uturn_pen
	)
#endif
};
