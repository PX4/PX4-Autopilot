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

#include <px4_platform_common/module_params.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>

class Navigator;
class RtlBase;

/** Optional route-safe-point planning and source-coherency state for RTL. */
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
		PositionYawSetpoint destination{NAN, NAN, NAN, NAN};
		uint8_t safe_point_index{UINT8_MAX};
	};

	RtlRouteSafePoint(ModuleParams *parent, Navigator *navigator);

	static constexpr bool available()
	{
		return CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0;
	}

	void reset();
	bool supportsVehicle(const vehicle_status_s &vehicle_status) const;
	bool inputsReady(const mission_s &mission) const;
	bool missionMatches(const mission_s &mission) const;
	bool sourceStillValid(const mission_s &mission) const;
	bool evaluationPending(const mission_s &mission) const;
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
	void recordExecutorProgress(const RtlBase &executor, bool preserve);
	void clearExecutorProgress();

	uint32_t missionGeneration() const;
	mission_route::Segment lastFlownLoopSegment() const;

private:
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	mission_route::PlannerConfig buildPlannerConfig(const vehicle_status_s &vehicle_status,
			const home_position_s &home_position,
			bool require_vtol_approach) const;
	static Goal convertGoal(mission_route::GoalType goal);
	static loiter_point_s chooseBestLandingApproach(const land_approaches_s &approaches,
			const wind_s &wind);
	bool sourceMatches(const mission_s &mission) const;

	Navigator *_navigator{nullptr};
	mission_route::RoutePlan _plan{};
	loiter_point_s _goal_land_approach{};
	mission_route::Segment _last_flown_loop_segment{};
	uint32_t _mission_id{0};
	uint32_t _mission_generation{0};
	uint32_t _safe_points_id{0};
	uint16_t _mission_count{0};
	uint16_t _safe_point_count{0};
	uint8_t _mission_dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_0};
	uint8_t _safe_points_dataman_id{DM_KEY_SAFE_POINTS_0};
	bool _source_valid{false};
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
