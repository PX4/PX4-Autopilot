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
 * @file mission_route_planner.h
 *
 * Mission-route planner interface for Navigator features that need the uploaded
 * mission as route geometry. The planner projects positions onto the mission path,
 * ranks candidate paths, and returns plain data for the caller to execute.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "mission_route_goal.h"
#include "mission_route_projection.h"
#include "mission_route_provider.h"
#include "mission_route_types.h"

#include <stdint.h>

#include <lib/perf/perf_counter.h>

class MissionRoutePlanner
{
public:
	explicit MissionRoutePlanner(const mission_route::Provider &provider);
	~MissionRoutePlanner();
	MissionRoutePlanner(const MissionRoutePlanner &) = delete;
	MissionRoutePlanner &operator=(const MissionRoutePlanner &) = delete;
	MissionRoutePlanner(MissionRoutePlanner &&) = delete;
	MissionRoutePlanner &operator=(MissionRoutePlanner &&) = delete;

	/** @brief Project the vehicle onto the mission route and choose the continuity-preserving branch-in candidate. */
	mission_route::VehicleProjectionResult collectVehicleProjection(const mission_route::Position &vehicle_position,
			int32_t mission_index, const mission_route::PlannerConfig &config) const;

	/** @brief Evaluate all valid safe points and choose the best route-follow return target. */
	mission_route::GoalSelection selectSafePoint(const mission_route::ProjectionContext &projection_context,
			const mission_route::PlannerConfig &config) const;
	/** @brief Choose the RTL goal, preferring a safe point and falling back to a mission endpoint when needed. */
	mission_route::GoalSelection selectBestGoal(const mission_route::ProjectionContext &projection_context,
			const mission_route::PlannerConfig &config) const;
	/** @brief Build the Mission-mode smart-rejoin plan back to the mission end using the nominal route direction. */
	mission_route::JoinPlanResult planMissionResumeJoin(const mission_route::Position &vehicle_position,
			int32_t mission_index, const mission_route::PlannerConfig &config) const;
	/** @brief Build the full RTL plan: vehicle projection, join context, and selected goal. */
	mission_route::RoutePlanResult planRouteToGoal(const mission_route::Position &vehicle_position,
			int32_t mission_index, const mission_route::PlannerConfig &config) const;
	/** @brief Check whether the vehicle is still close enough to the selected branch-off leg to skip route following. */
	bool closeToBranchOffSegment(const mission_route::Position &position, const mission_route::GoalSelection &selection,
				     float acceptance_radius, float altitude_acceptance_radius) const;

private:
	mission_route::MissionRouteProjection _projection;
	mission_route::MissionRouteGoalSelector _goal_selector;
	mission_route::ProjectionReferenceBatch &_reference_batch;
	perf_counter_t _collect_vehicle_projection_perf{perf_alloc(PC_ELAPSED, "rtl_route_collect_vehicle_proj")};
	perf_counter_t _select_best_goal_perf{perf_alloc(PC_ELAPSED, "rtl_route_select_best_goal")};
};
