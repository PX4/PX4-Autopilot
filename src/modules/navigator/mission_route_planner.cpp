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
 * @file mission_route_planner.cpp
 *
 * Route geometry and scoring code for Navigator callers that need to reason about
 * the whole uploaded mission. The planner reads mission and safe-point data through
 * a Provider, projects positions onto route segments, and returns the selected
 * join/goal data. It does not publish setpoints, own dataman state, or decide when
 * a flight mode should use the plan.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_planner.h"

#include <lib/perf/perf_counter.h>

#include <px4_platform_common/log.h>

using namespace mission_route;

/**
 * TODO: FIX: for now this does not work because some variables are init to NAN.
 * Therefore, the large struct lives in .data instead of .bss
 *
 * The ProjectionReferenceBatch (~370 bytes per kMaxSafePointBatch slot, i.e. ~24 KB at a batch
 * size of 64, board-tunable via CONFIG_RTL_SAFE_POINT_BATCH_SIZE) is reused by
 * collectVehicleProjection and selectSafePoint. It is file-static rather than stack-allocated so the
 * navigator task stack does not scale with the batch size. There is no thread concern, we just need to
 * find a way to define it in the .bss
 */
static ProjectionReferenceBatch _projection_reference_batch;

namespace
{

VehicleProjectionResult vehicleProjectionFailure(FailureReason reason)
{
	VehicleProjectionResult result{};
	result.success = false;
	result.failure_reason = reason;
	return result;
}

JoinPlanResult joinPlanFailure(FailureReason reason)
{
	JoinPlanResult result{};
	result.success = false;
	result.failure_reason = reason;
	return result;
}

JoinPlanResult joinPlanSuccess(const JoinPlan &plan)
{
	JoinPlanResult result{};
	result.success = true;
	result.failure_reason = FailureReason::kNone;
	result.plan = plan;
	return result;
}

RoutePlanResult routePlanFailure(FailureReason reason)
{
	RoutePlanResult result{};
	result.success = false;
	result.failure_reason = reason;
	return result;
}

RoutePlanResult routePlanSuccess(const RoutePlan &plan)
{
	RoutePlanResult result{};
	result.success = true;
	result.failure_reason = FailureReason::kNone;
	result.plan = plan;
	return result;
}

} // namespace

VehicleProjectionResult MissionRoutePlanner::collectVehicleProjection(const RoutePlanRequest &request) const
{
	if (!request.config.parameters.validForVehicleProjection()) {
		return vehicleProjectionFailure(FailureReason::kInvalidRequest);
	}

	perf_begin(_collect_vehicle_projection_perf.counter);
	const VehicleProjectionResult result = _projection.collectVehicleProjection(request, _projection_reference_batch);
	perf_end(_collect_vehicle_projection_perf.counter);
	return result;
}

GoalSelection MissionRoutePlanner::selectSafePoint(const ProjectionContext &projection_context,
		const PlannerConfig &config) const
{
	return _goal_selector.selectSafePoint(projection_context, config, _projection_reference_batch).selection;
}

GoalSelection MissionRoutePlanner::selectBestGoal(const ProjectionContext &projection_context,
		const PlannerConfig &config) const
{
	perf_begin(_select_best_goal_perf.counter);
	const GoalSelectionResult result = _goal_selector.selectBestGoal(projection_context, config, _projection_reference_batch);
	perf_end(_select_best_goal_perf.counter);
	return result.selection;
}

bool MissionRoutePlanner::closeToBranchOffSegment(const Position &position, const GoalSelection &selection,
		float acceptance_radius, float altitude_acceptance_radius) const
{
	return _goal_selector.closeToBranchOffSegment(position, selection, acceptance_radius, altitude_acceptance_radius);
}

JoinPlanResult MissionRoutePlanner::planMissionResumeJoin(const RoutePlanRequest &request) const
{
	if (!request.vehicle_position.valid()) {
		return joinPlanFailure(FailureReason::kNoValidGlobalPos);
	}

	const VehicleProjectionResult projection = collectVehicleProjection(request);

	if (!projection.success) {
		return joinPlanFailure(projection.failure_reason);
	}

	JoinPlan plan{};
	plan.projection_context = projection.projection_context;

	plan.path = _goal_selector.findShortestPathAlongRoute(plan.projection_context.route_length,
			plan.projection_context, request.config, PathDirectionMode::kForceNominal);
	plan.join_context = _goal_selector.buildJoinContext(request.vehicle_position, plan.projection_context, plan.path);

	if (!plan.valid()) {
		return joinPlanFailure(FailureReason::kNoValidPath);
	}

	return joinPlanSuccess(plan);
}

RoutePlanResult MissionRoutePlanner::planRouteToGoal(const RoutePlanRequest &request) const
{
	if (!request.config.parameters.validForRouteToGoal()) {
		return routePlanFailure(FailureReason::kInvalidRequest);
	}

	const VehicleProjectionResult projection = collectVehicleProjection(request);

	if (!projection.success) {
		return routePlanFailure(projection.failure_reason);
	}

	RoutePlan plan{};
	plan.projection_context = projection.projection_context;

	// RTL skips DO_JUMP segments. Remaining loop counts from normal mission
	// execution must not force the return path to finish the current loop iteration.
	plan.projection_context.mission_loops_remaining = 0;

	// Find closest safe point, falling back to mission end points if none found
	perf_begin(_select_best_goal_perf.counter);
	const GoalSelectionResult selection = _goal_selector.selectBestGoal(plan.projection_context, request.config,
					      _projection_reference_batch);
	perf_end(_select_best_goal_perf.counter);

	if (!selection.success) {
		return routePlanFailure(selection.failure_reason);
	}

	plan.selection = selection.selection;
	plan.selection.skip_route_to_safe_point =
		_goal_selector.canSkipRouteFollowToSelectedGoal(request.vehicle_position, plan.selection, request.config);

	plan.join_context = _goal_selector.buildJoinContext(request.vehicle_position, plan.projection_context,
			    plan.selection.path);

	if (!plan.valid()) {
		return routePlanFailure(FailureReason::kNoValidPath);
	}

	PX4_DEBUG("RTL plan to %s target=%d rev=%u direct=%u skip_alt=%u branch_off=%d->%d",
		  goalTypeString(plan.selection.goal_type),
		  static_cast<int>(plan.selection.path.first_item_index),
		  static_cast<unsigned>(plan.selection.path.direction_reversed),
		  static_cast<unsigned>(plan.selection.skip_route_to_safe_point),
		  static_cast<unsigned>(plan.join_context.skip_altitude_requirement),
		  static_cast<int>(plan.selection.branch_off_segment.start.idx),
		  static_cast<int>(plan.selection.branch_off_segment.end.idx));

	return routePlanSuccess(plan);
}
