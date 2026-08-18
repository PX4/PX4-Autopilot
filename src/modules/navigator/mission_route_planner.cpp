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

#include "mission_route_goal.h"
#include "mission_route_projection.h"
#include "mission_route_provider.h"

#include <new>

#include <lib/perf/perf_counter.h>

#include <px4_platform_common/log.h>

using namespace mission_route;

namespace
{

/**
 * Shared scratch buffer reused by vehicle projection and safe-point selection, kept off the
 * navigator task stack because it scales with CONFIG_NAVIGATOR_SAFE_POINT_BATCH_SIZE (~380 bytes per slot).
 *
 * The buffer must not be a plain static object: ProjectionReferenceBatch has non-zero member
 * defaults (NAN/-1/NAV_CMD_INVALID), so a static instance would be initialized into
 * .data and store its full byte image in flash, which does not fit on some boards. Zeroed raw
 * storage stays in .bss (RAM reserved at link time, no flash image) and the object is
 * constructed in place on first use. Placement new on static storage cannot fail, so the
 * returned reference is always valid.
 */
ProjectionReferenceBatch &plannerScratchBatch()
{
	alignas(ProjectionReferenceBatch) static uint8_t storage[sizeof(ProjectionReferenceBatch)];
	static ProjectionReferenceBatch *batch = new (storage) ProjectionReferenceBatch{};

	return *batch;
}

bool activeJumpAnchorValidForMission(const ActiveJumpAnchor &anchor, int mission_count)
{
	return anchor.empty()
	       || (anchor.valid() && anchor.start_index < mission_count && anchor.target_index < mission_count);
}

void setVehicleState(PlannerConfig &config, bool current_route_direction_reversed,
		     bool is_fixed_wing, bool in_transition_to_fw, bool require_vtol_approach,
		     float velocity_north_m_s, float velocity_east_m_s)
{
	config.state.is_flying_reverse = current_route_direction_reversed;
	config.state.is_fixed_wing = is_fixed_wing;
	config.state.in_transition_to_fw = in_transition_to_fw;
	config.state.require_vtol_approach = require_vtol_approach;
	config.state.velocity_ne(0) = velocity_north_m_s;
	config.state.velocity_ne(1) = velocity_east_m_s;
	config.state.velocity_valid = config.state.velocity_ne.isAllFinite();
}

PlannerConfig makePlannerConfig(const MissionResumeRequest &request)
{
	PlannerConfig config{};
	config.parameters.vehicle_projection_search_dist = request.projection_search_distance_m;
	config.parameters.acceptance_radius = request.acceptance_radius_m;
	config.parameters.home_altitude_amsl = request.home_altitude_amsl;
	config.parameters.u_turn_penalty_m = request.u_turn_penalty_m;
	config.active_jump_anchor = request.active_jump_anchor;
	setVehicleState(config, request.current_route_direction_reversed, request.is_fixed_wing,
			request.in_transition_to_fw, false, request.velocity_north_m_s, request.velocity_east_m_s);
	return config;
}

PlannerConfig makePlannerConfig(const RouteToGoalRequest &request)
{
	PlannerConfig config{};
	config.parameters.vehicle_projection_search_dist = request.projection_search_distance_m;
	config.parameters.safe_point_projection_search_dist = request.safe_point_projection_search_distance_m;
	config.parameters.acceptance_radius = request.acceptance_radius_m;
	config.parameters.direct_acceptance_radius = request.direct_goal_acceptance_radius_m;
	config.parameters.altitude_acceptance_radius = request.altitude_acceptance_radius_m;
	config.parameters.home_altitude_amsl = request.home_altitude_amsl;
	config.parameters.u_turn_penalty_m = request.u_turn_penalty_m;
	config.active_jump_anchor = request.active_jump_anchor;
	setVehicleState(config, request.current_route_direction_reversed, request.is_fixed_wing,
			request.in_transition_to_fw, request.require_vtol_approach,
			request.velocity_north_m_s, request.velocity_east_m_s);
	return config;
}

ActiveJumpAnchor activeJumpAnchor(const ProjectionContext &projection_context)
{
	const Segment &segment = projection_context.route_projection.segment;

	if (segment.validLoop()) {
		return {segment.start.idx, segment.end.idx};
	}

	return {};
}

MissionResumePlan makeMissionResumePlan(const ProjectionContext &projection_context, const RoutePath &path,
					const JoinContext &join_context)
{
	MissionResumePlan plan{};
	plan.join_position = join_context.projection;
	plan.first_mission_item_index = path.first_item_index;
	plan.direction_reversed = path.direction_reversed;
	plan.use_current_altitude = join_context.use_current_altitude;
	plan.active_jump_anchor = activeJumpAnchor(projection_context);
	return plan;
}

RouteToGoalPlan makeRouteToGoalPlan(const ProjectionContext &projection_context, const JoinContext &join_context,
				    const GoalSelection &selection)
{
	RouteToGoalPlan plan{};
	plan.join_position = join_context.projection;
	plan.first_mission_item_index = selection.path.first_item_index;
	plan.direction_reversed = selection.path.direction_reversed;
	plan.use_current_altitude = join_context.use_current_altitude;
	plan.active_jump_anchor = activeJumpAnchor(projection_context);
	plan.goal_type = selection.goal_type;
	plan.goal_position = selection.goal_position;
	plan.safe_point_index = selection.safe_point_index;
	plan.fly_direct_to_goal = selection.fly_direct_to_goal;

	if (selection.goal_type == GoalType::kSafePoint) {
		plan.branch_off_position = selection.branch_off.projection;
		plan.branch_off_mission_item_index = selection.branchOffIndex();
	}

	return plan;
}

} // namespace

MissionRoutePlanner::MissionRoutePlanner(const Provider &provider) :
	_provider(provider)
{
}

MissionRoutePlanner::~MissionRoutePlanner()
{
	perf_free(_collect_vehicle_projection_perf);
	perf_free(_select_best_goal_perf);
}

FailureReason MissionRoutePlanner::planMissionResumeJoin(const MissionResumeRequest &request,
		MissionResumePlan &plan) const
{
	plan = {};

	if (!request.vehicle_position.valid()) {
		return FailureReason::kNoValidGlobalPos;
	}

	const PlannerConfig config = makePlannerConfig(request);

	if (!config.parameters.validForVehicleProjection()
	    || !activeJumpAnchorValidForMission(request.active_jump_anchor, _provider.missionCount())) {
		return FailureReason::kInvalidRequest;
	}

	ProjectionReferenceBatch &reference_batch = plannerScratchBatch();
	MissionRouteProjection projection{_provider};
	MissionRouteGoalSelector goal_selector{_provider, projection};

	ProjectionContext projection_context{};
	perf_begin(_collect_vehicle_projection_perf);
	const FailureReason projection_status = projection.collectVehicleProjection(request.vehicle_position,
						request.mission_index, config, reference_batch, projection_context);
	perf_end(_collect_vehicle_projection_perf);

	if (projection_status != FailureReason::kNone) {
		return projection_status;
	}

	const RoutePath path = goal_selector.findShortestPathAlongRoute(projection_context.route_length,
			       projection_context, config, PathDirectionMode::kForceNominal);
	const JoinContext join_context = goal_selector.buildJoinContext(request.vehicle_position, projection_context, path);
	const MissionResumePlan candidate = makeMissionResumePlan(projection_context, path, join_context);

	if (!projection_context.valid() || !path.valid() || !join_context.valid() || !candidate.valid()) {
		return FailureReason::kNoValidPath;
	}

	plan = candidate;
	return FailureReason::kNone;
}

FailureReason MissionRoutePlanner::planRouteToGoal(const RouteToGoalRequest &request,
		RouteToGoalPlan &plan) const
{
	plan = {};
	const PlannerConfig config = makePlannerConfig(request);

	// Preserve the existing failure precedence: invalid route settings are rejected before vehicle projection.
	if (!config.parameters.validForRouteToGoal()
	    || !activeJumpAnchorValidForMission(request.active_jump_anchor, _provider.missionCount())) {
		return FailureReason::kInvalidRequest;
	}

	ProjectionReferenceBatch &reference_batch = plannerScratchBatch();
	MissionRouteProjection projection{_provider};
	MissionRouteGoalSelector goal_selector{_provider, projection};

	ProjectionContext projection_context{};
	perf_begin(_collect_vehicle_projection_perf);
	const FailureReason projection_status = projection.collectVehicleProjection(request.vehicle_position,
						request.mission_index, config, reference_batch, projection_context);
	perf_end(_collect_vehicle_projection_perf);

	if (projection_status != FailureReason::kNone) {
		return projection_status;
	}

	// RTL skips DO_JUMP segments. Remaining loop counts from normal mission
	// execution must not force the return path to finish the current loop iteration.
	projection_context.mission_loops_remaining = 0;

	// Find closest safe point, falling back to mission end points if none found
	GoalSelection selection{};
	perf_begin(_select_best_goal_perf);
	const FailureReason selection_status = goal_selector.selectBestGoal(projection_context, config,
					       reference_batch, selection);
	perf_end(_select_best_goal_perf);

	if (selection_status != FailureReason::kNone) {
		return selection_status;
	}

	selection.fly_direct_to_goal =
		goal_selector.canSkipRouteFollowToSelectedGoal(request.vehicle_position, selection, config);

	const JoinContext join_context = goal_selector.buildJoinContext(request.vehicle_position, projection_context,
					 selection.path);
	const RouteToGoalPlan candidate = makeRouteToGoalPlan(projection_context, join_context, selection);

	if (!projection_context.valid() || !join_context.valid() || !selection.valid() || !candidate.valid()) {
		return FailureReason::kNoValidPath;
	}

	PX4_DEBUG("Route plan to %s target=%d rev=%u direct=%u skip_alt=%u branch_off=%d->%d",
		  goalTypeString(candidate.goal_type),
		  static_cast<int>(candidate.first_mission_item_index),
		  static_cast<unsigned>(candidate.direction_reversed),
		  static_cast<unsigned>(candidate.fly_direct_to_goal),
		  static_cast<unsigned>(candidate.use_current_altitude),
		  static_cast<int>(selection.branch_off.segment.start.idx),
		  static_cast<int>(selection.branch_off.segment.end.idx));

	plan = candidate;
	return FailureReason::kNone;
}
