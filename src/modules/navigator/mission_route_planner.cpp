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
 * Route planning policy and orchestration for Navigator callers that need to reason about
 * the whole uploaded mission. The planner reads mission and safe-point data through
 * a Provider, projects positions onto route segments, and returns the selected
 * join/goal data. It does not publish setpoints, own dataman state, or decide when
 * a flight mode should use the plan.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_planner.h"

#include "mission_route_projection.h"
#include "mission_route_provider.h"

#include <float.h>
#include <new>

#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include <px4_platform_common/log.h>

using namespace mission_route;

namespace
{

enum class PathDirectionMode : uint8_t {
	kAuto = 0,
	kForceNominal,
	kForceReverse
};

enum class RouteGoalSegmentType : uint8_t {
	kOutsideActiveLoopJump = 0,
	kOnActiveLoopJump
};

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

void loadSafePointBatch(const Provider &provider,
			const PlannerConfig &config,
			int &safe_point_cursor,
			ProjectionReferenceBatch &batch)
{
	batch.count = 0;

	const int safe_point_item_count = provider.safePointCount();

	while (safe_point_cursor < safe_point_item_count && batch.count < kMaxSafePointBatch) {
		const int safe_point_index = safe_point_cursor++;
		mission_item_s safe_point_item{};

		if (!provider.loadSafePointItem(safe_point_index, safe_point_item)) {
			PX4_WARN("Route safe point %d read failed", safe_point_index);
			continue;
		}

		Position safe_point_position{};

		if (!extractSafePointPosition(safe_point_item, config.parameters.home_altitude_amsl,
					      safe_point_position)) {
			PX4_DEBUG("Route safe point %d skipped, invalid position or frame", safe_point_index);
			continue;
		}

		if (config.state.require_vtol_approach
		    && !provider.hasVtolLandApproachesAtSafePointIndex(safe_point_index,
				    config.parameters.home_altitude_amsl)) {
			PX4_DEBUG("Route safe point %d skipped, no VTOL approach", safe_point_index);
			continue;
		}

		batch.items[batch.count].position = safe_point_position;
		batch.items[batch.count].source_index = safe_point_index;
		batch.count++;
	}
}

bool mustFlyReverse(float goal_route_along, float projection_route_along,
		    PathDirectionMode direction_mode)
{
	// The forced modes exist for the mission endpoints, where goal and projection can sit
	// on the same item and the along-route comparison is ambiguous.
	switch (direction_mode) {
	case PathDirectionMode::kForceNominal:
		return false;

	case PathDirectionMode::kForceReverse:
		return true;

	case PathDirectionMode::kAuto:
	default:
		return goal_route_along < projection_route_along;
	}
}

matrix::Vector2f computeDesiredCourseVector(const ProjectionContext &projection_context,
		float acceptance_radius,
		bool will_fly_reverse)
{
	static constexpr float kSmallLengthM = 5.f;
	const float far_from_route_m = math::max(acceptance_radius, kSmallLengthM);
	matrix::Vector2f desired_course_vec{};

	if (projection_context.route_projection.dist.segment_length < kSmallLengthM
	    || projection_context.route_projection.dist.xtrack > far_from_route_m) {
		get_vector_to_next_waypoint(projection_context.vehicle_position.lat, projection_context.vehicle_position.lon,
					    projection_context.route_projection.projection.lat,
					    projection_context.route_projection.projection.lon,
					    &desired_course_vec(0), &desired_course_vec(1));

	} else {
		const Position &segment_start = will_fly_reverse ? projection_context.route_projection.segment_positions.end
						: projection_context.route_projection.segment_positions.start;
		const Position &segment_end = will_fly_reverse ? projection_context.route_projection.segment_positions.start
					      : projection_context.route_projection.segment_positions.end;

		get_vector_to_next_waypoint(segment_start.lat, segment_start.lon,
					    segment_end.lat, segment_end.lon,
					    &desired_course_vec(0), &desired_course_vec(1));
	}

	return desired_course_vec;
}

bool uTurnRequired(const ProjectionContext &projection_context,
		   const PlannerConfig &config,
		   bool will_fly_reverse)
{
	if (!(config.state.is_fixed_wing || config.state.in_transition_to_fw)) {
		return false;
	}

	const VehicleStateContext &vehicle_state = projection_context.vehicle_state;

	if (!vehicle_state.velocity_valid || !vehicle_state.velocity_ne.isAllFinite()) {
		return false; // If no velocity, no need for a u-turn
	}

	const matrix::Vector2f desired_course = computeDesiredCourseVector(projection_context,
						config.parameters.acceptance_radius, will_fly_reverse);

	if (!desired_course.isAllFinite()) {
		return false; // Can't execute a proper u-turn without a desired course
	}

	if (vehicle_state.velocity_ne.norm_squared() <= FLT_EPSILON || desired_course.norm_squared() <= FLT_EPSILON) {
		return false;
	}

	// Velocity opposing the desired course requires a u-turn.
	return vehicle_state.velocity_ne.dot(desired_course) < 0.f;
}

RoutePath solveShortestRoutePath(float goal_route_along,
				 const ProjectionContext &projection_context, const PlannerConfig &config,
				 PathDirectionMode direction_mode)
{
	RoutePath path{};

	const bool will_fly_reverse = mustFlyReverse(goal_route_along,
				      projection_context.route_projection.dist.route_along,
				      direction_mode);
	const float abs_distance_projection_to_goal =
		fabsf(goal_route_along - projection_context.route_projection.dist.route_along);

	path.direction_reversed = will_fly_reverse;
	path.u_turn_required = uTurnRequired(projection_context, config, will_fly_reverse);
	path.total_cost_m = abs_distance_projection_to_goal + (path.u_turn_required ? config.parameters.u_turn_penalty_m : 0.f);

	// Choose which segment endpoint becomes the first target for route following.
	bool choose_item_start = false;
	const bool direction_change = (projection_context.vehicle_state.is_flying_reverse != will_fly_reverse);

	if (!direction_change
	    && isIndexInProjectionSegment(projection_context.route_projection.segment,
					  projection_context.mission_index,
					  projection_context.vehicle_state.is_flying_reverse)) {

		// E.g. seg [2,4] where 3 is a front transition, if we're targeting the FT (3),
		// chose item start (2) to ensure the FT is not skipped
		choose_item_start = projection_context.mission_index < projection_context.route_projection.segment.end.idx;

	} else {
		// Direction change or off-segment: choose start when reversing, end when nominal.
		choose_item_start = will_fly_reverse;
	}

	if (choose_item_start) {
		path.first_item_index = projection_context.route_projection.segment.start.idx;
		path.first_item_cmd = projection_context.route_projection.segment.start.nav_cmd;
		path.first_item_position = projection_context.route_projection.segment_positions.start;

	} else {
		path.first_item_index = projection_context.route_projection.segment.end.idx;
		path.first_item_cmd = projection_context.route_projection.segment.end.nav_cmd;
		path.first_item_position = projection_context.route_projection.segment_positions.end;
	}

	return path;
}

RoutePath solveShortestRoutePathFromActiveLoop(float goal_route_along,
		const ProjectionContext &projection_context, const PlannerConfig &config,
		PathDirectionMode direction_mode)
{
	// The u-turn check uses the direction flown on the loop jump itself, not the direction
	// after leaving it.

	// Path A: complete the remaining loop distance, then continue to the goal.
	const float dist_jump_remaining = fabsf(projection_context.route_projection.dist.segment_length
						- projection_context.route_projection.dist.segment_along);
	const bool path_a_u_turn = uTurnRequired(projection_context, config, /* will_fly_reverse */ false);
	const float path_a_cost = dist_jump_remaining
				  + fabsf(goal_route_along - projection_context.loop_context.along.end)
				  + (path_a_u_turn ? config.parameters.u_turn_penalty_m : 0.f);

	// Path B: backtrack the already-travelled loop distance, then continue to the goal.
	const float dist_jump_travelled = projection_context.route_projection.dist.segment_along;
	const bool path_b_u_turn = uTurnRequired(projection_context, config, /* will_fly_reverse */ true);
	const float path_b_cost = dist_jump_travelled
				  + fabsf(goal_route_along - projection_context.loop_context.along.start)
				  + (path_b_u_turn ? config.parameters.u_turn_penalty_m : 0.f);

	// While loop iterations remain the current one must be completed: force path A.
	const bool use_path_a = projection_context.mission_loops_remaining > 0 || path_a_cost < path_b_cost;

	RoutePath path{};

	if (use_path_a) {
		path.first_item_index = projection_context.loop_context.segment.end.idx;
		path.first_item_cmd = projection_context.loop_context.segment.end.nav_cmd;
		path.first_item_position = projection_context.route_projection.segment_positions.end;
		path.direction_reversed = mustFlyReverse(goal_route_along, projection_context.loop_context.along.end,
					  direction_mode);
		path.u_turn_required = path_a_u_turn;
		path.total_cost_m = path_a_cost;

	} else {
		path.first_item_index = projection_context.loop_context.segment.start.idx;
		path.first_item_cmd = projection_context.loop_context.segment.start.nav_cmd;
		path.first_item_position = projection_context.route_projection.segment_positions.start;
		path.direction_reversed = mustFlyReverse(goal_route_along, projection_context.loop_context.along.start,
					  direction_mode);
		path.u_turn_required = path_b_u_turn;
		path.total_cost_m = path_b_cost;
	}

	PX4_DEBUG("Route path on loop jump [A,B], loop_along[%.1f, %.1f], loops remaining: %u",
		  static_cast<double>(projection_context.loop_context.along.start),
		  static_cast<double>(projection_context.loop_context.along.end),
		  static_cast<unsigned>(projection_context.mission_loops_remaining));

	return path;
}

RoutePath findShortestPathAlongRoute(int mission_count,
				     float goal_route_along,
				     const ProjectionContext &projection_context, const PlannerConfig &config,
				     PathDirectionMode direction_mode, RouteGoalSegmentType goal_seg_type)
{
	const bool on_jump_segment_and_goal_elsewhere = projection_context.loop_context.valid()
			&& goal_seg_type != RouteGoalSegmentType::kOnActiveLoopJump;

	RoutePath path = on_jump_segment_and_goal_elsewhere
			 ? solveShortestRoutePathFromActiveLoop(goal_route_along, projection_context, config,
					 direction_mode)
			 : solveShortestRoutePath(goal_route_along, projection_context, config, direction_mode);

	const bool valid_path = path.valid();

	if (valid_path && path.first_item_index >= mission_count) {
		PX4_ERR("Route route path targets out-of-bounds index %d (mission count %d)",
			static_cast<int>(path.first_item_index), static_cast<int>(mission_count));
		return {};
	}

	// The vehicle can skip the route join when it is already within the first item's acceptance radius.
	float dist_to_first_item = NAN;

	if (valid_path) {
		dist_to_first_item = get_distance_to_next_waypoint(path.first_item_position.lat, path.first_item_position.lon,
				     projection_context.vehicle_position.lat, projection_context.vehicle_position.lon);
		path.in_first_item_acc_rad = PX4_ISFINITE(dist_to_first_item)
					     && dist_to_first_item < config.parameters.acceptance_radius;
	}

	PX4_DEBUG("Route path: trgt=%d cmd=%u rev=%u uturn=%u dist=%.1f in_acc=%u",
		  static_cast<int>(path.first_item_index),
		  static_cast<unsigned>(path.first_item_cmd),
		  static_cast<unsigned>(path.direction_reversed),
		  static_cast<unsigned>(path.u_turn_required),
		  static_cast<double>(path.total_cost_m),
		  static_cast<unsigned>(path.in_first_item_acc_rad));
	PX4_DEBUG("Route path detail: first=%.1f vehicle=%.1f goal=%.1f",
		  static_cast<double>(dist_to_first_item),
		  static_cast<double>(projection_context.route_projection.dist.route_along),
		  static_cast<double>(goal_route_along));

	return path;
}

bool closeToSafePointDirect(const Position &vehicle_position,
			    const Position &safe_point_position, const PlannerConfig &config)
{
	const float dist = get_distance_to_next_waypoint(vehicle_position.lat, vehicle_position.lon,
			   safe_point_position.lat, safe_point_position.lon);

	return PX4_ISFINITE(dist) && dist < config.parameters.direct_acceptance_radius;
}

bool closeToBranchOffSegment(const Position &position,
			     const GoalSelection &selection,
			     float acceptance_radius,
			     float altitude_acceptance_radius)
{
	if (!position.valid() || !selection.branch_off.projection.valid() || !selection.goal_position.valid()
	    || !PX4_ISFINITE(acceptance_radius) || acceptance_radius <= 0.f
	    || !PX4_ISFINITE(altitude_acceptance_radius) || altitude_acceptance_radius <= 0.f) {
		PX4_ERR("Route invalid inputs to determine distance to branch-off segment");
		return false;
	}

	const Position &branch_off_projection = selection.branch_off.projection;

	// NED vectors avoid extra trigonometry (matching processCandidateForSegment).
	matrix::Vector2f branch_vector{};   // branch-off projection -> goal (safe point)
	matrix::Vector2f position_vector{}; // branch-off projection -> vehicle position
	get_vector_to_next_waypoint(branch_off_projection.lat, branch_off_projection.lon,
				    selection.goal_position.lat, selection.goal_position.lon,
				    &branch_vector(0), &branch_vector(1));
	get_vector_to_next_waypoint(branch_off_projection.lat, branch_off_projection.lon,
				    position.lat, position.lon,
				    &position_vector(0), &position_vector(1));

	// Along-track fraction, clamped so the projection stays on the branch-off leg.
	const float branch_length_sq = branch_vector.norm_squared();
	const float t = (branch_length_sq > FLT_EPSILON)
			? math::constrain(position_vector.dot(branch_vector) / branch_length_sq, 0.f, 1.f)
			: 0.f;
	const float xtrack = static_cast<matrix::Vector2f>(position_vector - branch_vector * t).norm();

	if (!PX4_ISFINITE(xtrack) || xtrack >= acceptance_radius) {
		return false;
	}

	// Altitude check avoids flying diagonally toward the safe point when the vehicle is horizontally close to
	// the branch-off leg but at the wrong altitude (e.g. already descending to land while a far-away safe point
	// is projected onto the land point); in that case we first want to rejoin the branch-in vertically.
	const float expected_alt = branch_off_projection.alt
				   + t * (selection.goal_position.alt - branch_off_projection.alt);
	const float alt_error = fabsf(position.alt - expected_alt);

	return PX4_ISFINITE(alt_error) && alt_error < altitude_acceptance_radius;
}

bool canUseCurrentAltitudeForJoinTarget(const RoutePath &path)
{
	// The vehicle targets the landing at the mission land (or takeoff when reverse),
	// do not force a climb back to the branch-in alt.
	return path.valid()
	       && path.in_first_item_acc_rad
	       && (isLandingCmd(path.first_item_cmd)
		   || (path.direction_reversed && isTakeoffCmd(path.first_item_cmd)));
}

bool canSkipRouteFollowToSelectedGoal(const Position &vehicle_position,
				      const GoalSelection &selection, const PlannerConfig &config)
{
	if (!selection.valid()) {
		return false;
	}

	if (selection.goal_type != GoalType::kSafePoint) {
		return canUseCurrentAltitudeForJoinTarget(selection.path);
	}

	// Safe-point goals only skip when the selected destination or branch-off leg is already close.
	return closeToSafePointDirect(vehicle_position, selection.goal_position, config)
	       || closeToBranchOffSegment(vehicle_position, selection, config.parameters.acceptance_radius,
					  config.parameters.altitude_acceptance_radius);
}

JoinContext buildJoinContext(const Position &vehicle_position,
			     const ProjectionContext &projection_context,
			     const RoutePath &path)
{
	JoinContext join_context{};
	join_context.projection = projection_context.route_projection.projection;
	join_context.use_current_altitude = canUseCurrentAltitudeForJoinTarget(path);

	if (join_context.use_current_altitude) {
		join_context.projection.alt = vehicle_position.alt;
	}

	return join_context;
}

RoutePath scoreBranchOffCandidate(int mission_count,
				  const ProjectionContext &projection_context,
				  const PlannerConfig &config,
				  const RouteProjectionCandidate &branch_off)
{
	const bool same_active_loop =
		projection_context.loop_context.valid()
		&& branch_off.segment.is_loop
		&& branch_off.segment.start.idx == projection_context.loop_context.segment.start.idx
		&& branch_off.segment.end.idx == projection_context.loop_context.segment.end.idx;

	if (branch_off.segment.is_loop && !same_active_loop) {
		PX4_DEBUG("Route safe point loop candidate skipped, not on the active loop jump");
		return {};
	}

	const RouteGoalSegmentType goal_seg_type = same_active_loop ? RouteGoalSegmentType::kOnActiveLoopJump
			: RouteGoalSegmentType::kOutsideActiveLoopJump;

	RoutePath path = findShortestPathAlongRoute(mission_count, branch_off.dist.route_along,
			 projection_context, config, PathDirectionMode::kAuto, goal_seg_type);

	if (!path.valid()) {
		return {};
	}

	// Safe-point ranking must include both the route-following distance and the final
	// straight branch-off leg from the route projection to the safe point.
	const float total_path_cost = path.total_cost_m + branch_off.dist.xtrack;

	if (!PX4_ISFINITE(total_path_cost)) {
		return {};
	}

	path.total_cost_m = total_path_cost;
	return path;
}

GoalSelection selectLowestCostSafePoint(int mission_count,
					const ProjectionContext &projection_context,
					const PlannerConfig &config,
					const ProjectionReferenceBatch &batch)
{
	GoalSelection selection{};

	for (uint8_t batch_index = 0; batch_index < batch.count; ++batch_index) {
		const Position &safe_point_position = batch.items[batch_index].position;
		const int32_t safe_point_index = batch.items[batch_index].source_index;
		const ProjectionCandidateBuffer &candidate_buffer = batch.items[batch_index].candidate_buffer;
		RoutePath best_path{};
		int best_projection_index = -1;

		for (uint8_t candidate_index = 0; candidate_index < candidate_buffer.count; ++candidate_index) {
			const RouteProjectionCandidate &branch_off = candidate_buffer.candidates[candidate_index];

			PX4_DEBUG("Route safe point %d cand %u branch_off[%u,%u]",
				  static_cast<int>(safe_point_index),
				  static_cast<unsigned>(candidate_index),
				  static_cast<unsigned>(branch_off.segment.start.idx),
				  static_cast<unsigned>(branch_off.segment.end.idx));

			const RoutePath path = scoreBranchOffCandidate(mission_count, projection_context, config, branch_off);

			if (!path.valid()) {
				continue;
			}

			if (path.total_cost_m < best_path.total_cost_m) {
				best_path = path;
				best_projection_index = candidate_index;
			}
		}

		if (best_projection_index < 0) {
			PX4_DEBUG("Route safe point %d: no valid projection (nb cand: %u)",
				  static_cast<int>(safe_point_index),
				  static_cast<unsigned>(candidate_buffer.count));
			continue;
		}

		if (!selection.found() || best_path.total_cost_m < selection.path.total_cost_m) {
			selection.goal_type = GoalType::kSafePoint;
			selection.path = best_path;
			selection.safe_point_index = safe_point_index;
			selection.goal_position = safe_point_position;
			selection.branch_off.segment = candidate_buffer.candidates[best_projection_index].segment;
			selection.branch_off.projection = candidate_buffer.candidates[best_projection_index].projection;
		}
	}

	if (selection.found()) {
		if (!selection.valid()) {
			PX4_ERR("Route selected safe point is not valid");
			return {};
		}

		PX4_DEBUG("Route safe point %d selected: trgt=%d rev=%u branch_off=%u->%u",
			  static_cast<int>(selection.safe_point_index),
			  static_cast<int>(selection.path.first_item_index),
			  static_cast<unsigned>(selection.path.direction_reversed),
			  static_cast<unsigned>(selection.branch_off.segment.start.idx),
			  static_cast<unsigned>(selection.branch_off.segment.end.idx));
	}

	return selection;
}

FailureReason selectSafePoint(const Provider &provider,
			      const MissionRouteProjection &projection,
			      int mission_count,
			      const ProjectionContext &projection_context,
			      const PlannerConfig &config,
			      ProjectionReferenceBatch &batch,
			      GoalSelection &selection)
{
	selection = {};

	if (!projection_context.valid()) {
		return FailureReason::kInvalidProjectionContext;
	}

	batch.count = 0;

	// TODO: implement geofence-aware pruning: reject safe points and vehicle projections
	// that would require crossing a geofence boundary.
	const int safe_point_count = provider.safePointCount();

	if (safe_point_count <= 0) {
		PX4_DEBUG("Route search: no safe points available");
		return FailureReason::kNoValidSafePoints;
	}

	ProjectionScanRequest scan_request{};
	scan_request.home_altitude_amsl = config.parameters.home_altitude_amsl;
	scan_request.xtrack_margin_m = config.parameters.safe_point_projection_search_dist;
	scan_request.compute_current_segment_bounds = false;

	GoalSelection best{};
	FailureReason scan_failure_reason = FailureReason::kNone;
	int safe_point_cursor = 0;

	// The full mission route is scanned once per batch.
	while (safe_point_cursor < safe_point_count) {
		const int batch_start_index = safe_point_cursor;
		loadSafePointBatch(provider, config, safe_point_cursor, batch);

		if (batch.count == 0) {
			continue;
		}

		RouteDistanceSummary distance_summary{};
		const FailureReason scan_status = projection.findProjectionCandidates(scan_request, batch, distance_summary);

		if (scan_status != FailureReason::kNone) {
			PX4_DEBUG("Route safe point batch scan failed at offset %d: %s", batch_start_index,
				  failureReasonString(scan_status));
			scan_failure_reason = scan_status;
			continue;
		}

		const GoalSelection batch_best = selectLowestCostSafePoint(mission_count, projection_context, config, batch);

		if (batch_best.found() && (!best.found() || batch_best.path.total_cost_m < best.path.total_cost_m)) {
			best = batch_best;
		}
	}

	if (best.valid()) {
		selection = best;
		return FailureReason::kNone;
	}

	return (scan_failure_reason != FailureReason::kNone)
	       ? scan_failure_reason : FailureReason::kNoValidCandidateFound;
}

GoalSelection selectMissionEndpointFallback(const Provider &provider,
		const MissionRouteProjection &projection,
		int mission_count,
		const ProjectionContext &projection_context,
		const PlannerConfig &config)
{
	GoalSelection selection{};

	if (!projection_context.valid()) {
		return selection;
	}

	int32_t takeoff_index{-1};
	mission_item_s takeoff_item{};
	RoutePath path_to_takeoff{};
	Position takeoff_position{};

	if (provider.getMissionTakeoffItem(takeoff_index, takeoff_item)
	    && extractMissionPosition(takeoff_item, config.parameters.home_altitude_amsl, takeoff_position)) {
		path_to_takeoff = findShortestPathAlongRoute(mission_count, 0.f, projection_context, config,
				  PathDirectionMode::kForceReverse, RouteGoalSegmentType::kOutsideActiveLoopJump);
	}

	const bool path_to_takeoff_valid = path_to_takeoff.valid();

	if (path_to_takeoff_valid && PX4_ISFINITE(config.parameters.home_altitude_amsl)) {
		takeoff_position.alt = config.parameters.home_altitude_amsl;
	}

	int32_t land_index{-1};
	mission_item_s land_item{};
	RoutePath path_to_land{};
	Position land_position{};

	if (provider.getMissionLandItem(land_index, land_item)
	    && extractMissionPosition(land_item, config.parameters.home_altitude_amsl, land_position)) {
		const float land_route_along = projection.accumulateRouteDistance(0, land_index,
					       config.parameters.home_altitude_amsl);

		if (PX4_ISFINITE(land_route_along)) {
			path_to_land = findShortestPathAlongRoute(mission_count, land_route_along, projection_context, config,
					PathDirectionMode::kForceNominal, RouteGoalSegmentType::kOutsideActiveLoopJump);
		}
	}

	const bool path_to_land_valid = path_to_land.valid();

	if (!path_to_takeoff_valid && !path_to_land_valid) {
		return selection;
	}

	if (!path_to_land_valid || (path_to_takeoff_valid && path_to_takeoff.total_cost_m < path_to_land.total_cost_m)) {
		selection.goal_type = GoalType::kMissionTakeoff;
		selection.goal_position = takeoff_position;
		selection.path = path_to_takeoff;

	} else {
		selection.goal_type = GoalType::kMissionLand;
		selection.goal_position = land_position;
		selection.path = path_to_land;
	}

	if (!selection.valid()) {
		PX4_ERR("Route fallback selection is not valid");
		return {};
	}

	PX4_DEBUG("Route fallback %s target=%d rev=%u",
		  goalTypeString(selection.goal_type),
		  static_cast<int>(selection.path.first_item_index),
		  static_cast<unsigned>(selection.path.direction_reversed));

	return selection;
}

FailureReason selectBestGoal(const Provider &provider,
			     const MissionRouteProjection &projection,
			     int mission_count,
			     const ProjectionContext &projection_context,
			     const PlannerConfig &config,
			     ProjectionReferenceBatch &batch,
			     GoalSelection &selection)
{
	selection = {};
	GoalSelection safe_point{};
	const FailureReason safe_point_status = selectSafePoint(provider, projection, mission_count,
						projection_context, config, batch, safe_point);

	if (safe_point_status == FailureReason::kNone) {
		selection = safe_point;
		return FailureReason::kNone;
	}

	// No reachable safe point: fall back to the mission end points.
	const GoalSelection fallback = selectMissionEndpointFallback(provider, projection, mission_count,
				       projection_context, config);

	if (fallback.valid()) {
		selection = fallback;
		return FailureReason::kNone;
	}

	return safe_point_status;
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

	if (!config.parameters.validForVehicleProjection()) {
		return FailureReason::kInvalidRequest;
	}

	const int mission_count = _provider.missionCount();

	if (!activeJumpAnchorValidForMission(request.active_jump_anchor, mission_count)) {
		return FailureReason::kInvalidRequest;
	}

	ProjectionReferenceBatch &reference_batch = plannerScratchBatch();
	MissionRouteProjection projection{_provider};

	ProjectionContext projection_context{};
	perf_begin(_collect_vehicle_projection_perf);
	const FailureReason projection_status = projection.collectVehicleProjection(request.vehicle_position,
						request.mission_index, config, reference_batch, projection_context);
	perf_end(_collect_vehicle_projection_perf);

	if (projection_status != FailureReason::kNone) {
		return projection_status;
	}

	const RoutePath path = findShortestPathAlongRoute(mission_count, projection_context.route_length,
			       projection_context, config, PathDirectionMode::kForceNominal,
			       RouteGoalSegmentType::kOutsideActiveLoopJump);
	const JoinContext join_context = buildJoinContext(request.vehicle_position, projection_context, path);
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
	if (!config.parameters.validForRouteToGoal()) {
		return FailureReason::kInvalidRequest;
	}

	const int mission_count = _provider.missionCount();

	if (!activeJumpAnchorValidForMission(request.active_jump_anchor, mission_count)) {
		return FailureReason::kInvalidRequest;
	}

	ProjectionReferenceBatch &reference_batch = plannerScratchBatch();
	MissionRouteProjection projection{_provider};

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
	const FailureReason selection_status = selectBestGoal(_provider, projection, mission_count, projection_context,
					       config, reference_batch, selection);
	perf_end(_select_best_goal_perf);

	if (selection_status != FailureReason::kNone) {
		return selection_status;
	}

	selection.fly_direct_to_goal = canSkipRouteFollowToSelectedGoal(request.vehicle_position, selection, config);

	const JoinContext join_context = buildJoinContext(request.vehicle_position, projection_context, selection.path);
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
