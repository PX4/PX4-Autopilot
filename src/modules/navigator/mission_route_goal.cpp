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
 * @file mission_route_goal.cpp
 *
 * Route path solving, safe-point scoring, endpoint fallback selection, and
 * join/skip policy for mission-route planning.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_goal.h"

#include <float.h>

#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

#include <px4_platform_common/log.h>

using namespace math;

namespace mission_route
{
namespace
{

void loadSafePointBatch(const Provider &provider,
			const PlannerConfig &config,
			int offset,
			ProjectionReferenceBatch &batch)
{
	batch.count = 0;

	const int safe_point_item_count = provider.safePointCount();
	const int batch_end = math::min(offset + static_cast<int>(kMaxSafePointBatch), safe_point_item_count);

	for (int safe_point_index = offset; safe_point_index < batch_end; ++safe_point_index) {
		mission_item_s safe_point_item{};

		if (!provider.loadSafePointItem(safe_point_index, safe_point_item)) {
			PX4_WARN("RTL safe point %d read failed", safe_point_index);
			continue;
		}

		Position safe_point_position{};

		if (!extractSafePointPosition(safe_point_item, config.parameters.home_altitude_amsl,
					      safe_point_position)) {
			PX4_DEBUG("RTL safe point %d skipped, invalid position or frame", safe_point_index);
			continue;
		}

		if (config.state.require_vtol_approach
		    && !provider.hasVtolLandApproachesAtSafePointIndex(safe_point_index,
				    config.parameters.home_altitude_amsl)) {
			PX4_DEBUG("RTL safe point %d skipped, no VTOL approach", safe_point_index);
			continue;
		}

		batch.items[batch.count].position = safe_point_position;
		batch.items[batch.count].source_index = safe_point_index;
		batch.count++;
	}
}

} // namespace

bool MissionRouteGoalSelector::mustFlyReverse(float goal_route_along, float projection_route_along,
		PathDirectionMode direction_mode) const
{
	switch (direction_mode) {
	case PathDirectionMode::kForceNominal:
		// If we target the mission end item and if the vehicle is also projected onto
		// the mission end item, goal_route_along < projection_route_along is ambiguous.
		return false;

	case PathDirectionMode::kForceReverse:
		// If we target the mission start item and if the vehicle is also projected onto
		// the mission start item, goal_route_along < projection_route_along is ambiguous.
		return true;

	case PathDirectionMode::kAuto:
	default:
		// If the goal is closer to the mission start than the vehicle projection,
		// must fly reverse to reach the goal.
		return goal_route_along < projection_route_along;
	}
}

matrix::Vector2f MissionRouteGoalSelector::computeDesiredCourseVector(const ProjectionContext &projection_context,
		float acceptance_radius,
		bool will_fly_reverse) const
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

bool MissionRouteGoalSelector::uTurnRequired(const ProjectionContext &projection_context,
		const PlannerConfig &config,
		bool will_fly_reverse) const
{
	if (!(config.state.is_fixed_wing || config.state.in_transition_to_fw)) {
		return false;
	}

	if (!projection_context.velocity_valid || !projection_context.vehicle_vel_ne.isAllFinite()) {
		return false; // If no velocity, no need for a u-turn
	}

	const matrix::Vector2f desired_course = computeDesiredCourseVector(projection_context,
						config.parameters.acceptance_radius, will_fly_reverse);

	if (!desired_course.isAllFinite()) {
		return false; // Can't execute a proper u-turn without a desired course
	}

	if (projection_context.vehicle_vel_ne.norm_squared() <= FLT_EPSILON || desired_course.norm_squared() <= FLT_EPSILON) {
		return false;
	}

	// Dot product > 0 means angle < 90 (aligned)
	// Dot product < 0 means angle > 90 (opposed, U-turn required)
	return projection_context.vehicle_vel_ne.dot(desired_course) < 0.f;
}

RoutePath MissionRouteGoalSelector::solveShortestRoutePath(float goal_route_along,
		const ProjectionContext &projection_context, const PlannerConfig &config,
		PathDirectionMode direction_mode) const
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
	const bool direction_change = (projection_context.is_flying_reverse != will_fly_reverse);

	if (!direction_change
	    && _projection.isIndexInProjectionSegment(projection_context.route_projection.segment,
			    projection_context.mission_index,
			    projection_context.is_flying_reverse)) {

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

RoutePath MissionRouteGoalSelector::solveShortestRoutePathFromActiveLoop(float goal_route_along,
		const ProjectionContext &projection_context, const PlannerConfig &config,
		PathDirectionMode direction_mode) const
{
	RoutePath path{};

	// RoutePath A: complete the remaining loop distance then continue to the goal.
	const bool path_a_loop_reverse = false;
	const float dist_jump_remaining = fabsf(projection_context.route_projection.dist.segment_length
						- projection_context.route_projection.dist.segment_along);
	const float path_a_raw_cost = dist_jump_remaining
				      + fabsf(goal_route_along - projection_context.loop_context.along.end);
	// U-turn requirement depends on the direction for the loop jump, not after leaving the jump
	const bool is_rev_from_loop_end =
		mustFlyReverse(goal_route_along, projection_context.loop_context.along.end, direction_mode);
	const bool path_a_u_turn_required = uTurnRequired(projection_context, config, path_a_loop_reverse);
	const float path_a_cost = path_a_raw_cost + (path_a_u_turn_required ? config.parameters.u_turn_penalty_m : 0.f);

	// If loops remain, we must complete the current iteration: force RoutePath A.
	if (projection_context.mission_loops_remaining > 0) {
		path.first_item_index = projection_context.loop_context.segment.end.idx;
		path.first_item_cmd = projection_context.loop_context.segment.end.nav_cmd;
		path.direction_reversed = is_rev_from_loop_end;
		path.u_turn_required = path_a_u_turn_required;
		path.total_cost_m = path_a_cost;
		path.first_item_position = projection_context.route_projection.segment_positions.end;

	} else {
		// RoutePath B: backtrack the already-travelled loop distance then continue to the goal.
		const bool path_b_loop_reverse = true;
		const float dist_jump_travelled = projection_context.route_projection.dist.segment_along;
		const float path_b_raw_cost = dist_jump_travelled
					      + fabsf(goal_route_along - projection_context.loop_context.along.start);
		const bool is_rev_from_loop_start =
			mustFlyReverse(goal_route_along, projection_context.loop_context.along.start, direction_mode);
		// U-turn requirement depends on the direction for the loop jump, not after leaving the jump
		const bool path_b_u_turn_required = uTurnRequired(projection_context, config, path_b_loop_reverse);
		const float path_b_cost = path_b_raw_cost + (path_b_u_turn_required ? config.parameters.u_turn_penalty_m : 0.f);

		// Select the cheaper of RoutePath A (complete loop) vs RoutePath B (reverse loop).
		if (path_a_cost < path_b_cost) {
			path.first_item_index = projection_context.loop_context.segment.end.idx;
			path.first_item_cmd = projection_context.loop_context.segment.end.nav_cmd;
			path.direction_reversed = is_rev_from_loop_end;
			path.u_turn_required = path_a_u_turn_required;
			path.total_cost_m = path_a_cost;
			path.first_item_position = projection_context.route_projection.segment_positions.end;

		} else {
			path.first_item_index = projection_context.loop_context.segment.start.idx;
			path.first_item_cmd = projection_context.loop_context.segment.start.nav_cmd;
			path.direction_reversed = is_rev_from_loop_start;
			path.u_turn_required = path_b_u_turn_required;
			path.total_cost_m = path_b_cost;
			path.first_item_position = projection_context.route_projection.segment_positions.start;
		}
	}

	PX4_DEBUG("RTL path on loop jump [A,B], loop_along[%.1f, %.1f], loops remaining: %u",
		  static_cast<double>(projection_context.loop_context.along.start),
		  static_cast<double>(projection_context.loop_context.along.end),
		  static_cast<unsigned>(projection_context.mission_loops_remaining));

	return path;
}

RoutePath MissionRouteGoalSelector::findShortestPathAlongRoute(float goal_route_along,
		const ProjectionContext &projection_context, const PlannerConfig &config,
		PathDirectionMode direction_mode, RouteGoalSegmentType goal_seg_type) const
{
	const bool on_jump_segment_and_goal_elsewhere = projection_context.loop_context.valid()
			&& goal_seg_type != RouteGoalSegmentType::kOnActiveLoopJump;

	RoutePath path = on_jump_segment_and_goal_elsewhere
			 ? solveShortestRoutePathFromActiveLoop(goal_route_along, projection_context, config,
					 direction_mode)
			 : solveShortestRoutePath(goal_route_along, projection_context, config, direction_mode);

	const bool valid_path = path.valid();

	if (valid_path && path.first_item_index >= _provider.missionCount()) {
		PX4_ERR("RTL route path targets out-of-bounds index %d (mission count %d)",
			static_cast<int>(path.first_item_index), static_cast<int>(_provider.missionCount()));
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

	PX4_DEBUG("RTL path: trgt=%d cmd=%u rev=%u uturn=%u dist=%.1f in_acc=%u",
		  static_cast<int>(path.first_item_index),
		  static_cast<unsigned>(path.first_item_cmd),
		  static_cast<unsigned>(path.direction_reversed),
		  static_cast<unsigned>(path.u_turn_required),
		  static_cast<double>(path.total_cost_m),
		  static_cast<unsigned>(path.in_first_item_acc_rad));
	PX4_DEBUG("RTL path detail: first=%.1f vehicle=%.1f goal=%.1f",
		  static_cast<double>(dist_to_first_item),
		  static_cast<double>(projection_context.route_projection.dist.route_along),
		  static_cast<double>(goal_route_along));

	return path;
}

bool MissionRouteGoalSelector::closeToSafePointDirect(const Position &vehicle_position,
		const Position &safe_point_position, const PlannerConfig &config) const
{
	const float dist = get_distance_to_next_waypoint(vehicle_position.lat, vehicle_position.lon,
			   safe_point_position.lat, safe_point_position.lon);

	return PX4_ISFINITE(dist) && dist < config.parameters.direct_acceptance_radius;
}

bool MissionRouteGoalSelector::closeToBranchOffSegment(const Position &position,
		const GoalSelection &selection,
		float acceptance_radius,
		float altitude_acceptance_radius) const
{
	if (!position.valid() || !selection.branch_off_projection.valid() || !selection.goal_position.valid()
	    || !PX4_ISFINITE(acceptance_radius) || acceptance_radius <= 0.f
	    || !PX4_ISFINITE(altitude_acceptance_radius) || altitude_acceptance_radius <= 0.f) {
		PX4_ERR("RTL invalid inputs to determine distance to branch-off segment");
		return false;
	}

	// NED vectors avoid extra trigonometry (matching processCandidateForSegment).
	matrix::Vector2f branch_vector{};   // branch-off projection -> goal (safe point)
	matrix::Vector2f position_vector{}; // branch-off projection -> vehicle position
	get_vector_to_next_waypoint(selection.branch_off_projection.lat, selection.branch_off_projection.lon,
				    selection.goal_position.lat, selection.goal_position.lon,
				    &branch_vector(0), &branch_vector(1));
	get_vector_to_next_waypoint(selection.branch_off_projection.lat, selection.branch_off_projection.lon,
				    position.lat, position.lon,
				    &position_vector(0), &position_vector(1));

	// Along-track fraction, clamped so the projection stays on the branch-off leg.
	const float branch_length_sq = branch_vector.norm_squared();
	const float t = (branch_length_sq > FLT_EPSILON)
			? constrain(position_vector.dot(branch_vector) / branch_length_sq, 0.f, 1.f)
			: 0.f;
	const float xtrack = static_cast<matrix::Vector2f>(position_vector - branch_vector * t).norm();

	if (!PX4_ISFINITE(xtrack) || xtrack >= acceptance_radius) {
		return false;
	}

	// Altitude check avoids flying diagonally toward the safe point when the vehicle is horizontally close to
	// the branch-off leg but at the wrong altitude (e.g. already descending to land while a far-away safe point
	// is projected onto the land point); in that case we first want to rejoin the branch-in vertically.
	const float expected_alt = selection.branch_off_projection.alt
				   + t * (selection.goal_position.alt - selection.branch_off_projection.alt);
	const float alt_error = fabsf(position.alt - expected_alt);

	return PX4_ISFINITE(alt_error) && alt_error < altitude_acceptance_radius;
}

bool MissionRouteGoalSelector::canUseCurrentAltitudeForJoinTarget(const RoutePath &path) const
{
	// The vehicle targets the landing at the mission land (or takeoff when reverse),
	// do not force a climb back to the branch-in alt.
	return path.valid()
	       && path.in_first_item_acc_rad
	       && (isLandingCmd(path.first_item_cmd)
		   || (path.direction_reversed && isTakeoffCmd(path.first_item_cmd)));
}

bool MissionRouteGoalSelector::canSkipRouteFollowToSelectedGoal(const Position &vehicle_position,
		const GoalSelection &selection, const PlannerConfig &config) const
{
	if (!selection.valid()) {
		return false;
	}

	if (!selection.safe_point_found) {
		return canUseCurrentAltitudeForJoinTarget(selection.path);
	}

	// Safe-point goals only skip when the selected destination or branch-off leg is already close.
	return closeToSafePointDirect(vehicle_position, selection.safe_point_position, config)
	       || closeToBranchOffSegment(vehicle_position, selection, config.parameters.acceptance_radius,
					  config.parameters.altitude_acceptance_radius);
}

JoinContext MissionRouteGoalSelector::buildJoinContext(const Position &vehicle_position,
		const ProjectionContext &projection_context,
		const RoutePath &path) const
{
	JoinContext join_context{};
	join_context.projection = projection_context.route_projection.projection;
	join_context.direction_reversed = path.direction_reversed;
	join_context.skip_altitude_requirement = canUseCurrentAltitudeForJoinTarget(path);

	if (join_context.skip_altitude_requirement) {
		join_context.projection.alt = vehicle_position.alt;
	}

	return join_context;
}

RoutePath MissionRouteGoalSelector::scoreBranchOffCandidate(const ProjectionContext &projection_context,
		const PlannerConfig &config,
		const RouteProjectionCandidate &branch_off) const
{
	const bool same_active_loop =
		projection_context.loop_context.valid()
		&& branch_off.segment.is_loop
		&& branch_off.segment.start.idx == projection_context.loop_context.segment.start.idx
		&& branch_off.segment.end.idx == projection_context.loop_context.segment.end.idx;

	if (branch_off.segment.is_loop && !same_active_loop) {
		PX4_DEBUG("RTL safe point loop candidate skipped, not on the active loop jump");
		return {};
	}

	const RouteGoalSegmentType goal_seg_type = same_active_loop ? RouteGoalSegmentType::kOnActiveLoopJump
			: RouteGoalSegmentType::kOutsideActiveLoopJump;

	RoutePath path = findShortestPathAlongRoute(branch_off.dist.route_along,
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

GoalSelection MissionRouteGoalSelector::selectLowestCostSafePoint(const ProjectionContext &projection_context,
		const PlannerConfig &config,
		const ProjectionReferenceBatch &batch) const
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

			PX4_DEBUG("RTL safe point %d cand %u branch_off[%u,%u]",
				  static_cast<int>(safe_point_index),
				  static_cast<unsigned>(candidate_index),
				  static_cast<unsigned>(branch_off.segment.start.idx),
				  static_cast<unsigned>(branch_off.segment.end.idx));

			const RoutePath path = scoreBranchOffCandidate(projection_context, config, branch_off);

			if (!path.valid()) {
				continue;
			}

			if (path.total_cost_m < best_path.total_cost_m) {
				best_path = path;
				best_projection_index = candidate_index;
			}
		}

		if (best_projection_index < 0) {
			PX4_DEBUG("RTL safe point %d: no valid projection (nb cand: %u)",
				  static_cast<int>(safe_point_index),
				  static_cast<unsigned>(candidate_buffer.count));
			continue;
		}

		if (!selection.found || best_path.total_cost_m < selection.path.total_cost_m) {
			selection.found = true;
			selection.safe_point_found = true;
			selection.goal_type = GoalType::kSafePoint;
			selection.path = best_path;
			selection.safe_point_index = safe_point_index;
			selection.safe_point_position = safe_point_position;
			selection.goal_position = safe_point_position;
			selection.branch_off_segment = candidate_buffer.candidates[best_projection_index].segment;
			selection.branch_off_projection = candidate_buffer.candidates[best_projection_index].projection;
		}
	}

	if (selection.found) {
		if (!selection.valid()) {
			PX4_ERR("RTL selected safe point is not valid");
			return {};
		}

		PX4_DEBUG("RTL safe point %d selected: trgt=%d rev=%u branch_off=%u->%u",
			  static_cast<int>(selection.safe_point_index),
			  static_cast<int>(selection.path.first_item_index),
			  static_cast<unsigned>(selection.path.direction_reversed),
			  static_cast<unsigned>(selection.branch_off_segment.start.idx),
			  static_cast<unsigned>(selection.branch_off_segment.end.idx));
	}

	return selection;
}

GoalSelectionResult MissionRouteGoalSelector::selectSafePoint(const ProjectionContext &projection_context,
		const PlannerConfig &config,
		ProjectionReferenceBatch &batch) const
{
	GoalSelectionResult result{};

	if (!projection_context.valid()) {
		result.failure_reason = FailureReason::kInvalidProjectionContext;
		return result;
	}

	batch.count = 0;

	// TODO: implement geofence-aware pruning: reject safe points and vehicle projections
	// that would require crossing a geofence boundary.
	const int safe_point_count = _provider.safePointCount();

	if (safe_point_count <= 0) {
		PX4_DEBUG("RTL search: no safe points available");
		result.failure_reason = FailureReason::kNoValidSafePoints;
		return result;
	}

	ProjectionScanRequest scan_request{};
	scan_request.home_altitude_amsl = config.parameters.home_altitude_amsl;
	scan_request.xtrack_margin_m = config.parameters.safe_point_projection_search_dist;
	scan_request.compute_current_segment_bounds = false;

	GoalSelection best{};
	FailureReason scan_failure_reason = FailureReason::kNone;

	// The full mission route is scanned once per batch.
	for (int offset = 0; offset < safe_point_count; offset += kMaxSafePointBatch) {
		loadSafePointBatch(_provider, config, offset, batch);

		if (batch.count == 0) {
			continue;
		}

		const ProjectionScanResult scan_result = _projection.findProjectionCandidates(scan_request, batch);

		if (!scan_result.success) {
			PX4_DEBUG("RTL safe point batch scan failed at offset %d: %s", offset,
				  failureReasonString(scan_result.failure_reason));
			scan_failure_reason = scan_result.failure_reason;
			continue;
		}

		const GoalSelection batch_best = selectLowestCostSafePoint(projection_context, config, batch);

		if (batch_best.found && (!best.found || batch_best.path.total_cost_m < best.path.total_cost_m)) {
			best = batch_best;
		}
	}

	if (best.found && best.valid()) {
		result.success = true;
		result.failure_reason = FailureReason::kNone;
		result.selection = best;
		return result;
	}

	result.failure_reason = (scan_failure_reason != FailureReason::kNone)
				? scan_failure_reason : FailureReason::kNoValidCandidateFound;
	return result;
}

GoalSelection MissionRouteGoalSelector::selectMissionEndpointFallback(const ProjectionContext &projection_context,
		const PlannerConfig &config) const
{
	GoalSelection selection{};

	if (!projection_context.valid()) {
		return selection;
	}

	int32_t takeoff_index{-1};
	mission_item_s takeoff_item{};
	bool have_takeoff = _provider.getMissionTakeoffItem(takeoff_index, takeoff_item);

	RoutePath path_to_takeoff{};
	Position takeoff_position{};
	const bool path_to_takeoff_valid = have_takeoff
					   && extractMissionPosition(takeoff_item, config.parameters.home_altitude_amsl,
							   takeoff_position)
					   && (path_to_takeoff = findShortestPathAlongRoute(0.f, projection_context,
							   config, PathDirectionMode::kForceReverse)).valid();

	if (path_to_takeoff_valid && PX4_ISFINITE(config.parameters.home_altitude_amsl)) {
		takeoff_position.alt = config.parameters.home_altitude_amsl;
	}

	int32_t land_index{-1};
	mission_item_s land_item{};
	bool have_land = _provider.getMissionLandItem(land_index, land_item);

	RoutePath path_to_land{};
	Position land_position{};
	const bool path_to_land_valid = have_land
					&& extractMissionPosition(land_item, config.parameters.home_altitude_amsl,
							land_position)
					&& (path_to_land = findShortestPathAlongRoute(
							projection_context.route_length,
							projection_context, config,
							PathDirectionMode::kForceNominal)).valid();

	if (!path_to_takeoff_valid && !path_to_land_valid) {
		return selection;
	}

	selection.found = true;
	selection.safe_point_found = false;

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
		PX4_ERR("RTL fallback selection is not valid");
		return {};
	}

	PX4_DEBUG("RTL fallback %s target=%d rev=%u",
		  goalTypeString(selection.goal_type),
		  static_cast<int>(selection.path.first_item_index),
		  static_cast<unsigned>(selection.path.direction_reversed));

	return selection;
}

GoalSelectionResult MissionRouteGoalSelector::selectBestGoal(const ProjectionContext &projection_context,
		const PlannerConfig &config,
		ProjectionReferenceBatch &batch) const
{
	const GoalSelectionResult safe_point = selectSafePoint(projection_context, config, batch);

	if (safe_point.success) {
		return safe_point;
	}

	// No reachable safe point: fall back to the mission end points.
	const GoalSelection fallback = selectMissionEndpointFallback(projection_context, config);

	if (fallback.found && fallback.valid()) {
		GoalSelectionResult result{};
		result.success = true;
		result.failure_reason = FailureReason::kNone;
		result.selection = fallback;
		return result;
	}

	GoalSelectionResult result{};
	result.failure_reason = safe_point.failure_reason;
	return result;
}

} // namespace mission_route
