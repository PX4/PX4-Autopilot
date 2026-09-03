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
 * Route planning policy: projects the vehicle onto the mission route, scores candidate
 * paths and goals, and returns plain plans. Reads data through a Provider, publishes nothing.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_planner.h"

#include "mission_item_utils.h"
#include "mission_route_land_approaches.h"
#include "mission_route_projection.h"
#include "mission_route_provider.h"

#include <float.h>
#include <new>

#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include <px4_platform_common/log.h>

using namespace mission_route;

namespace
{

// Dictates how the direction is chosen (nominal or reverse)
enum class PathDirectionMode : uint8_t {
	kAuto = 0, // used when the goal is a safe point that can be in front or behind
	kForceNominal, // used when the goal is land
	kForceReverse // used when the goal is takeoff
};

// Where the goal projects onto the route (for safe points: the branch-off segment), relative
// to the vehicle's active loop jump. A goal outside the loop requires exiting the loop first.
enum class GoalProjectionLocation : uint8_t {
	kOutsideActiveLoopJump = 0,
	kOnActiveLoopJump
};

/**
 * Scratch buffer shared by vehicle projection and safe-point selection; too big for the
 * navigator task stack (~380 bytes per CONFIG_NAVIGATOR_SAFE_POINT_BATCH_SIZE slot).
 *
 * No heap and no runtime allocation: the storage is reserved at link time (.bss) and the
 * non-allocating placement new only runs the constructor there on first use, so it cannot
 * fail. A plain static object would sit in .data instead (non-zero member defaults), whose
 * flash image does not fit on some boards.
 */
ProjectionReferenceBatch &plannerScratchBatch()
{
	alignas(ProjectionReferenceBatch) static uint8_t storage[sizeof(ProjectionReferenceBatch)];
	static ProjectionReferenceBatch *batch = new (storage) ProjectionReferenceBatch{};

	return *batch;
}

template<typename RequestT>
PlannerConfig makeCommonPlannerConfig(const RequestT &request)
{
	PlannerConfig config{};
	config.parameters.vehicle_projection_search_dist_m = request.projection_search_distance_m;
	config.parameters.nav_acceptance_radius_m = request.acceptance_radius_m;
	config.parameters.home_altitude_amsl = request.home_altitude_amsl;
	config.parameters.fw_u_turn_penalty_m = request.fw_u_turn_penalty_m;
	config.parameters.vtol_state_on_mission_upload = request.vtol_state_on_mission_upload;
	config.active_jump_anchor = request.active_jump_anchor;
	config.state.is_flying_reverse = request.current_route_direction_reversed;
	config.state.is_vtol = request.is_vtol;
	config.state.is_fixed_wing = request.is_fixed_wing;
	config.state.in_transition_to_fw = request.in_transition_to_fw;
	config.state.velocity_ne(0) = request.velocity_north_m_s;
	config.state.velocity_ne(1) = request.velocity_east_m_s;
	config.state.velocity_valid = config.state.velocity_ne.isAllFinite();
	return config;
}

PlannerConfig makePlannerConfig(const MissionResumeRequest &request)
{
	PlannerConfig config = makeCommonPlannerConfig(request);
	config.respect_jump_repeats = true;
	return config;
}

PlannerConfig makePlannerConfig(const RtlRouteRequest &request)
{
	PlannerConfig config = makeCommonPlannerConfig(request);
	config.parameters.safe_point_projection_search_dist_m = request.safe_point_projection_search_distance_m;
	config.parameters.straight_to_safe_point_rad_m = request.direct_goal_acceptance_radius_m;
	config.parameters.altitude_acceptance_radius_m = request.altitude_acceptance_radius_m;
	config.state.require_vtol_approach = request.require_vtol_approach;
	// Return-to-goal uses jump geometry without replaying mission repeats.
	config.respect_jump_repeats = false;
	return config;
}

bool mustFlyReverse(float goal_dist_along_route_m, float proj_dist_along_route_m,
		    PathDirectionMode direction_mode)
{
	switch (direction_mode) {
	case PathDirectionMode::kForceNominal:
		return false;

	case PathDirectionMode::kForceReverse:
		return true;

	case PathDirectionMode::kAuto:
	default:
		return goal_dist_along_route_m < proj_dist_along_route_m;
	}
}

/**
 * Course the vehicle is about to fly, compared against its velocity to detect a needed u-turn:
 *    - Far from the route (or on a degenerate segment): follow the vector to the branch-in projection,
 *      since the vehicle joins the route first
 *    - Close to route: follow the current route leg
 */
matrix::Vector2f computeDesiredCourseVector(const ProjectionContext &projection_context,
		float acceptance_radius_m,
		bool will_fly_reverse)
{
	static constexpr float kSmallLengthM = 5.f;
	const float far_from_route_m = math::max(acceptance_radius_m, kSmallLengthM);
	matrix::Vector2f desired_course_vec{};

	if (projection_context.route_projection.dist.segment_length_m < kSmallLengthM
	    || projection_context.route_projection.dist.xtrack_m > far_from_route_m) {
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
	const VehicleStateContext &vehicle_state = projection_context.vehicle_state;

	// When in MC or in back transition, no need for u-turn
	if (!(vehicle_state.is_fixed_wing || vehicle_state.in_transition_to_fw)) {
		return false;
	}

	// When projected onto a MC segment we will reach it in MC, no u-turn needed
	if (projection_context.projected_segment_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC) {
		return false;
	}

	// Without valid vel, cannot compare current course to desired course
	if (!vehicle_state.velocity_valid) {
		return false;
	}

	const matrix::Vector2f desired_course = computeDesiredCourseVector(projection_context,
						config.parameters.nav_acceptance_radius_m, will_fly_reverse);

	if (!desired_course.isAllFinite()) {
		return false;
	}

	// If static, u-turn is done in place
	if (vehicle_state.velocity_ne.norm_squared() <= FLT_EPSILON || desired_course.norm_squared() <= FLT_EPSILON) {
		return false;
	}

	// Velocity opposing the desired course requires a u-turn.
	return vehicle_state.velocity_ne.dot(desired_course) < 0.f;
}

FirstRouteItem firstRouteItem(const SegmentEndpoint &endpoint, const Position &position)
{
	FirstRouteItem item{};
	item.index = endpoint.idx;
	item.nav_cmd = endpoint.nav_cmd;
	item.position = position;
	return item;
}

RoutePath solveShortestRoutePath(float goal_dist_along_route_m,
				 const ProjectionContext &projection_context, const PlannerConfig &config,
				 PathDirectionMode direction_mode)
{
	RoutePath path{};

	const bool will_fly_reverse = mustFlyReverse(goal_dist_along_route_m,
				      projection_context.route_projection.dist.along_route_m,
				      direction_mode);
	const float abs_distance_projection_to_goal =
		fabsf(goal_dist_along_route_m - projection_context.route_projection.dist.along_route_m);

	path.direction_reversed = will_fly_reverse;
	path.u_turn_required = uTurnRequired(projection_context, config, will_fly_reverse);
	path.total_cost_m = abs_distance_projection_to_goal + (path.u_turn_required ? config.parameters.fw_u_turn_penalty_m : 0.f);

	// Choose which segment endpoint becomes the first target for route following.
	bool choose_segment_start = false;
	const bool direction_change = (projection_context.vehicle_state.is_flying_reverse != will_fly_reverse);

	if (!direction_change
	    && isIndexInProjectionSegment(projection_context.route_projection.segment,
					  projection_context.mission_index,
					  projection_context.vehicle_state.is_flying_reverse)) {

		// E.g. seg [2,4] where 3 is a front transition, if we're targeting the FT (3),
		// choose item start (2) to ensure the FT is not skipped
		choose_segment_start = projection_context.mission_index < projection_context.route_projection.segment.end.idx;

	} else {
		// Direction change or off-segment: choose start when reversing, end when nominal.
		choose_segment_start = will_fly_reverse;
	}

	const RouteProjectionCandidate &route_projection = projection_context.route_projection;
	path.first_item = choose_segment_start
			  ? firstRouteItem(route_projection.segment.start, route_projection.segment_positions.start)
			  : firstRouteItem(route_projection.segment.end, route_projection.segment_positions.end);

	return path;
}

RoutePath solveShortestRoutePathFromActiveLoop(float goal_dist_along_route_m,
		const ProjectionContext &projection_context, const PlannerConfig &config,
		PathDirectionMode direction_mode)
{
	// The u-turn check uses the direction flown on the loop jump itself, not the direction
	// after leaving it.

	// Path A: complete the remaining loop distance, then continue to the goal.
	const float dist_jump_remaining = fabsf(projection_context.route_projection.dist.segment_length_m
						- projection_context.route_projection.dist.along_segment_m);
	const bool path_a_u_turn = uTurnRequired(projection_context, config, /* will_fly_reverse */ false);
	const float path_a_cost = dist_jump_remaining
				  + fabsf(goal_dist_along_route_m - projection_context.loop_context.bounds.end_dist_along_route_m)
				  + (path_a_u_turn ? config.parameters.fw_u_turn_penalty_m : 0.f);

	// Path B: backtrack the already-travelled loop distance, then continue to the goal.
	const float dist_jump_travelled = projection_context.route_projection.dist.along_segment_m;
	const bool path_b_u_turn = uTurnRequired(projection_context, config, /* will_fly_reverse */ true);
	const float path_b_cost = dist_jump_travelled
				  + fabsf(goal_dist_along_route_m - projection_context.loop_context.bounds.start_dist_along_route_m)
				  + (path_b_u_turn ? config.parameters.fw_u_turn_penalty_m : 0.f);

	// Mission finishes the current repeat before leaving the loop.
	const bool use_path_a = (config.respect_jump_repeats
				 && projection_context.route_projection.segment.has_remaining_repeats)
				|| path_a_cost < path_b_cost;

	const LoopContext &loop_context = projection_context.loop_context;
	RoutePath path{};

	if (use_path_a) {
		path.first_item = firstRouteItem(loop_context.segment.end, loop_context.segment_positions.end);
		path.direction_reversed = mustFlyReverse(goal_dist_along_route_m,
					  loop_context.bounds.end_dist_along_route_m, direction_mode);
		path.u_turn_required = path_a_u_turn;
		path.total_cost_m = path_a_cost;

	} else {
		path.first_item = firstRouteItem(loop_context.segment.start, loop_context.segment_positions.start);
		path.direction_reversed = mustFlyReverse(goal_dist_along_route_m,
					  loop_context.bounds.start_dist_along_route_m, direction_mode);
		path.u_turn_required = path_b_u_turn;
		path.total_cost_m = path_b_cost;
	}

	PX4_DEBUG("Route path on loop jump [A,B], loop bounds [%.1f, %.1f], force repeat: %u",
		  static_cast<double>(projection_context.loop_context.bounds.start_dist_along_route_m),
		  static_cast<double>(projection_context.loop_context.bounds.end_dist_along_route_m),
		  static_cast<unsigned>(config.respect_jump_repeats
					&& projection_context.route_projection.segment.has_remaining_repeats));

	return path;
}

RoutePath findShortestPathAlongRoute(int mission_count,
				     float goal_dist_along_route_m,
				     const ProjectionContext &projection_context, const PlannerConfig &config,
				     PathDirectionMode direction_mode, GoalProjectionLocation goal_projection_location)
{
	const bool on_jump_segment_and_goal_elsewhere = projection_context.loop_context.valid()
			&& goal_projection_location != GoalProjectionLocation::kOnActiveLoopJump;

	RoutePath path = on_jump_segment_and_goal_elsewhere
			 ? solveShortestRoutePathFromActiveLoop(goal_dist_along_route_m, projection_context, config,
					 direction_mode)
			 : solveShortestRoutePath(goal_dist_along_route_m, projection_context, config, direction_mode);

	const bool valid_path = path.valid();

	if (valid_path && path.first_item.index >= mission_count) {
		PX4_ERR("Invalid route path idx: %d (count %d)",
			static_cast<int>(path.first_item.index), static_cast<int>(mission_count));
		return {};
	}

	// The vehicle can skip the route join when it is already within the first item's acceptance radius.
	float dist_to_first_item = NAN;

	if (valid_path) {
		dist_to_first_item = get_distance_to_next_waypoint(path.first_item.position.lat, path.first_item.position.lon,
				     projection_context.vehicle_position.lat, projection_context.vehicle_position.lon);
		path.first_item.in_acc_rad = PX4_ISFINITE(dist_to_first_item)
					     && dist_to_first_item < config.parameters.nav_acceptance_radius_m;
	}

	PX4_DEBUG("Route path: trgt=%d cmd=%u rev=%u uturn=%u dist=%.1f in_acc=%u",
		  static_cast<int>(path.first_item.index),
		  static_cast<unsigned>(path.first_item.nav_cmd),
		  static_cast<unsigned>(path.direction_reversed),
		  static_cast<unsigned>(path.u_turn_required),
		  static_cast<double>(path.total_cost_m),
		  static_cast<unsigned>(path.first_item.in_acc_rad));
	PX4_DEBUG("Route path detail: first=%.1f vehicle=%.1f goal=%.1f",
		  static_cast<double>(dist_to_first_item),
		  static_cast<double>(projection_context.route_projection.dist.along_route_m),
		  static_cast<double>(goal_dist_along_route_m));

	return path;
}

void loadSafePointBatch(const Provider &provider,
			const PlannerConfig &config,
			int &safe_point_cursor,
			ProjectionReferenceBatch &batch)
{
	batch.count = 0;

	const int safe_point_item_count = provider.safePointCount();

	// The cursor persists across calls to resume from the previous batch.
	for (; safe_point_cursor < safe_point_item_count && batch.count < kMaxSafePointBatch; ++safe_point_cursor) {
		const int safe_point_index = safe_point_cursor;
		mission_item_s safe_point_item{};

		if (!provider.loadSafePointItem(safe_point_index, safe_point_item)) {
			PX4_WARN("Safe point %d read failed", safe_point_index);
			continue;
		}

		Position safe_point_position{};

		if (!extractSafePointPosition(safe_point_item, config.parameters.home_altitude_amsl,
					      safe_point_position)) {
			PX4_DEBUG("Route safe point %d skipped, invalid position or frame", safe_point_index);
			continue;
		}

		if (config.state.require_vtol_approach
		    && !hasVtolLandApproachesAtSafePointIndex(provider, safe_point_index,
				    config.parameters.home_altitude_amsl)) {
			PX4_DEBUG("Route safe point %d skipped, no VTOL approach", safe_point_index);
			continue;
		}

		batch.items[batch.count].position = safe_point_position;
		batch.items[batch.count].source_index = safe_point_index;
		batch.count++;
	}
}

RoutePath scoreBranchOffCandidate(int mission_count,
				  const ProjectionContext &projection_context,
				  const PlannerConfig &config,
				  const RouteProjectionCandidate &branch_off)
{
	const bool same_active_loop = projection_context.loop_context.valid()
				      && branch_off.segment.isLoop()
				      && branch_off.segment.jump_item_index == projection_context.loop_context.segment.jump_item_index;

	// Simplifies the high level logic, no need to handle how to get onto the loop jump segment
	if (branch_off.segment.isLoop() && !same_active_loop) {
		PX4_DEBUG("Route safe point loop candidate skipped, not on the active loop jump");
		return {};
	}

	const GoalProjectionLocation goal_projection_location = same_active_loop ? GoalProjectionLocation::kOnActiveLoopJump
			: GoalProjectionLocation::kOutsideActiveLoopJump;

	RoutePath path = findShortestPathAlongRoute(mission_count, branch_off.dist.along_route_m,
			 projection_context, config, PathDirectionMode::kAuto, goal_projection_location);

	if (!path.valid()) {
		return {};
	}

	// Both were validated and are finite.
	path.total_cost_m += branch_off.dist.xtrack_m;
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
			PX4_ERR("Invalid route safe point");
			return {};
		}

		PX4_DEBUG("Route safe point %d selected: trgt=%d rev=%u branch_off=%u->%u",
			  static_cast<int>(selection.safe_point_index),
			  static_cast<int>(selection.path.first_item.index),
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

	// TODO: implement geofence-aware pruning: reject safe points and vehicle projections
	// that would require crossing a geofence boundary.
	const int safe_point_count = provider.safePointCount();

	if (safe_point_count <= 0) {
		PX4_DEBUG("Route search: no safe points available");
		return FailureReason::kNoValidSafePoints;
	}

	// Safe points use their own search window and do not need current segment bounds.
	ProjectionScanRequest scan_request{};
	scan_request.home_altitude_amsl = config.parameters.home_altitude_amsl;
	scan_request.xtrack_margin_m = config.parameters.safe_point_projection_search_dist_m;

	GoalSelection best{};
	FailureReason scan_failure_reason = FailureReason::kNone;
	int safe_point_cursor = 0;

	// The full mission route is scanned once per batch.
	while (safe_point_cursor < safe_point_count) {
		const int batch_start_index = safe_point_cursor;
		// always advances the cursor, so every pass makes progress.
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

bool findMissionTakeoffItem(const Provider &provider, int mission_count, int32_t &index,
			    mission_item_s &takeoff_item)
{
	for (int i = 0; i < mission_count; ++i) {
		mission_item_s item{};

		if (!provider.loadMissionItem(i, item)) {
			break;
		}

		if (isTakeoffCmd(item.nav_cmd)) {
			index = i;
			takeoff_item = item;
			return true;
		}

		if (mission_item_contains_position(item)) {
			break;
		}
	}

	return false;
}

GoalSelection selectMissionEndpointFallback(const Provider &provider,
		int mission_count,
		int32_t mission_land_index,
		const ProjectionContext &projection_context,
		const PlannerConfig &config)
{
	GoalSelection selection{};

	int32_t takeoff_index{-1};
	mission_item_s takeoff_item{};
	RoutePath path_to_takeoff{};
	Position takeoff_position{};

	if (findMissionTakeoffItem(provider, mission_count, takeoff_index, takeoff_item)
	    && extractMissionPosition(takeoff_item, config.parameters.home_altitude_amsl, takeoff_position)) {
		path_to_takeoff = findShortestPathAlongRoute(mission_count, 0.f, projection_context, config,
				  PathDirectionMode::kForceReverse, GoalProjectionLocation::kOutsideActiveLoopJump);
	}

	const bool path_to_takeoff_valid = path_to_takeoff.valid();

	// The takeoff item carries the climb altitude; the return goal is the takeoff location at home altitude.
	if (path_to_takeoff_valid && PX4_ISFINITE(config.parameters.home_altitude_amsl)) {
		takeoff_position.alt = config.parameters.home_altitude_amsl;
	}

	mission_item_s land_item{};
	RoutePath path_to_land{};
	Position land_position{};

	if (mission_land_index >= 0 && mission_land_index < mission_count
	    && mission_land_index == projection_context.route_end_index
	    && provider.loadMissionItem(mission_land_index, land_item)
	    && isLandingCmd(land_item.nav_cmd)
	    && extractMissionPosition(land_item, config.parameters.home_altitude_amsl, land_position)) {
		path_to_land = findShortestPathAlongRoute(mission_count, projection_context.route_length,
				projection_context, config, PathDirectionMode::kForceNominal,
				GoalProjectionLocation::kOutsideActiveLoopJump);
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
		PX4_ERR("Invalid route fallback");
		return {};
	}

	PX4_DEBUG("Route fallback %s target=%d rev=%u",
		  goalTypeString(selection.goal_type),
		  static_cast<int>(selection.path.first_item.index),
		  static_cast<unsigned>(selection.path.direction_reversed));

	return selection;
}

FailureReason selectBestGoal(const Provider &provider,
			     const MissionRouteProjection &projection,
			     int mission_count,
			     int32_t mission_land_index,
			     const ProjectionContext &projection_context,
			     const PlannerConfig &config,
			     ProjectionReferenceBatch &batch,
			     GoalSelection &selection)
{
	const FailureReason safe_point_status = selectSafePoint(provider, projection, mission_count,
						projection_context, config, batch, selection);

	if (safe_point_status == FailureReason::kNone) {
		return FailureReason::kNone;
	}

	// No reachable safe point: fall back to the mission end points.
	selection = selectMissionEndpointFallback(provider, mission_count, mission_land_index,
			projection_context, config);

	if (selection.valid()) {
		return FailureReason::kNone;
	}

	selection = {};
	return safe_point_status;
}

bool canUseCurrentAltitudeForJoinTarget(const RoutePath &path)
{
	// The altitude of the land item is set at ground level.
	// Keep vehicle altitude at a landing item, or a takeoff item in reverse.
	return path.valid()
	       && path.first_item.in_acc_rad
	       && (isLandingCmd(path.first_item.nav_cmd)
		   || (path.direction_reversed && isTakeoffCmd(path.first_item.nav_cmd)));
}

bool closeToBranchOffSegment(const Position &position,
			     const GoalSelection &selection,
			     float acceptance_radius_m,
			     float altitude_acceptance_radius_m)
{
	if (!position.valid() || !selection.branch_off.projection.valid() || !selection.goal_position.valid()
	    || !PX4_ISFINITE(acceptance_radius_m) || acceptance_radius_m < 0.f
	    || !PX4_ISFINITE(altitude_acceptance_radius_m) || altitude_acceptance_radius_m < 0.f) {
		PX4_ERR("Invalid branch-off dist inputs");
		return false;
	}

	const Position &branch_off_projection = selection.branch_off.projection;

	// NED vectors avoid extra trigonometry
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
	const float xtrack_m = static_cast<matrix::Vector2f>(position_vector - branch_vector * t).norm();

	if (!PX4_ISFINITE(xtrack_m) || xtrack_m >= acceptance_radius_m) {
		return false;
	}

	// Altitude check avoids flying diagonally toward the safe point when the vehicle is horizontally close to
	// the branch-off leg but at the wrong altitude (e.g. already descending to land while a far-away safe point
	// is projected onto the land point); in that case we first want to rejoin the branch-in vertically.
	const float expected_alt = branch_off_projection.alt
				   + t * (selection.goal_position.alt - branch_off_projection.alt);
	const float alt_error = fabsf(position.alt - expected_alt);

	return PX4_ISFINITE(alt_error) && alt_error < altitude_acceptance_radius_m;
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
	const float dist_to_goal_m = get_distance_to_next_waypoint(vehicle_position.lat, vehicle_position.lon,
				     selection.goal_position.lat, selection.goal_position.lon);
	const bool close_to_safe_point_direct = PX4_ISFINITE(dist_to_goal_m)
						&& dist_to_goal_m < config.parameters.straight_to_safe_point_rad_m;

	return close_to_safe_point_direct
	       || closeToBranchOffSegment(vehicle_position, selection, config.parameters.nav_acceptance_radius_m,
					  config.parameters.altitude_acceptance_radius_m);
}

// Plans carry the jump identity only when the vehicle is projected onto a valid loop.
ActiveJumpAnchor activeJumpAnchor(const Segment &segment)
{
	if (segment.validLoop()) {
		return {segment.jump_item_index};
	}

	return {};
}

/**
 * The transition needed to bring the vehicle to target_vtol_state.
 * kNone when it already matches, the target is unknown, or the vehicle is not a VTOL.
 */
VtolTransitionAction vtolTransitionActionForState(uint8_t target_vtol_state,
		const VehicleStateContext &vehicle_state)
{
	if (!vehicle_state.is_vtol) {
		return VtolTransitionAction::kNone;
	}

	const bool fw_or_transition_to_fw = vehicle_state.is_fixed_wing || vehicle_state.in_transition_to_fw;

	// A back transition is missed when the segment needs MC but the vehicle flies FW (or heads there).
	if (target_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC && fw_or_transition_to_fw) {
		return VtolTransitionAction::kBackTransition;
	}

	// A front transition is missed when the segment needs FW but the vehicle flies MC (or heads there).
	if (target_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW && !fw_or_transition_to_fw) {
		return VtolTransitionAction::kFrontTransition;
	}

	return VtolTransitionAction::kNone;
}

template<typename PlanT>
void fillRouteJoinPlan(PlanT &plan, const ProjectionContext &projection_context, const RoutePath &path,
		       const Position &vehicle_position)
{
	plan.join_position = projection_context.route_projection.projection;
	plan.first_mission_item_index = path.first_item.index;
	plan.direction_reversed = path.direction_reversed;
	plan.use_current_altitude = canUseCurrentAltitudeForJoinTarget(path);
	plan.active_jump_anchor = activeJumpAnchor(projection_context.route_projection.segment);
	plan.vtol_transition_action = vtolTransitionActionForState(projection_context.projected_segment_vtol_state,
				      projection_context.vehicle_state);

	if (plan.use_current_altitude) {
		plan.join_position.alt = vehicle_position.alt;
	}
}

RtlRoutePlan makeRtlRoutePlan(const ProjectionContext &projection_context, const Position &vehicle_position,
			      const GoalSelection &selection)
{
	RtlRoutePlan plan{};
	fillRouteJoinPlan(plan, projection_context, selection.path, vehicle_position);
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

// First stage of both plans: check the anchor, project the vehicle, resolve the segment's VTOL state.
FailureReason buildProjectionContext(const Provider &provider, const PlannerConfig &config,
				     const Position &vehicle_position, int32_t mission_index,
				     ProjectionReferenceBatch &batch, ProjectionContext &projection_context)
{
	if (!config.active_jump_anchor.validForMission(provider.missionCount())) {
		return FailureReason::kInvalidRequest;
	}

	const MissionRouteProjection projection{provider};
	const FailureReason projection_status = projection.collectVehicleProjection(vehicle_position, mission_index,
						config, batch, projection_context);

	if (projection_status != FailureReason::kNone) {
		return projection_status;
	}

	// Only VTOLs read the segment state: the u-turn check and plan assembly use it.
	if (config.state.is_vtol) {
		projection_context.projected_segment_vtol_state = vtolStateForSegment(provider,
				projection_context.route_projection.segment, config.parameters.vtol_state_on_mission_upload);
	}

	return FailureReason::kNone;
}

} // namespace

MissionRoutePlanner::MissionRoutePlanner(const Provider &provider) :
	_provider(provider)
{
}

FailureReason MissionRoutePlanner::planMissionResumeJoin(const MissionResumeRequest &request,
		MissionResumePlan &plan) const
{
	plan = {};
	const PlannerConfig config = makePlannerConfig(request);

	if (!config.parameters.validForVehicleProjection()) {
		return FailureReason::kInvalidRequest;
	}

	ProjectionContext projection_context{};
	const FailureReason projection_status = buildProjectionContext(_provider, config, request.vehicle_position,
						request.mission_index, plannerScratchBatch(), projection_context);

	if (projection_status != FailureReason::kNone) {
		return projection_status;
	}

	const RoutePath path = findShortestPathAlongRoute(_provider.missionCount(), projection_context.route_length,
			       projection_context, config, PathDirectionMode::kForceNominal,
			       GoalProjectionLocation::kOutsideActiveLoopJump);
	MissionResumePlan candidate{};
	fillRouteJoinPlan(candidate, projection_context, path, request.vehicle_position);

	if (!candidate.valid()) {
		return FailureReason::kNoValidPath;
	}

	plan = candidate;
	return FailureReason::kNone;
}

FailureReason MissionRoutePlanner::planRtlRoute(const RtlRouteRequest &request,
		RtlRoutePlan &plan) const
{
	plan = {};
	const PlannerConfig config = makePlannerConfig(request);

	if (!config.parameters.validForRtlRoute()) {
		return FailureReason::kInvalidRequest;
	}

	ProjectionReferenceBatch &reference_batch = plannerScratchBatch();
	ProjectionContext projection_context{};
	const FailureReason projection_status = buildProjectionContext(_provider, config, request.vehicle_position,
						request.mission_index, reference_batch, projection_context);

	if (projection_status != FailureReason::kNone) {
		return projection_status;
	}

	// Find the lowest-cost safe point, falling back to mission endpoints if none is usable.
	const MissionRouteProjection projection{_provider};
	GoalSelection selection{};
	const FailureReason selection_status = selectBestGoal(_provider, projection, _provider.missionCount(),
					       request.mission_land_index, projection_context, config,
					       reference_batch, selection);

	if (selection_status != FailureReason::kNone) {
		return selection_status;
	}

	selection.fly_direct_to_goal = canSkipRouteFollowToSelectedGoal(request.vehicle_position, selection, config);

	const RtlRoutePlan candidate = makeRtlRoutePlan(projection_context, request.vehicle_position, selection);

	if (!candidate.valid()) {
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
