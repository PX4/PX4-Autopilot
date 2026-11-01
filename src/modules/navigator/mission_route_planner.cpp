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
#include "mission_route_path_checker.h"
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
	kForceReverse, // used when the goal is takeoff
	kResume // Mission resume: nominal, and keeps a loop with repeats left
};

// Where the goal projects onto the route (for safe points: the branch-off segment), relative
// to the vehicle's active loop jump. A goal outside the loop requires exiting the loop first.
enum class GoalProjectionLocation : uint8_t {
	kOutsideActiveLoopJump = 0,
	kOnActiveLoopJump
};

// Scratch shared by vehicle projection and safe-point selection, too big for the task stack.
// Placement new keeps it in .bss; a plain static would land in .data and cost flash.
ProjectionReferenceBatch &plannerScratchBatch()
{
	alignas(ProjectionReferenceBatch) static uint8_t storage[sizeof(ProjectionReferenceBatch)];
	static ProjectionReferenceBatch *batch = new (storage) ProjectionReferenceBatch{};

	return *batch;
}

bool validDistance(float distance_m)
{
	return PX4_ISFINITE(distance_m) && distance_m >= 0.f;
}

// home_altitude_amsl is not checked: NAN is valid for absolute-altitude missions.
bool validJoinRequest(const MissionResumeRequest &request)
{
	return validDistance(request.projection_search_distance_m) && validDistance(request.acceptance_radius_m)
	       && validDistance(request.fw_u_turn_penalty_m);
}

bool validRtlRequest(const RtlRouteRequest &request)
{
	return validJoinRequest(request) && validDistance(request.safe_point_projection_search_distance_m)
	       && validDistance(request.direct_goal_acceptance_radius_m) && validDistance(request.altitude_acceptance_radius_m);
}

bool mustFlyReverse(float goal_dist_along_route_m, float proj_dist_along_route_m,
		    PathDirectionMode direction_mode)
{
	switch (direction_mode) {
	case PathDirectionMode::kForceNominal:
	case PathDirectionMode::kResume:
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
matrix::Vector2f computeDesiredCourseVector(const MissionResumeRequest &request,
		const ProjectionContext &projection_context, bool will_fly_reverse)
{
	static constexpr float kSmallLengthM = 5.f;
	const float far_from_route_m = math::max(request.acceptance_radius_m, kSmallLengthM);
	matrix::Vector2f desired_course_vec{};

	if (projection_context.route_projection.dist.segment_length_m < kSmallLengthM
	    || projection_context.route_projection.dist.xtrack_m > far_from_route_m) {
		get_vector_to_next_waypoint(request.vehicle_position.lat, request.vehicle_position.lon,
					    projection_context.route_projection.projection.lat,
					    projection_context.route_projection.projection.lon,
					    &desired_course_vec(0), &desired_course_vec(1));

	} else {
		const Position &segment_start = will_fly_reverse ? projection_context.route_segment_positions.end
						: projection_context.route_segment_positions.start;
		const Position &segment_end = will_fly_reverse ? projection_context.route_segment_positions.start
					      : projection_context.route_segment_positions.end;

		get_vector_to_next_waypoint(segment_start.lat, segment_start.lon,
					    segment_end.lat, segment_end.lon,
					    &desired_course_vec(0), &desired_course_vec(1));
	}

	return desired_course_vec;
}

bool uTurnRequired(const MissionResumeRequest &request, const ProjectionContext &projection_context,
		   bool will_fly_reverse)
{
	// When in MC or in back transition, no need for u-turn
	if (!(request.is_fixed_wing || request.in_transition_to_fw)) {
		return false;
	}

	// When projected onto a MC segment we will reach it in MC, no u-turn needed
	if (projection_context.projected_segment_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC) {
		return false;
	}

	const matrix::Vector2f velocity_ne{request.velocity_north_m_s, request.velocity_east_m_s};

	// Without valid vel, cannot compare current course to desired course
	if (!velocity_ne.isAllFinite()) {
		return false;
	}

	const matrix::Vector2f desired_course = computeDesiredCourseVector(request, projection_context, will_fly_reverse);

	if (!desired_course.isAllFinite()) {
		return false;
	}

	// If static, u-turn is done in place
	if (velocity_ne.norm_squared() <= FLT_EPSILON || desired_course.norm_squared() <= FLT_EPSILON) {
		return false;
	}

	// Velocity opposing the desired course requires a u-turn.
	return velocity_ne.dot(desired_course) < 0.f;
}

FirstRouteItem firstRouteItem(const SegmentEndpoint &endpoint, const Position &position)
{
	FirstRouteItem item{};
	item.index = endpoint.idx;
	item.nav_cmd = endpoint.nav_cmd;
	item.position = position;
	return item;
}

RoutePath solveShortestRoutePath(float goal_dist_along_route_m, const MissionResumeRequest &request,
				 const ProjectionContext &projection_context, PathDirectionMode direction_mode)
{
	RoutePath path{};

	const bool will_fly_reverse = mustFlyReverse(goal_dist_along_route_m,
				      projection_context.route_projection.dist.along_route_m,
				      direction_mode);
	const float abs_distance_projection_to_goal =
		fabsf(goal_dist_along_route_m - projection_context.route_projection.dist.along_route_m);

	path.direction_reversed = will_fly_reverse;
	path.total_cost_m = abs_distance_projection_to_goal
			    + (will_fly_reverse ? projection_context.u_turn_cost_reverse_m : projection_context.u_turn_cost_nominal_m);

	// Choose which segment endpoint becomes the first target for route following.
	bool choose_segment_start = false;
	const bool direction_change = (request.current_route_direction_reversed != will_fly_reverse);

	if (!direction_change
	    && isIndexInProjectionSegment(projection_context.route_projection.segment, request.mission_index,
					  request.current_route_direction_reversed)) {

		// E.g. seg [2,4] where 3 is a front transition, if we're targeting the FT (3),
		// choose item start (2) to ensure the FT is not skipped
		choose_segment_start = request.mission_index < projection_context.route_projection.segment.end.idx;

	} else {
		// Direction change or off-segment: choose start when reversing, end when nominal.
		choose_segment_start = will_fly_reverse;
	}

	const RouteProjectionCandidate &route_projection = projection_context.route_projection;
	path.first_item = choose_segment_start
			  ? firstRouteItem(route_projection.segment.start, projection_context.route_segment_positions.start)
			  : firstRouteItem(route_projection.segment.end, projection_context.route_segment_positions.end);

	return path;
}

RoutePath solveShortestRoutePathFromActiveLoop(float goal_dist_along_route_m,
		const ProjectionContext &projection_context, PathDirectionMode direction_mode)
{
	// The u-turn check uses the direction flown on the loop jump itself, not the direction
	// after leaving it.

	// Path A: complete the remaining loop distance, then continue to the goal.
	const float dist_jump_remaining = fabsf(projection_context.route_projection.dist.segment_length_m
						- projection_context.route_projection.dist.along_segment_m);
	const float path_a_cost = dist_jump_remaining
				  + fabsf(goal_dist_along_route_m - projection_context.loop_bounds.end_dist_along_route_m)
				  + projection_context.u_turn_cost_nominal_m;

	// Path B: backtrack the already-travelled loop distance, then continue to the goal.
	const float dist_jump_travelled = projection_context.route_projection.dist.along_segment_m;
	const float path_b_cost = dist_jump_travelled
				  + fabsf(goal_dist_along_route_m - projection_context.loop_bounds.start_dist_along_route_m)
				  + projection_context.u_turn_cost_reverse_m;

	// With repeats left, backtracking to the jump source would only jump back. The pass in progress
	// is not counted, so a pause on the last pass may skip it. Return ignores repeats.
	const bool use_path_a = (direction_mode == PathDirectionMode::kResume
				 && projection_context.route_projection.segment.has_remaining_repeats)
				|| path_a_cost < path_b_cost;

	const RouteProjectionCandidate &loop_edge = projection_context.route_projection;
	RoutePath path{};

	if (use_path_a) {
		path.first_item = firstRouteItem(loop_edge.segment.end, projection_context.route_segment_positions.end);
		path.direction_reversed = mustFlyReverse(goal_dist_along_route_m,
					  projection_context.loop_bounds.end_dist_along_route_m, direction_mode);
		path.total_cost_m = path_a_cost;

	} else {
		path.first_item = firstRouteItem(loop_edge.segment.start, projection_context.route_segment_positions.start);
		path.direction_reversed = mustFlyReverse(goal_dist_along_route_m,
					  projection_context.loop_bounds.start_dist_along_route_m, direction_mode);
		path.total_cost_m = path_b_cost;
	}

	return path;
}

RoutePath findShortestPathAlongRoute(float goal_dist_along_route_m, const MissionResumeRequest &request,
				     const ProjectionContext &projection_context, PathDirectionMode direction_mode,
				     GoalProjectionLocation goal_projection_location)
{
	const bool on_jump_segment_and_goal_elsewhere = projection_context.route_projection.segment.isLoop()
			&& goal_projection_location != GoalProjectionLocation::kOnActiveLoopJump;

	return on_jump_segment_and_goal_elsewhere
	       ? solveShortestRoutePathFromActiveLoop(goal_dist_along_route_m, projection_context, direction_mode)
	       : solveShortestRoutePath(goal_dist_along_route_m, request, projection_context, direction_mode);
}

void loadSafePointBatch(const Provider &provider,
			const RtlRouteRequest &request,
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

		if (!extractSafePointPosition(safe_point_item, request.home_altitude_amsl, safe_point_position)) {
			PX4_DEBUG("Route safe point %d skipped, invalid position or frame", safe_point_index);
			continue;
		}

		if (request.require_vtol_approach
		    && !hasVtolLandApproachesAtSafePointIndex(provider, safe_point_index, request.home_altitude_amsl)) {
			PX4_DEBUG("Route safe point %d skipped, no VTOL approach", safe_point_index);
			continue;
		}

		batch.items[batch.count].position = safe_point_position;
		batch.items[batch.count].source_index = safe_point_index;
		batch.count++;
	}
}

RoutePath scoreBranchOffCandidate(const RtlRouteRequest &request, const ProjectionContext &projection_context,
				  const RouteProjectionCandidate &branch_off)
{
	const Segment &vehicle_segment = projection_context.route_projection.segment;
	// The safe point scan keeps the nominal twin in place of a jump edge, so it also stands for the vehicle's own.
	const bool same_active_loop = vehicle_segment.isLoop()
				      && (branch_off.segment.jump_item_index == vehicle_segment.jump_item_index
					  || branch_off.segment.isTwin(vehicle_segment));

	// A loop edge is only reachable from the vehicle's own active loop.
	if (branch_off.segment.isLoop() && !same_active_loop) {
		return {};
	}

	const GoalProjectionLocation goal_projection_location = same_active_loop ? GoalProjectionLocation::kOnActiveLoopJump
			: GoalProjectionLocation::kOutsideActiveLoopJump;

	RoutePath path = findShortestPathAlongRoute(branch_off.dist.along_route_m, request, projection_context,
			 PathDirectionMode::kAuto, goal_projection_location);

	if (!path.valid()) {
		return {};
	}

	path.total_cost_m += branch_off.dist.xtrack_m;
	return path;
}

// Fold one projected batch into selection, which keeps the cheapest safe point; the first one wins a tie.
void selectLowestCostSafePoint(const RtlRouteRequest &request, const ProjectionContext &projection_context,
			       const ProjectionReferenceBatch &batch, GoalSelection &selection)
{
	for (uint8_t batch_index = 0; batch_index < batch.count; ++batch_index) {
		const ProjectionReference &reference = batch.items[batch_index];
		const ProjectionCandidateBuffer &candidate_buffer = reference.candidate_buffer;

		for (uint8_t candidate_index = 0; candidate_index < candidate_buffer.count; ++candidate_index) {
			const RouteProjectionCandidate &branch_off = candidate_buffer.candidates[candidate_index];
			const RoutePath path = scoreBranchOffCandidate(request, projection_context, branch_off);

			if (path.valid() && path.total_cost_m < selection.path.total_cost_m) {
				selection.goal_type = GoalType::kSafePoint;
				selection.goal_position = reference.position;
				selection.safe_point_index = reference.source_index;
				selection.path = path;
				selection.branch_off.segment = branch_off.segment;
				selection.branch_off.projection = branch_off.projection;
			}
		}
	}
}

// selection must start empty and is only meaningful when kNone is returned.
FailureReason selectSafePoint(const Provider &provider,
			      const MissionRouteProjection &projection,
			      const RtlRouteRequest &request,
			      const ProjectionContext &projection_context,
			      ProjectionReferenceBatch &batch,
			      GoalSelection &selection)
{
	const int safe_point_count = provider.safePointCount();

	if (safe_point_count <= 0) {
		return FailureReason::kNoValidSafePoints;
	}

	ProjectionScanRequest scan_request{};
	scan_request.home_altitude_amsl = request.home_altitude_amsl;
	scan_request.xtrack_margin_m = request.safe_point_projection_search_distance_m;
	// Every jump edge is projected, flown or not, so the candidates match the pre-flight projections.
	// Scoring rejects a jump edge the vehicle is not on.

	FailureReason scan_failure_reason = FailureReason::kNoValidCandidateFound;
	int safe_point_cursor = 0;

	// The branch-in point is already chosen: project safe points in batches and score the paths from it.
	while (safe_point_cursor < safe_point_count) {
		// Always advances the cursor, so every pass makes progress.
		loadSafePointBatch(provider, request, safe_point_cursor, batch);

		if (batch.count == 0) {
			continue;
		}

		RouteDistanceSummary distance_summary{};
		const FailureReason scan_status = projection.findProjectionCandidates(scan_request, batch, distance_summary);

		if (scan_status == FailureReason::kGeofenceUnavailable) {
			return scan_status;
		}

		if (scan_status != FailureReason::kNone) {
			scan_failure_reason = scan_status;
			continue;
		}

		selectLowestCostSafePoint(request, projection_context, batch, selection);
	}

	return selection.found() ? FailureReason::kNone : scan_failure_reason;
}

// The takeoff endpoint is the first position item, if it is a takeoff.
bool findMissionTakeoffPosition(const Provider &provider, float home_altitude_amsl, Position &position)
{
	for (int i = 0; i < provider.missionCount(); ++i) {
		mission_item_s item{};

		if (!provider.loadMissionItem(i, item)) {
			return false;
		}

		if (mission_item_contains_position(item)) {
			return isTakeoffCmd(item.nav_cmd) && extractMissionPosition(item, home_altitude_amsl, position);
		}
	}

	return false;
}

// Fill the empty selection with the mission takeoff or landing, whichever costs less along the route.
void selectMissionEndpointFallback(const Provider &provider,
				   const RtlRouteRequest &request,
				   const ProjectionContext &projection_context,
				   GoalSelection &selection)
{
	const float home_altitude_amsl = request.home_altitude_amsl;
	const int32_t mission_land_index = request.mission_land_index;
	Position takeoff_position{};

	// Takeoff carries the climb altitude, so its location is only a usable return goal
	// when the caller also supplies the home altitude for the destination.
	if (PX4_ISFINITE(home_altitude_amsl) && findMissionTakeoffPosition(provider, home_altitude_amsl, takeoff_position)) {
		takeoff_position.alt = home_altitude_amsl;
		selection.goal_type = GoalType::kMissionTakeoff;
		selection.goal_position = takeoff_position;
		selection.path = findShortestPathAlongRoute(0.f, request, projection_context, PathDirectionMode::kForceReverse,
				 GoalProjectionLocation::kOutsideActiveLoopJump);
	}

	mission_item_s land_item{};
	Position land_position{};

	// route_end_index is a valid item index, so it also bounds mission_land_index.
	if (mission_land_index == projection_context.route_end_index
	    && provider.loadMissionItem(mission_land_index, land_item)
	    && isLandingCmd(land_item.nav_cmd)
	    && extractMissionPosition(land_item, home_altitude_amsl, land_position)) {
		const RoutePath path_to_land = findShortestPathAlongRoute(projection_context.route_length, request,
					       projection_context, PathDirectionMode::kForceNominal,
					       GoalProjectionLocation::kOutsideActiveLoopJump);

		// The landing wins a tie.
		if (!selection.found() || path_to_land.total_cost_m <= selection.path.total_cost_m) {
			selection.goal_type = GoalType::kMissionLand;
			selection.goal_position = land_position;
			selection.path = path_to_land;
		}
	}
}

// Lowest-cost safe point, else the cheaper mission endpoint.
FailureReason selectBestGoal(const Provider &provider,
			     const MissionRouteProjection &projection,
			     const RtlRouteRequest &request,
			     const ProjectionContext &projection_context,
			     ProjectionReferenceBatch &batch,
			     GoalSelection &selection)
{
	const FailureReason safe_point_status = selectSafePoint(provider, projection, request, projection_context, batch,
						selection);

	if (safe_point_status == FailureReason::kNone || safe_point_status == FailureReason::kGeofenceUnavailable) {
		return safe_point_status;
	}

	selectMissionEndpointFallback(provider, request, projection_context, selection);
	return selection.found() ? FailureReason::kNone : safe_point_status;
}

// Landing items, and takeoff items flown in reverse, sit at ground level: keep the vehicle
// altitude when it is already inside the item's acceptance radius.
bool canUseCurrentAltitudeForJoinTarget(const RoutePath &path, const Position &vehicle_position,
					float acceptance_radius_m)
{
	if (!isLandingCmd(path.first_item.nav_cmd)
	    && !(path.direction_reversed && isTakeoffCmd(path.first_item.nav_cmd))) {
		return false;
	}

	const float dist_to_first_item = get_distance_to_next_waypoint(path.first_item.position.lat,
					 path.first_item.position.lon, vehicle_position.lat, vehicle_position.lon);
	return PX4_ISFINITE(dist_to_first_item) && dist_to_first_item < acceptance_radius_m;
}

// With use_current_altitude, callers may skip the branch-in point and fly level to the first item. Candidate
// pruning only checked the segment to the branch-in point, so check this one too. It ends above the item, at the
// landing leg's altitude.
FailureReason checkCurrentAltitudeJoin(const PathChecker &path_checker, const Position &vehicle_position,
				       const RoutePath &path)
{
	bool clear = false;

	if (!path_checker.checkPath(vehicle_position, path.first_item.position, clear)) {
		return FailureReason::kGeofenceUnavailable;
	}

	return clear ? FailureReason::kNone : FailureReason::kNoValidPath;
}

bool closeToBranchOffSegment(const Position &position,
			     const GoalSelection &selection,
			     float acceptance_radius_m,
			     float altitude_acceptance_radius_m)
{
	const Position &branch_off_projection = selection.branch_off.projection;

	// Compare the vehicle and branch-off leg in the same local north/east frame.
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

	// Callers must fly the branch-off leg level at the branch-off altitude. A vehicle at another
	// altitude must rejoin the route before taking this shortcut.
	const float alt_error = fabsf(position.alt - branch_off_projection.alt);

	return PX4_ISFINITE(alt_error) && alt_error < altitude_acceptance_radius_m;
}

bool canSkipRouteFollowToSelectedGoal(const RtlRouteRequest &request, const GoalSelection &selection)
{
	const Position &vehicle_position = request.vehicle_position;

	if (selection.goal_type != GoalType::kSafePoint) {
		return canUseCurrentAltitudeForJoinTarget(selection.path, vehicle_position, request.acceptance_radius_m);
	}

	// Safe-point goals only skip when the selected destination or branch-off leg is already close.
	const float dist_to_goal_m = get_distance_to_next_waypoint(vehicle_position.lat, vehicle_position.lon,
				     selection.goal_position.lat, selection.goal_position.lon);
	const bool close_to_safe_point_direct = PX4_ISFINITE(dist_to_goal_m)
						&& dist_to_goal_m < request.direct_goal_acceptance_radius_m;

	return close_to_safe_point_direct
	       || closeToBranchOffSegment(vehicle_position, selection, request.acceptance_radius_m,
					  request.altitude_acceptance_radius_m);
}

// Plans carry the jump identity only when the vehicle is projected onto a valid loop.
ActiveJumpAnchor activeJumpAnchor(const Segment &segment)
{
	if (segment.isLoop()) {
		return {segment.jump_item_index};
	}

	return {};
}

void fillRouteJoinPlan(MissionResumePlan &plan, const MissionResumeRequest &request,
		       const ProjectionContext &projection_context, const RoutePath &path)
{
	plan.join_position = projection_context.route_projection.projection;
	plan.first_mission_item_index = path.first_item.index;
	plan.direction_reversed = path.direction_reversed;
	plan.use_current_altitude = canUseCurrentAltitudeForJoinTarget(path, request.vehicle_position,
				    request.acceptance_radius_m);
	plan.active_jump_anchor = activeJumpAnchor(projection_context.route_projection.segment);
	plan.vtol_transition_action = request.is_vtol
				      ? vtolTransitionActionForState(projection_context.projected_segment_vtol_state,
						      request.is_fixed_wing || request.in_transition_to_fw)
				      : VtolTransitionAction::kNone;

	if (plan.use_current_altitude) {
		plan.join_position.alt = request.vehicle_position.alt;
	}
}

// First stage of both plans: project the vehicle, resolve the segment's VTOL state and the u-turn costs.
FailureReason buildProjectionContext(const Provider &provider, const MissionResumeRequest &request,
				     ProjectionReferenceBatch &batch, ProjectionContext &projection_context,
				     const PathChecker *path_checker)
{
	const MissionRouteProjection projection{provider, path_checker};
	const FailureReason projection_status = projection.collectVehicleProjection(request, batch, projection_context);

	if (projection_status != FailureReason::kNone) {
		return projection_status;
	}

	// Only VTOLs read the segment state: the u-turn check and plan assembly use it.
	if (request.is_vtol) {
		projection_context.projected_segment_vtol_state = vtolStateForSegment(provider,
				projection_context.route_projection.segment);
	}

	// The u-turn check only depends on the direction flown from the projection.
	const float u_turn_penalty_m = request.fw_u_turn_penalty_m;
	projection_context.u_turn_cost_nominal_m = uTurnRequired(request, projection_context, false) ? u_turn_penalty_m : 0.f;
	projection_context.u_turn_cost_reverse_m = uTurnRequired(request, projection_context, true) ? u_turn_penalty_m : 0.f;

	return FailureReason::kNone;
}

} // namespace

MissionRoutePlanner::MissionRoutePlanner(const Provider &provider, const PathChecker *path_checker) :
	_provider(provider), _path_checker(path_checker)
{
}

FailureReason MissionRoutePlanner::planMissionResumeJoin(const MissionResumeRequest &request,
		MissionResumePlan &plan) const
{
	plan = {};

	if (!validJoinRequest(request)) {
		return FailureReason::kInvalidRequest;
	}

	ProjectionContext projection_context{};
	const FailureReason projection_status = buildProjectionContext(_provider, request, plannerScratchBatch(),
						projection_context, _path_checker);

	if (projection_status != FailureReason::kNone) {
		return projection_status;
	}

	const RoutePath path = findShortestPathAlongRoute(projection_context.route_length, request, projection_context,
			       PathDirectionMode::kResume, GoalProjectionLocation::kOutsideActiveLoopJump);
	MissionResumePlan candidate{};
	fillRouteJoinPlan(candidate, request, projection_context, path);

	if (!candidate.valid()) {
		return FailureReason::kNoValidPath;
	}

	if (_path_checker && candidate.use_current_altitude) {
		const FailureReason join_status = checkCurrentAltitudeJoin(*_path_checker, request.vehicle_position, path);

		if (join_status != FailureReason::kNone) {
			return join_status;
		}
	}

	plan = candidate;
	return FailureReason::kNone;
}

FailureReason MissionRoutePlanner::planRtlRoute(const RtlRouteRequest &request,
		RtlRoutePlan &plan) const
{
	plan = {};

	if (!validRtlRequest(request)) {
		return FailureReason::kInvalidRequest;
	}

	ProjectionReferenceBatch &reference_batch = plannerScratchBatch();
	ProjectionContext projection_context{};
	const FailureReason projection_status = buildProjectionContext(_provider, request, reference_batch,
						projection_context, _path_checker);

	if (projection_status != FailureReason::kNone) {
		return projection_status;
	}

	// Find the lowest-cost safe point, falling back to mission endpoints if none is usable.
	const MissionRouteProjection projection{_provider, _path_checker};
	GoalSelection selection{};
	const FailureReason selection_status = selectBestGoal(_provider, projection, request, projection_context,
					       reference_batch, selection);

	if (selection_status != FailureReason::kNone) {
		return selection_status;
	}

	bool fly_direct_to_goal = canSkipRouteFollowToSelectedGoal(request, selection);
	bool geofence_prevents_shortcut = false;

	if (_path_checker && fly_direct_to_goal) {
		// Pruning checked vehicle-to-branch-in and branch-off-to-safe-point segments.
		// Flying straight to the goal skips those segments and needs its own check.
		bool clear = false;

		if (!_path_checker->checkPath(request.vehicle_position, selection.goal_position, clear)) {
			return FailureReason::kGeofenceUnavailable;
		}

		if (!clear && selection.goal_type != GoalType::kSafePoint) {
			// The branch-in point near this landing/takeoff endpoint is already within
			// acceptance radius. Restoring it would still skip straight to the endpoint.
			return FailureReason::kNoValidPath;
		}

		// Keep the checked route if the direct segment fails, and expose the reason
		// so callers can report it when using this plan.
		geofence_prevents_shortcut = !clear;
		fly_direct_to_goal = clear;
	}

	fillRouteJoinPlan(plan, request, projection_context, selection.path);
	plan.goal_type = selection.goal_type;
	plan.goal_position = selection.goal_position;
	plan.safe_point_index = selection.safe_point_index;
	plan.fly_direct_to_goal = fly_direct_to_goal;
	plan.geofence_prevents_shortcut = geofence_prevents_shortcut;

	if (selection.goal_type == GoalType::kSafePoint) {
		// The branch-off point replaces the item flown toward on its segment.
		const Segment &branch_off_segment = selection.branch_off.segment;
		plan.branch_off_position = selection.branch_off.projection;
		plan.branch_off_mission_item_index = selection.path.direction_reversed
						     ? branch_off_segment.start.idx : branch_off_segment.end.idx;
	}

	if (!plan.valid()) {
		plan = {};
		return FailureReason::kNoValidPath;
	}

	// A direct flight to the goal was checked above.
	if (_path_checker && plan.use_current_altitude && !plan.fly_direct_to_goal) {
		const FailureReason join_status = checkCurrentAltitudeJoin(*_path_checker, request.vehicle_position,
						  selection.path);

		if (join_status != FailureReason::kNone) {
			plan = {};
			return join_status;
		}
	}

	return FailureReason::kNone;
}
