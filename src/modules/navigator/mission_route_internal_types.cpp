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
 * @file mission_route_internal_types.cpp
 *
 * Planner-internal type validation.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_internal_types.h"

namespace mission_route
{

bool ProjectionDistance::valid() const
{
	return PX4_ISFINITE(xtrack_m) && xtrack_m >= 0.f
	       && PX4_ISFINITE(route_along_m) && route_along_m >= 0.f
	       && PX4_ISFINITE(segment_length_m) && segment_length_m >= 0.f
	       && PX4_ISFINITE(along_segment_m) && along_segment_m >= 0.f
	       && along_segment_m < (segment_length_m + kRoundingToleranceM);
}

bool SegmentEndpoint::valid() const
{
	return idx >= 0 && nav_cmd != NAV_CMD_INVALID;
}

bool SegmentPositions::valid() const
{
	return start.valid() && end.valid();
}

bool Segment::valid() const
{
	const bool valid_jump = jump_item_index >= 0
				&& jump_item_index != start.idx
				&& jump_item_index != end.idx;
	const bool valid_nominal = jump_item_index == -1 && !has_remaining_repeats && start.idx < end.idx;

	return start.valid() && end.valid() && start.idx != end.idx && (valid_jump || valid_nominal);
}

bool Segment::validLoop() const
{
	return isLoop() && valid();
}

bool SegmentDistanceAlong::valid() const
{
	return PX4_ISFINITE(route_start_dist_m) && route_start_dist_m > -FLT_EPSILON
	       && PX4_ISFINITE(route_end_dist_m) && route_end_dist_m > -FLT_EPSILON;
}

bool RouteProjectionCandidate::valid() const
{
	return segment.valid() && segment_positions.valid() && projection.valid() && dist.valid();
}

bool LoopContext::valid() const
{
	return segment.validLoop() && along.valid() && segment_positions.valid();
}

bool ProjectionContext::valid() const
{
	return vehicle_position.valid() && route_projection.valid();
}

bool RoutePath::valid() const
{
	return first_item_index >= 0 && first_item_cmd != NAV_CMD_INVALID
	       && PX4_ISFINITE(total_cost_m) && total_cost_m >= 0.f;
}

bool BranchOff::valid() const
{
	return segment.valid() && projection.valid();
}

bool GoalSelection::valid() const
{
	if (goal_type == GoalType::kNone || !path.valid() || !goal_position.valid()) {
		return false;
	}

	if (goal_type == GoalType::kSafePoint) {
		return safe_point_index >= 0 && branch_off.valid();
	}

	return goal_type == GoalType::kMissionLand || goal_type == GoalType::kMissionTakeoff;
}

int32_t GoalSelection::branchOffIndex() const
{
	if (branch_off.segment.valid()) {
		return path.direction_reversed ? branch_off.segment.start.idx : branch_off.segment.end.idx;
	}

	return path.first_item_index;
}

bool PlannerParameters::validForVehicleProjection() const
{
	// home_altitude_amsl is intentionally not checked: NAN is valid for absolute-altitude missions.
	return PX4_ISFINITE(vehicle_projection_search_dist_m) && vehicle_projection_search_dist_m >= 0.f
	       && PX4_ISFINITE(acceptance_radius_m) && acceptance_radius_m >= 0.f
	       && PX4_ISFINITE(u_turn_penalty_m) && u_turn_penalty_m >= 0.f;
}

bool PlannerParameters::validForRouteToGoal() const
{
	return validForVehicleProjection()
	       && PX4_ISFINITE(safe_point_projection_search_dist_m) && safe_point_projection_search_dist_m >= 0.f
	       && PX4_ISFINITE(direct_acceptance_radius_m) && direct_acceptance_radius_m >= 0.f
	       && PX4_ISFINITE(altitude_acceptance_radius_m) && altitude_acceptance_radius_m >= 0.f;
}

} // namespace mission_route
