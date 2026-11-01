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
 * @file mission_route_internal_types.h
 *
 * Internal route-scanning and goal-selection data, outside the planner's public interface.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "mission_route_types.h"

#include <float.h>

#include <dataman/dataman.h>
#include <uORB/topics/vtol_vehicle_status.h>

namespace mission_route
{

static constexpr float kRoundingToleranceM{0.1f};
static constexpr double kCornerLatLonTolDeg{1e-5};
/** Closest projections kept per point. The fence check runs before the search margin is applied. */
static constexpr uint8_t kMaxSegmentCandidates{4};

/** Safe points projected per route walk: larger batches trade fixed RAM for fewer walks. */
static constexpr uint8_t kMaxSafePointBatch{CONFIG_NAVIGATOR_SAFE_POINT_BATCH_SIZE};
static_assert(kMaxSafePointBatch >= 16 && kMaxSafePointBatch <= DM_KEY_SAFE_POINTS_MAX,
	      "CONFIG_NAVIGATOR_SAFE_POINT_BATCH_SIZE must be 16..DM_KEY_SAFE_POINTS_MAX");

/** @brief Key distances for a point projected onto one segment. */
struct ProjectionDistance {
	float xtrack_m{NAN}; /**< Cross-track distance from the reference point to the projection. */
	float along_route_m{NAN}; /**< Along-route distance from the route start to the projection. */
	float segment_length_m{NAN};
	float along_segment_m{NAN}; /**< Along-segment distance from the segment start to the projection. */

	bool valid() const;
};

/** @brief One mission endpoint used to build route segments. */
struct SegmentEndpoint {
	int32_t idx{-1};
	uint16_t nav_cmd{NAV_CMD_INVALID};

	bool valid() const;
};

/** @brief The start and end positions corresponding to one route segment. */
struct SegmentPositions {
	Position start{};
	Position end{};

	bool valid() const;
};

/** @brief One mission segment, optionally representing a synthetic DO_JUMP edge. */
struct Segment {
	SegmentEndpoint start{};
	SegmentEndpoint end{};
	int32_t jump_item_index{-1}; /**< DO_JUMP command index; -1 for a nominal segment. */
	bool has_remaining_repeats{false};

	bool isLoop() const { return jump_item_index >= 0; }
	bool valid() const;
	bool validLoop() const;

	/** A jump edge and a nominal segment with the same endpoints: one leg, flown with or without the jump. */
	bool isTwin(const Segment &other) const
	{
		return isLoop() != other.isLoop() && start.idx == other.start.idx && end.idx == other.end.idx;
	}
};

/** @brief The along-track interval of one segment within the full route. */
struct SegmentDistanceAlong {
	float start_dist_along_route_m{NAN};
	float end_dist_along_route_m{NAN};

	bool valid() const;
};

/** @brief One projected route candidate retained during a mission scan. */
struct RouteProjectionCandidate {
	Segment segment{};
	SegmentPositions segment_positions{};
	Position projection{};
	ProjectionDistance dist{};

	bool valid() const;
};

/** @brief Fixed-size cross-track sorted buffer of projection candidates for one reference point. */
struct ProjectionCandidateBuffer {
	RouteProjectionCandidate candidates[kMaxSegmentCandidates] {};
	uint8_t count{0};
};

/** @brief Internal vehicle projection and route state used for scoring. */
struct ProjectionContext {
	RouteProjectionCandidate route_projection{};
	float route_length{0.f};
	int32_t route_end_index{-1};
	SegmentDistanceAlong loop_bounds{}; /**< Along-route distances of both exits when route_projection is on a loop. */
	/** Required VTOL state on the vehicle's projected segment, resolved once per planning pass. */
	uint8_t projected_segment_vtol_state{vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED};
	/** U-turn cost of following the route nominally or in reverse, resolved once per planning pass. */
	float u_turn_cost_nominal_m{0.f};
	float u_turn_cost_reverse_m{0.f};
};

/** @brief First route item to target when the vehicle starts following the path. */
struct FirstRouteItem {
	int32_t index{-1};
	uint16_t nav_cmd{NAV_CMD_INVALID};
	Position position{};
};

/** @brief One scored route path to a goal. */
struct RoutePath {
	bool direction_reversed{false}; /**< True when flying towards the mission start (takeoff) */
	FirstRouteItem first_item{};
	float total_cost_m{FLT_MAX};

	bool valid() const;
};

/** @brief The branch-off is the point on the route where the UAV leaves it towards its goal (e.g. rally point). */
struct BranchOff {
	Segment segment{};
	Position projection{};
};

/** @brief Internal winning goal and the scored path used to reach it. */
struct GoalSelection {
	GoalType goal_type{GoalType::kNone};
	Position goal_position{};
	int32_t safe_point_index{-1};
	RoutePath path{};
	BranchOff branch_off{};

	bool found() const { return goal_type != GoalType::kNone; }
};

} // namespace mission_route
