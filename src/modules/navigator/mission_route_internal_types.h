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
 * Internal route-scanning and goal-selection data. None of these types are part of the planner's consumer contract.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "mission_route_types.h"

#include <float.h>

#include <dataman/dataman.h>
#include <matrix/math.hpp>

namespace mission_route
{

static constexpr float kRoundingToleranceM{0.1f};
static constexpr double kNullIslandThresholdDeg{1e-7};
static constexpr double kCornerLatLonTolDeg{1e-5};
static constexpr uint8_t kMaxSegmentCandidates{3};

/**
 * Number of safe points projected during one mission scan pass.
 *
 * Larger batches trade RAM for fewer complete mission scans. The value remains an implementation detail until
 * sequential safe-point evaluation is benchmarked in the memory-ownership step.
 */
static constexpr uint8_t kMaxSafePointBatch{CONFIG_NAVIGATOR_SAFE_POINT_BATCH_SIZE};
static_assert(CONFIG_NAVIGATOR_SAFE_POINT_BATCH_SIZE >= 1 && CONFIG_NAVIGATOR_SAFE_POINT_BATCH_SIZE <= 255,
	      "CONFIG_NAVIGATOR_SAFE_POINT_BATCH_SIZE must fit the uint8_t batch counters");
static_assert(kMaxSafePointBatch <= DM_KEY_SAFE_POINTS_MAX,
	      "CONFIG_NAVIGATOR_SAFE_POINT_BATCH_SIZE must not exceed DM_KEY_SAFE_POINTS_MAX");

/** @brief Key distances for a point projected onto one segment. */
struct ProjectionDistance {
	float xtrack{NAN}; /**< Cross-track distance from the reference point to the projection. */
	float route_along{NAN}; /**< Along-route distance from the route start to the projection. */
	float segment_length{NAN};
	float segment_along{NAN}; /**< Along-segment distance from the segment start to the projection. */

	bool valid() const;
};

/** @brief One mission endpoint used to build route segments. */
struct SegmentEndpoint {
	int32_t idx{-1};
	uint16_t nav_cmd{NAV_CMD_INVALID};

	bool valid() const;
};

/** @brief The start and end world positions corresponding to one route segment. */
struct SegmentPositions {
	Position start{};
	Position end{};

	bool valid() const;
};

/** @brief One mission segment, optionally representing a synthetic DO_JUMP edge. */
struct Segment {
	SegmentEndpoint start{};
	SegmentEndpoint end{};
	bool is_loop{false};
	uint8_t loops_remaining{0}; /**< Remaining repeats loaded from the DO_JUMP; zero for nominal segments. */

	bool valid() const;
	bool validLoop() const;
};

/** @brief The along-track interval of one segment within the full route. */
struct SegmentDistanceAlong {
	float start{NAN};
	float end{NAN};

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

/** @brief Fixed-size cross-track-sorted buffer of projection candidates for one reference point. */
struct ProjectionCandidateBuffer {
	RouteProjectionCandidate candidates[kMaxSegmentCandidates] {};
	uint8_t count{0};
};

/** @brief Active DO_JUMP geometry and along-route interval used while solving a path. */
struct LoopContext {
	Segment segment{};
	SegmentPositions segment_positions{};
	SegmentDistanceAlong along{};

	bool valid() const;
};

/** @brief Live vehicle kinematics and mode flags copied into one planning pass. */
struct VehicleStateContext {
	bool is_flying_reverse{false};
	matrix::Vector2f velocity_ne{NAN, NAN};
	bool velocity_valid{false};
	bool is_fixed_wing{false};
	bool in_transition_to_fw{false};
	bool require_vtol_approach{false};
};

/** @brief Internal vehicle projection and route state used for scoring. */
struct ProjectionContext {
	Position vehicle_position{};
	int32_t mission_index{-1};
	RouteProjectionCandidate route_projection{};
	VehicleStateContext vehicle_state{};
	float route_length{0.f};
	uint8_t mission_loops_remaining{0};
	LoopContext loop_context{};

	bool valid() const;
};

/** @brief Internal join geometry before it is flattened into a consumer plan. */
struct JoinContext {
	Position projection{};
	bool use_current_altitude{false};

	bool valid() const;
};

/** @brief One scored route path to a goal. */
struct RoutePath {
	bool direction_reversed{false};
	bool u_turn_required{false};
	bool in_first_item_acc_rad{false};
	int32_t first_item_index{-1};
	uint16_t first_item_cmd{NAV_CMD_INVALID};
	Position first_item_position{};
	float total_cost_m{FLT_MAX};

	bool valid() const;
};

/** @brief Internal safe-point branch-off geometry. */
struct BranchOff {
	Segment segment{};
	Position projection{};

	bool valid() const;
};

/** @brief Internal winning goal and the scored path used to reach it. */
struct GoalSelection {
	GoalType goal_type{GoalType::kNone};
	Position goal_position{};
	int32_t safe_point_index{-1};
	RoutePath path{};
	BranchOff branch_off{};
	bool fly_direct_to_goal{false};

	bool found() const { return goal_type != GoalType::kNone; }
	bool valid() const;
	int32_t branchOffIndex() const;
};

/** @brief Normalized parameters shared by projection and goal-selection helpers during one planning pass. */
struct PlannerParameters {
	float vehicle_projection_search_dist{0.f};
	float safe_point_projection_search_dist{0.f};
	float acceptance_radius{0.f};
	float direct_acceptance_radius{0.f};
	float altitude_acceptance_radius{0.f};
	float home_altitude_amsl{NAN};
	float u_turn_penalty_m{4000.f};

	bool validForVehicleProjection() const;
	bool validForRouteToGoal() const;
};

struct PlannerConfig {
	PlannerParameters parameters{};
	VehicleStateContext state{};
	ActiveJumpAnchor active_jump_anchor{};
};

} // namespace mission_route
