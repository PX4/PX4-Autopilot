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
 * @file mission_route_types.h
 *
 * Mission-route planner data types.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "navigation.h"
#include "safe_point_land.hpp"

#include <float.h>
#include <math.h>
#include <stdint.h>

#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>

namespace mission_route
{

static constexpr float kRoundingToleranceM{0.1f};
static constexpr double kNullIslandThresholdDeg{1e-7};
static constexpr double kCornerLatLonTolDeg{1e-5};
static constexpr float kLandApproachAssociationDistanceM{10.f};
static constexpr uint8_t kMaxSegmentCandidates{3};
/**
 * Number of rally (safe) points evaluated per mission-route scan pass.
 *
 * The planner scores rally points using a fixed-size projection batch buffer of this size. When more
 * eligible rally points are configured than fit in one batch, they are processed in successive batches:
 * the full mission route is re-scanned once per batch. A larger value trades RAM for fewer mission passes.
 *
 * Stored in uint8_t batch counters/indices, so it must stay in [1, 255].
 */
static constexpr uint8_t kMaxSafePointBatch{CONFIG_RTL_SAFE_POINT_BATCH_SIZE};
static_assert(CONFIG_RTL_SAFE_POINT_BATCH_SIZE >= 1 && CONFIG_RTL_SAFE_POINT_BATCH_SIZE <= 255,
	      "CONFIG_RTL_SAFE_POINT_BATCH_SIZE must be in [1, 255] to fit the uint8_t batch counters");

enum class GoalType : uint8_t {
	kNone = 0,
	kSafePoint,
	kMissionLand,
	kMissionTakeoff
};

enum class FailureReason : uint8_t {
	kNone = 0,
	kNoValidGlobalPos,
	kInvalidRequest,
	kNoValidWaypoints,
	kNoValidSafePoints,
	kNoValidPath,
	kNoSegmentsFound,
	kInternalError,
	kLoadFailed,
	kInvalidProjectionContext,
	kNoLocalMinFound,
	kPositionItemInvalid,
	kNoValidCandidateFound,
	kUnknown
};

enum class VtolTransitionAction : uint8_t {
	kNone = 0,
	kFrontTransition = 1,
	kBackTransition = 2
};

/** @brief A global geographic coordinate used by the route planner. */
struct Position {
	double lat{static_cast<double>(NAN)};
	double lon{static_cast<double>(NAN)};
	float alt{NAN};

	bool valid() const;
};

/** @brief Key distances for a point projected onto one segment. */
struct ProjectionDistance {
	float xtrack{NAN};        /**< Cross-track distance from the reference point to the projection. */
	float route_along{NAN};   /**< Along-route distance from the route start to the projection. */
	float segment_length{NAN};
	float segment_along{NAN};  /**< Along-segment distance from the segment start to the projection. */

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

/**
 * @brief One mission segment, optionally representing a DO_JUMP loop.
 */
struct Segment {
	SegmentEndpoint start{};
	SegmentEndpoint end{};
	bool is_loop{false}; /**< True when this is the synthetic DO_JUMP edge from the attached position to the jump target. */
	uint8_t loops_remaining{0}; /**< Remaining repeats on the DO_JUMP that created this segment; zero for nominal segments. */

	bool valid() const;
	bool validLoop() const;
};

/** @brief The along-track distance of one segment within the full route. */
struct SegmentDistanceAlong {
	float start{NAN};
	float end{NAN};

	bool valid() const;
};

/** @brief One projected route candidate. */
struct RouteProjectionCandidate {
	Segment segment{};
	SegmentPositions segment_positions{};
	Position projection{};
	ProjectionDistance dist{};

	bool valid() const;
};

/** @brief Fixed-size xtrack-sorted buffer of the best projection candidates for one point. */
struct ProjectionCandidateBuffer {
	RouteProjectionCandidate candidates[kMaxSegmentCandidates] {};
	uint8_t count{0};
};

/** @brief Active DO_JUMP loop state carried across replans to preserve mission continuity. */
struct LoopContext {
	Segment segment{};
	SegmentPositions segment_positions{};
	SegmentDistanceAlong along{};

	bool valid() const;
};

/** @brief The vehicle's projected state on the route at the time planning starts. */
struct ProjectionContext {
	Position vehicle_position{};
	int32_t mission_index{-1};
	RouteProjectionCandidate route_projection{};
	bool is_flying_reverse{false};
	matrix::Vector2f vehicle_vel_ne{NAN, NAN};
	bool velocity_valid{false};
	float route_length{0.f};
	uint8_t mission_loops_remaining{0};
	LoopContext loop_context{};

	bool valid() const;
};

/** @brief How the executor should join the nominal route from the current vehicle state. */
struct JoinContext {
	Position projection{};
	bool direction_reversed{false};
	bool skip_altitude_requirement{false}; /**< Keep the join waypoint at live vehicle altitude. */
	VtolTransitionAction transition_action{VtolTransitionAction::kNone};

	bool valid() const;
};

/** @brief One computed route to a goal: direction, the first route item to target, and its location. */
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

/** @brief The winning goal selection, including route, branch-off point, and goal position. */
struct GoalSelection {
	RoutePath path{};
	bool found{false};
	bool safe_point_found{false};
	bool skip_route_to_safe_point{false}; /**< Go straight to the selected goal. */
	int32_t safe_point_index{-1};
	GoalType goal_type{GoalType::kNone};
	Segment branch_off_segment{};
	Position branch_off_projection{};
	Position safe_point_position{};
	Position goal_position{};

	bool valid() const;

	int32_t branchOffIndex() const;
};

/** @brief Planner tuning values and mission-global inputs used during one planning pass. */
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

/** @brief Live vehicle kinematics and mode flags relevant to route planning. */
struct VehicleStateContext {
	bool is_flying_reverse{false};
	matrix::Vector2f velocity_ne{NAN, NAN};
	bool velocity_valid{false};
	bool is_fixed_wing{false};
	bool in_transition_to_fw{false};
	bool require_vtol_approach{false};
};

/** @brief One self-contained planner input bundle combining parameters, state, and persisted loop context. */
struct PlannerConfig {
	PlannerParameters parameters{};
	VehicleStateContext state{};
	Segment last_flown_loop_segment{}; /**< Cached DO_JUMP edge anchor when preserving an active loop. */
};

/** @brief A single planning request: the vehicle state, its mission index, and the planner configuration. */
struct RoutePlanRequest {
	Position vehicle_position{};
	int32_t mission_index{-1};
	PlannerConfig config{};
};

/** @brief A mission-resume join plan composed of projection, route path, and join details. */
struct JoinPlan {
	ProjectionContext projection_context{};
	RoutePath path{};
	JoinContext join_context{};

	bool valid() const;
};

/** @brief The full planner output returned for route safe point RTL. */
struct RoutePlan {
	ProjectionContext projection_context{};
	JoinContext join_context{};
	GoalSelection selection{};

	bool valid() const;
};

/** @brief Result of a vehicle-projection pass: status plus the projected context. */
struct VehicleProjectionResult {
	bool success{false};
	FailureReason failure_reason{FailureReason::kUnknown};
	ProjectionContext projection_context{};
};

/** @brief Result of a mission-resume join pass: status plus the join plan. */
struct JoinPlanResult {
	bool success{false};
	FailureReason failure_reason{FailureReason::kUnknown};
	JoinPlan plan{};
};

/** @brief Result of a route-to-goal pass: status plus the full route plan. */
struct RoutePlanResult {
	bool success{false};
	FailureReason failure_reason{FailureReason::kUnknown};
	RoutePlan plan{};
};

bool isLandingCmd(uint16_t nav_cmd);
bool isTakeoffCmd(uint16_t nav_cmd);

float getAbsoluteAltitudeForMissionItem(const mission_item_s &mission_item, float home_altitude_amsl);

/** @brief Extract the attached valid position from a mission item, if it carries one. */
bool extractMissionPosition(const mission_item_s &mission_item, float home_altitude_amsl, Position &position);

/** @brief Extract the valid position from a rally-point safe-point item. */
bool extractSafePointPosition(const mission_item_s &safe_point_item, float home_altitude_amsl, Position &position);

/** @brief Copy a valid Position into a PositionYawSetpoint (yaw left unset). Returns false for an invalid position. */
bool copyPositionToYawSetpoint(const Position &position, PositionYawSetpoint &setpoint);

/** @brief Convert one LOITER_TO_ALT safe-point item into a concrete landing-approach point. */
loiter_point_s makeVtolLandApproachPoint(const mission_item_s &mission_item, float home_altitude_amsl);

const char *failureReasonString(FailureReason failure_reason);
const char *goalTypeString(GoalType goal_type);

} // namespace mission_route
