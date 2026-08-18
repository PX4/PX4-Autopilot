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
 * Public MissionRoutePlanner requests, plans, status, and shared value types.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "navigation.h"
#include "safe_point_land.hpp"

#include <math.h>
#include <stdint.h>

#include <px4_platform_common/defines.h>

namespace mission_route
{

static constexpr float kLandApproachAssociationDistanceM{10.f};

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

/** @brief A global geographic coordinate used by the route planner. */
struct Position {
	double lat{static_cast<double>(NAN)};
	double lon{static_cast<double>(NAN)};
	float alt{NAN};

	bool valid() const;
};

/** @brief Index-only identity of the active synthetic DO_JUMP edge. */
struct ActiveJumpAnchor {
	int32_t start_index{-1}; /**< Position attached immediately before the DO_JUMP command. */
	int32_t target_index{-1}; /**< Resolved position at or after the command's jump target. */

	bool empty() const;
	bool valid() const;
};

/** @brief Inputs used only when planning a join back into Mission execution. */
struct MissionResumeRequest {
	Position vehicle_position{};
	int32_t mission_index{-1};
	bool current_route_direction_reversed{false}; /**< Route direction being flown before replanning. */
	ActiveJumpAnchor active_jump_anchor{};
	float home_altitude_amsl{NAN}; /**< May be NAN when every relevant mission item uses absolute altitude. */
	float projection_search_distance_m{0.f};
	float acceptance_radius_m{0.f};
	bool is_fixed_wing{false};
	bool in_transition_to_fw{false};
	float velocity_north_m_s{NAN}; /**< NAN when horizontal velocity is unavailable. */
	float velocity_east_m_s{NAN}; /**< NAN when horizontal velocity is unavailable. */
	float u_turn_penalty_m{4000.f};
};

/** @brief Inputs used only when planning a route-following return to a goal. */
struct RouteToGoalRequest {
	Position vehicle_position{};
	int32_t mission_index{-1};
	bool current_route_direction_reversed{false}; /**< Route direction being flown before replanning. */
	ActiveJumpAnchor active_jump_anchor{};
	float home_altitude_amsl{NAN}; /**< May be NAN when every relevant item uses absolute altitude. */
	float projection_search_distance_m{0.f};
	float safe_point_projection_search_distance_m{0.f};
	float acceptance_radius_m{0.f};
	float direct_goal_acceptance_radius_m{0.f};
	float altitude_acceptance_radius_m{0.f};
	bool is_fixed_wing{false};
	bool in_transition_to_fw{false};
	bool require_vtol_approach{false};
	float velocity_north_m_s{NAN}; /**< NAN when horizontal velocity is unavailable. */
	float velocity_east_m_s{NAN}; /**< NAN when horizontal velocity is unavailable. */
	float u_turn_penalty_m{4000.f};
};

/** @brief Execution-oriented Mission join produced by the planner. */
struct MissionResumePlan {
	Position join_position{};
	int32_t first_mission_item_index{-1};
	bool direction_reversed{false};
	bool use_current_altitude{false}; /**< Use live altitude when publishing the virtual join item. */
	ActiveJumpAnchor active_jump_anchor{};

	bool valid() const;
};

/** @brief Execution-oriented route-following return plan produced by the planner. */
struct RouteToGoalPlan {
	Position join_position{};
	int32_t first_mission_item_index{-1};
	bool direction_reversed{false};
	bool use_current_altitude{false}; /**< Use live altitude when publishing the virtual join item. */
	ActiveJumpAnchor active_jump_anchor{};
	GoalType goal_type{GoalType::kNone};
	Position goal_position{};
	int32_t safe_point_index{-1}; /**< Set only when goal_type is kSafePoint. */
	Position branch_off_position{}; /**< Set only when goal_type is kSafePoint. */
	/** Mission item replaced by branch_off_position: segment end nominally, segment start in reverse. */
	int32_t branch_off_mission_item_index{-1};
	bool fly_direct_to_goal{false}; /**< Bypass route following for either a safe point or endpoint goal. */

	bool valid() const;
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
