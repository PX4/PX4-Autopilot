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

#include <uORB/topics/vtol_vehicle_status.h>

namespace mission_route
{

static constexpr float kLandApproachAssociationDistanceM{10.f};

enum class GoalType : uint8_t {
	kNone = 0,
	kSafePoint,
	kMissionLand,
	kMissionTakeoff
};

/** @brief Transition needed before the vehicle joins the route. */
enum class VtolTransitionAction : uint8_t {
	kNone = 0,
	kFrontTransition,
	kBackTransition
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

/** @brief global position used by the route planner. */
struct Position {
	double lat{static_cast<double>(NAN)};
	double lon{static_cast<double>(NAN)};
	float alt{NAN};

	bool valid() const;
};

/** @brief Current/selected DO_JUMP identity, valid while the mission index space is unchanged. */
struct ActiveJumpAnchor {
	int32_t jump_item_index{-1}; /**< Index of the DO_JUMP mission item. */

	bool empty() const;
	bool valid() const;
	/** @brief Empty, or a valid index within a mission of mission_count items. */
	bool validForMission(int mission_count) const;
};

/** @brief Inputs used only when planning a join mission execution. */
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
	bool is_vtol{false};
	/** VTOL state at mission upload (MC or FW; uploading mid-transition counts as MC). UNDEFINED when unknown. */
	uint8_t vtol_state_on_mission_upload{vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED};
	float velocity_north_m_s{NAN}; /**< NAN when horizontal velocity is unavailable. */
	float velocity_east_m_s{NAN}; /**< NAN when horizontal velocity is unavailable. */
	float fw_u_turn_penalty_m{4000.f}; /**< FW-only cost of a u-turn (default estimated from turn time and route deviation)*/
};

/** @brief Inputs used only when planning the RTL route-following return. */
struct RtlRouteRequest {
	Position vehicle_position{};
	int32_t mission_index{-1};
	/** LAND/VTOL_LAND index in the same validated mission source; -1 if unavailable. */
	int32_t mission_land_index{-1};
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
	bool is_vtol{false};
	/** VTOL state at mission upload (MC or FW; uploading mid-transition counts as MC). UNDEFINED when unknown. */
	uint8_t vtol_state_on_mission_upload{vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED};
	bool require_vtol_approach{false};
	float velocity_north_m_s{NAN}; /**< NAN when horizontal velocity is unavailable. */
	float velocity_east_m_s{NAN}; /**< NAN when horizontal velocity is unavailable. */
	float fw_u_turn_penalty_m{4000.f}; /**< FW-only cost of a u-turn (default estimated from turn time and route deviation)*/
};

/** @brief Mission join plan produced by the planner. */
struct MissionResumePlan {
	Position join_position{};
	int32_t first_mission_item_index{-1};
	bool direction_reversed{false};
	bool use_current_altitude{false}; /**< Use live altitude when publishing the virtual join item. */
	ActiveJumpAnchor active_jump_anchor{};
	VtolTransitionAction vtol_transition_action{VtolTransitionAction::kNone}; /**< Transition to command when joining the route. */

	bool valid() const;
};

/** @brief RTL route-following plan produced by the planner. */
struct RtlRoutePlan {
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
	VtolTransitionAction vtol_transition_action{VtolTransitionAction::kNone}; /**< Transition to command when joining the route. */

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
