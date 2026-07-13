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
 * @file mission_route_types.cpp
 *
 * Shared mission-route planner types:
 * data-struct validity checks and the parsing/string helpers declared in
 * mission_route_types.h.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_types.h"

#include "mission_item_utils.h"

#include <px4_platform_common/log.h>

namespace mission_route
{

bool Position::valid() const
{
	return PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(alt)
	       && !((fabs(lat) < kNullIslandThresholdDeg) && (fabs(lon) < kNullIslandThresholdDeg))
	       && (fabs(lat) <= 90.0) && (fabs(lon) <= 180.0);
}

bool ProjectionDistance::valid() const
{
	return PX4_ISFINITE(xtrack) && xtrack >= 0.f
	       && PX4_ISFINITE(route_along) && route_along >= 0.f
	       && PX4_ISFINITE(segment_length) && segment_length >= 0.f
	       && PX4_ISFINITE(segment_along) && segment_along >= 0.f
	       && segment_along < (segment_length + kRoundingToleranceM);
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
	return start.valid() && end.valid() && start.idx != end.idx
	       && (is_loop || start.idx < end.idx);
}

bool Segment::validLoop() const
{
	return is_loop && valid();
}

bool SegmentDistanceAlong::valid() const
{
	return PX4_ISFINITE(start) && start > -FLT_EPSILON
	       && PX4_ISFINITE(end) && end > -FLT_EPSILON;
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

bool JoinContext::valid() const
{
	return projection.valid();
}

bool RoutePath::valid() const
{
	return first_item_index >= 0 && first_item_cmd != NAV_CMD_INVALID
	       && PX4_ISFINITE(total_cost_m) && total_cost_m >= 0.f;
}

bool GoalSelection::valid() const
{
	if (!found || !path.valid() || goal_type == GoalType::kNone || !goal_position.valid()) {
		return false;
	}

	if (!safe_point_found) {
		return goal_type == GoalType::kMissionLand || goal_type == GoalType::kMissionTakeoff;
	}

	return goal_type == GoalType::kSafePoint
	       && safe_point_index >= 0
	       && branch_off_segment.valid()
	       && branch_off_projection.valid()
	       && safe_point_position.valid();
}

int32_t GoalSelection::branchOffIndex() const
{
	if (branch_off_segment.valid()) {
		return path.direction_reversed ? branch_off_segment.start.idx : branch_off_segment.end.idx;
	}

	return path.first_item_index;
}

bool JoinPlan::valid() const
{
	return projection_context.valid() && path.valid() && join_context.valid();
}

bool RoutePlan::valid() const
{
	return projection_context.valid() && join_context.valid() && selection.valid();
}

bool PlannerParameters::validForVehicleProjection() const
{
	// home_altitude_amsl is intentionally not checked: NAN is valid for absolute-altitude missions.
	return PX4_ISFINITE(vehicle_projection_search_dist) && vehicle_projection_search_dist >= 0.f
	       && PX4_ISFINITE(acceptance_radius) && acceptance_radius >= 0.f
	       && PX4_ISFINITE(u_turn_penalty_m) && u_turn_penalty_m >= 0.f;
}

bool PlannerParameters::validForRouteToGoal() const
{
	return validForVehicleProjection()
	       && PX4_ISFINITE(safe_point_projection_search_dist) && safe_point_projection_search_dist >= 0.f
	       && PX4_ISFINITE(direct_acceptance_radius) && direct_acceptance_radius >= 0.f
	       && PX4_ISFINITE(altitude_acceptance_radius) && altitude_acceptance_radius >= 0.f;
}

bool isLandingCmd(uint16_t nav_cmd)
{
	return nav_cmd == NAV_CMD_LAND || nav_cmd == NAV_CMD_VTOL_LAND;
}

bool isTakeoffCmd(uint16_t nav_cmd)
{
	return nav_cmd == NAV_CMD_TAKEOFF || nav_cmd == NAV_CMD_VTOL_TAKEOFF;
}

float getAbsoluteAltitudeForMissionItem(const mission_item_s &mission_item, float home_altitude_amsl)
{
	if (mission_item.altitude_is_relative) {
		return PX4_ISFINITE(home_altitude_amsl) ? mission_item.altitude + home_altitude_amsl : NAN;
	}

	return mission_item.altitude;
}

bool extractMissionPosition(const mission_item_s &mission_item, float home_altitude_amsl, Position &position)
{
	if (!mission_item_contains_position(mission_item)) {
		return false;
	}

	position.lat = mission_item.lat;
	position.lon = mission_item.lon;
	position.alt = getAbsoluteAltitudeForMissionItem(mission_item, home_altitude_amsl);
	return position.valid();
}

bool extractSafePointPosition(const mission_item_s &safe_point_item, float home_altitude_amsl, Position &position)
{
	if (safe_point_item.nav_cmd != NAV_CMD_RALLY_POINT) {
		return false;
	}

	// Rally-point safe points are expected in global AMSL or global-relative-altitude frames.
	switch (safe_point_item.frame) {
	case NAV_FRAME_GLOBAL:
	case NAV_FRAME_GLOBAL_INT:
		position.alt = safe_point_item.altitude;
		break;

	case NAV_FRAME_GLOBAL_RELATIVE_ALT:
	case NAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
		if (!PX4_ISFINITE(home_altitude_amsl)) {
			return false;
		}

		position.alt = safe_point_item.altitude + home_altitude_amsl; // alt of safe point is rel to home
		break;

	default:
		PX4_WARN("RTL safe point frame %u unsupported", static_cast<unsigned>(safe_point_item.frame));
		return false;
	}

	position.lat = safe_point_item.lat;
	position.lon = safe_point_item.lon;
	return position.valid();
}

bool copyPositionToYawSetpoint(const Position &position, PositionYawSetpoint &setpoint)
{
	if (!position.valid()) {
		return false;
	}

	setpoint.lat = position.lat;
	setpoint.lon = position.lon;
	setpoint.alt = position.alt;
	setpoint.yaw = NAN;
	return true;
}

loiter_point_s makeVtolLandApproachPoint(const mission_item_s &mission_item, float home_altitude_amsl)
{
	loiter_point_s approach{};
	approach.lat = mission_item.lat;
	approach.lon = mission_item.lon;
	approach.height_m = getAbsoluteAltitudeForMissionItem(mission_item, home_altitude_amsl);
	approach.loiter_radius_m = mission_item.loiter_radius;
	return approach;
}

const char *failureReasonString(FailureReason failure_reason)
{
	switch (failure_reason) {
	case FailureReason::kNone:
		return "None";

	case FailureReason::kNoValidGlobalPos:
		return "NoValidGlobalPos";

	case FailureReason::kInvalidRequest:
		return "InvalidRequest";

	case FailureReason::kNoValidWaypoints:
		return "NoValidWaypoints";

	case FailureReason::kNoValidSafePoints:
		return "NoValidSafePoints";

	case FailureReason::kNoValidPath:
		return "NoValidPath";

	case FailureReason::kNoSegmentsFound:
		return "NoSegmentsFound";

	case FailureReason::kInternalError:
		return "InternalError";

	case FailureReason::kLoadFailed:
		return "LoadFailed";

	case FailureReason::kInvalidProjectionContext:
		return "InvalidProjectionContext";

	case FailureReason::kNoLocalMinFound:
		return "NoLocalMinFound";

	case FailureReason::kPositionItemInvalid:
		return "PositionItemInvalid";

	case FailureReason::kNoValidCandidateFound:
		return "NoValidCandidateFound";

	case FailureReason::kUnknown:
	default:
		return "Unknown";
	}
}

const char *goalTypeString(GoalType goal_type)
{
	switch (goal_type) {
	case GoalType::kSafePoint:
		return "safe_point";

	case GoalType::kMissionLand:
		return "mission_land";

	case GoalType::kMissionTakeoff:
		return "mission_takeoff";

	case GoalType::kNone:
	default:
		return "none";
	}
}

} // namespace mission_route
