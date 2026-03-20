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
 * @file rtl_route_planner.cpp
 *
 * Route planner for safe-point RTL (RTL_TYPE = 6).  Projects the vehicle
 * and every safe point onto the uploaded mission geometry, then selects
 * the goal with the shortest along-route cost.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "rtl_route_planner.h"

#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

#include <px4_platform_common/log.h>
#include <uORB/topics/vtol_vehicle_status.h>

using namespace math;

/**
 * Shared static SafePointBatch to keep the ~25 KB struct off the stack on
 * constrained targets.  All three call-sites (findProjectionCandidates,
 * collectVehicleProjection, selectSafePoint) reinitialize the batch before
 * use and are never called concurrently
 */
static RtlRoutePlanner::SafePointBatch s_safe_point_batch{};

const char *RtlRoutePlanner::failureReasonString(FailureReason failure_reason)
{
	switch (failure_reason) {
	case FailureReason::None:
		return "None";

	case FailureReason::NoValidGlobalPos:
		return "NoValidGlobalPos";

	case FailureReason::NoValidWaypoints:
		return "NoValidWaypoints";

	case FailureReason::NoSegmentsFound:
		return "NoSegmentsFound";

	case FailureReason::InternalError:
		return "InternalError";

	case FailureReason::NoLocalMinFound:
		return "NoLocalMinFound";

	case FailureReason::PositionItemInvalid:
		return "PositionItemInvalid";

	case FailureReason::NoValidCandidateFound:
		return "NoValidCandidateFound";

	case FailureReason::Unknown:
	default:
		return "Unknown";
	}
}

const char *RtlRoutePlanner::goalTypeString(GoalType goal_type)
{
	switch (goal_type) {
	case GoalType::SafePoint:
		return "safe_point";

	case GoalType::MissionLand:
		return "mission_land";

	case GoalType::MissionTakeoff:
		return "mission_takeoff";

	case GoalType::None:
	default:
		return "none";
	}
}

/** @brief Validate a global route-planning position. */
bool RtlRoutePlanner::Position::valid() const
{
	return PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(alt)
	       && !((fabs(lat) < kNullIslandThresholdDeg) && (fabs(lon) < kNullIslandThresholdDeg))
	       && (fabs(lat) < 90.0) && (fabs(lon) < 180.0);
}

/** @brief Validate the along-track and cross-track distances of a projection. */
bool RtlRoutePlanner::PointDistance::valid() const
{
	return PX4_ISFINITE(xtrack) && xtrack >= 0.f
	       && PX4_ISFINITE(along) && along >= 0.f
	       && PX4_ISFINITE(segment_length) && segment_length >= 0.f
	       && PX4_ISFINITE(on_segment) && on_segment >= 0.f
	       && on_segment < (segment_length + kRoundingToleranceM);
}

/** @brief Validate a mission segment endpoint descriptor. */
bool RtlRoutePlanner::SegmentEndpoint::valid() const
{
	return idx >= 0 && nav_cmd != NAV_CMD_INVALID;
}

/** @brief Validate a mission segment descriptor. */
bool RtlRoutePlanner::Segment::valid() const
{
	return start.valid() && end.valid() && start.idx != end.idx
	       && (is_loop || start.idx < end.idx);
}

/** @brief Validate the along-track interval of a segment. */
bool RtlRoutePlanner::SegmentDistanceAlong::valid() const
{
	return PX4_ISFINITE(start) && start > -FLT_EPSILON
	       && PX4_ISFINITE(end) && end > -FLT_EPSILON;
}

/** @brief Validate the start and end positions of a segment. */
bool RtlRoutePlanner::SegmentPositions::valid() const
{
	return start.valid() && end.valid();
}

/** @brief Validate a concrete segment projection candidate. */
bool RtlRoutePlanner::SegmentCandidate::valid() const
{
	return segment.valid() && segment_positions.valid() && projection.valid() && dist.valid();
}

/** @brief Validate loop-specific context captured from a jump segment projection. */
bool RtlRoutePlanner::LoopContext::valid() const
{
	return segment.is_loop && along.valid() && segment_positions.valid();
}

/** @brief Validate the vehicle projection context used for route planning. */
bool RtlRoutePlanner::ProjectionContext::valid() const
{
	return vehicle_pos.valid() && projection.valid();
}

/** @brief Validate a mission path decision. */
bool RtlRoutePlanner::Path::valid() const
{
	return first_item_index >= 0 && first_item_cmd != NAV_CMD_INVALID && PX4_ISFINITE(dist);
}

/** @brief Validate the selected SRP goal and its cached branch-off geometry. */
bool RtlRoutePlanner::Selection::valid() const
{
	if (!found || !path.valid() || goal_type == GoalType::None || !goal_position.valid()) {
		return false;
	}

	if (!safe_point_found) {
		return goal_type == GoalType::MissionLand || goal_type == GoalType::MissionTakeoff;
	}

	return goal_type == GoalType::SafePoint
	       && safe_point_index >= 0
	       && branch_off_segment.valid()
	       && branch_off_projection.valid()
	       && safe_point_position.valid();
}

/** @brief Return the mission index where SRP branches away from the mission path. */
int32_t RtlRoutePlanner::Selection::branchOffIndex() const
{
	if (branch_off_segment.valid()) {
		return path.direction_reversed ? branch_off_segment.start.idx : branch_off_segment.end.idx;
	}

	return path.first_item_index;
}

/** @brief Return whether a mission item contributes a position to path planning. */
bool RtlRoutePlanner::itemContainsPosition(const mission_item_s &mission_item)
{
	return mission_item.nav_cmd == NAV_CMD_WAYPOINT
	       || mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED
	       || mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT
	       || mission_item.nav_cmd == NAV_CMD_LAND
	       || mission_item.nav_cmd == NAV_CMD_TAKEOFF
	       || mission_item.nav_cmd == NAV_CMD_LOITER_TO_ALT
	       || mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
	       || mission_item.nav_cmd == NAV_CMD_VTOL_LAND;
}

/** @brief Convert a mission item's altitude to AMSL when required. */
float RtlRoutePlanner::getAbsoluteAltitudeForMissionItem(const mission_item_s &mission_item, float home_altitude_amsl)
{
	if (mission_item.altitude_is_relative && PX4_ISFINITE(home_altitude_amsl)) {
		return mission_item.altitude + home_altitude_amsl;
	}

	return mission_item.altitude;
}

/** @brief Extract a valid global position from a mission item. */
bool RtlRoutePlanner::extractMissionPosition(const mission_item_s &mission_item, float home_altitude_amsl,
		Position &position)
{
	if (!itemContainsPosition(mission_item)) {
		return false;
	}

	position.lat = mission_item.lat;
	position.lon = mission_item.lon;
	position.alt = getAbsoluteAltitudeForMissionItem(mission_item, home_altitude_amsl);
	return position.valid();
}

/** @brief Extract a valid global position from a safe-point item. */
bool RtlRoutePlanner::extractSafePointPosition(const mission_item_s &safe_point_item, float home_altitude_amsl,
		Position &position)
{
	if (safe_point_item.nav_cmd != NAV_CMD_RALLY_POINT) {
		return false;
	}

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

		position.alt = safe_point_item.altitude + home_altitude_amsl;
		break;

	default:
		return false;
	}

	position.lat = safe_point_item.lat;
	position.lon = safe_point_item.lon;
	return position.valid();
}

/** @brief Read a mission item and return its attached valid position if it has one. */
bool RtlRoutePlanner::readValidMissionPosition(int index, float home_altitude_amsl, Position &position,
		uint16_t *nav_cmd) const
{
	mission_item_s mission_item{};

	if (!_provider.loadMissionItem(index, mission_item)) {
		return false;
	}

	if (!extractMissionPosition(mission_item, home_altitude_amsl, position)) {
		return false;
	}

	if (nav_cmd != nullptr) {
		*nav_cmd = mission_item.nav_cmd;
	}

	return true;
}

/** @brief Find the next mission index carrying a valid position item. */
bool RtlRoutePlanner::findNextValidPositionIndex(uint16_t start_index, float home_altitude_amsl,
		uint16_t &next_position_index) const
{
	for (uint16_t index = start_index; index < _provider.missionCount(); ++index) {
		Position position{};

		if (readValidMissionPosition(index, home_altitude_amsl, position)) {
			next_position_index = index;
			return true;
		}
	}

	return false;
}

/** @brief Find the position item attached to or preceding the given mission index. */
bool RtlRoutePlanner::findAttachedValidPositionIndex(uint16_t start_index, float home_altitude_amsl,
		uint16_t &attached_position_index) const
{
	for (int32_t index = start_index; index >= 0; --index) {
		mission_item_s mission_item{};

		if (!_provider.loadMissionItem(index, mission_item)) {
			return false;
		}

		if (itemContainsPosition(mission_item)) {
			Position position{};

			if (extractMissionPosition(mission_item, home_altitude_amsl, position)) {
				attached_position_index = index;
				return true;
			}

			return false;
		}
	}

	return false;
}

/** @brief Load all valid safe points once so the route can be scanned in a single batch. */
bool RtlRoutePlanner::loadSafePointBatch(float home_altitude_amsl, SafePointBatch &batch) const
{
	batch = {};

	const int safe_point_count = _provider.safePointCount();

	if (safe_point_count <= 0) {
		return false;
	}

	const int safe_point_limit = min(safe_point_count, static_cast<int>(MAX_SAFE_POINT_BATCH));

	if (safe_point_count > safe_point_limit) {
		PX4_WARN("RTL SRP safe point count %d exceeds batch limit %u, clamping",
			 safe_point_count, static_cast<unsigned>(MAX_SAFE_POINT_BATCH));
	}

	for (int safe_point_index = 0; safe_point_index < safe_point_limit; ++safe_point_index) {
		mission_item_s safe_point_item{};

		if (!_provider.loadSafePointItem(safe_point_index, safe_point_item)) {
			PX4_WARN("RTL SRP safe point %d read failed", safe_point_index);
			continue;
		}

		Position safe_point_position{};

		if (!extractSafePointPosition(safe_point_item, home_altitude_amsl, safe_point_position)) {
			PX4_DEBUG("RTL SRP safe point %d skipped, invalid position or frame", safe_point_index);
			continue;
		}

		batch.items[batch.count].position = safe_point_position;
		batch.items[batch.count].source_index = safe_point_index;
		batch.count++;
	}

	return batch.count > 0;
}

/** @brief Clear all per-safe-point candidate buffers before running a new batch scan. */
void RtlRoutePlanner::resetSafePointBatchResults(SafePointBatch &batch) const
{
	for (uint8_t i = 0; i < batch.count; ++i) {
		batch.items[i].candidate_buffer.count = 0;
	}
}

/** @brief Advance mission scanning state to the next position-bearing segment end. */
bool RtlRoutePlanner::prepareNextSegment(uint16_t index, Segment &segment, SegmentPositions &segment_positions,
		float home_altitude_amsl, FailureReason &failure_reason) const
{
	mission_item_s mission_item{};

	if (!_provider.loadMissionItem(index, mission_item)) {
		failure_reason = FailureReason::InternalError;
		return false;
	}

	segment.end.idx = index;
	segment.end.nav_cmd = mission_item.nav_cmd;
	segment.loops_remaining = 0;
	segment.is_loop = (mission_item.nav_cmd == NAV_CMD_DO_JUMP);

	if (segment.is_loop) {
		const int32_t remaining_loops = static_cast<int32_t>(mission_item.do_jump_repeat_count)
						- static_cast<int32_t>(mission_item.do_jump_current_count);
		segment.loops_remaining = static_cast<uint8_t>(constrain(remaining_loops, 0, 255));

		uint16_t jump_to_index{0};

		if (!findNextValidPositionIndex(static_cast<uint16_t>(mission_item.do_jump_mission_index),
						home_altitude_amsl, jump_to_index)) {
			failure_reason = FailureReason::InternalError;
			return false;
		}

		if (!_provider.loadMissionItem(jump_to_index, mission_item)) {
			failure_reason = FailureReason::InternalError;
			return false;
		}

		segment.end.idx = jump_to_index;
		segment.end.nav_cmd = mission_item.nav_cmd;
	}

	if (!itemContainsPosition(mission_item)) {
		return false;
	}

	if (!extractMissionPosition(mission_item, home_altitude_amsl, segment_positions.end)) {
		failure_reason = FailureReason::PositionItemInvalid;
		return false;
	}

	return true;
}

/** @brief Apply the same local-minimum pruning rule used by legacy SRP projections. */
bool RtlRoutePlanner::localMinimumOnSegment(bool proj_on_start, bool proj_on_end, bool prev_proj_on_end,
		bool jumping, bool last_segment) const
{
	const bool proj_on_corner = proj_on_start || proj_on_end;

	if (jumping) {
		return !proj_on_corner;
	}

	if (last_segment && proj_on_end) {
		return true;
	}

	return !proj_on_corner || (proj_on_start && prev_proj_on_end);
}

/** @brief Sanity-check a projection candidate before inserting it into the buffer. */
bool RtlRoutePlanner::validateCandidate(const SegmentCandidate &candidate) const
{
	if (!candidate.valid()) {
		return false;
	}

	if (candidate.segment.start.idx >= _provider.missionCount()
	    || candidate.segment.end.idx >= _provider.missionCount()) {
		return false;
	}

	if (candidate.segment.start.nav_cmd == NAV_CMD_INVALID || candidate.segment.end.nav_cmd == NAV_CMD_INVALID) {
		return false;
	}

	return true;
}

/** @brief Insert a segment candidate into the xtrack-sorted candidate buffer. */
void RtlRoutePlanner::insertCandidateSorted(CandidateBuffer &candidate_buffer, const SegmentCandidate &candidate) const
{
	uint8_t insert_index = 0;
	const uint8_t buffer_size = min(candidate_buffer.count, MAX_SEGMENT_CANDIDATES);

	while (insert_index < buffer_size
	       && candidate_buffer.candidates[insert_index].dist.xtrack <= candidate.dist.xtrack) {
		++insert_index;
	}

	if (insert_index >= MAX_SEGMENT_CANDIDATES) {
		return;
	}

	int16_t shift_dest = (buffer_size == MAX_SEGMENT_CANDIDATES) ? buffer_size - 1 : buffer_size;

	for (int16_t j = shift_dest; j > insert_index; --j) {
		candidate_buffer.candidates[j] = candidate_buffer.candidates[j - 1];
	}

	candidate_buffer.candidates[insert_index] = candidate;
	candidate_buffer.count = min(static_cast<uint8_t>(buffer_size + 1), MAX_SEGMENT_CANDIDATES);
}

/** @brief Trim the projection candidate buffer once a tighter xtrack window is known. */
void RtlRoutePlanner::pruneProjectionCandidates(CandidateBuffer &candidate_buffer, float xtrack_limit) const
{
	if (candidate_buffer.count < 2) {
		return;
	}

	const uint8_t buffer_size = min(candidate_buffer.count, MAX_SEGMENT_CANDIDATES);
	uint8_t index = buffer_size;

	do {
		--index;

		if (candidate_buffer.candidates[index].dist.xtrack <= xtrack_limit) {
			candidate_buffer.count = index + 1U;
			return;
		}
	} while (index > 0);

	candidate_buffer.count = 0;
}

/** @brief Fill the along-track bounds of the last flown segment once they are reached during scanning. */
bool RtlRoutePlanner::fillDistAlongToLastFlownIfNecessary(int32_t mission_index, const Segment &segment_to_consider,
		bool is_flying_reverse, float total_dist, float segment_length,
		const Segment &last_flown_loop_segment,
		SegmentDistanceAlong &dist_along_to_last_flown_segment) const
{
	if (last_flown_loop_segment.is_loop) {
		if (segment_to_consider.start.idx == last_flown_loop_segment.start.idx) {
			dist_along_to_last_flown_segment.start = total_dist;
			PX4_DEBUG("RTL SRP fill dist_along_to_last_flown.start on loop: %.1f",
				  static_cast<double>(dist_along_to_last_flown_segment.start));

		} else if (segment_to_consider.start.idx == last_flown_loop_segment.end.idx) {
			dist_along_to_last_flown_segment.end = total_dist;
			PX4_DEBUG("RTL SRP fill dist_along_to_last_flown.end on loop: %.1f",
				  static_cast<double>(dist_along_to_last_flown_segment.end));
		}

		return dist_along_to_last_flown_segment.valid();
	}

	if (mission_index == 0 && !is_flying_reverse) {
		dist_along_to_last_flown_segment.start = 0.f;
		dist_along_to_last_flown_segment.end = 0.f;
		PX4_DEBUG("RTL SRP fill dist_along_to_last_flown with zeros for first mission item");
		return true;
	}

	if (isIndexInProjectionSegment(segment_to_consider, mission_index, is_flying_reverse)) {
		dist_along_to_last_flown_segment.start = total_dist;
		dist_along_to_last_flown_segment.end = total_dist + segment_length;
		PX4_DEBUG("RTL SRP fill dist_along_to_last_flown nominal case: [%.3f, %.3f]",
			  static_cast<double>(dist_along_to_last_flown_segment.start),
			  static_cast<double>(dist_along_to_last_flown_segment.end));
		return true;
	}

	return false;
}

/** @brief Check whether a mission index lies inside a projected segment for the current travel direction. */
bool RtlRoutePlanner::isIndexInProjectionSegment(const Segment &projection_segment, int32_t mission_index,
		bool is_flying_reverse) const
{
	if (!projection_segment.valid()) {
		return false;
	}

	if (is_flying_reverse) {
		return mission_index >= projection_segment.start.idx && mission_index < projection_segment.end.idx;
	}

	return mission_index > projection_segment.start.idx && mission_index <= projection_segment.end.idx;
}

/**
 * @brief Evaluate one mission segment for one reference point and keep only locally minimal projections.
 *
 * Algorithm outline:
 *   1. If the segment is zero-length, the projection is the endpoint itself (xtrack = direct distance).
 *   2. Otherwise, compute the orthogonal projection of the reference point onto the segment:
 *      - Vector A→B: segment start to segment end (local NED frame).
 *      - Vector A→P: segment start to reference point.
 *      - Projection parameter: t = (A→P · A→B) / |A→B|².
 *      - Clamp t to [0, 1] to stay on the segment.
 *   3. Corner detection (5 cm tolerance):
 *      - proj_on_start: t * segment_length < 5 cm.
 *      - proj_on_end: (1 − t) * segment_length < 5 cm.
 *   4. Local minimum filter (localMinimumOnSegment):
 *      - Jump/loop segments: only non-corner projections are local minima.
 *      - Last segment: proj_on_end is always accepted.
 *      - Normal segments: accept non-corner projections, or accept a corner when consecutive
 *        segments share the same vertex (proj_on_start AND previous segment's proj_on_end).
 *   5. Reject if xtrack is non-finite or exceeds the current shrinking search window.
 *   6. Reconstruct the projection in global coordinates (lat/lon via local NED vector addition,
 *      altitude via linear interpolation).
 *   7. Validate the candidate (finite distances, valid indices).
 *   8. If xtrack < current min_xtrack, tighten the search window and prune stale candidates.
 *   9. Insert the candidate into the buffer sorted by xtrack (up to MAX_SEGMENT_CANDIDATES = 3).
 */
void RtlRoutePlanner::processCandidateForSegment(const Position &reference_position, const Segment &segment,
		const SegmentPositions &segment_positions, float segment_length,
		const matrix::Vector2f &segment_vector, float total_dist,
		float extra_xtrack_dist, bool last_segment, bool segment_has_no_length,
		CandidateSearchState &state, uint32_t &local_min_found, uint32_t &valid_candidate_found,
		CandidateBuffer &candidate_buffer) const
{
	float xtrack = 0.f;
	float on_segment = 0.f;
	float t_clamped = 0.f;

	matrix::Vector2f projection_vector(0.f, 0.f);
	bool proj_on_start = false;
	bool proj_on_end = false;

	// --- Step 1: Degenerate (zero-length) segment handling ---
	if (segment_has_no_length) {
		// If the segment is a point, the projection is the point itself.
		xtrack = get_distance_to_next_waypoint(reference_position.lat, reference_position.lon,
						       segment_positions.end.lat, segment_positions.end.lon);
		proj_on_start = true;
		proj_on_end = true;

	} else {
		// --- Step 2: Orthogonal projection onto the segment ---
		// segment_vector (A→B) is pre-computed by the caller once per segment.
		matrix::Vector2f reference_vector; // Vector A→P (segment start to reference point)

		get_vector_to_next_waypoint(segment_positions.start.lat, segment_positions.start.lon,
					    reference_position.lat, reference_position.lon,
					    &reference_vector(0), &reference_vector(1));

		// t = (A→P · A→B) / |A→B|²  — unclamped scalar projection parameter.
		const float path_len_sq = segment_vector.norm_squared();
		const float t = (path_len_sq > FLT_EPSILON) ? (reference_vector.dot(segment_vector) / path_len_sq) : 0.f;

		// --- Step 3: Corner detection ---
		// If t is negative, (t * len) is negative which is < tol, so this check handles both cases.
		static constexpr float kCornerToleranceM = 0.05f;
		proj_on_start = (t * segment_length) < kCornerToleranceM;
		proj_on_end = ((1.f - t) * segment_length) < kCornerToleranceM;

		// Final clamped projection.
		t_clamped = constrain(t, 0.f, 1.f);
		projection_vector = segment_vector * t_clamped;
		xtrack = static_cast<matrix::Vector2f>(reference_vector - projection_vector).norm();
		on_segment = t_clamped * segment_length;
	}

	state.proj_on_end_for_segment = proj_on_end;

	// --- Step 4: Local minimum filter ---
	// Only keep locally minimal projections.  This avoids emitting both sides of the same shared
	// corner unless the legacy corner rule explicitly allows it, which keeps the candidate set
	// stable around turns and loops.
	if (!localMinimumOnSegment(proj_on_start, proj_on_end, state.prev_proj_on_end, segment.is_loop, last_segment)) {
		return;
	}

	local_min_found++;

	// --- Step 5: Reject non-finite or out-of-window projections ---
	// We abort here before doing LatLon reconstruction (which is more expensive).
	if (!PX4_ISFINITE(xtrack) || xtrack < 0.f || xtrack >= state.xtrack_limit) {
		return;
	}

	// --- Step 6: Construct the candidate with global coordinates ---
	SegmentCandidate candidate{};
	candidate.segment = segment;
	candidate.segment_positions = segment_positions;
	candidate.dist.xtrack = xtrack;
	candidate.dist.along = total_dist + on_segment;
	candidate.dist.segment_length = segment_length;
	candidate.dist.on_segment = min(on_segment, segment_length);

	if (segment_has_no_length) {
		candidate.projection = segment_positions.end;

	} else {
		// Reconstruct the projected lat/lon from the local NED offset vector.
		double lat_res;
		double lon_res;
		add_vector_to_global_position(segment_positions.start.lat, segment_positions.start.lon,
					      projection_vector(0), projection_vector(1),
					      &lat_res, &lon_res);
		candidate.projection.lat = lat_res;
		candidate.projection.lon = lon_res;
		// Interpolate altitude linearly along the segment.
		candidate.projection.alt = segment_positions.start.alt
					   + t_clamped * (segment_positions.end.alt - segment_positions.start.alt);
	}

	// --- Step 7: Validate the candidate ---
	if (!validateCandidate(candidate)) {
		return;
	}

	valid_candidate_found++;

	// --- Steps 8–9: Tighten search window and insert sorted ---
	if (xtrack < state.min_xtrack) {
		// A new closest projection tightens the search window, so prune stale candidates first.
		state.min_xtrack = xtrack;
		state.xtrack_limit = state.min_xtrack + extra_xtrack_dist;
		pruneProjectionCandidates(candidate_buffer, state.xtrack_limit);
	}

	insertCandidateSorted(candidate_buffer, candidate);
}

/** @brief Find locally optimal mission projections for a batch of safe points in one route scan. */
bool RtlRoutePlanner::findProjectionCandidatesBatch(int32_t mission_index, float home_altitude_amsl,
		bool is_flying_reverse, float extra_xtrack_dist,
		const Segment &last_flown_loop_segment,
		SafePointBatch &batch,
		SegmentDistanceAlong *dist_along_to_last_flown_segment,
		float *dist_along_to_route_end,
		uint8_t *loops_remaining,
		FailureReason *failure_reason) const
{
	if (failure_reason != nullptr) {
		*failure_reason = FailureReason::Unknown;
	}

	if (_provider.missionCount() < 2) {
		if (failure_reason != nullptr) {
			*failure_reason = FailureReason::NoValidWaypoints;
		}

		return false;
	}

	if (batch.count == 0 || batch.count > MAX_SAFE_POINT_BATCH) {
		return false;
	}

	resetSafePointBatchResults(batch);

	if (loops_remaining != nullptr) {
		*loops_remaining = 0;
	}

	SegmentDistanceAlong last_flown_segment{};
	bool search_last_flown_segment = (mission_index != INT32_MAX);

	if (search_last_flown_segment) {
		search_last_flown_segment = !fillDistAlongToLastFlownIfNecessary(mission_index, Segment{}, is_flying_reverse,
					    0.f, 0.f, last_flown_loop_segment, last_flown_segment);
	}

	uint16_t first_position_index{0};
	uint16_t last_position_index{0};

	if (!findNextValidPositionIndex(0, home_altitude_amsl, first_position_index)
	    || !findAttachedValidPositionIndex(static_cast<uint16_t>(_provider.missionCount() - 1),
					       home_altitude_amsl, last_position_index)) {
		if (failure_reason != nullptr) {
			*failure_reason = FailureReason::NoValidWaypoints;
		}

		return false;
	}

	Segment segment{};
	SegmentPositions segment_positions{};
	bool have_previous = false;
	float total_dist = 0.f;
	BatchSearchState batch_state{};

	for (uint16_t index = first_position_index; index < _provider.missionCount(); ++index) {
		FailureReason loop_failure_reason = FailureReason::Unknown;

		if (!prepareNextSegment(index, segment, segment_positions, home_altitude_amsl, loop_failure_reason)) {
			if (loop_failure_reason != FailureReason::Unknown) {
				if (failure_reason != nullptr) {
					*failure_reason = loop_failure_reason;
				}

				return false;
			}

			continue;
		}

		if (segment.is_loop && loops_remaining != nullptr) {
			*loops_remaining = segment.loops_remaining;
		}

		if (!have_previous) {
			// The first valid position-bearing item seeds the scan; a segment only exists once a second
			// endpoint (or loop edge) has been discovered.
			have_previous = true;
			segment.start = segment.end;
			segment_positions.start = segment_positions.end;
			continue;
		}

		batch_state.segments_processed++;

		if (segment.end.nav_cmd == NAV_CMD_LAND) {
			segment_positions.end.alt = segment_positions.start.alt;
		}

		const bool last_segment = (segment.end.idx == last_position_index);
		const bool first_segment = (segment.start.idx == first_position_index);
		const bool segment_has_no_length = fabs(segment_positions.start.lat - segment_positions.end.lat) <= kCornerLatLonTolDeg
						   && fabs(segment_positions.start.lon - segment_positions.end.lon) <= kCornerLatLonTolDeg;

		if (segment_has_no_length && !last_segment && !first_segment) {
			const float segment_length = 0.f;

			if (search_last_flown_segment) {
				search_last_flown_segment = !fillDistAlongToLastFlownIfNecessary(mission_index, segment,
							    is_flying_reverse, total_dist, segment_length,
							    last_flown_loop_segment, last_flown_segment);
			}

			if (!segment.is_loop) {
				segment.start = segment.end;
				segment_positions.start = segment_positions.end;
			}

			continue;
		}

		const float segment_length = segment_has_no_length ? 0.f : get_distance_to_next_waypoint(segment_positions.start.lat,
					     segment_positions.start.lon, segment_positions.end.lat, segment_positions.end.lon);

		// Compute the segment direction vector once per segment (expensive trig)
		// rather than redundantly inside processCandidateForSegment for each safe point.
		matrix::Vector2f segment_vector(0.f, 0.f);

		if (!segment_has_no_length) {
			get_vector_to_next_waypoint(segment_positions.start.lat, segment_positions.start.lon,
						    segment_positions.end.lat, segment_positions.end.lon,
						    &segment_vector(0), &segment_vector(1));
		}

		for (uint8_t i = 0; i < batch.count; ++i) {
			processCandidateForSegment(batch.items[i].position, segment, segment_positions, segment_length,
						   segment_vector, total_dist,
						   extra_xtrack_dist, last_segment, segment_has_no_length,
						   batch_state.candidate_states[i], batch_state.local_min_found,
						   batch_state.valid_candidate_found, batch.items[i].candidate_buffer);
		}

		if (segment.is_loop) {
			// Loop edges contribute geometry for projection and path solving, but they do not advance the
			// nominal along-route accumulator because mission execution does not "walk through" them here.
			continue;
		}

		if (search_last_flown_segment) {
			search_last_flown_segment = !fillDistAlongToLastFlownIfNecessary(mission_index, segment, is_flying_reverse,
						    total_dist, segment_length, last_flown_loop_segment, last_flown_segment);
		}

		total_dist += segment_length;
		segment.start = segment.end;
		segment_positions.start = segment_positions.end;

		for (uint8_t i = 0; i < batch.count; ++i) {
			batch_state.candidate_states[i].prev_proj_on_end =
				segment_has_no_length ? false : batch_state.candidate_states[i].proj_on_end_for_segment;
		}

		if (last_segment) {
			break;
		}
	}

	if (dist_along_to_last_flown_segment != nullptr) {
		*dist_along_to_last_flown_segment = last_flown_segment;
	}

	if (dist_along_to_route_end != nullptr) {
		*dist_along_to_route_end = total_dist;
	}

	bool any_candidate_found = false;

	for (uint8_t i = 0; i < batch.count; ++i) {
		if (batch.items[i].candidate_buffer.count > 0) {
			any_candidate_found = true;
			break;
		}
	}

	PX4_DEBUG("RTL SRP batch items: %u, segs: %u, mins: %u, valid: %u",
		  static_cast<unsigned>(batch.count),
		  static_cast<unsigned>(batch_state.segments_processed),
		  static_cast<unsigned>(batch_state.local_min_found),
		  static_cast<unsigned>(batch_state.valid_candidate_found));

	if (any_candidate_found) {
		if (failure_reason != nullptr) {
			*failure_reason = FailureReason::None;
		}

		return true;
	}

	if (failure_reason != nullptr) {
		if (batch_state.segments_processed == 0) {
			*failure_reason = FailureReason::NoSegmentsFound;

		} else if (batch_state.local_min_found == 0) {
			*failure_reason = FailureReason::NoLocalMinFound;

		} else if (batch_state.valid_candidate_found == 0) {
			*failure_reason = FailureReason::NoValidCandidateFound;

		} else {
			*failure_reason = FailureReason::Unknown;
		}
	}

	return false;
}

/** @brief Find all locally optimal mission projections for a single reference position. */
bool RtlRoutePlanner::findProjectionCandidates(const Position &reference_position, int32_t mission_index,
		float home_altitude_amsl,
		bool is_flying_reverse, float extra_xtrack_dist, CandidateBuffer &candidate_buffer,
		SegmentDistanceAlong *dist_along_to_last_flown_segment, float *dist_along_to_route_end,
		uint8_t *loops_remaining, FailureReason *failure_reason) const
{
	if (failure_reason != nullptr) {
		*failure_reason = FailureReason::Unknown;
	}

	if (!reference_position.valid()) {
		if (failure_reason != nullptr) {
			*failure_reason = FailureReason::NoValidGlobalPos;
		}

		return false;
	}

	s_safe_point_batch = {};
	s_safe_point_batch.count = 1;
	s_safe_point_batch.items[0].position = reference_position;

	const bool any_candidate_found = findProjectionCandidatesBatch(mission_index, home_altitude_amsl, is_flying_reverse,
					 extra_xtrack_dist, Segment{}, s_safe_point_batch,
					 dist_along_to_last_flown_segment, dist_along_to_route_end,
					 loops_remaining, failure_reason);

	candidate_buffer = s_safe_point_batch.items[0].candidate_buffer;
	return any_candidate_found && candidate_buffer.count > 0;
}

/** @brief Clamp a mission index into the valid mission range before projection. */
bool RtlRoutePlanner::clampMissionIndex(int32_t &mission_index) const
{
	if (_provider.missionCount() <= 0) {
		return false;
	}

	mission_index = constrain(mission_index, 0, _provider.missionCount() - 1);
	return true;
}

/** @brief Project the vehicle onto the mission while preferring the currently active segment. */
bool RtlRoutePlanner::collectVehicleProjection(const Position &vehicle_position, int32_t mission_index,
		const Config &config, ProjectionContext &projection_context, FailureReason *failure_reason) const
{
	const int32_t requested_mission_index = mission_index;

	if (!clampMissionIndex(mission_index)) {
		if (failure_reason != nullptr) {
			*failure_reason = FailureReason::NoValidWaypoints;
		}

		return false;
	}

	if (mission_index != requested_mission_index) {
		PX4_ERR("RTL SRP invalid mission index: %d setting to: %d",
			static_cast<int>(requested_mission_index), static_cast<int>(mission_index));
	}

	s_safe_point_batch = {};
	s_safe_point_batch.count = 1;
	s_safe_point_batch.items[0].position = vehicle_position;
	SegmentDistanceAlong dist_along_to_last_flown_segment{};
	float dist_along_to_route_end = 0.f;
	uint8_t loops_remaining = 0;

	if (!findProjectionCandidatesBatch(mission_index, config.home_altitude_amsl,
					   config.is_flying_reverse, config.vehicle_projection_search_dist,
					   config.last_flown_loop_segment, s_safe_point_batch,
					   &dist_along_to_last_flown_segment, &dist_along_to_route_end,
					   &loops_remaining, failure_reason)) {
		return false;
	}

	const CandidateBuffer &candidate_buffer = s_safe_point_batch.items[0].candidate_buffer;

	PX4_DEBUG("RTL SRP select UAV proj: cand cnt: %u dist_proj_to_last_flown[%.1f; %.1f] mis_idx=%d",
		  candidate_buffer.count,
		  static_cast<double>(dist_along_to_last_flown_segment.start),
		  static_cast<double>(dist_along_to_last_flown_segment.end),
		  static_cast<int>(mission_index));

	if (!dist_along_to_last_flown_segment.valid()) {
		PX4_ERR("RTL SRP select UAV proj: invalid last flown segment (mission_idx=%d), setting to zero",
			static_cast<int>(mission_index));
		dist_along_to_last_flown_segment.start = 0.f;
		dist_along_to_last_flown_segment.end = 0.f;
	}

	int best_candidate_index = -1;
	float min_path_distance = FLT_MAX;

	for (uint8_t i = 0; i < candidate_buffer.count; ++i) {
		const SegmentCandidate &candidate = candidate_buffer.candidates[i];
		const float dist_to_start = fabsf(candidate.dist.along - dist_along_to_last_flown_segment.start);
		const float dist_to_end = fabsf(candidate.dist.along - dist_along_to_last_flown_segment.end);
		const float projection_to_segment_dist = fminf(dist_to_start, dist_to_end);
		// Legacy SRP keeps the current leg when possible. Otherwise it chooses the candidate that is
		// cheapest to reach from the last flown segment anchor, not simply the raw smallest xtrack.
		const float candidate_path_distance = candidate.dist.xtrack + projection_to_segment_dist;

		PX4_DEBUG("RTL SRP UAV proj cand %u on seg [%u->%u], path_dist=%.1f, along=%.1f xtrack=%.1f on_seg=%.1f",
			  static_cast<unsigned>(i),
			  static_cast<unsigned>(candidate.segment.start.idx),
			  static_cast<unsigned>(candidate.segment.end.idx),
			  static_cast<double>(candidate_path_distance),
			  static_cast<double>(candidate.dist.along),
			  static_cast<double>(candidate.dist.xtrack),
			  static_cast<double>(candidate.dist.on_segment));

		bool is_priority_match = false;

		if (config.last_flown_loop_segment.is_loop) {
			// When the vehicle was last known to be on a loop edge, prefer that exact edge over a nearby
			// nominal segment candidate to avoid "forgetting" the active loop context across replans.
			is_priority_match = config.last_flown_loop_segment.start.idx == candidate.segment.start.idx
					    && config.last_flown_loop_segment.end.idx == candidate.segment.end.idx;

			if (is_priority_match) {
				PX4_DEBUG("RTL SRP UAV proj prioritizing cand %u (loop segment match)",
					  static_cast<unsigned>(i));
			}

		} else {
			is_priority_match = isIndexInProjectionSegment(candidate.segment, mission_index,
					    config.is_flying_reverse);

			if (is_priority_match) {
				PX4_DEBUG("RTL SRP UAV proj prioritizing cand %u (segment match)",
					  static_cast<unsigned>(i));
			}
		}

		const bool is_shorter_path = PX4_ISFINITE(candidate_path_distance)
					     && candidate_path_distance < min_path_distance;

		if (is_priority_match || is_shorter_path) {
			min_path_distance = candidate_path_distance;
			best_candidate_index = i;

			if (is_priority_match) {
				break;
			}
		}
	}

	if (best_candidate_index < 0) {
		PX4_ERR("RTL SRP UAV proj invalid best candidate index %d, defaulting to first candidate",
			best_candidate_index);
		best_candidate_index = 0;
	}

	projection_context = {};
	projection_context.vehicle_pos = vehicle_position;
	projection_context.mission_index = mission_index;
	projection_context.projection = candidate_buffer.candidates[best_candidate_index];
	projection_context.is_flying_reverse = config.is_flying_reverse;
	projection_context.vehicle_velocity_north = config.vehicle_velocity_north;
	projection_context.vehicle_velocity_east = config.vehicle_velocity_east;
	projection_context.vehicle_velocity_valid = config.vehicle_velocity_valid;
	projection_context.dist_along_to_route_end = dist_along_to_route_end;
	projection_context.mission_loops_remaining = loops_remaining;

	PX4_DEBUG("RTL SRP UAV proj selected cand %d (of %u) on seg [%u->%u], path_dist=%.1f",
		  best_candidate_index,
		  candidate_buffer.count,
		  static_cast<unsigned>(projection_context.projection.segment.start.idx),
		  static_cast<unsigned>(projection_context.projection.segment.end.idx),
		  static_cast<double>(min_path_distance));

	if (projection_context.projection.segment.is_loop) {
		projection_context.loop_ctx = buildLoopContext(projection_context.projection, config.home_altitude_amsl);
	}

	if (failure_reason != nullptr) {
		*failure_reason = FailureReason::None;
	}

	return projection_context.valid();
}

/** @brief Accumulate the 2D mission distance between two position-bearing mission indices. */
float RtlRoutePlanner::accumulateRouteDistance(uint16_t from_index, uint16_t to_index, float home_altitude_amsl) const
{
	if (from_index > to_index || to_index >= _provider.missionCount()) {
		return NAN;
	}

	if (from_index == to_index) {
		return 0.f;
	}

	Position previous_position{};
	bool have_previous = false;
	float accumulated = 0.f;

	for (uint16_t index = from_index; index <= to_index; ++index) {
		mission_item_s mission_item{};

		if (!_provider.loadMissionItem(index, mission_item)) {
			return NAN;
		}

		Position current_position{};

		if (!extractMissionPosition(mission_item, home_altitude_amsl, current_position)) {
			continue;
		}

		if (!have_previous) {
			previous_position = current_position;
			have_previous = true;
			continue;
		}

		const float segment_length = get_distance_to_next_waypoint(previous_position.lat, previous_position.lon,
					     current_position.lat, current_position.lon);

		if (PX4_ISFINITE(segment_length)) {
			accumulated += segment_length;
		}

		previous_position = current_position;
	}

	return accumulated;
}

/** @brief Build the loop-jump context used when the vehicle is projected onto a DO_JUMP segment. */
RtlRoutePlanner::LoopContext RtlRoutePlanner::buildLoopContext(const SegmentCandidate &vehicle_projection,
		float home_altitude_amsl) const
{
	LoopContext loop_context{};

	if (!vehicle_projection.segment.valid() || !vehicle_projection.segment.is_loop) {
		return loop_context;
	}

	loop_context.segment = vehicle_projection.segment;
	loop_context.segment_positions = vehicle_projection.segment_positions;
	loop_context.along.start = vehicle_projection.dist.along - vehicle_projection.dist.on_segment;
	loop_context.along.end = accumulateRouteDistance(0, vehicle_projection.segment.end.idx, home_altitude_amsl);

	PX4_DEBUG("RTL SRP loop ctx: seg[%u-%u], along[%.1f, %.1f], loops remaining: %u",
		  static_cast<unsigned>(loop_context.segment.start.idx),
		  static_cast<unsigned>(loop_context.segment.end.idx),
		  static_cast<double>(loop_context.along.start),
		  static_cast<double>(loop_context.along.end),
		  static_cast<unsigned>(loop_context.segment.loops_remaining));

	return loop_context;
}

/** @brief Force or infer the direction used to reach a goal from the projected vehicle location. */
bool RtlRoutePlanner::mustFlyReverse(float goal_dist_along, float projection_dist_along,
				     PathDirectionMode direction_mode) const
{
	switch (direction_mode) {
	case PathDirectionMode::ForceNominal:
		return false;

	case PathDirectionMode::ForceReverse:
		return true;

	case PathDirectionMode::Auto:
	default:
		return goal_dist_along < projection_dist_along;
	}
}

/** @brief Compute the desired course used for fixed-wing U-turn detection. */
void RtlRoutePlanner::computeDesiredCourseVector(const ProjectionContext &projection_context, bool will_fly_reverse,
		float &desired_course_north, float &desired_course_east) const
{
	static constexpr float kSmallLengthM = 5.f;
	static constexpr float kFarFromRouteM = 10.f;
	matrix::Vector2f desired_course_vec{};

	if (projection_context.projection.dist.segment_length < kSmallLengthM
	    || projection_context.projection.dist.xtrack > kFarFromRouteM) {
		get_vector_to_next_waypoint(projection_context.vehicle_pos.lat, projection_context.vehicle_pos.lon,
					    projection_context.projection.projection.lat, projection_context.projection.projection.lon,
					    &desired_course_vec(0), &desired_course_vec(1));

	} else {
		const Position &segment_start = will_fly_reverse ? projection_context.projection.segment_positions.end
						: projection_context.projection.segment_positions.start;
		const Position &segment_end = will_fly_reverse ? projection_context.projection.segment_positions.start
					      : projection_context.projection.segment_positions.end;

		get_vector_to_next_waypoint(segment_start.lat, segment_start.lon,
					    segment_end.lat, segment_end.lon,
					    &desired_course_vec(0), &desired_course_vec(1));
	}

	desired_course_north = desired_course_vec(0);
	desired_course_east = desired_course_vec(1);
}

/** @brief Check whether a fixed-wing route change implies an immediate U-turn. */
bool RtlRoutePlanner::uTurnRequired(const ProjectionContext &projection_context, const Config &config,
				    bool will_fly_reverse) const
{
	if (!(config.vehicle_is_fixed_wing || config.vehicle_in_transition_to_fw)) {
		return false;
	}

	if (!projection_context.vehicle_velocity_valid
	    || !PX4_ISFINITE(projection_context.vehicle_velocity_north)
	    || !PX4_ISFINITE(projection_context.vehicle_velocity_east)) {
		return false;
	}

	float desired_course_north = NAN;
	float desired_course_east = NAN;
	computeDesiredCourseVector(projection_context, will_fly_reverse, desired_course_north, desired_course_east);

	if (!PX4_ISFINITE(desired_course_north) || !PX4_ISFINITE(desired_course_east)) {
		return false;
	}

	matrix::Vector2f local_velocity{projection_context.vehicle_velocity_north, projection_context.vehicle_velocity_east};
	matrix::Vector2f desired_course{desired_course_north, desired_course_east};

	if (local_velocity.norm_squared() <= FLT_EPSILON || desired_course.norm_squared() <= FLT_EPSILON) {
		return false;
	}

	return local_velocity.dot(desired_course) < 0.f;
}

/**
 * @brief Compute the shortest along-route path from the vehicle projection to a goal projection.
 *
 * Case 1 — Normal path (vehicle NOT on a DO_JUMP loop, or goal on the same loop):
 *   - Direction decision: reverse if goal_dist_along < vehicle projection dist_along.
 *   - Distance = |goal_dist_along − vehicle_dist_along|.
 *   - FW/VTOL-FW U-turn penalty: +4000 m when reversing would require an immediate turn-around.
 *   - First item selection: when direction is unchanged and the mission index lies within the
 *     projected segment, choose the segment start (unless already at end) so the join target
 *     stays on the current leg.  When direction changes, choose start if reverse, end otherwise.
 *
 * Case 2 — Vehicle on a DO_JUMP loop, goal outside the loop:
 *   - Path A (complete loop): remaining loop distance + distance from loop end to goal.
 *   - Path B (reverse loop):  already-travelled loop distance + distance from loop start to goal.
 *   - If loops_remaining > 0: force Path A (must complete the loop iteration).
 *   - Otherwise: choose the cheaper path.
 *   - Each path independently evaluates its FW U-turn penalty.
 */
RtlRoutePlanner::Path RtlRoutePlanner::findShortestPath(uint16_t goal_segment_end_idx, float goal_dist_along,
		const ProjectionContext &projection_context, const Config &config,
		PathDirectionMode direction_mode) const
{
	Path path{};
	Position first_item_location = projection_context.projection.segment_positions.end;
	const bool goal_is_on_jump_segment = (goal_segment_end_idx == projection_context.loop_ctx.segment.end.idx);
	const bool on_jump_segment_and_goal_elsewhere = projection_context.loop_ctx.valid() && !goal_is_on_jump_segment;

	const float u_turn_penalty = config.u_turn_penalty_m;
	const auto calculate_cost = [u_turn_penalty](float raw_distance, bool u_turn_required) {
		return raw_distance + (u_turn_required ? u_turn_penalty : 0.f);
	};

	// --- Case 1: Normal path (no active loop, or goal on the same loop) ---
	if (!on_jump_segment_and_goal_elsewhere) {
		const bool will_fly_reverse = mustFlyReverse(goal_dist_along, projection_context.projection.dist.along,
					      direction_mode);
		const float abs_distance_projection_to_goal = fabsf(goal_dist_along - projection_context.projection.dist.along);

		path.direction_reversed = will_fly_reverse;
		path.u_turn_required = uTurnRequired(projection_context, config, will_fly_reverse);
		path.dist = calculate_cost(abs_distance_projection_to_goal, path.u_turn_required);

		// Choose which segment endpoint becomes the first target for route following.
		bool choose_item_start = false;
		const bool direction_change = (projection_context.is_flying_reverse != will_fly_reverse);

		if (!direction_change
		    && isIndexInProjectionSegment(projection_context.projection.segment,
						  projection_context.mission_index,
						  projection_context.is_flying_reverse)) {
			// No direction change, still on the same segment: prefer segment start unless
			// already past it, so the mission stays anchored on the current leg.
			choose_item_start = projection_context.mission_index < projection_context.projection.segment.end.idx;

		} else {
			// Direction change or off-segment: choose start when reversing, end when nominal.
			choose_item_start = will_fly_reverse;
		}

		if (choose_item_start) {
			path.first_item_index = projection_context.projection.segment.start.idx;
			path.first_item_cmd = projection_context.projection.segment.start.nav_cmd;
			first_item_location = projection_context.projection.segment_positions.start;

		} else {
			path.first_item_index = projection_context.projection.segment.end.idx;
			path.first_item_cmd = projection_context.projection.segment.end.nav_cmd;
			first_item_location = projection_context.projection.segment_positions.end;
		}

	} else {
		// --- Case 2: Vehicle on a DO_JUMP loop, goal outside the loop ---

		// Path A: complete the remaining loop distance then continue to the goal.
		const float dist_jump_remaining = fabsf(projection_context.projection.dist.segment_length
							- projection_context.projection.dist.on_segment);
		const float path_a_raw_cost = dist_jump_remaining
					      + fabsf(goal_dist_along - projection_context.loop_ctx.along.end);
		const bool is_rev_from_loop_end = mustFlyReverse(goal_dist_along, projection_context.loop_ctx.along.end,
						  direction_mode);
		const bool path_a_u_turn_required = uTurnRequired(projection_context, config, is_rev_from_loop_end);
		const float path_a_cost = calculate_cost(path_a_raw_cost, path_a_u_turn_required);

		// If loops remain, we must complete the current iteration — force Path A.
		if (projection_context.loop_ctx.segment.loops_remaining > 0) {
			path.first_item_index = projection_context.loop_ctx.segment.end.idx;
			path.first_item_cmd = projection_context.loop_ctx.segment.end.nav_cmd;
			path.direction_reversed = is_rev_from_loop_end;
			path.u_turn_required = path_a_u_turn_required;
			path.dist = path_a_cost;
			first_item_location = projection_context.projection.segment_positions.end;

		} else {
			// Path B: backtrack the already-travelled loop distance then continue to the goal.
			const float dist_jump_travelled = projection_context.projection.dist.on_segment;
			const float path_b_raw_cost = dist_jump_travelled
						      + fabsf(goal_dist_along - projection_context.loop_ctx.along.start);
			const bool is_rev_from_loop_start = mustFlyReverse(goal_dist_along, projection_context.loop_ctx.along.start,
							    direction_mode);
			const bool path_b_u_turn_required = uTurnRequired(projection_context, config, is_rev_from_loop_start);
			const float path_b_cost = calculate_cost(path_b_raw_cost, path_b_u_turn_required);

			// Select the cheaper of Path A (complete loop) vs Path B (reverse loop).
			if (path_a_cost < path_b_cost) {
				path.first_item_index = projection_context.loop_ctx.segment.end.idx;
				path.first_item_cmd = projection_context.loop_ctx.segment.end.nav_cmd;
				path.direction_reversed = is_rev_from_loop_end;
				path.u_turn_required = path_a_u_turn_required;
				path.dist = path_a_cost;
				first_item_location = projection_context.projection.segment_positions.end;

			} else {
				path.first_item_index = projection_context.loop_ctx.segment.start.idx;
				path.first_item_cmd = projection_context.loop_ctx.segment.start.nav_cmd;
				path.direction_reversed = is_rev_from_loop_start;
				path.u_turn_required = path_b_u_turn_required;
				path.dist = path_b_cost;
				first_item_location = projection_context.projection.segment_positions.start;
			}
		}
	}

	const float dist_to_first_item = get_distance_to_next_waypoint(first_item_location.lat, first_item_location.lon,
					 projection_context.vehicle_pos.lat, projection_context.vehicle_pos.lon);
	path.in_first_item_acc_rad = PX4_ISFINITE(dist_to_first_item) && dist_to_first_item < config.acceptance_radius;

	if (on_jump_segment_and_goal_elsewhere) {
		PX4_DEBUG("RTL SRP path on loop jump [A,B], loop_along[%.1f, %.1f], loops remaining: %u",
			  static_cast<double>(projection_context.loop_ctx.along.start),
			  static_cast<double>(projection_context.loop_ctx.along.end),
			  static_cast<unsigned>(projection_context.loop_ctx.segment.loops_remaining));
	}

	PX4_DEBUG("RTL SRP path: idx: %d, cmd %u, rev? %u, u-turn? %u, path dist: %.1f, in_rad? %u",
		  static_cast<int>(path.first_item_index),
		  static_cast<unsigned>(path.first_item_cmd),
		  static_cast<unsigned>(path.direction_reversed),
		  static_cast<unsigned>(path.u_turn_required),
		  static_cast<double>(path.dist),
		  static_cast<unsigned>(path.in_first_item_acc_rad));
	PX4_DEBUG("RTL SRP path: dist_to_first_item %.1f, uav_along %.1f, goal_along %.1f",
		  static_cast<double>(dist_to_first_item),
		  static_cast<double>(projection_context.projection.dist.along),
		  static_cast<double>(goal_dist_along));

	return path;
}

/** @brief Allow multicopters already close to a safe point to skip the route join and fly straight to it. */
bool RtlRoutePlanner::directToSafePoint(const Position &safe_point_position, const Position &vehicle_position,
					const Config &config) const
{
	if (!config.is_multicopter) {
		return false;
	}

	const float dist = get_distance_to_next_waypoint(vehicle_position.lat, vehicle_position.lon,
			   safe_point_position.lat, safe_point_position.lon);

	return PX4_ISFINITE(dist) && dist < config.direct_acceptance_radius;
}

/**
 * @brief Scan every safe point, project it onto the mission, and pick the best route-follow goal.
 *
 * Algorithm:
 *   1. Load all safe points from dataman into a batch (up to MAX_SAFE_POINT_BATCH = 64).
 *   2. Project every safe point onto the mission geometry in a single batched scan, producing up
 *      to 3 local-minimum candidates per safe point.
 *   3. For each safe point, evaluate every valid candidate:
 *      - Loop candidates are only considered when the vehicle itself is on a loop edge.
 *      - findShortestPath() computes the along-route cost from vehicle projection to candidate.
 *      - Track the candidate with the lowest path cost.
 *   4. First priority: if the vehicle is a multicopter already within direct acceptance radius of
 *      any safe point, select it immediately and stop (direct-to-safe-point shortcut).
 *   5. Second priority: select the safe point with the shortest valid path cost.
 *   6. Record the winning safe point's position, branch-off segment, and projection.
 */
RtlRoutePlanner::Selection RtlRoutePlanner::selectSafePoint(const ProjectionContext &projection_context,
		const Config &config) const
{
	Selection selection{};

	if (!projection_context.valid()) {
		return selection;
	}

	// TODO: implement geofence-aware pruning: reject safe points and vehicle projections
	// that would require crossing a geofence boundary.

	s_safe_point_batch = {};

	if (!loadSafePointBatch(config.home_altitude_amsl, s_safe_point_batch)) {
		PX4_DEBUG("RTL SRP search: no safe points available");
		return selection;
	}

	FailureReason failure_reason = FailureReason::Unknown;

	if (!findProjectionCandidatesBatch(INT32_MAX, config.home_altitude_amsl, false,
					   config.safe_point_projection_search_dist, Segment{},
					   s_safe_point_batch, nullptr, nullptr, nullptr, &failure_reason)) {
		PX4_DEBUG("RTL SRP safe point batch scan failed: %s", failureReasonString(failure_reason));
		return selection;
	}

	for (uint8_t batch_index = 0; batch_index < s_safe_point_batch.count; ++batch_index) {
		const Position &safe_point_position = s_safe_point_batch.items[batch_index].position;
		const int32_t safe_point_index = s_safe_point_batch.items[batch_index].source_index;
		const CandidateBuffer &candidate_buffer = s_safe_point_batch.items[batch_index].candidate_buffer;
		Path best_path{};
		int best_projection_index = -1;

		for (uint8_t candidate_index = 0; candidate_index < candidate_buffer.count; ++candidate_index) {
			const SegmentCandidate &projection_candidate = candidate_buffer.candidates[candidate_index];

			PX4_DEBUG("RTL SRP safe point %d cand %u b_off[%u;%u]",
				  static_cast<int>(safe_point_index),
				  static_cast<unsigned>(candidate_index),
				  static_cast<unsigned>(projection_candidate.segment.start.idx),
				  static_cast<unsigned>(projection_candidate.segment.end.idx));

			if (projection_candidate.segment.is_loop && !projection_context.loop_ctx.valid()) {
				PX4_DEBUG("RTL SRP safe point %d loop candidate skipped, vehicle not on loop jump",
					  static_cast<int>(safe_point_index));
				continue;
			}

			// TODO: include the cross-track in the computation of the shortest path to ensure that we minimize the overall distance.
			const Path path = findShortestPath(projection_candidate.segment.end.idx,
							   projection_candidate.dist.along,
							   projection_context, config);

			if (!path.valid()) {
				continue;
			}

			if (path.dist < best_path.dist) {
				best_path = path;
				best_projection_index = candidate_index;
			}
		}

		if (best_projection_index < 0) {
			PX4_DEBUG("RTL SRP safe point %d: no valid projection (nb cand: %u)",
				  static_cast<int>(safe_point_index),
				  static_cast<unsigned>(candidate_buffer.count));
			continue;
		}

		const bool direct_to_safe_point = directToSafePoint(safe_point_position, projection_context.vehicle_pos, config);

		if (!selection.found || direct_to_safe_point || best_path.dist < selection.path.dist) {
			selection.found = true;
			selection.safe_point_found = true;
			selection.goal_type = GoalType::SafePoint;
			selection.path = best_path;
			selection.safe_point_index = safe_point_index;
			selection.safe_point_position = safe_point_position;
			selection.goal_position = safe_point_position;
			selection.branch_off_segment = candidate_buffer.candidates[best_projection_index].segment;
			selection.branch_off_projection = candidate_buffer.candidates[best_projection_index].projection;
			selection.direct_to_safe_point = direct_to_safe_point;
		}

		if (selection.direct_to_safe_point) {
			break;
		}
	}

	if (selection.found) {
		if (!selection.valid()) {
			PX4_ERR("RTL SRP selected safe point is not valid");
			return {};
		}

		PX4_DEBUG("RTL SRP closest safe point %d rev? %u trgt: %d b_off[%u;%u] direct? %u",
			  static_cast<int>(selection.safe_point_index),
			  static_cast<unsigned>(selection.path.direction_reversed),
			  static_cast<int>(selection.path.first_item_index),
			  static_cast<unsigned>(selection.branch_off_segment.start.idx),
			  static_cast<unsigned>(selection.branch_off_segment.end.idx),
			  static_cast<unsigned>(selection.direct_to_safe_point));
	}

	return selection;
}

/** @brief Fall back to the closer mission endpoint when no safe point can be used. */
RtlRoutePlanner::Selection RtlRoutePlanner::selectMissionEndpointFallback(const ProjectionContext &projection_context,
		const Config &config) const
{
	Selection selection{};

	if (!projection_context.valid()) {
		return selection;
	}

	uint16_t takeoff_index{0};
	bool have_takeoff = findNextValidPositionIndex(0, config.home_altitude_amsl, takeoff_index);
	uint16_t land_index{0};
	bool have_land = findAttachedValidPositionIndex(static_cast<uint16_t>(_provider.missionCount() - 1),
			 config.home_altitude_amsl, land_index);

	Path path_to_takeoff{};
	Position takeoff_position{};
	const bool path_to_takeoff_valid = have_takeoff
					   && readValidMissionPosition(takeoff_index, config.home_altitude_amsl, takeoff_position)
					   && (path_to_takeoff = findShortestPath(takeoff_index, 0.f,
							   projection_context, config,
							   PathDirectionMode::ForceReverse)).valid();

	Path path_to_land{};
	Position land_position{};
	const bool path_to_land_valid = have_land
					&& readValidMissionPosition(land_index, config.home_altitude_amsl, land_position)
					&& (path_to_land = findShortestPath(land_index,
							projection_context.dist_along_to_route_end,
							projection_context, config,
							PathDirectionMode::ForceNominal)).valid();

	if (!path_to_takeoff_valid && !path_to_land_valid) {
		return selection;
	}

	selection.found = true;
	selection.safe_point_found = false;
	selection.direct_to_safe_point = false;

	if (!path_to_land_valid || (path_to_takeoff_valid && path_to_takeoff.dist < path_to_land.dist)) {
		selection.goal_type = GoalType::MissionTakeoff;
		selection.goal_position = takeoff_position;
		selection.path = path_to_takeoff;

	} else {
		selection.goal_type = GoalType::MissionLand;
		selection.goal_position = land_position;
		selection.path = path_to_land;
	}

	if (!selection.valid()) {
		PX4_ERR("RTL SRP fallback selection is not valid");
		return {};
	}

	PX4_DEBUG("RTL SRP no safe point found, falling back to %s (target idx %d, rev? %u)",
		  goalTypeString(selection.goal_type),
		  static_cast<int>(selection.path.first_item_index),
		  static_cast<unsigned>(selection.path.direction_reversed));

	return selection;
}

/** @brief Choose the best SRP goal, preferring a safe point and falling back to mission endpoints. */
RtlRoutePlanner::Selection RtlRoutePlanner::selectBestGoal(const ProjectionContext &projection_context,
		const Config &config) const
{
	Selection selection = selectSafePoint(projection_context, config);

	if (selection.found) {
		return selection;
	}

	return selectMissionEndpointFallback(projection_context, config);
}

/** @brief Return the VTOL state that applies to the segment ending at the given position anchor. */
uint8_t RtlRoutePlanner::getVtolStateAtAnchor(uint16_t anchor_index) const
{
	for (int32_t index = static_cast<int32_t>(anchor_index); index >= 0; --index) {
		mission_item_s mission_item{};

		if (!_provider.loadMissionItem(index, mission_item)) {
			return vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
		}

		if (mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION) {
			const int transition_mode = static_cast<int>(roundf(mission_item.params[0]));

			if (transition_mode == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC) {
				return vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;

			} else if (transition_mode == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW) {
				return vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;
			}
		}
	}

	return vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
}

/** @brief Map a target index to the segment-end anchor that defines its VTOL state. */
bool RtlRoutePlanner::findSegmentAnchorForTargetIndex(int32_t target_index, bool direction_reversed,
		uint16_t &anchor_index) const
{
	if (target_index < 0 || target_index >= _provider.missionCount()) {
		return false;
	}

	int32_t anchor_search_start_index = target_index;

	if (direction_reversed) {
		if (target_index >= (_provider.missionCount() - 1)) {
			return false;
		}

		anchor_search_start_index = target_index + 1;
	}

	for (int32_t index = anchor_search_start_index; index < _provider.missionCount(); ++index) {
		mission_item_s mission_item{};

		if (!_provider.loadMissionItem(index, mission_item)) {
			return false;
		}

		if (itemContainsPosition(mission_item)) {
			anchor_index = static_cast<uint16_t>(index);
			return true;
		}
	}

	return false;
}

/** @brief Detect whether joining or targeting a segment requires an immediate back transition. */
bool RtlRoutePlanner::joinRequiresBackTransition(int32_t target_index, bool direction_reversed,
		const Config &config) const
{
	if (!config.vehicle_is_vtol || target_index < 0 || _provider.missionCount() == 0) {
		return false;
	}

	if (!(config.vehicle_is_fixed_wing || config.vehicle_in_transition_to_fw)) {
		return false;
	}

	uint16_t segment_anchor_index{};

	if (!findSegmentAnchorForTargetIndex(target_index, direction_reversed, segment_anchor_index)) {
		return false;
	}

	const uint8_t target_segment_state = getVtolStateAtAnchor(segment_anchor_index);
	return target_segment_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
}

/** @brief Return the transition action required to enter the segment containing the target index. */
RtlRoutePlanner::TransitionAction RtlRoutePlanner::transitionActionForTargetIndex(int32_t target_index,
		bool direction_reversed, const Config &config) const
{
	if (!config.vehicle_is_vtol) {
		return TransitionAction::None;
	}

	uint16_t segment_anchor_index{};

	if (!findSegmentAnchorForTargetIndex(target_index, direction_reversed, segment_anchor_index)) {
		return TransitionAction::None;
	}

	const uint8_t target_segment_state = getVtolStateAtAnchor(segment_anchor_index);
	const bool currently_fw = config.vehicle_is_fixed_wing || config.vehicle_in_transition_to_fw;

	if (target_segment_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC && currently_fw) {
		return TransitionAction::BackTransition;
	}

	if (target_segment_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW && !currently_fw) {
		return TransitionAction::FrontTransition;
	}

	return TransitionAction::None;
}

/** @brief Check if the vehicle is still close enough to the branch-off leg to skip rejoining the mission. */
bool RtlRoutePlanner::closeToBranchOffSegment(const Position &position, const Selection &selection,
		float acceptance_radius) const
{
	if (!position.valid() || !selection.branch_off_projection.valid() || !selection.goal_position.valid()
	    || !PX4_ISFINITE(acceptance_radius) || acceptance_radius <= 0.f) {
		PX4_ERR("RTL SRP invalid inputs to determine distance to branch-off segment");
		return false;
	}

	matrix::Vector2f branch_vector{};
	matrix::Vector2f position_vector{};
	get_vector_to_next_waypoint(selection.branch_off_projection.lat, selection.branch_off_projection.lon,
				    selection.goal_position.lat, selection.goal_position.lon,
				    &branch_vector(0), &branch_vector(1));
	get_vector_to_next_waypoint(selection.branch_off_projection.lat, selection.branch_off_projection.lon,
				    position.lat, position.lon,
				    &position_vector(0), &position_vector(1));

	const float branch_length_sq = branch_vector.norm_squared();
	const float t = (branch_length_sq > FLT_EPSILON) ? (position_vector.dot(branch_vector) / branch_length_sq) : 0.f;
	const float t_clamped = constrain(t, 0.f, 1.f);
	const matrix::Vector2f projected_position = branch_vector * t_clamped;
	const float xtrack = static_cast<matrix::Vector2f>(position_vector - projected_position).norm();

	return PX4_ISFINITE(xtrack) && xtrack < acceptance_radius;
}

/** @brief Build the full route-to-goal plan used by upstream SRP RTL execution. */
bool RtlRoutePlanner::planRouteToGoal(const Position &vehicle_position, int32_t mission_index,
				      const Config &config, Plan &plan, FailureReason *failure_reason) const
{
	plan = {};

	if (!collectVehicleProjection(vehicle_position, mission_index, config, plan.projection_context, failure_reason)) {
		return false;
	}

	// SRP should choose the shortest exit even if a DO_JUMP still has repeats pending in normal mission execution.
	if (plan.projection_context.loop_ctx.valid()) {
		plan.projection_context.loop_ctx.segment.loops_remaining = 0;
	}

	plan.selection = selectBestGoal(plan.projection_context, config);

	if (!plan.selection.found) {
		if (failure_reason != nullptr) {
			*failure_reason = FailureReason::NoValidCandidateFound;
		}

		return false;
	}

	if (!plan.selection.valid()) {
		if (failure_reason != nullptr) {
			*failure_reason = FailureReason::NoValidCandidateFound;
		}

		PX4_ERR("RTL SRP plan rejected, selected goal is not valid");
		return false;
	}

	plan.join_context.projection = plan.projection_context.projection.projection;
	plan.join_context.vtol_back_transition_required = joinRequiresBackTransition(plan.selection.path.first_item_index,
			plan.selection.path.direction_reversed, config);

	// Landing fallback keeps the current altitude if the landing item is already within XY acceptance radius.
	if (plan.selection.goal_type == GoalType::MissionLand && plan.selection.path.in_first_item_acc_rad) {
		plan.join_context.projection.alt = vehicle_position.alt;
		plan.join_context.skip_altitude_requirement = true;
	}

	if (failure_reason != nullptr) {
		*failure_reason = FailureReason::None;
	}

	PX4_DEBUG("RTL SRP plan goal=%s target=%d rev=%u direct=%u branch_off[%d;%d] bt=%u",
		  goalTypeString(plan.selection.goal_type),
		  static_cast<int>(plan.selection.path.first_item_index),
		  static_cast<unsigned>(plan.selection.path.direction_reversed),
		  static_cast<unsigned>(plan.selection.direct_to_safe_point),
		  static_cast<int>(plan.selection.branch_off_segment.start.idx),
		  static_cast<int>(plan.selection.branch_off_segment.end.idx),
		  static_cast<unsigned>(plan.join_context.vtol_back_transition_required));

	return plan.valid();
}
