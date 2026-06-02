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
 * @file mission_route_projection.cpp
 *
 * Mission segment scanning, projection math, candidate buffering, and vehicle
 * branch-in selection for mission-route planning.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_projection.h"

#include "mission_item_utils.h"

#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

#include <px4_platform_common/log.h>

using namespace math;

namespace mission_route
{
namespace
{

/** @brief Raw local projection of a reference point onto one route segment. */
struct RawSegmentProjection {
	bool valid{false};

	float xtrack{NAN};
	float segment_along{NAN};
	// Normalized position of the projection along the segment, clamped to [0, 1] (0 = start, 1 = end).
	float along_fraction{NAN};

	bool projection_on_start{false};
	bool projection_on_end{false};

	matrix::Vector2f projection_vector{0.f, 0.f};
};

RawSegmentProjection projectReferenceToSegment(const Position &reference_position,
		const SegmentPositions &segment_positions,
		const matrix::Vector2f &segment_vector,
		float segment_length,
		bool segment_has_no_length)
{
	RawSegmentProjection projection{};

	if (segment_has_no_length) {
		// If the segment is a point, the projection is the point itself.
		projection.xtrack = get_distance_to_next_waypoint(reference_position.lat, reference_position.lon,
				    segment_positions.end.lat, segment_positions.end.lon);
		projection.segment_along = 0.f;
		projection.along_fraction = 0.f;
		projection.projection_on_start = true;
		projection.projection_on_end = true;
		projection.valid = PX4_ISFINITE(projection.xtrack) && projection.xtrack >= 0.f;
		return projection;
	}

	// Orthogonal projection onto the segment.
	// segment_vector (A to B) is pre-computed by the caller once per segment.
	matrix::Vector2f reference_vector; // Vector A to P (segment start to reference point)

	get_vector_to_next_waypoint(segment_positions.start.lat, segment_positions.start.lon,
				    reference_position.lat, reference_position.lon,
				    &reference_vector(0), &reference_vector(1));

	// t = dot(A to P, A to B) / |A to B|^2, unclamped.
	const float path_len_sq = segment_vector.norm_squared();
	const float t = (path_len_sq > FLT_EPSILON) ? (reference_vector.dot(segment_vector) / path_len_sq) : 0.f;

	// If t is negative, (t * len) is negative which is < tol, so this check handles both cases.
	static constexpr float kCornerToleranceM = 0.05f;
	projection.projection_on_start = (t * segment_length) < kCornerToleranceM;
	projection.projection_on_end = ((1.f - t) * segment_length) < kCornerToleranceM;

	projection.along_fraction = constrain(t, 0.f, 1.f);
	projection.projection_vector = segment_vector * projection.along_fraction;
	projection.segment_along = projection.along_fraction * segment_length;
	projection.xtrack = static_cast<matrix::Vector2f>(reference_vector - projection.projection_vector).norm();
	projection.valid = PX4_ISFINITE(projection.xtrack) && projection.xtrack >= 0.f;

	return projection;
}

bool buildProjectionCandidate(const Segment &segment,
			      const SegmentPositions &segment_positions,
			      const RawSegmentProjection &projection,
			      float route_along,
			      float segment_length,
			      bool segment_has_no_length,
			      RouteProjectionCandidate &candidate)
{
	candidate = {};
	candidate.segment = segment;
	candidate.segment_positions = segment_positions;
	candidate.dist.xtrack = projection.xtrack;
	candidate.dist.route_along = route_along + projection.segment_along;
	candidate.dist.segment_length = segment_length;
	candidate.dist.segment_along = min(projection.segment_along, segment_length);

	if (segment_has_no_length) {
		candidate.projection = segment_positions.end;
		return candidate.valid();
	}

	// Reconstruct the projected lat/lon from the local NED offset vector.
	double lat_res;
	double lon_res;
	add_vector_to_global_position(segment_positions.start.lat, segment_positions.start.lon,
				      projection.projection_vector(0), projection.projection_vector(1),
				      &lat_res, &lon_res);
	candidate.projection.lat = lat_res;
	candidate.projection.lon = lon_res;
	candidate.projection.alt = segment_positions.start.alt
				   + projection.along_fraction * (segment_positions.end.alt - segment_positions.start.alt);

	return candidate.valid();
}

FailureReason positionLookupFailureReason(PositionLookupStatus status, FailureReason not_found_reason)
{
	switch (status) {
	case PositionLookupStatus::kLoadFailed:
		return FailureReason::kLoadFailed;

	case PositionLookupStatus::kInvalidPosition:
		return FailureReason::kPositionItemInvalid;

	case PositionLookupStatus::kNoPositionFound:
	default:
		return not_found_reason;
	}
}

} // namespace

PositionLookupStatus MissionRouteProjection::findNextValidPositionIndex(int32_t start_index, float home_altitude_amsl,
		int32_t &next_position_index) const
{
	if (start_index < 0) {
		return PositionLookupStatus::kNoPositionFound;
	}

	for (int32_t index = start_index; index < _provider.missionCount(); ++index) {
		mission_item_s mission_item{};

		if (!_provider.loadMissionItem(index, mission_item)) {
			return PositionLookupStatus::kLoadFailed;
		}

		if (!mission_item_contains_position(mission_item)) {
			continue;
		}

		Position position{};

		if (!extractMissionPosition(mission_item, home_altitude_amsl, position)) {
			return PositionLookupStatus::kInvalidPosition;
		}

		next_position_index = index;
		return PositionLookupStatus::kFound;
	}

	return PositionLookupStatus::kNoPositionFound;
}

bool MissionRouteProjection::findAttachedValidPositionIndex(int32_t start_index, float home_altitude_amsl,
		int32_t &attached_position_index) const
{
	if (start_index < 0 || _provider.missionCount() <= 0 || start_index >= _provider.missionCount()) {
		return false;
	}

	for (int32_t index = start_index; index >= 0; --index) {
		mission_item_s mission_item{};

		if (!_provider.loadMissionItem(index, mission_item)) {
			return false;
		}

		if (mission_item_contains_position(mission_item)) {
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

bool MissionRouteProjection::prepareNextSegment(int32_t index, Segment &segment, SegmentPositions &segment_positions,
		float home_altitude_amsl, FailureReason &failure_reason) const
{
	mission_item_s mission_item{};

	if (!_provider.loadMissionItem(index, mission_item)) {
		failure_reason = FailureReason::kLoadFailed;
		return false;
	}

	segment.end.idx = index;
	segment.end.nav_cmd = mission_item.nav_cmd;
	segment.loops_remaining = 0;
	segment.is_loop = (mission_item.nav_cmd == NAV_CMD_DO_JUMP);

	if (segment.is_loop) {
		const int32_t remaining_loops = static_cast<int32_t>(mission_item.do_jump_repeat_count)
						- static_cast<int32_t>(mission_item.do_jump_current_count);
		segment.loops_remaining = static_cast<uint8_t>(constrain(remaining_loops, static_cast<int32_t>(0),
					  static_cast<int32_t>(UINT8_MAX)));

		if (mission_item.do_jump_mission_index < 0) {
			PX4_ERR("RTL invalid DO_JUMP target index %d", static_cast<int>(mission_item.do_jump_mission_index));
			failure_reason = FailureReason::kInternalError;
			return false;
		}

		int32_t jump_to_index{0};

		const PositionLookupStatus jump_status =
			findNextValidPositionIndex(mission_item.do_jump_mission_index, home_altitude_amsl, jump_to_index);

		if (jump_status != PositionLookupStatus::kFound) {
			failure_reason = positionLookupFailureReason(jump_status, FailureReason::kInternalError);
			return false;
		}

		if (!_provider.loadMissionItem(jump_to_index, mission_item)) {
			failure_reason = FailureReason::kLoadFailed;
			return false;
		}

		segment.end.idx = jump_to_index;
		segment.end.nav_cmd = mission_item.nav_cmd;
	}

	if (!mission_item_contains_position(mission_item)) {
		return false;
	}

	if (!extractMissionPosition(mission_item, home_altitude_amsl, segment_positions.end)) {
		failure_reason = FailureReason::kPositionItemInvalid;
		return false;
	}

	return true;
}

bool MissionRouteProjection::localMinimumOnSegment(bool proj_on_start, bool proj_on_end,
		bool prev_proj_on_end, bool jumping, bool last_segment) const
{
	const bool proj_on_corner = proj_on_start || proj_on_end;

	// DO_JUMP loop-edge corner projections are rejected because those corners
	// already belong to their nominal route segments, so only
	// interior projections on the loop jump segment are kept:
	if (jumping) {
		return !proj_on_corner;
	}

	// The terminal route endpoint (last segment, proj_on_end) is accepted
	// because there is no following segment to compare against.
	if (last_segment && proj_on_end) {
		return true;
	}

	/** Nominal case:

	 1. Interior projection (!proj_on_corner) are always a local minimum.

	 2. Corner projection is a local minimum when both the previous and the current segment
	    project onto the same shared corner. This happens when two consecutive segments form
	    a V-shape and the reference point is closest to the apex:

	     prev seg  curr seg
	       A \    / C        The reference point P projects onto corner B
	          \  /           from both segments. Both projections land on B,
	         B \/            so prev_proj_on_end && proj_on_start -> local min.

	            ^
	            P
	*/
	return !proj_on_corner || (proj_on_start && prev_proj_on_end);
}

bool MissionRouteProjection::validateCandidate(const RouteProjectionCandidate &candidate) const
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

void MissionRouteProjection::insertCandidateSorted(ProjectionCandidateBuffer &candidate_buffer,
		const RouteProjectionCandidate &candidate) const
{
	uint8_t insert_index = 0;
	const uint8_t buffer_size = min(candidate_buffer.count, kMaxSegmentCandidates);

	while (insert_index < buffer_size
	       && candidate_buffer.candidates[insert_index].dist.xtrack <= candidate.dist.xtrack) {
		++insert_index;
	}

	if (insert_index >= kMaxSegmentCandidates) {
		return;
	}

	int16_t shift_dest = (buffer_size == kMaxSegmentCandidates) ? buffer_size - 1 : buffer_size;

	for (int16_t j = shift_dest; j > insert_index; --j) {
		candidate_buffer.candidates[j] = candidate_buffer.candidates[j - 1];
	}

	candidate_buffer.candidates[insert_index] = candidate;
	candidate_buffer.count = min(static_cast<uint8_t>(buffer_size + 1), kMaxSegmentCandidates);
}

void MissionRouteProjection::pruneProjectionCandidates(ProjectionCandidateBuffer &candidate_buffer,
		float xtrack_limit) const
{
	const uint8_t buffer_size = min(candidate_buffer.count, kMaxSegmentCandidates);

	if (buffer_size == 0) {
		return;
	}

	for (uint8_t index = buffer_size - 1U; index < buffer_size; --index) {
		if (candidate_buffer.candidates[index].dist.xtrack <= xtrack_limit) {
			candidate_buffer.count = index + 1U;
			return;
		}

		if (index < 1U) {
			break;
		}
	}

	candidate_buffer.count = 0;
}

bool MissionRouteProjection::fillCurrentSegmentAlongIfNeeded(int32_t mission_index,
		const Segment &segment_to_consider, bool is_flying_reverse,
		float route_along, float segment_length,
		const Segment &last_flown_loop_segment,
		SegmentDistanceAlong &current_segment_along) const
{
	if (last_flown_loop_segment.validLoop()) {
		if (segment_to_consider.start.idx == last_flown_loop_segment.start.idx) {
			current_segment_along.start = route_along;
			PX4_DEBUG("RTL fill current_segment_along.start on loop: %.1f",
				  static_cast<double>(current_segment_along.start));

		} else if (segment_to_consider.start.idx == last_flown_loop_segment.end.idx) {
			current_segment_along.end = route_along;
			PX4_DEBUG("RTL fill current_segment_along.end on loop: %.1f",
				  static_cast<double>(current_segment_along.end));
		}

		return current_segment_along.valid();
	}

	if (mission_index == 0 && !is_flying_reverse) {
		current_segment_along.start = 0.f;
		current_segment_along.end = 0.f;
		PX4_DEBUG("RTL fill current_segment_along with zeros for first mission item");
		return true;
	}

	if (isIndexInProjectionSegment(segment_to_consider, mission_index, is_flying_reverse)) {
		current_segment_along.start = route_along;
		current_segment_along.end = route_along + segment_length;
		PX4_DEBUG("RTL fill current_segment_along nominal case: [%.3f, %.3f]",
			  static_cast<double>(current_segment_along.start),
			  static_cast<double>(current_segment_along.end));
		return true;
	}

	return false;
}

bool MissionRouteProjection::isIndexInProjectionSegment(const Segment &projection_segment, int32_t mission_index,
		bool is_flying_reverse) const
{
	if (!projection_segment.valid()) {
		return false;
	}

	// Check if the vehicle is on the same segment as the current mission idx
	// In nominal direction we navigate [start -> end], and in reverse direction [end -> start]
	// All segments are defined between consecutive position items in nominal direction [start, end]
	// E.g. we have segment [start -> end] = [2 -> 4], with 3 a non-position item.
	// 	- flying reverse: we are on this segment if we target 2 or 3
	// 	- flying nominal: we are on this segment if we target 3 or 4
	if (is_flying_reverse) {
		// flying reverse:
		// (mission_idx >= segment_start) --> valid options: [2,3,4,5,...]
		// && (mission_idx < segment_end) --> valid options: [2,3]
		return mission_index >= projection_segment.start.idx && mission_index < projection_segment.end.idx;
	}

	// flying nominal:
	// (mission_idx > segment_start) --> valid options: [3,4,5,...]
	// && (mission_idx <= segment_end) --> valid options: [3,4]
	return mission_index > projection_segment.start.idx && mission_index <= projection_segment.end.idx;
}

void MissionRouteProjection::processCandidateForSegment(const Position &reference_position,
		const RouteSegmentView &segment,
		float xtrack_margin_m,
		CandidateSearchState &state,
		ProjectionCandidateBuffer &candidate_buffer,
		ProjectionScanStats &stats) const
{
	const RawSegmentProjection projection = projectReferenceToSegment(reference_position, segment.positions,
						segment.segment_vector, segment.length_m, segment.zero_length_xy);

	state.projection_on_end_for_segment = projection.projection_on_end;

	if (!localMinimumOnSegment(projection.projection_on_start, projection.projection_on_end,
				   state.prev_projection_on_end, segment.segment.is_loop, segment.last_segment)) {
		return;
	}

	stats.local_min_found++;

	// Reject non-finite or out-of-window projections.
	// We abort here before doing LatLon reconstruction (which is more expensive).
	if (!projection.valid || projection.xtrack >= state.xtrack_limit) {
		return;
	}

	RouteProjectionCandidate candidate{};

	if (!buildProjectionCandidate(segment.segment, segment.positions, projection, segment.route_along_start_m,
				      segment.length_m, segment.zero_length_xy, candidate)) {
		return;
	}

	if (!validateCandidate(candidate)) {
		return;
	}

	stats.valid_candidate_found++;

	if (projection.xtrack < state.min_xtrack) {
		// A new closest projection tightens the search window, so prune stale candidates first.
		state.min_xtrack = projection.xtrack;
		state.xtrack_limit = state.min_xtrack + xtrack_margin_m;
		pruneProjectionCandidates(candidate_buffer, state.xtrack_limit);
	}

	insertCandidateSorted(candidate_buffer, candidate);
}

ProjectionScanResult MissionRouteProjection::findProjectionCandidates(const ProjectionScanRequest &request,
		ProjectionReferenceBatch &batch) const
{
	ProjectionScanResult result{};
	result.failure_reason = FailureReason::kUnknown;

	if (batch.count == 0 || batch.count > kMaxSafePointBatch || !(request.xtrack_margin_m >= 0.f)) {
		result.failure_reason = FailureReason::kInvalidRequest;
		return result;
	}

	if (_provider.missionCount() < 2) {
		result.failure_reason = FailureReason::kNoValidWaypoints;
		return result;
	}

	for (uint8_t i = 0; i < batch.count; ++i) {
		batch.items[i].candidate_buffer.count = 0;
	}

	SegmentDistanceAlong current_segment_along{};
	// For branch-in selection, we need to know where the segment the vehicle is currently flying lies along the route
	// The flag is cleared once the bounds have been filled so we stop re-checking it on the remaining segments.
	bool need_current_segment_bounds = request.compute_current_segment_bounds;

	if (need_current_segment_bounds) {
		need_current_segment_bounds = !fillCurrentSegmentAlongIfNeeded(request.mission_index, Segment{},
					      request.is_flying_reverse, 0.f, 0.f,
					      request.last_flown_loop_segment,
					      current_segment_along);
	}

	// First and last mission indices need special handling.
	// A zero-length (stacked) segment is normally skipped, but at the very
	// first or very last position it is the only projection available there, so it must still be evaluated.
	// The last_position_index is also accepted as a local minimum because there is no next segment to compare it to.
	int32_t first_position_index{0};
	int32_t last_position_index{0};

	const PositionLookupStatus first_status =
		findNextValidPositionIndex(0, request.home_altitude_amsl, first_position_index);

	if (first_status != PositionLookupStatus::kFound) {
		result.failure_reason = positionLookupFailureReason(first_status, FailureReason::kNoValidWaypoints);
		return result;
	}

	if (!findAttachedValidPositionIndex(_provider.missionCount() - 1,
					    request.home_altitude_amsl, last_position_index)) {
		result.failure_reason = FailureReason::kNoValidWaypoints;
		return result;
	}

	Segment segment{};
	SegmentPositions segment_positions{};
	bool have_previous = false;
	float total_dist = 0.f;
	BatchSearchState batch_state{};

	// Scan the full mission once, evaluating segments against every batch item.
	for (int32_t index = first_position_index; index < _provider.missionCount(); ++index) {
		FailureReason loop_failure_reason = FailureReason::kUnknown;

		if (!prepareNextSegment(index, segment, segment_positions, request.home_altitude_amsl, loop_failure_reason)) {
			if (loop_failure_reason != FailureReason::kUnknown) {
				result.failure_reason = loop_failure_reason;
				return result;
			}

			continue;
		}

		if (!have_previous) {
			// A segment only exists once a second endpoint (or loop edge) has been processed.
			have_previous = true;
			segment.start = segment.end;
			segment_positions.start = segment_positions.end;
			continue;
		}

		batch_state.stats.segments_processed++;

		if (isLandingCmd(segment.end.nav_cmd)) {
			// In a nominal mission the landing waypoint's altitude is ignored while flying towards it:
			// the vehicle holds the previous waypoint's altitude until it reaches the landing point and
			// only then descends. Model the segment accordingly so the projected altitude does not ramp down.
			segment_positions.end.alt = segment_positions.start.alt;
		}

		RouteSegmentView segment_view{};
		segment_view.segment = segment;
		segment_view.positions = segment_positions;
		segment_view.route_along_start_m = total_dist;
		segment_view.last_segment = (segment.end.idx == last_position_index);
		segment_view.zero_length_xy =
			fabs(segment_positions.start.lat - segment_positions.end.lat) <= kCornerLatLonTolDeg
			&& fabs(segment_positions.start.lon - segment_positions.end.lon) <= kCornerLatLonTolDeg;

		const bool is_first_segment = (segment.start.idx == first_position_index);

		if (segment_view.zero_length_xy && !segment_view.last_segment && !is_first_segment) {
			// Interior zero-length stacks cannot contribute a unique XY projection candidate.
			if (need_current_segment_bounds) {
				need_current_segment_bounds = !fillCurrentSegmentAlongIfNeeded(request.mission_index, segment,
							      request.is_flying_reverse, total_dist, segment_view.length_m,
							      request.last_flown_loop_segment, current_segment_along);
			}

			if (!segment.is_loop) {
				segment.start = segment.end;
				segment_positions.start = segment_positions.end;
			}

			// Skip to the next segment without changing the xtrack state.
			continue;
		}

		// Compute expensive trig once per segment for all ProjectionReferenceBatch
		if (!segment_view.zero_length_xy) {
			segment_view.length_m = get_distance_to_next_waypoint(segment_positions.start.lat,
						segment_positions.start.lon, segment_positions.end.lat, segment_positions.end.lon);
			get_vector_to_next_waypoint(segment_positions.start.lat, segment_positions.start.lon,
						    segment_positions.end.lat, segment_positions.end.lon,
						    &segment_view.segment_vector(0), &segment_view.segment_vector(1));
		}

		for (uint8_t i = 0; i < batch.count; ++i) {
			processCandidateForSegment(batch.items[i].position, segment_view, request.xtrack_margin_m,
						   batch_state.candidate_states[i], batch.items[i].candidate_buffer,
						   batch_state.stats);
		}

		if (segment.is_loop) {
			// Loop edges do not advance the nominal along-route accumulator
			// because mission execution does not "walk through" them here.
			continue;
		}

		if (need_current_segment_bounds) {
			need_current_segment_bounds = !fillCurrentSegmentAlongIfNeeded(request.mission_index, segment,
						      request.is_flying_reverse, total_dist, segment_view.length_m,
						      request.last_flown_loop_segment, current_segment_along);
		}

		total_dist += segment_view.length_m;
		segment.start = segment.end;
		segment_positions.start = segment_positions.end;

		// Carry this segment's end-corner projection into the next iteration as its "previous", so the next
		// segment can detect a shared V-corner (prev_projection_on_end && projection_on_start -> local
		// minimum at the apex, see localMinimumOnSegment).
		// A zero-length-XY segment cannot form a V apex, so it never seeds a shared-corner match.
		for (uint8_t i = 0; i < batch.count; ++i) {
			batch_state.candidate_states[i].prev_projection_on_end =
				segment_view.zero_length_xy ? false : batch_state.candidate_states[i].projection_on_end_for_segment;
		}

		if (segment_view.last_segment) {
			break;
		}
	}

	result.current_segment_along = current_segment_along;
	result.route_length = total_dist;

	bool any_candidate_found = false;

	for (uint8_t i = 0; i < batch.count; ++i) {
		if (batch.items[i].candidate_buffer.count > 0) {
			any_candidate_found = true;
			break;
		}
	}

	PX4_DEBUG("RTL batch items: %u, segs: %u, mins: %u, valid: %u",
		  static_cast<unsigned>(batch.count),
		  static_cast<unsigned>(batch_state.stats.segments_processed),
		  static_cast<unsigned>(batch_state.stats.local_min_found),
		  static_cast<unsigned>(batch_state.stats.valid_candidate_found));

	if (any_candidate_found) {
		result.success = true;
		result.failure_reason = FailureReason::kNone;
		return result;
	}

	if (batch_state.stats.segments_processed == 0) {
		result.failure_reason = FailureReason::kNoSegmentsFound;

	} else if (batch_state.stats.local_min_found == 0) {
		result.failure_reason = FailureReason::kNoLocalMinFound;

	} else if (batch_state.stats.valid_candidate_found == 0) {
		result.failure_reason = FailureReason::kNoValidCandidateFound;

	} else {
		result.failure_reason = FailureReason::kUnknown;
	}

	return result;
}

struct MissionRouteProjection::BranchInSelectionResult {
	bool success{false};
	FailureReason failure_reason{FailureReason::kUnknown};
	RouteProjectionCandidate candidate{};
	int candidate_index{-1};
	float score_m{FLT_MAX};
};

MissionRouteProjection::BranchInSelectionResult MissionRouteProjection::selectBranchInCandidate(
	const ProjectionCandidateBuffer &candidate_buffer,
	const SegmentDistanceAlong &current_segment_along,
	int32_t mission_index,
	bool is_flying_reverse,
	const Segment &last_flown_loop_segment) const
{
	BranchInSelectionResult result{};
	int best_candidate_index = -1;
	float min_path_distance = FLT_MAX;
	SegmentDistanceAlong segment_along = current_segment_along;

	if (!segment_along.valid()) {
		PX4_ERR("RTL select UAV proj: invalid current segment (mission_idx=%d), setting to zero",
			static_cast<int>(mission_index));
		segment_along.start = 0.f;
		segment_along.end = 0.f;
	}

	for (uint8_t i = 0; i < candidate_buffer.count; ++i) {
		const RouteProjectionCandidate &candidate = candidate_buffer.candidates[i];
		const float dist_to_start = fabsf(candidate.dist.route_along - segment_along.start);
		const float dist_to_end = fabsf(candidate.dist.route_along - segment_along.end);
		const float projection_to_segment_dist = fminf(dist_to_start, dist_to_end);
		const float candidate_path_distance = candidate.dist.xtrack + projection_to_segment_dist;

		if (!PX4_ISFINITE(candidate_path_distance)) {
			PX4_DEBUG("RTL UAV proj cand %u skipped, non-finite path distance",
				  static_cast<unsigned>(i));
			continue;
		}

		PX4_DEBUG("RTL UAV proj cand %u on seg [%u->%u], path_dist=%.1f, along=%.1f xtrack=%.1f on_seg=%.1f",
			  static_cast<unsigned>(i),
			  static_cast<unsigned>(candidate.segment.start.idx),
			  static_cast<unsigned>(candidate.segment.end.idx),
			  static_cast<double>(candidate_path_distance),
			  static_cast<double>(candidate.dist.route_along),
			  static_cast<double>(candidate.dist.xtrack),
			  static_cast<double>(candidate.dist.segment_along));

		bool priority_match = false;

		if (last_flown_loop_segment.validLoop()) {
			priority_match = last_flown_loop_segment.start.idx == candidate.segment.start.idx
					 && last_flown_loop_segment.end.idx == candidate.segment.end.idx;

			if (priority_match) {
				PX4_DEBUG("RTL UAV proj prioritizing cand %u (loop segment match)", static_cast<unsigned>(i));
			}

		} else {
			priority_match = isIndexInProjectionSegment(candidate.segment, mission_index, is_flying_reverse);

			if (priority_match) {
				PX4_DEBUG("RTL UAV proj prioritizing cand %u (segment match)", static_cast<unsigned>(i));
			}
		}

		const bool is_shorter_path = candidate_path_distance < min_path_distance;

		if (priority_match || is_shorter_path) {
			min_path_distance = candidate_path_distance;
			best_candidate_index = i;

			if (priority_match) {
				break;
			}
		}
	}

	if (best_candidate_index < 0) {
		PX4_ERR("RTL UAV proj failed, no valid candidate selected");
		result.failure_reason = FailureReason::kNoValidCandidateFound;
		return result;
	}

	result.success = true;
	result.failure_reason = FailureReason::kNone;
	result.candidate = candidate_buffer.candidates[best_candidate_index];
	result.candidate_index = best_candidate_index;
	result.score_m = min_path_distance;
	return result;
}

VehicleProjectionResult MissionRouteProjection::collectVehicleProjection(const RoutePlanRequest &request,
		ProjectionReferenceBatch &batch) const
{
	VehicleProjectionResult result{};
	int32_t mission_index = request.mission_index;

	if (!request.vehicle_position.valid()) {
		result.failure_reason = FailureReason::kNoValidGlobalPos;
		return result;
	}

	if (_provider.missionCount() <= 0) {
		result.failure_reason = FailureReason::kNoValidWaypoints;
		return result;
	}

	if (mission_index < 0 || mission_index >= _provider.missionCount()) {
		PX4_ERR("RTL invalid mission index: %d (mission count: %d)",
			static_cast<int>(mission_index), static_cast<int>(_provider.missionCount()));
		result.failure_reason = FailureReason::kInvalidRequest;
		return result;
	}

	batch.items[0] = {};
	batch.count = 1;
	batch.items[0].position = request.vehicle_position;

	ProjectionScanRequest scan_request{};
	scan_request.home_altitude_amsl = request.config.parameters.home_altitude_amsl;
	scan_request.xtrack_margin_m = request.config.parameters.vehicle_projection_search_dist;
	scan_request.compute_current_segment_bounds = true;
	scan_request.mission_index = mission_index;
	scan_request.is_flying_reverse = request.config.state.is_flying_reverse;
	scan_request.last_flown_loop_segment = request.config.last_flown_loop_segment;

	const ProjectionScanResult scan_result = findProjectionCandidates(scan_request, batch);

	if (!scan_result.success) {
		result.failure_reason = scan_result.failure_reason;
		return result;
	}

	const ProjectionCandidateBuffer &candidate_buffer = batch.items[0].candidate_buffer;

	PX4_DEBUG("RTL vehicle projection: cands=%u current_segment[%.1f, %.1f] idx=%d",
		  static_cast<unsigned>(candidate_buffer.count),
		  static_cast<double>(scan_result.current_segment_along.start),
		  static_cast<double>(scan_result.current_segment_along.end),
		  static_cast<int>(mission_index));

	const BranchInSelectionResult branch_in = selectBranchInCandidate(candidate_buffer,
			scan_result.current_segment_along, mission_index,
			request.config.state.is_flying_reverse,
			request.config.last_flown_loop_segment);

	if (!branch_in.success) {
		result.failure_reason = branch_in.failure_reason;
		return result;
	}

	ProjectionContext projection_context{};
	projection_context.vehicle_position = request.vehicle_position;
	projection_context.mission_index = mission_index;
	projection_context.route_projection = branch_in.candidate;
	projection_context.is_flying_reverse = request.config.state.is_flying_reverse;
	projection_context.vehicle_vel_ne = request.config.state.velocity_ne;
	projection_context.velocity_valid = request.config.state.velocity_valid;
	projection_context.route_length = scan_result.route_length;
	// Use the repeat count from the selected projection loop itself. A later DO_JUMP elsewhere
	// in the mission must not overwrite the active loop state carried by this projection.
	projection_context.mission_loops_remaining = projection_context.route_projection.segment.validLoop()
			? projection_context.route_projection.segment.loops_remaining : 0;

	PX4_DEBUG("RTL UAV proj selected cand %d (of %u) on seg [%u->%u], path_dist=%.1f",
		  branch_in.candidate_index,
		  static_cast<unsigned>(candidate_buffer.count),
		  static_cast<unsigned>(projection_context.route_projection.segment.start.idx),
		  static_cast<unsigned>(projection_context.route_projection.segment.end.idx),
		  static_cast<double>(branch_in.score_m));

	if (projection_context.route_projection.segment.validLoop()) {
		projection_context.loop_context = buildLoopContext(projection_context.route_projection,
						  request.config.parameters.home_altitude_amsl);
	}

	if (!projection_context.valid()) {
		result.failure_reason = FailureReason::kInvalidProjectionContext;
		return result;
	}

	result.success = true;
	result.failure_reason = FailureReason::kNone;
	result.projection_context = projection_context;
	return result;
}

float MissionRouteProjection::accumulateRouteDistance(int32_t from_index, int32_t to_index,
		float home_altitude_amsl) const
{
	if (from_index < 0 || to_index < 0 || from_index > to_index || to_index >= _provider.missionCount()) {
		return NAN;
	}

	if (from_index == to_index) {
		return 0.f;
	}

	Position previous_position{};
	bool have_previous = false;
	float accumulated = 0.f;

	for (int32_t index = from_index; index <= to_index; ++index) {
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

LoopContext MissionRouteProjection::buildLoopContext(const RouteProjectionCandidate &vehicle_projection,
		float home_altitude_amsl) const
{
	LoopContext loop_context{};

	if (!vehicle_projection.segment.validLoop()) {
		return loop_context;
	}

	loop_context.segment = vehicle_projection.segment;
	loop_context.segment_positions = vehicle_projection.segment_positions;
	loop_context.along.start = vehicle_projection.dist.route_along - vehicle_projection.dist.segment_along;
	loop_context.along.end = accumulateRouteDistance(0, vehicle_projection.segment.end.idx, home_altitude_amsl);

	PX4_DEBUG("RTL loop ctx: seg[%u-%u], along[%.1f, %.1f], loops remaining: %u",
		  static_cast<unsigned>(loop_context.segment.start.idx),
		  static_cast<unsigned>(loop_context.segment.end.idx),
		  static_cast<double>(loop_context.along.start),
		  static_cast<double>(loop_context.along.end),
		  static_cast<unsigned>(loop_context.segment.loops_remaining));

	return loop_context;
}

} // namespace mission_route
