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

	// Orthogonal projection of the reference point P onto the segment A (start) -> B (end).
	matrix::Vector2f reference_vector; // A to P

	get_vector_to_next_waypoint(segment_positions.start.lat, segment_positions.start.lon,
				    reference_position.lat, reference_position.lon,
				    &reference_vector(0), &reference_vector(1));

	// t = dot(A to P, A to B) / |A to B|^2, unclamped.
	const float path_len_sq = segment_vector.norm_squared();
	const float t = (path_len_sq > FLT_EPSILON) ? (reference_vector.dot(segment_vector) / path_len_sq) : 0.f;

	// t < 0 makes (t * len) negative, so projections before the start also count as on-start.
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

/**
 * @brief Locates the along-route interval of the segment the vehicle is currently flying.
 *
 * Branch-in selection prefers the candidate closest along the route to where the vehicle
 * already is (mission continuity over raw distance), and that position is only known once
 * the scan has walked past the vehicle's segment. Feed every nominal segment to observe().
 */
class CurrentSegmentBoundsTracker
{
public:
	explicit CurrentSegmentBoundsTracker(const ProjectionScanRequest &request) : _request(request)
	{
		_wanted = request.compute_current_segment_bounds;

		// On the first mission item (flying nominal) the vehicle has not entered the route yet.
		if (_wanted && !request.active_jump_anchor.valid()
		    && request.mission_index == 0 && !request.is_flying_reverse) {
			_bounds.start = 0.f;
			_bounds.end = 0.f;
			_located = true;
			PX4_DEBUG("Route current_segment_along zero for first mission item");
		}
	}

	void observe(const RouteSegmentView &segment_view)
	{
		if (_wanted && !_located) {
			_located = fill(segment_view);
		}
	}

	const SegmentDistanceAlong &bounds() const { return _bounds; }

private:
	bool fill(const RouteSegmentView &segment_view)
	{
		const Segment &segment = segment_view.segment;
		const ActiveJumpAnchor &active_jump = _request.active_jump_anchor;

		if (active_jump.valid()) {
			// The loop interval spans from the jump target back to the jump item,
			// so its ends are collected from two different walk steps.
			if (segment.start.idx == active_jump.start_index) {
				_bounds.start = segment_view.route_along_start_m;

			} else if (segment.start.idx == active_jump.target_index) {
				_bounds.end = segment_view.route_along_start_m;
			}

			return _bounds.valid();
		}

		if (isIndexInProjectionSegment(segment, _request.mission_index, _request.is_flying_reverse)) {
			_bounds.start = segment_view.route_along_start_m;
			_bounds.end = segment_view.route_along_start_m + segment_view.length_m;
			PX4_DEBUG("Route current_segment_along: [%.3f, %.3f]",
				  static_cast<double>(_bounds.start), static_cast<double>(_bounds.end));
			return true;
		}

		return false;
	}

	const ProjectionScanRequest &_request;
	SegmentDistanceAlong _bounds{};
	bool _wanted{false};
	bool _located{false};
};

} // namespace

PositionLookupStatus RouteSegmentCursor::findNextValidPositionIndex(int32_t start_index,
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

		if (!extractMissionPosition(mission_item, _home_altitude_amsl, position)) {
			return PositionLookupStatus::kInvalidPosition;
		}

		next_position_index = index;
		return PositionLookupStatus::kFound;
	}

	return PositionLookupStatus::kNoPositionFound;
}

bool RouteSegmentCursor::findAttachedValidPositionIndex(int32_t start_index,
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

			if (extractMissionPosition(mission_item, _home_altitude_amsl, position)) {
				attached_position_index = index;
				return true;
			}

			return false;
		}
	}

	return false;
}

bool RouteSegmentCursor::prepareNextSegment(int32_t index, FailureReason &failure_reason)
{
	mission_item_s mission_item{};

	if (!_provider.loadMissionItem(index, mission_item)) {
		failure_reason = FailureReason::kLoadFailed;
		return false;
	}

	_segment.end.idx = index;
	_segment.end.nav_cmd = mission_item.nav_cmd;
	_segment.loops_remaining = 0;
	_segment.is_loop = (mission_item.nav_cmd == NAV_CMD_DO_JUMP);

	if (_segment.is_loop) {
		const int32_t remaining_loops = static_cast<int32_t>(mission_item.do_jump_repeat_count)
						- static_cast<int32_t>(mission_item.do_jump_current_count);
		_segment.loops_remaining = static_cast<uint8_t>(constrain(remaining_loops, static_cast<int32_t>(0),
					   static_cast<int32_t>(UINT8_MAX)));

		if (mission_item.do_jump_mission_index < 0) {
			PX4_ERR("Route invalid DO_JUMP target index %d", static_cast<int>(mission_item.do_jump_mission_index));
			failure_reason = FailureReason::kInternalError;
			return false;
		}

		int32_t jump_to_index{0};

		const PositionLookupStatus jump_status =
			findNextValidPositionIndex(mission_item.do_jump_mission_index, jump_to_index);

		if (jump_status != PositionLookupStatus::kFound) {
			failure_reason = positionLookupFailureReason(jump_status, FailureReason::kInternalError);
			return false;
		}

		if (!_provider.loadMissionItem(jump_to_index, mission_item)) {
			failure_reason = FailureReason::kLoadFailed;
			return false;
		}

		_segment.end.idx = jump_to_index;
		_segment.end.nav_cmd = mission_item.nav_cmd;
	}

	if (!mission_item_contains_position(mission_item)) {
		return false;
	}

	if (!extractMissionPosition(mission_item, _home_altitude_amsl, _positions.end)) {
		failure_reason = FailureReason::kPositionItemInvalid;
		return false;
	}

	return true;
}

bool RouteSegmentCursor::init()
{
	if (_provider.missionCount() < 2) {
		_failure_reason = FailureReason::kNoValidWaypoints;
		return false;
	}

	// Zero-length segments are only projectable at the route ends, so locate both ends up front.
	const PositionLookupStatus first_status = findNextValidPositionIndex(0, _first_position_index);

	if (first_status != PositionLookupStatus::kFound) {
		_failure_reason = positionLookupFailureReason(first_status, FailureReason::kNoValidWaypoints);
		return false;
	}

	if (!findAttachedValidPositionIndex(_provider.missionCount() - 1, _last_position_index)) {
		_failure_reason = FailureReason::kNoValidWaypoints;
		return false;
	}

	// a segment only exists once a second endpoint (or loop edge) has been processed.
	_index = _first_position_index;

	while (_index < _provider.missionCount()) {
		FailureReason failure_reason = FailureReason::kUnknown;

		if (!prepareNextSegment(_index++, failure_reason)) {
			if (failure_reason != FailureReason::kUnknown) {
				_failure_reason = failure_reason;
				return false;
			}

			continue;
		}

		_segment.start = _segment.end;
		_positions.start = _positions.end;
		// A route that starts with a landing has no segments to walk.
		_done = isLandingCmd(_segment.end.nav_cmd);
		return true;
	}

	// No position items: the walk is empty.
	_done = true;
	return true;
}

bool RouteSegmentCursor::next(RouteSegmentView &view)
{
	if (_done) {
		return false;
	}

	while (_index < _provider.missionCount()) {
		FailureReason failure_reason = FailureReason::kUnknown;

		if (!prepareNextSegment(_index++, failure_reason)) {
			if (failure_reason != FailureReason::kUnknown) {
				_failure_reason = failure_reason;
				_done = true;
				return false;
			}

			continue;
		}

		if (isLandingCmd(_segment.end.nav_cmd)) {
			// The vehicle flies to a landing waypoint at the previous altitude and only
			// descends at the point itself, so the segment keeps the start altitude.
			_positions.end.alt = _positions.start.alt;
		}

		view = {};
		view.segment = _segment;
		view.positions = _positions;
		view.route_along_start_m = _route_along_m;
		view.first_segment = (_segment.start.idx == _first_position_index);
		view.last_segment = (_segment.end.idx == _last_position_index) || isLandingCmd(_segment.end.nav_cmd);
		view.zero_length_xy =
			fabs(_positions.start.lat - _positions.end.lat) <= kCornerLatLonTolDeg
			&& fabs(_positions.start.lon - _positions.end.lon) <= kCornerLatLonTolDeg;

		// Computed once per segment; every batch reference reuses them.
		if (!view.zero_length_xy) {
			view.length_m = get_distance_to_next_waypoint(_positions.start.lat, _positions.start.lon,
					_positions.end.lat, _positions.end.lon);
			get_vector_to_next_waypoint(_positions.start.lat, _positions.start.lon,
						    _positions.end.lat, _positions.end.lon,
						    &view.segment_vector(0), &view.segment_vector(1));
		}

		// Loop edges are detours off the nominal route: the walk continues from the same start.
		if (!_segment.is_loop) {
			_route_along_m += view.length_m;
			_segment.start = _segment.end;
			_positions.start = _positions.end;
			_done = view.last_segment;
		}

		return true;
	}

	return false;
}

bool MissionRouteProjection::localMinimumOnSegment(bool proj_on_start, bool proj_on_end,
		bool prev_proj_on_end, bool jumping, bool last_segment) const
{
	// The full local-minimum rules are documented on the declaration.
	const bool proj_on_corner = proj_on_start || proj_on_end;

	// Loop-edge corners already belong to their nominal route segments.
	if (jumping) {
		return !proj_on_corner;
	}

	// The terminal route endpoint has no following segment to compare against.
	if (last_segment && proj_on_end) {
		return true;
	}

	// Interior projection, or a V-corner shared with the previous segment.
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
	const int buffer_size = min(candidate_buffer.count, kMaxSegmentCandidates);

	// The buffer is sorted by ascending xtrack, break after first outside of limit
	for (int index = buffer_size - 1; index >= 0; --index) {
		if (candidate_buffer.candidates[index].dist.xtrack <= xtrack_limit) {
			candidate_buffer.count = index + 1;
			return;
		}
	}

	candidate_buffer.count = 0;
}

bool isIndexInProjectionSegment(const Segment &projection_segment, int32_t mission_index, bool is_flying_reverse)
{
	if (!projection_segment.valid()) {
		return false;
	}

	// Segments span consecutive position items in nominal direction [start, end].
	// E.g. on segment [2 -> 4] with 3 a non-position item: flying nominal the
	// vehicle targets 3 or 4, flying reverse it targets 2 or 3.
	if (is_flying_reverse) {
		return mission_index >= projection_segment.start.idx && mission_index < projection_segment.end.idx;
	}

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

	// Reject non-finite or out-of-window projections before the more expensive lat/lon reconstruction.
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

FailureReason MissionRouteProjection::findProjectionCandidates(const ProjectionScanRequest &request,
		ProjectionReferenceBatch &batch, RouteDistanceSummary &distance_summary) const
{
	distance_summary = {};

	if (batch.count == 0 || batch.count > kMaxSafePointBatch || !(request.xtrack_margin_m >= 0.f)) {
		return FailureReason::kInvalidRequest;
	}

	for (uint8_t i = 0; i < batch.count; ++i) {
		batch.items[i].candidate_buffer.count = 0;
		batch.items[i].search_state = {};
	}

	CurrentSegmentBoundsTracker bounds_tracker(request);
	RouteSegmentCursor cursor(_provider, request.home_altitude_amsl);

	if (!cursor.init()) {
		return cursor.failureReason();
	}

	ProjectionScanStats stats{};
	RouteSegmentView segment_view{};

	while (cursor.next(segment_view)) {
		stats.segments_processed++;

		if (segment_view.zero_length_xy && !segment_view.last_segment && !segment_view.first_segment) {
			// Interior zero-length stacks cannot contribute a unique XY projection candidate.
			bounds_tracker.observe(segment_view);
			continue;
		}

		for (uint8_t i = 0; i < batch.count; ++i) {
			processCandidateForSegment(batch.items[i].position, segment_view, request.xtrack_margin_m,
						   batch.items[i].search_state, batch.items[i].candidate_buffer,
						   stats);
		}

		if (segment_view.segment.is_loop) {
			// Loop edges are not part of the nominal route.
			continue;
		}

		bounds_tracker.observe(segment_view);

		// Carry the end-corner state for V-corner detection (see localMinimumOnSegment);
		// a zero-length segment cannot form a V apex.
		for (uint8_t i = 0; i < batch.count; ++i) {
			CandidateSearchState &search_state = batch.items[i].search_state;
			search_state.prev_projection_on_end =
				segment_view.zero_length_xy ? false : search_state.projection_on_end_for_segment;
		}
	}

	if (cursor.failed()) {
		return cursor.failureReason();
	}

	const SegmentDistanceAlong current_segment_along = bounds_tracker.bounds();
	const float route_length = cursor.routeLength();

	bool any_candidate_found = false;

	for (uint8_t i = 0; i < batch.count; ++i) {
		if (batch.items[i].candidate_buffer.count > 0) {
			any_candidate_found = true;
			break;
		}
	}

	PX4_DEBUG("Route batch items: %u, segs: %u, mins: %u, valid: %u",
		  static_cast<unsigned>(batch.count),
		  static_cast<unsigned>(stats.segments_processed),
		  static_cast<unsigned>(stats.local_min_found),
		  static_cast<unsigned>(stats.valid_candidate_found));

	if (any_candidate_found) {
		distance_summary.current_segment_along = current_segment_along;
		distance_summary.route_length = route_length;
		return FailureReason::kNone;
	}

	if (stats.segments_processed == 0) {
		return FailureReason::kNoSegmentsFound;

	} else if (stats.local_min_found == 0) {
		return FailureReason::kNoLocalMinFound;

	} else if (stats.valid_candidate_found == 0) {
		return FailureReason::kNoValidCandidateFound;
	}

	return FailureReason::kUnknown;
}

struct MissionRouteProjection::BranchInSelection {
	RouteProjectionCandidate candidate{};
	int candidate_index{-1};
	float score_m{FLT_MAX};
};

FailureReason MissionRouteProjection::selectBranchInCandidate(
	const ProjectionCandidateBuffer &candidate_buffer,
	const SegmentDistanceAlong &current_segment_along,
	int32_t mission_index,
	bool is_flying_reverse,
	const ActiveJumpAnchor &active_jump_anchor,
	BranchInSelection &selection) const
{
	selection = {};
	int best_candidate_index = -1;
	float min_path_distance = FLT_MAX;
	SegmentDistanceAlong segment_along = current_segment_along;

	if (!segment_along.valid()) {
		PX4_ERR("Route select UAV proj: invalid current segment (mission_idx=%d), setting to zero",
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
			PX4_DEBUG("Route UAV proj cand %u skipped, non-finite path distance",
				  static_cast<unsigned>(i));
			continue;
		}

		PX4_DEBUG("Route UAV proj cand %u on seg [%u->%u], path_dist=%.1f, along=%.1f xtrack=%.1f on_seg=%.1f",
			  static_cast<unsigned>(i),
			  static_cast<unsigned>(candidate.segment.start.idx),
			  static_cast<unsigned>(candidate.segment.end.idx),
			  static_cast<double>(candidate_path_distance),
			  static_cast<double>(candidate.dist.route_along),
			  static_cast<double>(candidate.dist.xtrack),
			  static_cast<double>(candidate.dist.segment_along));

		bool priority_match = false;

		if (active_jump_anchor.valid()) {
			priority_match = active_jump_anchor.start_index == candidate.segment.start.idx
					 && active_jump_anchor.target_index == candidate.segment.end.idx;

			if (priority_match) {
				PX4_DEBUG("Route UAV proj prioritizing cand %u (loop segment match)", static_cast<unsigned>(i));
			}

		} else {
			priority_match = isIndexInProjectionSegment(candidate.segment, mission_index, is_flying_reverse);

			if (priority_match) {
				PX4_DEBUG("Route UAV proj prioritizing cand %u (segment match)", static_cast<unsigned>(i));
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
		PX4_ERR("Route UAV proj failed, no valid candidate selected");
		return FailureReason::kNoValidCandidateFound;
	}

	selection.candidate = candidate_buffer.candidates[best_candidate_index];
	selection.candidate_index = best_candidate_index;
	selection.score_m = min_path_distance;
	return FailureReason::kNone;
}

FailureReason MissionRouteProjection::collectVehicleProjection(const Position &vehicle_position,
		int32_t mission_index, const PlannerConfig &config, ProjectionReferenceBatch &batch,
		ProjectionContext &projection_context) const
{
	const Position input_vehicle_position = vehicle_position;
	projection_context = {};

	if (!input_vehicle_position.valid()) {
		return FailureReason::kNoValidGlobalPos;
	}

	if (_provider.missionCount() <= 0) {
		return FailureReason::kNoValidWaypoints;
	}

	if (mission_index < 0 || mission_index >= _provider.missionCount()) {
		PX4_ERR("Route invalid mission index: %d (mission count: %d)",
			static_cast<int>(mission_index), static_cast<int>(_provider.missionCount()));
		return FailureReason::kInvalidRequest;
	}

	batch.items[0] = {};
	batch.count = 1;
	batch.items[0].position = input_vehicle_position;

	ProjectionScanRequest scan_request{};
	scan_request.home_altitude_amsl = config.parameters.home_altitude_amsl;
	scan_request.xtrack_margin_m = config.parameters.vehicle_projection_search_dist;
	scan_request.compute_current_segment_bounds = true;
	scan_request.mission_index = mission_index;
	scan_request.is_flying_reverse = config.state.is_flying_reverse;
	scan_request.active_jump_anchor = config.active_jump_anchor;

	RouteDistanceSummary distance_summary{};
	const FailureReason scan_status = findProjectionCandidates(scan_request, batch, distance_summary);

	if (scan_status != FailureReason::kNone) {
		return scan_status;
	}

	const ProjectionCandidateBuffer &candidate_buffer = batch.items[0].candidate_buffer;

	PX4_DEBUG("Route vehicle projection: cands=%u current_segment[%.1f, %.1f] idx=%d",
		  static_cast<unsigned>(candidate_buffer.count),
		  static_cast<double>(distance_summary.current_segment_along.start),
		  static_cast<double>(distance_summary.current_segment_along.end),
		  static_cast<int>(mission_index));

	BranchInSelection branch_in{};
	const FailureReason branch_in_status = selectBranchInCandidate(candidate_buffer,
					       distance_summary.current_segment_along, mission_index,
					       config.state.is_flying_reverse,
					       config.active_jump_anchor, branch_in);

	if (branch_in_status != FailureReason::kNone) {
		return branch_in_status;
	}

	projection_context.vehicle_position = input_vehicle_position;
	projection_context.mission_index = mission_index;
	projection_context.route_projection = branch_in.candidate;
	projection_context.vehicle_state = config.state;
	projection_context.route_length = distance_summary.route_length;
	// Use the repeat count from the selected projection loop itself. A later DO_JUMP elsewhere
	// in the mission must not overwrite the active loop state carried by this projection.
	projection_context.mission_loops_remaining = projection_context.route_projection.segment.validLoop()
			? projection_context.route_projection.segment.loops_remaining : 0;

	PX4_DEBUG("Route UAV proj selected cand %d (of %u) on seg [%u->%u], path_dist=%.1f",
		  branch_in.candidate_index,
		  static_cast<unsigned>(candidate_buffer.count),
		  static_cast<unsigned>(projection_context.route_projection.segment.start.idx),
		  static_cast<unsigned>(projection_context.route_projection.segment.end.idx),
		  static_cast<double>(branch_in.score_m));

	if (projection_context.route_projection.segment.validLoop()) {
		projection_context.loop_context = buildLoopContext(projection_context.route_projection,
						  config.parameters.home_altitude_amsl);
	}

	if (!projection_context.valid()) {
		projection_context = {};
		return FailureReason::kInvalidProjectionContext;
	}

	return FailureReason::kNone;
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

	PX4_DEBUG("Route loop ctx: seg[%u-%u], along[%.1f, %.1f], loops remaining: %u",
		  static_cast<unsigned>(loop_context.segment.start.idx),
		  static_cast<unsigned>(loop_context.segment.end.idx),
		  static_cast<double>(loop_context.along.start),
		  static_cast<double>(loop_context.along.end),
		  static_cast<unsigned>(loop_context.segment.loops_remaining));

	return loop_context;
}

} // namespace mission_route
