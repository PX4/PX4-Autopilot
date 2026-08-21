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

enum class PositionLookupStatus : uint8_t {
	kFound,
	kNoPositionFound,
	kLoadFailed,
	kInvalidPosition
};

struct LoadedPosition {
	SegmentEndpoint endpoint{};
	Position position{};
};

enum class CursorItemType : uint8_t {
	kEnd,
	kPosition,
	kJump
};

struct CursorItem {
	CursorItemType type{CursorItemType::kEnd};
	int32_t index{-1};
	mission_item_s mission_item{};
	LoadedPosition loaded_position{};
};

/** @brief Raw local projection of a reference point onto one route segment. */
struct RawSegmentProjection {
	bool valid{false};

	float xtrack_m{NAN};
	float along_segment_m{NAN};
	// Normalized position of the projection along the segment, clamped to [0, 1] (0 = start, 1 = end).
	float along_fraction{NAN};

	bool projection_on_start{false};
	bool projection_on_end{false};

	matrix::Vector2f projection_vector{0.f, 0.f};
};

RawSegmentProjection projectReferenceToSegment(const Position &reference_position,
		const SegmentPositions &segment_positions,
		const matrix::Vector2f &segment_vector,
		float segment_length_m,
		bool segment_has_no_length)
{
	RawSegmentProjection projection{};

	if (segment_has_no_length) {
		// If the segment is a point, the projection is the point itself.
		projection.xtrack_m = get_distance_to_next_waypoint(reference_position.lat, reference_position.lon,
				      segment_positions.end.lat, segment_positions.end.lon);
		projection.along_segment_m = 0.f;
		projection.along_fraction = 0.f;
		projection.projection_on_start = true;
		projection.projection_on_end = true;
		projection.valid = PX4_ISFINITE(projection.xtrack_m) && projection.xtrack_m >= 0.f;
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
	projection.projection_on_start = (t * segment_length_m) < kCornerToleranceM;
	projection.projection_on_end = ((1.f - t) * segment_length_m) < kCornerToleranceM;

	projection.along_fraction = constrain(t, 0.f, 1.f);
	projection.projection_vector = segment_vector * projection.along_fraction;
	projection.along_segment_m = projection.along_fraction * segment_length_m;
	projection.xtrack_m = static_cast<matrix::Vector2f>(reference_vector - projection.projection_vector).norm();
	projection.valid = PX4_ISFINITE(projection.xtrack_m) && projection.xtrack_m >= 0.f;

	return projection;
}

bool buildProjectionCandidate(const Segment &segment,
			      const SegmentPositions &segment_positions,
			      const RawSegmentProjection &projection,
			      float route_along_m,
			      float segment_length_m,
			      bool segment_has_no_length,
			      RouteProjectionCandidate &candidate)
{
	candidate.segment = segment;
	candidate.segment_positions = segment_positions;
	candidate.dist.xtrack_m = projection.xtrack_m;
	candidate.dist.route_along_m = route_along_m + projection.along_segment_m;
	candidate.dist.segment_length_m = segment_length_m;
	candidate.dist.along_segment_m = min(projection.along_segment_m, segment_length_m);

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
	candidate.projection.lon = matrix::wrap(lon_res, -180.0, 180.0);
	candidate.projection.alt = segment_positions.start.alt
				   + projection.along_fraction * (segment_positions.end.alt - segment_positions.start.alt);

	return candidate.valid();
}

bool extractIndexedPosition(int32_t index, const mission_item_s &mission_item, float home_altitude_amsl,
			    LoadedPosition &loaded_position)
{
	loaded_position = {};
	loaded_position.endpoint.idx = index;
	loaded_position.endpoint.nav_cmd = mission_item.nav_cmd;
	return extractMissionPosition(mission_item, home_altitude_amsl, loaded_position.position);
}

FailureReason scanNextCursorItem(const Provider &provider, float home_altitude_amsl,
				 int32_t &next_index, CursorItem &item)
{
	item = {};

	while (next_index < provider.missionCount()) {
		item.index = next_index++;

		if (!provider.loadMissionItem(item.index, item.mission_item)) {
			return FailureReason::kLoadFailed;
		}

		if (mission_item_contains_position(item.mission_item)) {
			if (!extractIndexedPosition(item.index, item.mission_item, home_altitude_amsl, item.loaded_position)) {
				return FailureReason::kPositionItemInvalid;
			}

			item.type = CursorItemType::kPosition;
			return FailureReason::kNone;
		}

		if (item.mission_item.nav_cmd == NAV_CMD_DO_JUMP) {
			item.type = CursorItemType::kJump;
			return FailureReason::kNone;
		}
	}

	return FailureReason::kNone;
}

PositionLookupStatus findNextMissionPosition(const Provider &provider, float home_altitude_amsl,
		int32_t start_index, LoadedPosition &loaded_position)
{
	loaded_position = {};

	if (start_index < 0) {
		return PositionLookupStatus::kNoPositionFound;
	}

	for (int32_t index = start_index; index < provider.missionCount(); ++index) {
		mission_item_s mission_item{};

		if (!provider.loadMissionItem(index, mission_item)) {
			return PositionLookupStatus::kLoadFailed;
		}

		if (!mission_item_contains_position(mission_item)) {
			continue;
		}

		if (!extractIndexedPosition(index, mission_item, home_altitude_amsl, loaded_position)) {
			return PositionLookupStatus::kInvalidPosition;
		}

		return PositionLookupStatus::kFound;
	}

	return PositionLookupStatus::kNoPositionFound;
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

struct ActiveJumpContext {
	int32_t jump_item_index{-1};
	int32_t target_index{-1};

	bool valid() const { return jump_item_index >= 0 && target_index >= 0; }
};

FailureReason resolveActiveJump(const Provider &provider, float home_altitude_amsl,
				const ActiveJumpAnchor &anchor, ActiveJumpContext &context)
{
	context = {};

	if (anchor.empty()) {
		return FailureReason::kNone;
	}

	if (!anchor.valid() || anchor.jump_item_index >= provider.missionCount()) {
		return FailureReason::kInvalidRequest;
	}

	mission_item_s jump_item{};

	if (!provider.loadMissionItem(anchor.jump_item_index, jump_item)) {
		return FailureReason::kLoadFailed;
	}

	if (jump_item.nav_cmd != NAV_CMD_DO_JUMP) {
		return FailureReason::kInvalidRequest;
	}

	if (jump_item.do_jump_mission_index < 0) {
		return FailureReason::kInternalError;
	}

	LoadedPosition target{};
	const PositionLookupStatus target_status = findNextMissionPosition(provider, home_altitude_amsl,
			jump_item.do_jump_mission_index, target);

	if (target_status != PositionLookupStatus::kFound) {
		return positionLookupFailureReason(target_status, FailureReason::kInternalError);
	}

	context.jump_item_index = anchor.jump_item_index;
	context.target_index = target.endpoint.idx;
	return FailureReason::kNone;
}

/**
 * @brief Locates the along-route interval of the segment the vehicle is currently flying.
 *
 * Branch-in selection prefers the candidate closest along the route to where the vehicle
 * already is (mission continuity over raw distance), and that position is only known once
 * the scan has walked past the vehicle's segment. Feed every emitted segment to observe().
 */
class CurrentSegmentBoundsTracker
{
public:
	CurrentSegmentBoundsTracker(const ProjectionScanRequest &request, const ActiveJumpContext &active_jump) :
		_request(request), _active_jump(active_jump)
	{
		_wanted = request.compute_current_segment_bounds;
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

		if (_active_jump.valid()) {
			if (segment.isLoop() && segment.jump_item_index == _active_jump.jump_item_index) {
				_bounds.route_start_dist_m = segment_view.route_along_start_m;

			} else if (!segment.isLoop() && segment.start.idx == _active_jump.target_index) {
				_bounds.route_end_dist_m = segment_view.route_along_start_m;

			} else if (!segment.isLoop() && segment.end.idx == _active_jump.target_index) {
				_bounds.route_end_dist_m = segment_view.route_along_start_m + segment_view.length_m;
			}

			return _bounds.valid();
		}

		// At or before the first position item (flying nominal) the vehicle has not entered the route yet.
		if (segment_view.first_segment && !_request.is_flying_reverse
		    && _request.mission_index <= segment.start.idx) {
			_bounds.route_start_dist_m = 0.f;
			_bounds.route_end_dist_m = 0.f;
			PX4_DEBUG("Route current_segment_along zero for pre-route mission index %d",
				  static_cast<int>(_request.mission_index));
			return true;
		}

		if (!segment.isLoop()
		    && isIndexInProjectionSegment(segment, _request.mission_index, _request.is_flying_reverse)) {
			_bounds.route_start_dist_m = segment_view.route_along_start_m;
			_bounds.route_end_dist_m = segment_view.route_along_start_m + segment_view.length_m;
			PX4_DEBUG("Route current_segment_along: [%.3f, %.3f]",
				  static_cast<double>(_bounds.route_start_dist_m),
				  static_cast<double>(_bounds.route_end_dist_m));
			return true;
		}

		return false;
	}

	const ProjectionScanRequest &_request;
	const ActiveJumpContext &_active_jump;
	SegmentDistanceAlong _bounds{};
	bool _wanted{false};
	bool _located{false};
};

} // namespace

ProjectionScanRequest makeVehicleScanRequest(const PlannerConfig &config, int32_t mission_index)
{
	ProjectionScanRequest request{};
	request.home_altitude_amsl = config.parameters.home_altitude_amsl;
	request.xtrack_margin_m = config.parameters.vehicle_projection_search_dist_m;
	request.compute_current_segment_bounds = true;
	request.mission_index = mission_index;
	request.is_flying_reverse = config.state.is_flying_reverse;
	request.active_jump_anchor = config.active_jump_anchor;
	return request;
}

ProjectionScanRequest makeSafePointScanRequest(const PlannerConfig &config)
{
	ProjectionScanRequest request{};
	request.home_altitude_amsl = config.parameters.home_altitude_amsl;
	request.xtrack_margin_m = config.parameters.safe_point_projection_search_dist_m;
	request.compute_current_segment_bounds = false;
	return request;
}

bool RouteSegmentCursor::fail(FailureReason failure_reason)
{
	_failure_reason = failure_reason;
	_done = true;
	return false;
}

void RouteSegmentCursor::prepareSegmentView(SegmentBoundary boundary, RouteSegmentView &view)
{
	if (isLandingCmd(_segment.end.nav_cmd)) {
		// Landing descent starts after reaching the waypoint.
		_positions.end.alt = _positions.start.alt;
	}

	view = {};
	view.segment = _segment;
	view.positions = _positions;
	view.route_along_start_m = _route_along_m;
	view.first_segment = (_segment.start.idx == _first_position_index);
	view.last_segment = (boundary == SegmentBoundary::kFinal);
	view.zero_length_xy =
		fabs(_positions.start.lat - _positions.end.lat) <= kCornerLatLonTolDeg
		&& fabs(_positions.start.lon - _positions.end.lon) <= kCornerLatLonTolDeg;

	if (!view.zero_length_xy) {
		view.length_m = get_distance_to_next_waypoint(_positions.start.lat, _positions.start.lon,
				_positions.end.lat, _positions.end.lon);
		get_vector_to_next_waypoint(_positions.start.lat, _positions.start.lon,
					    _positions.end.lat, _positions.end.lon,
					    &view.segment_vector(0), &view.segment_vector(1));
	}
}

void RouteSegmentCursor::prepareNominalSegment(const SegmentEndpoint &end, const Position &end_position,
		SegmentBoundary boundary, RouteSegmentView &view)
{
	_segment.end = end;
	_segment.jump_item_index = -1;
	_segment.has_remaining_repeats = false;
	_positions.end = end_position;
	prepareSegmentView(boundary, view);

	_route_along_m += view.length_m;
	_segment.start = _segment.end;
	_positions.start = _positions.end;
}

bool RouteSegmentCursor::prepareJumpSegment(int32_t index, const mission_item_s &jump_item,
		RouteSegmentView &view)
{
	if (jump_item.do_jump_mission_index < 0) {
		PX4_ERR("Route invalid DO_JUMP target index %d", static_cast<int>(jump_item.do_jump_mission_index));
		return fail(FailureReason::kInternalError);
	}

	LoadedPosition target{};
	const PositionLookupStatus target_status = findNextMissionPosition(_provider, _home_altitude_amsl,
			jump_item.do_jump_mission_index, target);

	if (target_status != PositionLookupStatus::kFound) {
		return fail(positionLookupFailureReason(target_status, FailureReason::kInternalError));
	}

	_segment.end = target.endpoint;
	_segment.jump_item_index = index;
	_segment.has_remaining_repeats = jump_item.do_jump_current_count < jump_item.do_jump_repeat_count;
	_positions.end = target.position;
	prepareSegmentView(SegmentBoundary::kIntermediate, view);
	return true;
}

bool RouteSegmentCursor::init()
{
	LoadedPosition first{};
	const PositionLookupStatus first_status = findNextMissionPosition(_provider, _home_altitude_amsl, 0, first);

	if (first_status != PositionLookupStatus::kFound) {
		return fail(positionLookupFailureReason(first_status, FailureReason::kNoValidWaypoints));
	}

	_segment.start = first.endpoint;
	_positions.start = first.position;
	_first_position_index = first.endpoint.idx;
	_index = first.endpoint.idx + 1;
	_done = isLandingCmd(first.endpoint.nav_cmd);
	return true;
}

bool RouteSegmentCursor::next(RouteSegmentView &view)
{
	if (_done) {
		return false;
	}

	LoadedPosition segment_end{};
	CursorItem item{};

	if (_next_position_cached) {
		segment_end.endpoint = _next_endpoint;
		segment_end.position = _next_position;
		_next_position_cached = false;

	} else {
		const FailureReason scan_failure = scanNextCursorItem(_provider, _home_altitude_amsl, _index, item);

		if (scan_failure != FailureReason::kNone) {
			return fail(scan_failure);
		}

		switch (item.type) {
		case CursorItemType::kEnd:
			_done = true;
			return false;

		case CursorItemType::kPosition:
			segment_end = item.loaded_position;
			break;

		case CursorItemType::kJump:
			return prepareJumpSegment(item.index, item.mission_item, view);
		}
	}

	if (isLandingCmd(segment_end.endpoint.nav_cmd)) {
		prepareNominalSegment(segment_end.endpoint, segment_end.position, SegmentBoundary::kFinal, view);
		_done = true;
		return true;
	}

	// Look ahead so the route endpoint can be marked as a local minimum.
	SegmentBoundary boundary{SegmentBoundary::kFinal};
	const FailureReason scan_failure = scanNextCursorItem(_provider, _home_altitude_amsl, _index, item);

	if (scan_failure != FailureReason::kNone) {
		return fail(scan_failure);
	}

	switch (item.type) {
	case CursorItemType::kEnd:
		break;

	case CursorItemType::kPosition:
		_next_endpoint = item.loaded_position.endpoint;
		_next_position = item.loaded_position.position;
		_next_position_cached = true;
		boundary = SegmentBoundary::kIntermediate;
		break;

	case CursorItemType::kJump:
		// Process the jump on the next call, after the nominal segment.
		_index = item.index;
		boundary = SegmentBoundary::kIntermediate;
		break;
	}

	prepareNominalSegment(segment_end.endpoint, segment_end.position, boundary, view);
	_done = boundary == SegmentBoundary::kFinal;
	return true;
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
	    || candidate.segment.end.idx >= _provider.missionCount()
	    || (candidate.segment.isLoop() && candidate.segment.jump_item_index >= _provider.missionCount())) {
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
	       && candidate_buffer.candidates[insert_index].dist.xtrack_m <= candidate.dist.xtrack_m) {
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
		float xtrack_limit_m) const
{
	const int buffer_size = min(candidate_buffer.count, kMaxSegmentCandidates);

	// The buffer is sorted by ascending xtrack, break after first outside of limit
	for (int index = buffer_size - 1; index >= 0; --index) {
		if (candidate_buffer.candidates[index].dist.xtrack_m <= xtrack_limit_m) {
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
				   state.prev_projection_on_end, segment.segment.isLoop(), segment.last_segment)) {
		return;
	}

	stats.local_min_found++;

	// Reject non-finite or out-of-window projections before the more expensive lat/lon reconstruction.
	if (!projection.valid || projection.xtrack_m >= state.xtrack_limit_m) {
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

	if (projection.xtrack_m < state.min_xtrack_m) {
		// A new closest projection tightens the search window, so prune stale candidates first.
		state.min_xtrack_m = projection.xtrack_m;
		state.xtrack_limit_m = state.min_xtrack_m + xtrack_margin_m;
		pruneProjectionCandidates(candidate_buffer, state.xtrack_limit_m);
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

	ActiveJumpContext active_jump{};
	const FailureReason active_jump_status = resolveActiveJump(_provider, request.home_altitude_amsl,
			request.active_jump_anchor, active_jump);

	if (active_jump_status != FailureReason::kNone) {
		return active_jump_status;
	}

	CurrentSegmentBoundsTracker bounds_tracker(request, active_jump);
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

		bounds_tracker.observe(segment_view);

		if (segment_view.segment.isLoop()) {
			// Loop edges are not part of the nominal route.
			continue;
		}

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
	const int32_t route_end_index = cursor.nominalPositionIndex();

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
		distance_summary.route_end_index = route_end_index;
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
	float min_path_distance_m = FLT_MAX;
	SegmentDistanceAlong segment_bounds = current_segment_along;

	if (!segment_bounds.valid()) {
		PX4_WARN("Route select UAV proj: invalid current segment (mission_idx=%d), setting to zero",
			 static_cast<int>(mission_index));
		segment_bounds.route_start_dist_m = 0.f;
		segment_bounds.route_end_dist_m = 0.f;
	}

	for (uint8_t i = 0; i < candidate_buffer.count; ++i) {
		const RouteProjectionCandidate &candidate = candidate_buffer.candidates[i];
		const float dist_to_start_m = fabsf(candidate.dist.route_along_m - segment_bounds.route_start_dist_m);
		const float dist_to_end_m = fabsf(candidate.dist.route_along_m - segment_bounds.route_end_dist_m);
		const float projection_to_segment_dist_m = fminf(dist_to_start_m, dist_to_end_m);
		const float candidate_path_distance_m = candidate.dist.xtrack_m + projection_to_segment_dist_m;

		if (!PX4_ISFINITE(candidate_path_distance_m)) {
			PX4_DEBUG("Route UAV proj cand %u skipped, non-finite path distance",
				  static_cast<unsigned>(i));
			continue;
		}

		PX4_DEBUG("Route UAV proj cand %u on seg [%u->%u], path_dist=%.1f, along=%.1f xtrack=%.1f on_seg=%.1f",
			  static_cast<unsigned>(i),
			  static_cast<unsigned>(candidate.segment.start.idx),
			  static_cast<unsigned>(candidate.segment.end.idx),
			  static_cast<double>(candidate_path_distance_m),
			  static_cast<double>(candidate.dist.route_along_m),
			  static_cast<double>(candidate.dist.xtrack_m),
			  static_cast<double>(candidate.dist.along_segment_m));

		bool priority_match = false;

		if (active_jump_anchor.valid()) {
			priority_match = candidate.segment.validLoop()
					 && active_jump_anchor.jump_item_index == candidate.segment.jump_item_index;

			if (priority_match) {
				PX4_DEBUG("Route UAV proj prioritizing cand %u (loop segment match)", static_cast<unsigned>(i));
			}

		} else {
			priority_match = isIndexInProjectionSegment(candidate.segment, mission_index, is_flying_reverse);

			if (priority_match) {
				PX4_DEBUG("Route UAV proj prioritizing cand %u (segment match)", static_cast<unsigned>(i));
			}
		}

		const bool is_shorter_path = candidate_path_distance_m < min_path_distance_m;

		if (priority_match || is_shorter_path) {
			min_path_distance_m = candidate_path_distance_m;
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
	selection.score_m = min_path_distance_m;
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

	const ProjectionScanRequest scan_request = makeVehicleScanRequest(config, mission_index);

	RouteDistanceSummary distance_summary{};
	const FailureReason scan_status = findProjectionCandidates(scan_request, batch, distance_summary);

	if (scan_status != FailureReason::kNone) {
		return scan_status;
	}

	const ProjectionCandidateBuffer &candidate_buffer = batch.items[0].candidate_buffer;

	PX4_DEBUG("Route vehicle projection: cands=%u current_segment[%.1f, %.1f] idx=%d",
		  static_cast<unsigned>(candidate_buffer.count),
		  static_cast<double>(distance_summary.current_segment_along.route_start_dist_m),
		  static_cast<double>(distance_summary.current_segment_along.route_end_dist_m),
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
	projection_context.route_end_index = distance_summary.route_end_index;

	PX4_DEBUG("Route UAV proj selected cand %d (of %u) on seg [%u->%u], path_dist=%.1f",
		  branch_in.candidate_index,
		  static_cast<unsigned>(candidate_buffer.count),
		  static_cast<unsigned>(projection_context.route_projection.segment.start.idx),
		  static_cast<unsigned>(projection_context.route_projection.segment.end.idx),
		  static_cast<double>(branch_in.score_m));

	if (projection_context.route_projection.segment.validLoop()) {
		const FailureReason loop_status = buildLoopContext(projection_context.route_projection,
						  config.parameters.home_altitude_amsl,
						  projection_context.loop_context);

		if (loop_status != FailureReason::kNone) {
			projection_context = {};
			return loop_status;
		}
	}

	if (!projection_context.valid()) {
		projection_context = {};
		return FailureReason::kInvalidProjectionContext;
	}

	return FailureReason::kNone;
}

FailureReason MissionRouteProjection::nominalRouteDistanceToItem(int32_t target_index,
		float home_altitude_amsl, float &distance_m) const
{
	distance_m = NAN;

	if (target_index < 0 || target_index >= _provider.missionCount()) {
		return FailureReason::kInvalidRequest;
	}

	RouteSegmentCursor cursor(_provider, home_altitude_amsl);

	if (!cursor.init()) {
		return cursor.failureReason();
	}

	RouteSegmentView segment_view{};

	while (cursor.nominalPositionIndex() != target_index) {
		if (!cursor.next(segment_view)) {
			break;
		}
	}

	if (cursor.failed()) {
		return cursor.failureReason();
	}

	if (cursor.nominalPositionIndex() != target_index) {
		return FailureReason::kNoValidPath;
	}

	distance_m = cursor.routeLength();
	return FailureReason::kNone;
}

FailureReason MissionRouteProjection::buildLoopContext(const RouteProjectionCandidate &vehicle_projection,
		float home_altitude_amsl, LoopContext &loop_context) const
{
	loop_context = {};

	if (!vehicle_projection.segment.validLoop()) {
		return FailureReason::kInvalidProjectionContext;
	}

	float loop_end_route_along_m{NAN};
	const FailureReason distance_status = nominalRouteDistanceToItem(vehicle_projection.segment.end.idx,
					      home_altitude_amsl, loop_end_route_along_m);

	if (distance_status != FailureReason::kNone) {
		return distance_status;
	}

	loop_context.segment = vehicle_projection.segment;
	loop_context.segment_positions = vehicle_projection.segment_positions;
	loop_context.along.route_start_dist_m = vehicle_projection.dist.route_along_m
						- vehicle_projection.dist.along_segment_m;
	loop_context.along.route_end_dist_m = loop_end_route_along_m;

	if (!loop_context.valid()) {
		loop_context = {};
		return FailureReason::kInvalidProjectionContext;
	}

	PX4_DEBUG("Route loop ctx: seg[%u-%u], along[%.1f, %.1f], repeat pending: %u",
		  static_cast<unsigned>(loop_context.segment.start.idx),
		  static_cast<unsigned>(loop_context.segment.end.idx),
		  static_cast<double>(loop_context.along.route_start_dist_m),
		  static_cast<double>(loop_context.along.route_end_dist_m),
		  static_cast<unsigned>(loop_context.segment.has_remaining_repeats));

	return FailureReason::kNone;
}

} // namespace mission_route
