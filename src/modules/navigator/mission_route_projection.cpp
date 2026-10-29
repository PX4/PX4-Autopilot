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
#include "mission_route_path_checker.h"

#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

#include <px4_platform_common/log.h>

using namespace math;

namespace mission_route
{

namespace
{

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

// Within the corner tolerance: the segment between the two points is vertical.
bool sameLatLon(const Position &a, const Position &b)
{
	return fabs(a.lat - b.lat) <= kCornerLatLonTolDeg && fabs(a.lon - b.lon) <= kCornerLatLonTolDeg;
}

// Cross-track lower bound without projection trig: the distance to the segment start, at least the latitude
// difference and the distance to the start's meridian plane, less the segment length.
float crossTrackLowerBound(const Position &reference_position, float cos_lat, const RouteSegmentView &segment_view)
{
	// A zero-length segment measures the distance to its end.
	const Position &start = segment_view.zero_length_xy ? segment_view.positions.end : segment_view.positions.start;
	const double d_lat = fabs(math::radians(reference_position.lat - start.lat));
	const double d_lon = fabs(math::radians(reference_position.lon - start.lon));
	double angle = d_lat;

	if (d_lon < 1.0) {
		// sin x >= x - x^3 / 6
		angle = fmax(angle, static_cast<double>(cos_lat) * d_lon * (1.0 - d_lon * d_lon / 6.0));
	}

	return static_cast<float>(angle * CONSTANTS_RADIUS_OF_EARTH) - segment_view.length_m;
}

RawSegmentProjection projectReferenceToSegment(const Position &reference_position,
		const RouteSegmentView &segment_view)
{
	const SegmentPositions &segment_positions = segment_view.positions;
	const matrix::Vector2f &segment_vector = segment_view.segment_vector;
	const float segment_length_m = segment_view.length_m;
	RawSegmentProjection projection{};

	if (segment_view.zero_length_xy) {
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

	segment_view.map_projection.project(reference_position.lat, reference_position.lon,
					    reference_vector(0), reference_vector(1));

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

bool buildProjectionCandidate(const RouteSegmentView &segment_view,
			      const RawSegmentProjection &projection,
			      RouteProjectionCandidate &candidate)
{
	const SegmentPositions &segment_positions = segment_view.positions;
	candidate.segment = segment_view.segment;
	candidate.segment_positions = segment_positions;
	candidate.dist.xtrack_m = projection.xtrack_m;
	candidate.dist.along_route_m = segment_view.start_dist_along_route_m + projection.along_segment_m;
	candidate.dist.segment_length_m = segment_view.length_m;
	candidate.dist.along_segment_m = min(projection.along_segment_m, segment_view.length_m);

	if (segment_view.zero_length_xy) {
		candidate.projection = segment_positions.end;
		return candidate.valid();
	}

	// Invert the same map projection used for the segment and reference vectors.
	double lat_res;
	double lon_res;
	segment_view.map_projection.reproject(projection.projection_vector(0), projection.projection_vector(1),
					      lat_res, lon_res);
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

// Finds the first position item at or after start_index; not_found_reason is returned when there is none.
FailureReason findNextMissionPosition(const Provider &provider, float home_altitude_amsl, int32_t start_index,
				      LoadedPosition &loaded_position, FailureReason not_found_reason)
{
	loaded_position = {};

	if (start_index < 0) {
		return not_found_reason;
	}

	for (int32_t index = start_index; index < provider.missionCount(); ++index) {
		mission_item_s mission_item{};

		if (!provider.loadMissionItem(index, mission_item)) {
			return FailureReason::kLoadFailed;
		}

		if (!mission_item_contains_position(mission_item)) {
			continue;
		}

		if (!extractIndexedPosition(index, mission_item, home_altitude_amsl, loaded_position)) {
			return FailureReason::kPositionItemInvalid;
		}

		return FailureReason::kNone;
	}

	return not_found_reason;
}

struct ActiveJumpContext {
	int32_t jump_item_index{-1};
	int32_t target_index{-1};

	bool valid() const { return jump_item_index >= 0 && target_index >= 0; }
};

// The anchor only names the DO_JUMP item. Locating the vehicle's current segment on a loop also
// needs the item the jump lands on, so resolve and validate both once before walking the route.
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
	const FailureReason target_status = findNextMissionPosition(provider, home_altitude_amsl,
					    jump_item.do_jump_mission_index, target, FailureReason::kInternalError);

	if (target_status != FailureReason::kNone) {
		return target_status;
	}

	context.jump_item_index = anchor.jump_item_index;
	context.target_index = target.endpoint.idx;
	return FailureReason::kNone;
}

// Finds the along-route interval of the segment the vehicle flies, which branch-in selection
// prefers for continuity. Segments arrive in route order; returns true once it is found.
bool locateCurrentSegment(const RouteSegmentView &segment_view, const ProjectionScanRequest &request,
			  const ActiveJumpContext &active_jump, SegmentDistanceAlong &bounds)
{
	const Segment &segment = segment_view.segment;

	if (active_jump.valid()) {
		if (segment.isLoop() && segment.jump_item_index == active_jump.jump_item_index) {
			bounds.start_dist_along_route_m = segment_view.start_dist_along_route_m;

		} else if (!segment.isLoop() && segment.start.idx == active_jump.target_index) {
			bounds.end_dist_along_route_m = segment_view.start_dist_along_route_m;

		} else if (!segment.isLoop() && segment.end.idx == active_jump.target_index) {
			bounds.end_dist_along_route_m = segment_view.start_dist_along_route_m + segment_view.length_m;
		}

		return bounds.valid();
	}

	if (segment.isLoop()) {
		return false;
	}

	// Before the first position item, nominal flight has not entered the route yet.
	if (segment_view.first_segment && !request.is_flying_reverse && request.mission_index <= segment.start.idx) {
		bounds.start_dist_along_route_m = 0.f;
		bounds.end_dist_along_route_m = 0.f;
		return true;
	}

	// A trailing command in nominal flight retains the last flown segment, so continuity
	// is measured from its nearer endpoint, just as for an index within the segment.
	const bool after_last_position = segment_view.last_segment && !request.is_flying_reverse
					 && request.mission_index > segment.end.idx;

	if (after_last_position || isIndexInProjectionSegment(segment, request.mission_index, request.is_flying_reverse)) {
		bounds.start_dist_along_route_m = segment_view.start_dist_along_route_m;
		bounds.end_dist_along_route_m = segment_view.start_dist_along_route_m + segment_view.length_m;
		return true;
	}

	return false;
}

} // namespace

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
	view.start_dist_along_route_m = _dist_along_route_m;
	view.first_segment = (_segment.start.idx == _first_position_index);
	view.last_segment = (boundary == SegmentBoundary::kFinal);
	view.zero_length_xy = sameLatLon(_positions.start, _positions.end);

	if (!view.zero_length_xy) {
		view.map_projection.initReference(_positions.start.lat, _positions.start.lon);
		view.map_projection.project(_positions.end.lat, _positions.end.lon,
					    view.segment_vector(0), view.segment_vector(1));
		view.length_m = view.segment_vector.norm();
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

	_dist_along_route_m += view.length_m;
	_segment.start = _segment.end;
	_positions.start = _positions.end;
}

bool RouteSegmentCursor::prepareJumpSegment(int32_t index, const mission_item_s &jump_item,
		RouteSegmentView &view)
{
	if (jump_item.do_jump_mission_index < 0) {
		PX4_ERR("Invalid DO_JUMP target %d", static_cast<int>(jump_item.do_jump_mission_index));
		return fail(FailureReason::kInternalError);
	}

	LoadedPosition target{};
	const FailureReason target_status = findNextMissionPosition(_provider, _home_altitude_amsl,
					    jump_item.do_jump_mission_index, target, FailureReason::kInternalError);

	if (target_status != FailureReason::kNone) {
		return fail(target_status);
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
	const FailureReason first_status = findNextMissionPosition(_provider, _home_altitude_amsl, 0, first,
					   FailureReason::kNoValidWaypoints);

	if (first_status != FailureReason::kNone) {
		return fail(first_status);
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
	bool next_zero_length_xy = false;
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
		next_zero_length_xy = sameLatLon(segment_end.position, _next_position);
		break;

	case CursorItemType::kJump: {
			// Process the jump on the next call, after the nominal segment.
			_index = item.index;

			// Jump corners belong to nominal segments, so only a later nominal position
			// makes this endpoint intermediate. Terminal jumps must keep this endpoint final.
			LoadedPosition next_position{};
			const FailureReason next_position_status = findNextMissionPosition(_provider, _home_altitude_amsl,
					item.index + 1, next_position, FailureReason::kNone);

			if (next_position_status != FailureReason::kNone) {
				return fail(next_position_status);
			}

			if (next_position.endpoint.idx >= 0) {
				boundary = SegmentBoundary::kIntermediate;
				next_zero_length_xy = sameLatLon(segment_end.position, next_position.position);
			}

			break;
		}
	}

	prepareNominalSegment(segment_end.endpoint, segment_end.position, boundary, view);
	view.next_zero_length_xy = next_zero_length_xy;
	_done = item.type == CursorItemType::kEnd;
	return true;
}

// A projection is kept only at a local minimum of the distance to the route: an interior point,
// a V-corner shared with the previous leg, or the route end. See mission_route_planning.md.
static bool localMinimumOnSegment(bool proj_on_start, bool proj_on_end, bool prev_proj_on_end, bool jumping,
				  bool last_segment)
{
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

static void insertCandidateSorted(ProjectionCandidateBuffer &candidate_buffer,
				  const RouteProjectionCandidate &candidate)
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

static void pruneProjectionCandidates(ProjectionCandidateBuffer &candidate_buffer, float xtrack_limit_m)
{
	const int buffer_size = min(candidate_buffer.count, kMaxSegmentCandidates);

	// Drop trailing candidates outside the cross-track window.
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
	// Mission-index ranges identify nominal segments. Jump edges are identified by their active anchor.
	if (!projection_segment.valid() || projection_segment.isLoop()) {
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

// The segment the vehicle flies: the one holding mission_index, or while the initial TAKEOFF is
// current, the vertical segment above it (nothing precedes that TAKEOFF).
static bool isVehicleSegment(const Segment &segment, float along_route_m, float segment_length_m,
			     int32_t mission_index, bool is_flying_reverse)
{
	const bool initial_takeoff_stack = !is_flying_reverse && !segment.isLoop() && isTakeoffCmd(segment.start.nav_cmd)
					   && mission_index == segment.start.idx && along_route_m <= 0.f && segment_length_m <= 0.f;
	return initial_takeoff_stack || isIndexInProjectionSegment(segment, mission_index, is_flying_reverse);
}

// Infer the required mode from mission commands, independently of the vehicle mode at upload.
uint8_t vtolStateForSegment(const Provider &provider, const Segment &segment)
{
	const int mission_count = provider.missionCount();

	if (!segment.valid() || mission_count <= 0
	    || segment.start.idx >= mission_count || segment.end.idx >= mission_count
	    || segment.jump_item_index >= mission_count) {
		return vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED;
	}

	uint8_t state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED;

	bool lookback_complete = true;

	// The last transition in an executed command range determines the state, if there is one.
	auto resolve_state_in_range = [&](int32_t first_index, int32_t end_index) {
		for (int32_t i = end_index - 1; i >= first_index; --i) {
			mission_item_s mission_item{};

			if (!provider.loadMissionItem(i, mission_item)) {
				PX4_WARN("VTOL state: item %d read failed", static_cast<int>(i));
				lookback_complete = false;
				continue;
			}

			if (updateVtolStateFromMissionItem(mission_item, state)) {
				return true;
			}
		}

		return false;
	};

	if (segment.isLoop()) {
		mission_item_s jump_item{};

		if (!provider.loadMissionItem(segment.jump_item_index, jump_item)) {
			PX4_WARN("VTOL state: jump item %d read failed", static_cast<int>(segment.jump_item_index));
			return state;
		}

		if (jump_item.nav_cmd != NAV_CMD_DO_JUMP || jump_item.do_jump_mission_index < 0
		    || jump_item.do_jump_mission_index > segment.end.idx) {
			return state;
		}

		// The jump may target commands before its resolved waypoint. Those commands run last,
		// so their transitions override the state reached before DO_JUMP.
		if (resolve_state_in_range(jump_item.do_jump_mission_index, segment.end.idx)) {
			return state;
		}
	}

	// Include every command between the source waypoint and DO_JUMP when there was no target transition.
	if (resolve_state_in_range(0, segment.isLoop() ? segment.jump_item_index : segment.end.idx)
	    || !lookback_complete) {
		return state;
	}

	// Only an undefined prefix uses lookahead, in nominal order even when RTL flies in reverse.
	// Ignore DO_JUMP repeats and targets, scanning as if all repeats were consumed.
	for (int32_t index = segment.end.idx; index < mission_count; ++index) {
		mission_item_s mission_item{};

		if (!provider.loadMissionItem(index, mission_item)) {
			break;
		}

		// Do not infer a mode from another takeoff or landing section.
		if (isTakeoffCmd(mission_item.nav_cmd) || isLandingCmd(mission_item.nav_cmd)) {
			break;
		}

		if (updateVtolStateFromMissionItem(mission_item, state, true)) {
			break;
		}
	}

	return state;
}

// A nominal segment and a jump edge can share endpoints but have different execution state.
static bool hasSegmentTwin(const ProjectionCandidateBuffer &candidate_buffer, const Segment &segment)
{
	for (uint8_t i = 0; i < candidate_buffer.count; ++i) {
		const Segment &other = candidate_buffer.candidates[i].segment;

		if (other.isLoop() != segment.isLoop() && other.start.idx == segment.start.idx && other.end.idx == segment.end.idx) {
			return true;
		}
	}

	return false;
}

// The nominal twin replaces a jump edge the vehicle does not fly, which would otherwise win their tie.
// Taking the edge's slot keeps the twin even when closer candidates fill the buffer.
static bool replaceUnflownJumpTwin(ProjectionCandidateBuffer &candidate_buffer,
				   const RouteProjectionCandidate &candidate, int32_t flown_jump_item_index)
{
	if (candidate.segment.isLoop()) {
		return false;
	}

	for (uint8_t i = 0; i < candidate_buffer.count; ++i) {
		RouteProjectionCandidate &other = candidate_buffer.candidates[i];

		if (other.segment.isLoop() && other.segment.jump_item_index != flown_jump_item_index
		    && other.segment.start.idx == candidate.segment.start.idx && other.segment.end.idx == candidate.segment.end.idx) {
			other = candidate;
			return true;
		}
	}

	return false;
}

// Clears one reference's candidates and window before a walk.
static void resetCandidateSearch(ProjectionReference &reference, bool rescan)
{
	reference.candidate_buffer.count = 0;
	reference.search_state = {};
	reference.search_state.rescan = rescan;
	reference.search_state.cos_lat = static_cast<float>(cos(math::radians(reference.position.lat)));
}

// The prune removed the candidate that set the window, or one from a full buffer that may have pushed out a
// farther candidate. Either may hide candidates that pass.
static bool pruneMayHideCandidates(const ProjectionReference &reference)
{
	const CandidateSearchState &state = reference.search_state;
	const ProjectionCandidateBuffer &candidates = reference.candidate_buffer;
	const bool lost_closest = state.min_xtrack_m < FLT_MAX
				  && (candidates.count == 0 || candidates.candidates[0].dist.xtrack_m > state.min_xtrack_m);
	return lost_closest || (state.unpruned_count == kMaxSegmentCandidates && candidates.count < kMaxSegmentCandidates);
}

void MissionRouteProjection::processCandidateForSegment(ProjectionReference &reference,
		const RouteSegmentView &segment_view,
		float xtrack_margin_m,
		bool force_candidate,
		int32_t flown_jump_item_index,
		ProjectionScanStats &stats) const
{
	const Position &reference_position = reference.position;
	CandidateSearchState &state = reference.search_state;
	ProjectionCandidateBuffer &candidate_buffer = reference.candidate_buffer;

	// Most references are far from most segments: skip their projection trig. A skipped segment's corner
	// state only matters at its end, out of the window too, unless vertical segments carry it further.
	static constexpr float kLowerBoundMarginM = 2.f; // rounding
	const bool out_of_window = !segment_view.next_zero_length_xy
				   && crossTrackLowerBound(reference_position, state.cos_lat, segment_view)
				   > state.xtrack_limit_m + kLowerBoundMarginM;
	const RawSegmentProjection projection = out_of_window ? RawSegmentProjection{}
						: projectReferenceToSegment(reference_position, segment_view);

	const bool prev_projection_on_end = state.prev_projection_on_end;

	// Only horizontal nominal legs carry the corner state to the next segment.
	if (!segment_view.segment.isLoop() && !segment_view.zero_length_xy) {
		state.prev_projection_on_end = projection.projection_on_end;
	}

	if (out_of_window) {
		return;
	}

	if (!force_candidate) {
		// The final two waypoints may share latitude/longitude, e.g. a waypoint directly above LAND.
		// Keep that location only when the reference projects onto the end of the incoming horizontal leg.
		// For an entirely vertical route, the initial corner state keeps the final point.
		if (segment_view.zero_length_xy && segment_view.last_segment && !prev_projection_on_end) {
			return;
		}

		if (!localMinimumOnSegment(projection.projection_on_start, projection.projection_on_end,
					   prev_projection_on_end, segment_view.segment.isLoop(), segment_view.last_segment)) {
			return;
		}
	}

	stats.local_min_found++;

	// Positive margins include the boundary. Zero margin keeps only the first closest candidate and the
	// nominal twin of a jump edge: without its anchor the jump edge is not flown, nor scored as a branch-off.
	if (!projection.valid || projection.xtrack_m > state.xtrack_limit_m
	    || (xtrack_margin_m <= 0.f && projection.xtrack_m >= state.min_xtrack_m
		&& !hasSegmentTwin(candidate_buffer, segment_view.segment))) {
		return;
	}

	RouteProjectionCandidate candidate{};

	if (!buildProjectionCandidate(segment_view, projection, candidate)) {
		return;
	}

	if (replaceUnflownJumpTwin(candidate_buffer, candidate, flown_jump_item_index)) {
		return;
	}

	// A blocked candidate must neither narrow the rescanned window nor take a place in it.
	if (state.rescan && _path_checker) {
		bool clear = false;

		if (!_path_checker->checkCandidate(reference, candidate, clear)) {
			stats.geofence_unavailable = true;
			return;
		}

		if (!clear) {
			return;
		}
	}

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

	if (batch.count == 0 || batch.count > kMaxSafePointBatch
	    || !PX4_ISFINITE(request.xtrack_margin_m) || request.xtrack_margin_m < 0.f) {
		return FailureReason::kInvalidRequest;
	}

	for (uint8_t i = 0; i < batch.count; ++i) {
		resetCandidateSearch(batch.items[i], false);
	}

	ActiveJumpContext active_jump{};
	const FailureReason active_jump_status = resolveActiveJump(_provider, request.home_altitude_amsl,
			request.active_jump_anchor, active_jump);

	if (active_jump_status != FailureReason::kNone) {
		return active_jump_status;
	}

	SegmentDistanceAlong current_segment_bounds{};
	bool current_segment_located = !request.compute_current_segment_bounds;
	float route_length = 0.f;
	int32_t route_end_index = -1;
	ProjectionScanStats stats{};
	RouteSegmentView segment_view{};

	// A second walk rescans only the references whose candidates the prune may have hidden, so without a fence
	// obstacle there is one walk and one batched prune.
	for (int walk = 0; walk < 2; ++walk) {
		const bool rescan = walk > 0;
		RouteSegmentCursor cursor(_provider, request.home_altitude_amsl);

		if (!cursor.init()) {
			return cursor.failureReason();
		}

		while (cursor.next(segment_view)) {
			stats.segments_processed++;

			if (!current_segment_located) {
				current_segment_located = locateCurrentSegment(segment_view, request, active_jump, current_segment_bounds);
			}

			// An unflown jump edge is no branch-off: skip it before it narrows the window.
			if (request.skip_unflown_jump_edges && segment_view.segment.isLoop()
			    && segment_view.segment.jump_item_index != active_jump.jump_item_index) {
				continue;
			}

			// Keep the vertical segment the vehicle flies (e.g. B' -> LAND while descending) despite a small
			// drift toward the horizontal leg. During a jump the vehicle flies the jump edge instead.
			const bool flying_vertical_segment = segment_view.zero_length_xy && !segment_view.segment.isLoop()
							     && !active_jump.valid()
							     && isVehicleSegment(segment_view.segment, segment_view.start_dist_along_route_m,
									     segment_view.length_m, request.mission_index, request.is_flying_reverse);

			// Skip other vertical segments; the next horizontal leg provides the candidate at their
			// location. Preserve the previous corner state, including true at the route start.
			if (segment_view.zero_length_xy && !segment_view.last_segment && !flying_vertical_segment) {
				continue;
			}

			for (uint8_t i = 0; i < batch.count; ++i) {
				if (!rescan || batch.items[i].search_state.rescan) {
					processCandidateForSegment(batch.items[i], segment_view, request.xtrack_margin_m, flying_vertical_segment,
								   active_jump.jump_item_index, stats);
				}
			}

			if (stats.geofence_unavailable) {
				return FailureReason::kGeofenceUnavailable;
			}
		}

		if (cursor.failed()) {
			return cursor.failureReason();
		}

		if (rescan) {
			break;
		}

		route_length = cursor.routeLength();
		route_end_index = cursor.nominalPositionIndex();

		if (!_path_checker) {
			break;
		}

		// Geofence checks prune the retained candidates before any selection.
		for (uint8_t i = 0; i < batch.count; ++i) {
			batch.items[i].search_state.unpruned_count = batch.items[i].candidate_buffer.count;
		}

		if (!_path_checker->pruneCandidates(batch)) {
			return FailureReason::kGeofenceUnavailable;
		}

		bool rescan_needed = false;

		for (uint8_t i = 0; i < batch.count; ++i) {
			ProjectionReference &reference = batch.items[i];
			bool clear = false;

			if (!pruneMayHideCandidates(reference)) {
				continue;
			}

			if (!_path_checker->checkReference(reference, clear)) {
				return FailureReason::kGeofenceUnavailable;
			}

			// Nothing passes from a reference outside the fence: it keeps no candidates.
			if (clear) {
				resetCandidateSearch(reference, true);
				rescan_needed = true;
			}
		}

		if (!rescan_needed) {
			break;
		}
	}

	bool any_candidate_found = false;

	for (uint8_t i = 0; i < batch.count; ++i) {
		if (batch.items[i].candidate_buffer.count > 0) {
			any_candidate_found = true;
			break;
		}
	}

	if (any_candidate_found) {
		distance_summary.current_segment_bounds = current_segment_bounds;
		distance_summary.route_length = route_length;
		distance_summary.route_end_index = route_end_index;
		return FailureReason::kNone;
	}

	if (stats.segments_processed == 0) {
		return FailureReason::kNoSegmentsFound;
	}

	if (stats.local_min_found == 0) {
		return FailureReason::kNoLocalMinFound;
	}

	return FailureReason::kNoValidCandidateFound;
}

// Returns the index of the branch-in candidate, or -1 when no candidate has a finite score.
static int selectBranchInCandidate(const ProjectionCandidateBuffer &candidate_buffer,
				   const SegmentDistanceAlong &current_segment_bounds, int32_t mission_index,
				   bool is_flying_reverse, const ActiveJumpAnchor &active_jump_anchor)
{
	int best_candidate_index = -1;
	float min_path_distance_m = FLT_MAX;
	SegmentDistanceAlong segment_bounds = current_segment_bounds;

	if (!segment_bounds.valid()) {
		PX4_WARN("Invalid current segment (idx %d), using 0",
			 static_cast<int>(mission_index));
		segment_bounds.start_dist_along_route_m = 0.f;
		segment_bounds.end_dist_along_route_m = 0.f;
	}

	for (uint8_t i = 0; i < candidate_buffer.count; ++i) {
		const RouteProjectionCandidate &candidate = candidate_buffer.candidates[i];
		const float dist_to_start_m = fabsf(candidate.dist.along_route_m - segment_bounds.start_dist_along_route_m);
		const float dist_to_end_m = fabsf(candidate.dist.along_route_m - segment_bounds.end_dist_along_route_m);
		const float projection_to_segment_dist_m = fminf(dist_to_start_m, dist_to_end_m);
		const float candidate_path_distance_m = candidate.dist.xtrack_m + projection_to_segment_dist_m;

		if (!PX4_ISFINITE(candidate_path_distance_m)) {
			continue;
		}

		// The segment being flown wins outright: the anchored jump edge, else the vehicle segment.
		// A current TAKEOFF stack needs it: its score only ties with the closer first leg.
		const bool priority_match = active_jump_anchor.valid()
					    ? (candidate.segment.validLoop()
					       && active_jump_anchor.jump_item_index == candidate.segment.jump_item_index)
					    : isVehicleSegment(candidate.segment, candidate.dist.along_route_m,
							    candidate.dist.segment_length_m, mission_index, is_flying_reverse);

		const bool is_shorter_path = candidate_path_distance_m < min_path_distance_m;

		if (priority_match || is_shorter_path) {
			min_path_distance_m = candidate_path_distance_m;
			best_candidate_index = i;

			if (priority_match) {
				break;
			}
		}
	}

	return best_candidate_index;
}

FailureReason MissionRouteProjection::collectVehicleProjection(const Position &vehicle_position,
		int32_t mission_index, const PlannerConfig &config, ProjectionReferenceBatch &batch,
		ProjectionContext &projection_context) const
{
	if (!vehicle_position.valid()) {
		return FailureReason::kNoValidGlobalPos;
	}

	if (_provider.missionCount() <= 0) {
		return FailureReason::kNoValidWaypoints;
	}

	if (mission_index < 0 || mission_index >= _provider.missionCount()) {
		PX4_ERR("Invalid mission idx: %d (count %d)",
			static_cast<int>(mission_index), static_cast<int>(_provider.missionCount()));
		return FailureReason::kInvalidRequest;
	}

	// The vehicle reuses the batched safe point scan as a batch of one, which resets its candidates.
	batch.count = 1;
	batch.items[0].position = vehicle_position;
	batch.items[0].source_index = -1;

	// The vehicle scan also locates the current segment used for the branch-in selection score.
	ProjectionScanRequest scan_request{};
	scan_request.home_altitude_amsl = config.parameters.home_altitude_amsl;
	scan_request.xtrack_margin_m = config.parameters.vehicle_projection_search_dist_m;
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
		  static_cast<double>(distance_summary.current_segment_bounds.start_dist_along_route_m),
		  static_cast<double>(distance_summary.current_segment_bounds.end_dist_along_route_m),
		  static_cast<int>(mission_index));

	const int branch_in_index = selectBranchInCandidate(candidate_buffer, distance_summary.current_segment_bounds,
				    mission_index, config.state.is_flying_reverse, config.active_jump_anchor);

	if (branch_in_index < 0) {
		PX4_ERR("Route projection: no candidate");
		return FailureReason::kNoValidCandidateFound;
	}

	projection_context.vehicle_position = vehicle_position;
	projection_context.mission_index = mission_index;
	projection_context.route_projection = candidate_buffer.candidates[branch_in_index];
	projection_context.vehicle_state = config.state;
	projection_context.route_length = distance_summary.route_length;
	projection_context.route_end_index = distance_summary.route_end_index;

	if (projection_context.route_projection.segment.validLoop()) {
		const FailureReason loop_status = findLoopBounds(projection_context.route_projection,
						  config.parameters.home_altitude_amsl, projection_context.loop_bounds);

		if (loop_status != FailureReason::kNone) {
			return loop_status;
		}
	}

	return FailureReason::kNone;
}

// On a loop jump segment the path solver weighs both exits as route distances:
// the loop start (where the vehicle entered the jump) and the loop end.
FailureReason MissionRouteProjection::findLoopBounds(const RouteProjectionCandidate &vehicle_projection,
		float home_altitude_amsl, SegmentDistanceAlong &loop_bounds) const
{
	loop_bounds = {};

	const int32_t target_index = vehicle_projection.segment.end.idx;
	RouteSegmentCursor cursor(_provider, home_altitude_amsl);

	if (!cursor.init()) {
		return cursor.failureReason();
	}

	RouteSegmentView segment_view{};

	while (cursor.nominalPositionIndex() != target_index && cursor.next(segment_view)) {
	}

	if (cursor.failed()) {
		return cursor.failureReason();
	}

	if (cursor.nominalPositionIndex() != target_index) {
		return FailureReason::kNoValidPath;
	}

	loop_bounds.start_dist_along_route_m = vehicle_projection.dist.along_route_m - vehicle_projection.dist.along_segment_m;
	loop_bounds.end_dist_along_route_m = cursor.routeLength();
	return FailureReason::kNone;
}

} // namespace mission_route
