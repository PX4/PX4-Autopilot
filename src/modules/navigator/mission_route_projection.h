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
 * @file mission_route_projection.h
 *
 * Mission-route projection scanner. This unit owns mission segment scanning,
 * route projection candidate buffering, and vehicle branch-in selection.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "mission_route_internal_types.h"
#include "mission_route_provider.h"

#include <float.h>
#include <stdint.h>

#include <lib/geo/geo.h>
#include <matrix/math.hpp>

namespace mission_route
{

class PathChecker;

struct ProjectionScanRequest {
	float home_altitude_amsl{NAN};
	float xtrack_margin_m{0.f};

	bool compute_current_segment_bounds{false};
	int32_t mission_index{-1};
	bool is_flying_reverse{false};
	ActiveJumpAnchor active_jump_anchor{};
};

struct RouteDistanceSummary {
	SegmentDistanceAlong current_segment_bounds{};
	float route_length{0.f};
	int32_t route_end_index{-1};
};

/** @brief Per-reference scan scratch maintained while walking the mission segments. */
struct CandidateSearchState {
	bool prev_projection_on_end{true};
	float min_xtrack_m{FLT_MAX};
	float xtrack_limit_m{FLT_MAX};
};

struct ProjectionReference {
	Position position{};
	int32_t source_index{-1};
	ProjectionCandidateBuffer candidate_buffer{};
	CandidateSearchState search_state{}; /**< Scratch used by findProjectionCandidates, no meaning after the scan. */
};

/** @brief Fixed-size batch of reference points evaluated during a single scan pass. */
struct ProjectionReferenceBatch {
	uint8_t count{0};
	ProjectionReference items[kMaxSafePointBatch] {};
};

/** @brief One ready-to-project segment of the mission route, yielded by RouteSegmentCursor. */
struct RouteSegmentView {
	Segment segment{};
	SegmentPositions positions{};
	MapProjection map_projection{}; /**< Centered on the segment start and reused for every reference in the batch. */
	matrix::Vector2f segment_vector{0.f, 0.f}; /**< Cached start->end map vector; zero for zero-length segments. */

	float start_dist_along_route_m{0.f}; /**< Along-route distance accumulated up to this segment's start. */
	float length_m{0.f};

	bool first_segment{false};
	bool last_segment{false}; /**< Last nominal segment; trailing jump edges may still follow. */
	bool zero_length_xy{false};
};

/**
 * @brief Forward walk over the segments of the uploaded mission route.
 *
 * next() yields one ready-to-project RouteSegmentView per call, so consumers
 * iterate complete segments without touching mission items or walk rules.
 */
class RouteSegmentCursor
{
public:
	RouteSegmentCursor(const Provider &provider, float home_altitude_amsl) :
		_provider(provider), _home_altitude_amsl(home_altitude_amsl) {}

	/** @brief Locate the first route position. False on failure (see failureReason()). */
	bool init();

	/** @brief Advance to the next segment. False once the walk is over or has failed(). */
	bool next(RouteSegmentView &view);

	bool failed() const { return _failure_reason != FailureReason::kNone; }
	FailureReason failureReason() const { return _failure_reason; }

	/** @brief Along-route distance walked so far; the route length once the walk is over. */
	float routeLength() const { return _dist_along_route_m; }

	/** @brief Current nominal position; route end after a successful walk. */
	int32_t nominalPositionIndex() const { return _segment.start.idx; }

private:
	enum class SegmentBoundary : uint8_t {
		kIntermediate,
		kFinal
	};

	bool prepareJumpSegment(int32_t index, const mission_item_s &jump_item, RouteSegmentView &view);
	void prepareNominalSegment(const SegmentEndpoint &end, const Position &end_position,
				   SegmentBoundary boundary, RouteSegmentView &view);
	void prepareSegmentView(SegmentBoundary boundary, RouteSegmentView &view);
	bool fail(FailureReason failure_reason);

	const Provider &_provider;
	const float _home_altitude_amsl;

	Segment _segment{};
	SegmentPositions _positions{};
	SegmentEndpoint _next_endpoint{};
	Position _next_position{};
	int32_t _index{0};
	int32_t _first_position_index{0};
	float _dist_along_route_m{0.f};
	bool _next_position_cached{false};
	bool _done{false};
	FailureReason _failure_reason{FailureReason::kNone};
};

/** @brief True when the vehicle targeting mission_index is flying on the given segment. */
bool isIndexInProjectionSegment(const Segment &segment, int32_t mission_index, bool is_flying_reverse);

/**
 * @brief Required VEHICLE_VTOL_STATE on one segment: the last explicit transition or implicit
 * VTOL_TAKEOFF front transition before its target waypoint. Jump edges use the state before
 * DO_JUMP, overridden by transitions between the jump target and its resolved waypoint.
 * If no preceding mode command exists, infer the opposite mode from the next transition in
 * nominal order, ignoring DO_JUMP targets and repeat counts. Lookahead stops at a landing,
 * takeoff or failed read. Returns VEHICLE_VTOL_STATE_UNDEFINED when unresolved or the segment is invalid.
 */
uint8_t vtolStateForSegment(const Provider &provider, const Segment &segment);

class MissionRouteProjection
{
public:
	explicit MissionRouteProjection(const Provider &provider, const PathChecker *path_checker = nullptr) :
		_provider(provider), _path_checker(path_checker) {}

	/**
	 * @brief Scan the mission once and collect projection candidates for every reference point in the batch.
	 *
	 * distance_summary is cleared on entry and populated only when kNone is returned.
	 */
	FailureReason findProjectionCandidates(const ProjectionScanRequest &request,
					       ProjectionReferenceBatch &batch, RouteDistanceSummary &distance_summary) const;

	/**
	 * @brief Project the vehicle onto the mission route and choose the continuity-preserving branch-in candidate.
	 *
	 * projection_context is only valid when kNone is returned.
	 */
	FailureReason collectVehicleProjection(const Position &vehicle_position, int32_t mission_index,
					       const PlannerConfig &config, ProjectionReferenceBatch &batch,
					       ProjectionContext &projection_context) const;

private:
	struct ProjectionScanStats {
		uint32_t segments_processed{0};
		uint32_t local_min_found{0};
	};

	/**
	 * @brief Project one reference point onto one segment, apply local-minimum rules,
	 * and maintain the shrinking cross-track candidate window.
	 *
	 * @param force_candidate Skip the local-minimum rules; set for the vertical segment the
	 * vehicle is currently flying.
	 */
	void processCandidateForSegment(const Position &reference_position,
					const RouteSegmentView &segment_view,
					float xtrack_margin_m,
					bool force_candidate,
					CandidateSearchState &state,
					ProjectionCandidateBuffer &candidate_buffer,
					ProjectionScanStats &stats) const;

	FailureReason findLoopBounds(const RouteProjectionCandidate &vehicle_projection, float home_altitude_amsl,
				     SegmentDistanceAlong &loop_bounds) const;

	const Provider &_provider;
	const PathChecker *_path_checker;
};

} // namespace mission_route
