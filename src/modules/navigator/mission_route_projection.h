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

#include <matrix/math.hpp>

namespace mission_route
{

struct ProjectionScanRequest {
	float home_altitude_amsl{NAN};
	float xtrack_margin_m{0.f};

	bool compute_current_segment_bounds{false};
	int32_t mission_index{-1};
	bool is_flying_reverse{false};
	ActiveJumpAnchor active_jump_anchor{};
};

struct RouteDistanceSummary {
	SegmentDistanceAlong current_segment_along{};
	float route_length{0.f};
};

/** @brief Per-reference scan scratch maintained while walking the mission segments. */
struct CandidateSearchState {
	bool prev_projection_on_end{true};
	bool projection_on_end_for_segment{false};
	float min_xtrack{FLT_MAX};
	float xtrack_limit{FLT_MAX};
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
	matrix::Vector2f segment_vector{0.f, 0.f}; /**< Cached start->end NED vector; zero for zero-length segments. */

	float route_along_start_m{0.f}; /**< Along-route distance accumulated up to this segment's start. */
	float length_m{0.f};

	bool first_segment{false};
	bool last_segment{false};
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

	/** @brief Locate the route endpoints and the first segment start. False on failure (see failureReason()). */
	bool init();

	/** @brief Advance to the next segment. False once the walk is over or has failed(). */
	bool next(RouteSegmentView &view);

	bool failed() const { return _failure_reason != FailureReason::kNone; }
	FailureReason failureReason() const { return _failure_reason; }

	/** @brief Along-route distance walked so far; the route length once the walk is over. */
	float routeLength() const { return _route_along_m; }

private:
	bool findAttachedValidPositionIndex(int32_t start_index, int32_t &attached_position_index) const;

	/** @brief Load the segment end at the given mission index, resolving DO_JUMP loop edges. */
	bool prepareNextSegment(int32_t index, FailureReason &failure_reason);

	const Provider &_provider;
	const float _home_altitude_amsl;

	Segment _segment{};
	SegmentPositions _positions{};
	int32_t _index{0};
	int32_t _first_position_index{0};
	int32_t _last_position_index{0};
	float _route_along_m{0.f};
	bool _done{false};
	FailureReason _failure_reason{FailureReason::kNone};
};

/** @brief True when the vehicle targeting mission_index is flying on the given segment. */
bool isIndexInProjectionSegment(const Segment &segment, int32_t mission_index, bool is_flying_reverse);

class MissionRouteProjection
{
public:
	explicit MissionRouteProjection(const Provider &provider) : _provider(provider) {}

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
	 * projection_context is cleared on entry and populated only when kNone is returned.
	 */
	FailureReason collectVehicleProjection(const Position &vehicle_position, int32_t mission_index,
					       const PlannerConfig &config, ProjectionReferenceBatch &batch,
					       ProjectionContext &projection_context) const;

	float accumulateRouteDistance(int32_t from_index, int32_t to_index,
				      float home_altitude_amsl) const;

	LoopContext buildLoopContext(const RouteProjectionCandidate &vehicle_projection,
				     float home_altitude_amsl) const;

private:
	struct BranchInSelection;

	struct ProjectionScanStats {
		uint32_t segments_processed{0};
		uint32_t local_min_found{0};
		uint32_t valid_candidate_found{0};
	};

	/**
	 * @brief Return true when the segment projection is a local minimum.
	 *
	 * Three cases:
	 *
	 * 1. Interior projections (not on a corner) are always a local minimum.
	 *
	 * 2. Corner projection is a local minimum when both the previous and the current segment
	 *    project onto the same shared corner. This happens when two consecutive segments form
	 *    a V-shape and the reference point is closest to the apex:
	 *
	 *     prev seg  curr seg
	 *       A \    / C        The reference point P projects onto corner B
	 *          \  /           from both segments, so
	 *         B \/            prev_proj_on_end && proj_on_start -> local minimum.
	 *
	 *            ^
	 *            P
	 *
	 *    The terminal route endpoint (last segment, projection on end) is also accepted because
	 *    there is no following segment to compare against.
	 *
	 * 3. DO_JUMP loop-edge corner projections are rejected because those corners already belong
	 *    to their nominal route segments, so only interior projections on the loop jump segment
	 *    are kept.
	 */
	bool localMinimumOnSegment(bool proj_on_start, bool proj_on_end,
				   bool prev_proj_on_end, bool jumping,
				   bool last_segment) const;

	bool validateCandidate(const RouteProjectionCandidate &candidate) const;

	void insertCandidateSorted(ProjectionCandidateBuffer &candidate_buffer,
				   const RouteProjectionCandidate &candidate) const;

	/** @brief Trim the projection candidate buffer based on xtrack window. */
	void pruneProjectionCandidates(ProjectionCandidateBuffer &candidate_buffer,
				       float xtrack_limit) const;

	/**
	 * @brief Project one reference point onto one segment, apply local-minimum rules,
	 * and maintain the shrinking cross-track candidate window.
	 */
	void processCandidateForSegment(const Position &reference_position,
					const RouteSegmentView &segment,
					float xtrack_margin_m,
					CandidateSearchState &state,
					ProjectionCandidateBuffer &candidate_buffer,
					ProjectionScanStats &stats) const;

	FailureReason selectBranchInCandidate(const ProjectionCandidateBuffer &candidate_buffer,
					      const SegmentDistanceAlong &current_segment_along,
					      int32_t mission_index,
					      bool is_flying_reverse,
					      const ActiveJumpAnchor &active_jump_anchor,
					      BranchInSelection &selection) const;

	const Provider &_provider;
};

} // namespace mission_route
