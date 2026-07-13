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

#include "mission_route_provider.h"
#include "mission_route_types.h"

#include <float.h>
#include <stdint.h>

#include <matrix/math.hpp>

class MissionRouteProjectionCandidateBufferTestPeer;

namespace mission_route
{

enum class PositionLookupStatus : uint8_t {
	kFound,
	kNoPositionFound,
	kLoadFailed,
	kInvalidPosition
};

struct ProjectionScanRequest {
	float home_altitude_amsl{NAN};
	float xtrack_margin_m{0.f};

	bool compute_current_segment_bounds{false};
	int32_t mission_index{-1};
	bool is_flying_reverse{false};
	Segment last_flown_loop_segment{};
};

struct ProjectionScanResult {
	bool success{false};
	FailureReason failure_reason{FailureReason::kUnknown};

	SegmentDistanceAlong current_segment_along{};
	float route_length{0.f};
};

/** @brief Per-reference scan scratch maintained while walking the mission segments. */
struct CandidateSearchState {
	bool prev_projection_on_end{false};
	bool projection_on_end_for_segment{false};
	float min_xtrack{0.f};
	float xtrack_limit{0.f};

	void reset()
	{
		prev_projection_on_end = true;
		projection_on_end_for_segment = false;
		min_xtrack = FLT_MAX;
		xtrack_limit = FLT_MAX;
	}
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

class MissionRouteProjection
{
public:
	explicit MissionRouteProjection(const Provider &provider) : _provider(provider) {}

	/** @brief Scan the mission once and collect projection candidates for every reference point in the batch. */
	ProjectionScanResult findProjectionCandidates(const ProjectionScanRequest &request,
			ProjectionReferenceBatch &batch) const;

	/** @brief Project the vehicle onto the mission route and choose the continuity-preserving branch-in candidate. */
	VehicleProjectionResult collectVehicleProjection(const Position &vehicle_position, int32_t mission_index,
			const PlannerConfig &config, ProjectionReferenceBatch &batch) const;

	bool isIndexInProjectionSegment(const Segment &segment, int32_t mission_index,
					bool is_flying_reverse) const;

	float accumulateRouteDistance(int32_t from_index, int32_t to_index,
				      float home_altitude_amsl) const;

	LoopContext buildLoopContext(const RouteProjectionCandidate &vehicle_projection,
				     float home_altitude_amsl) const;

private:
	friend class ::MissionRouteProjectionCandidateBufferTestPeer;

	struct BranchInSelectionResult;

	struct RouteSegmentView {
		Segment segment{};
		SegmentPositions positions{};
		matrix::Vector2f segment_vector{0.f, 0.f}; /**< Cached start->end NED vector; zero for zero-length segments. */

		float route_along_start_m{0.f}; /**< Along-route distance accumulated up to this segment's start. */
		float length_m{0.f};

		bool last_segment{false};
		bool zero_length_xy{false};
	};

	struct ProjectionScanStats {
		uint32_t segments_processed{0};
		uint32_t local_min_found{0};
		uint32_t valid_candidate_found{0};
	};

	PositionLookupStatus findNextValidPositionIndex(int32_t start_index, float home_altitude_amsl,
			int32_t &next_position_index) const;

	bool findAttachedValidPositionIndex(int32_t start_index, float home_altitude_amsl,
					    int32_t &attached_position_index) const;

	bool prepareNextSegment(int32_t index, Segment &segment,
				SegmentPositions &segment_positions,
				float home_altitude_amsl,
				FailureReason &failure_reason) const;

	/** @brief Fill the along-track bounds of the current segment once they are reached during scanning. */
	bool fillCurrentSegmentAlongIfNeeded(int32_t mission_index,
					     const Segment &segment,
					     bool is_flying_reverse,
					     float route_along,
					     float segment_length,
					     const Segment &last_flown_loop_segment,
					     SegmentDistanceAlong &current_segment_along) const;

	/**
	 * @brief Return true when the segment projection is a local minimum.
	 *
	 * Three cases:
	 *
	 * 1. Interior projections (not on a corner) are always a local minimum.
	 *
	 * 2. Corner projection is a local minimum when both the previous and the current segment
	 *    project onto the same shared corner. This happens when two consecutive segments form
	 *    a V-shape and the reference point is closest to the apex.
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

	BranchInSelectionResult selectBranchInCandidate(const ProjectionCandidateBuffer &candidate_buffer,
			const SegmentDistanceAlong &current_segment_along,
			int32_t mission_index,
			bool is_flying_reverse,
			const Segment &last_flown_loop_segment) const;

	const Provider &_provider;
};

} // namespace mission_route
