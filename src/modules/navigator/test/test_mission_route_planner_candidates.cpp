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
 * @file test_mission_route_planner_candidates.cpp
 *
 * Helper-level tests for MissionRoutePlanner candidate-buffer insertion and pruning.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include <gtest/gtest.h>

#include "mission_route_planner.h"
#include "test_RTL_helpers.h"

#include <limits>
#include <vector>

using rtl_test_reference::kAlt;
using rtl_test_reference::kBaseLat;
using rtl_test_reference::kBaseLon;

class MissionRoutePlannerCandidateBufferTestPeer : public MissionRoutePlanner
{
public:
	explicit MissionRoutePlannerCandidateBufferTestPeer(const Provider &provider) : MissionRoutePlanner(provider) {}

	void insertCandidateSortedForTest(CandidateBuffer &candidate_buffer, const SegmentCandidate &candidate) const
	{
		insertCandidateSorted(candidate_buffer, candidate);
	}

	void pruneProjectionCandidatesForTest(CandidateBuffer &candidate_buffer, float xtrack_limit) const
	{
		pruneProjectionCandidates(candidate_buffer, xtrack_limit);
	}

	bool validateCandidateForTest(const SegmentCandidate &candidate) const
	{
		return validateCandidate(candidate);
	}

	bool isIndexInProjectionSegmentForTest(const Segment &projection_segment, int32_t mission_index,
					       bool is_flying_reverse) const
	{
		return isIndexInProjectionSegment(projection_segment, mission_index, is_flying_reverse);
	}

	bool localMinimumOnSegmentForTest(bool proj_on_start, bool proj_on_end, bool prev_proj_on_end,
					  bool jumping, bool last_segment) const
	{
		return localMinimumOnSegment(proj_on_start, proj_on_end, prev_proj_on_end, jumping, last_segment);
	}
};

class MissionRoutePlannerCandidateBufferTest : public ::testing::Test
{
protected:
	static std::vector<mission_item_s> makeMissionItems()
	{
		return {
			makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f,   0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f,   0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f,   0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f,   0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 600.f,   0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 700.f,   0.f, kAlt),
		};
	}

	static MissionRoutePlanner::SegmentCandidate makeCandidate(int32_t id, float xtrack)
	{
		MissionRoutePlanner::SegmentCandidate candidate{};
		candidate.segment.start.idx = id;
		candidate.dist.xtrack = xtrack;
		return candidate;
	}

	static MissionRoutePlanner::SegmentCandidate makeValidCandidate()
	{
		MissionRoutePlanner::SegmentCandidate candidate{};
		candidate.segment.start.idx = 2;
		candidate.segment.start.nav_cmd = NAV_CMD_WAYPOINT;
		candidate.segment.end.idx = 4;
		candidate.segment.end.nav_cmd = NAV_CMD_WAYPOINT;
		candidate.segment_positions.start = makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt);
		candidate.segment_positions.end = makePositionFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt);
		candidate.projection = makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt);
		candidate.dist.xtrack = 5.0f;
		candidate.dist.along = 100.0f;
		candidate.dist.segment_length = 200.0f;
		candidate.dist.on_segment = 100.0f;
		return candidate;
	}

	static void expectCandidateIds(const MissionRoutePlanner::CandidateBuffer &buffer, const std::vector<int32_t> &expected_ids)
	{
		ASSERT_EQ(buffer.count, expected_ids.size());

		for (size_t i = 0; i < expected_ids.size(); ++i) {
			EXPECT_EQ(buffer.candidates[i].segment.start.idx, expected_ids[i]);
		}
	}

	void fillBufferToCapacity(MissionRoutePlanner::CandidateBuffer &buffer)
	{
		for (int32_t i = 0; i < MissionRoutePlanner::MAX_SEGMENT_CANDIDATES; ++i) {
			planner.insertCandidateSortedForTest(buffer, makeCandidate(i + 1, static_cast<float>(i + 1)));
		}
	}

	static std::vector<int32_t> expectedSequentialIds(int32_t first_id, int32_t count)
	{
		std::vector<int32_t> ids;
		ids.reserve(static_cast<size_t>(count));

		for (int32_t i = 0; i < count; ++i) {
			ids.push_back(first_id + i);
		}

		return ids;
	}

	VectorProvider provider{makeMissionItems(), {}};
	MissionRoutePlannerCandidateBufferTestPeer planner{provider};
};

// WHY: An empty candidate buffer should accept the first projection candidate unchanged.
// WHAT: Empty buffer plus one candidate -> insertCandidateSorted stores that candidate at index 0.
TEST_F(MissionRoutePlannerCandidateBufferTest, InsertCandidateSortedEmptyBufferInserts)
{
	// GIVEN: An empty xtrack-sorted candidate buffer.
	MissionRoutePlanner::CandidateBuffer buffer{};

	// WHEN: The first candidate is inserted.
	planner.insertCandidateSortedForTest(buffer, makeCandidate(1, 5.0f));

	// THEN: The buffer contains exactly that candidate.
	ASSERT_EQ(buffer.count, 1U);
	expectCandidateIds(buffer, {1});
}

// WHY: Inserting a better candidate must shift the existing entries to preserve sorted order.
// WHAT: Buffer [1@5m] plus candidate 2@3m -> insertCandidateSorted stores [2,1].
TEST_F(MissionRoutePlannerCandidateBufferTest, InsertCandidateSortedInsertsAtFrontAndShifts)
{
	// GIVEN: A one-entry buffer with a worse xtrack value than the incoming candidate.
	MissionRoutePlanner::CandidateBuffer buffer{};
	planner.insertCandidateSortedForTest(buffer, makeCandidate(1, 5.0f));

	// WHEN: A better candidate is inserted.
	planner.insertCandidateSortedForTest(buffer, makeCandidate(2, 3.0f));

	// THEN: The new candidate moves to the front and the previous one shifts back.
	ASSERT_EQ(buffer.count, 2U);
	expectCandidateIds(buffer, {2, 1});
}

// WHY: Worse candidates still need to append cleanly so the fixed-size buffer remains globally sorted.
// WHAT: Buffer [1@1m] plus candidate 2@3m -> insertCandidateSorted stores [1,2].
TEST_F(MissionRoutePlannerCandidateBufferTest, InsertCandidateSortedInsertsAtEnd)
{
	// GIVEN: A one-entry buffer with a better xtrack value than the incoming candidate.
	MissionRoutePlanner::CandidateBuffer buffer{};
	planner.insertCandidateSortedForTest(buffer, makeCandidate(1, 1.0f));

	// WHEN: A worse candidate is inserted.
	planner.insertCandidateSortedForTest(buffer, makeCandidate(2, 3.0f));

	// THEN: The new candidate appends at the end without disturbing the order.
	ASSERT_EQ(buffer.count, 2U);
	expectCandidateIds(buffer, {1, 2});
}

// WHY: Equal-xtrack insertions must stay stable so earlier segment candidates keep priority.
// WHAT: Two candidates with identical xtrack -> insertCandidateSorted preserves insertion order.
TEST_F(MissionRoutePlannerCandidateBufferTest, InsertCandidateSortedEqualXtrackIsStable)
{
	// GIVEN: A buffer that already contains one candidate at xtrack 5 m.
	MissionRoutePlanner::CandidateBuffer buffer{};
	planner.insertCandidateSortedForTest(buffer, makeCandidate(1, 5.0f));

	// WHEN: A second candidate with the same xtrack is inserted.
	planner.insertCandidateSortedForTest(buffer, makeCandidate(2, 5.0f));

	// THEN: The original candidate remains ahead of the equal-distance insertion.
	ASSERT_EQ(buffer.count, 2U);
	expectCandidateIds(buffer, {1, 2});
}

// WHY: Once the fixed-size buffer is full, a worse candidate must be rejected instead of evicting a better one.
// WHAT: Full buffer [1,2,3] plus worse candidate 4 -> insertCandidateSorted keeps [1,2,3].
TEST_F(MissionRoutePlannerCandidateBufferTest, InsertCandidateSortedFullBufferRejectsWorseCandidate)
{
	// GIVEN: A full buffer already containing the best xtrack candidates up to capacity.
	MissionRoutePlanner::CandidateBuffer buffer{};
	fillBufferToCapacity(buffer);

	// WHEN: A candidate worse than every buffered entry is inserted.
	planner.insertCandidateSortedForTest(buffer,
					     makeCandidate(99, static_cast<float>(MissionRoutePlanner::MAX_SEGMENT_CANDIDATES) + 100.f));

	// THEN: The buffer remains unchanged.
	ASSERT_EQ(buffer.count, MissionRoutePlanner::MAX_SEGMENT_CANDIDATES);
	expectCandidateIds(buffer, expectedSequentialIds(1, MissionRoutePlanner::MAX_SEGMENT_CANDIDATES));
}

// WHY: The fixed-size candidate buffer must keep the closest projections and drop the farthest.
// WHAT: Full buffer [1,2,3] plus closer candidate 0 -> insertCandidateSorted keeps [0,1,2].
TEST_F(MissionRoutePlannerCandidateBufferTest, InsertCandidateSortedFullBufferDropsWorstCandidate)
{
	// GIVEN: A full sorted buffer at the current capacity limit.
	MissionRoutePlanner::CandidateBuffer buffer{};
	fillBufferToCapacity(buffer);

	// WHEN: A new best candidate is inserted.
	planner.insertCandidateSortedForTest(buffer, makeCandidate(0, 0.5f));

	// THEN: The farthest candidate is dropped and the remaining three stay sorted.
	ASSERT_EQ(buffer.count, MissionRoutePlanner::MAX_SEGMENT_CANDIDATES);
	std::vector<int32_t> expected_ids{0};
	const std::vector<int32_t> retained_ids = expectedSequentialIds(1, MissionRoutePlanner::MAX_SEGMENT_CANDIDATES - 1);
	expected_ids.insert(expected_ids.end(), retained_ids.begin(), retained_ids.end());
	expectCandidateIds(buffer, expected_ids);
}

// WHY: Mid-buffer insertion is a separate branch from front insertion and must still evict only the worst entry.
// WHAT: Full buffer [1,2,3] plus candidate 25@2.5m -> insertCandidateSorted keeps [1,2,25].
TEST_F(MissionRoutePlannerCandidateBufferTest, InsertCandidateSortedFullBufferInsertInMiddleDropsWorstCandidate)
{
	// GIVEN: A full sorted buffer where the new candidate belongs before the current worst entry.
	MissionRoutePlanner::CandidateBuffer buffer{};
	fillBufferToCapacity(buffer);

	// WHEN: A mid-ranked candidate is inserted.
	planner.insertCandidateSortedForTest(buffer,
					     makeCandidate(25, static_cast<float>(MissionRoutePlanner::MAX_SEGMENT_CANDIDATES) - 0.5f));

	// THEN: The insertion lands in the middle and the previous worst candidate is dropped.
	ASSERT_EQ(buffer.count, MissionRoutePlanner::MAX_SEGMENT_CANDIDATES);
	std::vector<int32_t> expected_ids =
		expectedSequentialIds(1, MissionRoutePlanner::MAX_SEGMENT_CANDIDATES - 1);
	expected_ids.push_back(25);
	expectCandidateIds(buffer, expected_ids);
}

// WHY: Defensive clamping matters because count is mutable state and the helper must not trust corrupted callers.
// WHAT: count=20 with first three sorted candidates and candidate 0@0.5m -> insertCandidateSorted clamps and keeps [0,1,2].
TEST_F(MissionRoutePlannerCandidateBufferTest, InsertCandidateSortedOverfullCountIsClamped)
{
	// GIVEN: An overfull count value with valid sorted candidates already stored up to capacity.
	MissionRoutePlanner::CandidateBuffer buffer{};
	buffer.count = 20;

	for (int32_t i = 0; i < MissionRoutePlanner::MAX_SEGMENT_CANDIDATES; ++i) {
		buffer.candidates[i] = makeCandidate(i + 1, static_cast<float>(i + 1));
	}

	// WHEN: A new best candidate is inserted.
	planner.insertCandidateSortedForTest(buffer, makeCandidate(0, 0.5f));

	// THEN: The helper clamps to the real capacity and keeps the best candidates.
	ASSERT_EQ(buffer.count, MissionRoutePlanner::MAX_SEGMENT_CANDIDATES);
	std::vector<int32_t> expected_ids{0};
	const std::vector<int32_t> retained_ids = expectedSequentialIds(1, MissionRoutePlanner::MAX_SEGMENT_CANDIDATES - 1);
	expected_ids.insert(expected_ids.end(), retained_ids.begin(), retained_ids.end());
	expectCandidateIds(buffer, expected_ids);
}

// WHY: Tightening the xtrack window must trim every stale tail candidate that now lies outside the limit.
// WHAT: Sorted buffer [1m, 2m, 4m] with limit 2.5m -> pruneProjectionCandidates keeps [1m, 2m].
TEST_F(MissionRoutePlannerCandidateBufferTest, PruneProjectionCandidatesTrimsTailOutsideLimit)
{
	// GIVEN: Three sorted candidates where only the farthest one violates the new xtrack limit.
	MissionRoutePlanner::CandidateBuffer buffer{};
	planner.insertCandidateSortedForTest(buffer, makeCandidate(1, 1.0f));
	planner.insertCandidateSortedForTest(buffer, makeCandidate(2, 2.0f));
	planner.insertCandidateSortedForTest(buffer, makeCandidate(3, 4.0f));

	// WHEN: The candidate window is tightened to 2.5 m.
	planner.pruneProjectionCandidatesForTest(buffer, 2.5f);

	// THEN: Only the in-window prefix remains.
	ASSERT_EQ(buffer.count, 2U);
	expectCandidateIds(buffer, {1, 2});
}

// WHY: The single-entry path is exactly where the old implementation regressed, so both keep and clear cases
//      need direct coverage.
// WHAT: One candidate at 4m with limit 5m -> pruneProjectionCandidates keeps the buffer unchanged.
TEST_F(MissionRoutePlannerCandidateBufferTest, PruneProjectionCandidatesSingleInLimitCandidateIsKept)
{
	// GIVEN: A one-entry buffer whose only candidate is already within the tighter xtrack limit.
	MissionRoutePlanner::CandidateBuffer buffer{};
	buffer.count = 1;
	buffer.candidates[0] = makeCandidate(7, 4.0f);

	// WHEN: The candidate buffer is pruned against the new limit.
	planner.pruneProjectionCandidatesForTest(buffer, 5.0f);

	// THEN: The in-range candidate remains in place.
	ASSERT_EQ(buffer.count, 1U);
	expectCandidateIds(buffer, {7});
}

// WHY: The previous pruneProjectionCandidates implementation returned early for count<2, which left
//      a single stale candidate alive after the search window tightened and let invalid geometry survive.
// WHAT: One candidate at 8m with limit 5m -> pruneProjectionCandidates clears the buffer.
TEST_F(MissionRoutePlannerCandidateBufferTest, PruneProjectionCandidatesSingleOutOfLimitCandidateClearsBuffer)
{
	// GIVEN: A one-entry buffer whose only candidate is already outside the tighter xtrack limit.
	MissionRoutePlanner::CandidateBuffer buffer{};
	buffer.count = 1;
	buffer.candidates[0] = makeCandidate(7, 8.0f);

	// WHEN: The candidate buffer is pruned against the new limit.
	planner.pruneProjectionCandidatesForTest(buffer, 5.0f);

	// THEN: The stale candidate is discarded instead of surviving the prune pass.
	EXPECT_EQ(buffer.count, 0U);
}

// WHY: Defensive clamping matters because the helpers treat count as external state and must stay safe
//      even if a caller accidentally leaves it larger than the fixed storage capacity.
// WHAT: count=20 with valid first three candidates and limit 2.5m -> pruneProjectionCandidates clamps to capacity and keeps [1,2].
TEST_F(MissionRoutePlannerCandidateBufferTest, PruneProjectionCandidatesOverfullCountIsClamped)
{
	// GIVEN: An overfull count value with valid sorted candidates in the fixed buffer storage.
	MissionRoutePlanner::CandidateBuffer buffer{};
	buffer.count = 20;

	for (int32_t i = 0; i < MissionRoutePlanner::MAX_SEGMENT_CANDIDATES; ++i) {
		buffer.candidates[i] = makeCandidate(i + 1, static_cast<float>(i + 1));
	}

	// WHEN: The tighter xtrack limit is applied.
	planner.pruneProjectionCandidatesForTest(buffer, 2.5f);

	// THEN: The helper clamps to the real capacity and keeps only the in-window prefix.
	ASSERT_EQ(buffer.count, 2U);
	expectCandidateIds(buffer, {1, 2});
}

// WHY: validateCandidate is the final guard before a geometric projection enters the candidate buffer,
//      so it must accept both normal segments and loop segments that run backward in mission index order.
// WHAT: A fully populated candidate is valid in nominal form and remains valid when converted to a loop segment.
TEST_F(MissionRoutePlannerCandidateBufferTest, ValidateCandidateAcceptsValidNominalAndLoopCandidates)
{
	// GIVEN: A fully populated valid nominal projection candidate.
	MissionRoutePlanner::SegmentCandidate candidate = makeValidCandidate();

	// WHEN: The candidate is validated as-is and then as a loop segment.
	const bool nominal_valid = planner.validateCandidateForTest(candidate);
	candidate.segment.start.idx = 7;
	candidate.segment.end.idx = 2;
	candidate.segment.is_loop = true;
	const bool loop_valid = planner.validateCandidateForTest(candidate);

	// THEN: Both the nominal and loop variants are accepted.
	EXPECT_TRUE(nominal_valid);
	EXPECT_TRUE(loop_valid);
}

// WHY: Candidate validation must fail fast on corrupt geometry so later selection logic never sees
//      impossible indices, invalid coordinates, or nonsensical distances.
// WHAT: Mutating a valid candidate across index, nav_cmd, distance, and position fields makes validateCandidate return false.
TEST_F(MissionRoutePlannerCandidateBufferTest, ValidateCandidateRejectsInvalidCandidates)
{
	// GIVEN: A valid baseline candidate that exercises the nominal non-loop path.
	const MissionRoutePlanner::SegmentCandidate base = makeValidCandidate();
	const float nanf = std::numeric_limits<float>::quiet_NaN();
	const double nand = std::numeric_limits<double>::quiet_NaN();

	auto expect_invalid = [&](const char *label, auto &&mutator) {
		MissionRoutePlanner::SegmentCandidate candidate = base;
		ASSERT_TRUE(planner.validateCandidateForTest(candidate)) << "Baseline corrupted!";
		mutator(candidate);
		EXPECT_FALSE(planner.validateCandidateForTest(candidate)) << label;
	};

	// WHEN: Each invalid mutation is applied independently.
	expect_invalid("missing start index", [](auto & candidate) { candidate.segment.start.idx = -1; });
	expect_invalid("missing end index", [](auto & candidate) { candidate.segment.end.idx = -1; });
	expect_invalid("out of range start index", [this](auto & candidate) {
		candidate.segment.start.idx = provider.missionCount();
	});
	expect_invalid("out of range end index", [this](auto & candidate) {
		candidate.segment.end.idx = provider.missionCount();
	});
	expect_invalid("end before start outside loop", [](auto & candidate) { candidate.segment.end.idx = candidate.segment.start.idx - 1; });
	expect_invalid("end equal start", [](auto & candidate) { candidate.segment.end.idx = candidate.segment.start.idx; });
	expect_invalid("invalid start nav_cmd", [](auto & candidate) { candidate.segment.start.nav_cmd = NAV_CMD_INVALID; });
	expect_invalid("invalid end nav_cmd", [](auto & candidate) { candidate.segment.end.nav_cmd = NAV_CMD_INVALID; });
	expect_invalid("negative xtrack", [](auto & candidate) { candidate.dist.xtrack = -1.0f; });
	expect_invalid("nan xtrack", [&](auto & candidate) { candidate.dist.xtrack = nanf; });
	expect_invalid("negative along", [](auto & candidate) { candidate.dist.along = -5.0f; });
	expect_invalid("nan along", [&](auto & candidate) { candidate.dist.along = nanf; });
	expect_invalid("negative segment length", [](auto & candidate) { candidate.dist.segment_length = -3.0f; });
	expect_invalid("nan segment length", [&](auto & candidate) { candidate.dist.segment_length = nanf; });
	expect_invalid("negative distance on segment", [](auto & candidate) { candidate.dist.on_segment = -0.1f; });
	expect_invalid("nan distance on segment", [&](auto & candidate) { candidate.dist.on_segment = nanf; });
	expect_invalid("distance on segment beyond length", [](auto & candidate) { candidate.dist.on_segment = candidate.dist.segment_length + 1.0f; });
	expect_invalid("nan segment start lat", [&](auto & candidate) { candidate.segment_positions.start.lat = nand; });
	expect_invalid("nan segment end lon", [&](auto & candidate) { candidate.segment_positions.end.lon = nand; });
	expect_invalid("nan proj lat", [&](auto & candidate) { candidate.projection.lat = nand; });
	expect_invalid("nan proj lon", [&](auto & candidate) { candidate.projection.lon = nand; });
	expect_invalid("nan proj alt", [&](auto & candidate) { candidate.projection.alt = nanf; });

	// THEN: Every corrupted candidate is rejected.
}

// WHY: isIndexInProjectionSegment defines which target index owns a projected segment, so the
//      nominal and reverse boundary rules must stay pinned down with direct helper coverage.
// WHAT: Segment [2-4] matches {3,4} in nominal flight and {2,3} in reverse flight.
TEST_F(MissionRoutePlannerCandidateBufferTest, IsIndexInProjectionSegmentHandlesDirectionBoundaries)
{
	// GIVEN: A nominal route segment spanning mission indices 2 through 4.
	MissionRoutePlanner::Segment segment{};
	segment.start.idx = 2;
	segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	segment.end.idx = 4;
	segment.end.nav_cmd = NAV_CMD_WAYPOINT;

	auto expect_match = [&](int32_t mission_index, bool reverse, bool expected) {
		EXPECT_EQ(planner.isIndexInProjectionSegmentForTest(segment, mission_index, reverse), expected)
				<< "mission_index=" << mission_index << " reverse=" << reverse;
	};

	// WHEN/THEN: Nominal flight owns the upper endpoint, reverse flight owns the lower endpoint.
	expect_match(2, false, false);
	expect_match(3, false, true);
	expect_match(4, false, true);
	expect_match(5, false, false);
	expect_match(2, true, true);
	expect_match(3, true, true);
	expect_match(4, true, false);
	expect_match(1, true, false);
	expect_match(-1, true, false);
}

// WHY: localMinimumOnSegment contains the planner's corner-filtering policy, and direct helper tests
//      are the clearest way to keep nominal-corner, terminal-endpoint, and loop-corner behavior stable.
// WHAT: Interior nominal projections are accepted, shared nominal corners are accepted only with a matching
//       previous endpoint, terminal endpoints are accepted, and loop-corner projections are rejected.
TEST_F(MissionRoutePlannerCandidateBufferTest, LocalMinimumOnSegmentHandlesNominalAndLoopCornerCases)
{
	// GIVEN: Representative nominal, terminal, and loop-corner projection states.
	const bool interior_nominal = planner.localMinimumOnSegmentForTest(false, false, false, false, false);
	const bool shared_nominal_corner = planner.localMinimumOnSegmentForTest(true, false, true, false, false);
	const bool rejected_nominal_corner = planner.localMinimumOnSegmentForTest(true, false, false, false, false);
	const bool terminal_endpoint = planner.localMinimumOnSegmentForTest(false, true, false, false, true);
	const bool loop_corner = planner.localMinimumOnSegmentForTest(true, false, true, true, false);

	// WHEN/THEN: Only the allowed local-minimum cases return true.
	EXPECT_TRUE(interior_nominal);
	EXPECT_TRUE(shared_nominal_corner);
	EXPECT_FALSE(rejected_nominal_corner);
	EXPECT_TRUE(terminal_endpoint);
	EXPECT_FALSE(loop_corner);
}
