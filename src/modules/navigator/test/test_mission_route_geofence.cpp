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

#include "mission_route_geofence.h"
#include "mission_route_projection.h"
#include "support/geofence_test_helpers.h"

using mission_route::GeofencePathChecker;
using mission_route::Position;
using mission_route::ProjectionReference;
using mission_route::ProjectionReferenceBatch;

class MissionRouteGeofenceTest : public navigator_test::GeofenceTestBase
{
protected:
	static constexpr float kAltitude = 500.f;

	void SetUp() override
	{
		param_reset_all();
		_navigator.updateParams();
		ASSERT_TRUE(resetFence(kAltitude - 100.f));
		ASSERT_TRUE(loadFence({}));
	}

	void TearDown() override { param_reset_all(); }

	Position routePosition(float north, float east, float altitude = kAltitude) const
	{
		const matrix::Vector2d coordinates = position(north, east);
		return {coordinates(0), coordinates(1), altitude};
	}

	void addCandidate(ProjectionReference &reference, const Position &projection, int32_t tag = 0)
	{
		ASSERT_LT(reference.candidate_buffer.count, mission_route::kMaxSegmentCandidates);
		auto &candidate = reference.candidate_buffer.candidates[reference.candidate_buffer.count++];
		candidate.projection = projection;
		candidate.segment.start.idx = tag;
	}
};

TEST_F(MissionRouteGeofenceTest, PrunesCrossingSegmentsAndPreservesCandidateOrder)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	const Position vehicle = routePosition(0.f, 100.f);
	GeofencePathChecker checker{_fence, vehicle};
	ProjectionReferenceBatch batch{};
	batch.count = 1;
	batch.items[0].position = vehicle;
	addCandidate(batch.items[0], routePosition(0.f, 500.f), 1);
	addCandidate(batch.items[0], routePosition(200.f, 500.f), 2);
	addCandidate(batch.items[0], routePosition(100.f, 100.f), 3);

	ASSERT_TRUE(checker.pruneCandidates(batch));
	ASSERT_EQ(batch.items[0].candidate_buffer.count, 2);
	EXPECT_EQ(batch.items[0].candidate_buffer.candidates[0].segment.start.idx, 2);
	EXPECT_EQ(batch.items[0].candidate_buffer.candidates[1].segment.start.idx, 3);
}

TEST_F(MissionRouteGeofenceTest, ChecksMoreThanOnePathBatchAndThePartialTail)
{
	// Thirteen references produce 39 paths: a full batch and a partial batch,
	// with the eleventh reference split across the boundary of 32 paths.
	static constexpr uint8_t kReferenceCount = 13;

	if (mission_route::kMaxSafePointBatch < kReferenceCount) {
		GTEST_SKIP() << "requires at least thirteen configured projection batch slots";
	}

	ASSERT_TRUE(loadFence(exclusionSquare()));
	GeofencePathChecker checker{_fence, routePosition(0.f, 0.f)};
	ProjectionReferenceBatch batch{};
	batch.count = kReferenceCount;

	for (uint8_t i = 0; i < batch.count; ++i) {
		const float north = i % 2 == 0 ? 0.f : 60.f;
		batch.items[i].position = routePosition(north, 100.f);
		addCandidate(batch.items[i], routePosition(north, 500.f), 1);
		addCandidate(batch.items[i], routePosition(north, 200.f), 2);
		addCandidate(batch.items[i], routePosition(north, 600.f), 3);
	}

	ASSERT_TRUE(checker.pruneCandidates(batch));

	for (uint8_t i = 0; i < batch.count; ++i) {
		SCOPED_TRACE(i);
		const auto &candidates = batch.items[i].candidate_buffer;

		if (i % 2 == 0) {
			ASSERT_EQ(candidates.count, 1);
			EXPECT_EQ(candidates.candidates[0].segment.start.idx, 2);

		} else {
			ASSERT_EQ(candidates.count, 3);

			for (uint8_t j = 0; j < candidates.count; ++j) {
				EXPECT_EQ(candidates.candidates[j].segment.start.idx, j + 1);
			}
		}
	}
}

TEST_F(MissionRouteGeofenceTest, RejectsReferenceWhollyInsideExclusionPolygon)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	GeofencePathChecker checker{_fence, routePosition(0.f, 0.f)};
	ProjectionReferenceBatch batch{};
	batch.count = 1;
	batch.items[0].position = routePosition(0.f, 300.f);
	addCandidate(batch.items[0], routePosition(10.f, 310.f));

	ASSERT_TRUE(checker.pruneCandidates(batch));
	EXPECT_EQ(batch.items[0].candidate_buffer.count, 0);
	bool clear = true;
	ASSERT_TRUE(checker.checkPath(routePosition(0.f, 300.f), routePosition(10.f, 310.f), clear));
	EXPECT_FALSE(clear);
}

TEST_F(MissionRouteGeofenceTest, DirectShortcutMustClearTheFenceIndependently)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	GeofencePathChecker checker{_fence, routePosition(0.f, 100.f)};
	bool clear = true;
	ASSERT_TRUE(checker.checkPath(routePosition(0.f, 100.f), routePosition(0.f, 500.f), clear));
	EXPECT_FALSE(clear);
	ASSERT_TRUE(checker.checkPath(routePosition(60.f, 100.f), routePosition(60.f, 500.f), clear));
	EXPECT_TRUE(clear);
}

TEST_F(MissionRouteGeofenceTest, LoadedEmptyFenceStillChecksHomeAndAltitudeLimits)
{
	const float maximum_horizontal = 300.f;
	const float maximum_vertical = 150.f;
	ASSERT_EQ(param_set(param_find("GF_MAX_HOR_DIST"), &maximum_horizontal), PX4_OK);
	ASSERT_EQ(param_set(param_find("GF_MAX_VER_DIST"), &maximum_vertical), PX4_OK);
	_navigator.updateParams();
	const Position vehicle = routePosition(0.f, 100.f);
	GeofencePathChecker checker{_fence, vehicle};
	ProjectionReferenceBatch batch{};
	batch.count = 1;
	batch.items[0].position = vehicle;
	addCandidate(batch.items[0], routePosition(0.f, 200.f), 1);
	addCandidate(batch.items[0], routePosition(0.f, 400.f), 2);
	addCandidate(batch.items[0], routePosition(0.f, 200.f, 600.f), 3);

	ASSERT_TRUE(checker.pruneCandidates(batch));
	ASSERT_EQ(batch.items[0].candidate_buffer.count, 1);
	EXPECT_EQ(batch.items[0].candidate_buffer.candidates[0].segment.start.idx, 1);
	bool clear = true;
	ASSERT_TRUE(checker.checkPath(vehicle, routePosition(0.f, 400.f), clear));
	EXPECT_FALSE(clear);
	ASSERT_TRUE(checker.checkPath(vehicle, routePosition(0.f, 200.f, 600.f), clear));
	EXPECT_FALSE(clear);
}

TEST_F(MissionRouteGeofenceTest, ChecksLoadedAltitudeBandAtBothEnds)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	GeofenceTestPeer::setAltitudeBand(_fence, 450.f, 550.f);
	GeofencePathChecker checker{_fence, routePosition(0.f, 0.f)};
	ProjectionReferenceBatch batch{};
	batch.count = 1;
	batch.items[0].position = routePosition(0.f, 100.f);
	addCandidate(batch.items[0], routePosition(0.f, 200.f, 600.f));
	ASSERT_TRUE(checker.pruneCandidates(batch));
	EXPECT_EQ(batch.items[0].candidate_buffer.count, 0);

	batch.items[0].position = routePosition(0.f, 100.f, 400.f);
	addCandidate(batch.items[0], routePosition(0.f, 200.f));
	ASSERT_TRUE(checker.pruneCandidates(batch));
	EXPECT_EQ(batch.items[0].candidate_buffer.count, 0);
}

TEST_F(MissionRouteGeofenceTest, LoadingFenceClearsCandidatesAndDoesNotEnableRecovery)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	_fence.updateFence();
	ASSERT_FALSE(_fence.isReadyForPathChecks());
	GeofencePathChecker checker{_fence, routePosition(0.f, 300.f)};
	ProjectionReferenceBatch batch{};
	batch.count = 1;
	batch.items[0].position = routePosition(0.f, 100.f);
	addCandidate(batch.items[0], routePosition(0.f, 200.f));
	EXPECT_FALSE(checker.pruneCandidates(batch));
	EXPECT_EQ(batch.items[0].candidate_buffer.count, 0);
	bool clear = true;
	EXPECT_FALSE(checker.checkPath(routePosition(0.f, 100.f), routePosition(0.f, 200.f), clear));
	EXPECT_FALSE(clear);
}

TEST_F(MissionRouteGeofenceTest, UnloadedEmptyFenceIsUnavailable)
{
	ASSERT_TRUE(resetFence());
	ASSERT_TRUE(_fence.isEmpty());
	ASSERT_FALSE(_fence.isReadyForPathChecks());
	GeofencePathChecker checker{_fence, routePosition(0.f, 0.f)};
	bool clear = true;
	EXPECT_FALSE(checker.checkPath(routePosition(0.f, 100.f), routePosition(0.f, 200.f), clear));
	EXPECT_FALSE(clear);
}

TEST_F(MissionRouteGeofenceTest, UnavailablePathBatchDiscardsAllCandidates)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	GeofencePathChecker checker{_fence, routePosition(0.f, 0.f)};
	// Keep point membership valid, but make the cached path metadata inconsistent.
	GeofenceTestPeer::setVertexCount(_fence, 3);
	ProjectionReferenceBatch batch{};
	batch.count = 1;
	batch.items[0].position = routePosition(60.f, 100.f);
	addCandidate(batch.items[0], routePosition(60.f, 500.f));
	EXPECT_FALSE(checker.pruneCandidates(batch));
	EXPECT_EQ(batch.items[0].candidate_buffer.count, 0);
}

TEST_F(MissionRouteGeofenceTest, UnavailableTailDiscardsPreviouslyAcceptedPathBatch)
{
	static constexpr uint8_t kReferenceCount = 13;

	if (mission_route::kMaxSafePointBatch < kReferenceCount) {
		GTEST_SKIP() << "requires at least thirteen configured projection batch slots";
	}

	ASSERT_TRUE(loadFence(exclusionSquare()));
	GeofencePathChecker checker{_fence, routePosition(0.f, 0.f)};
	ProjectionReferenceBatch batch{};
	batch.count = kReferenceCount;

	for (uint8_t i = 0; i < batch.count; ++i) {
		batch.items[i].position = routePosition(60.f, 100.f);

		for (uint8_t j = 0; j < mission_route::kMaxSegmentCandidates; ++j) {
			addCandidate(batch.items[i], routePosition(60.f, 500.f));
		}
	}

	// This late segment exceeds the polygon subdivision budget. The first
	// 32 paths succeed, but none may survive the unavailable second chunk.
	batch.items[kReferenceCount - 1].position = {47.0, -80.0, kAltitude};
	batch.items[kReferenceCount - 1].candidate_buffer.candidates[0].projection = {47.0, 80.0, kAltitude};
	EXPECT_FALSE(checker.pruneCandidates(batch));

	for (uint8_t i = 0; i < batch.count; ++i) {
		EXPECT_EQ(batch.items[i].candidate_buffer.count, 0);
	}
}

TEST_F(MissionRouteGeofenceTest, InvalidVehiclePositionDoesNotEnableRecovery)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	GeofencePathChecker checker{_fence, Position{}};
	bool clear = true;
	ASSERT_TRUE(checker.checkPath(routePosition(0.f, 300.f), routePosition(0.f, 100.f), clear));
	EXPECT_FALSE(clear);
}

TEST_F(MissionRouteGeofenceTest, ExistingBreachSkipsOnlyVehicleSegments)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	const Position vehicle = routePosition(0.f, 300.f);
	GeofencePathChecker recovery{_fence, vehicle};
	ProjectionReferenceBatch batch{};
	batch.count = 2;
	batch.items[0].position = vehicle;
	addCandidate(batch.items[0], routePosition(0.f, 100.f));
	// A safe point's branch-off segments are still checked during recovery.
	batch.items[1].position = routePosition(0.f, 100.f);
	batch.items[1].source_index = 0;
	addCandidate(batch.items[1], routePosition(0.f, 500.f), 1);
	addCandidate(batch.items[1], routePosition(100.f, 100.f), 2);
	ASSERT_TRUE(recovery.pruneCandidates(batch));
	EXPECT_EQ(batch.items[0].candidate_buffer.count, 1);
	ASSERT_EQ(batch.items[1].candidate_buffer.count, 1);
	EXPECT_EQ(batch.items[1].candidate_buffer.candidates[0].segment.start.idx, 2);
	bool clear = false;
	ASSERT_TRUE(recovery.checkPath(vehicle, routePosition(0.f, 100.f), clear));
	EXPECT_TRUE(clear);

	// Recovery snapshots never authorize paths through an unavailable new fence.
	_fence.updateFence();
	EXPECT_FALSE(recovery.checkPath(vehicle, routePosition(0.f, 100.f), clear));
	EXPECT_FALSE(clear);
}
