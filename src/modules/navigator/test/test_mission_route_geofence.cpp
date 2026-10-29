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
#include "mission_route_planner.h"
#include "mission_route_projection.h"
#include "support/geofence_test_helpers.h"
#include "support/mission_route_test_helpers.h"

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

	mission_item_s routeItem(float north, float east, uint16_t nav_cmd = NAV_CMD_WAYPOINT) const
	{
		const Position at = routePosition(north, east);
		return makePositionItem(at.lat, at.lon, at.alt, nav_cmd);
	}

	// Use 30 m vehicle and safe-point search margins for these scenarios.
	mission_route::RtlRouteRequest rtlRequest(const Position &vehicle, int32_t mission_index) const
	{
		mission_route::RtlRouteRequest request{};
		request.vehicle_position = vehicle;
		request.mission_index = mission_index;
		request.home_altitude_amsl = kAltitude - 100.f;
		request.projection_search_distance_m = 30.f;
		request.safe_point_projection_search_distance_m = 30.f;
		request.acceptance_radius_m = 10.f;
		request.direct_goal_acceptance_radius_m = 10.f;
		request.altitude_acceptance_radius_m = 10.f;
		return request;
	}

	// Rectangle A(0, 0) B(600, 0) C(600, 200) D(0, 200), with a box beside leg C-D that no leg crosses.
	std::vector<mission_item_s> rectangleMission(uint16_t last_nav_cmd = NAV_CMD_WAYPOINT) const
	{
		return {routeItem(0.f, 0.f), routeItem(600.f, 0.f), routeItem(600.f, 200.f), routeItem(0.f, 200.f, last_nav_cmd)};
	}

	FencePoints boxBesideLegCD() const
	{
		return polygon(false, {{250.f, 160.f}, {350.f, 160.f}, {350.f, 190.f}, {250.f, 190.f}});
	}

	mission_item_s safePointItem(float north, float east) const
	{
		const Position at = routePosition(north, east);
		return makeSafePointAbsolute(at.lat, at.lon, at.alt);
	}

	// A thin wall 4 m east of a safe point at (300, 100): it blocks the legs east of the safe point.
	FencePoints wallEastOfSafePoint() const
	{
		return polygon(false, {{290.f, 104.f}, {310.f, 104.f}, {310.f, 106.f}, {290.f, 106.f}});
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
	batch.items[0].source_index = 0;
	addCandidate(batch.items[0], routePosition(0.f, 200.f, 600.f));
	ASSERT_TRUE(checker.pruneCandidates(batch));
	EXPECT_EQ(batch.items[0].candidate_buffer.count, 0);

	batch.items[0].position = routePosition(0.f, 100.f, 400.f);
	addCandidate(batch.items[0], routePosition(0.f, 200.f));
	ASSERT_TRUE(checker.pruneCandidates(batch));
	EXPECT_EQ(batch.items[0].candidate_buffer.count, 0);

	// A vehicle below the band does not recover: its segments may start there, but must end inside the band.
	const Position low_vehicle = routePosition(0.f, 100.f, 400.f);
	const GeofencePathChecker vehicle_checker{_fence, low_vehicle};
	batch.items[0] = {};
	batch.items[0].position = low_vehicle;
	addCandidate(batch.items[0], routePosition(0.f, 200.f), 1);
	addCandidate(batch.items[0], routePosition(0.f, 200.f, 600.f), 2);
	ASSERT_TRUE(vehicle_checker.pruneCandidates(batch));
	ASSERT_EQ(batch.items[0].candidate_buffer.count, 1);
	EXPECT_EQ(batch.items[0].candidate_buffer.candidates[0].segment.start.idx, 1);
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

// The rescan checks a reference once and then each of its candidates: together they must agree with the prune,
// for every fence type and limit, and recovery exempts only the vehicle.
TEST_F(MissionRouteGeofenceTest, ReferenceAndCandidateChecksAgreeWithPrune)
{
	struct FenceCase {
		const char *name;
		FencePoints points;
		float max_horizontal_distance;
	};

	const FenceCase fence_cases[] {
		{"exclusion polygon", exclusionSquare(), 0.f},
		{"inclusion polygon", polygon(true, {{-250.f, -250.f}, {250.f, -250.f}, {250.f, 400.f}, {-250.f, 400.f}}), 0.f},
		{"exclusion circle", circle(false, {0.f, 300.f}, 60.f), 0.f},
		{"inclusion circle", circle(true, {0.f, 150.f}, 300.f), 0.f},
		{"maximum distance", {}, 400.f},
	};
	// Each fence case has spots outside the allowed area. The vehicle recovers there, except beyond only the
	// maximum distance, where its segments are still checked.
	const Position spots[] {routePosition(0.f, 100.f), routePosition(100.f, 100.f), routePosition(0.f, 300.f),
				routePosition(0.f, 500.f), routePosition(-300.f, 0.f)
			       };
	unsigned clear_count = 0;
	unsigned blocked_count = 0;

	for (const FenceCase &fence_case : fence_cases) {
		SCOPED_TRACE(fence_case.name);
		ASSERT_EQ(param_set(param_find("GF_MAX_HOR_DIST"), &fence_case.max_horizontal_distance), PX4_OK);
		_navigator.updateParams();
		ASSERT_TRUE(loadFence(fence_case.points));

		for (size_t spot = 0; spot < 5; ++spot) {
			SCOPED_TRACE(::testing::Message() << "vehicle at spot " << spot);
			const GeofencePathChecker checker{_fence, spots[spot]};
			ProjectionReferenceBatch batch{};
			batch.count = 2;
			batch.items[0].position = spots[spot];
			batch.items[1].position = spots[(spot + 1) % 5];
			batch.items[1].source_index = 0;
			bool clear[2][3] {};

			for (uint8_t i = 0; i < batch.count; ++i) {
				bool reference_clear = false;
				ASSERT_TRUE(checker.checkReference(batch.items[i], reference_clear));

				for (uint8_t j = 0; j < 3; ++j) {
					addCandidate(batch.items[i], spots[(spot + 2 + j) % 5], j);
					bool candidate_clear = false;
					ASSERT_TRUE(checker.checkCandidate(batch.items[i], batch.items[i].candidate_buffer.candidates[j],
									   candidate_clear));
					clear[i][j] = reference_clear && candidate_clear;
					clear_count += clear[i][j];
					blocked_count += !clear[i][j];
				}
			}

			ASSERT_TRUE(checker.pruneCandidates(batch));

			for (uint8_t i = 0; i < batch.count; ++i) {
				const auto &candidates = batch.items[i].candidate_buffer;
				bool kept[3] {};

				for (uint8_t k = 0; k < candidates.count; ++k) {
					kept[candidates.candidates[k].segment.start.idx] = true;
				}

				for (uint8_t j = 0; j < 3; ++j) {
					EXPECT_EQ(kept[j], clear[i][j]) << "reference " << int(i) << ", candidate " << int(j);
				}
			}
		}
	}

	EXPECT_GT(clear_count, 0u);
	EXPECT_GT(blocked_count, 0u);
}

// Leg C-D is the safe point's closest leg, but its branch-off leg crosses the box. It must not narrow the
// window and hide leg A-B, clear 150 m away: the plan failed, or with a LAND at D landed there.
TEST_F(MissionRouteGeofenceTest, BlockedClosestLegDoesNotHideClearSafePointBranchOff)
{
	ASSERT_TRUE(loadFence(boxBesideLegCD()));
	const Position vehicle = routePosition(100.f, 0.f);
	const GeofencePathChecker checker{_fence, vehicle};

	for (const bool land_at_d : {false, true}) {
		SCOPED_TRACE(::testing::Message() << "land at D " << land_at_d);
		const auto provider = makeRouteProvider(rectangleMission(land_at_d ? NAV_CMD_LAND : NAV_CMD_WAYPOINT),
		{safePointItem(300.f, 150.f)});
		MissionRoutePlanner planner{provider, &checker};
		mission_route::RtlRouteRequest request = rtlRequest(vehicle, 1);
		request.mission_land_index = land_at_d ? 3 : -1;
		mission_route::RtlRoutePlan plan{};

		const mission_route::FailureReason status = planner.planRtlRoute(request, plan);

		ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
		EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
		EXPECT_EQ(plan.safe_point_index, 0);
		EXPECT_FALSE(plan.direction_reversed);
		EXPECT_EQ(plan.branch_off_mission_item_index, 1);
	}
}

// After a GoTo, the vehicle's closest leg is C-D behind the box. It must not narrow the window and hide leg A-B,
// clear 150 m away: smart rejoin and Return both had no route.
TEST_F(MissionRouteGeofenceTest, BlockedClosestLegDoesNotHideClearBranchIn)
{
	ASSERT_TRUE(loadFence(boxBesideLegCD()));
	const Position vehicle = routePosition(300.f, 150.f);
	const GeofencePathChecker checker{_fence, vehicle};
	const auto provider = makeRouteProvider(rectangleMission(), {safePointItem(0.f, -50.f)});
	MissionRoutePlanner planner{provider, &checker};
	const mission_route::RtlRouteRequest request = rtlRequest(vehicle, 1);

	mission_route::MissionResumePlan join{};
	const mission_route::FailureReason join_status = planner.planMissionResumeJoin(request, join);
	ASSERT_EQ(join_status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(join_status);
	EXPECT_EQ(join.first_mission_item_index, 1);

	mission_route::RtlRoutePlan plan{};
	const mission_route::FailureReason status = planner.planRtlRoute(request, plan);
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_TRUE(plan.direction_reversed);
	EXPECT_EQ(plan.first_mission_item_index, 0);
	EXPECT_EQ(plan.branch_off_mission_item_index, 0);
}

// An altitude breach alone does not exempt the vehicle's segments from the polygons: above GF_MAX_VER_DIST,
// the join and Return used to cross the box to leg C-D. Next to a landing, the join still lands there.
TEST_F(MissionRouteGeofenceTest, AltitudeBreachKeepsPolygonChecks)
{
	const float maximum_vertical = 150.f;
	ASSERT_EQ(param_set(param_find("GF_MAX_VER_DIST"), &maximum_vertical), PX4_OK);
	_navigator.updateParams();
	ASSERT_TRUE(loadFence(boxBesideLegCD()));
	const float breach_altitude = kAltitude + 60.f; // 160 m above Home
	ASSERT_FALSE(_fence.isBelowMaxAltitude(breach_altitude));

	const Position vehicle = routePosition(300.f, 150.f, breach_altitude);
	const GeofencePathChecker checker{_fence, vehicle};
	const auto provider = makeRouteProvider(rectangleMission(), {safePointItem(0.f, -50.f)});
	MissionRoutePlanner planner{provider, &checker};
	mission_route::MissionResumePlan join{};
	mission_route::FailureReason status = planner.planMissionResumeJoin(rtlRequest(vehicle, 1), join);
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(join.first_mission_item_index, 1);

	mission_route::RtlRoutePlan plan{};
	status = planner.planRtlRoute(rtlRequest(vehicle, 1), plan);
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_TRUE(plan.direction_reversed);
	EXPECT_EQ(plan.first_mission_item_index, 0);
	EXPECT_EQ(plan.branch_off_mission_item_index, 0);

	const Position above_land = routePosition(3.f, 198.f, breach_altitude);
	const GeofencePathChecker land_checker{_fence, above_land};
	const auto land_provider = makeRouteProvider(rectangleMission(NAV_CMD_LAND), {});
	MissionRoutePlanner land_planner{land_provider, &land_checker};
	status = land_planner.planMissionResumeJoin(rtlRequest(above_land, 3), join);
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_TRUE(join.use_current_altitude);
	EXPECT_EQ(join.first_mission_item_index, 3);
}

// The closest leg is blocked, but a leg 30 m away passes inside the first window. The rescan must widen the window
// to the leg the vehicle flies, 50 m away and cheaper; rescanning only empty buffers would miss it.
TEST_F(MissionRouteGeofenceTest, BlockedClosestLegDoesNotHideCheaperLegBeyondSurvivor)
{
	ASSERT_TRUE(loadFence(wallEastOfSafePoint()));
	const std::vector<mission_item_s> mission{
		routeItem(0.f, 50.f), routeItem(600.f, 50.f),   // flown, 50 m from the safe point
		routeItem(600.f, 2000.f), routeItem(0.f, 2000.f),
		routeItem(0.f, 70.f), routeItem(600.f, 70.f),   // 30 m
		routeItem(600.f, 110.f), routeItem(0.f, 110.f), // 10 m, behind the wall
	};
	const Position vehicle = routePosition(100.f, 45.f);
	const GeofencePathChecker checker{_fence, vehicle};
	const auto provider = makeRouteProvider(mission, {safePointItem(300.f, 100.f)});
	MissionRoutePlanner planner{provider, &checker};
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(rtlRequest(vehicle, 1), plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_FALSE(plan.direction_reversed);
	EXPECT_EQ(plan.branch_off_mission_item_index, 1);
}

// The closest leg passes, but two blocked legs fill the three places and push out the leg the vehicle flies,
// 25 m away and cheaper. A full buffer that loses candidates to the prune is rescanned, which finds it again.
TEST_F(MissionRouteGeofenceTest, BlockedLegsInFullBufferDoNotPushOutCheaperLeg)
{
	ASSERT_TRUE(loadFence(wallEastOfSafePoint()));
	const std::vector<mission_item_s> mission{
		routeItem(0.f, 75.f), routeItem(600.f, 75.f),   // flown, 25 m from the safe point
		routeItem(600.f, 2000.f), routeItem(0.f, 2000.f),
		routeItem(0.f, 90.f), routeItem(600.f, 90.f),   // 10 m
		routeItem(600.f, 115.f), routeItem(0.f, 115.f), // 15 m, behind the wall
		routeItem(0.f, 120.f), routeItem(600.f, 120.f), // 20 m, behind the wall
	};
	const Position vehicle = routePosition(100.f, 75.f);
	const GeofencePathChecker checker{_fence, vehicle};
	const auto provider = makeRouteProvider(mission, {safePointItem(300.f, 100.f)});
	MissionRoutePlanner planner{provider, &checker};
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(rtlRequest(vehicle, 1), plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_FALSE(plan.direction_reversed);
	EXPECT_EQ(plan.branch_off_mission_item_index, 1);
}
