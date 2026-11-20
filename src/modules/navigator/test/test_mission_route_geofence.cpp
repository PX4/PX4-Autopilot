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

	void setMargin(float margin)
	{
		ASSERT_EQ(param_set(param_find("GF_CONT_H_MARGIN"), &margin), PX4_OK);
		_navigator.updateParams();
	}

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
	GeofencePathChecker checker{_fence};
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

	ASSERT_TRUE(loadFence(exclusionSquare()));
	GeofencePathChecker checker{_fence};

	// With a margin, the check of each vehicle takes the next free path of the batch, also after a full batch.
	for (const float margin : {0.f, 40.f}) {
		SCOPED_TRACE(margin);
		setMargin(margin);
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

			// The segments at 60 m north pass 10 m from the square: clear without a margin only.
			if (i % 2 == 0 || margin > 0.f) {
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
}

TEST_F(MissionRouteGeofenceTest, RejectsSafePointWhollyInsideExclusionPolygon)
{
	for (const FenceLevel level : {FenceLevel::Emergency, FenceLevel::Contingency}) {
		ASSERT_TRUE(loadFence(exclusionSquare(level)));
		GeofencePathChecker checker{_fence};
		ProjectionReferenceBatch batch{};
		batch.count = 1;
		batch.items[0].position = routePosition(0.f, 300.f);
		batch.items[0].source_index = 0;
		addCandidate(batch.items[0], routePosition(10.f, 310.f));

		ASSERT_TRUE(checker.pruneCandidates(batch));
		EXPECT_EQ(batch.items[0].candidate_buffer.count, 0);
	}
}

TEST_F(MissionRouteGeofenceTest, DirectShortcutMustClearTheFenceIndependently)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	GeofencePathChecker checker{_fence};
	bool clear = true;
	ASSERT_TRUE(checker.checkPath(routePosition(0.f, 100.f), routePosition(0.f, 500.f), clear));
	EXPECT_FALSE(clear);
	ASSERT_TRUE(checker.checkPath(routePosition(60.f, 100.f), routePosition(60.f, 500.f), clear));
	EXPECT_TRUE(clear);
}

// A safe point below the loaded altitude band keeps no candidates.
TEST_F(MissionRouteGeofenceTest, SafePointBelowAltitudeBandKeepsNoCandidates)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	GeofenceTestPeer::setAltitudeBand(_fence, 450.f, 550.f);
	GeofencePathChecker checker{_fence};
	ProjectionReferenceBatch batch{};
	batch.count = 1;
	batch.items[0].position = routePosition(0.f, 100.f, 400.f);
	batch.items[0].source_index = 0;
	addCandidate(batch.items[0], routePosition(0.f, 200.f));
	ASSERT_TRUE(checker.pruneCandidates(batch));
	EXPECT_EQ(batch.items[0].candidate_buffer.count, 0);
}

// A safe point beyond GF_MAX_HOR_DIST keeps no candidates.
TEST_F(MissionRouteGeofenceTest, SafePointBeyondHomeDistanceKeepsNoCandidates)
{
	const float maximum_horizontal = 300.f;
	ASSERT_EQ(param_set(param_find("GF_MAX_HOR_DIST"), &maximum_horizontal), PX4_OK);
	_navigator.updateParams();
	GeofencePathChecker checker{_fence};
	ProjectionReferenceBatch batch{};
	batch.count = 1;
	batch.items[0].position = routePosition(0.f, 400.f);
	batch.items[0].source_index = 0;
	addCandidate(batch.items[0], routePosition(0.f, 200.f));
	ASSERT_TRUE(checker.pruneCandidates(batch));
	EXPECT_EQ(batch.items[0].candidate_buffer.count, 0);
}

// The distance from Home never blocks a segment from the vehicle, with or without the margin. Example: the
// Return that GF_ACTION starts beyond GF_MAX_HOR_DIST still gets its way back.
TEST_F(MissionRouteGeofenceTest, VehicleBeyondHomeDistanceKeepsItsCandidates)
{
	const float maximum_horizontal = 300.f;
	ASSERT_EQ(param_set(param_find("GF_MAX_HOR_DIST"), &maximum_horizontal), PX4_OK);
	_navigator.updateParams();
	setMargin(40.f);
	const Position vehicle = routePosition(0.f, 400.f);
	const Position inside = routePosition(0.f, 200.f);
	ASSERT_FALSE(_fence.isCloserThanMaxDistToHome(vehicle.lat, vehicle.lon, vehicle.alt));
	GeofencePathChecker checker{_fence};
	ProjectionReferenceBatch batch{};
	batch.count = 1;
	batch.items[0].position = vehicle;
	addCandidate(batch.items[0], inside);
	ASSERT_TRUE(checker.pruneCandidates(batch));
	EXPECT_EQ(batch.items[0].candidate_buffer.count, 1);

	bool clear = false;
	ASSERT_TRUE(checker.checkPath(vehicle, inside, clear));
	EXPECT_TRUE(clear);
}

// A loading fence is unavailable, also for a vehicle in breach.
TEST_F(MissionRouteGeofenceTest, LoadingFenceIsUnavailable)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	_fence.updateFence();
	ASSERT_FALSE(_fence.isReadyForPathChecks());
	GeofencePathChecker checker{_fence};
	const Position vehicle = routePosition(0.f, 300.f);
	ProjectionReferenceBatch batch{};
	batch.count = 1;
	batch.items[0].position = vehicle;
	addCandidate(batch.items[0], routePosition(0.f, 100.f));
	EXPECT_FALSE(checker.pruneCandidates(batch));
	bool clear = true;
	EXPECT_FALSE(checker.checkPath(vehicle, routePosition(0.f, 100.f), clear));
	EXPECT_FALSE(clear);
}

// A fence that never loaded is unavailable too, although it is empty.
TEST_F(MissionRouteGeofenceTest, UnloadedEmptyFenceIsUnavailable)
{
	ASSERT_TRUE(resetFence());
	ASSERT_TRUE(_fence.isEmpty());
	ASSERT_FALSE(_fence.isReadyForPathChecks());
	GeofencePathChecker checker{_fence};
	ProjectionReferenceBatch batch{};
	batch.count = 1;
	batch.items[0].position = routePosition(0.f, 100.f);
	addCandidate(batch.items[0], routePosition(0.f, 200.f));
	EXPECT_FALSE(checker.pruneCandidates(batch));
	bool clear = true;
	EXPECT_FALSE(checker.checkPath(routePosition(0.f, 100.f), routePosition(0.f, 200.f), clear));
	EXPECT_FALSE(clear);
}

TEST_F(MissionRouteGeofenceTest, UnavailableTailFailsThePrune)
{
	static constexpr uint8_t kReferenceCount = 13;

	ASSERT_TRUE(loadFence(exclusionSquare()));
	GeofencePathChecker checker{_fence};
	ProjectionReferenceBatch batch{};
	batch.count = kReferenceCount;

	for (uint8_t i = 0; i < batch.count; ++i) {
		batch.items[i].position = routePosition(60.f, 100.f);

		for (uint8_t j = 0; j < mission_route::kMaxSegmentCandidates; ++j) {
			addCandidate(batch.items[i], routePosition(60.f, 500.f));
		}
	}

	// This late segment exceeds the polygon subdivision budget. The first
	// 32 paths succeed, but the unavailable second chunk fails the whole prune.
	batch.items[kReferenceCount - 1].position = {47.0, -80.0, kAltitude};
	batch.items[kReferenceCount - 1].candidate_buffer.candidates[0].projection = {47.0, 80.0, kAltitude};
	EXPECT_FALSE(checker.pruneCandidates(batch));
}

// An invalid position does not recover: neither it nor a path to it is clear.
TEST_F(MissionRouteGeofenceTest, InvalidPositionIsNotClear)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	GeofencePathChecker checker{_fence};
	bool clear = true;
	ASSERT_TRUE(checker.checkPath(Position{}, routePosition(0.f, 100.f), clear));
	EXPECT_FALSE(clear);
	clear = true;
	ASSERT_TRUE(checker.checkPath(routePosition(0.f, 100.f), Position{}, clear));
	EXPECT_FALSE(clear);
}

TEST_F(MissionRouteGeofenceTest, ExistingBreachSkipsOnlyVehicleSegments)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	const Position vehicle = routePosition(0.f, 300.f);
	GeofencePathChecker recovery{_fence};
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

	// Recovery never authorizes a path while the fence loads.
	_fence.updateFence();
	EXPECT_FALSE(recovery.checkPath(vehicle, routePosition(0.f, 100.f), clear));
	EXPECT_FALSE(clear);
}

// The prune and checkPath() apply the same rules to a segment from the vehicle, for every fence type,
// also while the vehicle recovers.
TEST_F(MissionRouteGeofenceTest, VehicleCandidatesInPruneAgreeWithCheckPath)
{
	struct FenceCase {
		const char *name;
		FencePoints points;
		float margin;
	};

	// With the margin, each fence case also has spots closer to the fence than that.
	const FencePoints inclusion = polygon(true, {{-250.f, -250.f}, {250.f, -250.f}, {250.f, 400.f}, {-250.f, 400.f}});
	const FenceCase fence_cases[] {
		{"exclusion polygon", exclusionSquare(), 0.f},
		{"inclusion polygon", inclusion, 0.f},
		{"exclusion circle", circle(false, {0.f, 300.f}, 60.f), 0.f},
		{"inclusion circle", circle(true, {0.f, 150.f}, 300.f), 0.f},
		{"exclusion polygon with a margin", exclusionSquare(), 160.f},
		{"inclusion polygon with a margin", inclusion, 160.f},
		{"exclusion circle with a margin", circle(false, {0.f, 300.f}, 60.f), 160.f},
		{"inclusion circle with a margin", circle(true, {0.f, 150.f}, 300.f), 160.f},
	};
	// Each fence case has spots outside the allowed area, where the vehicle recovers.
	const Position spots[] {routePosition(0.f, 100.f), routePosition(100.f, 100.f), routePosition(0.f, 300.f),
				routePosition(0.f, 500.f), routePosition(-300.f, 0.f)
			       };
	unsigned clear_count = 0;
	unsigned blocked_count = 0;

	for (const FenceCase &fence_case : fence_cases) {
		SCOPED_TRACE(fence_case.name);
		ASSERT_TRUE(loadFence(fence_case.points));
		setMargin(fence_case.margin);

		for (size_t spot = 0; spot < 5; ++spot) {
			SCOPED_TRACE(::testing::Message() << "vehicle at spot " << spot);
			const GeofencePathChecker checker{_fence};
			ProjectionReferenceBatch batch{};
			batch.count = 1;
			batch.items[0].position = spots[spot];
			bool clear[3] {};

			for (uint8_t j = 0; j < 3; ++j) {
				const Position &end = spots[(spot + 2 + j) % 5];
				addCandidate(batch.items[0], end, j);
				ASSERT_TRUE(checker.checkPath(spots[spot], end, clear[j]));
				clear_count += clear[j];
				blocked_count += !clear[j];
			}

			ASSERT_TRUE(checker.pruneCandidates(batch));
			const auto &candidates = batch.items[0].candidate_buffer;
			bool kept[3] {};

			for (uint8_t k = 0; k < candidates.count; ++k) {
				kept[candidates.candidates[k].segment.start.idx] = true;
			}

			for (uint8_t j = 0; j < 3; ++j) {
				EXPECT_EQ(kept[j], clear[j]) << "candidate " << int(j);
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
	const GeofencePathChecker checker{_fence};

	for (const bool land_at_d : {false, true}) {
		SCOPED_TRACE(::testing::Message() << "land at D " << land_at_d);
		const auto provider = makeRouteProvider(rectangleMission(land_at_d ? NAV_CMD_LAND : NAV_CMD_WAYPOINT),
		{safePointItem(300.f, 150.f)});
		MissionRoutePlanner planner{provider, &checker};
		mission_route::RtlRouteRequest request = rtlRequest(vehicle, 1);
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
	const GeofencePathChecker checker{_fence};
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
	const GeofencePathChecker checker{_fence};
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
	const auto land_provider = makeRouteProvider(rectangleMission(NAV_CMD_LAND), {});
	MissionRoutePlanner land_planner{land_provider, &checker};
	status = land_planner.planMissionResumeJoin(rtlRequest(above_land, 3), join);
	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_TRUE(join.use_current_altitude);
	EXPECT_EQ(join.first_mission_item_index, 3);
}

// A safe point above GF_MAX_VER_DIST loses its candidates in the prune: Return picks the farther one inside the limits.
TEST_F(MissionRouteGeofenceTest, SafePointOutsideLimitsIsNotSelected)
{
	const float maximum_vertical = 150.f;
	ASSERT_EQ(param_set(param_find("GF_MAX_VER_DIST"), &maximum_vertical), PX4_OK);
	_navigator.updateParams();
	const Position vehicle = routePosition(100.f, 0.f);
	const GeofencePathChecker checker{_fence};
	mission_item_s high_safe_point = safePointItem(300.f, 20.f);
	high_safe_point.altitude = kAltitude + 60.f; // 160 m above Home
	const auto provider = makeRouteProvider(rectangleMission(), {high_safe_point, safePointItem(500.f, 20.f)});
	MissionRoutePlanner planner{provider, &checker};
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(rtlRequest(vehicle, 1), plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(plan.safe_point_index, 1);
}

// The closest leg is blocked, and a leg 30 m away passes. The window comes from that leg, so it reaches the leg
// the vehicle flies, 50 m away and cheaper.
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
	const GeofencePathChecker checker{_fence};
	const auto provider = makeRouteProvider(mission, {safePointItem(300.f, 100.f)});
	MissionRoutePlanner planner{provider, &checker};
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(rtlRequest(vehicle, 1), plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_FALSE(plan.direction_reversed);
	EXPECT_EQ(plan.branch_off_mission_item_index, 1);
}

// The closest leg passes, and two blocked legs take two of the four places. The leg the vehicle flies, 25 m away
// and cheaper, keeps the fourth.
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
	const GeofencePathChecker checker{_fence};
	const auto provider = makeRouteProvider(mission, {safePointItem(300.f, 100.f)});
	MissionRoutePlanner planner{provider, &checker};
	mission_route::RtlRoutePlan plan{};

	const mission_route::FailureReason status = planner.planRtlRoute(rtlRequest(vehicle, 1), plan);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_FALSE(plan.direction_reversed);
	EXPECT_EQ(plan.branch_off_mission_item_index, 1);
}

// The documented limit: four blocked legs take the four places, so the clear leg the vehicle flies, 30 m away,
// is never kept. The safe point has no branch-off and, without a mission landing, Return has no route.
TEST_F(MissionRouteGeofenceTest, FourBlockedClosestLegsHideClearLeg)
{
	ASSERT_TRUE(loadFence(wallEastOfSafePoint()));
	const std::vector<mission_item_s> mission{
		routeItem(0.f, 70.f), routeItem(600.f, 70.f),   // flown, 30 m from the safe point
		routeItem(600.f, 2000.f), routeItem(0.f, 2000.f),
		routeItem(0.f, 110.f), routeItem(600.f, 110.f), // 10 m, behind the wall
		routeItem(600.f, 115.f), routeItem(0.f, 115.f), // 15 m, behind the wall
		routeItem(0.f, 120.f), routeItem(600.f, 120.f), // 20 m, behind the wall
		routeItem(600.f, 125.f), routeItem(0.f, 125.f), // 25 m, behind the wall
	};
	const Position vehicle = routePosition(100.f, 70.f);
	const GeofencePathChecker checker{_fence};
	const auto provider = makeRouteProvider(mission, {safePointItem(300.f, 100.f)});
	MissionRoutePlanner planner{provider, &checker};
	mission_route::RtlRoutePlan plan{};

	EXPECT_EQ(planner.planRtlRoute(rtlRequest(vehicle, 1), plan), mission_route::FailureReason::kNoValidCandidateFound);
}

// Within the acceptance radius of LAND, the vehicle keeps its altitude and may fly straight to it. That hop crosses
// a wall that no pruned segment touches, so neither Return nor the mission join may use it.
TEST_F(MissionRouteGeofenceTest, CurrentAltitudeHopToLandingIsChecked)
{
	ASSERT_TRUE(loadFence(polygon(false, {{597.f, 1.f}, {598.f, 1.f}, {598.f, 10.f}, {597.f, 10.f}})));
	const Position vehicle = routePosition(595.f, 5.f);
	const GeofencePathChecker checker{_fence};
	const std::vector<mission_item_s> mission{routeItem(0.f, 0.f), routeItem(600.f, 0.f, NAV_CMD_LAND)};
	const auto provider = makeRouteProvider(mission, {safePointItem(700.f, 0.f)});
	MissionRoutePlanner planner{provider, &checker};

	mission_route::RtlRoutePlan plan{};
	EXPECT_EQ(planner.planRtlRoute(rtlRequest(vehicle, 1), plan), mission_route::FailureReason::kNoValidPath);
	EXPECT_FALSE(plan.valid());

	mission_route::MissionResumePlan join{};
	EXPECT_EQ(planner.planMissionResumeJoin(rtlRequest(vehicle, 1), join), mission_route::FailureReason::kNoValidPath);
	EXPECT_FALSE(join.valid());
}

// A segment keeps GF_CONT_H_MARGIN from the fence, like a mission leg. Both segments pass north of the exclusion
// square: one 50 m from it, the other 25 m.
TEST_F(MissionRouteGeofenceTest, SegmentsKeepTheContingencyMargin)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	const Position vehicle = routePosition(100.f, 100.f);
	const Position far_end = routePosition(100.f, 500.f);
	const Position near_end = routePosition(60.f, 500.f);
	GeofencePathChecker checker{_fence};

	for (const float margin : {0.f, 40.f}) {
		SCOPED_TRACE(margin);
		setMargin(margin);
		ProjectionReferenceBatch batch{};
		batch.count = 1;
		batch.items[0].position = vehicle;
		addCandidate(batch.items[0], far_end, 1);
		addCandidate(batch.items[0], near_end, 2);
		ASSERT_TRUE(checker.pruneCandidates(batch));
		ASSERT_EQ(batch.items[0].candidate_buffer.count, margin > 0.f ? 1 : 2);
		EXPECT_EQ(batch.items[0].candidate_buffer.candidates[0].segment.start.idx, 1);

		bool clear = false;
		ASSERT_TRUE(checker.checkPath(vehicle, far_end, clear));
		EXPECT_TRUE(clear);
		ASSERT_TRUE(checker.checkPath(vehicle, near_end, clear));
		EXPECT_EQ(clear, margin <= 0.f);
	}
}

// Drawn contingency areas replace the margin for the planner's segments, as for a mission. The segment passes
// 25 m north of the exclusion square and stays well inside the drawn circle.
TEST_F(MissionRouteGeofenceTest, DrawnContingencyAreasReplaceTheMargin)
{
	FencePoints points = exclusionSquare();
	const FencePoints area = circle(true, {0.f, 300.f}, 400.f, FenceLevel::Contingency);
	points.insert(points.end(), area.begin(), area.end());
	ASSERT_TRUE(loadFence(points));
	setMargin(40.f);
	const Position vehicle = routePosition(100.f, 100.f);
	const Position near_end = routePosition(60.f, 500.f);
	GeofencePathChecker checker{_fence};
	ProjectionReferenceBatch batch{};
	batch.count = 1;
	batch.items[0].position = vehicle;
	addCandidate(batch.items[0], near_end);
	ASSERT_TRUE(checker.pruneCandidates(batch));
	EXPECT_EQ(batch.items[0].candidate_buffer.count, 1);

	bool clear = false;
	ASSERT_TRUE(checker.checkPath(vehicle, near_end, clear));
	EXPECT_TRUE(clear);
}

// A vehicle outside a drawn contingency area but inside the emergency geofence is handled like one inside the
// margin: its segments may lead back in or stay in between, but not across the emergency geofence.
TEST_F(MissionRouteGeofenceTest, VehicleOutsideADrawnContingencyAreaIsCheckedAgainstTheEmergencyGeofence)
{
	// The emergency geofence reaches 500 m from Home, the contingency geofence 300 m.
	const FencePoints squares[] = {
		polygon(true, {{-500.f, -500.f}, {500.f, -500.f}, {500.f, 500.f}, {-500.f, 500.f}}),
		polygon(true, {{-300.f, -300.f}, {300.f, -300.f}, {300.f, 300.f}, {-300.f, 300.f}}, FenceLevel::Contingency)
	};
	const FencePoints circles[] = {circle(true, {0.f, 0.f}, 500.f), circle(true, {0.f, 0.f}, 300.f, FenceLevel::Contingency)};
	const Position vehicle = routePosition(0.f, 400.f);
	const Position back = routePosition(0.f, 100.f); // inside the contingency geofence
	const Position between = routePosition(200.f, 400.f); // between the two geofences, like the vehicle
	const Position across = routePosition(0.f, 700.f); // outside the emergency geofence

	for (const FencePoints *fences : {squares, circles}) {
		SCOPED_TRACE(fences == squares ? "polygons" : "circles");
		FencePoints points = fences[0];
		points.insert(points.end(), fences[1].begin(), fences[1].end());
		ASSERT_TRUE(loadFence(points));
		GeofencePathChecker checker{_fence};
		ProjectionReferenceBatch batch{};
		batch.count = 1;
		batch.items[0].position = vehicle;
		addCandidate(batch.items[0], across, 1);
		addCandidate(batch.items[0], back, 2);
		addCandidate(batch.items[0], between, 3);
		ASSERT_TRUE(checker.pruneCandidates(batch));
		ASSERT_EQ(batch.items[0].candidate_buffer.count, 2);
		EXPECT_EQ(batch.items[0].candidate_buffer.candidates[0].segment.start.idx, 2);
		EXPECT_EQ(batch.items[0].candidate_buffer.candidates[1].segment.start.idx, 3);

		bool clear = false;
		ASSERT_TRUE(checker.checkPath(vehicle, back, clear));
		EXPECT_TRUE(clear);
		ASSERT_TRUE(checker.checkPath(vehicle, across, clear));
		EXPECT_FALSE(clear);

		// A vehicle inside the contingency geofence has to stay inside it.
		const Position inside = routePosition(0.f, 200.f);
		ASSERT_TRUE(checker.checkPath(inside, back, clear));
		EXPECT_TRUE(clear);
		ASSERT_TRUE(checker.checkPath(inside, between, clear));
		EXPECT_FALSE(clear);

		// Only a vehicle outside the emergency geofence recovers unchecked.
		ASSERT_TRUE(checker.checkPath(across, routePosition(0.f, 900.f), clear));
		EXPECT_TRUE(clear);

		// A safe point in the same batch still has to keep the contingency geofence, in either order.
		for (const bool vehicle_first : {true, false}) {
			SCOPED_TRACE(vehicle_first);
			ProjectionReferenceBatch mixed{};
			mixed.count = 2;
			ProjectionReference &vehicle_reference = mixed.items[vehicle_first ? 0 : 1];
			ProjectionReference &safe_point = mixed.items[vehicle_first ? 1 : 0];
			vehicle_reference.position = vehicle;
			addCandidate(vehicle_reference, back, 1);
			safe_point.position = inside;
			safe_point.source_index = 0;
			addCandidate(safe_point, between, 2);
			addCandidate(safe_point, back, 3);
			ASSERT_TRUE(checker.pruneCandidates(mixed));
			EXPECT_EQ(vehicle_reference.candidate_buffer.count, 1);
			ASSERT_EQ(safe_point.candidate_buffer.count, 1);
			EXPECT_EQ(safe_point.candidate_buffer.candidates[0].segment.start.idx, 3);
		}
	}
}

// A safe point closer to the fence than the margin keeps no candidates, also for GF_MAX_HOR_DIST.
TEST_F(MissionRouteGeofenceTest, SafePointWithinTheContingencyMarginKeepsNoCandidates)
{
	const auto candidates_kept = [this](const Position & anchor, const Position & projection, int32_t source_index = 0) {
		GeofencePathChecker checker{_fence};
		ProjectionReferenceBatch batch{};
		batch.count = 1;
		batch.items[0].position = anchor;
		batch.items[0].source_index = source_index;
		addCandidate(batch.items[0], projection);
		EXPECT_TRUE(checker.pruneCandidates(batch));
		return batch.items[0].candidate_buffer.count;
	};

	// 30 m west of the exclusion square, with a segment that leads away from it.
	ASSERT_TRUE(loadFence(exclusionSquare()));
	EXPECT_EQ(candidates_kept(routePosition(0.f, 220.f), routePosition(0.f, 100.f)), 1);
	setMargin(40.f);
	EXPECT_EQ(candidates_kept(routePosition(0.f, 220.f), routePosition(0.f, 100.f)), 0);
	EXPECT_EQ(candidates_kept(routePosition(0.f, 200.f), routePosition(0.f, 100.f)), 1);

	// 280 m from Home with a limit of 300 m.
	ASSERT_TRUE(loadFence({}));
	const float maximum_horizontal = 300.f;
	ASSERT_EQ(param_set(param_find("GF_MAX_HOR_DIST"), &maximum_horizontal), PX4_OK);
	_navigator.updateParams();
	EXPECT_EQ(candidates_kept(routePosition(0.f, 280.f), routePosition(0.f, 200.f)), 0);
	EXPECT_EQ(candidates_kept(routePosition(0.f, 250.f), routePosition(0.f, 200.f)), 1);
	// A vehicle there keeps its candidate: the Return that GF_CONT_ACTION starts at the limit needs a route.
	EXPECT_EQ(candidates_kept(routePosition(0.f, 280.f), routePosition(0.f, 200.f), -1), 1);
}

// A vehicle closer to the fence than the margin cannot keep it: its segments are checked against the fence itself.
// The Return that GF_CONT_ACTION starts there gets a route, but not one through the exclusion square.
TEST_F(MissionRouteGeofenceTest, VehicleWithinTheContingencyMarginIsCheckedAgainstTheFence)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	setMargin(40.f);
	const Position vehicle = routePosition(0.f, 220.f); // 30 m west of the square
	const Position away = routePosition(0.f, 100.f);
	const Position across = routePosition(0.f, 500.f);
	GeofencePathChecker checker{_fence};
	ProjectionReferenceBatch batch{};
	batch.count = 1;
	batch.items[0].position = vehicle;
	addCandidate(batch.items[0], across, 1);
	addCandidate(batch.items[0], away, 2);
	ASSERT_TRUE(checker.pruneCandidates(batch));
	ASSERT_EQ(batch.items[0].candidate_buffer.count, 1);
	EXPECT_EQ(batch.items[0].candidate_buffer.candidates[0].segment.start.idx, 2);

	bool clear = false;
	ASSERT_TRUE(checker.checkPath(vehicle, away, clear));
	EXPECT_TRUE(clear);
	ASSERT_TRUE(checker.checkPath(vehicle, across, clear));
	EXPECT_FALSE(clear);
}
