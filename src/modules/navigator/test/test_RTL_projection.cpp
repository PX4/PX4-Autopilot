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
 * @file test_RTL_projection.cpp
 *
 * Google Test suite for RtlRoutePlanner vehicle projection and
 * candidate detection behavior. Tests how the vehicle projects onto
 * the mission route and how projection candidates are detected.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "test_RTL_helpers.h"
#include "test_RTL_data.h"

static constexpr double kBaseLat = 47.397742;
static constexpr double kBaseLon = 8.545594;
static constexpr float  kAlt     = 500.f;

// ============================================================================
// Test fixture
// ============================================================================

class RtlProjectionTest : public RtlRoutePlannerTestBase {};

// ============================================================================
// GROUP 1: Vehicle projection onto current/nearby segment
// ============================================================================

// WHY: The planner should prefer the segment containing the current mission index to maintain continuity.
// WHAT: Vehicle near seg [0-1] with mission_index=1; verify it projects onto [0-1] with ~10m xtrack.
TEST_F(RtlProjectionTest, PrefersCurrentMissionSegment)
{
	// GIVEN: 3-wp mission (straight then right turn), vehicle offset 90N 10E
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};
	VectorProvider provider(mission, {});
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 90.f, 10.f, kAlt);

	// WHEN: project with mission_index=1
	bool ok = planner.collectVehicleProjection(vehicle, 1, config, ctx, &reason);

	// THEN: projects onto segment [0-1] with cross-track approximately 10 m
	ASSERT_TRUE(ok);
	EXPECT_EQ(ctx.seg_candidate.segment.start.idx, 0);
	EXPECT_EQ(ctx.seg_candidate.segment.end.idx, 1);
	EXPECT_NEAR(ctx.seg_candidate.dist.xtrack, 10.f, kDistanceTolerance);
}

// WHY: A negative or out-of-range mission index should be clamped rather than causing undefined behavior.
// WHAT: Pass various out-of-range indices, verify they all produce the same result as index 0.
// NOTE: Uses TEST_P to independently test each invalid index value.
class RtlProjectionClampTest : public RtlRoutePlannerTestBase, public ::testing::WithParamInterface<int> {};

TEST_P(RtlProjectionClampTest, ClampsOutOfRangeMissionIndex)
{
	const int bad_index = GetParam();

	// GIVEN: 3-wp mission, vehicle at offset (100N, 60E)
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};
	VectorProvider provider(mission, {});
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 60.f, kAlt);

	RtlRoutePlanner::ProjectionContext ctx_bad{};
	RtlRoutePlanner::ProjectionContext ctx_zero{};

	// WHEN: project with the bad index and with index 0
	bool ok_bad  = planner.collectVehicleProjection(vehicle, bad_index, config, ctx_bad, &reason);
	bool ok_zero = planner.collectVehicleProjection(vehicle, 0, config, ctx_zero, &reason);

	// THEN: both succeed and produce the same segment indices
	ASSERT_TRUE(ok_bad);
	ASSERT_TRUE(ok_zero);
	EXPECT_EQ(ctx_bad.seg_candidate.segment.start.idx, ctx_zero.seg_candidate.segment.start.idx);
	EXPECT_EQ(ctx_bad.seg_candidate.segment.end.idx, ctx_zero.seg_candidate.segment.end.idx);
}

INSTANTIATE_TEST_SUITE_P(
	InvalidIndices,
	RtlProjectionClampTest,
	::testing::Values(-42, -1, 0)
);

// WHY: When the vehicle was last flying a DO_JUMP loop segment, the planner should prefer that segment even if another non-loop segment is closer.
// WHAT: Vehicle at (75N, 10E) with stored loop context [2->0], verify loop segment is selected.
TEST_F(RtlProjectionTest, PrefersStoredLoopAnchor)
{
	// GIVEN: 5-item mission with DO_JUMP at idx 3 (jumps to 0, repeat 2)
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makeDoJump(0, 2, 0),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f,   0.f, kAlt),
	};
	VectorProvider provider(mission, {});
	RtlRoutePlanner planner(provider);

	// Set the stored loop anchor to segment [2->0]
	config.last_flown_loop_segment.start.idx = 2;
	config.last_flown_loop_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.end.idx = 0;
	config.last_flown_loop_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.is_loop = true;
	config.last_flown_loop_segment.loops_remaining = 1;

	RtlRoutePlanner::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 75.f, 10.f, kAlt);

	// WHEN: project with mission_index=0 (inside the loop)
	bool ok = planner.collectVehicleProjection(vehicle, 0, config, ctx, &reason);

	// THEN: projection segment is the loop segment [2->0]
	ASSERT_TRUE(ok);
	EXPECT_TRUE(ctx.seg_candidate.segment.is_loop);
	EXPECT_EQ(ctx.seg_candidate.segment.start.idx, 2);
	EXPECT_EQ(ctx.seg_candidate.segment.end.idx, 0);
}

// ============================================================================
// GROUP 2: Default dataset - vehicle projection at various positions
// ============================================================================

// WHY: With real GPS coordinates, the planner should still prefer the mission-index segment.
// WHAT: Vehicle near seg [1-2], mission_index=2 -> projects onto [1-2].
TEST_F(RtlProjectionTest, DefaultMission_OnCurrentSegment)
{
	// GIVEN: default dataset mission, vehicle near segment [1-2]
	VectorProvider provider(default_dataset::mission(), default_dataset::safePoints());
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle = makePositionAbsolute(46.10508903154495, 2.302372024012729, 463.0f);

	// WHEN: project with mission_index=2
	bool ok = planner.collectVehicleProjection(vehicle, 2, config, ctx, &reason);

	// THEN: projects onto segment [1-2]
	ASSERT_TRUE(ok);
	EXPECT_EQ(ctx.seg_candidate.segment.start.idx, 1);
	EXPECT_EQ(ctx.seg_candidate.segment.end.idx, 2);
}

// WHY: When the vehicle is clearly on the segment matching its mission index, projection should match.
// WHAT: Vehicle on segment [0-1], mission_index=1.
TEST_F(RtlProjectionTest, DefaultMission_OnSameSegment)
{
	// GIVEN: default dataset mission, vehicle on segment [0-1]
	VectorProvider provider(default_dataset::mission(), default_dataset::safePoints());
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle = makePositionAbsolute(46.098944316424465, 2.2977800821792327, 475.6f);

	// WHEN: project with mission_index=1
	bool ok = planner.collectVehicleProjection(vehicle, 1, config, ctx, &reason);

	// THEN: projects onto segment [0-1]
	ASSERT_TRUE(ok);
	EXPECT_EQ(ctx.seg_candidate.segment.start.idx, 0);
	EXPECT_EQ(ctx.seg_candidate.segment.end.idx, 1);
}

// WHY: Vehicle near a segment different from its mission index should still project correctly.
// WHAT: Vehicle near seg [4-5], mission_index=5.
TEST_F(RtlProjectionTest, DefaultMission_FrontBackDifferentSegment)
{
	// GIVEN: default dataset mission, vehicle near segment [4-5]
	VectorProvider provider(default_dataset::mission(), default_dataset::safePoints());
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle = makePositionAbsolute(46.10795279737903, 2.299475977516394, 454.4f);

	// WHEN: project with mission_index=5
	bool ok = planner.collectVehicleProjection(vehicle, 5, config, ctx, &reason);

	// THEN: projects onto segment [4-5]
	ASSERT_TRUE(ok);
	EXPECT_EQ(ctx.seg_candidate.segment.start.idx, 4);
	EXPECT_EQ(ctx.seg_candidate.segment.end.idx, 5);
}

// WHY: When the route doubles back (segments [7-8] and [11-12] run roughly parallel), the planner must use mission_index to disambiguate.
// WHAT: Vehicle near overlapping segments, mission_index=8. Expect seg [7-8].
TEST_F(RtlProjectionTest, DefaultMission_CoincidingSegments)
{
	// GIVEN: default dataset mission, vehicle near coinciding segments
	VectorProvider provider(default_dataset::mission(), default_dataset::safePoints());
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle = makePositionAbsolute(46.11174050459439, 2.2876843642852362, 475.9f);

	// WHEN: project with mission_index=8
	bool ok = planner.collectVehicleProjection(vehicle, 8, config, ctx, &reason);

	// THEN: projects onto segment [7-8] (not [11-12])
	ASSERT_TRUE(ok);
	EXPECT_EQ(ctx.seg_candidate.segment.start.idx, 7);
	EXPECT_EQ(ctx.seg_candidate.segment.end.idx, 8);
}

// WHY: Projection near the last mission segment must work correctly at the route boundary.
// WHAT: Vehicle near seg [14-15], mission_index=15. Expect seg [14-15].
TEST_F(RtlProjectionTest, DefaultMission_AtRouteEnd)
{
	// GIVEN: default dataset mission, vehicle near the final segment
	VectorProvider provider(default_dataset::mission(), default_dataset::safePoints());
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle = makePositionAbsolute(46.112843317707494, 2.3059421291432525, 455.4f);

	// WHEN: project with mission_index=15
	bool ok = planner.collectVehicleProjection(vehicle, 15, config, ctx, &reason);

	// THEN: projects onto segment [14-15]
	ASSERT_TRUE(ok);
	EXPECT_EQ(ctx.seg_candidate.segment.start.idx, 14);
	EXPECT_EQ(ctx.seg_candidate.segment.end.idx, 15);
}

// ============================================================================
// GROUP 3: Corner dataset - vehicle projection
// ============================================================================

// WHY: Validates projection near a corner waypoint selects the correct segment.
// WHAT: Vehicle at corner location, mission_index=2. Expect seg [1-2].
TEST_F(RtlProjectionTest, CornerMission_OnSeg1To2)
{
	// GIVEN: corner dataset mission, vehicle near segment [1-2]
	VectorProvider provider(corner_dataset::mission(), corner_dataset::safePoints());
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle = makePositionAbsolute(46.103348739288705, 2.3235968076446945, 479.7f);

	// WHEN: project with mission_index=2
	bool ok = planner.collectVehicleProjection(vehicle, 2, config, ctx, &reason);

	// THEN: projects onto segment [1-2]
	ASSERT_TRUE(ok);
	EXPECT_EQ(ctx.seg_candidate.segment.start.idx, 1);
	EXPECT_EQ(ctx.seg_candidate.segment.end.idx, 2);
}

// WHY: Validates projection after a VTOL transition command on a different segment.
// WHAT: Vehicle near seg [4-5], mission_index=5. Expect seg [4-5].
TEST_F(RtlProjectionTest, CornerMission_OnSeg4To5)
{
	// GIVEN: corner dataset mission, vehicle near segment [4-5]
	VectorProvider provider(corner_dataset::mission(), corner_dataset::safePoints());
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle = makePositionAbsolute(46.10205080248656, 2.318838207366314, 462.1f);

	// WHEN: project with mission_index=5
	bool ok = planner.collectVehicleProjection(vehicle, 5, config, ctx, &reason);

	// THEN: projects onto segment [4-5]
	ASSERT_TRUE(ok);
	EXPECT_EQ(ctx.seg_candidate.segment.start.idx, 4);
	EXPECT_EQ(ctx.seg_candidate.segment.end.idx, 5);
}

// WHY: Segment [12-13] is only ~14m long; projection must still work on very short segments.
// WHAT: Vehicle near the small segment, mission_index=13. Expect seg [12-13].
TEST_F(RtlProjectionTest, CornerMission_OnSmallSegment)
{
	// GIVEN: corner dataset mission, vehicle near tiny segment [12-13]
	VectorProvider provider(corner_dataset::mission(), corner_dataset::safePoints());
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle = makePositionAbsolute(46.10361319095525, 2.3183349874167636, 462.6f);

	// WHEN: project with mission_index=13
	bool ok = planner.collectVehicleProjection(vehicle, 13, config, ctx, &reason);

	// THEN: projects onto segment [12-13]
	ASSERT_TRUE(ok);
	EXPECT_EQ(ctx.seg_candidate.segment.start.idx, 12);
	EXPECT_EQ(ctx.seg_candidate.segment.end.idx, 13);
}

// ============================================================================
// GROUP 4: Candidate buffer and local-minimum detection
// ============================================================================

// WHY: A rally point behind the takeoff should project onto the first segment [0-1], detecting takeoff as a local minimum.
// WHAT: Rally behind takeoff, vehicle at takeoff. Verify branch-off on [0-1] near takeoff position.
TEST_F(RtlProjectionTest, TakeoffIsLocalMinimum)
{
	// GIVEN: 4-item mission (takeoff, wp, vtol_trans, wp), rally behind takeoff
	std::vector<mission_item_s> mission = {
		makeTakeoffItem(47.0000000, 8.0000000, 500.f),
		makePositionItem(47.0000000, 8.0010000, 500.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(47.0000000, 8.0020000, 500.f),
	};
	std::vector<mission_item_s> safe_points = {
		makeSafePointAbsolute(47.0000000, 7.9990000, 500.f),
	};
	VectorProvider provider(mission, safe_points);
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle = makePositionAbsolute(47.0000000, 8.0000000, 500.f);
	RtlRoutePlanner::ProjectionContext proj_ctx{};

	// WHEN: project vehicle and select safe point
	bool ok = planner.collectVehicleProjection(vehicle, 0, config, proj_ctx, &reason);
	ASSERT_TRUE(ok);
	RtlRoutePlanner::Selection selection = planner.selectSafePoint(proj_ctx, config);

	// THEN: selection found, branch-off on segment [0-1], projection near takeoff
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 0);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 1);
	EXPECT_NEAR(selection.branch_off_projection.lat, 47.0, kLatLonToleranceDeg);
	EXPECT_NEAR(selection.branch_off_projection.lon, 8.0, kLatLonToleranceDeg);
}

// WHY: A zero-length segment (wp stacked on takeoff) must not break projection.
// WHAT: Same as above but wp1 is at same position as takeoff. Rally behind should still find projection.
TEST_F(RtlProjectionTest, StackedWaypointAboveTakeoff)
{
	// GIVEN: wp1 stacked on takeoff position
	std::vector<mission_item_s> mission = {
		makeTakeoffItem(47.0000000, 8.0000000, 500.f),
		makePositionItem(47.0000000, 8.0000000, 550.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(47.0000000, 8.0020000, 500.f),
	};
	std::vector<mission_item_s> safe_points = {
		makeSafePointAbsolute(47.0000000, 7.9990000, 500.f),
	};
	VectorProvider provider(mission, safe_points);
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle = makePositionAbsolute(47.0000000, 8.0000000, 500.f);
	RtlRoutePlanner::ProjectionContext proj_ctx{};

	// WHEN: project vehicle and select safe point
	bool ok = planner.collectVehicleProjection(vehicle, 0, config, proj_ctx, &reason);
	ASSERT_TRUE(ok);
	RtlRoutePlanner::Selection selection = planner.selectSafePoint(proj_ctx, config);

	// THEN: selection found, projection near takeoff position
	ASSERT_TRUE(selection.found);
	EXPECT_NEAR(selection.branch_off_projection.lat, 47.0, kLatLonToleranceDeg);
	EXPECT_NEAR(selection.branch_off_projection.lon, 8.0, kLatLonToleranceDeg);
}

// WHY: A zero-length segment at the land point must not break projection.
// WHAT: wp2 stacked on LAND position. Safe point beyond land should project onto land segment.
TEST_F(RtlProjectionTest, StackedWaypointAboveLand)
{
	// GIVEN: wp2 stacked on land position
	std::vector<mission_item_s> mission = {
		makeTakeoffItem(47.0000000, 8.0000000, 500.f),
		makePositionItem(47.0000000, 8.0010000, 500.f),
		makePositionItem(47.0000000, 8.0020000, 550.f),
		makeLandItem(47.0000000, 8.0020000, 500.f),
	};
	std::vector<mission_item_s> safe_points = {
		makeSafePointAbsolute(47.0000000, 8.0030000, 500.f),
	};
	VectorProvider provider(mission, safe_points);
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle = makePositionAbsolute(47.0000000, 8.0010000, 500.f);
	RtlRoutePlanner::ProjectionContext proj_ctx{};

	// WHEN: project vehicle and select safe point
	bool ok = planner.collectVehicleProjection(vehicle, 2, config, proj_ctx, &reason);
	ASSERT_TRUE(ok);
	RtlRoutePlanner::Selection selection = planner.selectSafePoint(proj_ctx, config);

	// THEN: selection found, projection near land position
	ASSERT_TRUE(selection.found);
	EXPECT_NEAR(selection.branch_off_projection.lat, 47.0, kLatLonToleranceDeg);
	EXPECT_NEAR(selection.branch_off_projection.lon, 8.002, kLatLonToleranceDeg);
}

// WHY: On a straight-line route, intermediate waypoint "corners" are not true local minima and should be pruned.
// WHAT: 10-wp straight line, rally offset east of mid-point -> projects onto [4-5] only.
TEST_F(RtlProjectionTest, StraightLineIgnoresNonMinCorners)
{
	// GIVEN: 10-waypoint straight line going north, rally offset east near mid-point
	std::vector<mission_item_s> mission;

	for (int i = 0; i < 10; ++i) {
		mission.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon,
				  static_cast<float>(i * 100), 0.f, kAlt));
	}

	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 450.f, 50.f, kAlt),
	};
	VectorProvider provider(mission, safe_points);
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 450.f, 0.f, kAlt);
	RtlRoutePlanner::ProjectionContext proj_ctx{};

	// WHEN: project vehicle and select safe point
	bool ok = planner.collectVehicleProjection(vehicle, 5, config, proj_ctx, &reason);
	ASSERT_TRUE(ok);
	RtlRoutePlanner::Selection selection = planner.selectSafePoint(proj_ctx, config);

	// THEN: branch-off segment is [4-5], not corner artifacts
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 4);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 5);
}

// WHY: A rectangle mission has 4 local minima (one per side). With MAX_SEGMENT_CANDIDATES=3, the farthest must be dropped.
// WHAT: Rectangle route, rally biased toward left side. Farthest segment [1-2] (right side) should NOT be selected.
TEST_F(RtlProjectionTest, RectangleKeepsThreeClosestSegments)
{
	// GIVEN: rectangle mission ABCDA, rally biased left
	//   0 -- 1
	//   |    |
	//   3 -- 2
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,    0.f,    0.f, kAlt),   // 0: bottom-left
		makePositionItemFromOffset(kBaseLat, kBaseLon,    0.f, 1000.f, kAlt),   // 1: bottom-right
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 1000.f, kAlt),   // 2: top-right
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f,    0.f, kAlt),   // 3: top-left
		makePositionItemFromOffset(kBaseLat, kBaseLon,    0.f,    0.f, kAlt),   // 4: close back at start
	};

	// Rally point biased toward the left side (west), away from segment [1-2]
	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 500.f, -50.f, kAlt),
	};
	VectorProvider provider(mission, safe_points);
	RtlRoutePlanner planner(provider);

	config.vehicle_projection_search_dist = 2000.f;
	config.safe_point_projection_search_dist = 2000.f;
	RtlRoutePlanner::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt);
	RtlRoutePlanner::ProjectionContext proj_ctx{};

	// WHEN: project vehicle and select safe point
	bool ok = planner.collectVehicleProjection(vehicle, 3, config, proj_ctx, &reason);
	ASSERT_TRUE(ok);
	RtlRoutePlanner::Selection selection = planner.selectSafePoint(proj_ctx, config);

	// THEN: the farthest segment [1-2] (right side) should not be selected
	ASSERT_TRUE(selection.found);
	EXPECT_NE(selection.branch_off_segment.start.idx, 1);
}

// WHY: Multiple identical waypoints at a corner must not create false local minima that fill the candidate buffer and evict real projections.
// WHAT: L-shape with 8 duplicated corner waypoints. Duplicates must not fill buffer and evict the real non-corner projection.
TEST_F(RtlProjectionTest, DuplicateCornerWaypointsDoNotEvictValidCandidates)
{
	// GIVEN: route with duplicated corner waypoints
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),     // 0
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f,   0.f, kAlt),     // 1
	};

	// Add 8 duplicate waypoints at the same corner position
	for (int i = 0; i < 8; ++i) {
		mission.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 200.f, kAlt));
	}

	// Final waypoint
	mission.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 200.f, kAlt));

	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 250.f, kAlt),
	};
	VectorProvider provider(mission, safe_points);
	RtlRoutePlanner planner(provider);

	config.vehicle_projection_search_dist = 500.f;
	config.safe_point_projection_search_dist = 500.f;
	RtlRoutePlanner::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 200.f, kAlt);
	RtlRoutePlanner::ProjectionContext proj_ctx{};

	// WHEN: project vehicle and select safe point
	bool ok = planner.collectVehicleProjection(vehicle, 9, config, proj_ctx, &reason);
	ASSERT_TRUE(ok);
	RtlRoutePlanner::Selection selection = planner.selectSafePoint(proj_ctx, config);

	// THEN: selection found despite duplicate corner waypoints
	EXPECT_TRUE(selection.found);
}

// ============================================================================
// GROUP 5: Edge cases and error handling
// ============================================================================

// WHY: NaN and Infinity coordinates should cause projection to fail gracefully rather than producing undefined results.
// WHAT: Pass NaN, +Infinity, and -Infinity for latitude and longitude. collectVehicleProjection should return false.
class RtlProjectionInvalidCoordTest : public RtlRoutePlannerTestBase,
	public ::testing::WithParamInterface<std::pair<double, double>> {};

TEST_P(RtlProjectionInvalidCoordTest, InvalidVehiclePositionFails)
{
	const auto [lat, lon] = GetParam();

	// GIVEN: simple 3-wp mission
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};
	VectorProvider provider(mission, {});
	RtlRoutePlanner planner(provider);

	// WHEN: vehicle with invalid coordinate
	RtlRoutePlanner::Position vehicle{};
	vehicle.lat = lat;
	vehicle.lon = lon;
	vehicle.alt = kAlt;
	bool ok = planner.collectVehicleProjection(vehicle, 1, config, ctx, &reason);

	// THEN: projection fails
	EXPECT_FALSE(ok);
}

INSTANTIATE_TEST_SUITE_P(
	InvalidCoordinates,
	RtlProjectionInvalidCoordTest,
	::testing::Values(
		std::make_pair(NAN, kBaseLon),                    // NaN latitude
		std::make_pair(kBaseLat, NAN),                    // NaN longitude
		std::make_pair(INFINITY, kBaseLon),               // +Infinity latitude
		std::make_pair(kBaseLat, INFINITY),               // +Infinity longitude
		std::make_pair(-INFINITY, kBaseLon),              // -Infinity latitude
		std::make_pair(kBaseLat, -INFINITY)               // -Infinity longitude
	)
);

// WHY: A mission with only one waypoint has no segments and cannot support projection.
// WHAT: Single-item mission, collectVehicleProjection should return false.
TEST_F(RtlProjectionTest, SingleWaypointMissionFails)
{
	// GIVEN: single-waypoint mission
	std::vector<mission_item_s> mission = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
	};
	VectorProvider provider(mission, {});
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Position vehicle = makePositionAbsolute(kBaseLat, kBaseLon, kAlt);

	// WHEN: attempt projection
	bool ok = planner.collectVehicleProjection(vehicle, 0, config, ctx, &reason);

	// THEN: projection fails (no segments)
	EXPECT_FALSE(ok);
}

// WHY: A mission that zigzags creates many local minima; the candidate buffer (MAX_SEGMENT_CANDIDATES=3) must keep only the closest three and not overflow.
// WHAT: 8-waypoint zigzag pattern with a safe point near one segment. Verify selection.found is true.
TEST_F(RtlProjectionTest, ZigzagMissionStressesCandidateBuffer)
{
	// GIVEN: 8-waypoint zigzag pattern going east, alternating N/S offsets
	std::vector<mission_item_s> mission;

	for (int i = 0; i < 8; ++i) {
		float north = (i % 2 == 0) ? 0.f : 200.f;
		float east  = static_cast<float>(i * 150);
		mission.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, north, east, kAlt));
	}

	// Safe point near the center of the zigzag
	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 100.f, 680.f, kAlt),
	};
	VectorProvider provider(mission, safe_points);
	RtlRoutePlanner planner(provider);

	config.vehicle_projection_search_dist = 500.f;
	config.safe_point_projection_search_dist = 500.f;
	RtlRoutePlanner::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 600.f, kAlt);
	RtlRoutePlanner::ProjectionContext proj_ctx{};

	// WHEN: project vehicle and select safe point
	bool ok = planner.collectVehicleProjection(vehicle, 5, config, proj_ctx, &reason);
	ASSERT_TRUE(ok);
	RtlRoutePlanner::Selection selection = planner.selectSafePoint(proj_ctx, config);

	// THEN: selection found despite many local minima stressing the buffer
	ASSERT_TRUE(selection.found);
}
