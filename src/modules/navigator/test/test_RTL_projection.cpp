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
 * Google Test suite for MissionRoutePlanner vehicle projection and
 * candidate detection behavior. Tests how the vehicle projects onto
 * the mission route and how projection candidates are detected.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "test_RTL_helpers.h"
#include "test_RTL_data.h"

#include <array>

using rtl_test_reference::kAlt;
using rtl_test_reference::kBaseLat;
using rtl_test_reference::kBaseLon;

/** @brief Shared fixture for route-projection planner tests. */
class RtlProjectionTestBase : public MissionRoutePlannerTestBase {};
/** @brief Covers vehicle projection onto the current or nearby mission segment. */
class RtlProjectionLocalSegmentTest : public RtlProjectionTestBase {};
/** @brief Covers candidate-buffer and local-minimum behavior during projection. */
class RtlProjectionCandidateSelectionTest : public RtlProjectionTestBase {};
/** @brief Covers projection edge cases and rejection of invalid inputs. */
class RtlProjectionEdgeCaseTest : public RtlProjectionTestBase {};

// WHY: The planner should prefer the segment containing the current mission index to maintain continuity.
// WHAT: Vehicle near seg [0-1] with mission_index=1; verify it projects onto [0-1] with ~10m xtrack.
TEST_F(RtlProjectionLocalSegmentTest, PrefersCurrentMissionSegment)
{
	// GIVEN: 3-wp mission (straight then right turn), vehicle offset 90N 10E
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};
	VectorProvider provider(mission, {});
	MissionRoutePlanner planner(provider);

	MissionRoutePlanner::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 90.f, 10.f, kAlt);

	// WHEN: project with mission_index=1
	bool ok = planner.collectVehicleProjection(vehicle, 1, config, ctx, reason);

	// THEN: projects onto segment [0-1] with cross-track approximately 10 m
	ASSERT_TRUE(ok);
	EXPECT_EQ(ctx.seg_candidate.segment.start.idx, 0);
	EXPECT_EQ(ctx.seg_candidate.segment.end.idx, 1);
	EXPECT_NEAR(ctx.seg_candidate.dist.xtrack, 10.f, kDistanceTolerance);
}

// WHY: A negative or out-of-range mission index should be clamped rather than causing undefined behavior.
// WHAT: Negative indices clamp to the route start and overly large indices clamp to the route end.
// NOTE: Uses TEST_P to independently test each invalid index value.
/** @brief Parameterized fixture for mission-index clamping during projection. */
class RtlProjectionClampTest : public RtlProjectionTestBase, public ::testing::WithParamInterface<int> {};

TEST_P(RtlProjectionClampTest, ClampsOutOfRangeMissionIndex)
{
	const int bad_index = GetParam();

	// GIVEN: A 3-waypoint L-shaped mission and a vehicle near the final segment.
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};
	VectorProvider provider(mission, {});
	MissionRoutePlanner planner(provider);

	MissionRoutePlanner::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 60.f, kAlt);

	MissionRoutePlanner::ProjectionContext ctx_bad{};
	MissionRoutePlanner::ProjectionContext ctx_reference{};
	const int reference_index = (bad_index < 0) ? 0 : static_cast<int>(mission.size()) - 1;

	// WHEN: Projection runs with the invalid index and with the corresponding clamped reference index.
	bool ok_bad = planner.collectVehicleProjection(vehicle, bad_index, config, ctx_bad, reason);
	bool ok_reference = planner.collectVehicleProjection(vehicle, reference_index, config, ctx_reference, reason);

	// THEN: The invalid index behaves exactly like the clamped mission boundary.
	ASSERT_TRUE(ok_bad);
	ASSERT_TRUE(ok_reference);
	EXPECT_EQ(ctx_bad.seg_candidate.segment.start.idx, ctx_reference.seg_candidate.segment.start.idx);
	EXPECT_EQ(ctx_bad.seg_candidate.segment.end.idx, ctx_reference.seg_candidate.segment.end.idx);
}

INSTANTIATE_TEST_SUITE_P(
	InvalidIndices,
	RtlProjectionClampTest,
	::testing::Values(-42, -1, 99)
);

// WHY: Relative-altitude mission items require a valid home altitude in order to produce a safe AMSL target.
// WHAT: A relative-altitude item with NaN home altitude yields NaN absolute altitude and is rejected as a mission position.
TEST_F(RtlProjectionLocalSegmentTest, RelativeAltitudeRequiresFiniteHomeAltitude)
{
	mission_item_s mission_item = makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, 50.f);
	mission_item.altitude_is_relative = true;

	MissionRoutePlanner::Position position{};
	const float absolute_altitude = MissionRoutePlanner::getAbsoluteAltitudeForMissionItem(mission_item, NAN);

	EXPECT_FALSE(PX4_ISFINITE(absolute_altitude));
	EXPECT_FALSE(MissionRoutePlanner::extractMissionPosition(mission_item, NAN, position));
}

// WHY: Reverse route following changes which segment owns a boundary mission index.
// WHAT: Vehicle on the waypoint-1 corner with mission_index=1 should map to [1-2] when flying in reverse.
TEST_F(RtlProjectionLocalSegmentTest, ReverseFlightPrefersReverseCurrentSegment)
{
	// GIVEN: 3-wp L-shaped mission and reverse-flight configuration.
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};
	VectorProvider provider(mission, {});
	MissionRoutePlanner planner(provider);
	config.state.is_flying_reverse = true;

	const MissionRoutePlanner::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 10.f, kAlt);

	// WHEN: collectVehicleProjection is called at the segment boundary.
	bool ok = planner.collectVehicleProjection(vehicle, 1, config, ctx, reason);

	// THEN: The reverse-owned segment [1-2] is selected.
	ASSERT_TRUE(ok);
	EXPECT_EQ(ctx.seg_candidate.segment.start.idx, 1);
	EXPECT_EQ(ctx.seg_candidate.segment.end.idx, 2);
}

// WHY: When the vehicle was last flying a DO_JUMP loop segment, the planner should prefer that segment even if another non-loop segment is closer.
// WHAT: Vehicle at (75N, 10E) with stored loop context [2->0], verify loop segment is selected.
TEST_F(RtlProjectionLocalSegmentTest, PrefersStoredLoopAnchor)
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
	MissionRoutePlanner planner(provider);

	// Set the stored loop anchor to segment [2->0]
	config.execution.last_flown_loop_segment.start.idx = 2;
	config.execution.last_flown_loop_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	config.execution.last_flown_loop_segment.end.idx = 0;
	config.execution.last_flown_loop_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	config.execution.last_flown_loop_segment.is_loop = true;
	config.execution.last_flown_loop_segment.loops_remaining = 1;

	MissionRoutePlanner::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 75.f, 10.f, kAlt);

	// WHEN: project with mission_index=0 (inside the loop)
	bool ok = planner.collectVehicleProjection(vehicle, 0, config, ctx, reason);

	// THEN: projection segment is the loop segment [2->0]
	ASSERT_TRUE(ok);
	EXPECT_TRUE(ctx.seg_candidate.segment.is_loop);
	EXPECT_EQ(ctx.seg_candidate.segment.start.idx, 2);
	EXPECT_EQ(ctx.seg_candidate.segment.end.idx, 0);
}

struct ProjectionDatasetCase {
	const char *name;
	bool use_corner_dataset;
	double lat;
	double lon;
	float alt;
	int mission_index;
	int expected_start_idx;
	int expected_end_idx;
};

static constexpr std::array<ProjectionDatasetCase, 8> kProjectionDatasetCases{{
		{"DefaultOnCurrentSegment", false, 46.10508903154495, 2.302372024012729, 463.0f, 2, 1, 2},
		{"DefaultOnSameSegment", false, 46.098944316424465, 2.2977800821792327, 475.6f, 1, 0, 1},
		{"DefaultFrontBackDifferentSegment", false, 46.10795279737903, 2.299475977516394, 454.4f, 5, 4, 5},
		{"DefaultCoincidingSegments", false, 46.11174050459439, 2.2876843642852362, 475.9f, 8, 7, 8},
		{"DefaultAtRouteEnd", false, 46.112843317707494, 2.3059421291432525, 455.4f, 15, 14, 15},
		{"CornerOnSeg1To2", true, 46.103348739288705, 2.3235968076446945, 479.7f, 2, 1, 2},
		{"CornerOnSeg4To5", true, 46.10205080248656, 2.318838207366314, 462.1f, 5, 4, 5},
		{"CornerOnSmallSegment", true, 46.10361319095525, 2.3183349874167636, 462.6f, 13, 12, 13},
	}};

/** @brief Parameterized fixture for dataset-driven projection segment checks. */
class RtlProjectionDatasetTest : public RtlProjectionTestBase,
	public ::testing::WithParamInterface<ProjectionDatasetCase> {};

// WHY: Dataset-driven projection checks all exercise the same contract and are easier to maintain
//      when their geometry is captured as explicit scenarios instead of duplicated test bodies.
// WHAT: Each named dataset scenario projects onto the expected route segment.
TEST_P(RtlProjectionDatasetTest, SelectsExpectedSegment)
{
	// GIVEN: A named projection scenario on either the default or corner dataset.
	const ProjectionDatasetCase &scenario = GetParam();
	const auto mission = scenario.use_corner_dataset ? corner_dataset::mission() : default_dataset::mission();
	const auto safe_points = scenario.use_corner_dataset ? corner_dataset::safePoints() : default_dataset::safePoints();
	VectorProvider provider(mission, safe_points);
	MissionRoutePlanner planner(provider);
	const MissionRoutePlanner::Position vehicle = makePositionAbsolute(scenario.lat, scenario.lon, scenario.alt);

	// WHEN: collectVehicleProjection is called for the scenario mission index.
	bool ok = planner.collectVehicleProjection(vehicle, scenario.mission_index, config, ctx, reason);

	// THEN: The expected route segment is selected.
	ASSERT_TRUE(ok);
	EXPECT_EQ(ctx.seg_candidate.segment.start.idx, scenario.expected_start_idx);
	EXPECT_EQ(ctx.seg_candidate.segment.end.idx, scenario.expected_end_idx);
}

INSTANTIATE_TEST_SUITE_P(
	DatasetScenarios,
	RtlProjectionDatasetTest,
	::testing::ValuesIn(kProjectionDatasetCases),
	[](const ::testing::TestParamInfo<ProjectionDatasetCase> &param_info)
{
	return param_info.param.name;
}
);

// WHY: A rally point behind the takeoff should project onto the first segment [0-1], detecting takeoff as a local minimum.
// WHAT: Rally behind takeoff, vehicle at takeoff. Verify branch-off on [0-1] near takeoff position.
TEST_F(RtlProjectionCandidateSelectionTest, TakeoffIsLocalMinimum)
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
	MissionRoutePlanner planner(provider);

	MissionRoutePlanner::Position vehicle = makePositionAbsolute(47.0000000, 8.0000000, 500.f);
	MissionRoutePlanner::ProjectionContext proj_ctx{};

	// WHEN: project vehicle and select safe point
	bool ok = planner.collectVehicleProjection(vehicle, 0, config, proj_ctx, reason);
	ASSERT_TRUE(ok);
	MissionRoutePlanner::Selection selection = planner.selectSafePoint(proj_ctx, config);

	// THEN: selection found, branch-off on segment [0-1], projection near takeoff
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 0);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 1);
	EXPECT_NEAR(selection.branch_off_projection.lat, 47.0, kLatLonToleranceDeg);
	EXPECT_NEAR(selection.branch_off_projection.lon, 8.0, kLatLonToleranceDeg);
}

// WHY: A zero-length segment (wp stacked on takeoff) must not break projection.
// WHAT: Same as above but wp1 is at same position as takeoff. Rally behind should still find projection.
TEST_F(RtlProjectionCandidateSelectionTest, StackedWaypointAboveTakeoff)
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
	MissionRoutePlanner planner(provider);

	MissionRoutePlanner::Position vehicle = makePositionAbsolute(47.0000000, 8.0000000, 500.f);
	MissionRoutePlanner::ProjectionContext proj_ctx{};

	// WHEN: project vehicle and select safe point
	bool ok = planner.collectVehicleProjection(vehicle, 0, config, proj_ctx, reason);
	ASSERT_TRUE(ok);
	MissionRoutePlanner::Selection selection = planner.selectSafePoint(proj_ctx, config);

	// THEN: selection found, projection near takeoff position
	ASSERT_TRUE(selection.found);
	EXPECT_NEAR(selection.branch_off_projection.lat, 47.0, kLatLonToleranceDeg);
	EXPECT_NEAR(selection.branch_off_projection.lon, 8.0, kLatLonToleranceDeg);
}

// WHY: A zero-length segment at the land point must not break projection.
// WHAT: wp2 stacked on LAND position. Safe point beyond land should project onto land segment.
TEST_F(RtlProjectionCandidateSelectionTest, StackedWaypointAboveLand)
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
	MissionRoutePlanner planner(provider);

	MissionRoutePlanner::Position vehicle = makePositionAbsolute(47.0000000, 8.0010000, 500.f);
	MissionRoutePlanner::ProjectionContext proj_ctx{};

	// WHEN: project vehicle and select safe point
	bool ok = planner.collectVehicleProjection(vehicle, 2, config, proj_ctx, reason);
	ASSERT_TRUE(ok);
	MissionRoutePlanner::Selection selection = planner.selectSafePoint(proj_ctx, config);

	// THEN: selection found, projection near land position
	ASSERT_TRUE(selection.found);
	EXPECT_NEAR(selection.branch_off_projection.lat, 47.0, kLatLonToleranceDeg);
	EXPECT_NEAR(selection.branch_off_projection.lon, 8.002, kLatLonToleranceDeg);
}

// WHY: On a straight-line route, intermediate waypoint "corners" are not true local minima and should be pruned.
// WHAT: 10-wp straight line, rally offset east of mid-point -> projects onto [4-5] only.
TEST_F(RtlProjectionCandidateSelectionTest, StraightLineIgnoresNonMinCorners)
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
	MissionRoutePlanner planner(provider);

	MissionRoutePlanner::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 450.f, 0.f, kAlt);
	MissionRoutePlanner::ProjectionContext proj_ctx{};

	// WHEN: project vehicle and select safe point
	bool ok = planner.collectVehicleProjection(vehicle, 5, config, proj_ctx, reason);
	ASSERT_TRUE(ok);
	MissionRoutePlanner::Selection selection = planner.selectSafePoint(proj_ctx, config);

	// THEN: branch-off segment is [4-5], not corner artifacts
	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 4);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 5);
}

// WHY: A rectangle mission has 4 local minima (one per side). With MAX_SEGMENT_CANDIDATES=3, the farthest must be dropped.
// WHAT: Rectangle route, rally biased toward left side. Farthest segment [1-2] (right side) should NOT be selected.
TEST_F(RtlProjectionCandidateSelectionTest, RectangleKeepsThreeClosestSegments)
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
	MissionRoutePlanner planner(provider);

	config.parameters.vehicle_projection_search_dist = 2000.f;
	config.parameters.safe_point_projection_search_dist = 2000.f;
	MissionRoutePlanner::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt);
	MissionRoutePlanner::ProjectionContext proj_ctx{};

	// WHEN: project vehicle and select safe point
	bool ok = planner.collectVehicleProjection(vehicle, 3, config, proj_ctx, reason);
	ASSERT_TRUE(ok);
	MissionRoutePlanner::Selection selection = planner.selectSafePoint(proj_ctx, config);

	// THEN: the farthest segment [1-2] (right side) should not be selected
	ASSERT_TRUE(selection.found);
	EXPECT_NE(selection.branch_off_segment.start.idx, 1);
}

// WHY: Multiple identical waypoints at a corner must not create false local minima that fill the candidate buffer and evict real projections.
// WHAT: L-shape with 8 duplicated corner waypoints. Duplicates must not fill buffer and evict the real non-corner projection.
TEST_F(RtlProjectionCandidateSelectionTest, DuplicateCornerWaypointsDoNotEvictValidCandidates)
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
	MissionRoutePlanner planner(provider);

	config.parameters.vehicle_projection_search_dist = 500.f;
	config.parameters.safe_point_projection_search_dist = 500.f;
	MissionRoutePlanner::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 200.f, kAlt);
	MissionRoutePlanner::ProjectionContext proj_ctx{};

	// WHEN: project vehicle and select safe point
	bool ok = planner.collectVehicleProjection(vehicle, 9, config, proj_ctx, reason);
	ASSERT_TRUE(ok);
	MissionRoutePlanner::Selection selection = planner.selectSafePoint(proj_ctx, config);

	// THEN: The real non-corner branch-off candidate survives the duplicate corner stack.
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.goal_type, MissionRoutePlanner::GoalType::SafePoint);
	EXPECT_EQ(selection.safe_point_index, 0);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 9);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 10);
}

// WHY: Projection should reject both non-finite and finite-but-invalid global positions early.
// WHAT: Invalid coordinates return false with FailureReason::NoValidGlobalPos.
/**
 * @brief Parameterized fixture for invalid vehicle-position rejection checks.
 */
class RtlProjectionInvalidVehiclePositionTest : public RtlProjectionEdgeCaseTest,
	public ::testing::WithParamInterface<std::pair<double, double>> {};

TEST_P(RtlProjectionInvalidVehiclePositionTest, RejectsInvalidVehiclePosition)
{
	const auto [lat, lon] = GetParam();

	// GIVEN: A simple 3-wp mission.
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};
	VectorProvider provider(mission, {});
	MissionRoutePlanner planner(provider);

	// WHEN: collectVehicleProjection is called with an invalid vehicle position.
	MissionRoutePlanner::Position vehicle{};
	vehicle.lat = lat;
	vehicle.lon = lon;
	vehicle.alt = kAlt;
	bool ok = planner.collectVehicleProjection(vehicle, 1, config, ctx, reason);

	// THEN: Projection fails before any segment selection occurs.
	EXPECT_FALSE(ok);
	EXPECT_EQ(reason, MissionRoutePlanner::FailureReason::NoValidGlobalPos);
}

INSTANTIATE_TEST_SUITE_P(
	InvalidVehiclePositions,
	RtlProjectionInvalidVehiclePositionTest,
	::testing::Values(
		std::make_pair(NAN, kBaseLon),
		std::make_pair(kBaseLat, NAN),
		std::make_pair(INFINITY, kBaseLon),
		std::make_pair(kBaseLat, INFINITY),
		std::make_pair(-INFINITY, kBaseLon),
		std::make_pair(kBaseLat, -INFINITY),
		std::make_pair(0.0, 0.0),
		std::make_pair(90.0, kBaseLon),
		std::make_pair(-90.0, kBaseLon),
		std::make_pair(kBaseLat, 180.0),
		std::make_pair(kBaseLat, -180.0),
		std::make_pair(91.0, kBaseLon),
		std::make_pair(kBaseLat, 181.0)
	)
);

// WHY: A mission with only one waypoint has no segments and cannot support projection.
// WHAT: Single-item mission, collectVehicleProjection should return false.
TEST_F(RtlProjectionEdgeCaseTest, SingleWaypointMissionFails)
{
	// GIVEN: single-waypoint mission
	std::vector<mission_item_s> mission = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
	};
	VectorProvider provider(mission, {});
	MissionRoutePlanner planner(provider);

	MissionRoutePlanner::Position vehicle = makePositionAbsolute(kBaseLat, kBaseLon, kAlt);

	// WHEN: attempt projection
	bool ok = planner.collectVehicleProjection(vehicle, 0, config, ctx, reason);

	// THEN: projection fails (no segments)
	EXPECT_FALSE(ok);
}

// WHY: A mission that zigzags creates many local minima; the candidate buffer (MAX_SEGMENT_CANDIDATES=3) must keep only the closest three and not overflow.
// WHAT: 8-waypoint zigzag pattern with a safe point near one segment. Verify selection.found is true.
TEST_F(RtlProjectionEdgeCaseTest, ZigzagMissionStressesCandidateBuffer)
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
	MissionRoutePlanner planner(provider);

	config.parameters.vehicle_projection_search_dist = 500.f;
	config.parameters.safe_point_projection_search_dist = 500.f;
	MissionRoutePlanner::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 600.f, kAlt);
	MissionRoutePlanner::ProjectionContext proj_ctx{};

	// WHEN: project vehicle and select safe point
	bool ok = planner.collectVehicleProjection(vehicle, 5, config, proj_ctx, reason);
	ASSERT_TRUE(ok);
	MissionRoutePlanner::Selection selection = planner.selectSafePoint(proj_ctx, config);

	// THEN: The closest central segment survives the candidate pruning.
	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.safe_point_index, 0);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 4);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 5);
}
