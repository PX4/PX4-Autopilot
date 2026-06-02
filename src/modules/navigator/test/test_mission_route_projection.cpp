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
 * @file test_mission_route_projection.cpp
 *
 * Mission-route projection and candidate-selection tests.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_projection.h"
#include "support/mission_route_test_helpers.h"
#include "test_mission_route_data.h"

#include <array>
#include <limits>
#include <vector>

using navigator_test::route_test_reference::kAlt;
using navigator_test::route_test_reference::kBaseLat;
using navigator_test::route_test_reference::kBaseLon;

class MissionRouteProjectionTestBase : public MissionRouteTestBase {};
class MissionRouteProjectionLocalSegmentTest : public MissionRouteProjectionTestBase {};
class MissionRouteProjectionCandidateSelectionTest : public MissionRouteProjectionTestBase {};
class MissionRouteProjectionEdgeCaseTest : public MissionRouteProjectionTestBase {};

// Prefer the segment that owns mission_index to keep route continuity.
TEST_F(MissionRouteProjectionLocalSegmentTest, PrefersCurrentMissionSegment)
{
	// 3-waypoint mission: straight north, then right turn; vehicle at 90N 10E.
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};
	VectorProvider provider(mission, {});
	MissionRoutePlanner planner(provider);

	mission_route::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 90.f, 10.f, kAlt);

	bool ok = collectVehicleProjection(planner, vehicle, 1, config, ctx, reason);

	ASSERT_TRUE(ok) << mission_route::failureReasonString(reason);
	EXPECT_EQ(ctx.route_projection.segment.start.idx, 0);
	EXPECT_EQ(ctx.route_projection.segment.end.idx, 1);
	EXPECT_NEAR(ctx.route_projection.dist.xtrack, 10.f, kDistanceTolerance);
}

class MissionRouteProjectionInvalidIndexTest : public MissionRouteProjectionTestBase,
	public ::testing::WithParamInterface<int> {};

// Invalid mission indices should fail before segment selection.
TEST_P(MissionRouteProjectionInvalidIndexTest, RejectsOutOfRangeMissionIndex)
{
	const int bad_index = GetParam();

	// 3-waypoint L-shaped mission; vehicle is near the final segment.
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};
	VectorProvider provider(mission, {});
	MissionRoutePlanner planner(provider);

	mission_route::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 60.f, kAlt);

	mission_route::ProjectionContext ctx_bad{};

	bool ok_bad = collectVehicleProjection(planner, vehicle, bad_index, config, ctx_bad, reason);

	EXPECT_FALSE(ok_bad);
	EXPECT_EQ(reason, mission_route::FailureReason::kInvalidRequest);
}

INSTANTIATE_TEST_SUITE_P(
	InvalidIndices,
	MissionRouteProjectionInvalidIndexTest,
	::testing::Values(-42, -1, 99)
);

// Relative mission altitudes need a finite home altitude to become AMSL.
TEST_F(MissionRouteProjectionLocalSegmentTest, RelativeAltitudeRequiresFiniteHomeAltitude)
{
	// NaN home altitude leaves the relative item without a valid AMSL position.
	mission_item_s mission_item = makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, 50.f);
	mission_item.altitude_is_relative = true;

	mission_route::Position position{};
	const float absolute_altitude = mission_route::getAbsoluteAltitudeForMissionItem(mission_item, NAN);

	EXPECT_FALSE(PX4_ISFINITE(absolute_altitude));
	EXPECT_FALSE(mission_route::extractMissionPosition(mission_item, NAN, position));
}

// Reverse route following changes which segment owns a boundary waypoint.
TEST_F(MissionRouteProjectionLocalSegmentTest, ReverseFlightPrefersReverseCurrentSegment)
{
	// Same L-shaped mission; vehicle sits just beyond waypoint 1 on [1-2].
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};
	VectorProvider provider(mission, {});
	MissionRoutePlanner planner(provider);
	config.state.is_flying_reverse = true;

	const mission_route::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 10.f, kAlt);

	bool ok = collectVehicleProjection(planner, vehicle, 1, config, ctx, reason);

	ASSERT_TRUE(ok) << mission_route::failureReasonString(reason);
	EXPECT_EQ(ctx.route_projection.segment.start.idx, 1);
	EXPECT_EQ(ctx.route_projection.segment.end.idx, 2);
}

// The stored DO_JUMP loop segment wins over a closer nominal segment.
TEST_F(MissionRouteProjectionLocalSegmentTest, PrefersStoredLoopAnchor)
{
	// Mission loops from waypoint 2 back to 0, then continues to waypoint 4.
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
	config.last_flown_loop_segment.start.idx = 2;
	config.last_flown_loop_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.end.idx = 0;
	config.last_flown_loop_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.is_loop = true;
	config.last_flown_loop_segment.loops_remaining = 1;

	// Vehicle is closer to a nominal segment, so this checks loop anchoring.
	mission_route::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 75.f, 10.f, kAlt);

	bool ok = collectVehicleProjection(planner, vehicle, 0, config, ctx, reason);

	ASSERT_TRUE(ok) << mission_route::failureReasonString(reason);
	EXPECT_TRUE(ctx.route_projection.segment.is_loop);
	EXPECT_EQ(ctx.route_projection.segment.start.idx, 2);
	EXPECT_EQ(ctx.route_projection.segment.end.idx, 0);
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

class MissionRouteProjectionDatasetTest : public MissionRouteProjectionTestBase,
	public ::testing::WithParamInterface<ProjectionDatasetCase> {};

// Dataset cases pin expected projected segments without duplicating test bodies.
TEST_P(MissionRouteProjectionDatasetTest, SelectsExpectedSegment)
{
	// Each row supplies the dataset, vehicle pose, mission index, and expected segment.
	const ProjectionDatasetCase &scenario = GetParam();
	const auto mission = scenario.use_corner_dataset ? corner_dataset::mission() : default_dataset::mission();
	const auto safe_points = scenario.use_corner_dataset ? corner_dataset::safePoints() : default_dataset::safePoints();
	VectorProvider provider(mission, safe_points);
	MissionRoutePlanner planner(provider);
	const mission_route::Position vehicle = makePositionAbsolute(scenario.lat, scenario.lon, scenario.alt);

	bool ok = collectVehicleProjection(planner, vehicle, scenario.mission_index, config, ctx, reason);

	ASSERT_TRUE(ok) << mission_route::failureReasonString(reason);
	EXPECT_EQ(ctx.route_projection.segment.start.idx, scenario.expected_start_idx);
	EXPECT_EQ(ctx.route_projection.segment.end.idx, scenario.expected_end_idx);
}

INSTANTIATE_TEST_SUITE_P(
	DatasetScenarios,
	MissionRouteProjectionDatasetTest,
	::testing::ValuesIn(kProjectionDatasetCases),
	[](const ::testing::TestParamInfo<ProjectionDatasetCase> &param_info)
{
	return param_info.param.name;
}
);

// A rally behind takeoff and zero-length stacks at takeoff/land must still expose the branch-off point.
struct EndpointLocalMinimumCase {
	const char *name;
	std::vector<mission_item_s> mission;
	std::vector<mission_item_s> safe_points;
	mission_route::Position vehicle;
	int32_t mission_index;
	double expected_projection_lat;
	double expected_projection_lon;
	int expected_branch_start; // -1 to skip
	int expected_branch_end;   // -1 to skip
};

class MissionRouteProjectionEndpointLocalMinimumTest : public MissionRouteProjectionTestBase,
	public ::testing::WithParamInterface<EndpointLocalMinimumCase> {};

TEST_P(MissionRouteProjectionEndpointLocalMinimumTest, ExposesBranchOffAtEndpoint)
{
	const EndpointLocalMinimumCase &scenario = GetParam();
	VectorProvider provider(scenario.mission, scenario.safe_points);
	MissionRoutePlanner planner(provider);
	mission_route::ProjectionContext proj_ctx{};

	ASSERT_TRUE(collectVehicleProjection(planner, scenario.vehicle, scenario.mission_index, config, proj_ctx, reason))
			<< mission_route::failureReasonString(reason);

	const mission_route::GoalSelection selection = planner.selectSafePoint(proj_ctx, config);

	ASSERT_TRUE(selection.found);
	EXPECT_NEAR(selection.branch_off_projection.lat, scenario.expected_projection_lat, kLatLonToleranceDeg);
	EXPECT_NEAR(selection.branch_off_projection.lon, scenario.expected_projection_lon, kLatLonToleranceDeg);

	if (scenario.expected_branch_start >= 0) {
		EXPECT_EQ(selection.branch_off_segment.start.idx, scenario.expected_branch_start);
	}

	if (scenario.expected_branch_end >= 0) {
		EXPECT_EQ(selection.branch_off_segment.end.idx, scenario.expected_branch_end);
	}
}

INSTANTIATE_TEST_SUITE_P(
	EndpointLocalMinima,
	MissionRouteProjectionEndpointLocalMinimumTest,
	::testing::Values(
		// Rally behind takeoff branches from the first segment [0-1] at the takeoff point.
EndpointLocalMinimumCase{
	"RallyBehindTakeoff",
	{
		makeTakeoffItem(47.0000000, 8.0000000, 500.f),
		makePositionItem(47.0000000, 8.0010000, 500.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(47.0000000, 8.0020000, 500.f),
	},
	{makeSafePointAbsolute(47.0000000, 7.9990000, 500.f)},
	makePositionAbsolute(47.0000000, 8.0000000, 500.f), 0,
	47.0, 8.0, 0, 1
},
// Zero-length stack at takeoff must not hide the branch-off point.
EndpointLocalMinimumCase{
	"StackedWaypointAboveTakeoff",
	{
		makeTakeoffItem(47.0000000, 8.0000000, 500.f),
		makePositionItem(47.0000000, 8.0000000, 550.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(47.0000000, 8.0020000, 500.f),
	},
	{makeSafePointAbsolute(47.0000000, 7.9990000, 500.f)},
	makePositionAbsolute(47.0000000, 8.0000000, 500.f), 0,
	47.0, 8.0, -1, -1
},
// Zero-length stack at LAND must not break safe-point projection.
EndpointLocalMinimumCase{
	"StackedWaypointAboveLand",
	{
		makeTakeoffItem(47.0000000, 8.0000000, 500.f),
		makePositionItem(47.0000000, 8.0010000, 500.f),
		makePositionItem(47.0000000, 8.0020000, 550.f),
		makeLandItem(47.0000000, 8.0020000, 500.f),
	},
	{makeSafePointAbsolute(47.0000000, 8.0030000, 500.f)},
	makePositionAbsolute(47.0000000, 8.0010000, 500.f), 2,
	47.0, 8.002, -1, -1
}
	),
[](const ::testing::TestParamInfo<EndpointLocalMinimumCase> &param_info)
{
	return param_info.param.name;
}
);

// Intermediate waypoints on a straight line are not real local minima.
TEST_F(MissionRouteProjectionCandidateSelectionTest, StraightLineIgnoresNonMinCorners)
{
	// Ten waypoints run north in a straight line; safe point is offset mid-route.
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

	mission_route::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 450.f, 0.f, kAlt);
	mission_route::ProjectionContext proj_ctx{};

	bool ok = collectVehicleProjection(planner, vehicle, 5, config, proj_ctx, reason);
	ASSERT_TRUE(ok) << mission_route::failureReasonString(reason);
	mission_route::GoalSelection selection = planner.selectSafePoint(proj_ctx, config);

	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 4);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 5);
}

// Four side candidates fit into a three-slot buffer by dropping the far side.
TEST_F(MissionRouteProjectionCandidateSelectionTest, RectangleKeepsThreeClosestSegments)
{
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

	// Bias the rally point left, away from segment [1-2].
	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 500.f, -50.f, kAlt),
	};
	VectorProvider provider(mission, safe_points);
	MissionRoutePlanner planner(provider);

	config.parameters.vehicle_projection_search_dist = 2000.f;
	config.parameters.safe_point_projection_search_dist = 2000.f;
	mission_route::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt);
	mission_route::ProjectionContext proj_ctx{};

	bool ok = collectVehicleProjection(planner, vehicle, 3, config, proj_ctx, reason);
	ASSERT_TRUE(ok) << mission_route::failureReasonString(reason);
	mission_route::GoalSelection selection = planner.selectSafePoint(proj_ctx, config);

	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.safe_point_index, 0);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 3);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 4);
	EXPECT_NE(selection.branch_off_segment.start.idx, 1);
	EXPECT_TRUE(selection.branch_off_projection.valid());
}

// Duplicate corner waypoints must not evict the real branch-off candidate.
TEST_F(MissionRouteProjectionCandidateSelectionTest, DuplicateCornerWaypointsDoNotEvictValidCandidates)
{
	// Route turns at 200N 200E and stacks duplicates before the final leg.
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),     // 0
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f,   0.f, kAlt),     // 1
	};

	// Stack duplicate waypoints at the same corner position.
	for (int i = 0; i < 8; ++i) {
		mission.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 200.f, kAlt));
	}

	// The real segment after the duplicated corner still needs to survive.
	mission.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 200.f, kAlt));

	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 250.f, kAlt),
	};
	VectorProvider provider(mission, safe_points);
	MissionRoutePlanner planner(provider);

	config.parameters.vehicle_projection_search_dist = 500.f;
	config.parameters.safe_point_projection_search_dist = 500.f;
	mission_route::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 200.f, kAlt);
	mission_route::ProjectionContext proj_ctx{};

	bool ok = collectVehicleProjection(planner, vehicle, 9, config, proj_ctx, reason);
	ASSERT_TRUE(ok) << mission_route::failureReasonString(reason);
	mission_route::GoalSelection selection = planner.selectSafePoint(proj_ctx, config);

	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.goal_type, mission_route::GoalType::kSafePoint);
	EXPECT_EQ(selection.safe_point_index, 0);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 9);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 10);
}

class MissionRouteProjectionInvalidVehiclePositionTest : public MissionRouteProjectionEdgeCaseTest,
	public ::testing::WithParamInterface<std::pair<double, double>> {};

// Reject non-finite and finite-but-invalid vehicle coordinates early.
TEST_P(MissionRouteProjectionInvalidVehiclePositionTest, RejectsInvalidVehiclePosition)
{
	const auto [lat, lon] = GetParam();

	// Simple 3-waypoint L-shaped mission; only the vehicle coordinate changes.
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};
	VectorProvider provider(mission, {});
	MissionRoutePlanner planner(provider);

	mission_route::Position vehicle{};
	vehicle.lat = lat;
	vehicle.lon = lon;
	vehicle.alt = kAlt;
	bool ok = collectVehicleProjection(planner, vehicle, 1, config, ctx, reason);

	EXPECT_FALSE(ok);
	EXPECT_EQ(reason, mission_route::FailureReason::kNoValidGlobalPos);
}

INSTANTIATE_TEST_SUITE_P(
	InvalidVehiclePositions,
	MissionRouteProjectionInvalidVehiclePositionTest,
	::testing::Values(
		std::make_pair(NAN, kBaseLon),
		std::make_pair(kBaseLat, NAN),
		std::make_pair(INFINITY, kBaseLon),
		std::make_pair(kBaseLat, INFINITY),
		std::make_pair(-INFINITY, kBaseLon),
		std::make_pair(kBaseLat, -INFINITY),
		std::make_pair(0.0, 0.0),
		std::make_pair(91.0, kBaseLon),
		std::make_pair(kBaseLat, 181.0)
	)
);

// Single-item mission: no start/end segment pair exists.
TEST_F(MissionRouteProjectionEdgeCaseTest, SingleWaypointMissionFails)
{
	std::vector<mission_item_s> mission = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
	};
	VectorProvider provider(mission, {});
	MissionRoutePlanner planner(provider);

	mission_route::Position vehicle = makePositionAbsolute(kBaseLat, kBaseLon, kAlt);

	bool ok = collectVehicleProjection(planner, vehicle, 0, config, ctx, reason);

	EXPECT_FALSE(ok);
}

// Zigzag geometry stresses candidate-buffer pruning.
TEST_F(MissionRouteProjectionEdgeCaseTest, ZigzagMissionStressesCandidateBuffer)
{
	// Eight waypoints advance east while alternating between south and north.
	std::vector<mission_item_s> mission;

	for (int i = 0; i < 8; ++i) {
		float north = (i % 2 == 0) ? 0.f : 200.f;
		float east  = static_cast<float>(i * 150);
		mission.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, north, east, kAlt));
	}

	// Keep the safe point near the center of the zigzag.
	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 100.f, 680.f, kAlt),
	};
	VectorProvider provider(mission, safe_points);
	MissionRoutePlanner planner(provider);

	config.parameters.vehicle_projection_search_dist = 500.f;
	config.parameters.safe_point_projection_search_dist = 500.f;
	mission_route::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 600.f, kAlt);
	mission_route::ProjectionContext proj_ctx{};

	bool ok = collectVehicleProjection(planner, vehicle, 5, config, proj_ctx, reason);
	ASSERT_TRUE(ok) << mission_route::failureReasonString(reason);
	mission_route::GoalSelection selection = planner.selectSafePoint(proj_ctx, config);

	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.safe_point_index, 0);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 4);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 5);
}

class MissionRouteProjectionCandidateBufferTestPeer : public mission_route::MissionRouteProjection
{
public:
	explicit MissionRouteProjectionCandidateBufferTestPeer(const mission_route::Provider &provider) :
		mission_route::MissionRouteProjection(provider) {}

	void insertCandidateSortedForTest(mission_route::ProjectionCandidateBuffer &candidate_buffer,
					  const mission_route::RouteProjectionCandidate &candidate) const
	{
		insertCandidateSorted(candidate_buffer, candidate);
	}

	void pruneProjectionCandidatesForTest(mission_route::ProjectionCandidateBuffer &candidate_buffer, float xtrack_limit) const
	{
		pruneProjectionCandidates(candidate_buffer, xtrack_limit);
	}

	bool validateCandidateForTest(const mission_route::RouteProjectionCandidate &candidate) const
	{
		return validateCandidate(candidate);
	}

	bool isIndexInProjectionSegmentForTest(const mission_route::Segment &projection_segment, int32_t mission_index,
					       bool is_flying_reverse) const
	{
		return isIndexInProjectionSegment(projection_segment, mission_index, is_flying_reverse);
	}
};

class MissionRouteProjectionCandidateBufferTest : public ::testing::Test
{
protected:
	static constexpr int kCapacity = mission_route::kMaxSegmentCandidates;
	static constexpr uint8_t kCorruptCount = static_cast<uint8_t>(kCapacity + 1);

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

	static mission_route::RouteProjectionCandidate makeCandidate(int32_t id, float xtrack)
	{
		mission_route::RouteProjectionCandidate candidate{};
		candidate.segment.start.idx = id;
		candidate.dist.xtrack = xtrack;
		return candidate;
	}

	static mission_route::RouteProjectionCandidate makeValidCandidate()
	{
		mission_route::RouteProjectionCandidate candidate{};
		candidate.segment.start.idx = 2;
		candidate.segment.start.nav_cmd = NAV_CMD_WAYPOINT;
		candidate.segment.end.idx = 4;
		candidate.segment.end.nav_cmd = NAV_CMD_WAYPOINT;
		candidate.segment_positions.start = makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt);
		candidate.segment_positions.end = makePositionFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt);
		candidate.projection = makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt);
		candidate.dist.xtrack = 5.0f;
		candidate.dist.route_along = 100.0f;
		candidate.dist.segment_length = 200.0f;
		candidate.dist.segment_along = 100.0f;
		return candidate;
	}

	static void expectCandidateIds(const mission_route::ProjectionCandidateBuffer &buffer,
				       const std::vector<int32_t> &expected_ids)
	{
		ASSERT_EQ(buffer.count, expected_ids.size());

		for (size_t i = 0; i < expected_ids.size(); ++i) {
			EXPECT_EQ(buffer.candidates[i].segment.start.idx, expected_ids[i]);
		}
	}

	// Fills the buffer with ids 1..kCapacity at xtrack 1..kCapacity (worst entry is xtrack kCapacity).
	void fillBufferToCapacity(mission_route::ProjectionCandidateBuffer &buffer)
	{
		for (int32_t i = 0; i < kCapacity; ++i) {
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

	VectorMissionRouteProvider provider{makeMissionItems(), {}};
	MissionRouteProjectionCandidateBufferTestPeer planner{provider};
};

// Sorted insertion: closer candidates go to the front, equal xtrack is stable, and a full buffer
// keeps only the closest few (also when the stored count is corrupt).
TEST_F(MissionRouteProjectionCandidateBufferTest, InsertCandidateSortedKeepsClosestCandidatesInStableOrder)
{
	{
		// Empty insert, front-shift for a closer candidate, and a stable equal-xtrack append.
		SCOPED_TRACE("sorted stable insertion");
		mission_route::ProjectionCandidateBuffer buffer{};
		planner.insertCandidateSortedForTest(buffer, makeCandidate(1, 5.0f));
		planner.insertCandidateSortedForTest(buffer, makeCandidate(2, 3.0f));
		planner.insertCandidateSortedForTest(buffer, makeCandidate(3, 5.0f));
		expectCandidateIds(buffer, {2, 1, 3});
	}
	{
		// A full buffer rejects a candidate worse than every entry.
		SCOPED_TRACE("full buffer rejects worse candidate");
		mission_route::ProjectionCandidateBuffer buffer{};
		fillBufferToCapacity(buffer);
		planner.insertCandidateSortedForTest(buffer, makeCandidate(99, static_cast<float>(kCapacity) + 1.f));
		expectCandidateIds(buffer, expectedSequentialIds(1, kCapacity));
	}
	{
		// A new closest candidate takes the front and drops the worst.
		SCOPED_TRACE("full buffer drops worst for closer candidate");
		mission_route::ProjectionCandidateBuffer buffer{};
		fillBufferToCapacity(buffer);
		planner.insertCandidateSortedForTest(buffer, makeCandidate(0, 0.5f));
		std::vector<int32_t> expected_ids{0};
		const std::vector<int32_t> retained_ids = expectedSequentialIds(1, kCapacity - 1);
		expected_ids.insert(expected_ids.end(), retained_ids.begin(), retained_ids.end());
		expectCandidateIds(buffer, expected_ids);
	}
	{
		// A candidate just better than the worst lands in the last slot and drops the worst.
		SCOPED_TRACE("full buffer keeps new candidate in last slot");
		mission_route::ProjectionCandidateBuffer buffer{};
		fillBufferToCapacity(buffer);
		planner.insertCandidateSortedForTest(buffer, makeCandidate(25, static_cast<float>(kCapacity) - 0.5f));
		std::vector<int32_t> expected_ids = expectedSequentialIds(1, kCapacity - 1);
		expected_ids.push_back(25);
		expectCandidateIds(buffer, expected_ids);
	}
	{
		// A count above capacity is clamped before insertion.
		SCOPED_TRACE("overfull count is clamped");
		mission_route::ProjectionCandidateBuffer buffer{};
		buffer.count = kCorruptCount;

		for (int32_t i = 0; i < kCapacity; ++i) {
			buffer.candidates[i] = makeCandidate(i + 1, static_cast<float>(i + 1));
		}

		planner.insertCandidateSortedForTest(buffer, makeCandidate(0, 0.5f));
		std::vector<int32_t> expected_ids{0};
		const std::vector<int32_t> retained_ids = expectedSequentialIds(1, kCapacity - 1);
		expected_ids.insert(expected_ids.end(), retained_ids.begin(), retained_ids.end());
		expectCandidateIds(buffer, expected_ids);
	}
}

// Pruning removes candidates outside the xtrack window and clamps a corrupt over-capacity count.
TEST_F(MissionRouteProjectionCandidateBufferTest, PruneProjectionCandidatesTrimsAndClamps)
{
	{
		SCOPED_TRACE("removes tail outside limit");
		mission_route::ProjectionCandidateBuffer buffer{};
		planner.insertCandidateSortedForTest(buffer, makeCandidate(1, 1.0f));
		planner.insertCandidateSortedForTest(buffer, makeCandidate(2, 2.0f));
		planner.insertCandidateSortedForTest(buffer, makeCandidate(3, 4.0f));
		planner.pruneProjectionCandidatesForTest(buffer, 2.5f);
		expectCandidateIds(buffer, {1, 2});
	}
	{
		// A single in-limit candidate survives.
		SCOPED_TRACE("single in-limit candidate kept");
		mission_route::ProjectionCandidateBuffer buffer{};
		buffer.count = 1;
		buffer.candidates[0] = makeCandidate(7, 4.0f);
		planner.pruneProjectionCandidatesForTest(buffer, 5.0f);
		expectCandidateIds(buffer, {7});
	}
	{
		// A single out-of-limit candidate clears the buffer.
		SCOPED_TRACE("single out-of-limit candidate cleared");
		mission_route::ProjectionCandidateBuffer buffer{};
		buffer.count = 1;
		buffer.candidates[0] = makeCandidate(7, 8.0f);
		planner.pruneProjectionCandidatesForTest(buffer, 5.0f);
		EXPECT_EQ(buffer.count, 0U);
	}
	{
		// A count above capacity is clamped before pruning.
		SCOPED_TRACE("overfull count is clamped");
		mission_route::ProjectionCandidateBuffer buffer{};
		buffer.count = kCorruptCount;

		for (int32_t i = 0; i < kCapacity; ++i) {
			buffer.candidates[i] = makeCandidate(i + 1, static_cast<float>(i + 1));
		}

		planner.pruneProjectionCandidatesForTest(buffer, 2.5f);
		expectCandidateIds(buffer, {1, 2});
	}
}

// Candidate validation accepts nominal and reverse-index loop segments.
TEST_F(MissionRouteProjectionCandidateBufferTest, ValidateCandidateAcceptsValidNominalAndLoopCandidates)
{
	// Start with a valid nominal candidate.
	mission_route::RouteProjectionCandidate candidate = makeValidCandidate();

	// Validate it as nominal, then as a loop segment.
	const bool nominal_valid = planner.validateCandidateForTest(candidate);
	candidate.segment.start.idx = 7;
	candidate.segment.end.idx = 2;
	candidate.segment.is_loop = true;
	const bool loop_valid = planner.validateCandidateForTest(candidate);

	// Both variants are accepted.
	EXPECT_TRUE(nominal_valid);
	EXPECT_TRUE(loop_valid);
}

// Candidate validation rejects corrupt geometry before selection sees it.
TEST_F(MissionRouteProjectionCandidateBufferTest, ValidateCandidateRejectsInvalidCandidates)
{
	// Valid baseline
	const mission_route::RouteProjectionCandidate base = makeValidCandidate();
	const float nanf = std::numeric_limits<float>::quiet_NaN();
	const double nand = std::numeric_limits<double>::quiet_NaN();

	auto expect_invalid = [&](const char *label, auto &&mutator) {
		mission_route::RouteProjectionCandidate candidate = base;
		ASSERT_TRUE(planner.validateCandidateForTest(candidate)) << "Baseline corrupted!";
		mutator(candidate);
		EXPECT_FALSE(planner.validateCandidateForTest(candidate)) << label;
	};

	// Apply each invalid mutation independently.
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
	expect_invalid("negative along", [](auto & candidate) { candidate.dist.route_along = -5.0f; });
	expect_invalid("nan along", [&](auto & candidate) { candidate.dist.route_along = nanf; });
	expect_invalid("negative segment length", [](auto & candidate) { candidate.dist.segment_length = -3.0f; });
	expect_invalid("nan segment length", [&](auto & candidate) { candidate.dist.segment_length = nanf; });
	expect_invalid("negative distance on segment", [](auto & candidate) { candidate.dist.segment_along = -0.1f; });
	expect_invalid("nan distance on segment", [&](auto & candidate) { candidate.dist.segment_along = nanf; });
	expect_invalid("distance on segment beyond length", [](auto & candidate) { candidate.dist.segment_along = candidate.dist.segment_length + 1.0f; });
	expect_invalid("nan segment start lat", [&](auto & candidate) { candidate.segment_positions.start.lat = nand; });
	expect_invalid("nan segment end lon", [&](auto & candidate) { candidate.segment_positions.end.lon = nand; });
	expect_invalid("nan proj lat", [&](auto & candidate) { candidate.projection.lat = nand; });
	expect_invalid("nan proj lon", [&](auto & candidate) { candidate.projection.lon = nand; });
	expect_invalid("nan proj alt", [&](auto & candidate) { candidate.projection.alt = nanf; });
}

// Segment ownership differs at nominal and reverse boundaries.
TEST_F(MissionRouteProjectionCandidateBufferTest, IsIndexInProjectionSegmentHandlesDirectionBoundaries)
{
	// Segment spans mission indices 2 through 4.
	mission_route::Segment segment{};
	segment.start.idx = 2;
	segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	segment.end.idx = 4;
	segment.end.nav_cmd = NAV_CMD_WAYPOINT;

	auto expect_match = [&](int32_t mission_index, bool reverse, bool expected) {
		EXPECT_EQ(planner.isIndexInProjectionSegmentForTest(segment, mission_index, reverse), expected)
				<< "mission_index=" << mission_index << " reverse=" << reverse;
	};

	// Nominal owns [3, 4] (when targeting 2, it's the prev seg)
	expect_match(1, false, false);
	expect_match(2, false, false);
	expect_match(3, false, true);
	expect_match(4, false, true);
	expect_match(5, false, false);
	expect_match(-1, false, false);
	// reverse owns [2, 3] (when targeting 4, it's the prev seg)
	expect_match(1, true, false);
	expect_match(2, true, true);
	expect_match(3, true, true);
	expect_match(4, true, false);
	expect_match(5, true, false);
	expect_match(-1, true, false);
}
