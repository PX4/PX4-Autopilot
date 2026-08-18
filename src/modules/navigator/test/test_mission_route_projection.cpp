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
 * Mission-route scanner, projection, and candidate-selection tests.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_projection.h"
#include "support/mission_route_test_helpers.h"
#include "test_mission_route_data.h"

#include <array>
#include <cmath>
#include <vector>

using navigator_test::route_test_reference::kAlt;
using navigator_test::route_test_reference::kBaseLat;
using navigator_test::route_test_reference::kBaseLon;

static mission_route::PlannerConfig defaultProjectionConfig()
{
	mission_route::PlannerConfig config{};
	config.parameters.vehicle_projection_search_dist = 60.f;
	config.parameters.safe_point_projection_search_dist = 60.f;
	config.parameters.acceptance_radius = 10.f;
	config.parameters.direct_acceptance_radius = 10.f;
	config.parameters.altitude_acceptance_radius = 10.f;
	config.parameters.home_altitude_amsl = 500.f;
	return config;
}

class MissionRouteProjectionTestBase : public ::testing::Test
{
protected:
	mission_route::ProjectionReferenceBatch singleReferenceBatch(const mission_route::Position &position) const
	{
		mission_route::ProjectionReferenceBatch batch{};
		batch.count = 1;
		batch.items[0].position = position;
		return batch;
	}

	mission_route::ProjectionScanRequest scanRequest(float xtrack_margin_m) const
	{
		mission_route::ProjectionScanRequest request{};
		request.home_altitude_amsl = config.parameters.home_altitude_amsl;
		request.xtrack_margin_m = xtrack_margin_m;
		return request;
	}

	const mission_route::RouteProjectionCandidate *findCandidate(
		const mission_route::ProjectionCandidateBuffer &candidate_buffer,
		int32_t start_index, int32_t end_index) const
	{
		for (uint8_t i = 0; i < candidate_buffer.count; ++i) {
			const mission_route::RouteProjectionCandidate &candidate = candidate_buffer.candidates[i];

			if (candidate.segment.start.idx == start_index && candidate.segment.end.idx == end_index) {
				return &candidate;
			}
		}

		return nullptr;
	}

	mission_route::PlannerConfig config{defaultProjectionConfig()};
};

class MissionRouteProjectionLocalSegmentTest : public MissionRouteProjectionTestBase {};
class MissionRouteProjectionCandidateSelectionTest : public MissionRouteProjectionTestBase {};
class MissionRouteProjectionEdgeCaseTest : public MissionRouteProjectionTestBase {};
class RouteSegmentCursorTest : public MissionRouteProjectionTestBase {};

class MissionLoadCountingProvider : public mission_route::Provider
{
public:
	explicit MissionLoadCountingProvider(const mission_route::Provider &delegate) : _delegate(delegate) {}

	int missionCount() const override { return _delegate.missionCount(); }

	bool loadMissionItem(int index, mission_item_s &mission_item) const override
	{
		++_mission_item_load_count;
		return _delegate.loadMissionItem(index, mission_item);
	}

	int safePointCount() const override { return _delegate.safePointCount(); }

	bool loadSafePointItem(int index, mission_item_s &safe_point_item) const override
	{
		return _delegate.loadSafePointItem(index, safe_point_item);
	}

	int missionItemLoadCount() const { return _mission_item_load_count; }

private:
	const mission_route::Provider &_delegate;
	mutable int _mission_item_load_count{0};
};

// When two route legs are nearby, prefer the one that owns mission_index over the geometrically closer leg.
TEST_F(MissionRouteProjectionLocalSegmentTest, PrefersCurrentMissionSegmentOverCloserAlternative)
{
	// 3 -------- 2
	//            |
	//      P     |
	// 0 -------- 1
	const std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,    0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f, 1000.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 1000.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,    0.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission);
	mission_route::MissionRouteProjection projection(provider);
	auto batch = singleReferenceBatch({});
	const mission_route::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 500.f, kAlt);
	config.parameters.vehicle_projection_search_dist = 150.f;

	mission_route::ProjectionContext projection_context{};
	const mission_route::FailureReason status =
		projection.collectVehicleProjection(vehicle, 3, config, batch, projection_context);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_NE(findCandidate(batch.items[0].candidate_buffer, 0, 1), nullptr);
	EXPECT_NE(findCandidate(batch.items[0].candidate_buffer, 2, 3), nullptr);
	EXPECT_EQ(projection_context.route_projection.segment.start.idx, 2);
	EXPECT_EQ(projection_context.route_projection.segment.end.idx, 3);
	EXPECT_NEAR(projection_context.route_projection.dist.xtrack, 90.f, kDistanceTolerance);
}

class MissionRouteProjectionInvalidIndexTest : public MissionRouteProjectionTestBase,
	public ::testing::WithParamInterface<int32_t> {};

// Invalid mission indices fail.
TEST_P(MissionRouteProjectionInvalidIndexTest, RejectsOutOfRangeMissionIndex)
{
	const std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission);
	mission_route::MissionRouteProjection projection(provider);
	auto batch = singleReferenceBatch({});
	const mission_route::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 60.f, kAlt);

	mission_route::ProjectionContext projection_context{};
	projection_context.mission_index = 42;
	const mission_route::FailureReason status =
		projection.collectVehicleProjection(vehicle, GetParam(), config, batch, projection_context);

	EXPECT_EQ(status, mission_route::FailureReason::kInvalidRequest);
	EXPECT_EQ(projection_context.mission_index, -1);
}

INSTANTIATE_TEST_SUITE_P(
	InvalidIndices,
	MissionRouteProjectionInvalidIndexTest,
	::testing::Values(-1, 3)
);

// Relative mission altitudes need a finite home altitude to become AMSL.
TEST_F(MissionRouteProjectionLocalSegmentTest, RelativeAltitudeRequiresFiniteHomeAltitude)
{
	mission_item_s relative_item = makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, 50.f);
	relative_item.altitude_is_relative = true;
	const std::vector<mission_item_s> relative_mission = {
		relative_item,
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
	};
	VectorProvider relative_provider = makeRouteProvider(relative_mission);
	mission_route::RouteSegmentCursor relative_cursor(relative_provider, NAN);

	EXPECT_FALSE(relative_cursor.init());
	EXPECT_TRUE(relative_cursor.failed());
	EXPECT_EQ(relative_cursor.failureReason(), mission_route::FailureReason::kPositionItemInvalid);

	// Absolute altitude missions do not depend on home altitude.
	const std::vector<mission_item_s> absolute_mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
	};
	VectorProvider absolute_provider = makeRouteProvider(absolute_mission);
	mission_route::RouteSegmentCursor absolute_cursor(absolute_provider, NAN);
	mission_route::RouteSegmentView segment{};

	ASSERT_TRUE(absolute_cursor.init()) << mission_route::failureReasonString(absolute_cursor.failureReason());
	EXPECT_TRUE(absolute_cursor.next(segment));
	EXPECT_FALSE(absolute_cursor.failed());
}

// Reverse route following changes which segment owns a boundary waypoint.
TEST_F(MissionRouteProjectionLocalSegmentTest, ReverseFlightPrefersReverseCurrentSegment)
{
	const std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission);
	mission_route::MissionRouteProjection projection(provider);
	auto batch = singleReferenceBatch({});
	config.state.is_flying_reverse = true;
	const mission_route::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 10.f, kAlt);

	mission_route::ProjectionContext projection_context{};
	const mission_route::FailureReason status =
		projection.collectVehicleProjection(vehicle, 1, config, batch, projection_context);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(projection_context.route_projection.segment.start.idx, 1);
	EXPECT_EQ(projection_context.route_projection.segment.end.idx, 2);
}

// The stored DO_JUMP loop segment wins over a closer nominal segment.
TEST_F(MissionRouteProjectionLocalSegmentTest, PrefersStoredLoopAnchor)
{
	const std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makeDoJump(0, 2, 0),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f,   0.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission);
	mission_route::MissionRouteProjection projection(provider);
	auto batch = singleReferenceBatch({});

	config.active_jump_anchor.start_index = 2;
	config.active_jump_anchor.target_index = 0;

	const mission_route::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 75.f, 10.f, kAlt);
	mission_route::ProjectionContext projection_context{};
	const mission_route::FailureReason status =
		projection.collectVehicleProjection(vehicle, 0, config, batch, projection_context);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_TRUE(projection_context.route_projection.segment.is_loop);
	EXPECT_EQ(projection_context.route_projection.segment.start.idx, 2);
	EXPECT_EQ(projection_context.route_projection.segment.end.idx, 0);
}

struct ProjectionDatasetCase {
	const char *name;
	bool use_corner_dataset;
	double lat;
	double lon;
	float alt;
	int32_t mission_index;
	int32_t expected_start_idx;
	int32_t expected_end_idx;
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

TEST_P(MissionRouteProjectionDatasetTest, SelectsExpectedSegment)
{
	const ProjectionDatasetCase &scenario = GetParam();
	const std::vector<mission_item_s> mission =
		scenario.use_corner_dataset ? corner_dataset::mission() : default_dataset::mission();
	VectorProvider provider = makeRouteProvider(mission);
	mission_route::MissionRouteProjection projection(provider);
	auto batch = singleReferenceBatch({});
	const mission_route::Position vehicle = makePositionAbsolute(scenario.lat, scenario.lon, scenario.alt);

	mission_route::ProjectionContext projection_context{};
	const mission_route::FailureReason status =
		projection.collectVehicleProjection(vehicle, scenario.mission_index, config, batch, projection_context);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_EQ(projection_context.route_projection.segment.start.idx, scenario.expected_start_idx);
	EXPECT_EQ(projection_context.route_projection.segment.end.idx, scenario.expected_end_idx);
}

INSTANTIATE_TEST_SUITE_P(
	RealWorldGeometry,
	MissionRouteProjectionDatasetTest,
	::testing::ValuesIn(kProjectionDatasetCases),
	[](const ::testing::TestParamInfo<ProjectionDatasetCase> &param_info)
{
	return param_info.param.name;
}
);

enum class SegmentLengthExpectation {
	kNonzero,
	kZero,
};

struct EndpointLocalMinimumCase {
	const char *name;
	std::vector<mission_item_s> mission;
	mission_route::Position reference;
	double expected_projection_lat;
	double expected_projection_lon;
	int32_t expected_segment_start;
	int32_t expected_segment_end;
	SegmentLengthExpectation expected_segment_length;
};

class MissionRouteProjectionEndpointLocalMinimumTest : public MissionRouteProjectionTestBase,
	public ::testing::WithParamInterface<EndpointLocalMinimumCase> {};

// Route endpoints and stacked endpoint positions remain valid scanner minima.
TEST_P(MissionRouteProjectionEndpointLocalMinimumTest, KeepsEndpointCandidate)
{
	const EndpointLocalMinimumCase &scenario = GetParam();
	VectorProvider provider = makeRouteProvider(scenario.mission);
	mission_route::MissionRouteProjection projection(provider);
	auto batch = singleReferenceBatch(scenario.reference);

	mission_route::RouteDistanceSummary distance_summary{};
	const mission_route::FailureReason status =
		projection.findProjectionCandidates(scanRequest(config.parameters.safe_point_projection_search_dist), batch,
				distance_summary);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	const mission_route::RouteProjectionCandidate *candidate =
		findCandidate(batch.items[0].candidate_buffer, scenario.expected_segment_start, scenario.expected_segment_end);
	ASSERT_NE(candidate, nullptr);
	EXPECT_NEAR(candidate->projection.lat, scenario.expected_projection_lat, kLatLonToleranceDeg);
	EXPECT_NEAR(candidate->projection.lon, scenario.expected_projection_lon, kLatLonToleranceDeg);
	EXPECT_EQ(candidate->dist.segment_length <= FLT_EPSILON,
		  scenario.expected_segment_length == SegmentLengthExpectation::kZero);
}

INSTANTIATE_TEST_SUITE_P(
	EndpointLocalMinima,
	MissionRouteProjectionEndpointLocalMinimumTest,
	::testing::Values(
EndpointLocalMinimumCase{
	"RallyBehindTakeoff",
	{
		makeTakeoffItem(47.0000000, 8.0000000, 500.f),
		makePositionItem(47.0000000, 8.0010000, 500.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(47.0000000, 8.0020000, 500.f),
	},
	makePositionAbsolute(47.0000000, 7.9990000, 500.f),
	47.0, 8.0, 0, 1, SegmentLengthExpectation::kNonzero
},
EndpointLocalMinimumCase{
	"StackedWaypointAboveTakeoff",
	{
		makeTakeoffItem(47.0000000, 8.0000000, 500.f),
		makePositionItem(47.0000000, 8.0000000, 550.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(47.0000000, 8.0020000, 500.f),
	},
	makePositionAbsolute(47.0000000, 7.9990000, 500.f),
	47.0, 8.0, 0, 1, SegmentLengthExpectation::kZero
},
EndpointLocalMinimumCase{
	"StackedWaypointAboveLand",
	{
		makeTakeoffItem(47.0000000, 8.0000000, 500.f),
		makePositionItem(47.0000000, 8.0010000, 500.f),
		makePositionItem(47.0000000, 8.0020000, 550.f),
		makeLandItem(47.0000000, 8.0020000, 500.f),
	},
	makePositionAbsolute(47.0000000, 8.0030000, 500.f),
	47.0, 8.002, 2, 3, SegmentLengthExpectation::kZero
},
EndpointLocalMinimumCase{
	"RallyBeyondRouteEnd",
	{
		makeTakeoffItem(47.0000000, 8.0000000, 500.f),
		makePositionItem(47.0000000, 8.0010000, 500.f),
		makePositionItem(47.0000000, 8.0020000, 500.f),
	},
	makePositionAbsolute(47.0000000, 8.0030000, 500.f),
	47.0, 8.002, 1, 2, SegmentLengthExpectation::kNonzero
}
	),
[](const ::testing::TestParamInfo<EndpointLocalMinimumCase> &param_info)
{
	return param_info.param.name;
}
);

// A projection onto the shared apex from both legs of a V is a real local minimum.
TEST_F(MissionRouteProjectionCandidateSelectionTest, KeepsPositiveVCornerLocalMinimum)
{
	const std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f, -100.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,    0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,  100.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission);
	mission_route::MissionRouteProjection projection(provider);
	auto batch = singleReferenceBatch(makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt));

	mission_route::RouteDistanceSummary distance_summary{};
	const mission_route::FailureReason status =
		projection.findProjectionCandidates(scanRequest(60.f), batch, distance_summary);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	ASSERT_EQ(batch.items[0].candidate_buffer.count, 1U);
	const mission_route::RouteProjectionCandidate &candidate = batch.items[0].candidate_buffer.candidates[0];
	EXPECT_EQ(candidate.segment.start.idx, 1);
	EXPECT_EQ(candidate.segment.end.idx, 2);
	EXPECT_NEAR(candidate.projection.lat, mission[1].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(candidate.projection.lon, mission[1].lon, kLatLonToleranceDeg);
	EXPECT_NEAR(candidate.dist.xtrack, 100.f, kDistanceTolerance);
}

// Intermediate waypoints on a straight line are not real local minima.
TEST_F(MissionRouteProjectionCandidateSelectionTest, StraightLineIgnoresNonMinCorners)
{
	std::vector<mission_item_s> mission;

	for (int i = 0; i < 10; ++i) {
		mission.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon,
				  static_cast<float>(i * 100), 0.f, kAlt));
	}

	VectorProvider provider = makeRouteProvider(mission);
	mission_route::MissionRouteProjection projection(provider);
	auto batch = singleReferenceBatch(makePositionFromOffset(kBaseLat, kBaseLon, 450.f, 50.f, kAlt));

	mission_route::RouteDistanceSummary distance_summary{};
	const mission_route::FailureReason status =
		projection.findProjectionCandidates(scanRequest(60.f), batch, distance_summary);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	ASSERT_EQ(batch.items[0].candidate_buffer.count, 1U);
	EXPECT_EQ(batch.items[0].candidate_buffer.candidates[0].segment.start.idx, 4);
	EXPECT_EQ(batch.items[0].candidate_buffer.candidates[0].segment.end.idx, 5);
}

// References in one batch share the route walk instead of loading the mission once per reference.
TEST_F(MissionRouteProjectionCandidateSelectionTest, BatchedReferencesShareMissionItemReads)
{
	static constexpr uint8_t kReferenceCount{2};

	if (mission_route::kMaxSafePointBatch < kReferenceCount) {
		GTEST_SKIP() << "batching contract requires at least two reference slots";
	}

	const std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission);
	MissionLoadCountingProvider single_reference_provider(provider);
	MissionLoadCountingProvider batched_reference_provider(provider);
	mission_route::MissionRouteProjection single_reference_projection(single_reference_provider);
	mission_route::MissionRouteProjection batched_reference_projection(batched_reference_provider);

	const mission_route::Position first_reference =
		makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 10.f, kAlt);
	auto single_reference_batch = singleReferenceBatch(first_reference);
	mission_route::ProjectionReferenceBatch batched_references{};
	batched_references.count = kReferenceCount;
	batched_references.items[0].position = first_reference;
	batched_references.items[1].position = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, -15.f, kAlt);

	mission_route::RouteDistanceSummary single_reference_summary{};
	mission_route::RouteDistanceSummary batched_reference_summary{};
	const mission_route::FailureReason single_reference_status =
		single_reference_projection.findProjectionCandidates(scanRequest(60.f), single_reference_batch,
				single_reference_summary);
	const mission_route::FailureReason batched_reference_status =
		batched_reference_projection.findProjectionCandidates(scanRequest(60.f), batched_references,
				batched_reference_summary);

	ASSERT_EQ(single_reference_status, mission_route::FailureReason::kNone)
			<< mission_route::failureReasonString(single_reference_status);
	ASSERT_GT(single_reference_batch.items[0].candidate_buffer.count, 0U);
	ASSERT_EQ(batched_reference_status, mission_route::FailureReason::kNone)
			<< mission_route::failureReasonString(batched_reference_status);

	for (uint8_t i = 0; i < batched_references.count; ++i) {
		ASSERT_GT(batched_references.items[i].candidate_buffer.count, 0U) << "reference " << static_cast<int>(i);
	}

	EXPECT_GT(single_reference_provider.missionItemLoadCount(), 0);
	EXPECT_EQ(batched_reference_provider.missionItemLoadCount(), single_reference_provider.missionItemLoadCount());
}

// The scanner retains the three route segments with the smallest cross-track distance.
TEST_F(MissionRouteProjectionCandidateSelectionTest, KeepsThreeClosestRectangleSides)
{
	// 3 -------- 2
	// |  P       |
	// 4/0 ------ 1
	//
	// Waypoint 4 closes the left side by returning to waypoint 0's position.
	const std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,    0.f,    0.f, kAlt), // 0: bottom-left
		makePositionItemFromOffset(kBaseLat, kBaseLon,    0.f, 1000.f, kAlt), // 1: bottom-right
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 1000.f, kAlt), // 2: top-right
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f,    0.f, kAlt), // 3: top-left
		makePositionItemFromOffset(kBaseLat, kBaseLon,    0.f,    0.f, kAlt), // 4: closes left side
	};
	VectorProvider provider = makeRouteProvider(mission);
	mission_route::MissionRouteProjection projection(provider);
	auto batch = singleReferenceBatch(makePositionFromOffset(kBaseLat, kBaseLon, 500.f, 100.f, kAlt));

	mission_route::RouteDistanceSummary distance_summary{};
	const mission_route::FailureReason status =
		projection.findProjectionCandidates(scanRequest(2000.f), batch, distance_summary);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	const mission_route::ProjectionCandidateBuffer &candidates = batch.items[0].candidate_buffer;
	ASSERT_EQ(candidates.count, mission_route::kMaxSegmentCandidates);
	EXPECT_NE(findCandidate(candidates, 0, 1), nullptr);
	EXPECT_EQ(findCandidate(candidates, 1, 2), nullptr);
	EXPECT_NE(findCandidate(candidates, 2, 3), nullptr);
	EXPECT_NE(findCandidate(candidates, 3, 4), nullptr);
}

// Duplicate corner waypoints must not hide the real branch-off minimum after the stack.
TEST_F(MissionRouteProjectionCandidateSelectionTest, DuplicateCornerWaypointsDoNotEvictValidCandidate)
{
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f,   0.f, kAlt),
	};

	// Stack duplicate waypoints at the same corner position.
	for (int i = 0; i < 8; ++i) {
		mission.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 200.f, kAlt));
	}

	mission.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 200.f, kAlt));

	VectorProvider provider = makeRouteProvider(mission);
	mission_route::MissionRouteProjection projection(provider);
	auto batch = singleReferenceBatch(makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 250.f, kAlt));

	mission_route::RouteDistanceSummary distance_summary{};
	const mission_route::FailureReason status =
		projection.findProjectionCandidates(scanRequest(500.f), batch, distance_summary);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	const mission_route::RouteProjectionCandidate *candidate = findCandidate(batch.items[0].candidate_buffer, 9, 10);
	ASSERT_NE(candidate, nullptr);
	EXPECT_NEAR(candidate->dist.xtrack, 50.f, kDistanceTolerance);
}

struct InvalidVehiclePositionCase {
	const char *name;
	double lat;
	double lon;
};

class MissionRouteProjectionInvalidVehiclePositionTest : public MissionRouteProjectionTestBase,
	public ::testing::WithParamInterface<InvalidVehiclePositionCase> {};

// Keep one representative for each validation rule.
TEST_P(MissionRouteProjectionInvalidVehiclePositionTest, RejectsInvalidVehiclePosition)
{
	const InvalidVehiclePositionCase &scenario = GetParam();
	const std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission);
	mission_route::MissionRouteProjection projection(provider);
	auto batch = singleReferenceBatch({});
	const mission_route::Position vehicle{scenario.lat, scenario.lon, kAlt};

	mission_route::ProjectionContext projection_context{};
	const mission_route::FailureReason status =
		projection.collectVehicleProjection(vehicle, 1, config, batch, projection_context);

	EXPECT_EQ(status, mission_route::FailureReason::kNoValidGlobalPos);
}

INSTANTIATE_TEST_SUITE_P(
	InvalidVehiclePositions,
	MissionRouteProjectionInvalidVehiclePositionTest,
	::testing::Values(
		InvalidVehiclePositionCase{"NonFinite", NAN, kBaseLon},
		InvalidVehiclePositionCase{"NullIsland", 0.0, 0.0},
		InvalidVehiclePositionCase{"LatitudeOutOfRange", 91.0, kBaseLon},
		InvalidVehiclePositionCase{"LongitudeOutOfRange", kBaseLat, 181.0}
	),
	[](const ::testing::TestParamInfo<InvalidVehiclePositionCase> &param_info)
{
	return param_info.param.name;
}
);

// Single-item mission: no start/end segment pair exists.
TEST_F(MissionRouteProjectionEdgeCaseTest, SingleWaypointMissionFails)
{
	const std::vector<mission_item_s> mission = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission);
	mission_route::MissionRouteProjection projection(provider);
	auto batch = singleReferenceBatch({});
	const mission_route::Position vehicle = makePositionAbsolute(kBaseLat, kBaseLon, kAlt);

	mission_route::ProjectionContext projection_context{};
	const mission_route::FailureReason status =
		projection.collectVehicleProjection(vehicle, 0, config, batch, projection_context);

	EXPECT_EQ(status, mission_route::FailureReason::kNoValidWaypoints);
}

// Keep this dense zigzag as a regression for finding the right local minimum among many turns.
TEST_F(MissionRouteProjectionEdgeCaseTest, ZigzagMissionStressesCandidateBuffer)
{
	std::vector<mission_item_s> mission;

	for (int i = 0; i < 8; ++i) {
		const float north = (i % 2 == 0) ? 0.f : 200.f;
		const float east = static_cast<float>(i * 150);
		mission.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, north, east, kAlt));
	}

	VectorProvider provider = makeRouteProvider(mission);
	mission_route::MissionRouteProjection projection(provider);
	// Above the central apex, [4->5] and [5->6] both project onto waypoint 5. Only the latter is
	// the V-corner minimum; rejecting the duplicate leaves room for the next genuine minimum.
	auto batch = singleReferenceBatch(makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 680.f, kAlt));

	mission_route::RouteDistanceSummary distance_summary{};
	const mission_route::FailureReason status =
		projection.findProjectionCandidates(scanRequest(500.f), batch, distance_summary);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	ASSERT_EQ(batch.items[0].candidate_buffer.count, 3U);
	EXPECT_EQ(batch.items[0].candidate_buffer.candidates[0].segment.start.idx, 5);
	EXPECT_EQ(batch.items[0].candidate_buffer.candidates[0].segment.end.idx, 6);
	EXPECT_EQ(findCandidate(batch.items[0].candidate_buffer, 4, 5), nullptr);
	EXPECT_NE(findCandidate(batch.items[0].candidate_buffer, 3, 4), nullptr);
	EXPECT_NE(findCandidate(batch.items[0].candidate_buffer, 6, 7), nullptr);
}

// The route walk stops at the first landing command even if valid positions follow it.
TEST_F(RouteSegmentCursorTest, StopsAtFirstLandBeforePostLandPositions)
{
	for (const uint16_t landing_command : {
		     static_cast<uint16_t>(NAV_CMD_LAND),
		     static_cast<uint16_t>(NAV_CMD_VTOL_LAND)
	     }) {
		SCOPED_TRACE(::testing::Message() << "landing command " << landing_command);
		std::vector<mission_item_s> mission = {
			makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
			makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt - 50.f),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
		};
		mission[2].nav_cmd = landing_command;
		VectorProvider provider = makeRouteProvider(mission);
		mission_route::RouteSegmentCursor cursor(provider, config.parameters.home_altitude_amsl);
		std::vector<mission_route::RouteSegmentView> segments;
		mission_route::RouteSegmentView segment{};

		ASSERT_TRUE(cursor.init()) << mission_route::failureReasonString(cursor.failureReason());

		while (cursor.next(segment)) {
			segments.push_back(segment);
		}

		ASSERT_FALSE(cursor.failed()) << mission_route::failureReasonString(cursor.failureReason());
		ASSERT_EQ(segments.size(), 2U);
		EXPECT_EQ(segments[0].segment.start.idx, 0);
		EXPECT_EQ(segments[0].segment.end.idx, 1);
		EXPECT_FALSE(segments[0].last_segment);
		EXPECT_EQ(segments[1].segment.start.idx, 1);
		EXPECT_EQ(segments[1].segment.end.idx, 2);
		EXPECT_EQ(segments[1].segment.end.nav_cmd, landing_command);
		EXPECT_TRUE(segments[1].last_segment);
		EXPECT_NEAR(cursor.routeLength(), 200.f, kDistanceTolerance);
	}
}

struct DoJumpEdgeCase {
	const char *name;
	uint16_t current_count;
	uint8_t expected_loops_remaining;
};

class RouteSegmentCursorDoJumpTest : public MissionRouteProjectionTestBase,
	public ::testing::WithParamInterface<DoJumpEdgeCase> {};

// Active and exhausted DO_JUMPs both emit the synthetic edge; only the remaining count differs.
TEST_P(RouteSegmentCursorDoJumpTest, EmitsSyntheticLoopEdge)
{
	const DoJumpEdgeCase &scenario = GetParam();
	const std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makeDoJump(0, 3, scenario.current_count),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission);
	mission_route::RouteSegmentCursor cursor(provider, config.parameters.home_altitude_amsl);
	std::vector<mission_route::RouteSegmentView> segments;
	mission_route::RouteSegmentView segment{};

	ASSERT_TRUE(cursor.init()) << mission_route::failureReasonString(cursor.failureReason());

	while (cursor.next(segment)) {
		segments.push_back(segment);
	}

	ASSERT_FALSE(cursor.failed()) << mission_route::failureReasonString(cursor.failureReason());
	ASSERT_EQ(segments.size(), 3U);
	EXPECT_EQ(segments[0].segment.start.idx, 0);
	EXPECT_EQ(segments[0].segment.end.idx, 1);
	EXPECT_FALSE(segments[0].segment.is_loop);
	EXPECT_EQ(segments[1].segment.start.idx, 1);
	EXPECT_EQ(segments[1].segment.end.idx, 0);
	EXPECT_TRUE(segments[1].segment.validLoop());
	EXPECT_EQ(segments[1].segment.loops_remaining, scenario.expected_loops_remaining);
	EXPECT_EQ(segments[2].segment.start.idx, 1);
	EXPECT_EQ(segments[2].segment.end.idx, 3);
	EXPECT_FALSE(segments[2].segment.is_loop);
}

INSTANTIATE_TEST_SUITE_P(
	DoJumpState,
	RouteSegmentCursorDoJumpTest,
	::testing::Values(
		DoJumpEdgeCase{"Active", 1, 2},
		DoJumpEdgeCase{"Exhausted", 3, 0}
	),
	[](const ::testing::TestParamInfo<DoJumpEdgeCase> &param_info)
{
	return param_info.param.name;
}
);

// A mission read failure is fatal to the scan and remains distinguishable from missing geometry.
TEST_F(MissionRouteProjectionEdgeCaseTest, MissionLoadFailureIsFatal)
{
	const std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission);
	provider.setMissionLoadFailures({0});
	mission_route::MissionRouteProjection projection(provider);
	auto batch = singleReferenceBatch(makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 10.f, kAlt));

	mission_route::RouteDistanceSummary distance_summary{};
	distance_summary.route_length = 42.f;
	const mission_route::FailureReason status =
		projection.findProjectionCandidates(scanRequest(60.f), batch, distance_summary);

	EXPECT_EQ(status, mission_route::FailureReason::kLoadFailed);
	EXPECT_FLOAT_EQ(distance_summary.route_length, 0.f);
}

// Accumulation of route distance skips malformed position items.
TEST_F(MissionRouteProjectionEdgeCaseTest, AccumulateRouteDistanceSkipsMalformedPosition)
{
	mission_item_s malformed = makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);
	malformed.lat = NAN;
	const std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt),
		malformed,
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	VectorProvider provider = makeRouteProvider(mission);
	mission_route::MissionRouteProjection projection(provider);

	const float distance = projection.accumulateRouteDistance(0, 2, config.parameters.home_altitude_amsl);

	ASSERT_TRUE(PX4_ISFINITE(distance));
	EXPECT_NEAR(distance, 200.f, kDistanceTolerance);
}

// Segment ownership differs at nominal and reverse boundaries.
TEST_F(MissionRouteProjectionLocalSegmentTest, IsIndexInProjectionSegmentHandlesDirectionBoundaries)
{
	mission_route::Segment segment{};
	segment.start.idx = 2;
	segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	segment.end.idx = 4;
	segment.end.nav_cmd = NAV_CMD_WAYPOINT;

	auto expect_match = [&](int32_t mission_index, bool reverse, bool expected) {
		EXPECT_EQ(mission_route::isIndexInProjectionSegment(segment, mission_index, reverse), expected)
				<< "mission_index=" << mission_index << " reverse=" << reverse;
	};

	expect_match(1, false, false);
	expect_match(2, false, false);
	expect_match(3, false, true);
	expect_match(4, false, true);
	expect_match(5, false, false);
	expect_match(-1, false, false);
	expect_match(1, true, false);
	expect_match(2, true, true);
	expect_match(3, true, true);
	expect_match(4, true, false);
	expect_match(5, true, false);
	expect_match(-1, true, false);
}
