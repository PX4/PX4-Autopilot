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
 * Mission-route scanner, projection, and candidate selection tests.
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
	config.parameters.vehicle_projection_search_dist_m = 60.f;
	config.parameters.safe_point_projection_search_dist_m = 60.f;
	config.parameters.nav_acceptance_radius_m = 10.f;
	config.parameters.straight_to_safe_point_rad_m = 10.f;
	config.parameters.altitude_acceptance_radius_m = 10.f;
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

// Owns a vector-backed provider and a projection bound to it: the common one-line test setup.
class TestProjection
{
public:
	explicit TestProjection(const std::vector<mission_item_s> &mission_items) :
		_provider(makeRouteProvider(mission_items)),
		_projection(_provider)
	{}

	mission_route::FailureReason collectVehicleProjection(const mission_route::Position &vehicle_position,
			int32_t mission_index, const mission_route::PlannerConfig &planner_config,
			mission_route::ProjectionReferenceBatch &batch,
			mission_route::ProjectionContext &projection_context) const
	{
		return _projection.collectVehicleProjection(vehicle_position, mission_index, planner_config, batch,
				projection_context);
	}

	mission_route::FailureReason findProjectionCandidates(const mission_route::ProjectionScanRequest &request,
			mission_route::ProjectionReferenceBatch &batch,
			mission_route::RouteDistanceSummary &distance_summary) const
	{
		return _projection.findProjectionCandidates(request, batch, distance_summary);
	}

	VectorProvider &provider() { return _provider; }

private:
	VectorProvider _provider;
	mission_route::MissionRouteProjection _projection;
};

struct ProjectionCoordinateCase {
	const char *name;
	double lat;
	double lon;
	float north_m;
	float east_m;
};

class MissionRouteProjectionCoordinateTest : public MissionRouteProjectionTestBase,
	public ::testing::WithParamInterface<ProjectionCoordinateCase> {};

// An exact endpoint must reconstruct to itself, including on long legs and across the date line.
TEST_P(MissionRouteProjectionCoordinateTest, PreservesExactEndpoints)
{
	const auto &scenario = GetParam();
	mission_item_s end = makePositionItemFromOffset(scenario.lat, scenario.lon, scenario.north_m, scenario.east_m, kAlt + 100.f);
	end.lon = matrix::wrap(end.lon, -180.0, 180.0);
	const std::vector<mission_item_s> mission = {
		makePositionItem(scenario.lat, scenario.lon, kAlt),
		end,
	};
	TestProjection projection(mission);

	for (int endpoint_index = 0; endpoint_index < 2; ++endpoint_index) {
		SCOPED_TRACE(endpoint_index);
		const mission_item_s &endpoint = mission[endpoint_index];
		auto batch = singleReferenceBatch({endpoint.lat, endpoint.lon, endpoint.altitude});
		mission_route::RouteDistanceSummary distance_summary{};
		ASSERT_EQ(projection.findProjectionCandidates(scanRequest(60.f), batch, distance_summary),
			  mission_route::FailureReason::kNone);
		ASSERT_EQ(batch.items[0].candidate_buffer.count, 1);
		const auto &candidate = batch.items[0].candidate_buffer.candidates[0];
		EXPECT_NEAR(candidate.dist.xtrack_m, 0.f, 0.01f);
		EXPECT_LT(get_distance_to_next_waypoint(endpoint.lat, endpoint.lon,
							candidate.projection.lat, candidate.projection.lon), 0.01f);
		EXPECT_FLOAT_EQ(candidate.projection.alt, endpoint.altitude);
		EXPECT_NEAR(candidate.dist.along_segment_m, endpoint_index * candidate.dist.segment_length_m, 0.01f);
	}
}

// An independently computed interior point also catches a bad inverse hidden by endpoint handling.
TEST_P(MissionRouteProjectionCoordinateTest, PreservesGreatCircleMidpoint)
{
	const auto &scenario = GetParam();
	mission_item_s end = makePositionItemFromOffset(scenario.lat, scenario.lon, scenario.north_m, scenario.east_m, kAlt + 100.f);
	end.lon = matrix::wrap(end.lon, -180.0, 180.0);
	TestProjection projection({makePositionItem(scenario.lat, scenario.lon, kAlt), end});
	const float length_m = get_distance_to_next_waypoint(scenario.lat, scenario.lon, end.lat, end.lon);
	mission_route::Position midpoint{};
	create_waypoint_from_line_and_dist(scenario.lat, scenario.lon, end.lat, end.lon,
					   length_m * 0.5f, &midpoint.lat, &midpoint.lon);
	midpoint.lon = matrix::wrap(midpoint.lon, -180.0, 180.0);
	midpoint.alt = kAlt + 50.f;
	auto batch = singleReferenceBatch(midpoint);
	mission_route::RouteDistanceSummary distance_summary{};
	ASSERT_EQ(projection.findProjectionCandidates(scanRequest(60.f), batch, distance_summary),
		  mission_route::FailureReason::kNone);
	ASSERT_EQ(batch.items[0].candidate_buffer.count, 1);
	const auto &candidate = batch.items[0].candidate_buffer.candidates[0];
	EXPECT_NEAR(candidate.dist.xtrack_m, 0.f, 0.01f);
	EXPECT_LT(get_distance_to_next_waypoint(midpoint.lat, midpoint.lon,
						candidate.projection.lat, candidate.projection.lon), 0.01f);
	EXPECT_NEAR(candidate.dist.along_segment_m, length_m * 0.5f, 0.01f);
	EXPECT_NEAR(candidate.projection.alt, midpoint.alt, 0.01f);
}

INSTANTIATE_TEST_SUITE_P(
	LongLegs,
	MissionRouteProjectionCoordinateTest,
	::testing::Values(
		ProjectionCoordinateCase{"HighLatitude", 75.0, 8.5, 0.f, 20000.f},
		ProjectionCoordinateCase{"AcrossAntimeridian", 47.4, 179.9, 0.f, 20000.f}
	),
	[](const ::testing::TestParamInfo<ProjectionCoordinateCase> &param_info)
{
	return param_info.param.name;
}
);

// ---- Vehicle projection segment selection ----

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
	TestProjection projection(mission);
	auto batch = singleReferenceBatch({});
	const mission_route::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 500.f, kAlt);
	config.parameters.vehicle_projection_search_dist_m = 150.f;

	mission_route::ProjectionContext projection_context{};
	const mission_route::FailureReason status =
		projection.collectVehicleProjection(vehicle, 3, config, batch, projection_context);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_NE(findCandidate(batch.items[0].candidate_buffer, 0, 1), nullptr);
	EXPECT_NE(findCandidate(batch.items[0].candidate_buffer, 2, 3), nullptr);
	EXPECT_EQ(projection_context.route_projection.segment.start.idx, 2);
	EXPECT_EQ(projection_context.route_projection.segment.end.idx, 3);
	EXPECT_NEAR(projection_context.route_projection.dist.xtrack_m, 90.f, kDistanceTolerance);
}

// The anchor records that the vehicle is currently executing the DO_JUMP at idx 3, i.e. flying
// the [2->0] return leg toward target idx 0. Projection must stay on that leg (~46 m away) even
// though nominal [0-1] is only 10 m away: switching legs would silently drop the pending repeats.
TEST_F(MissionRouteProjectionLocalSegmentTest, PrefersStoredLoopAnchor)
{
	const std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makeDoJump(0, 2, 0),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f,   0.f, kAlt),
	};
	TestProjection projection(mission);
	auto batch = singleReferenceBatch({});

	config.active_jump_anchor.jump_item_index = 3; // mid-jump: the vehicle flies [2->0], not [0-1]

	const mission_route::Position vehicle = makePositionFromOffset(kBaseLat, kBaseLon, 75.f, 10.f, kAlt);
	mission_route::ProjectionContext projection_context{};
	const mission_route::FailureReason status =
		projection.collectVehicleProjection(vehicle, 0, config, batch, projection_context);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	EXPECT_TRUE(projection_context.route_projection.segment.validLoop());
	EXPECT_EQ(projection_context.route_projection.segment.jump_item_index, 3);
	EXPECT_TRUE(projection_context.route_projection.segment.has_remaining_repeats);
	EXPECT_EQ(projection_context.route_projection.segment.start.idx, 2);
	EXPECT_EQ(projection_context.route_projection.segment.end.idx, 0);
	ASSERT_TRUE(projection_context.loop_context.valid());
	EXPECT_NEAR(projection_context.loop_context.bounds.start_dist_along_route_m, 200.f, kDistanceTolerance);
	EXPECT_NEAR(projection_context.loop_context.bounds.end_dist_along_route_m, 0.f, kDistanceTolerance);
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

	// nominal direction ]2, 4]
	expect_match(1, false, false);
	expect_match(2, false, false);
	expect_match(3, false, true);
	expect_match(4, false, true);
	expect_match(5, false, false);
	expect_match(-1, false, false);

	// reverse direction [2, 4[
	expect_match(1, true, false);
	expect_match(2, true, true);
	expect_match(3, true, true);
	expect_match(4, true, false);
	expect_match(5, true, false);
	expect_match(-1, true, false);
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

static constexpr std::array<ProjectionDatasetCase, 2> kProjectionDatasetCases{{
		{"DefaultCoincidingSegments", false, 46.11174050459439, 2.2876843642852362, 475.9f, 8, 7, 8},
		{"CornerOnSmallSegment", true, 46.10361319095525, 2.3183349874167636, 462.6f, 13, 12, 13},
	}};

class MissionRouteProjectionDatasetTest : public MissionRouteProjectionTestBase,
	public ::testing::WithParamInterface<ProjectionDatasetCase> {};

TEST_P(MissionRouteProjectionDatasetTest, SelectsExpectedSegment)
{
	const ProjectionDatasetCase &scenario = GetParam();
	const std::vector<mission_item_s> mission =
		scenario.use_corner_dataset ? corner_dataset::mission() : default_dataset::mission();
	TestProjection projection(mission);
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

// ---- Candidate scanning and local minima ----

struct EndpointLocalMinimumCase {
	const char *name;
	std::vector<mission_item_s> mission;
	mission_route::Position reference;
	double expected_projection_lat;
	double expected_projection_lon;
	float expected_projection_alt;
	int32_t expected_segment_start;
	int32_t expected_segment_end;
};

class MissionRouteProjectionEndpointLocalMinimumTest : public MissionRouteProjectionTestBase,
	public ::testing::WithParamInterface<EndpointLocalMinimumCase> {};

// References beyond either end of the route keep the endpoint projection.
TEST_P(MissionRouteProjectionEndpointLocalMinimumTest, KeepsEndpointCandidate)
{
	const EndpointLocalMinimumCase &scenario = GetParam();
	TestProjection projection(scenario.mission);
	auto batch = singleReferenceBatch(scenario.reference);

	mission_route::RouteDistanceSummary distance_summary{};
	const mission_route::FailureReason status =
		projection.findProjectionCandidates(scanRequest(config.parameters.safe_point_projection_search_dist_m), batch,
				distance_summary);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	const mission_route::RouteProjectionCandidate *candidate =
		findCandidate(batch.items[0].candidate_buffer, scenario.expected_segment_start, scenario.expected_segment_end);
	ASSERT_NE(candidate, nullptr);
	EXPECT_NEAR(candidate->projection.lat, scenario.expected_projection_lat, kLatLonToleranceDeg);
	EXPECT_NEAR(candidate->projection.lon, scenario.expected_projection_lon, kLatLonToleranceDeg);
	EXPECT_NEAR(candidate->projection.alt, scenario.expected_projection_alt, 0.01f);
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
	47.0, 8.0, 500.f, 0, 1
},
EndpointLocalMinimumCase{
	"RallyBeyondRouteEnd",
	{
		makeTakeoffItem(47.0000000, 8.0000000, 500.f),
		makePositionItem(47.0000000, 8.0010000, 500.f),
		makePositionItem(47.0000000, 8.0020000, 500.f),
	},
	makePositionAbsolute(47.0000000, 8.0030000, 500.f),
	47.0, 8.002, 500.f, 1, 2
}
	),
[](const ::testing::TestParamInfo<EndpointLocalMinimumCase> &param_info)
{
	return param_info.param.name;
}
);

// Both V legs project the reference onto the shared apex. The corner is a genuine local
// minimum and must be kept, but only once.
TEST_F(MissionRouteProjectionCandidateSelectionTest, KeepsPositiveVCornerLocalMinimum)
{
	//      P
	//
	//      1
	//     / \  legs [0-1] and [1-2]
	//    0   2
	const std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f, -100.f, kAlt), // 0: left foot
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,    0.f, kAlt), // 1: apex
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,  100.f, kAlt), // 2: right foot
	};
	TestProjection projection(mission);
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
	EXPECT_NEAR(candidate.dist.xtrack_m, 100.f, kDistanceTolerance);
}

// Intermediate waypoints on a straight line are not real local minima.
TEST_F(MissionRouteProjectionCandidateSelectionTest, StraightLineIgnoresNonMinCorners)
{
	std::vector<mission_item_s> mission;

	for (int i = 0; i < 10; ++i) {
		mission.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon,
				  static_cast<float>(i * 100), 0.f, kAlt));
	}

	TestProjection projection(mission);
	auto batch = singleReferenceBatch(makePositionFromOffset(kBaseLat, kBaseLon, 450.f, 50.f, kAlt));

	mission_route::RouteDistanceSummary distance_summary{};
	const mission_route::FailureReason status =
		projection.findProjectionCandidates(scanRequest(60.f), batch, distance_summary);

	ASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);
	ASSERT_EQ(batch.items[0].candidate_buffer.count, 1U);
	EXPECT_EQ(batch.items[0].candidate_buffer.candidates[0].segment.start.idx, 4);
	EXPECT_EQ(batch.items[0].candidate_buffer.candidates[0].segment.end.idx, 5);
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
	TestProjection projection(mission);
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

// ---- Route segment cursor ----

// Relative mission altitudes need a finite home altitude to become AMSL.
TEST_F(MissionRouteProjectionLocalSegmentTest, RelativeAltitudeRequiresFiniteHomeAltitude)
{
	mission_item_s relative_item = makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, 50.f);
	relative_item.altitude_is_relative = true;
	const std::vector<mission_item_s> relative_mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		relative_item,
	};
	VectorProvider relative_provider = makeRouteProvider(relative_mission);
	mission_route::RouteSegmentCursor relative_cursor(relative_provider, NAN);
	mission_route::RouteSegmentView segment{};

	ASSERT_TRUE(relative_cursor.init()) << mission_route::failureReasonString(relative_cursor.failureReason());
	EXPECT_FALSE(relative_cursor.next(segment));
	EXPECT_TRUE(relative_cursor.failed());
	EXPECT_EQ(relative_cursor.failureReason(), mission_route::FailureReason::kPositionItemInvalid);

	// Absolute altitude missions do not depend on home altitude.
	const std::vector<mission_item_s> absolute_mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
	};
	VectorProvider absolute_provider = makeRouteProvider(absolute_mission);
	mission_route::RouteSegmentCursor absolute_cursor(absolute_provider, NAN);

	ASSERT_TRUE(absolute_cursor.init()) << mission_route::failureReasonString(absolute_cursor.failureReason());
	EXPECT_TRUE(absolute_cursor.next(segment));
	EXPECT_FALSE(absolute_cursor.failed());
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
	bool expected_remaining_repeats;
};

class RouteSegmentCursorDoJumpTest : public MissionRouteProjectionTestBase,
	public ::testing::WithParamInterface<DoJumpEdgeCase> {};

// A DO_JUMP has no geometry of its own: the cursor emits a synthetic edge from the position
// before the jump back to the jump target, so the return leg is projectable like any segment.
// Active and exhausted jumps both emit it; only the repeat obligation differs.
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
	MissionLoadCountingProvider counting_provider(provider);
	mission_route::RouteSegmentCursor cursor(counting_provider, config.parameters.home_altitude_amsl);
	std::vector<mission_route::RouteSegmentView> segments;
	mission_route::RouteSegmentView segment{};

	ASSERT_TRUE(cursor.init()) << mission_route::failureReasonString(cursor.failureReason());

	while (cursor.next(segment)) {
		segments.push_back(segment);
	}

	ASSERT_FALSE(cursor.failed()) << mission_route::failureReasonString(cursor.failureReason());

	struct ExpectedSegment {
		int32_t start_idx;
		int32_t end_idx;
		bool is_loop;
		int32_t jump_item_index;
		bool has_remaining_repeats;
	};

	// Nominal [0-1], the synthetic [1->0] edge owned by the jump at idx 2, then the walk
	// resumes from the pre-jump position toward the next item: [1-3].
	const std::array<ExpectedSegment, 3> expected{{
			{0, 1, false, -1, false},
			{1, 0, true, 2, scenario.expected_remaining_repeats},
			{1, 3, false, -1, false},
		}};

	ASSERT_EQ(segments.size(), expected.size());

	for (size_t i = 0; i < expected.size(); ++i) {
		SCOPED_TRACE(::testing::Message() << "segment " << i);
		EXPECT_EQ(segments[i].segment.start.idx, expected[i].start_idx);
		EXPECT_EQ(segments[i].segment.end.idx, expected[i].end_idx);
		EXPECT_EQ(segments[i].segment.isLoop(), expected[i].is_loop);
		EXPECT_EQ(segments[i].segment.jump_item_index, expected[i].jump_item_index);
		EXPECT_EQ(segments[i].segment.has_remaining_repeats, expected[i].has_remaining_repeats);
	}

	EXPECT_TRUE(segments[1].segment.validLoop());
	EXPECT_FALSE(segments[0].last_segment);
	EXPECT_FALSE(segments[1].last_segment);
	EXPECT_TRUE(segments[2].last_segment);
	// The loop edge does not extend the route: length stays the nominal [0-1] plus [1-3].
	EXPECT_NEAR(cursor.routeLength(), 200.f, kDistanceTolerance);
	EXPECT_EQ(counting_provider.missionItemLoadCount(), 7); // Includes nominal endpoint lookahead past the jump.
}

INSTANTIATE_TEST_SUITE_P(
	DoJumpState,
	RouteSegmentCursorDoJumpTest,
	::testing::Values(
		DoJumpEdgeCase{"Active", 1, true},
		DoJumpEdgeCase{"Exhausted", 3, false}
	),
	[](const ::testing::TestParamInfo<DoJumpEdgeCase> &param_info)
{
	return param_info.param.name;
}
);

// ---- VTOL segment state ----

// VTOL_TAKEOFF implicitly enters FW; a later explicit transition still takes precedence.
TEST_F(MissionRouteProjectionTestBase, VtolTakeoffSetsFwUntilNextExplicitTransition)
{
	constexpr uint8_t kMc = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
	constexpr uint8_t kFw = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;
	const VectorProvider provider = makeRouteProvider({
		makeVtolTransitionItem(kMc),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt, NAV_CMD_VTOL_TAKEOFF),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makeVtolTransitionItem(kMc),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	});
	mission_route::Segment segment{};
	segment.start = {1, NAV_CMD_VTOL_TAKEOFF};
	segment.end = {2, NAV_CMD_WAYPOINT};

	EXPECT_EQ(mission_route::vtolStateForSegment(provider, segment, kMc), kFw);

	segment.start = segment.end;
	segment.end = {4, NAV_CMD_WAYPOINT};
	EXPECT_EQ(mission_route::vtolStateForSegment(provider, segment, kMc), kMc);
}

// Include transitions after the jump source, but exclude commands that follow DO_JUMP itself.
TEST_F(MissionRouteProjectionTestBase, JumpStateIncludesTransitionsAfterSourceWaypoint)
{
	constexpr uint8_t kMc = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
	constexpr uint8_t kFw = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;

	for (const uint8_t target_state : {kMc, kFw}) {
		SCOPED_TRACE(static_cast<int>(target_state));
		const uint8_t previous_state = target_state == kMc ? kFw : kMc;
		const VectorProvider provider = makeRouteProvider({
			makeVtolTransitionItem(previous_state),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
			makeVtolTransitionItem(previous_state),
			makeVtolTransitionItem(target_state),
			makeDoJump(1, 2),
			makeVtolTransitionItem(previous_state), // Runs only after the jump has finished.
			makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		});
		mission_route::Segment segment{};
		segment.start = {2, NAV_CMD_WAYPOINT};
		segment.end = {1, NAV_CMD_WAYPOINT};
		segment.jump_item_index = 5;

		EXPECT_EQ(mission_route::vtolStateForSegment(provider, segment, previous_state), target_state);
	}
}

// Commands at the jump target run after DO_JUMP and override the state at its source.
TEST_F(MissionRouteProjectionTestBase, JumpTargetTransitionOverridesSourceState)
{
	constexpr uint8_t kMc = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
	constexpr uint8_t kFw = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;

	for (const uint8_t target_state : {kMc, kFw}) {
		SCOPED_TRACE(static_cast<int>(target_state));
		const uint8_t source_state = target_state == kMc ? kFw : kMc;
		const VectorProvider provider = makeRouteProvider({
			makeVtolTransitionItem(target_state), // The jump returns to this command.
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
			makeVtolTransitionItem(source_state),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
			makeDoJump(0, 2),
		});
		mission_route::Segment segment{};
		segment.start = {3, NAV_CMD_WAYPOINT};
		segment.end = {1, NAV_CMD_WAYPOINT};
		segment.jump_item_index = 4;

		EXPECT_EQ(mission_route::vtolStateForSegment(provider, segment, source_state), target_state);
	}
}

// Takeoff and items 1/2 share latitude/longitude. Item 2 starts the first horizontal leg,
// so that leg supplies both the start-corner projection and its altitude.
TEST_F(MissionRouteProjectionCandidateSelectionTest, LeadingStacksUseFirstHorizontalLeg)
{
	TestProjection projection({
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 75.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 100.f),
	});

	// References are before takeoff, abreast of it, and halfway along the leg, all 10 m off track.
	for (const float north_m : {-50.f, 0.f, 50.f}) {
		SCOPED_TRACE(north_m);
		auto batch = singleReferenceBatch(makePositionFromOffset(kBaseLat, kBaseLon, north_m, 10.f, kAlt));
		mission_route::RouteDistanceSummary summary{};
		ASSERT_EQ(projection.findProjectionCandidates(scanRequest(100.f), batch, summary), mission_route::FailureReason::kNone);
		ASSERT_EQ(batch.items[0].candidate_buffer.count, 1);
		const auto &candidate = batch.items[0].candidate_buffer.candidates[0];
		EXPECT_EQ(candidate.segment.start.idx, 2);
		EXPECT_EQ(candidate.segment.end.idx, 3);
		// The start uses +75 m; the midpoint interpolates between +75 m and +100 m.
		EXPECT_NEAR(candidate.projection.alt, north_m <= 0.f ? kAlt + 75.f : kAlt + 87.5f, 0.01f);
	}
}

// B, B' and LAND share latitude/longitude. Keep B' -> LAND only for references whose
// closest point on the incoming A -> B leg is B itself.
TEST_F(MissionRouteProjectionCandidateSelectionTest, TerminalStackRequiresIncomingEndpointMinimum)
{
	TestProjection projection({
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 75.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt - 100.f),
	});
	// All are 10 m off track: beside the leg's interior, abreast of LAND, and beyond LAND.
	// Scan together to verify that the endpoint decision stays independent for each reference.
	const std::array<float, 3> north_offsets{50.f, 100.f, 150.f};
	mission_route::ProjectionReferenceBatch batch{};
	batch.count = north_offsets.size();
	static_assert(mission_route::kMaxSafePointBatch >= north_offsets.size(), "test needs three references");

	for (uint8_t i = 0; i < batch.count; ++i) {
		batch.items[i].position = makePositionFromOffset(kBaseLat, kBaseLon, north_offsets[i], 10.f, kAlt);
	}

	mission_route::RouteDistanceSummary summary{};
	ASSERT_EQ(projection.findProjectionCandidates(scanRequest(100.f), batch, summary), mission_route::FailureReason::kNone);

	for (uint8_t i = 0; i < batch.count; ++i) {
		SCOPED_TRACE(north_offsets[i]);
		ASSERT_EQ(batch.items[i].candidate_buffer.count, 1);
		const auto &candidate = batch.items[i].candidate_buffer.candidates[0];
		// The first reference uses A -> B; the other two use B' -> LAND at B' altitude (+75 m).
		EXPECT_EQ(candidate.segment.start.idx, i == 0 ? 0 : 2);
		EXPECT_EQ(candidate.segment.end.idx, i == 0 ? 1 : 3);
		EXPECT_NEAR(candidate.projection.alt, i == 0 ? kAlt + 25.f : kAlt + 75.f, 0.01f);
	}
}

struct FlyingVerticalSegmentCase {
	const char *name;
	std::vector<mission_item_s> mission;
	int32_t mission_index;
	int32_t vertical_start;
	int32_t vertical_end;
	float vehicle_north_m;
	float expected_alt;
};

class MissionRouteProjectionFlyingVerticalSegmentTest : public MissionRouteProjectionTestBase,
	public ::testing::WithParamInterface<FlyingVerticalSegmentCase> {};

// The active vertical segment or initial TAKEOFF stays a vehicle candidate after a small drift
// toward the adjacent horizontal leg.
// Safe point scans carry no mission index and keep the geometric rules.
TEST_P(MissionRouteProjectionFlyingVerticalSegmentTest, RemainsVehicleCandidate)
{
	const FlyingVerticalSegmentCase &scenario = GetParam();
	TestProjection projection(scenario.mission);
	const auto vehicle = makePositionFromOffset(kBaseLat, kBaseLon, scenario.vehicle_north_m, 0.5f, kAlt);
	mission_route::RouteDistanceSummary summary{};

	// Without a mission index the drifted position only projects onto the horizontal leg.
	auto batch = singleReferenceBatch(vehicle);
	ASSERT_EQ(projection.findProjectionCandidates(scanRequest(60.f), batch, summary), mission_route::FailureReason::kNone);
	EXPECT_EQ(findCandidate(batch.items[0].candidate_buffer, scenario.vertical_start, scenario.vertical_end), nullptr);

	// With the index, the vertical segment is a candidate and mission continuity selects it.
	batch = singleReferenceBatch(vehicle);
	auto request = scanRequest(60.f);
	request.mission_index = scenario.mission_index;
	ASSERT_EQ(projection.findProjectionCandidates(request, batch, summary), mission_route::FailureReason::kNone);
	const auto *candidate = findCandidate(batch.items[0].candidate_buffer, scenario.vertical_start,
					      scenario.vertical_end);
	ASSERT_NE(candidate, nullptr);
	EXPECT_NEAR(candidate->projection.alt, scenario.expected_alt, 0.01f);

	mission_route::ProjectionContext context{};
	ASSERT_EQ(projection.collectVehicleProjection(vehicle, scenario.mission_index, config, batch, context),
		  mission_route::FailureReason::kNone);
	EXPECT_EQ(context.route_projection.segment.start.idx, scenario.vertical_start);
	EXPECT_EQ(context.route_projection.segment.end.idx, scenario.vertical_end);
}

INSTANTIATE_TEST_SUITE_P(
	FlyingVerticalSegments,
	MissionRouteProjectionFlyingVerticalSegmentTest,
	::testing::Values(
FlyingVerticalSegmentCase{
	"LandingDescentDriftedTowardApproachLeg",
	{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 75.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt - 100.f),
	},
	3, 2, 3, 98.f, kAlt + 75.f
},
FlyingVerticalSegmentCase{
	"InitialTakeoffDriftedTowardDepartureLeg",
	{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 75.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 75.f),
	},
	0, 0, 1, 2.f, kAlt + 50.f
}
	),
[](const ::testing::TestParamInfo<FlyingVerticalSegmentCase> &param_info)
{
	return param_info.param.name;
}
);

// Neither duplicate corner waypoints nor a backward jump may erase the incoming leg's corner state.
TEST_F(MissionRouteProjectionCandidateSelectionTest, StackedVCornerSurvivesInterveningJump)
{
	for (const bool include_jump : {false, true}) {
		SCOPED_TRACE(include_jump);
		std::vector<mission_item_s> mission{
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, -100.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 25.f),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 50.f),
		};

		if (include_jump) {
			mission.push_back(makeDoJump(0, 2));
		}

		mission.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 100.f, kAlt));
		TestProjection projection(mission);
		auto batch = singleReferenceBatch(makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt));
		mission_route::RouteDistanceSummary summary{};
		ASSERT_EQ(projection.findProjectionCandidates(scanRequest(100.f), batch, summary), mission_route::FailureReason::kNone);
		ASSERT_EQ(batch.items[0].candidate_buffer.count, 1);
		const auto &candidate = batch.items[0].candidate_buffer.candidates[0];
		EXPECT_EQ(candidate.segment.start.idx, 3);
		EXPECT_EQ(candidate.segment.end.idx, include_jump ? 5 : 4);
		EXPECT_NEAR(candidate.dist.xtrack_m, 100.f, kDistanceTolerance);
		EXPECT_NEAR(candidate.projection.alt, kAlt + 50.f, 0.01f);
	}
}
