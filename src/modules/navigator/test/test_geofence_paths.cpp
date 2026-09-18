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

#include <gtest/gtest.h>

#include "support/geofence_test_helpers.h"

#include <array>
#include <cmath>

class GeofenceTest : public navigator_test::GeofenceTestBase
{
protected:
	void SetUp() override { ASSERT_TRUE(resetFence()); }
};

enum class FenceShape { ExclusionPolygon, ConcaveInclusion, ExclusionCircle, InclusionCircle };

struct GeofencePathCase {
	const char *name;
	FenceShape shape;
	matrix::Vector2f start;
	matrix::Vector2f end;
	bool clear;
};

class GeofencePathTest : public GeofenceTest, public ::testing::WithParamInterface<GeofencePathCase> {};

TEST_P(GeofencePathTest, ChecksHorizontalPath)
{
	const GeofencePathCase &test = GetParam();
	FencePoints points;

	switch (test.shape) {
	case FenceShape::ExclusionPolygon:
		points = exclusionSquare();
		break;

	case FenceShape::ConcaveInclusion:
		// The northern arms have a 400 m gap. The path at 500 m north crosses it;
		// the path at 0 m north stays in the connecting base.
		points = polygon(true, {{-200.f, -600.f}, {-200.f, 600.f}, {600.f, 600.f}, {600.f, 200.f},
			{200.f, 200.f}, {200.f, -200.f}, {600.f, -200.f}, {600.f, -600.f}
		});
		break;

	case FenceShape::ExclusionCircle:
		points = circle(false, {0.f, 300.f}, 50.f);
		break;

	case FenceShape::InclusionCircle:
		points = circle(true, {0.f, 0.f}, 500.f);
		break;
	}

	ASSERT_TRUE(loadFence(points));
	const Geofence::PathCheck paths[] {path(test.start, test.end), path(test.end, test.start)};
	bool clear[2] {};
	ASSERT_TRUE(_fence.checkPathBatch(paths, 2, clear));
	EXPECT_EQ(clear[0], test.clear);
	EXPECT_EQ(clear[1], test.clear);
}

INSTANTIATE_TEST_SUITE_P(Paths, GeofencePathTest, ::testing::Values(
				 GeofencePathCase{"AcrossExclusionPolygon", FenceShape::ExclusionPolygon, {0.f, 100.f}, {0.f, 500.f}, false},
				 GeofencePathCase{"ClearOfExclusionPolygon", FenceShape::ExclusionPolygon, {60.f, 100.f}, {60.f, 500.f}, true},
				 GeofencePathCase{"AlongPolygonEdge", FenceShape::ExclusionPolygon, {50.f, 100.f}, {50.f, 500.f}, false},
				 GeofencePathCase{"TouchingPolygonVertex", FenceShape::ExclusionPolygon, {-100.f, 200.f}, {-50.f, 250.f}, false},
				 GeofencePathCase{"ZeroLengthOutsidePolygon", FenceShape::ExclusionPolygon, {0.f, 100.f}, {0.f, 100.f}, true},
				 GeofencePathCase{"ZeroLengthOnPolygonVertex", FenceShape::ExclusionPolygon, {-50.f, 250.f}, {-50.f, 250.f}, false},
				 GeofencePathCase{"AcrossInclusionNotch", FenceShape::ConcaveInclusion, {500.f, -400.f}, {500.f, 400.f}, false},
				 GeofencePathCase{"AcrossInclusionBase", FenceShape::ConcaveInclusion, {0.f, -400.f}, {0.f, 400.f}, true},
				 GeofencePathCase{"AcrossExclusionCircle", FenceShape::ExclusionCircle, {0.f, 100.f}, {0.f, 500.f}, false},
				 GeofencePathCase{"ClearOfExclusionCircle", FenceShape::ExclusionCircle, {60.f, 100.f}, {60.f, 500.f}, true},
				 GeofencePathCase{"ZeroLengthOutsideCircle", FenceShape::ExclusionCircle, {0.f, 100.f}, {0.f, 100.f}, true},
				 GeofencePathCase{"ZeroLengthInsideCircle", FenceShape::ExclusionCircle, {0.f, 300.f}, {0.f, 300.f}, false},
				 GeofencePathCase{"InsideInclusionCircle", FenceShape::InclusionCircle, {0.f, -400.f}, {0.f, 400.f}, true},
				 GeofencePathCase{"LeavingInclusionCircle", FenceShape::InclusionCircle, {0.f, 0.f}, {0.f, 600.f}, false},
				 GeofencePathCase{"OutsideInclusionCircle", FenceShape::InclusionCircle, {600.f, -100.f}, {600.f, 100.f}, false}),
			 [](const ::testing::TestParamInfo<GeofencePathCase> &test_info)
{
	return test_info.param.name;
});

struct LoiterFenceCase {
	const char *name;
	bool circle_fence;
	bool inclusion;
};

class GeofenceLoiterTest : public navigator_test::GeofenceTestBase,
	public ::testing::WithParamInterface<LoiterFenceCase>
{
public:
	GeofenceLoiterTest() : GeofenceTestBase(0.0, 0.0) {}

protected:
	void SetUp() override { ASSERT_TRUE(resetFence()); }
};

TEST_P(GeofenceLoiterTest, ChecksCircleClearanceInMixedBatch)
{
	const LoiterFenceCase &test = GetParam();
	FencePoints points;

	// Each fence has its nearest boundary 75 m from the circle centre at Home.
	if (test.circle_fence) {
		points = circle(test.inclusion, {0.f, test.inclusion ? -25.f : 100.f}, test.inclusion ? 100.f : 25.f);

	} else if (test.inclusion) {
		points = polygon(true, {{-75.f, -75.f}, {75.f, -75.f}, {75.f, 75.f}, {-75.f, 75.f}});

	} else {
		points = polygon(false, {{-25.f, 75.f}, {25.f, 75.f}, {25.f, 125.f}, {-25.f, 125.f}});
	}

	ASSERT_TRUE(loadFence(points));
	const auto center = position(0.f, 0.f);
	// Check a point, a clear circle, boundary contact, and a circle that encloses the entire exclusion.
	const Geofence::PathCheck paths[] {{center, center}, {center, center, 50.f}, {center, center, 75.f}, {center, center, 150.f}};
	bool clear[4] {};
	ASSERT_TRUE(_fence.checkPathBatch(paths, 4, clear));
	EXPECT_TRUE(clear[0]);
	EXPECT_TRUE(clear[1]);
	EXPECT_FALSE(clear[2]);
	EXPECT_FALSE(clear[3]);
}

INSTANTIATE_TEST_SUITE_P(LoiterFences, GeofenceLoiterTest, ::testing::Values(
				 LoiterFenceCase{"InclusionPolygon", false, true},
				 LoiterFenceCase{"ExclusionPolygon", false, false},
				 LoiterFenceCase{"InclusionCircle", true, true},
				 LoiterFenceCase{"ExclusionCircle", true, false}),
			 [](const ::testing::TestParamInfo<LoiterFenceCase> &test_info)
{
	return test_info.param.name;
});

TEST_F(GeofenceTest, CircleClearanceUsesMetresAtItsLatitude)
{
	// At 47 degrees latitude, the short east/west clearance needs longitude scaling.
	ASSERT_TRUE(loadFence(polygon(true, {{-1000.f, -100.f}, {1000.f, -100.f}, {1000.f, 100.f}, {-1000.f, 100.f}})));
	const auto center = position(0.f, 0.f);
	const Geofence::PathCheck paths[] {{center, center, 90.f}, {center, center, 110.f}};
	bool clear[2] {};
	ASSERT_TRUE(_fence.checkPathBatch(paths, 2, clear));
	EXPECT_TRUE(clear[0]);
	EXPECT_FALSE(clear[1]);
}

struct InvalidRadiusCase {
	const char *name;
	float radius;
};

class InvalidGeofenceRadiusTest : public GeofenceTest, public ::testing::WithParamInterface<InvalidRadiusCase> {};

TEST_P(InvalidGeofenceRadiusTest, RejectsWholeBatch)
{
	ASSERT_TRUE(loadFence({}));
	Geofence::PathCheck paths[] {path({0.f, 0.f}, {100.f, 0.f}), path({100.f, 0.f}, {100.f, 0.f})};
	paths[1].end_radius = GetParam().radius;
	bool clear[2] {true, true};
	EXPECT_FALSE(_fence.checkPathBatch(paths, 2, clear));
	EXPECT_FALSE(clear[0]);
	EXPECT_FALSE(clear[1]);
}

INSTANTIATE_TEST_SUITE_P(InvalidRadii, InvalidGeofenceRadiusTest, ::testing::Values(
				 InvalidRadiusCase{"Negative", -1.f},
				 InvalidRadiusCase{"NaN", NAN},
				 InvalidRadiusCase{"Infinite", INFINITY}),
			 [](const ::testing::TestParamInfo<InvalidRadiusCase> &test_info)
{
	return test_info.param.name;
});

TEST_F(GeofenceTest, FullBatchMatchesIndividualChecks)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	std::array<Geofence::PathCheck, Geofence::MAX_PATH_CHECKS> paths;
	std::array<bool, Geofence::MAX_PATH_CHECKS> clear{};

	// Alternate between crossing the square and passing 10 m north of it.
	for (size_t i = 0; i < paths.size(); ++i) {
		const float north = i % 2 == 0 ? 0.f : 60.f;
		paths[i] = path({north, 100.f}, {north, 500.f});
	}

	ASSERT_TRUE(_fence.checkPathBatch(paths.data(), paths.size(), clear.data()));

	for (size_t i = 0; i < paths.size(); ++i) {
		SCOPED_TRACE(i);
		bool single_clear = false;
		ASSERT_TRUE(_fence.checkPathBatch(&paths[i], 1, &single_clear));
		EXPECT_EQ(clear[i], single_clear);
		EXPECT_EQ(clear[i], i % 2 != 0);
	}
}

TEST_F(GeofenceTest, AllZonesApplyToTheBatch)
{
	// Two 100 m squares, with their centres 300 m apart north/south.
	FencePoints points = exclusionSquare();
	const FencePoints second = polygon(false, {{250.f, 250.f}, {350.f, 250.f}, {350.f, 350.f}, {250.f, 350.f}});
	points.insert(points.end(), second.begin(), second.end());
	ASSERT_TRUE(loadFence(points));
	// The first two paths cross one square each; the third passes through the gap.
	const Geofence::PathCheck paths[] {path({0.f, 100.f}, {0.f, 500.f}), path({300.f, 100.f}, {300.f, 500.f}),
					   path({150.f, 100.f}, {150.f, 500.f})
					  };
	bool clear[3] {};
	ASSERT_TRUE(_fence.checkPathBatch(paths, 3, clear));
	EXPECT_FALSE(clear[0]);
	EXPECT_FALSE(clear[1]);
	EXPECT_TRUE(clear[2]);
}

TEST_F(GeofenceTest, EmptyFenceIsAvailableOnlyAfterLoading)
{
	const Geofence::PathCheck paths[] {path({0.f, 0.f}, {100.f, 100.f}), path({0.f, 0.f}, {-100.f, -100.f})};
	bool clear[2] {true, true};
	EXPECT_FALSE(_fence.checkPathBatch(paths, 2, clear));
	EXPECT_FALSE(clear[0]);
	EXPECT_FALSE(clear[1]);
	ASSERT_TRUE(loadFence({}));
	ASSERT_TRUE(_fence.checkPathBatch(paths, 2, clear));
	EXPECT_TRUE(clear[0]);
	EXPECT_TRUE(clear[1]);
}

struct InvalidBatchCase {
	const char *name;
	size_t count;
	bool provide_paths;
	bool provide_results;
};

class InvalidGeofenceRequestTest : public GeofenceTest, public ::testing::WithParamInterface<InvalidBatchCase> {};

TEST_P(InvalidGeofenceRequestTest, RejectsRequest)
{
	const InvalidBatchCase &test = GetParam();
	ASSERT_TRUE(loadFence({}));
	const Geofence::PathCheck query = path({0.f, 0.f}, {100.f, 100.f});
	bool clear = true;
	EXPECT_FALSE(_fence.checkPathBatch(test.provide_paths ? &query : nullptr, test.count,
					   test.provide_results ? &clear : nullptr));

	if (!test.provide_paths) {
		EXPECT_FALSE(clear);
	}
}

INSTANTIATE_TEST_SUITE_P(InvalidRequests, InvalidGeofenceRequestTest, ::testing::Values(
				 InvalidBatchCase{"EmptyBatch", 0, true, true},
				 InvalidBatchCase{"BatchTooLarge", Geofence::MAX_PATH_CHECKS + 1, true, true},
				 InvalidBatchCase{"MissingPaths", 1, false, true},
				 InvalidBatchCase{"MissingResults", 1, true, false}),
			 [](const ::testing::TestParamInfo<InvalidBatchCase> &test_info)
{
	return test_info.param.name;
});

TEST_F(GeofenceTest, NonFinitePathRejectsWholeBatch)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	Geofence::PathCheck paths[] {path({100.f, 100.f}, {100.f, 500.f}), path({0.f, 100.f}, {0.f, 500.f})};
	paths[1].end(0) = NAN;
	bool clear[2] {true, true};
	EXPECT_FALSE(_fence.checkPathBatch(paths, 2, clear));
	EXPECT_FALSE(clear[0]);
	EXPECT_FALSE(clear[1]);
}

TEST_F(GeofenceTest, MissingCacheRejectsWholeBatch)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	GeofenceTestPeer::invalidateCache(_fence);
	const Geofence::PathCheck paths[] {path({100.f, 100.f}, {100.f, 500.f}), path({0.f, 100.f}, {0.f, 500.f})};
	bool clear[2] {true, true};
	EXPECT_FALSE(_fence.checkPathBatch(paths, 2, clear));
	EXPECT_FALSE(clear[0]);
	EXPECT_FALSE(clear[1]);
}

TEST_F(GeofenceTest, ReloadTemporarilyDisablesPathChecks)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	const Geofence::PathCheck query = path({100.f, 100.f}, {100.f, 500.f});
	bool clear = false;
	ASSERT_TRUE(_fence.checkPathBatch(&query, 1, &clear));
	ASSERT_TRUE(clear);
	_fence.updateFence();
	EXPECT_FALSE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_FALSE(clear);
	// An unchanged upload ID must restore availability once its metadata is read.
	ASSERT_TRUE(waitForFence());
	ASSERT_TRUE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_TRUE(clear);
}

TEST_F(GeofenceTest, ReadFailureKeepsPathChecksUnavailable)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	_fence.updateFence();
	_fence.run();
	_fence.run();
	ASSERT_TRUE(GeofenceTestPeer::failPendingRead(_fence));
	_fence.run();
	_fence.run();
	const Geofence::PathCheck query = path({100.f, 100.f}, {100.f, 500.f});
	bool clear = true;
	EXPECT_FALSE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_FALSE(clear);
	const auto inside_exclusion = position(0.f, 300.f);
	EXPECT_FALSE(_fence.checkPointAgainstAllGeofences(inside_exclusion(0), inside_exclusion(1), 500.f));
}

enum class InvalidFenceData { VertexCount, Longitude, Frame, Command };

struct InvalidFenceCase {
	const char *name;
	InvalidFenceData fault;
};

class InvalidGeofenceBatchTest : public GeofenceTest, public ::testing::WithParamInterface<InvalidFenceCase> {};

TEST_P(InvalidGeofenceBatchTest, RejectsWholeBatch)
{
	FencePoints points = exclusionSquare();
	ASSERT_TRUE(loadFence(points));

	if (GetParam().fault == InvalidFenceData::VertexCount) {
		GeofenceTestPeer::setVertexCount(_fence, 5);

	} else {
		mission_fence_point_s &point = points.back();

		if (GetParam().fault == InvalidFenceData::Longitude) {
			point.lon = NAN;

		} else if (GetParam().fault == InvalidFenceData::Frame) {
			point.frame = NAV_FRAME_LOCAL_NED;

		} else {
			point.nav_cmd = NAV_CMD_WAYPOINT;
		}

		ASSERT_TRUE(GeofenceTestPeer::replaceCachedPoint(_fence, points.size() - 1, point));
	}

	const Geofence::PathCheck paths[] {path({100.f, 100.f}, {100.f, 500.f}), path({0.f, 100.f}, {0.f, 500.f})};
	bool clear[2] {true, true};
	EXPECT_FALSE(_fence.checkPathBatch(paths, 2, clear));
	EXPECT_FALSE(clear[0]);
	EXPECT_FALSE(clear[1]);
}

TEST_P(InvalidGeofenceBatchTest, ClearsPartialResultsAndCanBeRetried)
{
	FencePoints points = exclusionSquare();
	const FencePoints second = polygon(false, {{250.f, 250.f}, {350.f, 250.f}, {350.f, 350.f}, {250.f, 350.f}});
	points.insert(points.end(), second.begin(), second.end());
	ASSERT_TRUE(loadFence(points));
	mission_fence_point_s invalid = points.back();

	switch (GetParam().fault) {
	case InvalidFenceData::VertexCount:
		invalid.vertex_count = static_cast<uint16_t>(points.size() + 1);
		break;

	case InvalidFenceData::Longitude:
		invalid.lon = NAN;
		break;

	case InvalidFenceData::Frame:
		invalid.frame = NAV_FRAME_LOCAL_NED;
		break;

	case InvalidFenceData::Command:
		invalid.nav_cmd = NAV_CMD_WAYPOINT;
		break;
	}

	ASSERT_TRUE(GeofenceTestPeer::replaceCachedPoint(_fence, points.size() - 1, invalid));
	// The first polygon clears one path and rejects the other before the second polygon fails.
	const Geofence::PathCheck paths[] {path({100.f, 100.f}, {100.f, 500.f}), path({0.f, 100.f}, {0.f, 500.f})};
	bool clear[2] {true, true};
	EXPECT_FALSE(_fence.checkPathBatch(paths, 2, clear));
	EXPECT_FALSE(clear[0]);
	EXPECT_FALSE(clear[1]);

	ASSERT_TRUE(GeofenceTestPeer::replaceCachedPoint(_fence, points.size() - 1, points.back()));
	ASSERT_TRUE(_fence.checkPathBatch(paths, 2, clear));
	EXPECT_TRUE(clear[0]);
	EXPECT_FALSE(clear[1]);
}

INSTANTIATE_TEST_SUITE_P(InvalidData, InvalidGeofenceBatchTest, ::testing::Values(
				 InvalidFenceCase{"VertexCountExceedsCache", InvalidFenceData::VertexCount},
				 InvalidFenceCase{"NonFiniteCoordinate", InvalidFenceData::Longitude},
				 InvalidFenceCase{"UnsupportedFrame", InvalidFenceData::Frame},
				 InvalidFenceCase{"UnexpectedCommand", InvalidFenceData::Command}),
			 [](const ::testing::TestParamInfo<InvalidFenceCase> &test_info)
{
	return test_info.param.name;
});

struct CircleBoundaryCase {
	const char *name;
	bool inclusion;
	matrix::Vector2f start;
	matrix::Vector2f end;
	bool clear;
};

class GeofenceCircleBoundaryTest : public GeofenceTest, public ::testing::WithParamInterface<CircleBoundaryCase> {};

TEST_P(GeofenceCircleBoundaryTest, ChecksContactInTheCircleProjection)
{
	// Build the path in the circle's equatorial frame to avoid rounding a tangent inward or outward.
	const CircleBoundaryCase &test = GetParam();
	FencePoints points = circle(test.inclusion, {0.f, 0.f}, 50.f);
	points.front().lat = 0.0;
	points.front().lon = 0.0;
	_navigator.get_home_position()->valid_hpos = false;
	GeofenceTestPeer::useEquatorProjection(_fence);
	ASSERT_TRUE(loadFence(points));
	const MapProjection projection(0.0, 0.0);
	Geofence::PathCheck query{};
	projection.reproject(test.start(0), test.start(1), query.start(0), query.start(1));
	projection.reproject(test.end(0), test.end(1), query.end(0), query.end(1));
	bool clear = false;
	ASSERT_TRUE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_EQ(clear, test.clear);
}

INSTANTIATE_TEST_SUITE_P(CircleBoundaries, GeofenceCircleBoundaryTest, ::testing::Values(
				 CircleBoundaryCase{"TangentExclusion", false, {-100.f, 50.f}, {100.f, 50.f}, false},
				 // The diagonal 3*north + 4*east = 250 lies exactly 50 m from the centre.
				 CircleBoundaryCase{"DiagonalTangentExclusion", false, {-30.f, 85.f}, {86.f, -2.f}, false},
				 CircleBoundaryCase{"ZeroLengthOnExclusion", false, {50.f, 0.f}, {50.f, 0.f}, false},
				 CircleBoundaryCase{"EndpointOnInclusion", true, {0.f, 0.f}, {50.f, 0.f}, false},
				 CircleBoundaryCase{"InsideInclusion", true, {0.f, 0.f}, {49.99f, 0.f}, true},
				 CircleBoundaryCase{"ClearOfExclusion", false, {-100.f, 50.01f}, {100.f, 50.01f}, true}),
			 [](const ::testing::TestParamInfo<CircleBoundaryCase> &test_info)
{
	return test_info.param.name;
});

struct CircleRoundingCase {
	const char *name;
	float radius;
	matrix::Vector2f endpoint;
	bool point_inside;
};

class GeofenceCircleRoundingTest : public GeofenceTest, public ::testing::WithParamInterface<CircleRoundingCase> {};

TEST_P(GeofenceCircleRoundingTest, RejectsContactInEitherCalculation)
{
	const CircleRoundingCase &test = GetParam();
	FencePoints points = circle(true, {0.f, 0.f}, test.radius);
	points.front().lat = 0.0;
	points.front().lon = 0.0;
	_navigator.get_home_position()->valid_hpos = false;
	GeofenceTestPeer::useEquatorProjection(_fence);
	ASSERT_TRUE(loadFence(points));
	const MapProjection projection(0.0, 0.0);
	Geofence::PathCheck query{};
	projection.reproject(test.endpoint(0), test.endpoint(1), query.end(0), query.end(1));
	float north, east;
	projection.project(query.end(0), query.end(1), north, east);
	ASSERT_FLOAT_EQ(north, test.endpoint(0));
	ASSERT_FLOAT_EQ(east, test.endpoint(1));
	EXPECT_EQ(test.point_inside, _fence.checkPointAgainstAllGeofences(query.end(0), query.end(1), 500.f));
	bool clear = true;
	ASSERT_TRUE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_FALSE(clear);
}

INSTANTIATE_TEST_SUITE_P(CircleRounding, GeofenceCircleRoundingTest, ::testing::Values(
				 // Float puts this point on the 100 m boundary; double puts it just inside.
				 CircleRoundingCase{"RoundedOntoBoundary", 100.f, {99.99484f, 1.0154841f}, false},
				 // A scaled 3-4-5 triangle: exactly on the circle, but float rounds it inside.
				 CircleRoundingCase{"RoundedInsideBoundary", 90.8984375f, {54.5390625f, 72.71875f}, true}),
			 [](const ::testing::TestParamInfo<CircleRoundingCase> &test_info)
{
	return test_info.param.name;
});

TEST_F(GeofenceTest, QueuedRefreshSurvivesCompletionOfPreviousLoad)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	_fence.updateFence();
	// Complete an older load while another refresh is still queued.
	GeofenceTestPeer::finishUpdate(_fence);
	const Geofence::PathCheck query = path({100.f, 100.f}, {100.f, 500.f});
	bool clear = true;
	EXPECT_FALSE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_FALSE(clear);
	ASSERT_TRUE(waitForFence());
	ASSERT_TRUE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_TRUE(clear);
}

TEST_F(GeofenceTest, QueuedRefreshSurvivesUnchangedFenceId)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	_fence.updateFence();
	_fence.run();
	_fence.run();
	ASSERT_TRUE(GeofenceTestPeer::waitForPendingRead(_fence));
	_fence.updateFence();
	_fence.run();
	const Geofence::PathCheck query = path({100.f, 100.f}, {100.f, 500.f});
	bool clear = true;
	EXPECT_FALSE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_FALSE(clear);
	ASSERT_TRUE(waitForFence());
	ASSERT_TRUE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_TRUE(clear);
}

struct InvalidCircleRadiusCase {
	const char *name;
	float radius;
};

class InvalidCircleRadiusTest : public GeofenceTest,
	public ::testing::WithParamInterface<InvalidCircleRadiusCase> {};

TEST_P(InvalidCircleRadiusTest, RejectsBatch)
{
	ASSERT_TRUE(loadFence(circle(false, {0.f, 300.f}, GetParam().radius)));
	const Geofence::PathCheck query = path({100.f, 100.f}, {100.f, 500.f});
	bool clear = true;
	EXPECT_FALSE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_FALSE(clear);
}

INSTANTIATE_TEST_SUITE_P(InvalidCircleRadii, InvalidCircleRadiusTest, ::testing::Values(
				 InvalidCircleRadiusCase{"Zero", 0.f},
				 InvalidCircleRadiusCase{"Negative", -50.f},
				 InvalidCircleRadiusCase{"NonFinite", NAN}),
			 [](const ::testing::TestParamInfo<InvalidCircleRadiusCase> &test_info)
{
	return test_info.param.name;
});

TEST_F(GeofenceTest, RepeatedVerticesDoNotChangeResults)
{
	ASSERT_TRUE(loadFence(polygon(false, {{-50.f, 250.f}, {50.f, 250.f}, {50.f, 250.f},
		{50.f, 350.f}, {-50.f, 350.f}, {-50.f, 250.f}
	})));
	const Geofence::PathCheck paths[] {path({100.f, 100.f}, {100.f, 500.f}), path({0.f, 100.f}, {0.f, 500.f})};
	bool clear[2] {};
	ASSERT_TRUE(_fence.checkPathBatch(paths, 2, clear));
	EXPECT_TRUE(clear[0]);
	EXPECT_FALSE(clear[1]);
}
