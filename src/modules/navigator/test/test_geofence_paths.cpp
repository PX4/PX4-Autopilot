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
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/mavlink_log.h>

class GeofenceTest : public navigator_test::GeofenceTestBase
{
protected:
	using navigator_test::GeofenceTestBase::GeofenceTestBase;

	void SetUp() override
	{
		ASSERT_TRUE(resetFence());
		// A new subscription still sees the last message of an earlier test.
		logContains("");
	}

	bool logContains(const char *text)
	{
		mavlink_log_s report{};
		bool found = false;

		while (_log_sub.update(&report)) {
			found |= strstr(reinterpret_cast<const char *>(report.text), text) != nullptr;
		}

		return found;
	}

	// A polygon that fills the fence storage, centred like exclusionSquare().
	FencePoints exclusionRing(size_t vertices) const
	{
		FencePoints points;

		for (size_t i = 0; i < vertices; ++i) {
			const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(vertices);
			const auto coordinate = position(static_cast<float>(50.0 * cos(angle)), static_cast<float>(300.0 + 50.0 * sin(angle)));
			mission_fence_point_s point{};
			point.nav_cmd = NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION;
			point.frame = NAV_FRAME_GLOBAL;
			point.lat = coordinate(0);
			point.lon = coordinate(1);
			point.vertex_count = static_cast<uint16_t>(vertices);
			points.push_back(point);
		}

		return points;
	}

	// Fail the metadata read of the next load attempt and return once the fence waits again.
	::testing::AssertionResult failFenceMetadataRead()
	{
		GeofenceTestPeer::expireRetryDelay(_fence);
		_fence.run(); // start the update
		_fence.run(); // send the metadata read

		if (!GeofenceTestPeer::failPendingRead(_fence)) {
			return ::testing::AssertionFailure() << "no fence read pending";
		}

		_fence.run(); // enter the error state
		_fence.run(); // return to waiting
		return ::testing::AssertionSuccess();
	}

	uORB::Subscription _log_sub{ORB_ID(mavlink_log)};
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

class GeofenceFileTest : public GeofenceTest, public ::testing::WithParamInterface<dm_item_t>
{
protected:
	~GeofenceFileTest() override { unlink(_filename); }

	char _filename[sizeof("/tmp/px4_geofence_XXXXXX")] {"/tmp/px4_geofence_XXXXXX"};
};

TEST_P(GeofenceFileTest, ImportedFenceAndSubsequentRefreshBecomeReady)
{
	const int fd = mkstemp(_filename);
	ASSERT_GE(fd, 0);
	const char contents[] = "0 1000\n46.999 7.999\n47.001 7.999\n47.001 8.001\n46.999 8.001\n";
	const ssize_t written = write(fd, contents, sizeof(contents) - 1);
	const int closed = close(fd);
	ASSERT_EQ(written, static_cast<ssize_t>(sizeof(contents) - 1));
	ASSERT_EQ(closed, 0);

	// Import must finish reading the current bank before choosing the inactive one.
	mission_stats_entry_s stats{};
	stats.dataman_id = GetParam();
	stats.opaque_id = ++runtime().next_fence_id;
	ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_FENCE_POINTS_STATE, 0,
					      reinterpret_cast<uint8_t *>(&stats), sizeof(stats)));
	ASSERT_EQ(_fence.loadFromFile(_filename), PX4_OK);
	ASSERT_TRUE(_dataman_client.readSync(DM_KEY_FENCE_POINTS_STATE, 0,
					     reinterpret_cast<uint8_t *>(&stats), sizeof(stats)));
	EXPECT_EQ(stats.dataman_id, GetParam() == DM_KEY_FENCE_POINTS_0 ? DM_KEY_FENCE_POINTS_1 : DM_KEY_FENCE_POINTS_0);
	EXPECT_EQ(stats.num_items, 4);
	_fence_id = stats.opaque_id;
	ASSERT_TRUE(waitForFence());
	EXPECT_FALSE(_fence.isEmpty());

	// One path stays inside the imported inclusion polygon; the other leaves it.
	const Geofence::PathCheck paths[] {path({0.f, -20.f}, {0.f, 20.f}), path({0.f, 0.f}, {0.f, 200.f})};
	bool clear[2] {};
	ASSERT_TRUE(_fence.checkPathBatch(paths, 2, clear));
	EXPECT_TRUE(clear[0]);
	EXPECT_FALSE(clear[1]);

	// The same client must remain usable for later asynchronous metadata reads.
	_fence.updateFence();
	ASSERT_TRUE(waitForFence());
	ASSERT_TRUE(_fence.checkPathBatch(paths, 2, clear));
	EXPECT_TRUE(clear[0]);
	EXPECT_FALSE(clear[1]);
}

INSTANTIATE_TEST_SUITE_P(StoredBanks, GeofenceFileTest,
			 ::testing::Values(DM_KEY_FENCE_POINTS_0, DM_KEY_FENCE_POINTS_1));

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

TEST_F(GeofenceTest, ReadFailureIsRetriedUntilPathChecksRecover)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	_fence.updateFence();
	ASSERT_TRUE(failFenceMetadataRead());
	EXPECT_TRUE(logContains("Geofence update failed, previous fence still active"));
	const Geofence::PathCheck query = path({100.f, 100.f}, {100.f, 500.f});
	bool clear = true;
	EXPECT_FALSE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_FALSE(clear);
	// The previous fence keeps serving point checks until the retry replaces it.
	const auto inside_exclusion = position(0.f, 300.f);
	EXPECT_FALSE(_fence.checkPointAgainstAllGeofences(inside_exclusion(0), inside_exclusion(1), 500.f));
	EXPECT_TRUE(GeofenceTestPeer::isUpdatePending(_fence));
	GeofenceTestPeer::expireRetryDelay(_fence);
	ASSERT_TRUE(waitForFence());
	EXPECT_FALSE(GeofenceTestPeer::isUpdatePending(_fence));
	ASSERT_TRUE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_TRUE(clear);
	EXPECT_TRUE(logContains("Geofence loaded, fence is active"));
}

TEST_F(GeofenceTest, RepeatedReadFailuresStopRetrying)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	_fence.updateFence();

	for (unsigned attempt = 0; attempt <= GeofenceTestPeer::maxLoadRetries(); ++attempt) {
		SCOPED_TRACE(attempt);
		ASSERT_TRUE(failFenceMetadataRead());
		// Only the first failure is reported; the retries run quietly.
		EXPECT_EQ(logContains("Geofence update failed, previous fence still active"), attempt == 0);
	}

	// The retry budget is spent: nothing is pending and no read is issued.
	EXPECT_FALSE(GeofenceTestPeer::isUpdatePending(_fence));
	EXPECT_FALSE(failFenceMetadataRead());
	const auto inside_exclusion = position(0.f, 300.f);
	EXPECT_FALSE(_fence.checkPointAgainstAllGeofences(inside_exclusion(0), inside_exclusion(1), 500.f));
	const Geofence::PathCheck query = path({100.f, 100.f}, {100.f, 500.f});
	bool clear = true;
	EXPECT_FALSE(_fence.checkPathBatch(&query, 1, &clear));
	// A new request starts over: its first failure is reported again, and its recovery announced.
	_fence.updateFence();
	ASSERT_TRUE(failFenceMetadataRead());
	EXPECT_TRUE(logContains("Geofence update failed, previous fence still active"));
	GeofenceTestPeer::expireRetryDelay(_fence);
	ASSERT_TRUE(waitForFence());
	EXPECT_TRUE(logContains("Geofence loaded, fence is active"));
	ASSERT_TRUE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_TRUE(clear);
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

TEST_F(GeofenceTest, FailedVertexReadClearsTheFenceUntilStorageRecovers)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	// The metadata claims one vertex more than the storage holds, so the dataman rejects that
	// cache slot and the load fails after the vertices were requested.
	ASSERT_TRUE(loadFence(exclusionRing(DM_KEY_FENCE_POINTS_MAX), geofence_status_s::GF_STATUS_FAILED, 1));
	EXPECT_TRUE(_fence.isEmpty());
	EXPECT_TRUE(GeofenceTestPeer::isUpdatePending(_fence));
	// A fragment is never kept, so point checks fail open while the retries run.
	const auto inside_exclusion = position(0.f, 300.f);
	EXPECT_TRUE(_fence.checkPointAgainstAllGeofences(inside_exclusion(0), inside_exclusion(1), 500.f));
	// The operator is told before the first retry, and only once.
	EXPECT_TRUE(logContains("Geofence load failed, fence is not active"));

	for (unsigned attempt = 0; attempt < GeofenceTestPeer::maxLoadRetries(); ++attempt) {
		SCOPED_TRACE(attempt);
		GeofenceTestPeer::expireRetryDelay(_fence);
		ASSERT_TRUE(waitForFence(geofence_status_s::GF_STATUS_FAILED));
	}

	EXPECT_FALSE(GeofenceTestPeer::isUpdatePending(_fence));
	EXPECT_FALSE(logContains("fence is not active"));
	// Repairing the stored fence and asking again recovers.
	ASSERT_TRUE(loadFence(exclusionSquare()));
	EXPECT_TRUE(logContains("Geofence loaded, fence is active"));
	EXPECT_FALSE(_fence.checkPointAgainstAllGeofences(inside_exclusion(0), inside_exclusion(1), 500.f));
	const Geofence::PathCheck query = path({100.f, 100.f}, {100.f, 500.f});
	bool clear = false;
	ASSERT_TRUE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_TRUE(clear);
}

TEST_F(GeofenceTest, InactiveFenceIsReportedEvenIfARetryRecovers)
{
	ASSERT_TRUE(loadFence(exclusionRing(DM_KEY_FENCE_POINTS_MAX), geofence_status_s::GF_STATUS_FAILED, 1));
	EXPECT_TRUE(logContains("Geofence load failed, fence is not active"));
	// The storage is repaired without a new request; the scheduled retry picks it up.
	ASSERT_TRUE(storeFence(exclusionSquare()));
	GeofenceTestPeer::expireRetryDelay(_fence);
	ASSERT_TRUE(waitForFence());
	EXPECT_TRUE(logContains("Geofence loaded, fence is active"));
	EXPECT_FALSE(GeofenceTestPeer::isUpdatePending(_fence));
	const auto inside_exclusion = position(0.f, 300.f);
	EXPECT_FALSE(_fence.checkPointAgainstAllGeofences(inside_exclusion(0), inside_exclusion(1), 500.f));
}

TEST_F(GeofenceTest, RetryThatDropsThePreviousFenceIsReported)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	// The next fence claims one vertex more than the storage holds.
	ASSERT_TRUE(storeFence(exclusionRing(DM_KEY_FENCE_POINTS_MAX), 1));
	_fence.updateFence();
	ASSERT_TRUE(failFenceMetadataRead());
	EXPECT_TRUE(logContains("Geofence update failed, previous fence still active"));
	// The retry reads the metadata, drops the previous fence and fails on the vertices.
	GeofenceTestPeer::expireRetryDelay(_fence);
	ASSERT_TRUE(waitForFence(geofence_status_s::GF_STATUS_FAILED));
	EXPECT_TRUE(_fence.isEmpty());
	EXPECT_TRUE(logContains("Geofence load failed, fence is not active"));
}

TEST_F(GeofenceTest, UnreadableMetadataWithoutFenceIsReportedBeforeRetrying)
{
	ASSERT_TRUE(failFenceMetadataRead());
	EXPECT_TRUE(GeofenceTestPeer::isUpdatePending(_fence));
	EXPECT_TRUE(logContains("Geofence load failed, fence is not active"));
}

TEST_F(GeofenceTest, ReadFailureWithEmptyFenceIsReportedAsInactive)
{
	ASSERT_TRUE(loadFence({}));
	_fence.updateFence();
	ASSERT_TRUE(failFenceMetadataRead());
	EXPECT_TRUE(logContains("Geofence load failed, fence is not active"));
	// Reading the unchanged empty fence succeeds, but does not restore protection.
	GeofenceTestPeer::expireRetryDelay(_fence);
	ASSERT_TRUE(waitForFence());
	EXPECT_TRUE(_fence.isEmpty());
	EXPECT_FALSE(GeofenceTestPeer::isUpdatePending(_fence));
	EXPECT_TRUE(logContains("Geofence loaded, no fence configured"));
}

TEST_F(GeofenceTest, RecoveryWithEmptyFenceDoesNotClaimProtection)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	_fence.updateFence();
	ASSERT_TRUE(failFenceMetadataRead());
	EXPECT_TRUE(logContains("Geofence update failed, previous fence still active"));
	// Clearing the stored fence lets the retry succeed, leaving no configured fence.
	ASSERT_TRUE(storeFence({}));
	GeofenceTestPeer::expireRetryDelay(_fence);
	ASSERT_TRUE(waitForFence());
	EXPECT_TRUE(_fence.isEmpty());
	EXPECT_FALSE(GeofenceTestPeer::isUpdatePending(_fence));
	EXPECT_TRUE(logContains("Geofence loaded, no fence configured"));
}

enum class InvalidFenceItem {
	TwoVertexPolygon, ZeroRadius, NegativeRadius, NonFiniteRadius, NonFiniteVertex, LocalFrame,
	MixedVertexType, VertexCountMismatch, TruncatedPolygon, AntimeridianEdge
};

struct InvalidFenceLoadCase {
	const char *name;
	InvalidFenceItem fault;
};

class InvalidGeofenceLoadTest : public GeofenceTest, public ::testing::WithParamInterface<InvalidFenceLoadCase> {};

TEST_P(InvalidGeofenceLoadTest, RejectsFenceWhenLoading)
{
	FencePoints points = exclusionSquare();

	switch (GetParam().fault) {
	case InvalidFenceItem::TwoVertexPolygon:
		points = polygon(false, {{-50.f, 250.f}, {50.f, 250.f}});
		break;

	case InvalidFenceItem::ZeroRadius:
		points = circle(false, {0.f, 300.f}, 0.f);
		break;

	case InvalidFenceItem::NegativeRadius:
		points = circle(false, {0.f, 300.f}, -50.f);
		break;

	case InvalidFenceItem::NonFiniteRadius:
		points = circle(false, {0.f, 300.f}, NAN);
		break;

	case InvalidFenceItem::NonFiniteVertex:
		points[2].lon = NAN;
		break;

	case InvalidFenceItem::LocalFrame:
		points[1].frame = NAV_FRAME_LOCAL_NED;
		break;

	case InvalidFenceItem::MixedVertexType:
		points[3].nav_cmd = NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION;
		break;

	case InvalidFenceItem::VertexCountMismatch:
		points[1].vertex_count = 3;
		break;

	case InvalidFenceItem::TruncatedPolygon:
		points.pop_back();
		break;

	case InvalidFenceItem::AntimeridianEdge:
		points[2].lon = -179.0;
		break;
	}

	ASSERT_TRUE(loadFence(points, geofence_status_s::GF_STATUS_FAILED));
	// Invalid data is reported once, not retried, and leaves no fence behind.
	EXPECT_FALSE(GeofenceTestPeer::isUpdatePending(_fence));
	EXPECT_TRUE(_fence.isEmpty());
	EXPECT_TRUE(logContains("invalid, fence is not active"));
	const Geofence::PathCheck query = path({0.f, 100.f}, {0.f, 500.f});
	bool clear = true;
	EXPECT_FALSE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_FALSE(clear);
	// Uploading a valid fence afterwards is announced.
	ASSERT_TRUE(loadFence(exclusionSquare()));
	EXPECT_TRUE(logContains("Geofence loaded, fence is active"));
}

INSTANTIATE_TEST_SUITE_P(InvalidFences, InvalidGeofenceLoadTest, ::testing::Values(
				 InvalidFenceLoadCase{"TwoVertexPolygon", InvalidFenceItem::TwoVertexPolygon},
				 InvalidFenceLoadCase{"ZeroRadius", InvalidFenceItem::ZeroRadius},
				 InvalidFenceLoadCase{"NegativeRadius", InvalidFenceItem::NegativeRadius},
				 InvalidFenceLoadCase{"NonFiniteRadius", InvalidFenceItem::NonFiniteRadius},
				 InvalidFenceLoadCase{"NonFiniteVertex", InvalidFenceItem::NonFiniteVertex},
				 InvalidFenceLoadCase{"LocalFrame", InvalidFenceItem::LocalFrame},
				 InvalidFenceLoadCase{"MixedVertexType", InvalidFenceItem::MixedVertexType},
				 InvalidFenceLoadCase{"VertexCountMismatch", InvalidFenceItem::VertexCountMismatch},
				 InvalidFenceLoadCase{"TruncatedPolygon", InvalidFenceItem::TruncatedPolygon},
				 InvalidFenceLoadCase{"AntimeridianEdge", InvalidFenceItem::AntimeridianEdge}),
			 [](const ::testing::TestParamInfo<InvalidFenceLoadCase> &test_info)
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

// Home sits next to the antimeridian.
class AntimeridianGeofenceTest : public GeofenceTest
{
protected:
	AntimeridianGeofenceTest() : GeofenceTest(47.0, 179.999) {}

	static mission_fence_point_s fencePoint(uint16_t nav_cmd, double lat, double lon)
	{
		mission_fence_point_s point{};
		point.nav_cmd = nav_cmd;
		point.frame = NAV_FRAME_GLOBAL;
		point.lat = lat;
		point.lon = lon;
		return point;
	}

	// A 0.0004 deg exclusion square, about 30 m by 44 m here.
	static FencePoints exclusionAt(double lat, double lon)
	{
		const double corners[4][2] {{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}};
		FencePoints points;

		for (const auto &corner : corners) {
			points.push_back(fencePoint(NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION, lat + 0.0002 * corner[0], lon + 0.0002 * corner[1]));
			points.back().vertex_count = 4;
		}

		return points;
	}
};

TEST_F(AntimeridianGeofenceTest, ShortPathsMayCrossTheAntimeridian)
{
	// About 150 m across the antimeridian, in both directions.
	const Geofence::PathCheck paths[] {{{47.0, 179.999}, {47.0, -179.999}}, {{47.0, -179.999}, {47.0, 179.999}}};
	mission_fence_point_s circle = fencePoint(NAV_CMD_FENCE_CIRCLE_EXCLUSION, 47.0, 180.0);
	circle.circle_radius = 20.f;

	const struct {
		const char *name;
		FencePoints fence;
		bool clear;
	} cases[] {
		{"NoFence", {}, true},
		{"ExclusionEastOfAntimeridian", exclusionAt(47.0, -179.9995), false},
		{"ExclusionWestOfAntimeridian", exclusionAt(47.0, 179.9995), false},
		{"ExclusionNorthOfPath", exclusionAt(47.001, -179.9995), true},
		{"ExclusionAcrossTheGlobe", exclusionAt(47.0, 0.0), true},
		{"ExclusionCircleOnAntimeridian", {circle}, false},
	};

	for (const auto &test : cases) {
		SCOPED_TRACE(test.name);
		ASSERT_TRUE(loadFence(test.fence));
		bool clear[2] {};
		ASSERT_TRUE(_fence.checkPathBatch(paths, 2, clear));
		EXPECT_EQ(clear[0], test.clear);
		EXPECT_EQ(clear[1], test.clear);
	}
}

TEST_F(AntimeridianGeofenceTest, MultiPiecePathsMayCrossTheAntimeridian)
{
	// This 30 km leg needs several pieces and bows about 19 m north of the straight lat/lon line.
	const Geofence::PathCheck leg{{47.0, 179.8}, {47.0, -179.8}};
	const Geofence::PathCheck paths[] {leg, {leg.end, leg.start}};
	const MapProjection projection(leg.start(0), leg.start(1));
	const matrix::Vector2f end = projection.project(leg.end(0), leg.end(1));
	const matrix::Vector2f corners[] {{-3.f, -3.f}, {3.f, -3.f}, {3.f, 3.f}, {-3.f, 3.f}};

	// Put small squares on either side of the longitude wrap, away from the endpoints.
	for (float fraction : {0.25f, 0.75f}) {
		SCOPED_TRACE(fraction);

		for (float north_offset : {0.f, -10.f}) {
			SCOPED_TRACE(north_offset);
			const matrix::Vector2f centre = end * fraction + matrix::Vector2f{north_offset, 0.f};
			FencePoints points = exclusionSquare();

			for (size_t i = 0; i < points.size(); ++i) {
				projection.reproject(centre(0) + corners[i](0), centre(1) + corners[i](1), points[i].lat, points[i].lon);
				points[i].lon = matrix::wrap(points[i].lon, -180.0, 180.0);
			}

			ASSERT_TRUE(loadFence(points));
			EXPECT_TRUE(_fence.checkPointAgainstAllGeofences(leg.start(0), leg.start(1), 500.f));
			EXPECT_TRUE(_fence.checkPointAgainstAllGeofences(leg.end(0), leg.end(1), 500.f));
			bool clear[2] {};
			ASSERT_TRUE(_fence.checkPathBatch(paths, 2, clear));
			EXPECT_EQ(clear[0], north_offset < 0.f);
			EXPECT_EQ(clear[1], north_offset < 0.f);
		}
	}
}

TEST_F(GeofenceTest, LongPathsFollowTheFlownLine)
{
	const MapProjection projection(_reference(0), _reference(1));

	for (float length : {20000.f, 50000.f, 100000.f}) {
		SCOPED_TRACE(length);
		// East-west legs 1 km north of Home. The 20 km leg flies about 8.4 m north of
		// the straight lat/lon line at its middle; derive the midpoint from the guidance projection.
		const Geofence::PathCheck leg = path({1000.f, -length / 2.f}, {1000.f, length / 2.f});
		const Geofence::PathCheck paths[] {leg, {leg.end, leg.start}};
		const matrix::Vector2f middle = (projection.project(leg.start(0), leg.start(1))
						 + projection.project(leg.end(0), leg.end(1))) * 0.5f;
		double middle_lat, middle_lon;
		projection.reproject(middle(0), middle(1), middle_lat, middle_lon);
		FencePoints points = exclusionSquare();
		const matrix::Vector2f corners[] {{-5.f, -5.f}, {5.f, -5.f}, {5.f, 5.f}, {-5.f, 5.f}};

		for (size_t i = 0; i < points.size(); ++i) {
			projection.reproject(middle(0) + corners[i](0), middle(1) + corners[i](1), points[i].lat, points[i].lon);
		}

		// A 10 m square on the flown line but clear of the straight lat/lon line.
		ASSERT_TRUE(loadFence(points));
		EXPECT_TRUE(_fence.checkPointAgainstAllGeofences(leg.start(0), leg.start(1), 500.f));
		EXPECT_TRUE(_fence.checkPointAgainstAllGeofences(leg.end(0), leg.end(1), 500.f));
		EXPECT_FALSE(_fence.checkPointAgainstAllGeofences(middle_lat, middle_lon, 500.f));
		bool clear[2] {true, true};
		ASSERT_TRUE(_fence.checkPathBatch(paths, 2, clear));
		EXPECT_FALSE(clear[0]);
		EXPECT_FALSE(clear[1]);

		// A 4 m square on the straight lat/lon line but clear of the flown line.
		ASSERT_TRUE(loadFence(polygon(false, {{998.f, -2.f}, {1002.f, -2.f}, {1002.f, 2.f}, {998.f, 2.f}})));
		EXPECT_TRUE(_fence.checkPointAgainstAllGeofences(middle_lat, middle_lon, 500.f));
		ASSERT_TRUE(_fence.checkPathBatch(paths, 2, clear));
		EXPECT_TRUE(clear[0]);
		EXPECT_TRUE(clear[1]);
	}
}

TEST_F(GeofenceTest, NearPolarPathsAccountForLatitudeAlongTheLeg)
{
	for (double latitude : {89.9, -89.9}) {
		SCOPED_TRACE(latitude);
		const Geofence::PathCheck paths[] {{{latitude, -60.0}, {latitude, 60.0}}, {{latitude, 60.0}, {latitude, -60.0}}};
		// This 19 km leg reaches almost twice as close to the pole as its endpoints.
		// Place a 6 m square just before its midpoint, on the straight projected route.
		const MapProjection projection(latitude, -60.0);
		float north, east;
		projection.project(latitude, 60.0, north, east);
		double center_lat, center_lon;
		projection.reproject(0.49f * north, 0.49f * east, center_lat, center_lon);
		const MapProjection fence_projection(center_lat, center_lon);
		FencePoints points = exclusionSquare();
		const matrix::Vector2f corners[] {{-3.f, -3.f}, {3.f, -3.f}, {3.f, 3.f}, {-3.f, 3.f}};

		for (size_t i = 0; i < points.size(); ++i) {
			fence_projection.reproject(corners[i](0), corners[i](1), points[i].lat, points[i].lon);
		}

		ASSERT_TRUE(loadFence(points));
		EXPECT_TRUE(_fence.checkPointAgainstAllGeofences(paths[0].start(0), paths[0].start(1), 500.f));
		EXPECT_TRUE(_fence.checkPointAgainstAllGeofences(paths[0].end(0), paths[0].end(1), 500.f));
		EXPECT_FALSE(_fence.checkPointAgainstAllGeofences(center_lat, center_lon, 500.f));
		bool clear[2] {true, true};
		ASSERT_TRUE(_fence.checkPathBatch(paths, 2, clear));
		EXPECT_FALSE(clear[0]);
		EXPECT_FALSE(clear[1]);
	}
}

TEST_F(GeofenceTest, UnresolvedPolygonPathsRejectTheWholeBatch)
{
	const Geofence::PathCheck unsupported[] {
		{{89.99, -90.0}, {89.99, 90.0}}, // crosses a pole, where longitude is undefined
		{{47.0, -80.0}, {47.0, 80.0}}, // exceeds the bounded subdivision budget
		{{12.0, 0.0}, {-12.0, 180.0}}, // antipodal endpoints do not define a unique great circle
	};

	for (const auto &query : unsupported) {
		ASSERT_TRUE(loadFence(exclusionSquare()));
		const Geofence::PathCheck paths[] {path({100.f, 100.f}, {100.f, 500.f}), query};
		bool clear[2] {true, true};
		EXPECT_FALSE(_fence.checkPathBatch(paths, 2, clear));
		EXPECT_FALSE(clear[0]);
		EXPECT_FALSE(clear[1]);
		// No polygon approximation is needed when there is no fence to check.
		ASSERT_TRUE(loadFence({}));
		ASSERT_TRUE(_fence.checkPathBatch(paths, 2, clear));
		EXPECT_TRUE(clear[0]);
		EXPECT_TRUE(clear[1]);
	}
}
