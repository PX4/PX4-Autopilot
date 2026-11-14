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

#include <parameters/param.h>

#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
#include "../RTLPlanner/geofence_avoidance_planner.h"
#endif

class PreGeofenceTest : public navigator_test::GeofenceTestBase
{
protected:
	void SetUp() override
	{
		param_reset_all();
		_navigator.updateParams();
		ASSERT_TRUE(resetFence());
	}

	void TearDown() override { param_reset_all(); }

	FencePoints inclusionSquare(float half_width, FenceLevel level = FenceLevel::Hard) const
	{
		return polygon(true, {{-half_width, -half_width}, {half_width, -half_width},
			{half_width, half_width}, {-half_width, half_width}
		}, level);
	}

	void setAction(int32_t action)
	{
		ASSERT_EQ(param_set(param_find("GF_PRE_ACTION"), &action), PX4_OK);
		_navigator.updateParams();
	}

	bool insideHard(float north, float east, float altitude = 400.f)
	{
		const auto coordinate = position(north, east);
		return _fence.isInsidePolygonOrCircle(coordinate(0), coordinate(1), altitude);
	}

	bool insidePre(float north, float east)
	{
		const auto coordinate = position(north, east);
		return _fence.isInsidePreGeofence(coordinate(0), coordinate(1));
	}

	bool insideAll(float north, float east)
	{
		const auto coordinate = position(north, east);
		return _fence.checkPointAgainstAllGeofences(coordinate(0), coordinate(1), 400.f);
	}
};

struct PreGeofenceShape {
	const char *name;
	bool circle_fence;
	bool inclusion;
};

class PreGeofenceShapeTest : public PreGeofenceTest, public ::testing::WithParamInterface<PreGeofenceShape> {};

TEST_P(PreGeofenceShapeTest, PointChecksSelectTheRequestedFenceLevels)
{
	const auto shape = GetParam();
	FencePoints points;

	for (const FenceLevel level : {FenceLevel::Hard, FenceLevel::Pre}) {
		const bool is_pre_geofence = level == FenceLevel::Pre;
		const float extent = shape.inclusion ? (is_pre_geofence ? 300.f : 500.f) : (is_pre_geofence ? 100.f : 50.f);
		FencePoints area;

		if (shape.circle_fence) {
			area = circle(shape.inclusion, {0.f, shape.inclusion ? 0.f : 300.f}, extent, level);

		} else if (shape.inclusion) {
			area = inclusionSquare(extent, level);

		} else {
			area = polygon(false, {{-extent, 300.f - extent}, {extent, 300.f - extent},
				{extent, 300.f + extent}, {-extent, 300.f + extent}
			}, level);
		}

		points.insert(points.end(), area.begin(), area.end());
	}

	ASSERT_TRUE(loadFence(points));
	EXPECT_NE(_fence.getPreGeofenceAction(), geofence_result_s::GF_ACTION_NONE);
	GeofenceInterface &interface = _fence;
	ASSERT_EQ(interface.getNumPolygons(), 2);
	EXPECT_EQ(interface.getPolygonInfoByIndex(0).fence_level, FenceLevel::Hard);
	EXPECT_EQ(interface.getPolygonInfoByIndex(1).fence_level, FenceLevel::Pre);
	EXPECT_TRUE(insideHard(0.f, 0.f));
	EXPECT_TRUE(insidePre(0.f, 0.f));
	EXPECT_TRUE(insideAll(0.f, 0.f));

	const float pre_breach_east = shape.inclusion ? 400.f : 225.f;
	EXPECT_TRUE(insideHard(0.f, pre_breach_east));
	EXPECT_FALSE(insidePre(0.f, pre_breach_east));
	EXPECT_FALSE(insideAll(0.f, pre_breach_east));
	const auto coordinate = position(0.f, pre_breach_east);
	EXPECT_FALSE(_fence.isInsideHorizontalFences(coordinate(0), coordinate(1), 400.f));

	const float both_breached_east = shape.inclusion ? 600.f : 300.f;
	EXPECT_FALSE(insideHard(0.f, both_breached_east));
	EXPECT_FALSE(insidePre(0.f, both_breached_east));
	EXPECT_FALSE(insideAll(0.f, both_breached_east));
}

INSTANTIATE_TEST_SUITE_P(Shapes, PreGeofenceShapeTest, ::testing::Values(
				 PreGeofenceShape{"InclusionPolygon", false, true},
				 PreGeofenceShape{"ExclusionPolygon", false, false},
				 PreGeofenceShape{"InclusionCircle", true, true},
				 PreGeofenceShape{"ExclusionCircle", true, false}),
			 [](const ::testing::TestParamInfo<PreGeofenceShape> &test_info)
{
	return test_info.param.name;
});

TEST_F(PreGeofenceTest, HardOnlyFenceHasNoPreAction)
{
	ASSERT_TRUE(loadFence(inclusionSquare(500.f)));
	EXPECT_EQ(_fence.getPreGeofenceAction(), geofence_result_s::GF_ACTION_NONE);
	EXPECT_FALSE(insideHard(0.f, 600.f));
	EXPECT_TRUE(insidePre(0.f, 600.f));
}

TEST_F(PreGeofenceTest, PreOnlyFenceDoesNotBecomeHardBoundary)
{
	ASSERT_TRUE(loadFence(inclusionSquare(300.f, FenceLevel::Pre)));
	EXPECT_TRUE(insideHard(0.f, 400.f));
	EXPECT_FALSE(insidePre(0.f, 400.f));
	EXPECT_FALSE(insideAll(0.f, 400.f));
}

TEST_F(PreGeofenceTest, PreActionAppliesOnlyWhilePreAreasAreLoaded)
{
	setAction(geofence_result_s::GF_ACTION_LAND);
	EXPECT_EQ(_fence.getPreGeofenceAction(), geofence_result_s::GF_ACTION_NONE);
	ASSERT_TRUE(loadFence(inclusionSquare(300.f, FenceLevel::Pre)));
	EXPECT_EQ(_fence.getPreGeofenceAction(), geofence_result_s::GF_ACTION_LAND);

	for (const int32_t unsupported : {0, 4, 6, -1}) {
		setAction(unsupported);
		EXPECT_EQ(_fence.getPreGeofenceAction(), geofence_result_s::GF_ACTION_WARN);
	}

	ASSERT_TRUE(loadFence({}));
	EXPECT_EQ(_fence.getPreGeofenceAction(), geofence_result_s::GF_ACTION_NONE);
}

TEST_F(PreGeofenceTest, PreReturnRequiresHome)
{
	setAction(geofence_result_s::GF_ACTION_RTL);
	EXPECT_FALSE(_fence.isHomeRequired());
	ASSERT_TRUE(loadFence(inclusionSquare(300.f, FenceLevel::Pre)));
	EXPECT_TRUE(_fence.isHomeRequired());
	setAction(geofence_result_s::GF_ACTION_LOITER);
	EXPECT_FALSE(_fence.isHomeRequired());
}

TEST_F(PreGeofenceTest, PreGeofenceHasNoAltitudeLimit)
{
	ASSERT_TRUE(loadFence(inclusionSquare(300.f, FenceLevel::Pre)));
	GeofenceTestPeer::setAltitudeBand(_fence, 450.f, 550.f);
	EXPECT_TRUE(insidePre(0.f, 0.f));
	EXPECT_FALSE(insideHard(0.f, 0.f, 400.f));
	EXPECT_TRUE(insideHard(0.f, 0.f, 500.f));
}

TEST_F(PreGeofenceTest, PreExclusionRejectsCrossingLegWithLegalEndpoints)
{
	ASSERT_TRUE(loadFence(circle(false, {0.f, 300.f}, 100.f, FenceLevel::Pre)));
	EXPECT_TRUE(insideAll(0.f, 0.f));
	EXPECT_TRUE(insideAll(0.f, 600.f));
	const Geofence::PathCheck query = path({0.f, 0.f}, {0.f, 600.f});
	bool clear = true;
	ASSERT_TRUE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_FALSE(clear);
}

#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
TEST_F(PreGeofenceTest, ReturnPlannerDetoursAroundPreGeofence)
{
	FencePoints points = polygon(false, {{-25.f, 275.f}, {25.f, 275.f}, {25.f, 325.f}, {-25.f, 325.f}});
	const FencePoints pre = polygon(false, {{-100.f, 200.f}, {100.f, 200.f}, {100.f, 400.f}, {-100.f, 400.f}},
	FenceLevel::Pre);
	points.insert(points.end(), pre.begin(), pre.end());
	ASSERT_TRUE(loadFence(points));
	const auto start = position(0.f, 0.f);
	const auto destination = position(0.f, 600.f);
	ASSERT_TRUE(insideAll(0.f, 0.f));
	ASSERT_TRUE(insideAll(0.f, 600.f));
	ASSERT_FALSE(insidePre(0.f, 300.f));

	GeofenceAvoidancePlanner planner;
	ASSERT_TRUE(planner.init(16)); // Both four-vertex polygons fit, including the corner-splitting allowance.
	planner.updateGraphFromGeofence(_fence, 10.f);
	planner.updateDestination(destination);
	const int waypoint_count = planner.updateStartAndFillPath(start);
	ASSERT_GE(waypoint_count, 2);
	ASSERT_FALSE(planner.needsStraightLineFallback());

	auto previous = start;

	for (int index = 0; index <= waypoint_count; ++index) {
		SCOPED_TRACE(index);
		const auto next = index < waypoint_count ? planner.waypointAtIndex(index) : destination;
		const auto midpoint = (previous + next) * 0.5;
		EXPECT_TRUE(_fence.isInsidePreGeofence(next(0), next(1)));
		EXPECT_TRUE(_fence.isInsidePreGeofence(midpoint(0), midpoint(1)));
		previous = next;
	}
}
#endif

TEST_F(PreGeofenceTest, RejectsUnknownLevelAndMixedPolygonLevels)
{
	FencePoints points = inclusionSquare(300.f, FenceLevel::Pre);
	points[1].fence_level = FenceLevel::Hard;
	ASSERT_TRUE(loadFence(points, geofence_status_s::GF_STATUS_FAILED));
	EXPECT_TRUE(_fence.isEmpty());
	EXPECT_EQ(_fence.getPreGeofenceAction(), geofence_result_s::GF_ACTION_NONE);
	ASSERT_TRUE(loadFence(circle(true, {0.f, 0.f}, 300.f, static_cast<FenceLevel>(2)), geofence_status_s::GF_STATUS_FAILED));
	EXPECT_TRUE(_fence.isEmpty());
	EXPECT_EQ(_fence.getPreGeofenceAction(), geofence_result_s::GF_ACTION_NONE);
}

TEST_F(PreGeofenceTest, BothLevelsCanUseTheirFullPointQuota)
{
	FencePoints points;

	for (const FenceLevel level : {FenceLevel::Hard, FenceLevel::Pre}) {
		const FencePoints area = circle(false, {0.f, 300.f}, level == FenceLevel::Hard ? 50.f : 100.f, level);
		points.insert(points.end(), kMaxFencePointsPerLevel, area.front());
	}

	ASSERT_TRUE(loadFence(points));
	EXPECT_TRUE(insideHard(0.f, 375.f));
	EXPECT_FALSE(insidePre(0.f, 375.f));
	EXPECT_FALSE(insideAll(0.f, 375.f));
	GeofenceInterface &interface = _fence;
	ASSERT_EQ(interface.getNumPolygons(), DM_KEY_FENCE_POINTS_MAX);
	EXPECT_EQ(interface.getPolygonInfoByIndex(DM_KEY_FENCE_POINTS_MAX - 1).dataman_index, DM_KEY_FENCE_POINTS_MAX - 1);

	const Geofence::PathCheck query = path({0.f, 375.f}, {0.f, 500.f});
	bool clear = true;
	ASSERT_TRUE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_FALSE(clear);
}

TEST_F(PreGeofenceTest, RejectsTooManyPointsAtEitherLevel)
{
	for (const FenceLevel level : {FenceLevel::Hard, FenceLevel::Pre}) {
		const FencePoints area = circle(false, {0.f, 300.f}, 100.f, level);
		const FencePoints points(kMaxFencePointsPerLevel + 1, area.front());
		ASSERT_TRUE(loadFence(points, geofence_status_s::GF_STATUS_FAILED));
		EXPECT_TRUE(_fence.isEmpty());
		EXPECT_FALSE(_fence.isFenceUpdatePending());
	}
}

TEST_F(PreGeofenceTest, PolygonsAndCirclesShareTheLevelPointQuota)
{
	for (const FenceLevel level : {FenceLevel::Hard, FenceLevel::Pre}) {
		FencePoints points = inclusionSquare(500.f, level);
		const FencePoints area = circle(false, {0.f, 300.f}, 100.f, level);
		points.insert(points.end(), kMaxFencePointsPerLevel - points.size(), area.front());
		ASSERT_TRUE(loadFence(points));
		points.push_back(area.front());
		ASSERT_TRUE(loadFence(points, geofence_status_s::GF_STATUS_FAILED));
		EXPECT_TRUE(_fence.isEmpty());
	}
}

TEST_F(PreGeofenceTest, RejectsPolygonLargerThanOneLevelQuota)
{
	constexpr unsigned vertex_count = kMaxFencePointsPerLevel + 1;
	FencePoints points;

	for (unsigned i = 0; i < vertex_count; ++i) {
		const double angle = 2.0 * M_PI * i / vertex_count;
		const auto coordinate = position(static_cast<float>(500.0 * cos(angle)), static_cast<float>(500.0 * sin(angle)));
		mission_fence_point_s point{};
		point.nav_cmd = NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION;
		point.frame = NAV_FRAME_GLOBAL;
		point.lat = coordinate(0);
		point.lon = coordinate(1);
		point.vertex_count = vertex_count;
		points.push_back(point);
	}

	for (const FenceLevel level : {FenceLevel::Hard, FenceLevel::Pre}) {
		for (auto &point : points) {
			point.fence_level = level;
		}

		ASSERT_TRUE(loadFence(points, geofence_status_s::GF_STATUS_FAILED));
		EXPECT_TRUE(_fence.isEmpty());
	}
}

// The action must not drop out for the moment in which the fence is empty during an update.
TEST_F(PreGeofenceTest, PreActionStaysWhileANewFenceLoads)
{
	ASSERT_TRUE(loadFence(inclusionSquare(300.f, FenceLevel::Pre)));
	ASSERT_TRUE(storeFence({}));
	_fence.updateFence();
	_fence.run(); // start the update
	_fence.run(); // send the metadata read
	ASSERT_TRUE(GeofenceTestPeer::waitForPendingRead(_fence));
	_fence.run(); // adopt the new ID and invalidate the data cache
	EXPECT_TRUE(_fence.isFenceUpdatePending());
	EXPECT_TRUE(_fence.isEmpty());
	EXPECT_EQ(_fence.getPreGeofenceAction(), geofence_result_s::GF_ACTION_LOITER);
	ASSERT_TRUE(waitForFence());
	EXPECT_EQ(_fence.getPreGeofenceAction(), geofence_result_s::GF_ACTION_NONE);
}

TEST_F(PreGeofenceTest, PreAreaMustContainHomeAndAirborneVehicle)
{
	const int32_t hard_action = geofence_result_s::GF_ACTION_NONE;
	ASSERT_EQ(param_set(param_find("GF_ACTION"), &hard_action), PX4_OK);
	_navigator.updateParams();
	// A pre area that excludes Home is discarded even when the hard action is disabled.
	ASSERT_TRUE(loadFence(circle(false, {0.f, 0.f}, 100.f, FenceLevel::Pre)));
	EXPECT_EQ(_fence.getPreGeofenceAction(), geofence_result_s::GF_ACTION_NONE);

	const auto current = position(0.f, 400.f);
	_navigator.get_global_position()->lat = current(0);
	_navigator.get_global_position()->lon = current(1);
	_navigator.get_land_detected()->landed = false;
	ASSERT_TRUE(loadFence(inclusionSquare(300.f, FenceLevel::Pre)));
	EXPECT_EQ(_fence.getPreGeofenceAction(), geofence_result_s::GF_ACTION_NONE);
}
