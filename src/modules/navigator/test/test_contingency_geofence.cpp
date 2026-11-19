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

class ContGeofenceTest : public navigator_test::GeofenceTestBase
{
protected:
	void SetUp() override
	{
		param_reset_all();
		_navigator.updateParams();
		ASSERT_TRUE(resetFence());
	}

	void TearDown() override { param_reset_all(); }

	FencePoints inclusionSquare(float half_width, FenceLevel level = FenceLevel::Emergency) const
	{
		return polygon(true, {{-half_width, -half_width}, {half_width, -half_width},
			{half_width, half_width}, {-half_width, half_width}
		}, level);
	}

	void setAction(int32_t action)
	{
		ASSERT_EQ(param_set(param_find("GF_CONT_ACTION"), &action), PX4_OK);
		_navigator.updateParams();
	}

	void setMargin(float margin)
	{
		ASSERT_EQ(param_set(param_find("GF_CONT_H_MARGIN"), &margin), PX4_OK);
		_navigator.updateParams();
	}

	bool insideEmergency(float north, float east, float altitude = 400.f)
	{
		const auto coordinate = position(north, east);
		return _fence.isInsidePolygonOrCircle(coordinate(0), coordinate(1), altitude);
	}

	bool insideCont(float north, float east)
	{
		const auto coordinate = position(north, east);
		return _fence.isInsideContGeofence(coordinate(0), coordinate(1));
	}

	// What the first position of a mission has to pass.
	bool insideAll(float north, float east)
	{
		const auto coordinate = position(north, east);
		return _fence.checkPointAgainstAllGeofences(coordinate(0), coordinate(1), 400.f)
		       && _fence.isInsideContGeofenceAreas(coordinate(0), coordinate(1));
	}
};

struct ContGeofenceShape {
	const char *name;
	bool circle_fence;
	bool inclusion;
};

class ContGeofenceShapeTest : public ContGeofenceTest, public ::testing::WithParamInterface<ContGeofenceShape> {};

TEST_P(ContGeofenceShapeTest, PointChecksSelectTheRequestedFenceLevels)
{
	const auto shape = GetParam();
	FencePoints points;

	for (const FenceLevel level : {FenceLevel::Emergency, FenceLevel::Contingency}) {
		const bool is_cont_geofence = level == FenceLevel::Contingency;
		const float extent = shape.inclusion ? (is_cont_geofence ? 300.f : 500.f) : (is_cont_geofence ? 100.f : 50.f);
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
	EXPECT_NE(_fence.getContGeofenceAction(), geofence_result_s::GF_ACTION_NONE);
	GeofenceInterface &interface = _fence;
	ASSERT_EQ(interface.getNumPolygons(), 2);
	EXPECT_EQ(interface.getPolygonInfoByIndex(0).fence_level, FenceLevel::Emergency);
	EXPECT_EQ(interface.getPolygonInfoByIndex(1).fence_level, FenceLevel::Contingency);
	EXPECT_TRUE(insideEmergency(0.f, 0.f));
	EXPECT_TRUE(insideCont(0.f, 0.f));
	EXPECT_TRUE(insideAll(0.f, 0.f));

	const float cont_breach_east = shape.inclusion ? 400.f : 225.f;
	EXPECT_TRUE(insideEmergency(0.f, cont_breach_east));
	EXPECT_FALSE(insideCont(0.f, cont_breach_east));
	EXPECT_FALSE(insideAll(0.f, cont_breach_east));
	const auto coordinate = position(0.f, cont_breach_east);
	EXPECT_FALSE(_fence.isInsideHorizontalFences(coordinate(0), coordinate(1), 400.f));

	const float both_breached_east = shape.inclusion ? 600.f : 300.f;
	EXPECT_FALSE(insideEmergency(0.f, both_breached_east));
	EXPECT_FALSE(insideCont(0.f, both_breached_east));
	EXPECT_FALSE(insideAll(0.f, both_breached_east));
}

INSTANTIATE_TEST_SUITE_P(Shapes, ContGeofenceShapeTest, ::testing::Values(
				 ContGeofenceShape{"InclusionPolygon", false, true},
				 ContGeofenceShape{"ExclusionPolygon", false, false},
				 ContGeofenceShape{"InclusionCircle", true, true},
				 ContGeofenceShape{"ExclusionCircle", true, false}),
			 [](const ::testing::TestParamInfo<ContGeofenceShape> &test_info)
{
	return test_info.param.name;
});

TEST_F(ContGeofenceTest, EmergencyOnlyFenceHasNoContingencyAction)
{
	ASSERT_TRUE(loadFence(inclusionSquare(500.f)));
	EXPECT_EQ(_fence.getContGeofenceAction(), geofence_result_s::GF_ACTION_NONE);
	EXPECT_FALSE(insideEmergency(0.f, 600.f));
	EXPECT_TRUE(insideCont(0.f, 600.f));
}

TEST_F(ContGeofenceTest, ContingencyOnlyFenceDoesNotBecomeEmergencyBoundary)
{
	ASSERT_TRUE(loadFence(inclusionSquare(300.f, FenceLevel::Contingency)));
	EXPECT_TRUE(insideEmergency(0.f, 400.f));
	EXPECT_FALSE(insideCont(0.f, 400.f));
	EXPECT_FALSE(insideAll(0.f, 400.f));
}

TEST_F(ContGeofenceTest, ContingencyActionAppliesOnlyWhileItsAreasAreLoaded)
{
	setAction(geofence_result_s::GF_ACTION_LAND);
	EXPECT_EQ(_fence.getContGeofenceAction(), geofence_result_s::GF_ACTION_NONE);
	ASSERT_TRUE(loadFence(inclusionSquare(300.f, FenceLevel::Contingency)));
	EXPECT_EQ(_fence.getContGeofenceAction(), geofence_result_s::GF_ACTION_LAND);

	// With None nothing is triggered, also while areas are loaded.
	setAction(geofence_result_s::GF_ACTION_NONE);
	EXPECT_EQ(_fence.getContGeofenceAction(), geofence_result_s::GF_ACTION_NONE);

	setAction(geofence_result_s::GF_ACTION_LAND);
	ASSERT_TRUE(loadFence({}));
	EXPECT_EQ(_fence.getContGeofenceAction(), geofence_result_s::GF_ACTION_NONE);
}

TEST_F(ContGeofenceTest, ContingencyReturnRequiresHome)
{
	setAction(geofence_result_s::GF_ACTION_RTL);
	EXPECT_FALSE(_fence.isHomeRequired());
	ASSERT_TRUE(loadFence(inclusionSquare(300.f, FenceLevel::Contingency)));
	EXPECT_TRUE(_fence.isHomeRequired());
	setAction(geofence_result_s::GF_ACTION_LOITER);
	EXPECT_FALSE(_fence.isHomeRequired());
}

TEST_F(ContGeofenceTest, ContGeofenceHasNoAltitudeLimit)
{
	ASSERT_TRUE(loadFence(inclusionSquare(300.f, FenceLevel::Contingency)));
	GeofenceTestPeer::setAltitudeBand(_fence, 450.f, 550.f);
	EXPECT_TRUE(insideCont(0.f, 0.f));
	EXPECT_FALSE(insideEmergency(0.f, 0.f, 400.f));
	EXPECT_TRUE(insideEmergency(0.f, 0.f, 500.f));
}

TEST_F(ContGeofenceTest, ContingencyExclusionRejectsCrossingLegWithLegalEndpoints)
{
	ASSERT_TRUE(loadFence(circle(false, {0.f, 300.f}, 100.f, FenceLevel::Contingency)));
	EXPECT_TRUE(insideAll(0.f, 0.f));
	EXPECT_TRUE(insideAll(0.f, 600.f));
	const Geofence::PathCheck query = path({0.f, 0.f}, {0.f, 600.f});
	bool clear = true;
	ASSERT_TRUE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_FALSE(clear);
}

#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
TEST_F(ContGeofenceTest, ReturnPlannerDetoursAroundContGeofence)
{
	FencePoints points = polygon(false, {{-25.f, 275.f}, {25.f, 275.f}, {25.f, 325.f}, {-25.f, 325.f}});
	const FencePoints contingency = polygon(false, {{-100.f, 200.f}, {100.f, 200.f}, {100.f, 400.f}, {-100.f, 400.f}},
	FenceLevel::Contingency);
	points.insert(points.end(), contingency.begin(), contingency.end());
	ASSERT_TRUE(loadFence(points));
	const auto start = position(0.f, 0.f);
	const auto destination = position(0.f, 600.f);
	ASSERT_TRUE(insideAll(0.f, 0.f));
	ASSERT_TRUE(insideAll(0.f, 600.f));
	ASSERT_FALSE(insideCont(0.f, 300.f));

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
		EXPECT_TRUE(_fence.isInsideContGeofence(next(0), next(1)));
		EXPECT_TRUE(_fence.isInsideContGeofence(midpoint(0), midpoint(1)));
		previous = next;
	}
}
#endif

TEST_F(ContGeofenceTest, RejectsUnknownLevelAndMixedPolygonLevels)
{
	FencePoints points = inclusionSquare(300.f, FenceLevel::Contingency);
	points[1].fence_level = FenceLevel::Emergency;
	ASSERT_TRUE(loadFence(points, geofence_status_s::GF_STATUS_FAILED));
	EXPECT_TRUE(_fence.isEmpty());
	EXPECT_EQ(_fence.getContGeofenceAction(), geofence_result_s::GF_ACTION_NONE);
	ASSERT_TRUE(loadFence(circle(true, {0.f, 0.f}, 300.f, static_cast<FenceLevel>(2)), geofence_status_s::GF_STATUS_FAILED));
	EXPECT_TRUE(_fence.isEmpty());
	EXPECT_EQ(_fence.getContGeofenceAction(), geofence_result_s::GF_ACTION_NONE);
}

TEST_F(ContGeofenceTest, BothLevelsCanUseTheirFullPointQuota)
{
	FencePoints points;

	for (const FenceLevel level : {FenceLevel::Emergency, FenceLevel::Contingency}) {
		const FencePoints area = circle(false, {0.f, 300.f}, level == FenceLevel::Emergency ? 50.f : 100.f, level);
		points.insert(points.end(), kMaxFencePointsPerLevel, area.front());
	}

	ASSERT_TRUE(loadFence(points));
	EXPECT_TRUE(insideEmergency(0.f, 375.f));
	EXPECT_FALSE(insideCont(0.f, 375.f));
	EXPECT_FALSE(insideAll(0.f, 375.f));
	GeofenceInterface &interface = _fence;
	ASSERT_EQ(interface.getNumPolygons(), DM_KEY_FENCE_POINTS_MAX);
	EXPECT_EQ(interface.getPolygonInfoByIndex(DM_KEY_FENCE_POINTS_MAX - 1).dataman_index, DM_KEY_FENCE_POINTS_MAX - 1);

	const Geofence::PathCheck query = path({0.f, 375.f}, {0.f, 500.f});
	bool clear = true;
	ASSERT_TRUE(_fence.checkPathBatch(&query, 1, &clear));
	EXPECT_FALSE(clear);
}

TEST_F(ContGeofenceTest, RejectsPolygonLargerThanOneLevelQuota)
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

	for (const FenceLevel level : {FenceLevel::Emergency, FenceLevel::Contingency}) {
		for (auto &point : points) {
			point.fence_level = level;
		}

		ASSERT_TRUE(loadFence(points, geofence_status_s::GF_STATUS_FAILED));
		EXPECT_TRUE(_fence.isEmpty());
	}
}

// The action must not drop out for the moment in which the fence is empty during an update.
TEST_F(ContGeofenceTest, ContingencyActionStaysWhileANewFenceLoads)
{
	ASSERT_TRUE(loadFence(inclusionSquare(300.f, FenceLevel::Contingency)));
	ASSERT_TRUE(storeFence({}));
	_fence.updateFence();
	_fence.run(); // start the update
	_fence.run(); // send the metadata read
	ASSERT_TRUE(GeofenceTestPeer::waitForPendingRead(_fence));
	_fence.run(); // adopt the new ID and invalidate the data cache
	EXPECT_TRUE(_fence.isFenceUpdatePending());
	EXPECT_TRUE(_fence.isEmpty());
	EXPECT_EQ(_fence.getContGeofenceAction(), geofence_result_s::GF_ACTION_LOITER);
	ASSERT_TRUE(waitForFence());
	EXPECT_EQ(_fence.getContGeofenceAction(), geofence_result_s::GF_ACTION_NONE);
}

// A contingency geofence never makes a plan invalid: an area that excludes Home or the flying vehicle still loads,
// and the vehicle is then outside it. The emergency geofence keeps both checks.
TEST_F(ContGeofenceTest, ContingencyAreaLoadsWithoutHomeOrTheVehicleInside)
{
	ASSERT_TRUE(loadFence(circle(false, {0.f, 0.f}, 100.f, FenceLevel::Contingency)));
	EXPECT_EQ(_fence.getContGeofenceAction(), geofence_result_s::GF_ACTION_LOITER);
	EXPECT_FALSE(insideCont(0.f, 0.f));

	const auto current = position(0.f, 400.f);
	_navigator.get_global_position()->lat = current(0);
	_navigator.get_global_position()->lon = current(1);
	_navigator.get_land_detected()->landed = false;
	ASSERT_TRUE(loadFence(inclusionSquare(300.f, FenceLevel::Contingency)));
	EXPECT_EQ(_fence.getContGeofenceAction(), geofence_result_s::GF_ACTION_LOITER);
	EXPECT_FALSE(insideCont(0.f, 400.f));

	ASSERT_TRUE(loadFence(inclusionSquare(300.f)));
	GeofenceInterface &interface = _fence;
	EXPECT_EQ(interface.getNumPolygons(), 0);
}

// Drawn contingency geofence areas replace GF_CONT_H_MARGIN for the polygons and circles of the plan.
TEST_F(ContGeofenceTest, DrawnAreasReplaceTheMargin)
{
	setMargin(100.f);
	ASSERT_TRUE(loadFence(inclusionSquare(500.f)));
	EXPECT_FLOAT_EQ(_fence.getContGeofencePlanMargin(), 100.f);
	EXPECT_TRUE(insideCont(0.f, 380.f));
	EXPECT_FALSE(insideCont(0.f, 420.f));

	FencePoints points = inclusionSquare(500.f);
	const FencePoints contingency = inclusionSquare(450.f, FenceLevel::Contingency);
	points.insert(points.end(), contingency.begin(), contingency.end());
	ASSERT_TRUE(loadFence(points));
	EXPECT_FLOAT_EQ(_fence.getContGeofencePlanMargin(), 0.f);
	EXPECT_TRUE(insideCont(0.f, 420.f));
	EXPECT_FALSE(insideCont(0.f, 470.f));

	// Back to the margin once the areas are gone.
	ASSERT_TRUE(loadFence(inclusionSquare(500.f)));
	EXPECT_FLOAT_EQ(_fence.getContGeofencePlanMargin(), 100.f);
	EXPECT_FALSE(insideCont(0.f, 420.f));
}
