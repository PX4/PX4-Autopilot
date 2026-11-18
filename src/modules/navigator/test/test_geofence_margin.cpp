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

#include <cmath>

#include <parameters/param.h>

// Circles checked against the plan fences with Geofence::isCircleInside(). Home is at the origin.
class GeofenceCircleTest : public navigator_test::GeofenceTestBase
{
protected:
	using navigator_test::GeofenceTestBase::GeofenceTestBase;

	void SetUp() override
	{
		param_reset_all();
		_navigator.updateParams();
		ASSERT_TRUE(resetFence());
	}

	void TearDown() override { param_reset_all(); }

	void setParam(const char *name, float value)
	{
		ASSERT_EQ(param_set(param_find(name), &value), PX4_OK);
		_navigator.updateParams();
	}

	// A 1000 m square inclusion fence centred on Home.
	FencePoints inclusionSquare() const
	{
		return polygon(true, {{-500.f, -500.f}, {500.f, -500.f}, {500.f, 500.f}, {-500.f, 500.f}});
	}

	bool circleInside(float north, float east, float radius)
	{
		const matrix::Vector2d center = position(north, east);
		return _fence.isCircleInside(center(0), center(1), radius);
	}
};

struct CircleFenceCase {
	const char *name;
	bool circle_fence;
	bool inclusion;
};

class GeofenceCircleShapeTest : public GeofenceCircleTest, public ::testing::WithParamInterface<CircleFenceCase>
{
public:
	// Metre offsets are exact at the equator.
	GeofenceCircleShapeTest() : GeofenceCircleTest(0.0, 0.0) {}

protected:
	// Each fence has its nearest boundary 75 m from Home and keeps Home on its allowed side.
	FencePoints fence() const
	{
		const CircleFenceCase &test = GetParam();

		if (test.circle_fence) {
			return circle(test.inclusion, {0.f, test.inclusion ? -25.f : 100.f}, test.inclusion ? 100.f : 25.f);
		}

		if (test.inclusion) {
			return polygon(true, {{-75.f, -75.f}, {75.f, -75.f}, {75.f, 75.f}, {-75.f, 75.f}});
		}

		return polygon(false, {{-25.f, 75.f}, {25.f, 75.f}, {25.f, 125.f}, {-25.f, 125.f}});
	}
};

TEST_P(GeofenceCircleShapeTest, BoundaryContactIsNotInside)
{
	ASSERT_TRUE(loadFence(fence()));
	EXPECT_TRUE(circleInside(0.f, 0.f, 0.f));
	EXPECT_TRUE(circleInside(0.f, 0.f, 50.f));
	EXPECT_FALSE(circleInside(0.f, 0.f, 75.f)); // touches the boundary
	EXPECT_FALSE(circleInside(0.f, 0.f, 150.f)); // encloses the whole exclusion
	EXPECT_FALSE(circleInside(0.f, 0.f, INFINITY));
}

TEST_P(GeofenceCircleShapeTest, CenterMustBeOnTheAllowedSide)
{
	ASSERT_TRUE(loadFence(fence()));
	// 100 m east of Home is outside each inclusion and inside each exclusion,
	// at least 20 m from the boundary: the circle itself touches nothing.
	EXPECT_FALSE(circleInside(0.f, 100.f, 1.f));
}

INSTANTIATE_TEST_SUITE_P(Fences, GeofenceCircleShapeTest, ::testing::Values(
				 CircleFenceCase{"InclusionPolygon", false, true},
				 CircleFenceCase{"ExclusionPolygon", false, false},
				 CircleFenceCase{"InclusionCircle", true, true},
				 CircleFenceCase{"ExclusionCircle", true, false}),
			 [](const ::testing::TestParamInfo<CircleFenceCase> &test_info)
{
	return test_info.param.name;
});

TEST_F(GeofenceCircleTest, PolygonCornerCountsLikeAnEdge)
{
	// The exclusion square spans 250 m to 350 m east; its south-west corner is 70.7 m from the center.
	ASSERT_TRUE(loadFence(exclusionSquare()));
	EXPECT_TRUE(circleInside(-100.f, 200.f, 69.f));
	EXPECT_FALSE(circleInside(-100.f, 200.f, 72.f));
}

TEST_F(GeofenceCircleTest, EveryFenceApplies)
{
	FencePoints points = inclusionSquare();
	const FencePoints exclusion = exclusionSquare();
	points.insert(points.end(), exclusion.begin(), exclusion.end());
	ASSERT_TRUE(loadFence(points));
	EXPECT_TRUE(circleInside(0.f, 0.f, 80.f));
	EXPECT_FALSE(circleInside(0.f, 200.f, 80.f)); // touches the exclusion
	EXPECT_FALSE(circleInside(-450.f, 0.f, 80.f)); // touches the inclusion
}

// The Home and altitude limits have their own pre-geofence parameters.
TEST_F(GeofenceCircleTest, HomeAndAltitudeLimitsAreNotChecked)
{
	setParam("GF_MAX_HOR_DIST", 100.f);
	setParam("GF_MAX_VER_DIST", 10.f);
	ASSERT_TRUE(loadFence(inclusionSquare()));
	GeofenceTestPeer::setAltitudeBand(_fence, 450.f, 550.f);
	EXPECT_TRUE(circleInside(0.f, 300.f, 80.f));
}

TEST_F(GeofenceCircleTest, NoFenceClearsEveryCircle)
{
	// Like the point checks, and unlike the path checks, an unloaded fence restricts nothing.
	EXPECT_TRUE(circleInside(0.f, 0.f, 80.f));
	ASSERT_TRUE(loadFence({}));
	EXPECT_TRUE(circleInside(0.f, 0.f, 80.f));
	EXPECT_TRUE(circleInside(0.f, 0.f, INFINITY));
}

TEST_F(GeofenceCircleTest, InvalidInputIsNotInside)
{
	ASSERT_TRUE(loadFence({}));
	const matrix::Vector2d center = position(0.f, 0.f);
	EXPECT_FALSE(_fence.isCircleInside(static_cast<double>(NAN), center(1), 80.f));
	EXPECT_FALSE(_fence.isCircleInside(center(0), static_cast<double>(NAN), 80.f));
	EXPECT_FALSE(_fence.isCircleInside(center(0), center(1), -1.f));
	EXPECT_FALSE(_fence.isCircleInside(center(0), center(1), NAN));
}

TEST_F(GeofenceCircleTest, LoadedFenceAnswersWhileAnUpdateIsPending)
{
	ASSERT_TRUE(loadFence(exclusionSquare()));
	_fence.updateFence();
	// The path checks wait for the update; the circle check keeps using the loaded fence.
	const matrix::Vector2d center = position(0.f, 160.f);
	const Geofence::PathCheck path{center, center, 80.f};
	bool clear = true;
	EXPECT_FALSE(_fence.checkPathBatch(&path, 1, &clear));
	EXPECT_TRUE(circleInside(0.f, 160.f, 80.f));
	EXPECT_FALSE(circleInside(0.f, 200.f, 80.f));
}

TEST_F(GeofenceCircleTest, UnreadableFenceIsNotInside)
{
	for (const FencePoints &points : {exclusionSquare(), circle(false, {0.f, 300.f}, 50.f)}) {
		SCOPED_TRACE(points.size());
		ASSERT_TRUE(loadFence(points));
		ASSERT_TRUE(circleInside(0.f, 0.f, 80.f));
		GeofenceTestPeer::invalidateCache(_fence);
		EXPECT_FALSE(circleInside(0.f, 0.f, 80.f));
	}
}

// The check in flight and the mission check must agree: a circle is inside exactly when its centre passes the
// point check and the path checker clears it, both as a loiter circle and as the margin of a single position.
TEST_F(GeofenceCircleTest, MatchesPointAndPathChecks)
{
	// A concave inclusion with a notch, an exclusion square and an exclusion circle inside it.
	FencePoints points = polygon(true, {{-400.f, -400.f}, {-400.f, 600.f}, {400.f, 600.f}, {400.f, 100.f},
		{150.f, 100.f}, {150.f, -100.f}, {400.f, -100.f}, {400.f, -400.f}
	});
	const FencePoints square = exclusionSquare();
	const FencePoints disc = circle(false, {-200.f, -150.f}, 60.f);
	points.insert(points.end(), square.begin(), square.end());
	points.insert(points.end(), disc.begin(), disc.end());
	ASSERT_TRUE(loadFence(points));

	int inside = 0;
	int outside = 0;

	for (float north = -487.f; north < 500.f; north += 50.f) {
		for (float east = -487.f; east < 750.f; east += 50.f) {
			for (const float radius : {1.f, 20.f, 60.f, 150.f}) {
				SCOPED_TRACE(::testing::Message() << north << " m north, " << east << " m east, radius " << radius);
				const matrix::Vector2d center = position(north, east);
				const Geofence::PathCheck paths[] {{center, center, radius}, {center, center, 0.f, radius}};
				bool clear[2] {};
				ASSERT_TRUE(_fence.checkPathBatch(paths, 2, clear));
				const bool center_inside = _fence.isInsideHorizontalFences(center(0), center(1), 0.f);
				const bool expected = clear[0] && center_inside;
				EXPECT_EQ(clear[1] && center_inside, expected);
				EXPECT_EQ(_fence.isCircleInside(center(0), center(1), radius), expected);
				expected ? ++inside : ++outside;
			}
		}
	}

	// Both outcomes must be well represented.
	EXPECT_GT(inside, 100);
	EXPECT_GT(outside, 100);
}

// Paths that must keep a margin from every polygon and circle, as mission paths do with GF_PRE_MARGIN.
class GeofencePathMarginTest : public GeofenceCircleTest
{
protected:
	bool pathClear(const matrix::Vector2f &start, const matrix::Vector2f &end, float margin, float end_radius = 0.f)
	{
		const Geofence::PathCheck path{position(start(0), start(1)), position(end(0), end(1)), end_radius, margin};
		bool clear = false;
		EXPECT_TRUE(_fence.checkPathBatch(&path, 1, &clear));
		return clear;
	}
};

TEST_F(GeofencePathMarginTest, InclusionPolygon)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	EXPECT_TRUE(pathClear({0.f, 0.f}, {0.f, 400.f}, 80.f));
	EXPECT_FALSE(pathClear({0.f, 0.f}, {0.f, 430.f}, 80.f));
	EXPECT_FALSE(pathClear({0.f, 430.f}, {0.f, 0.f}, 80.f)); // the start keeps the margin too
	EXPECT_TRUE(pathClear({0.f, 0.f}, {0.f, 430.f}, 0.f));
}

TEST_F(GeofencePathMarginTest, VertexNearTheMiddleOfAPath)
{
	// The path passes 50 m west of the exclusion square, whose corners are far from the path ends.
	ASSERT_TRUE(loadFence(exclusionSquare()));
	EXPECT_TRUE(pathClear({-200.f, 200.f}, {200.f, 200.f}, 40.f));
	EXPECT_FALSE(pathClear({-200.f, 200.f}, {200.f, 200.f}, 60.f));
}

TEST_F(GeofencePathMarginTest, PathEndNearTheMiddleOfAnEdge)
{
	// The path ends 50 m south of the long northern edge, far from its corners.
	ASSERT_TRUE(loadFence(inclusionSquare()));
	EXPECT_TRUE(pathClear({0.f, 0.f}, {450.f, 0.f}, 40.f));
	EXPECT_FALSE(pathClear({0.f, 0.f}, {450.f, 0.f}, 60.f));
}

TEST_F(GeofencePathMarginTest, InclusionCircle)
{
	ASSERT_TRUE(loadFence(circle(true, {0.f, 0.f}, 500.f)));
	EXPECT_TRUE(pathClear({0.f, 0.f}, {0.f, 400.f}, 80.f));
	EXPECT_FALSE(pathClear({0.f, 0.f}, {0.f, 400.f}, 120.f));
	EXPECT_FALSE(pathClear({0.f, 400.f}, {0.f, 0.f}, 120.f));
}

TEST_F(GeofencePathMarginTest, ExclusionCircle)
{
	// The path passes 100 m from the centre of a 50 m exclusion circle.
	ASSERT_TRUE(loadFence(circle(false, {0.f, 300.f}, 50.f)));
	EXPECT_TRUE(pathClear({-200.f, 200.f}, {200.f, 200.f}, 40.f));
	EXPECT_FALSE(pathClear({-200.f, 200.f}, {200.f, 200.f}, 60.f));
}

TEST_F(GeofencePathMarginTest, LoiterCircleAddsTheMargin)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	EXPECT_TRUE(pathClear({0.f, 0.f}, {0.f, 300.f}, 80.f, 80.f));
	EXPECT_FALSE(pathClear({0.f, 0.f}, {0.f, 360.f}, 80.f, 80.f));
	EXPECT_TRUE(pathClear({0.f, 0.f}, {0.f, 360.f}, 0.f, 80.f));
}

// The distance from Home has its own pre-geofence limit; only a loiter circle is checked against it.
TEST_F(GeofencePathMarginTest, MarginDoesNotApplyToTheHomeDistance)
{
	setParam("GF_MAX_HOR_DIST", 500.f);
	ASSERT_TRUE(loadFence({}));
	EXPECT_TRUE(pathClear({0.f, 0.f}, {480.f, 0.f}, 80.f));
	EXPECT_TRUE(pathClear({0.f, 0.f}, {400.f, 0.f}, 80.f, 80.f));
	EXPECT_FALSE(pathClear({0.f, 0.f}, {440.f, 0.f}, 80.f, 80.f));
}

TEST_F(GeofencePathMarginTest, InvalidMarginIsUnavailable)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	const matrix::Vector2d center = position(0.f, 0.f);

	for (const float margin : {-1.f, NAN}) {
		const Geofence::PathCheck path{center, center, 0.f, margin};
		bool clear = true;
		EXPECT_FALSE(_fence.checkPathBatch(&path, 1, &clear));
		EXPECT_FALSE(clear);
	}
}

// A long east-west leg is flown along the great circle, which bows towards the pole: 53 m over 50 km at 47 deg N.
// The margin has to be kept along the flown path, not along the straight line between the two ends.
class GeofenceLongLegMarginTest : public GeofencePathMarginTest
{
protected:
	static constexpr double kLat = 47.0;
	static constexpr double kLon = 8.0;
	static constexpr double kHalfLegDeg = 0.32965; // 25 km of longitude at 47 deg N

	// An inclusion rectangle in latitude and longitude around the leg.
	::testing::AssertionResult loadRectangle(double north_lat, double east_lon)
	{
		FencePoints points;

		for (const matrix::Vector2d &vertex : {
			     matrix::Vector2d{kLat - 0.1, kLon - 0.5}, matrix::Vector2d{north_lat, kLon - 0.5},
			     matrix::Vector2d{north_lat, east_lon}, matrix::Vector2d{kLat - 0.1, east_lon}
		     }) {
			mission_fence_point_s point{};
			point.nav_cmd = NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION;
			point.frame = NAV_FRAME_GLOBAL;
			point.lat = vertex(0);
			point.lon = vertex(1);
			point.vertex_count = 4;
			points.push_back(point);
		}

		return loadFence(points);
	}

	bool legClear(float margin)
	{
		const Geofence::PathCheck path{{kLat, kLon - kHalfLegDeg}, {kLat, kLon + kHalfLegDeg}, 0.f, margin};
		bool clear = false;
		EXPECT_TRUE(_fence.checkPathBatch(&path, 1, &clear));
		return clear;
	}
};

TEST_F(GeofenceLongLegMarginTest, MarginIsKeptAlongTheGreatCircle)
{
	// The northern edge is 230 m north of both ends of the leg.
	ASSERT_TRUE(loadRectangle(kLat + math::degrees(230.0 / CONSTANTS_RADIUS_OF_EARTH), kLon + 0.5));
	EXPECT_TRUE(legClear(0.f));
	EXPECT_TRUE(legClear(150.f));
	// Only about 177 m are left in the middle of the leg.
	EXPECT_FALSE(legClear(200.f));
}

TEST_F(GeofenceLongLegMarginTest, EdgeBesideTheLegEnd)
{
	// The eastern edge is 150 m beyond the end of the leg, and the leg never reaches its longitude.
	const double east_lon = kLon + kHalfLegDeg
				+ math::degrees(150.0 / (CONSTANTS_RADIUS_OF_EARTH * cos(math::radians(kLat))));
	ASSERT_TRUE(loadRectangle(kLat + 0.1, east_lon));
	EXPECT_TRUE(legClear(100.f));
	EXPECT_FALSE(legClear(200.f));
}
