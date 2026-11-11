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

// Circles checked against the horizontal fences with Geofence::isCircleInside(). Home is at the origin.
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

	void setMaximumDistanceFromHome(float maximum)
	{
		ASSERT_EQ(param_set(param_find("GF_MAX_HOR_DIST"), &maximum), PX4_OK);
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

TEST_F(GeofenceCircleTest, MaximumDistanceFromHome)
{
	setMaximumDistanceFromHome(500.f);
	ASSERT_TRUE(loadFence({}));
	EXPECT_TRUE(circleInside(410.f, 0.f, 80.f));
	EXPECT_FALSE(circleInside(430.f, 0.f, 80.f)); // reaches past the limit
	EXPECT_FALSE(circleInside(510.f, 0.f, 0.f)); // center past the limit
}

TEST_F(GeofenceCircleTest, MaximumDistanceNeedsValidHome)
{
	setMaximumDistanceFromHome(500.f);
	ASSERT_TRUE(loadFence({}));
	_navigator.get_home_position()->valid_hpos = false;
	EXPECT_TRUE(circleInside(600.f, 0.f, 80.f));
}

TEST_F(GeofenceCircleTest, AltitudeLimitsAreNotChecked)
{
	const float maximum_altitude = 10.f;
	ASSERT_EQ(param_set(param_find("GF_MAX_VER_DIST"), &maximum_altitude), PX4_OK);
	_navigator.updateParams();
	ASSERT_TRUE(loadFence(inclusionSquare()));
	GeofenceTestPeer::setAltitudeBand(_fence, 450.f, 550.f);
	EXPECT_TRUE(circleInside(0.f, 0.f, 80.f));
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
	EXPECT_FALSE(_fence.isCircleInside(NAN, center(1), 80.f));
	EXPECT_FALSE(_fence.isCircleInside(center(0), NAN, 80.f));
	EXPECT_FALSE(_fence.isCircleInside(91.0, center(1), 80.f));
	EXPECT_FALSE(_fence.isCircleInside(center(0), 181.0, 80.f));
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

// The circle check must agree with a point check of the center plus the path check of the circle alone.
TEST_F(GeofenceCircleTest, MatchesPointAndPathChecks)
{
	setMaximumDistanceFromHome(700.f);
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
				const Geofence::PathCheck path{center, center, radius};
				bool path_clear = false;
				ASSERT_TRUE(_fence.checkPathBatch(&path, 1, &path_clear));
				const bool expected = path_clear && _fence.isInsideHorizontalFences(center(0), center(1), 0.f)
						      && _fence.isCloserThanMaxDistToHome(center(0), center(1), 0.f);
				EXPECT_EQ(_fence.isCircleInside(center(0), center(1), radius), expected);
				expected ? ++inside : ++outside;
			}
		}
	}

	// Both outcomes must be well represented.
	EXPECT_GT(inside, 100);
	EXPECT_GT(outside, 100);
}
