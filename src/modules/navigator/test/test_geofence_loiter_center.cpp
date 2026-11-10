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

// Loiter centers chosen after a fixed-wing breaches a horizontal fence. Home is at the origin.
class GeofenceLoiterCenterTest : public navigator_test::GeofenceTestBase
{
protected:
	static constexpr float kRadius = 80.f;
	static constexpr float kOffset = Geofence::kLoiterCenterOffsetRadii * kRadius;
	static constexpr float kTolerance = 1.f; // [m]

	void SetUp() override
	{
		ASSERT_TRUE(resetFence());
	}

	void TearDown() override
	{
		param_reset(param_find("GF_MAX_HOR_DIST"));
		_navigator.updateParams();
	}

	// A 1000 m square inclusion fence centred on Home.
	FencePoints inclusionSquare() const
	{
		return polygon(true, {{-500.f, -500.f}, {500.f, -500.f}, {500.f, 500.f}, {-500.f, 500.f}});
	}

	// Course in degrees from north, as flown when the breach was detected.
	::testing::AssertionResult centerFor(const matrix::Vector2f &vehicle, float course_deg, matrix::Vector2d &center)
	{
		const matrix::Vector2d at = position(vehicle(0), vehicle(1));
		center = matrix::Vector2d{NAN, NAN};

		if (!_fence.findLoiterCenterInside(at(0), at(1), math::radians(course_deg), kRadius, center(0), center(1))) {
			return ::testing::AssertionFailure() << "no clear loiter center";
		}

		return ::testing::AssertionSuccess();
	}

	::testing::AssertionResult near(const matrix::Vector2d &center, const matrix::Vector2f &expected) const
	{
		const matrix::Vector2d target = position(expected(0), expected(1));
		const float error = get_distance_to_next_waypoint(center(0), center(1), target(0), target(1));

		if (error > kTolerance) {
			return ::testing::AssertionFailure() << "center is " << error << " m from the expected point";
		}

		return ::testing::AssertionSuccess();
	}
};

TEST_F(GeofenceLoiterCenterTest, MovesPerpendicularIntoInclusionPolygon)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	matrix::Vector2d center;
	ASSERT_TRUE(centerFor({0.f, 505.f}, 90.f, center));
	EXPECT_TRUE(near(center, {0.f, 505.f - kOffset}));
}

TEST_F(GeofenceLoiterCenterTest, ShallowCrossingStillMovesPerpendicular)
{
	// Going back along a 20 degree crossing would leave the circle touching the eastern edge.
	ASSERT_TRUE(loadFence(inclusionSquare()));
	matrix::Vector2d center;
	ASSERT_TRUE(centerFor({0.f, 505.f}, 20.f, center));
	EXPECT_TRUE(near(center, {0.f, 505.f - kOffset}));
}

TEST_F(GeofenceLoiterCenterTest, LeavesExclusionPolygonTheWayItCame)
{
	// Entered the 100 m exclusion square from the west; past its eastern edge would also be clear.
	ASSERT_TRUE(loadFence(exclusionSquare()));
	matrix::Vector2d center;
	ASSERT_TRUE(centerFor({0.f, 255.f}, 90.f, center));
	EXPECT_TRUE(near(center, {0.f, 255.f - kOffset}));
}

TEST_F(GeofenceLoiterCenterTest, MovesTowardsCenterOfInclusionCircle)
{
	ASSERT_TRUE(loadFence(circle(true, {0.f, 0.f}, 500.f)));
	matrix::Vector2d center;
	ASSERT_TRUE(centerFor({0.f, 505.f}, 0.f, center));
	EXPECT_TRUE(near(center, {0.f, 505.f - kOffset}));
}

TEST_F(GeofenceLoiterCenterTest, MovesTowardsHomeAfterMaxDistanceBreach)
{
	const float maximum = 500.f;
	ASSERT_EQ(param_set(param_find("GF_MAX_HOR_DIST"), &maximum), PX4_OK);
	_navigator.updateParams();
	ASSERT_TRUE(loadFence(circle(true, {0.f, 0.f}, 2000.f)));
	matrix::Vector2d center;
	ASSERT_TRUE(centerFor({505.f, 0.f}, 90.f, center));
	EXPECT_TRUE(near(center, {505.f - kOffset, 0.f}));
}

TEST_F(GeofenceLoiterCenterTest, AltitudeBandBreachKeepsCenter)
{
	// Horizontally inside, 100 m from the eastern edge: a clear center 2R west would exist.
	ASSERT_TRUE(loadFence(inclusionSquare()));
	GeofenceTestPeer::setAltitudeBand(_fence, 450.f, 550.f);
	matrix::Vector2d center;
	EXPECT_FALSE(centerFor({0.f, 400.f}, 90.f, center));
	EXPECT_FALSE(PX4_ISFINITE(center(0)));
}

TEST_F(GeofenceLoiterCenterTest, FailsWhenNoCircleFits)
{
	// A 100 m square cannot hold a circle of 80 m radius anywhere.
	ASSERT_TRUE(loadFence(polygon(true, {{-50.f, -50.f}, {50.f, -50.f}, {50.f, 50.f}, {-50.f, 50.f}})));
	matrix::Vector2d center;
	EXPECT_FALSE(centerFor({0.f, 55.f}, 90.f, center));
	EXPECT_FALSE(PX4_ISFINITE(center(0)));
}
