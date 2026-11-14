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

// Loiter circles checked for the fixed-wing geofence hold center. Home is at the origin.
class GeofenceLoiterCircleTest : public navigator_test::GeofenceTestBase
{
protected:
	static constexpr float kRadius = 80.f;

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

	bool circleInside(float north, float east, float radius = kRadius)
	{
		const matrix::Vector2d center = position(north, east);
		return _fence.isLoiterCircleInside(center(0), center(1), radius);
	}
};

TEST_F(GeofenceLoiterCircleTest, InclusionPolygon)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	EXPECT_TRUE(circleInside(0.f, 0.f));
	EXPECT_TRUE(circleInside(0.f, 410.f));
	EXPECT_FALSE(circleInside(0.f, 430.f)); // crosses the eastern edge
	EXPECT_FALSE(circleInside(0.f, 505.f)); // center outside
}

TEST_F(GeofenceLoiterCircleTest, CheckRadiusAddsMargin)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	EXPECT_TRUE(circleInside(0.f, 410.f, kRadius));
	EXPECT_FALSE(circleInside(0.f, 410.f, 1.5f * kRadius));
}

TEST_F(GeofenceLoiterCircleTest, ExclusionPolygon)
{
	// Exclusion square spans 250 m to 350 m east.
	ASSERT_TRUE(loadFence(exclusionSquare()));
	EXPECT_TRUE(circleInside(0.f, 160.f));
	EXPECT_FALSE(circleInside(0.f, 200.f)); // touches the western edge
	EXPECT_FALSE(circleInside(0.f, 300.f)); // center inside the exclusion
}

TEST_F(GeofenceLoiterCircleTest, InclusionCircle)
{
	ASSERT_TRUE(loadFence(circle(true, {0.f, 0.f}, 500.f)));
	EXPECT_TRUE(circleInside(410.f, 0.f));
	EXPECT_FALSE(circleInside(430.f, 0.f));
}

TEST_F(GeofenceLoiterCircleTest, MaximumDistanceFromHome)
{
	const float maximum = 500.f;
	ASSERT_EQ(param_set(param_find("GF_MAX_HOR_DIST"), &maximum), PX4_OK);
	_navigator.updateParams();
	ASSERT_TRUE(loadFence(circle(true, {0.f, 0.f}, 2000.f)));
	EXPECT_TRUE(circleInside(410.f, 0.f));
	EXPECT_FALSE(circleInside(430.f, 0.f));
}

TEST_F(GeofenceLoiterCircleTest, IgnoresAltitudeBand)
{
	// The hold keeps the current altitude, so only the horizontal fences matter.
	ASSERT_TRUE(loadFence(inclusionSquare()));
	GeofenceTestPeer::setAltitudeBand(_fence, 450.f, 550.f);
	EXPECT_TRUE(circleInside(0.f, 0.f));
}

TEST_F(GeofenceLoiterCircleTest, RejectsInvalidInput)
{
	ASSERT_TRUE(loadFence(inclusionSquare()));
	EXPECT_FALSE(_fence.isLoiterCircleInside(NAN, NAN, kRadius)); // no center stored yet
	EXPECT_FALSE(circleInside(0.f, 0.f, 0.f));
}
