/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * Unit tests for TrajMath speed / braking helpers
 * make tests TESTFILTER=TrajMathSpeed
 */

#include <gtest/gtest.h>
#include <cmath>

#include <matrix/matrix/math.hpp>
#include "TrajMath.hpp"

using math::trajectory::computeBrakingDistanceFromVelocity;
using math::trajectory::computeMaxSpeedFromDistance;
using math::trajectory::computeMaxSpeedInWaypoint;

TEST(TrajMathSpeed, maxSpeedFromDistanceAtLeastFinalSpeed)
{
	const float jerk = 10.f;
	const float accel = 2.f;
	const float final_speed = 3.f;
	const float braking_distance = 5.f;
	const float max_speed = computeMaxSpeedFromDistance(jerk, accel, braking_distance, final_speed);
	EXPECT_GE(max_speed, final_speed);
	EXPECT_TRUE(std::isfinite(max_speed));
}

TEST(TrajMathSpeed, maxSpeedFromDistanceGrowsWithDistance)
{
	const float jerk = 8.f;
	const float accel = 1.5f;
	const float final_speed = 1.f;
	const float near = computeMaxSpeedFromDistance(jerk, accel, 1.f, final_speed);
	const float far = computeMaxSpeedFromDistance(jerk, accel, 20.f, final_speed);
	EXPECT_GT(far, near);
}

TEST(TrajMathSpeed, maxSpeedInWaypointPositive)
{
	const float v = computeMaxSpeedInWaypoint(1.0f, 2.f, 4.f);
	EXPECT_GT(v, 0.f);
	EXPECT_TRUE(std::isfinite(v));
}

TEST(TrajMathSpeed, maxSpeedInWaypointSmallAlphaSmallSpeed)
{
	const float tiny = computeMaxSpeedInWaypoint(0.01f, 2.f, 4.f);
	const float wider = computeMaxSpeedInWaypoint(1.2f, 2.f, 4.f);
	EXPECT_LT(tiny, wider);
}

TEST(TrajMathSpeed, brakingDistanceZeroVelocity)
{
	EXPECT_FLOAT_EQ(computeBrakingDistanceFromVelocity(0.f, 5.f, 2.f, 2.f), 0.f);
}

TEST(TrajMathSpeed, brakingDistanceScalesWithVelocity)
{
	const float d1 = computeBrakingDistanceFromVelocity(2.f, 5.f, 2.f, 2.f);
	const float d2 = computeBrakingDistanceFromVelocity(4.f, 5.f, 2.f, 2.f);
	EXPECT_GT(d2, d1);
	EXPECT_GT(d1, 0.f);
}
