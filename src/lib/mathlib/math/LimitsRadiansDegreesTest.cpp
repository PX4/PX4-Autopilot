/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <cmath>
#include <gtest/gtest.h>
#include <mathlib/math/Limits.hpp>

using math::degrees;
using math::radians;

TEST(LimitsRadiansDegrees, IdentityPairs)
{
	EXPECT_NEAR(degrees(radians(0.f)), 0.f, 1e-5f);
	EXPECT_NEAR(degrees(radians(90.f)), 90.f, 1e-4f);
	EXPECT_NEAR(degrees(radians(-45.f)), -45.f, 1e-4f);
	EXPECT_NEAR(degrees(radians(180.f)), 180.f, 1e-3f);
}

TEST(LimitsRadiansDegrees, KnownValues)
{
	EXPECT_NEAR(radians(180.f), static_cast<float>(M_PI), 1e-5f);
	EXPECT_NEAR(degrees(static_cast<float>(M_PI_2)), 90.f, 1e-4f);
}

TEST(LimitsRadiansDegrees, Double)
{
	EXPECT_NEAR(degrees(radians(30.0)), 30.0, 1e-9);
}
