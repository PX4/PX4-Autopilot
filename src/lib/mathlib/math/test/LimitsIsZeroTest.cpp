/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file LimitsIsZeroTest.cpp
 * Unit coverage for math::isZero (float/double near-zero helpers).
 */

#include <gtest/gtest.h>
#include <float.h>
#include <mathlib/math/Limits.hpp>

using namespace math;

TEST(LimitsIsZeroTest, FloatExactAndNearZero)
{
	EXPECT_TRUE(isZero(0.0f));
	EXPECT_TRUE(isZero(-0.0f));
	EXPECT_TRUE(isZero(FLT_EPSILON * 0.5f));
	EXPECT_TRUE(isZero(-FLT_EPSILON * 0.5f));
	EXPECT_FALSE(isZero(FLT_EPSILON * 2.f));
	EXPECT_FALSE(isZero(-FLT_EPSILON * 2.f));
	EXPECT_FALSE(isZero(1.0f));
	EXPECT_FALSE(isZero(-1.0f));
}

TEST(LimitsIsZeroTest, DoubleExactAndNearZero)
{
	EXPECT_TRUE(isZero(0.0));
	EXPECT_TRUE(isZero(-0.0));
	EXPECT_TRUE(isZero(DBL_EPSILON * 0.5));
	EXPECT_TRUE(isZero(-DBL_EPSILON * 0.5));
	EXPECT_FALSE(isZero(DBL_EPSILON * 2.0));
	EXPECT_FALSE(isZero(-DBL_EPSILON * 2.0));
	EXPECT_FALSE(isZero(1.0));
}

TEST(LimitsIsZeroTest, FloatNotNanInfAsZero)
{
	// NaN and Inf are not near zero; fabs comparison does not treat them as zero.
	EXPECT_FALSE(isZero(NAN));
	EXPECT_FALSE(isZero(INFINITY));
	EXPECT_FALSE(isZero(-INFINITY));
}
