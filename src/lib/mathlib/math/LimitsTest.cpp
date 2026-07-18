/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <mathlib/math/Limits.hpp>
#include <cstdint>
#include <cmath>

using namespace math;

TEST(LimitsTest, ConstrainFloatToInt16ClipsExtremes)
{
	EXPECT_EQ(constrainFloatToInt16(0.f), 0);
	EXPECT_EQ(constrainFloatToInt16(32767.f), INT16_MAX);
	EXPECT_EQ(constrainFloatToInt16(32768.f), INT16_MAX);
	EXPECT_EQ(constrainFloatToInt16(1.0e9f), INT16_MAX);
	EXPECT_EQ(constrainFloatToInt16(-32768.f), INT16_MIN);
	EXPECT_EQ(constrainFloatToInt16(-32769.f), INT16_MIN);
	EXPECT_EQ(constrainFloatToInt16(-1.0e9f), INT16_MIN);
}

TEST(LimitsTest, ConstrainFloatPreservesInRange)
{
	EXPECT_EQ(constrainFloatToInt16(42.4f), 42);
	EXPECT_EQ(constrainFloatToInt16(-99.9f), -99);
	EXPECT_EQ(constrainFloatToInt16(32767.0f), INT16_MAX);
}

TEST(LimitsTest, ConstrainGenericAndIsInRange)
{
	EXPECT_FLOAT_EQ(constrain(5.f, 0.f, 1.f), 1.f);
	EXPECT_FLOAT_EQ(constrain(-1.f, 0.f, 1.f), 0.f);
	EXPECT_FLOAT_EQ(constrain(0.5f, 0.f, 1.f), 0.5f);
	EXPECT_TRUE(isInRange(0.5f, 0.f, 1.f));
	EXPECT_FALSE(isInRange(1.5f, 0.f, 1.f));
}
