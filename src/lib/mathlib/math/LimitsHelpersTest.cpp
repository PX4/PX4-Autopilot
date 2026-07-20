/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * make tests TESTFILTER=LimitsHelpers
 * (constrainFloatToInt16 covered separately)
 */

#include <gtest/gtest.h>

#include "Limits.hpp"

using math::constrain;
using math::isInRange;
using math::max;
using math::min;

TEST(LimitsHelpers, minMaxTwoAndThreeArg)
{
	EXPECT_EQ(min(3, 1), 1);
	EXPECT_EQ(max(3, 1), 3);
	EXPECT_EQ(min(3, 1, 2), 1);
	EXPECT_EQ(max(3, 1, 2), 3);
	EXPECT_FLOAT_EQ(min(1.5f, -0.5f, 0.f), -0.5f);
	EXPECT_FLOAT_EQ(max(1.5f, -0.5f, 0.f), 1.5f);
}

TEST(LimitsHelpers, constrainClips)
{
	EXPECT_EQ(constrain(5, 0, 3), 3);
	EXPECT_EQ(constrain(-2, 0, 3), 0);
	EXPECT_EQ(constrain(2, 0, 3), 2);
}

TEST(LimitsHelpers, isInRangeInclusive)
{
	EXPECT_TRUE(isInRange(0, 0, 10));
	EXPECT_TRUE(isInRange(10, 0, 10));
	EXPECT_TRUE(isInRange(5, 0, 10));
	EXPECT_FALSE(isInRange(-1, 0, 10));
	EXPECT_FALSE(isInRange(11, 0, 10));
}
