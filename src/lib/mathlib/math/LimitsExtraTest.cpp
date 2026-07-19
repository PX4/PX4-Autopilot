/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <mathlib/math/Limits.hpp>

using namespace math;

TEST(LimitsExtraTest, minMaxThreeArg)
{
	EXPECT_FLOAT_EQ(min(3.f, 1.f, 2.f), 1.f);
	EXPECT_FLOAT_EQ(min(-1.f, -5.f, 0.f), -5.f);
	EXPECT_FLOAT_EQ(max(3.f, 1.f, 2.f), 3.f);
	EXPECT_FLOAT_EQ(max(-1.f, -5.f, 0.f), 0.f);

	EXPECT_EQ(min(3, 1, 2), 1);
	EXPECT_EQ(max(3, 1, 2), 3);
	EXPECT_EQ(min(7, 7, 7), 7);
	EXPECT_EQ(max(7, 7, 7), 7);
}
