/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include <matrix/math.hpp>

using matrix::isEqualF;

TEST(MatrixIsEqualF, basicTolerance)
{
	EXPECT_TRUE(isEqualF(1.f, 1.f));
	EXPECT_TRUE(isEqualF(1.f, 1.f + 1e-5f, 1e-4f));
	EXPECT_FALSE(isEqualF(1.f, 1.1f, 1e-3f));
}

TEST(MatrixIsEqualF, nanEqualsNan)
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	EXPECT_TRUE(isEqualF(nan, nan));
	EXPECT_TRUE(isEqualF(nan, -nan));
	EXPECT_FALSE(isEqualF(nan, 0.f));
}

TEST(MatrixIsEqualF, infinitySigns)
{
	const float pinf = std::numeric_limits<float>::infinity();
	const float ninf = -std::numeric_limits<float>::infinity();
	EXPECT_TRUE(isEqualF(pinf, pinf));
	EXPECT_TRUE(isEqualF(ninf, ninf));
	EXPECT_FALSE(isEqualF(pinf, ninf));
	EXPECT_FALSE(isEqualF(pinf, 1e6f));
}
