/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <limits>
#include <matrix/matrix/math.hpp>

#include "Functions.hpp"

using math::isFinite;
using matrix::Vector2f;
using matrix::Vector3f;

TEST(FunctionsIsFinite, floatScalar)
{
	EXPECT_TRUE(isFinite(0.f));
	EXPECT_TRUE(isFinite(1.25f));
	EXPECT_FALSE(isFinite(std::numeric_limits<float>::quiet_NaN()));
	EXPECT_FALSE(isFinite(std::numeric_limits<float>::infinity()));
	EXPECT_FALSE(isFinite(-std::numeric_limits<float>::infinity()));
}

TEST(FunctionsIsFinite, vector2)
{
	EXPECT_TRUE(isFinite(Vector2f(1.f, -2.f)));
	EXPECT_FALSE(isFinite(Vector2f(std::numeric_limits<float>::quiet_NaN(), 0.f)));
	EXPECT_FALSE(isFinite(Vector2f(0.f, std::numeric_limits<float>::infinity())));
}

TEST(FunctionsIsFinite, vector3Complement)
{
	// Baseline path already exists; ensure NaN-in-Z is rejected
	EXPECT_FALSE(isFinite(Vector3f(0.f, 0.f, std::numeric_limits<float>::quiet_NaN())));
	EXPECT_TRUE(isFinite(Vector3f(0.f, 1.f, -1.f)));
}
