/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * Unit tests for SearchMin goldensection
 * make tests TESTFILTER=SearchMin
 */

#include <gtest/gtest.h>
#include <cmath>

#include "SearchMin.hpp"

using math::abs_t;
using math::goldensection;

static float quad_at_zero(float x)
{
	return x * x;
}

static float quad_at_three(float x)
{
	return (x - 3.f) * (x - 3.f);
}

TEST(SearchMin, abs_t)
{
	EXPECT_FLOAT_EQ(abs_t(3.5f), 3.5f);
	EXPECT_FLOAT_EQ(abs_t(-2.25f), 2.25f);
	EXPECT_FLOAT_EQ(abs_t(0.f), 0.f);
}

TEST(SearchMin, goldenSectionMinimizesX2)
{
	const float x = goldensection(-2.f, 2.f, quad_at_zero, 1e-4f);
	EXPECT_NEAR(x, 0.f, 1e-3f);
}

TEST(SearchMin, goldenSectionMinimizesShifted)
{
	const float x = goldensection(0.f, 5.f, quad_at_three, 1e-4f);
	EXPECT_NEAR(x, 3.f, 1e-3f);
}
