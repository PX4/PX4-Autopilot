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

static double quad_at_zero(double x)
{
	return x * x;
}

static double quad_at_three(double x)
{
	return (x - 3.0) * (x - 3.0);
}

TEST(SearchMin, abs_t)
{
	EXPECT_FLOAT_EQ(abs_t(3.5f), 3.5f);
	EXPECT_FLOAT_EQ(abs_t(-2.25f), 2.25f);
	EXPECT_FLOAT_EQ(abs_t(0.f), 0.f);
}

TEST(SearchMin, goldenSectionMinimizesX2)
{
	const double x = goldensection(-2.0, 2.0, quad_at_zero, 1e-4);
	EXPECT_NEAR(x, 0.0, 1e-3);
}

TEST(SearchMin, goldenSectionMinimizesShifted)
{
	const double x = goldensection(0.0, 5.0, quad_at_three, 1e-4);
	EXPECT_NEAR(x, 3.0, 1e-3);
}
