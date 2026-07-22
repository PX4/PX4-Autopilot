/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * Unit tests for matrix::wrap_2pi
 * make tests TESTFILTER=MatrixWrap2pi
 */

#include <gtest/gtest.h>
#include <matrix/math.hpp>
#include <cmath>

using matrix::wrap_2pi;

TEST(MatrixWrap2piTest, InRangeUnchanged)
{
	EXPECT_NEAR(wrap_2pi(0.f), 0.f, 1e-6f);
	EXPECT_NEAR(wrap_2pi(1.f), 1.f, 1e-6f);
	EXPECT_NEAR(wrap_2pi(static_cast<float>(M_PI)), static_cast<float>(M_PI), 1e-5f);
}

TEST(MatrixWrap2piTest, NegativeWrapsPositive)
{
	const float twopi = static_cast<float>(2.0 * M_PI);
	EXPECT_NEAR(wrap_2pi(-0.5f), twopi - 0.5f, 1e-5f);
	EXPECT_NEAR(wrap_2pi(-static_cast<float>(M_PI)), static_cast<float>(M_PI), 1e-5f);
}

TEST(MatrixWrap2piTest, MultiTurn)
{
	const float twopi = static_cast<float>(2.0 * M_PI);
	EXPECT_NEAR(wrap_2pi(twopi + 0.25f), 0.25f, 1e-4f);
	EXPECT_NEAR(wrap_2pi(-twopi - 0.25f), twopi - 0.25f, 1e-4f);
	EXPECT_NEAR(wrap_2pi(3.f * twopi + 1.1f), 1.1f, 1e-3f);
}

TEST(MatrixWrap2piTest, Double)
{
	EXPECT_NEAR(wrap_2pi(0.0), 0.0, 1e-12);
	EXPECT_NEAR(wrap_2pi(-0.25), 2.0 * M_PI - 0.25, 1e-9);
}
