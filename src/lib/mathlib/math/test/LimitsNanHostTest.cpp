/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * Host-side regression for Limits min/max/constrain with NaN inputs.
 * Complements QURT-specific fixes (see PR discussion on Hexagon sfmin/sfmax).
 */

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <mathlib/math/Limits.hpp>

using namespace math;

TEST(LimitsNanHostTest, ConstrainPreservesNan)
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	// On non-QURT, constrain uses comparisons; NaN comparisons are false so result stays NaN
	// when val is NaN under the (val < min) ? min : ((val > max) ? max : val) chain.
	const float out = constrain(nan, -1.f, 1.f);
	EXPECT_TRUE(std::isnan(out));
}

TEST(LimitsNanHostTest, MinMaxFinitePreferredPaths)
{
	EXPECT_FLOAT_EQ(min(1.f, 2.f), 1.f);
	EXPECT_FLOAT_EQ(max(1.f, 2.f), 2.f);
	EXPECT_FLOAT_EQ(constrain(0.5f, -1.f, 1.f), 0.5f);
	EXPECT_FLOAT_EQ(constrain(-5.f, -1.f, 1.f), -1.f);
	EXPECT_FLOAT_EQ(constrain(5.f, -1.f, 1.f), 1.f);
}
