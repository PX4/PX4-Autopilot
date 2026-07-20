/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * make tests TESTFILTER=LowPassFilter2pReset
 */

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

using math::LowPassFilter2p;

TEST(LowPassFilter2pReset, resetSeedsStateNearSample)
{
	LowPassFilter2p<float> lpf(200.f, 25.f);

	// drive away from zero
	for (int i = 0; i < 50; i++) {
		(void)lpf.apply(-5.f);
	}

	const float y = lpf.reset(2.f);
	EXPECT_TRUE(std::isfinite(y));
	// subsequent sample near reset value stays finite / near path
	const float y2 = lpf.apply(2.f);
	EXPECT_NEAR(y2, 2.f, 0.75f);
}

TEST(LowPassFilter2pReset, resetNonFiniteUsesZeroish)
{
	LowPassFilter2p<float> lpf(200.f, 25.f);
	const float y = lpf.reset(std::numeric_limits<float>::quiet_NaN());
	EXPECT_TRUE(std::isfinite(y));
}

TEST(LowPassFilter2pReset, getCutoffReflectsConfig)
{
	LowPassFilter2p<float> lpf(200.f, 25.f);
	EXPECT_NEAR(lpf.get_cutoff_freq(), 25.f, 1e-3f);
	EXPECT_NEAR(lpf.get_sample_freq(), 200.f, 1e-3f);
}
