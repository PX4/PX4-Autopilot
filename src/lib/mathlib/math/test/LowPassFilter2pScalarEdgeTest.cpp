/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * make tests TESTFILTER=LowPassFilter2pScalarEdge
 */

#include <gtest/gtest.h>
#include <cmath>

#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

using math::LowPassFilter2p;

TEST(LowPassFilter2pScalarEdge, invalidCutoffDisables)
{
	LowPassFilter2p<float> lpf(100.f, 10.f);
	// Nyquist violation / non-positive should disable
	lpf.set_cutoff_frequency(100.f, 0.f);
	// apply should not blow up
	float y = lpf.apply(1.5f);
	EXPECT_TRUE(std::isfinite(y));
	lpf.set_cutoff_frequency(-1.f, 10.f);
	y = lpf.apply(2.f);
	EXPECT_TRUE(std::isfinite(y));
	lpf.set_cutoff_frequency(100.f, 80.f); // >= fs/2
	y = lpf.apply(3.f);
	EXPECT_TRUE(std::isfinite(y));
}

TEST(LowPassFilter2pScalarEdge, validCutoffTracksStepEventually)
{
	LowPassFilter2p<float> lpf(200.f, 20.f);
	float y = 0.f;

	for (int i = 0; i < 400; i++) {
		y = lpf.apply(1.f);
	}

	EXPECT_NEAR(y, 1.f, 0.05f);
}

TEST(LowPassFilter2pScalarEdge, notFiniteSampleFreqDisables)
{
	LowPassFilter2p<float> lpf;
	lpf.set_cutoff_frequency(NAN, 10.f);
	EXPECT_TRUE(std::isfinite(lpf.apply(1.f)));
}
