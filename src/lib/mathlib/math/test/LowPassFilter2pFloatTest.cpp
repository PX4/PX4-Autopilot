/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * Unit tests for LowPassFilter2p float disable / cutoff edges.
 * make tests TESTFILTER=LowPassFilter2pFloat
 */

#include <gtest/gtest.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

using math::LowPassFilter2p;

TEST(LowPassFilter2pFloat, ConstructValidCutoff)
{
	LowPassFilter2p<float> lpf(1000.f, 30.f);
	EXPECT_FLOAT_EQ(lpf.get_sample_freq(), 1000.f);
	EXPECT_NEAR(lpf.get_cutoff_freq(), 30.f, 1e-4f);
}

TEST(LowPassFilter2pFloat, InvalidCutoffDisables)
{
	LowPassFilter2p<float> lpf;
	lpf.set_cutoff_frequency(1000.f, -1.f);
	EXPECT_FLOAT_EQ(lpf.get_cutoff_freq(), 0.f);
	EXPECT_FLOAT_EQ(lpf.get_sample_freq(), 0.f);
	// disabled: apply is identity (b0=1)
	EXPECT_FLOAT_EQ(lpf.apply(1.25f), 1.25f);
}

TEST(LowPassFilter2pFloat, NyquistRejectDisables)
{
	LowPassFilter2p<float> lpf(200.f, 120.f); // cutoff >= sample/2
	EXPECT_FLOAT_EQ(lpf.get_cutoff_freq(), 0.f);
}

TEST(LowPassFilter2pFloat, ResetFinite)
{
	LowPassFilter2p<float> lpf(800.f, 40.f);
	const float y = lpf.reset(3.f);
	EXPECT_TRUE(std::isfinite(y));
}

TEST(LowPassFilter2pFloat, DisableIdentity)
{
	LowPassFilter2p<float> lpf(500.f, 20.f);
	lpf.disable();
	EXPECT_FLOAT_EQ(lpf.apply(9.f), 9.f);
	EXPECT_FLOAT_EQ(lpf.apply(-2.f), -2.f);
}
