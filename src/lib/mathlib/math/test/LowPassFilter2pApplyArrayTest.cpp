/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * Unit tests for LowPassFilter2p::applyArray
 */

#include <gtest/gtest.h>
#include <cmath>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

using math::LowPassFilter2p;

TEST(LowPassFilter2pApplyArray, ConstantSequenceStable)
{
	LowPassFilter2p<float> lpf(1000.f, 50.f);
	float samples[8];
	for (int i = 0; i < 8; i++) {
		samples[i] = 2.5f;
	}
	lpf.reset(2.5f);
	lpf.applyArray(samples, 8);
	for (int i = 0; i < 8; i++) {
		EXPECT_NEAR(samples[i], 2.5f, 1e-3f);
	}
}

TEST(LowPassFilter2pApplyArray, EmptyIsNoOp)
{
	LowPassFilter2p<float> lpf(1000.f, 50.f);
	float samples[1] = {1.f};
	lpf.applyArray(samples, 0);
	EXPECT_FLOAT_EQ(samples[0], 1.f);
}
