/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <cmath>

#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

using math::LowPassFilter2p;

TEST(LowPassFilter2pScalarTest, InvalidCutoffDisablesPassthrough)
{
	LowPassFilter2p<float> lpf(100.f, -1.f); // invalid → disable
	EXPECT_FLOAT_EQ(lpf.get_cutoff_freq(), 0.f);
	EXPECT_FLOAT_EQ(lpf.apply(3.5f), 3.5f);
	EXPECT_FLOAT_EQ(lpf.apply(-1.f), -1.f);
}

TEST(LowPassFilter2pScalarTest, NyquistCutoffDisables)
{
	LowPassFilter2p<float> lpf;
	lpf.set_cutoff_frequency(200.f, 100.f); // == sample/2 invalid
	EXPECT_FLOAT_EQ(lpf.get_cutoff_freq(), 0.f);
	EXPECT_FLOAT_EQ(lpf.apply(2.f), 2.f);
}

TEST(LowPassFilter2pScalarTest, ResetSettlesTowardConstant)
{
	LowPassFilter2p<float> lpf(1000.f, 50.f);
	// start from reset to 0, drive constant input
	EXPECT_TRUE(std::isfinite(lpf.reset(0.f)));

	float y = 0.f;

	for (int i = 0; i < 500; i++) {
		y = lpf.apply(5.f);
	}

	EXPECT_NEAR(y, 5.f, 0.05f);
}

TEST(LowPassFilter2pScalarTest, ApplyArrayMatchesSequential)
{
	LowPassFilter2p<float> a(500.f, 20.f);
	LowPassFilter2p<float> b(500.f, 20.f);
	a.reset(0.f);
	b.reset(0.f);

	float samples[4] = {1.f, 2.f, 3.f, 4.f};
	float seq[4];

	for (int i = 0; i < 4; i++) {
		seq[i] = a.apply(samples[i]);
	}

	b.applyArray(samples, 4);

	for (int i = 0; i < 4; i++) {
		EXPECT_FLOAT_EQ(samples[i], seq[i]);
	}
}
