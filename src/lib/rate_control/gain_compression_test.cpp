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

#include "gain_compression.hpp"

TEST(GainCompressionTest, ResetStartsAtUnity)
{
	GainCompression gc;
	gc.reset();
	gc.setCompressionGainMin(0.3f);
	gc.setLpfCutoffFrequency(100.f, 5.f);
	gc.setHpfCutoffFrequency(100.f, 10.f);

	// quiescent input should leave gain near 1
	float g = 1.f;

	for (int i = 0; i < 20; i++) {
		g = gc.update(0.f, 0.01f);
	}

	EXPECT_NEAR(g, 1.f, 0.05f);
}

TEST(GainCompressionTest, NonFiniteInputKeepsGain)
{
	GainCompression gc;
	gc.reset();
	gc.setCompressionGainMin(0.25f);
	gc.setLpfCutoffFrequency(200.f, 5.f);
	gc.setHpfCutoffFrequency(200.f, 20.f);

	const float before = gc.update(0.1f, 0.01f);
	const float after_nan = gc.update(NAN, 0.01f);
	EXPECT_FLOAT_EQ(after_nan, before);

	const float after_inf = gc.update(INFINITY, 0.01f);
	EXPECT_FLOAT_EQ(after_inf, before);
}

TEST(GainCompressionTest, GainBoundedByMinAndOne)
{
	GainCompression gc;
	gc.reset();
	const float gmin = 0.4f;
	gc.setCompressionGainMin(gmin);
	gc.setLpfCutoffFrequency(100.f, 2.f);
	gc.setHpfCutoffFrequency(100.f, 5.f);

	// vigorous oscillating input encourages compression
	float g = 1.f;

	for (int i = 0; i < 500; i++) {
		const float u = (i % 2 == 0) ? 10.f : -10.f;
		g = gc.update(u, 0.01f);
		EXPECT_GE(g, gmin - 1e-5f);
		EXPECT_LE(g, 1.f + 1e-5f);
	}
}
