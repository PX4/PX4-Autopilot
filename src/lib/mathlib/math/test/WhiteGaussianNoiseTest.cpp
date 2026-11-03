/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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


/**
 * Test for the shared white Gaussian noise generator
 */

#include <gtest/gtest.h>
#include <mathlib/math/WhiteGaussianNoise.hpp>

#include <cmath>
#include <cstdlib>

TEST(WhiteGaussianNoiseTest, unitNormalStatistics)
{
	srand(1234);
	constexpr int n = 200000;
	double sum = 0.0;
	double sum_sq = 0.0;
	float min = 0.f;
	float max = 0.f;

	for (int i = 0; i < n; i++) {
		const float x = math::generate_wgn();
		EXPECT_TRUE(std::isfinite(x));
		sum += (double)x;
		sum_sq += (double)x * (double)x;
		min = fminf(min, x);
		max = fmaxf(max, x);
	}

	const double mean = sum / n;
	const double var = sum_sq / n - mean * mean;

	EXPECT_NEAR(mean, 0.0, 0.02);
	EXPECT_NEAR(var, 1.0, 0.02);
	EXPECT_LT(min, -3.f);
	EXPECT_GT(max, 3.f);
}

TEST(WhiteGaussianNoiseTest, vectorScalesByStd)
{
	srand(42);
	constexpr int n = 100000;
	matrix::Vector3f sum_sq{};

	for (int i = 0; i < n; i++) {
		const matrix::Vector3f v = math::noiseGauss3f(0.5f, 1.f, 2.f);
		sum_sq += v.emult(v);
	}

	EXPECT_NEAR(sqrtf(sum_sq(0) / n), 0.5f, 0.02f);
	EXPECT_NEAR(sqrtf(sum_sq(1) / n), 1.f, 0.03f);
	EXPECT_NEAR(sqrtf(sum_sq(2) / n), 2.f, 0.05f);
}
