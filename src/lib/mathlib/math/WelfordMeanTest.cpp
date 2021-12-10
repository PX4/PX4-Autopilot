/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
#include <random>
#include <lib/matrix/matrix/math.hpp>
#include "WelfordMean.hpp"

using namespace math;
using matrix::Vector3f;

TEST(WelfordMeanTest, NoisySignal)
{
	const float std_dev = 3.f;
	std::normal_distribution<float> standard_normal_distribution{0.f, std_dev};
	std::default_random_engine random_generator{}; // Pseudo-random generator with constant seed
	random_generator.seed(42);
	WelfordMean<Vector3f> welford{};

	for (int i = 0; i < 50; i++) {
		const float noisy_value = standard_normal_distribution(random_generator);
		welford.update(Vector3f(noisy_value, noisy_value - 1.f, noisy_value + 1.f));
	}

	EXPECT_TRUE(welford.valid());
	const Vector3f mean = welford.mean();
	const Vector3f var = welford.variance();
	const float var_real = std_dev * std_dev;

	EXPECT_NEAR(mean(0), 0.f, 0.7f);
	EXPECT_NEAR(mean(1), -1.f, 0.7f);
	EXPECT_NEAR(mean(2), 1.f, 0.7f);
	EXPECT_NEAR(var(0), var_real, 0.1f);
	EXPECT_NEAR(var(1), var_real, 0.1f);
	EXPECT_NEAR(var(2), var_real, 0.1f);
}
