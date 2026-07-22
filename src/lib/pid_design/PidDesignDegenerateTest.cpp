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
#include <matrix/matrix/math.hpp>
#include <cmath>
#include "pid_design.hpp"

using matrix::Vector3f;

TEST(PidDesignDegenerateTest, ZeroNumeratorReturnsZeroGains)
{
	// num all zeros → nu ~ 0 → fail-closed empty gains
	const Vector3f num(0.f, 0.f, 0.f);
	const Vector3f den(1.f, -1.5f, 0.7f);
	const Vector3f gains = pid_design::computePidGmvc(num, den, 0.01f, 0.1f, 1.f, 0.5f);

	EXPECT_FLOAT_EQ(gains(0), 0.f);
	EXPECT_FLOAT_EQ(gains(1), 0.f);
	EXPECT_FLOAT_EQ(gains(2), 0.f);
}

TEST(PidDesignDegenerateTest, ParameterClampsStillFinite)
{
	// Extreme out-of-range design params are clamped inside helper
	const Vector3f num(0.1f, 0.05f, 0.f);
	const Vector3f den(1.f, -1.2f, 0.3f);
	const Vector3f gains = pid_design::computePidGmvc(num, den, 0.02f, 5.f /*sigma*/, -1.f /*delta*/, 50.f /*lbda*/);

	EXPECT_TRUE(std::isfinite(gains(0)));
	EXPECT_TRUE(std::isfinite(gains(1)));
	EXPECT_TRUE(std::isfinite(gains(2)));
}
