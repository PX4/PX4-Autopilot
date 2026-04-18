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

#include <gtest/gtest.h>
#include <RpmControl.hpp>

namespace
{
constexpr float kMaxRpm = 12500.f;
constexpr float kDt     = 0.002f;
}

TEST(RpmControlTest, ZeroGainsPassesSetpointThrough)
{
	RpmControl c;
	c.setMaxRpm(kMaxRpm);
	c.setGains({0.f, 0.f, 0.f, 0.5f});

	EXPECT_FLOAT_EQ(c.update(0, 0.5f, 0.f, kDt), 0.5f);
	EXPECT_FLOAT_EQ(c.update(0, 1.f,  0.f, kDt), 1.0f);   // clamped to 1
	EXPECT_FLOAT_EQ(c.update(0, 0.f,  0.f, kDt), 0.0f);
}

TEST(RpmControlTest, ProportionalCorrectionOnNegativeError)
{
	// Measured RPM above setpoint → command should drop below setpoint.
	RpmControl c;
	c.setMaxRpm(kMaxRpm);
	c.setGains({1.f, 0.f, 0.f, 0.5f});

	const float sp_norm = 0.5f;
	const float meas    = kMaxRpm;              // running at 100%
	const float cmd     = c.update(0, sp_norm, meas, kDt);

	// error = (0.5*max - max) / max = -0.5 → cmd = 0.5 + 1.0 * (-0.5) = 0 (clamped)
	EXPECT_FLOAT_EQ(cmd, 0.f);
}

TEST(RpmControlTest, ProportionalCorrectionOnPositiveError)
{
	RpmControl c;
	c.setMaxRpm(kMaxRpm);
	c.setGains({1.f, 0.f, 0.f, 0.5f});

	const float sp_norm = 0.5f;
	const float meas    = 0.f;                  // not spinning
	const float cmd     = c.update(0, sp_norm, meas, kDt);

	// error = +0.5, cmd = 0.5 + 0.5 = 1.0
	EXPECT_FLOAT_EQ(cmd, 1.f);
}

TEST(RpmControlTest, IntegratorSaturatesAtILimit)
{
	RpmControl c;
	c.setMaxRpm(kMaxRpm);
	c.setGains({0.f, 10.f, 0.f, 0.25f});

	// Large persistent error should push the integrator to the limit.
	for (int i = 0; i < 1000; i++) {
		c.update(0, 1.f, 0.f, kDt);
	}

	EXPECT_NEAR(c.integral(0), 0.25f, 1e-4f);
}

TEST(RpmControlTest, ResetClearsState)
{
	RpmControl c;
	c.setMaxRpm(kMaxRpm);
	c.setGains({0.5f, 1.f, 0.f, 0.5f});

	for (int i = 0; i < 100; i++) {
		c.update(0, 1.f, 0.f, kDt);
	}

	EXPECT_GT(c.integral(0), 0.f);

	c.reset(0);

	EXPECT_FLOAT_EQ(c.integral(0),     0.f);
	EXPECT_FLOAT_EQ(c.lastCmdNorm(0),  0.f);
	EXPECT_FLOAT_EQ(c.setpointRpm(0),  0.f);
	EXPECT_FLOAT_EQ(c.measuredRpm(0),  0.f);
}

TEST(RpmControlTest, OutOfRangeMotorIsSafe)
{
	RpmControl c;
	c.setMaxRpm(kMaxRpm);
	c.setGains({1.f, 1.f, 0.f, 0.5f});

	// Must not crash; returns clamped setpoint.
	EXPECT_FLOAT_EQ(c.update(-1,   0.5f, 0.f, kDt), 0.5f);
	EXPECT_FLOAT_EQ(c.update(9999, 0.5f, 0.f, kDt), 0.5f);
}

TEST(RpmControlTest, ZeroDtSkipsDerivative)
{
	RpmControl c;
	c.setMaxRpm(kMaxRpm);
	c.setGains({0.f, 0.f, 1.f, 0.5f});

	// With D=1 and dt=0 the update must not produce NaN/Inf.
	const float cmd = c.update(0, 0.5f, 0.f, 0.f);
	EXPECT_TRUE(cmd >= 0.f && cmd <= 1.f);
}
