/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
#include <lib/rate_control/rate_control.hpp>

using namespace matrix;

TEST(RateControlTest, AllZeroCase)
{
	RateControl rate_control;
	Vector3f torque = rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.f, false);
	EXPECT_EQ(torque, Vector3f());
}

namespace
{

constexpr float kIntegratorLimit = 0.3f;
constexpr float kIntegralGain = 1.f;
constexpr float kDt = 0.01f;

// Rate controller with only an integral gain, so the returned torque is the integrator itself.
RateControl createIntegratorOnlyRateControl()
{
	RateControl rate_control;
	rate_control.setPidGains(Vector3f(), Vector3f(kIntegralGain, kIntegralGain, kIntegralGain), Vector3f());
	rate_control.setIntegratorLimit(Vector3f(kIntegratorLimit, kIntegratorLimit, kIntegratorLimit));
	return rate_control;
}

// Run the controller with a constant rate error until the integrator settles, and return the
// integrator state of the roll axis.
float windUpRollIntegrator(RateControl &rate_control, float rate_error)
{
	for (int i = 0; i < 1000; i++) {
		rate_control.update(Vector3f(), Vector3f(rate_error, 0.f, 0.f), Vector3f(), kDt, false);
	}

	rate_ctrl_status_s status{};
	rate_control.getRateControlStatus(status);
	return status.rollspeed_integ;
}

} // namespace


TEST(RateControlTest, NothingRefusedKeepsFullIntegratorLimit)
{
	RateControl rate_control = createIntegratorOnlyRateControl();
	rate_control.setRefusedTorqueFraction(Vector3f());

	EXPECT_NEAR(windUpRollIntegrator(rate_control, 1.f), kIntegratorLimit, 1e-6f);
	EXPECT_NEAR(windUpRollIntegrator(rate_control, -1.f), -kIntegratorLimit, 1e-6f);
}

TEST(RateControlTest, RefusedPositiveFractionLowersIntegratorCeiling)
{
	// A quarter of the requested positive torque was refused.
	RateControl rate_control = createIntegratorOnlyRateControl();
	rate_control.setRefusedTorqueFraction(Vector3f(0.25f, 0.f, 0.f));

	EXPECT_NEAR(windUpRollIntegrator(rate_control, 1.f), 0.75f * kIntegratorLimit, 1e-6f);
}

TEST(RateControlTest, RefusedNegativeFractionRaisesIntegratorFloor)
{
	RateControl rate_control = createIntegratorOnlyRateControl();
	rate_control.setRefusedTorqueFraction(Vector3f(-0.25f, 0.f, 0.f));

	EXPECT_NEAR(windUpRollIntegrator(rate_control, -1.f), -0.75f * kIntegratorLimit, 1e-6f);
}

TEST(RateControlTest, RefusalLeavesOppositeSignTrimUntouched)
{
	RateControl rate_control = createIntegratorOnlyRateControl();

	// Build up negative trim while the allocator delivers everything.
	rate_control.setRefusedTorqueFraction(Vector3f());
	const float trim = windUpRollIntegrator(rate_control, -1.f);
	EXPECT_NEAR(trim, -kIntegratorLimit, 1e-6f);

	// Refusing positive torque must not squeeze a negative integrator.
	rate_control.setRefusedTorqueFraction(Vector3f(0.5f, 0.f, 0.f));
	rate_control.update(Vector3f(), Vector3f(), Vector3f(), kDt, false);

	rate_ctrl_status_s status{};
	rate_control.getRateControlStatus(status);
	EXPECT_NEAR(status.rollspeed_integ, trim, 1e-6f);
}

TEST(RateControlTest, RefusalExceedingTheRequestDoesNotFlipIntegratorSign)
{
	// More was refused than was asked for, which the [0,1] constraint has to absorb.
	RateControl rate_control = createIntegratorOnlyRateControl();
	rate_control.setRefusedTorqueFraction(Vector3f(5.f, 0.f, 0.f));

	EXPECT_NEAR(windUpRollIntegrator(rate_control, 1.f), 0.f, 1e-6f);
}

TEST(RateControlTest, IntegratorRecoversWhenRefusalClears)
{
	RateControl rate_control = createIntegratorOnlyRateControl();

	rate_control.setRefusedTorqueFraction(Vector3f(0.25f, 0.f, 0.f));
	EXPECT_NEAR(windUpRollIntegrator(rate_control, 1.f), 0.75f * kIntegratorLimit, 1e-6f);

	rate_control.setRefusedTorqueFraction(Vector3f());
	EXPECT_NEAR(windUpRollIntegrator(rate_control, 1.f), kIntegratorLimit, 1e-6f);
}

TEST(RateControlTest, ReportsIntegratorLimits)
{
	RateControl rate_control = createIntegratorOnlyRateControl();
	rate_ctrl_status_s status{};

	// Nothing refused: both bounds at the parameter value.
	rate_control.setRefusedTorqueFraction(Vector3f());
	rate_control.getRateControlStatus(status);
	EXPECT_NEAR(status.integrator_limit_upper[0], kIntegratorLimit, 1e-6f);
	EXPECT_NEAR(status.integrator_limit_lower[0], -kIntegratorLimit, 1e-6f);

	// Refused positive torque lowers the ceiling only.
	rate_control.setRefusedTorqueFraction(Vector3f(0.25f, 0.f, 0.f));
	rate_control.getRateControlStatus(status);
	EXPECT_NEAR(status.integrator_limit_upper[0], 0.75f * kIntegratorLimit, 1e-6f);
	EXPECT_NEAR(status.integrator_limit_lower[0], -kIntegratorLimit, 1e-6f);

	// Refused negative torque raises the floor only.
	rate_control.setRefusedTorqueFraction(Vector3f(-0.25f, 0.f, 0.f));
	rate_control.getRateControlStatus(status);
	EXPECT_NEAR(status.integrator_limit_upper[0], kIntegratorLimit, 1e-6f);
	EXPECT_NEAR(status.integrator_limit_lower[0], -0.75f * kIntegratorLimit, 1e-6f);
}

TEST(RateControlTest, ResetIntegralClearsRefusedFraction)
{
	RateControl rate_control = createIntegratorOnlyRateControl();

	rate_control.setRefusedTorqueFraction(Vector3f(0.25f, 0.f, 0.f));
	rate_control.resetIntegral();

	EXPECT_NEAR(windUpRollIntegrator(rate_control, 1.f), kIntegratorLimit, 1e-6f);
}
