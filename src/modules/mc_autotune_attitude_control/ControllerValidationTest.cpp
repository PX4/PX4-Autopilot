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
#include <memory>
#include "ControllerValidation.hpp"

class ControllerValidationTest : public ::testing::Test
{
protected:
	using C = ControllerValidation::Complex;
	std::unique_ptr<ControllerValidation> validation{new ControllerValidation};
	ControllerValidation::Gains old;
	static constexpr float dt = .004f;
	void SetUp() override
	{
		old.p = matrix::Vector3f(.2f, .2f, .2f);
		old.i = matrix::Vector3f(.1f, .1f, .1f);
		old.d = matrix::Vector3f(.005f, .005f, .005f);
		old.attitude = matrix::Vector3f(2.f, 2.f, 2.f);
		validation->configure(8.f, 20.f);
	}
	void measure(bool missing_excitation = false, float noise = 0.f)
	{
		C S[ControllerValidation::MaxFrequencies][3] {};
		C H[ControllerValidation::MaxFrequencies][3] {};
		C A[ControllerValidation::MaxFrequencies][3] {};

		for (int bin = 0; bin < validation->frequencies(); ++bin) {
			const float w = 2.f * M_PI_F * validation->frequency(bin);
			const C s(0.f, w), q = C::polar(1.f, -w * dt);
			const C plant = 20.f / (s * (1.f + .1f * s));
			const C derivative = s / (1.f + .01f * s);

			for (int row = 0; row < 3; ++row) {
				C filter(1.f);

				if (row == 2) { const float alpha = dt / (dt + 1.f / (4.f * M_PI_F)); filter = alpha / (1.f - (1.f - alpha) * q); }

				const C cp = filter * (old.p(row) + old.i(row) * dt * q / (1.f - q));
				const C controller = cp * (1.f + old.attitude(row) / s) + filter * old.d(row) * derivative;
				S[bin][row] = 1.f / (1.f + controller * plant);
				H[bin][row] = plant * S[bin][row]; A[bin][row] = derivative * H[bin][row];
			}
		}

		for (int axis = 0; axis < 3; ++axis) {
			constexpr uint64_t start = 1000000;
			validation->beginAxis(axis, start, .003f);
			const int steps = int(ControllerValidation::TotalPeriods * 8.f / dt);

			for (int n = 1; n <= steps + 1; ++n) {
				const float t = n * dt;
				const int group = t >= ControllerValidation::GroupPeriods * 8.f ? 1 : 0;
				matrix::Vector3f u{}, y{}, a{};
				float e = 0.f;

				for (int bin = 0; bin < validation->frequencies(); ++bin) {
					const float phase = M_PI_F * bin * (bin - 1) / validation->frequencies() + .9f * group * (bin + 1);
					const C input = C::polar(.003f, 2.f * M_PI_F * validation->frequency(bin) * t + phase - M_PI_F * .5f);
					e += input.real();
					u(axis) += (S[bin][axis] * input).real();
					y(axis) += (H[bin][axis] * input).real();
					a(axis) += (A[bin][axis] * input).real();
				}

				// An unrelated disturbance and DC/ramp expose false confidence from smooth outputs.
				y(axis) += .002f + .00001f * t + noise * sinf(2.f * M_PI_F * .773f * t);
				validation->update(start + uint64_t(n) * 4000, dt, u, y, a, missing_excitation ? 0.f : e);
			}

			if (!missing_excitation) { EXPECT_TRUE(validation->finished()); }
		}
	}
};

TEST_F(ControllerValidationTest, AcceptsMeasuredSmallControllerChange)
{
	measure();
	auto candidate = old;
	candidate.p *= .9f; candidate.i *= .8f; candidate.d *= 1.2f;
	float bound = 0.f;
	EXPECT_EQ(validation->check(old, candidate, 2.f, bound), ControllerValidation::Result::Pass) << bound;
}

TEST_F(ControllerValidationTest, RejectsExcessiveOuterGainDespiteFinitePID)
{
	measure();
	auto candidate = old; candidate.attitude *= 20.f;
	float bound = 0.f;
	EXPECT_EQ(validation->check(old, candidate, 2.f, bound), ControllerValidation::Result::Reject) << bound;
}

TEST_F(ControllerValidationTest, RejectsNoActualExcitation)
{
	measure(true);
	float bound = 0.f;
	EXPECT_EQ(validation->check(old, old, 2.f, bound), ControllerValidation::Result::Reject);
}

TEST_F(ControllerValidationTest, RejectsNoisyMeasurementOfChangedController)
{
	measure(false, .2f);
	auto candidate = old; candidate.attitude *= 2.f;
	float bound = 0.f;
	EXPECT_NE(validation->check(old, candidate, 2.f, bound), ControllerValidation::Result::Pass);
}

TEST_F(ControllerValidationTest, MissingPeriodsCannotPassEvenForUnchangedGains)
{
	validation->beginAxis(0, 1000000, .003f);
	validation->update(30000000, dt, {}, {}, {}, .003f);
	float bound = 0.f;
	EXPECT_NE(validation->check(old, old, 2.f, bound), ControllerValidation::Result::Pass);
}

TEST_F(ControllerValidationTest, InvalidConfigurationCannotPass)
{
	validation->configure(NAN, 20.f);
	validation->beginAxis(0, 1000000, .003f);
	EXPECT_FALSE(validation->validData());
	float bound = 0.f;
	EXPECT_EQ(validation->check(old, old, 2.f, bound), ControllerValidation::Result::Reject);
}

TEST_F(ControllerValidationTest, RepeatedTimestampDoesNotAddASecondObservation)
{
	validation->beginAxis(0, 1000000, .003f);
	EXPECT_TRUE(validation->update(1004000, dt, {}, {}, {}, .003f));
	EXPECT_FALSE(validation->update(1004000, dt, {}, {}, {}, .003f));
	EXPECT_TRUE(validation->validData());
	EXPECT_TRUE(validation->update(1008000, dt, {}, {}, {}, .003f));
}
