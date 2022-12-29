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

/**
 * Test code for the 1st order low-pass filter
 * Run this test only using make tests TESTFILTER=LowPassFilter1pVector3f
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>

#include "LowPassFilter1p.hpp"

using matrix::Vector3f;

class LowPassFilter1pVector3fTest : public ::testing::Test
{
public:
	void runSimulatedFilter(const Vector3f &signal_freq_hz, const Vector3f &phase_delay_deg, const Vector3f &gain_db,
				float dt);

	math::LowPassFilter1p<Vector3f> _lpf{30.f};

	const float _epsilon_near = 0.01f;
};

void LowPassFilter1pVector3fTest::runSimulatedFilter(const Vector3f &signal_freq_hz, const Vector3f &phase_delay_deg,
		const Vector3f &gain_db, const float dt)
{
	const Vector3f phase_delay = phase_delay_deg * M_PI_F / 180.f;
	const Vector3f omega = 2.f * M_PI_F * signal_freq_hz;
	Vector3f gain;

	for (int i = 0; i < 3; i++) {
		gain(i) = powf(10.f, gain_db(i) / 20.f);
	}

	const int n_steps = roundf(1.f / dt); // run for 1 second

	float t = 0.f;

	for (int i = 0; i < n_steps; i++) {
		Vector3f input{0.f, sinf(omega(1) * t), -sinf(omega(2) * t)};
		Vector3f output_expected{0.f,
					 gain(1) *sinf(omega(1) * t - phase_delay(1)),
					 -gain(2) *sinf(omega(2) * t - phase_delay(2))};
		Vector3f out = _lpf.apply(input, dt);
		t = i * dt;

		// Let some time for the filter to settle
		if (t > 0.3f) {
			EXPECT_EQ(out(0), 0.f);
			EXPECT_NEAR(out(1), output_expected(1), _epsilon_near);
			EXPECT_NEAR(out(2), output_expected(2), _epsilon_near);
		}
	}
}

TEST_F(LowPassFilter1pVector3fTest, setGet)
{
	const float cutoff_freq = 50.f;

	_lpf.set_cutoff_frequency(0.f);;
	EXPECT_TRUE(_lpf.disabled());

	_lpf.set_cutoff_frequency(cutoff_freq);
	EXPECT_FALSE(_lpf.disabled());

	EXPECT_EQ(_lpf.get_cutoff_freq(), cutoff_freq);
}

TEST_F(LowPassFilter1pVector3fTest, simulate80HzCutoff)
{
	const float cutoff_freq = 50.f;
	const Vector3f signal_freq_hz{0.f, 50.f, 100.f};

	{
		// 500Hz sample frequency
		const float sample_freq = 500.f;
		const float dt = 1.f / sample_freq;
		const Vector3f phase_delay_deg = Vector3f{1.f, 35.66f, 35.79f}; // Given by simulation
		const Vector3f gain_db{0.f, -4.11f, -8.26f}; // given by simulation
		_lpf.set_cutoff_frequency(cutoff_freq);
		runSimulatedFilter(signal_freq_hz, phase_delay_deg, gain_db, dt);
	}

	{
		// 1000Hz sample frequency
		const float sample_freq = 1000.f;
		const float dt = 1.f / sample_freq;
		const Vector3f phase_delay_deg = Vector3f{0.f, 40.4f, 49.32f}; // Given by simulation
		const Vector3f gain_db{0.f, -3.62f, -7.84f}; // Given by simulation
		_lpf.set_cutoff_frequency(cutoff_freq);
		runSimulatedFilter(signal_freq_hz, phase_delay_deg, gain_db, dt);
	}
	const float dt = 0.01f;

	// Check filter ignores zero dt (should return same value)
	Vector3f previous = _lpf.get();
	EXPECT_NEAR(previous(1), _lpf.apply(Vector3f(100., 100., 100.), 0.f)(1), _epsilon_near);

	// Check reset and disable without resetting states works
	_lpf.set_cutoff_frequency(cutoff_freq, false);
	EXPECT_NEAR(previous(1), _lpf.get()(1), _epsilon_near);
	_lpf.disable(false);
	EXPECT_NEAR(previous(1), _lpf.get()(1), _epsilon_near);

	// Check reset and disable with resetting states works
	_lpf.set_cutoff_frequency(cutoff_freq);
	EXPECT_NEAR(0.f, _lpf.get()(1), _epsilon_near);
	_lpf.reset(Vector3f(100., 100., 100.));
	EXPECT_NEAR(100.f, _lpf.get()(1), _epsilon_near);
	_lpf.disable();
	EXPECT_NEAR(0.f, _lpf.get()(1), _epsilon_near);

	// Check filter returns last set value if disabled
	_lpf.disable();
	EXPECT_NEAR(100.f, _lpf.apply(Vector3f(100., 100., 100.), dt)(1), _epsilon_near);
	EXPECT_NEAR(100.f, _lpf.get()(1), _epsilon_near);
}
