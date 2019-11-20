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

/**
 * Test code for the Notch filter
 * Run this test only using make tests TESTFILTER=NotchFilter
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>

#include "NotchFilter.hpp"

using namespace matrix;
using namespace math;

class NotchFilterTest : public ::testing::Test
{
public:
	NotchFilter<float> _notch_float;
	const float _sample_freq = 1000.f;
	const float _cutoff_freq = 50.f;
	const float _bandwidth = 15.f;

	// a and b coefficients computed from an external python script
	const float b_expected[3] = {0.95496499f, -1.81645136f, 0.95496499f};
	const float a_expected[3] = {1.f, -1.81645136f, 0.90992999f};

	const float _epsilon_near = 1e-6f;
};

TEST_F(NotchFilterTest, simple)
{
	_notch_float.setParameters(_sample_freq, _cutoff_freq, _bandwidth);
	EXPECT_EQ(_notch_float.getBandwidth(), _bandwidth);
	EXPECT_EQ(_notch_float.getCutoffFreq(), _cutoff_freq);

	float a[3];
	float b[3];
	_notch_float.getCoefficients(a, b);

	for (int i = 0; i < 3; i++) {
		EXPECT_NEAR(a[i], a_expected[i], _epsilon_near);
		EXPECT_NEAR(b[i], b_expected[i], _epsilon_near);
	}
}

TEST_F(NotchFilterTest, filteringLowSide)
{
	// Send a 25Hz sinusoidal signal into a 50Hz notch filter
	_notch_float.setParameters(_sample_freq, _cutoff_freq, _bandwidth);
	const float signal_freq = 25.f;
	const float omega = 2.f * M_PI_F * signal_freq;
	float phase_delay = 11.4f * M_PI_F / 180.f; // Given by simulation
	float t = 0.f;
	float dt = 1.f / _sample_freq;
	float out = 0.f;

	for (int i = 0; i < 1000; i++) {
		float input = sinf(omega * t);
		float output_expected = sinf(omega * t - phase_delay);
		out = _notch_float.apply(input);
		t = i * dt;

		// Let some time for the filter to settle
		if (i > 30) {
			EXPECT_NEAR(out, output_expected, 0.05f);
		}
	}
}

TEST_F(NotchFilterTest, filteringHighSide)
{
	// Send a 98 sinusoidal signal into a 50Hz notch filter
	_notch_float.setParameters(_sample_freq, _cutoff_freq, _bandwidth);
	const float signal_freq = 98.4f;
	const float omega = 2.f * M_PI_F * signal_freq;
	float phase_delay = 11.4f * M_PI_F / 180.f; // Given by simulation
	float t = 0.f;
	float dt = 1.f / _sample_freq;
	float out = 0.f;

	for (int i = 0; i < 1000; i++) {
		float input = sinf(omega * t);
		float output_expected = sinf(omega * t + phase_delay);
		out = _notch_float.apply(input);
		t = i * dt;

		// Let some time for the filter to settle
		if (i > 30) {
			EXPECT_NEAR(out, output_expected, 0.05f);
		}
	}
}

TEST_F(NotchFilterTest, filterOnCutoff)
{
	// Send a 50 sinusoidal signal into a 50Hz notch filter
	_notch_float.setParameters(_sample_freq, _cutoff_freq, _bandwidth);
	const float signal_freq = 50.0f;
	const float omega = 2.f * M_PI_F * signal_freq;
	float t = 0.f;
	float dt = 1.f / _sample_freq;
	float out = 0.f;

	for (int i = 0; i < 1000; i++) {
		float input = sinf(omega * t);
		out = _notch_float.apply(input);
		t = i * dt;

		// Let some time for the filter to settle
		if (i > 50) {
			EXPECT_NEAR(out, 0.f, 0.1f);
		}
	}
}
