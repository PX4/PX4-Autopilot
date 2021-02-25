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
 * Test code for the 2nd order Butterworth low-pass filter
 * Run this test only using make tests TESTFILTER=LowPassFilter2pVector3f
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>

#include "LowPassFilter2pVector3f.hpp"

using matrix::Vector3f;

class LowPassFilter2pVector3fTest : public ::testing::Test
{
public:
	math::LowPassFilter2pVector3f _lpf{800.f, 30.f};
	const float _sample_freq = 1000.f;
	const float _cutoff_freq = 80.f;

	const float _epsilon_near = 10e-3f;
};

TEST_F(LowPassFilter2pVector3fTest, setGet)
{
	_lpf.set_cutoff_frequency(_sample_freq, _cutoff_freq);
	EXPECT_EQ(_lpf.get_sample_freq(), _sample_freq);
	EXPECT_EQ(_lpf.get_cutoff_freq(), _cutoff_freq);
}

TEST_F(LowPassFilter2pVector3fTest, belowCutoff)
{
	_lpf.set_cutoff_frequency(_sample_freq, _cutoff_freq);

	const float signal_freq = 10.f;
	const float omega = 2.f * M_PI_F * signal_freq;
	const float phase_delay = 10.4f * M_PI_F / 180.f; // Given by simulation
	const float dt = 1.f / _sample_freq;

	float t = 0.f;

	for (int i = 0; i < 1000; i++) {
		float input = sinf(omega * t);
		float output_expected = sinf(omega * t - phase_delay);
		Vector3f out = _lpf.apply(Vector3f(0.f, input, -input));
		t = i * dt;

		// Let some time for the filter to settle
		if (i > 30) {
			EXPECT_EQ(out(0), 0.f);
			EXPECT_NEAR(out(1), output_expected, _epsilon_near);
			EXPECT_NEAR(out(2), -output_expected, _epsilon_near);
		}
	}
}

TEST_F(LowPassFilter2pVector3fTest, aboveCutoff)
{
	_lpf.set_cutoff_frequency(_sample_freq, _cutoff_freq);

	const float signal_freq = 100.f;
	const float omega = 2.f * M_PI_F * signal_freq;
	const float phase_delay = 108.5f * M_PI_F / 180.f; // Given by simulation
	const float gain = 0.52f; // = -5.66 dB, given by simulation
	const float dt = 1.f / _sample_freq;

	float t = 0.f;

	for (int i = 0; i < 1000; i++) {
		float input = sinf(omega * t);
		float output_expected = gain * sinf(omega * t - phase_delay);
		Vector3f out = _lpf.apply(Vector3f(0.f, input, -input));
		t = i * dt;

		// Let some time for the filter to settle
		if (i > 30) {
			EXPECT_EQ(out(0), 0.f);
			EXPECT_NEAR(out(1), output_expected, _epsilon_near);
			EXPECT_NEAR(out(2), -output_expected, _epsilon_near);
		}
	}
}
