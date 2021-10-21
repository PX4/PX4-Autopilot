/****************************************************************************
 *
 *   Copyright (C) 2020-2021 PX4 Development Team. All rights reserved.
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
 * Test code for the SystemIdentification class
 * Run this test only using make tests TESTFILTER=system_identification
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>

#include "system_identification.hpp"
#include "test_data.h"

using namespace matrix;

class SystemIdentificationTest : public ::testing::Test
{
public:
	SystemIdentificationTest() {};
	float apply(float sample);
	void setCoefficients(float a1, float a2, float b0, float b1, float b2)
	{
		_a1 = a1;
		_a2 = a2;
		_b0 = b0;
		_b1 = b1;
		_b2 = b2;
	}

private:
	float _a1{};
	float _a2{};
	float _b0{};
	float _b1{};
	float _b2{};
	float _delay_element_1{};
	float _delay_element_2{};
};

float SystemIdentificationTest::apply(float sample)
{
	// Direct Form II implementation
	const float delay_element_0{sample - _delay_element_1 *_a1 - _delay_element_2 * _a2};
	const float output{delay_element_0 *_b0 + _delay_element_1 *_b1 + _delay_element_2 * _b2};

	_delay_element_2 = _delay_element_1;
	_delay_element_1 = delay_element_0;

	return output;
}

TEST_F(SystemIdentificationTest, basicTest)
{
	constexpr float fs = 800.f;

	SystemIdentification _sys_id;
	_sys_id.setHpfCutoffFrequency(fs, 0.05f);
	_sys_id.setLpfCutoffFrequency(fs, 30.f);
	_sys_id.setForgettingFactor(80.f, 1.f / fs);

	for (int i = 0; i < 5; i++) {
		// Fill the buffers with zeros
		_sys_id.update(0.f, 0.f);
	}

	for (int i = 0; i < 10; i += 2) {
		_sys_id.update(float(i), float(i + 1));
	}

	const Vector<float, 5> coefficients = _sys_id.getCoefficients();
	float data_check[] = {-3.96f, 1.30f, -2.28f, -0.33f, 0.369f};
	const Vector<float, 5> coefficients_check(data_check);
	float eps = 1e-2;
	EXPECT_TRUE((coefficients - coefficients_check).abs().max() < eps);
	coefficients.print();
	_sys_id.getVariances().print();
}

TEST_F(SystemIdentificationTest, resetTest)
{
	constexpr float fs = 800.f;

	SystemIdentification _sys_id;
	_sys_id.setHpfCutoffFrequency(fs, 0.05f);
	_sys_id.setLpfCutoffFrequency(fs, 30.f);
	_sys_id.setForgettingFactor(80.f, 1.f / fs);

	for (int i = 0; i < 10; i += 2) {
		_sys_id.update(float(i), float(i + 1));
	}

	const Vector<float, 5> coefficients = _sys_id.getCoefficients();

	// WHEN: resetting
	_sys_id.reset();

	// THEN: the variances and coefficients should be properly reset
	EXPECT_TRUE(_sys_id.getCoefficients().abs().max() < 1e-8f);
	EXPECT_TRUE(_sys_id.getVariances().min() > 9e3f);

	// AND WHEN: running the same sequence of inputs-outputs

	for (int i = 0; i < 10; i += 2) {
		_sys_id.update(float(i), float(i + 1));
	}

	// THEN: the result should be exactly the same
	EXPECT_TRUE((coefficients - _sys_id.getCoefficients()).abs().max() < 1e-8f);
	coefficients.print();
	_sys_id.getVariances().print();
}

TEST_F(SystemIdentificationTest, simulatedModelTest)
{
	constexpr float fs = 200.f;
	const float gyro_lpf_cutoff = 30.f;

	SystemIdentification _sys_id;
	_sys_id.setHpfCutoffFrequency(fs, 0.05f);
	_sys_id.setLpfCutoffFrequency(fs, gyro_lpf_cutoff);
	_sys_id.setForgettingFactor(60.f, 1.f / fs);

	// Simulated model with integrator
	const float a1 = -1.77f;
	const float a2 = 0.77f;
	const float b0 = 0.3812f;
	const float b1 = -0.25f;
	const float b2 = 0.2f;
	setCoefficients(a1, a2, b0, b1, b2);
	const float u_bias = -0.1f; // constant control offset
	const float y_bias = 0.2f; // measurement bias

	const float dt = 1.f / fs;
	const float duration = 2.f;
	float u = 0.f;
	float y = 0.f;

	for (int i = 0; i < static_cast<int>(duration / dt); i++) {

		// Generate square input signal
		if (_sys_id.areFiltersInitialized()
		    && (i % 30 == 0)) {
			if (u > 0.f) {
				u = -1.f;

			} else {
				u = 1.f;
			}
		}

		_sys_id.update(u + u_bias, y + y_bias); // apply new input and previous output

		y = apply(u);
#if 0
		const float t = i * dt;
		printf("%.6f, %.6f, %.6f\n", (double)t, (double)u, (double)y);
#endif
	}

	const Vector<float, 5> coefficients = _sys_id.getCoefficients();
	float data_check[] = {a1, a2, b0, b1, b2};
	const Vector<float, 5> coefficients_check(data_check);
	float eps = 1e-3;
	EXPECT_TRUE((coefficients - coefficients_check).abs().max() < eps);
	coefficients.print();
	_sys_id.getVariances().print();
}

TEST_F(SystemIdentificationTest, realDataTest)
{
	static constexpr int n_samples_dt_avg = 100;
	static constexpr int n_samples_test_data = sizeof(u_data) / sizeof(u_data[0]);
	float dt_sum = 0.f;

	for (int i = 1; i <= n_samples_dt_avg; i++) {
		dt_sum += t_data[i] - t_data[i - 1];
	}

	const float dt = dt_sum / n_samples_dt_avg;
	const float fs = 1.f / dt;

	printf("dt = %.6f fs = %.3f\n", (double)dt, (double)(1.f / dt));

	const float gyro_lpf_cutoff = 30.f;

	SystemIdentification _sys_id;
	_sys_id.setHpfCutoffFrequency(fs, 0.05f);
	_sys_id.setLpfCutoffFrequency(fs, gyro_lpf_cutoff);
	_sys_id.setForgettingFactor(60.f, dt);

	float u = 0.f;
	float y_prev = 0.f;

	for (int i = 0; i < n_samples_test_data; i++) {
		u = u_data[i];
		_sys_id.update(u, y_prev); // apply new input and previous output

		y_prev = y_data[i];

#if 0
		printf("%.6f, %.6f, %.6f\n", (double)t_data[i], (double)u, (double)y);
#endif
	}

	const Vector<float, 5> coefficients = _sys_id.getCoefficients();
	float data_check[] = {-1.916f, 0.916, -0.017f, -0.001f, 0.036f};
	const Vector<float, 5> coefficients_check(data_check);
	float eps = 1e-3;
	EXPECT_TRUE((coefficients - coefficients_check).abs().max() < eps);
	coefficients.print();
	_sys_id.getVariances().print();
}
