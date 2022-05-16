/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file baro_bias_estimator.hpp
 * @brief Implementation of a single-state baro bias estimator
 *
 * state: baro bias (in meters)
 *
 * The state is noise driven: Transition matrix A = 1
 * x[k+1] = Ax[k] + v with v ~ N(0, Q)
 * y[k] = x[k] + w with w ~ N(0, R)
 *
 * The difference between the current barometric altitude and another absolute
 * reference (e.g.: GNSS altitude) is used as a bias measurement.
 *
 * During the measurment update step, the Normalized Innovation Squared (NIS) is checked
 * and the measurement is rejected if larger than the gate size.
 *
 * @author Mathieu Bresciani 	<mathieu@auterion.com>
 */

#ifndef EKF_BARO_BIAS_ESTIMATOR_HPP
#define EKF_BARO_BIAS_ESTIMATOR_HPP

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/AlphaFilter.hpp>

class BaroBiasEstimator
{
public:
	struct status {
		float bias;
		float bias_var;
		float innov;
		float innov_var;
		float innov_test_ratio;
	};

	BaroBiasEstimator() = default;
	~BaroBiasEstimator() = default;

	void predict(float dt);
	void fuseBias(float measurement, float measurement_var);

	void setBias(float bias) { _state = bias; }
	void setProcessNoiseStdDev(float process_noise)
	{
		_process_var = process_noise * process_noise;
	}
	void setBiasStdDev(float state_noise) { _state_var = state_noise * state_noise; }
	void setInnovGate(float gate_size) { _gate_size = gate_size; }

	void setMaxStateNoise(float max_noise) { _state_var_max = max_noise * max_noise; }

	float getBias() const { return _state; }
	float getBiasVar() const { return _state_var; }
	const status &getStatus() const { return _status; }

private:
	float _state{0.f};
	float _state_max{100.f};
	float _dt{0.01f};

	float _gate_size{3.f}; ///< Used for innovation filtering (innovation test ratio)
	float _state_var{0.1f}; ///< Initial state uncertainty variance (m^2)
	float _process_var{25.0e-6f}; ///< State process noise variance (m^2/s^2)
	float _state_var_max{2.f}; ///< Used to constrain the state variance (m^2)

	// Innovation sequence monitoring; used to detect a bias in the state
	AlphaFilter<float> _signed_innov_test_ratio_lpf;
	float _time_since_last_negative_innov{0.f};
	float _time_since_last_positive_innov{0.f};

	status _status;

	void constrainStateVar();
	float computeKalmanGain(float innov_var) const;

	/*
	 * Compute the ratio between the Normalized Innovation Squared (NIS)
	 * and its maximum gate size. Use isTestRatioPassing to know if the
	 * measurement should be fused or not.
	 */
	float computeInnovTestRatio(float innov, float innov_var) const;
	bool isTestRatioPassing(float innov_test_ratio) const;

	void updateState(float K, float innov);
	void updateStateCovariance(float K);

	void updateOffsetDetection(float innov, float innov_test_ratio);
	bool isOffsetDetected() const;

	status packStatus(float innov, float innov_var, float innov_test_ratio) const;

	static constexpr float _innov_sequence_monitnoring_time_constant{10.f}; ///< in seconds
	static constexpr float _process_var_boost_gain{1.0e3f};
};
#endif // !EKF_BARO_BIAS_ESTIMATOR_HPP
