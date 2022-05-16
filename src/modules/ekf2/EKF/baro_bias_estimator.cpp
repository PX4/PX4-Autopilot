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
 * @file baro_bias_estimator.cpp
 *
 * @author Mathieu Bresciani 	<mathieu@auterion.com>
 */

#include "baro_bias_estimator.hpp"

void BaroBiasEstimator::predict(const float dt)
{
	// State is constant
	// Predict state covariance only
	float delta_state_var = _process_var * dt * dt;

	if (isOffsetDetected()) {
		// A bias in the state has been detected by the innovation sequence check
		// Boost the process noise until the offset is removed
		delta_state_var *= _process_var_boost_gain;
	}

	_state_var += delta_state_var;
	constrainStateVar();

	if (dt > FLT_EPSILON && fabsf(_dt - dt) > 0.001f) {
		_signed_innov_test_ratio_lpf.setParameters(dt, _innov_sequence_monitnoring_time_constant);
		_dt = dt;
	}

	_status.bias_var = _state_var;
}

void BaroBiasEstimator::constrainStateVar()
{
	_state_var = math::constrain(_state_var, 1e-8f, _state_var_max);
}

void BaroBiasEstimator::fuseBias(const float measurement, const float measurement_var)
{
	const float innov_var = _state_var + measurement_var;
	const float innov = measurement - _state;
	const float K = _state_var / innov_var;
	const float innov_test_ratio = computeInnovTestRatio(innov, innov_var);

	if (isTestRatioPassing(innov_test_ratio)) {
		updateState(K, innov);
		updateStateCovariance(K);

	}

	updateOffsetDetection(innov, innov_test_ratio);

	_status = packStatus(innov, innov_var, innov_test_ratio);
}

inline float BaroBiasEstimator::computeInnovTestRatio(const float innov, const float innov_var) const
{
	return innov * innov / (_gate_size * _gate_size * innov_var);
}

inline bool BaroBiasEstimator::isTestRatioPassing(const float innov_test_ratio) const
{
	return innov_test_ratio < 1.f;
}

inline void BaroBiasEstimator::updateState(const float K, const float innov)
{
	_state = math::constrain(_state + K * innov, -_state_max, _state_max);
}

inline void BaroBiasEstimator::updateStateCovariance(const float K)
{
	_state_var -= K * _state_var;
	constrainStateVar();
}

inline void BaroBiasEstimator::updateOffsetDetection(const float innov, const float innov_test_ratio)
{
	const float signed_innov_test_ratio = matrix::sign(innov) * innov_test_ratio;
	_signed_innov_test_ratio_lpf.update(math::constrain(signed_innov_test_ratio, -1.f, 1.f));

	if (innov > 0.f) {
		_time_since_last_positive_innov = 0.f;
		_time_since_last_negative_innov += _dt;

	} else {
		_time_since_last_negative_innov = 0.f;
		_time_since_last_positive_innov += _dt;
	}
}

inline bool BaroBiasEstimator::isOffsetDetected() const
{
	// There is an offset in the estimate if the average of innovation is statistically too large
	// or if the sign of the innovation is constantly the same
	return fabsf(_signed_innov_test_ratio_lpf.getState()) > 0.2f
	       || (_time_since_last_positive_innov > _innov_sequence_monitnoring_time_constant)
	       || (_time_since_last_negative_innov > _innov_sequence_monitnoring_time_constant);
}

inline BaroBiasEstimator::status BaroBiasEstimator::packStatus(const float innov, const float innov_var,
		const float innov_test_ratio) const
{
	// Send back status for logging
	status ret{};
	ret.bias = _state;
	ret.bias_var = _state_var;
	ret.innov = innov;
	ret.innov_var = innov_var;
	ret.innov_test_ratio = innov_test_ratio;

	return ret;
}
