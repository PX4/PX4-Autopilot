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
	_state_var += _process_var * dt * dt;
	constrainStateVar();

	if (dt > FLT_EPSILON && fabsf(_dt - dt) > 0.001f) {
		_signed_innov_test_ratio_lpf.setParameters(dt, _lpf_time_constant);
		_dt = dt;
	}
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

	if (isLargeOffsetDetected()) {
		// A bias in the state has been detected by the innovation
		// sequence check.
		bumpStateVariance();
	}

	const float signed_innov_test_ratio = matrix::sign(innov) * innov_test_ratio;
	_signed_innov_test_ratio_lpf.update(math::constrain(signed_innov_test_ratio, -1.f, 1.f));

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

inline bool BaroBiasEstimator::isLargeOffsetDetected() const
{
	return fabsf(_signed_innov_test_ratio_lpf.getState()) > 0.2f;
}

inline void BaroBiasEstimator::bumpStateVariance()
{
	_state_var += _process_var_boost_gain * _process_var * _dt * _dt;
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
