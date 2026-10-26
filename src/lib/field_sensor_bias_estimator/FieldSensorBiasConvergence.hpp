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

/**
 * @file FieldSensorBiasConvergence.hpp
 *
 * Decides whether a FieldSensorBiasEstimator estimate has converged.
 */

#pragma once

#include <matrix/matrix/math.hpp>

class FieldSensorBiasConvergence
{
public:
	void reset() { *this = {}; }

	/**
	 * @param gyro same gyro sample as passed to FieldSensorBiasEstimator::updateEstimate()
	 * @param field same field sample as passed to FieldSensorBiasEstimator::updateEstimate()
	 * @param bias estimate after the update
	 * @param learning_gain learning gain of the estimator
	 * @param dt time in seconds since the last update
	 */
	void update(const matrix::Vector3f &gyro, const matrix::Vector3f &field, const matrix::Vector3f &bias,
		    float learning_gain, float dt)
	{
		// the estimator error decays as exp(-excitation) along the directions the vehicle was rotated about
		const float gain = learning_gain * dt * dt;
		const float gyro_norm_sq = gyro.norm_squared();

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				_excitation(i, j) += gain * ((i == j ? gyro_norm_sq : 0.f) - gyro(i) * gyro(j));
			}
		}

		_window_rotation += sqrtf(gyro_norm_sq) * dt;

		const float field_norm = (field - bias).norm();
		_field_norm_sum += field_norm;
		_field_norm_sum_sq += field_norm * field_norm;
		_samples++;

		_window_time += dt;

		if (_window_time >= kWindowTime) {
			for (int axis = 0; axis < 3; axis++) {
				_observed[axis] = isObserved(axis);
			}

			const bool was_valid = _valid;

			if (_window_rotation > kMinWindowRotation) {
				const float mean = _field_norm_sum / _samples;
				const float field_norm_std = sqrtf(fmaxf(_field_norm_sum_sq / _samples - mean * mean, 0.f));

				const bool quiet = !(bias - _window_start_bias).longerThan(fmaxf(kQuietAbs, kQuietRel * bias.norm()));
				const bool consistent = field_norm_std < kMaxFieldNormStd;
				_valid = bias.isAllFinite() && bias.longerThan(kMinBias) && !bias.longerThan(kMaxBias)
					 && quiet && consistent && _observed[0] && _observed[1];

				if (_valid) {
					for (int axis = 0; axis < 3; axis++) {
						_bias(axis) = _observed[axis] ? bias(axis) : 0.f;
					}
				}

				_valid_time = (_valid && was_valid) ? fminf(_valid_time + _window_time, kStableTime) : 0.f;
			}

			_window_start_bias = bias;
			_window_rotation = 0.f;
			_window_time = 0.f;
			_field_norm_sum = 0.f;
			_field_norm_sum_sq = 0.f;
			_samples = 0;
		}
	}

	bool valid() const { return _valid; }
	bool stable() const { return _valid && (_valid_time >= kStableTime); }
	bool observed(int axis) const { return _observed[axis]; }

	/** @return estimate at the end of the last window that passed all checks, unobserved axes are 0 */
	const matrix::Vector3f &bias() const { return _bias; }

private:
	// an axis is observed once the remaining error along it is below exp(-kMinExcitation), ~5%
	bool isObserved(int axis) const
	{
		const int a = (axis + 1) % 3;
		const int b = (axis + 2) % 3;
		const float j_ii = _excitation(axis, axis) + kRegularization;
		const float j_aa = _excitation(a, a) + kRegularization;
		const float j_bb = _excitation(b, b) + kRegularization;
		const float j_ia = _excitation(axis, a);
		const float j_ib = _excitation(axis, b);
		const float j_ab = _excitation(a, b);

		const float minor = j_aa * j_bb - j_ab * j_ab;
		const float det = j_ii * minor - j_ia * (j_ia * j_bb - j_ab * j_ib) + j_ib * (j_ia * j_ab - j_aa * j_ib);
		return det > kMinExcitation * minor;
	}

	static constexpr float kMinExcitation{3.f};
	static constexpr float kRegularization{1e-3f};
	static constexpr float kWindowTime{2.f};		///< [s]
	static constexpr float kMinWindowRotation{0.5f};	///< [rad]
	static constexpr float kQuietAbs{0.015f};		///< [Gauss]
	static constexpr float kQuietRel{0.05f};
	static constexpr float kMaxFieldNormStd{0.025f};	///< [Gauss]
	static constexpr float kMinBias{0.04f};			///< [Gauss] smaller estimates are within the estimator accuracy
	static constexpr float kMaxBias{1.f};			///< [Gauss]
	static constexpr float kStableTime{5.f};		///< [s]

	matrix::SquareMatrix3f _excitation{};
	matrix::Vector3f _window_start_bias{};
	matrix::Vector3f _bias{};
	float _window_rotation{0.f};
	float _field_norm_sum{0.f};
	float _field_norm_sum_sq{0.f};
	int _samples{0};
	float _window_time{0.f};
	float _valid_time{0.f};
	bool _observed[3] {};
	bool _valid{false};
};
