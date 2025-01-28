/****************************************************************************
 *
 *   Copyright (c) 2022 PX4. All rights reserved.
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
 * @file range_finder_consistency_check.cpp
 */

#include <aid_sources/range_finder/range_finder_consistency_check.hpp>
#include "ekf_derivation/generated/range_validation_filter_h.h"

using namespace matrix;

void RangeFinderConsistencyCheck::init(const float &z, const float &z_var, const float &dist_bottom,
				       const float &dist_bottom_var)
{
	// assume no correlation between z and terrain on initialization
	_P(RangeFilter::z.idx, RangeFilter::z.idx) = z_var;
	_P(RangeFilter::z.idx, RangeFilter::terrain.idx) = 0.f;
	_P(RangeFilter::terrain.idx, RangeFilter::z.idx) = 0.f;
	_P(RangeFilter::terrain.idx, RangeFilter::terrain.idx) = z_var + dist_bottom_var;

	_Ht = sym::RangeValidationFilterH<float>();

	_x(RangeFilter::z.idx) = z;
	_x(RangeFilter::terrain.idx) = z - dist_bottom;
	_initialized = true;
	_state = KinematicState::UNKNOWN;
	_test_ratio_lpf.reset(0);
	_t_since_first_sample = 0.f;
}

void RangeFinderConsistencyCheck::update(const float &z, const float &z_var, const float &vz, const float &vz_var,
		const float &dist_bottom, const float &dist_bottom_var, const uint64_t &time_us)
{
	const float dt = static_cast<float>(time_us - _time_last_update_us) * 1e-6f;

	if (dt > 1.f) {
		_time_last_update_us = time_us;
		_initialized = false;
		return;

	} else if (!_initialized) {
		init(z, z_var, dist_bottom, dist_bottom_var);
		_test_ratio_lpf.setParameters(dt, 1.f);
		return;
	}

	// prediction step
	_time_last_update_us = time_us;
	_x(RangeFilter::z.idx) -= dt * vz;
	_P(RangeFilter::z.idx, RangeFilter::z.idx) += dt * dt * vz_var + 0.001f;
	_P(RangeFilter::terrain.idx, RangeFilter::terrain.idx) += terrain_process_noise;

	// iterate through both measurements (z and dist_bottom)
	const Vector2f measurements{z, dist_bottom};

	for (int measurement_idx = 0; measurement_idx < 2; measurement_idx++) {

		float hz; // Jacobian in respect to z
		float ht; // Jacobian in respect to terrain
		float R;
		bool reject = false;

		if (measurement_idx == 0) {
			// direct state measurement
			hz = 1.f;
			ht = 0.f;
			R = z_var;

		} else if (measurement_idx == 1) {
			hz = _Ht(0, 0);
			ht = _Ht(0, 1);
			R = dist_bottom_var;
		}

		// residual
		const float measurement_pred = hz * _x(RangeFilter::z.idx) + ht * _x(RangeFilter::terrain.idx);
		const float y = measurements(measurement_idx) - measurement_pred;

		// innovation variance H * P * H^T + R
		const float S = hz * (hz * _P(RangeFilter::z.idx, RangeFilter::z.idx)
				      + ht * _P(RangeFilter::terrain.idx, RangeFilter::z.idx))
				+ ht * (hz * _P(RangeFilter::z.idx, RangeFilter::terrain.idx)
					+ ht * _P(RangeFilter::terrain.idx, RangeFilter::terrain.idx))
				+ R;

		// kalman gain K = P * H^T / S
		float Kz = (hz * _P(RangeFilter::z.idx, RangeFilter::z.idx)
			    + ht * _P(RangeFilter::z.idx, RangeFilter::terrain.idx)) / S;
		const float Kt = (hz * _P(RangeFilter::terrain.idx, RangeFilter::z.idx)
				  + ht * _P(RangeFilter::terrain.idx, RangeFilter::terrain.idx)) / S;

		if (measurement_idx == 0) {
			Kz = 1.f;

		} else if (measurement_idx == 1) {
			_innov = y;
			const float test_ratio = fminf(sq(y) / (sq(_gate) * S), 4.f); // limit to 4 to limit sensitivity to outliers
			_test_ratio_lpf.update(sign(_innov) * test_ratio);
			reject = test_ratio > 1.f;
		}

		if (!reject) {
			// update step
			_x(RangeFilter::z.idx)       += Kz * y;
			_x(RangeFilter::terrain.idx) += Kt * y;

			// covariance update with Joseph form:
			// P = (I - K H) P (I - K H)^T + K R K^T
			Matrix2f I;
			I.setIdentity();
			Matrix<float, 2, 1> K;
			K(0, 0) = Kz;
			K(1, 0) = Kt;
			Vector2f H0;
			H0(0) = hz;
			H0(1) = ht;
			Matrix2f IKH0 = I - K * H0.transpose();
			_P = IKH0 * _P * IKH0.transpose() + K * R * K.transpose();
		}
	}

	evaluateState(dt, vz, vz_var);
}

void RangeFinderConsistencyCheck::evaluateState(const float &dt, const float &vz, const float &vz_var)
{
	// start the consistency check after 1s
	if (_t_since_first_sample + dt > 1.0f) {
		_t_since_first_sample = 2.0f;

		if (abs(_test_ratio_lpf.getState()) < 1.f) {
			const bool vertical_motion = sq(vz) > fmaxf(vz_var, 0.1f);

			if (!horizontal_motion && vertical_motion) {
				_state = KinematicState::CONSISTENT;

			} else {
				_state = KinematicState::UNKNOWN;
			}

		} else {
			_t_since_first_sample = 0.f;
			_state = KinematicState::INCONSISTENT;
		}

	} else {
		_t_since_first_sample += dt;
	}
}

void RangeFinderConsistencyCheck::run(const float &z, const float &vz,
				      const matrix::SquareMatrix<float, estimator::State::size> &P,
				      const float &dist_bottom, const float &dist_bottom_var, uint64_t time_us)
{
	const float z_var = P(estimator::State::pos.idx + 2, estimator::State::pos.idx + 2);
	const float vz_var = P(estimator::State::vel.idx + 2, estimator::State::vel.idx + 2);

	if (!_initialized || current_posD_reset_count != _last_posD_reset_count) {
		_last_posD_reset_count = current_posD_reset_count;
		_initialized = false;
	}

	update(z, z_var, vz, vz_var, dist_bottom, dist_bottom_var, time_us);
}
