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
#include "ekf_derivation/generated/range_validation_filter_P_init.h"

using namespace matrix;

void RangeFinderConsistencyCheck::init(const float z, const float z_var, const float dist_bottom,
				       const float dist_bottom_var)
{
	_P = sym::RangeValidationFilterPInit(z_var, dist_bottom_var);
	_Ht = sym::RangeValidationFilterH<float>();

	_x(RangeFilter::z.idx) = z;
	_x(RangeFilter::terrain.idx) = z - dist_bottom;
	_initialized = true;
	_test_ratio_lpf.reset(0.f);
	_t_since_first_sample = 0.f;
	_state = KinematicState::kUnknown;
}

void RangeFinderConsistencyCheck::update(const float z, const float z_var, const float vz, const float vz_var,
		const float dist_bottom, const float dist_bottom_var, const uint64_t time_us)
{
	const float dt = static_cast<float>(time_us - _time_last_update_us) * 1e-6f;

	if (dt > _kTestRatioLpfTimeConstant || _landed) {
		_time_last_update_us = time_us;
		_initialized = false;
		return;

	} else if (!_initialized) {
		init(z, z_var, dist_bottom, dist_bottom_var);
		return;
	}

	// prediction step
	_time_last_update_us = time_us;
	_x(RangeFilter::z.idx) -= dt * vz;
	_P(RangeFilter::z.idx, RangeFilter::z.idx) += dt * dt * vz_var;
	_P(RangeFilter::terrain.idx, RangeFilter::terrain.idx) += _terrain_process_noise;

	// iterate through both measurements (z and dist_bottom)
	const Vector2f measurements{z, dist_bottom};

	for (int measurement_idx = 0; measurement_idx < 2; measurement_idx++) {

		// dist_bottom
		Vector2f H = _Ht;
		float R = dist_bottom_var;

		// z, direct state measurement
		if (measurement_idx == 0) {
			H(RangeFilter::z.idx) = 1.f;
			H(RangeFilter::terrain.idx) = 0.f;
			R = z_var;

		}

		// residual
		const float measurement_pred = H * _x;
		const float y = measurements(measurement_idx) - measurement_pred;

		// for H as col-vector:
		// innovation variance S = H^T * P * H + R
		// kalman gain K = P * H / S
		const float S = (H.transpose() * _P * H + R)(0, 0);
		Vector2f K = (_P * H / S);

		if (measurement_idx == 0) {
			K(RangeFilter::z.idx) = 1.f;

		} else if (measurement_idx == 1) {
			_innov = y;
			const float test_ratio = fminf(sq(y) / (sq(_gate) * S), 4.f); // limit to 4 to limit sensitivity to outliers
			_test_ratio_lpf.update(test_ratio, dt);
		}

		// update step
		_x(RangeFilter::z.idx)       += K(RangeFilter::z.idx) * y;
		_x(RangeFilter::terrain.idx) += K(RangeFilter::terrain.idx) * y;

		// covariance update with Joseph form:
		// P = (I - K H) P (I - K H)^T + K R K^T
		Matrix2f I;
		I.setIdentity();
		Matrix2f IKH = I - K.multiplyByTranspose(H);
		_P = IKH * _P * IKH.transpose() + (K * R).multiplyByTranspose(K);
	}

	evaluateState(dt, vz, vz_var);
}

void RangeFinderConsistencyCheck::evaluateState(const float dt, const float vz, const float vz_var)
{
	// let the filter settle before starting the consistency check
	if (_t_since_first_sample > _kTestRatioLpfTimeConstant) {
		if (fabsf(_test_ratio_lpf.getState()) < 1.f) {
			const bool vertical_motion = sq(vz) > fmaxf(vz_var, 0.1f);

			if (!_horizontal_motion && vertical_motion) {
				_state = KinematicState::kConsistent;

			} else if (_state == KinematicState::kConsistent || vertical_motion) {
				_state = KinematicState::kUnknown;
			}

		} else {
			_t_since_first_sample = 0.f;
			_state = KinematicState::kInconsistent;
		}

	} else {
		_t_since_first_sample += dt;
	}
}

void RangeFinderConsistencyCheck::run(const float z, const float z_var, const float vz, const float vz_var,
				      const float dist_bottom, const float dist_bottom_var, const uint64_t time_us,
				      const uint8_t current_posD_reset_count
				     )
{
	if (!_initialized || current_posD_reset_count != _last_posD_reset_count) {
		_last_posD_reset_count = current_posD_reset_count;
		_initialized = false;
	}

	update(z, z_var, vz, vz_var, dist_bottom, dist_bottom_var, time_us);
}

void RangeFinderConsistencyCheck::setIsLanded(const bool landed)
{
	if (landed) {
		_state = KinematicState::kUnknown;
	}

	_landed = landed;
}

void RangeFinderConsistencyCheck::reset()
{
	if (_initialized && _state == KinematicState::kConsistent) {
		_state = KinematicState::kUnknown;
	}

	_initialized = false;
}

void RangeFinderConsistencyCheck::setHorizontalMotion(const bool horizontal_motion)
{
	if (!horizontal_motion && getHorizontalMotion()) {
		reset();
	}

	_horizontal_motion = horizontal_motion;
}
