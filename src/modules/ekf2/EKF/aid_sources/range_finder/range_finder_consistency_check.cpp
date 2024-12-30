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
#include "ekf_derivation/generated/range_validation_filter.h"

using namespace matrix;

void RangeFinderConsistencyCheck::init(const float &z, const float &z_var, const float &dist_bottom,
				       const float &dist_bottom_var)
{
	float p[4] = {z_var, z_var, z_var, z_var + dist_bottom_var};
	_P = SquareMatrix<float, RangeFilter::size>(p);
	_H = sym::RangeValidationFilter<float>();
	_x(RangeFilter::z.idx) = z;
	_x(RangeFilter::terrain.idx) = z - dist_bottom;
	_initialized = true;
	_state = KinematicState::UNKNOWN;
	_test_ratio_lpf.reset(2.f);
	_t_since_first_sample = 0.f;
	_test_ratio_lpf.setAlpha(0.2f);
}

void RangeFinderConsistencyCheck::update(const float &z, const float &z_var, const float &vz, const float &vz_var,
		const float &dist_bottom, const float &dist_bottom_var, const uint64_t &time_us)
{
	const float dt = static_cast<float>(time_us - _time_last_update_us) * 1e-6f;

	if (_time_last_update_us == 0 || dt > 1.f) {
		_time_last_update_us = time_us;
		init(z, z_var, dist_bottom, dist_bottom_var);
		return;
	}

	_time_last_update_us = time_us;

	_x(RangeFilter::z.idx) -= dt * vz;
	_P(0, 0) += dt * dt * vz_var + 0.001f;
	_P(1, 1) += terrain_process_noise;

	const Vector2f measurements(z, dist_bottom);

	Vector2f Kv{1.f, 0.f};
	Vector2f test_ratio{0.f, 0.f};
	Vector2f R{z_var, dist_bottom_var};
	Vector2f y;

	for (int i = 0; i < 2 ; i++) {
		y = measurements - _H * _x;
		Vector2f H = _H.row(i);
		_innov_var = (H.transpose() * _P * H + R(i))(0, 0);
		Kv(RangeFilter::terrain.idx) = _P(RangeFilter::terrain.idx, i) / _innov_var;

		Vector2f PH = _P.row(i);

		for (int u = 0; u < RangeFilter::size; u++) {
			for (int v = 0; v < RangeFilter::size; v++) {
				_P(u, v) -= Kv(u) * PH(v);
			}
		}

		PH = _P.col(i);

		for (int u = 0; u < RangeFilter::size; u++) {
			for (int v = 0; v <= u; v++) {
				_P(u, v) = _P(u, v) - PH(u) * Kv(v) + Kv(u) * R(i) * Kv(v);
				_P(v, u) = _P(u, v);
			}
		}

		test_ratio(i) = fminf(sq(y(i)) / sq(_gate), 4.f);

		if (i == (int)RangeFilter::z.idx && test_ratio(1) > 1.f) {
			Kv(1) = 0.f;
		}

		_x = _x + Kv * y;
	}

	_innov = y(RangeFilter::terrain.idx);
	_test_ratio_lpf.update(sign(_innov) * test_ratio(1));

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
		init(z, z_var, dist_bottom, dist_bottom_var);
	}

	update(z, z_var, vz, vz_var, dist_bottom, dist_bottom_var, time_us);
}
