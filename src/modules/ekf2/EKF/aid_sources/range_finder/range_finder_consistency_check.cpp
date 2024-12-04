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

using namespace matrix;

void RangeFinderConsistencyCheck::init(float var_z, float var_terrain, float z, float dist_bottom)
{
	_R.setZero();
	_A.setIdentity();
	float p[4] = {var_z, 0.f, 0.f, var_terrain};
	_P = Matrix<float, 2, 2>(p);
	float h[4] = {1.f, 0.f, -1.f, 1.f};
	_H = Matrix<float, 2, 2>(h);
	_x(0) = z;
	_x(1) = z + dist_bottom;
	_initialized = true;
	_sample_count = 0;
	_state = KinematicState::UNKNOWN;
}

void RangeFinderConsistencyCheck::update(float z, float z_var, float vz, float vz_var, float dist_bottom,
		float dist_bottom_var, uint64_t time_us)
{
	const float dt = static_cast<float>(time_us - _time_last_update_us) * 1e-6f;

	if (_time_last_update_us == 0 || dt > 1.f) {
		_time_last_update_us = time_us;
		return;
	}

	if (_min_nr_of_samples == 0) {
		_min_nr_of_samples = (int)(1.f / dt);
	}

	_time_last_update_us = time_us;

	_R(0, 0) = z_var;
	_R(1, 1) = dist_bottom_var;

	SquareMatrix<float, 2> Q;
	const float process_noise = 0.001f;
	Q(0, 0) = dt * dt * vz_var + process_noise;
	Q(1, 1) = process_noise;
	_x(0) += dt * vz;
	_P = _A * _P * _A.transpose() + Q;

	const Vector2f measurements(z, dist_bottom);
	const Vector2f y = measurements - _H * _x;
	const Matrix2f S = _H * _P * _H.transpose() + _R;
	float test_ratio = fminf(abs(sq(y(1)) / (sq(_gate) * S(1, 1))), 2.f);

	Matrix2f K = _P * _H.transpose() * S.I();

	K(0, 0) = 1.f;
	K(0, 1) = 0.f;

	if (test_ratio > 1.f) {
		K(1, 0) = 0.f;
		K(1, 1) = 0.f;
	}

	_x = _x + K * y;
	_P = _P - K * _H * _P;
	_P = 0.5f * (_P + _P.transpose()); // Ensure symmetry

	_innov = y(1);
	_innov_var = S(1, 1);

	if (_sample_count++ > _min_nr_of_samples) {

		if (_test_ratio_lpf.update(test_ratio) < 1.f) {
			_state = KinematicState::CONSISTENT;

		} else {
			_sample_count = 0;
			_state = KinematicState::INCONSISTENT;
		}

	} else {
		_test_ratio_lpf.update(test_ratio);
	}
}

void RangeFinderConsistencyCheck::run(const float z, const float vz,
				      const matrix::SquareMatrix<float, estimator::State::size> P,
				      const float dist_bottom, const float dist_bottom_var, uint64_t time_us)
{
	const float z_var = P(estimator::State::pos.idx + 2, estimator::State::pos.idx + 2);
	const float vz_var = P(estimator::State::vel.idx + 2, estimator::State::vel.idx + 2);

	if (!_initialized || current_posD_reset_count != _last_posD_reset_count) {
		_last_posD_reset_count = current_posD_reset_count;
		const float terrain_var = P(estimator::State::terrain.idx, estimator::State::terrain.idx);
		init(z_var, terrain_var, z, dist_bottom);
	}

	update(z, z_var, vz, vz_var, dist_bottom, dist_bottom_var, time_us);
}
