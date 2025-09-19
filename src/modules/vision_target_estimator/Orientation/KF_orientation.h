/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file KF_orientation.h
 * @brief Filter to estimate the orientation of static and moving targets. State: [yaw, yaw_rate]
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <matrix/Vector.hpp>

#pragma once

namespace vision_target_estimator
{

namespace State
{
static constexpr uint8_t yaw{0};
static constexpr uint8_t yaw_rate{1};
static constexpr uint8_t size{2};
};

class KF_orientation
{
public:

	KF_orientation() {};
	~KF_orientation() {};

	void predictState(float dt);
	void predictCov(float dt);

	bool update();

	// Backwards state prediciton
	void syncState(float dt);
	void set_H(const matrix::Vector<float, State::size> &h_meas) {_meas_matrix_row_vect = h_meas;}
	void set_state(const matrix::Vector<float, State::size> &state) {_state = state;}
	void set_state_covariance(const matrix::Vector<float, State::size> &var)
	{
		const matrix::SquareMatrix<float, State::size> var_mat = diag(var);
		_state_covariance = var_mat;
	};

	matrix::Vector<float, State::size> get_state() { return _state;}
	matrix::Vector<float, State::size> get_state_covariance()
	{
		const matrix::SquareMatrix<float, State::size> var_mat = _state_covariance;
		return var_mat.diag();
	};

	float computeInnovCov(float measUnc);
	float computeInnov(float meas);

	void set_nis_threshold(float nis_threshold) { _nis_threshold = nis_threshold; };
	float get_test_ratio() {if (fabsf(_innov_cov) < 1e-6f) {return -1.f;} else {return _innov / _innov_cov * _innov;} };

private:

	matrix::SquareMatrix<float, State::size> getPhi(float dt)
	{

		float data[State::size * State::size] = {
			1, dt,
			0, 1
		};

		return matrix::SquareMatrix<float, State::size>(data);
	}

	matrix::Vector<float, State::size> _state;

	matrix::Vector<float, State::size> _sync_state;

	matrix::Vector<float, State::size> _meas_matrix_row_vect;

	matrix::Matrix<float, State::size, State::size> _state_covariance;

	float _innov{0.0f}; // residual of last measurement update

	float _innov_cov{0.0f}; // innovation covariance of last measurement update

	float _nis_threshold{0.0f}; // Normalized innovation squared test threshold

};
} // namespace vision_target_estimator
