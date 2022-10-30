/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

/*
 * @file KFxyzDecoupledStatic.h
 * Simple Kalman Filter for static target
 *
 * @author Jonas Perolini <jonas.perolini@epfl.ch>
 *
 */

#include "KFxyzDecoupledMoving.h"

namespace landing_target_estimator
{

void KFxyzDecoupledMoving::predictState(float dt, float acc)
{
	// Total ops: 12

	// Intermediate terms (1)
	const float tmp0 = 0.5f * dt * dt;

	_state(0, 0) = _state(0, 0) + _state(1, 0) * dt + _state(3, 0) * tmp0 - tmp0 * acc;
	_state(1, 0) = _state(1, 0) + _state(3, 0) * dt - acc * dt;
	// _state(2, 0) = _state(2, 0);
	// _state(3, 0) = _state(3, 0);
}

void KFxyzDecoupledMoving::predictCov(float dt)
{
	// Total ops: 60

	// Input arrays

	// Intermediate terms (10)
	const float tmp0 = dt * dt;
	const float tmp1 = 0.5f * tmp0;
	const float tmp2 = _covariance(3, 3) * tmp1;
	const float tmp3 = _covariance(0, 3) + _covariance(1, 3) * dt + tmp2;
	const float tmp4 = _covariance(0, 1) + _covariance(1, 1) * dt + _covariance(3, 1) * tmp1;
	const float tmp5 = _covariance(3, 1) * dt;
	const float tmp6 = _covariance(1, 1) + tmp5;
	const float tmp7 = _covariance(3, 3) * dt;
	const float tmp8 = _covariance(1, 3) + tmp7;
	const float tmp9 = 0.5f * _input_var * dt * dt * dt;

	// Output terms (1)
	_covariance(0, 0) = _covariance(0, 0) + _covariance(1, 0) * dt + _covariance(3,
			    0) * tmp1 + 0.25f * _input_var * dt * dt * dt * dt + tmp1 * tmp3 + tmp4 * dt;
	_covariance(1, 0) = _covariance(1, 0) + _covariance(3, 0) * dt + tmp1 * tmp8 + tmp6 * dt + tmp9;
	_covariance(2, 0) = _covariance(2, 0) + _covariance(2, 1) * dt + _covariance(2, 3) * tmp1;
	_covariance(3, 0) = _covariance(3, 0) + tmp2 + tmp5;

	_covariance(1, 1) = _input_var * tmp0 + tmp6 + tmp8 * dt;
	_covariance(2, 1) = _covariance(2, 1) + _covariance(2, 3) * dt;
	_covariance(3, 1) = _covariance(3, 1) + tmp7;

	_covariance(2, 2) = _bias_var + _covariance(2, 2);
	// _covariance(3, 2) = _covariance(3, 2);

	_covariance(3, 3) = _acc_var + _covariance(3, 3);

	// Symmetric matrix:
	_covariance(0, 1) = _covariance(1, 0);
	_covariance(0, 2) = _covariance(2, 0);
	_covariance(1, 2) = _covariance(2, 1);
	_covariance(0, 3) = _covariance(3, 0);
	_covariance(1, 3) = _covariance(3, 1);
	// _covariance(2, 3) = _covariance(3, 2);
}


bool KFxyzDecoupledMoving::update()
{
	// outlier rejection
	if (_innov_cov  <= 0.000001f) {
		return false;
	}

	float beta = _innov / _innov_cov * _innov;

	// 5% false alarm probability
	if (beta > 3.84f) {
		return false;
	}

	matrix::Matrix<float, 4, 1> kalmanGain = _covariance * _meas_matrix.transpose() / _innov_cov;

	_state = _state + kalmanGain * _innov;

	_covariance = _covariance - kalmanGain * _meas_matrix * _covariance;

	return true;
}

void KFxyzDecoupledMoving::setH(matrix::Vector<float, 12> h_meas)
{
	// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]

	// For this filter: [rx, r_dotx, bx, atx]

	_meas_matrix(0, 0) = h_meas(0);
	_meas_matrix(1, 0) = h_meas(3);
	_meas_matrix(2, 0) = h_meas(6);
	_meas_matrix(3, 0) = h_meas(9);

}

void KFxyzDecoupledMoving::syncState(float dt, float acc)
{
	const float _tmp0 = 0.5f * dt * dt;
	const float _tmp1 = _state(1, 0);
	const float _tmp2 = _state(3, 0);

	_sync_state(0, 0) = _state(0, 0) + _state(3, 0) * _tmp0 - _tmp0 * acc - _tmp1 * dt;
	_sync_state(1, 0) = _tmp1 - _tmp2 * dt + acc * dt;
	_sync_state(2, 0) = _state(2, 0);
	_sync_state(3, 0) = _tmp2;
}

float KFxyzDecoupledMoving::computeInnovCov(float meas_unc)
{
	// Total ops: 36

	_innov_cov =
		_meas_matrix(0, 0) *
		(_covariance(0, 0) * _meas_matrix(0, 0) + _covariance(1, 0) * _meas_matrix(0, 1) +
		 _covariance(2, 0) * _meas_matrix(0, 2) + _covariance(3, 0) * _meas_matrix(0, 3)) +
		_meas_matrix(0, 1) *
		(_covariance(0, 1) * _meas_matrix(0, 0) + _covariance(1, 1) * _meas_matrix(0, 1) +
		 _covariance(2, 1) * _meas_matrix(0, 2) + _covariance(3, 1) * _meas_matrix(0, 3)) +
		_meas_matrix(0, 2) *
		(_covariance(0, 2) * _meas_matrix(0, 0) + _covariance(1, 2) * _meas_matrix(0, 1) +
		 _covariance(2, 2) * _meas_matrix(0, 2) + _covariance(3, 2) * _meas_matrix(0, 3)) +
		_meas_matrix(0, 3) *
		(_covariance(0, 3) * _meas_matrix(0, 0) + _covariance(1, 3) * _meas_matrix(0, 1) +
		 _covariance(2, 3) * _meas_matrix(0, 2) + _covariance(3, 3) * _meas_matrix(0, 3)) +
		meas_unc;

	return _innov_cov;
}

float KFxyzDecoupledMoving::computeInnov(float meas)
{
	/* z - H*x */
	_innov = meas - (_meas_matrix * _sync_state)(0, 0);
	return _innov;
}

} // namespace landing_target_estimator
