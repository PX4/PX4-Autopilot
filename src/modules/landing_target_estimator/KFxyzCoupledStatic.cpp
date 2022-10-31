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
 * @file KFxyzCoupledStatic.h
 * Simple Kalman Filter for static target
 *
 * @author Jonas Perolini <jonas.perolini@epfl.ch>
 *
 */

#include "KFxyzCoupledStatic.h"

namespace landing_target_estimator
{

void KFxyzCoupledStatic::predictState(float dt, matrix::Vector<float, 3> acc)
{
	// Total ops: 20

	const float _tmp0 = 0.5f * dt * dt;

	_state(0, 0) = _state(0, 0) + _state(3, 0) * dt - _tmp0 * acc(0);
	_state(1, 0) = _state(1, 0) + _state(4, 0) * dt - _tmp0 * acc(1);
	_state(2, 0) = _state(2, 0) + _state(5, 0) * dt - _tmp0 * acc(2);
	_state(3, 0) = _state(3, 0) - acc(0) * dt;
	_state(4, 0) = _state(4, 0) - acc(1) * dt;
	_state(5, 0) = _state(5, 0) - acc(2) * dt;
	// _state(6, 0) = _state(6, 0);
	// _state(7, 0) = _state(7, 0);
	// _state(8, 0) = _state(8, 0);
}

void KFxyzCoupledStatic::predictCov(float dt)
{

	// Total ops: 170

	// Input arrays

	// Intermediate terms (30)
	const float _tmp0 = 0.25f * dt * dt * dt * dt;
	const float _tmp1 = _covariance(3, 3) * dt;
	const float _tmp2 = _covariance(0, 3) + _tmp1;
	const float _tmp3 = _covariance(4, 3) * dt;
	const float _tmp4 = _covariance(1, 3) + _tmp3;
	const float _tmp5 = _covariance(5, 3) * dt;
	const float _tmp6 = _covariance(2, 3) + _tmp5;
	const float _tmp7 = 0.5f * dt * dt * dt;
	const float _tmp8 = _input_var(0, 0) * _tmp7;
	const float _tmp9 = _input_var(1, 0) * _tmp7;
	const float _tmp10 = _input_var(2, 0) * _tmp7;
	const float _tmp11 = _covariance(3, 4) * dt;
	// const float _tmp12 = _covariance(0, 4) + _tmp11;
	const float _tmp13 = _covariance(4, 4) * dt;
	const float _tmp14 = _covariance(1, 4) + _tmp13;
	const float _tmp15 = _covariance(5, 4) * dt;
	const float _tmp16 = _covariance(2, 4) + _tmp15;
	const float _tmp17 = _input_var(0, 1) * _tmp7;
	const float _tmp18 = _input_var(1, 1) * _tmp7;
	const float _tmp19 = _input_var(2, 1) * _tmp7;
	const float _tmp20 = _covariance(3, 5) * dt;
	// const float _tmp21 = _covariance(0, 5) + _tmp20;
	const float _tmp22 = _covariance(4, 5) * dt;
	// const float _tmp23 = _covariance(1, 5) + _tmp22;
	const float _tmp24 = _covariance(5, 5) * dt;
	const float _tmp25 = _covariance(2, 5) + _tmp24;
	const float _tmp26 = _input_var(0, 2) * _tmp7;
	const float _tmp27 = _input_var(1, 2) * _tmp7;
	const float _tmp28 = _input_var(2, 2) * _tmp7;
	const float _tmp29 = dt * dt;

	_covariance(0, 0) = _covariance(0, 0) + _covariance(3, 0) * dt + _input_var(0, 0) * _tmp0 + _tmp2 * dt;
	_covariance(1, 0) = _covariance(1, 0) + _covariance(4, 0) * dt + _input_var(1, 0) * _tmp0 + _tmp4 * dt;
	_covariance(2, 0) = _covariance(2, 0) + _covariance(5, 0) * dt + _input_var(2, 0) * _tmp0 + _tmp6 * dt;
	_covariance(3, 0) = _covariance(3, 0) + _tmp1 + _tmp8;
	_covariance(4, 0) = _covariance(4, 0) + _tmp3 + _tmp9;
	_covariance(5, 0) = _covariance(5, 0) + _tmp10 + _tmp5;
	_covariance(6, 0) = _covariance(6, 0) + _covariance(6, 3) * dt;
	_covariance(7, 0) = _covariance(7, 0) + _covariance(7, 3) * dt;
	_covariance(8, 0) = _covariance(8, 0) + _covariance(8, 3) * dt;

	_covariance(1, 1) = _covariance(1, 1) + _covariance(4, 1) * dt + _input_var(1, 1) * _tmp0 + _tmp14 * dt;
	_covariance(2, 1) = _covariance(2, 1) + _covariance(5, 1) * dt + _input_var(2, 1) * _tmp0 + _tmp16 * dt;
	_covariance(3, 1) = _covariance(3, 1) + _tmp11 + _tmp17;
	_covariance(4, 1) = _covariance(4, 1) + _tmp13 + _tmp18;
	_covariance(5, 1) = _covariance(5, 1) + _tmp15 + _tmp19;
	_covariance(6, 1) = _covariance(6, 1) + _covariance(6, 4) * dt;
	_covariance(7, 1) = _covariance(7, 1) + _covariance(7, 4) * dt;
	_covariance(8, 1) = _covariance(8, 1) + _covariance(8, 4) * dt;

	_covariance(2, 2) = _bias_var(0, 0) + _covariance(2, 2) + _covariance(5, 2) * dt + _input_var(2,
			    2) * _tmp0 + _tmp25 * dt;
	_covariance(3, 2) = _covariance(3, 2) + _tmp20 + _tmp26;
	_covariance(4, 2) = _covariance(4, 2) + _tmp22 + _tmp27;
	_covariance(5, 2) = _covariance(5, 2) + _tmp24 + _tmp28;
	_covariance(6, 2) = _covariance(6, 2) + _covariance(6, 5) * dt;
	_covariance(7, 2) = _covariance(7, 2) + _covariance(7, 5) * dt;
	_covariance(8, 2) = _covariance(8, 2) + _covariance(8, 5) * dt;

	_covariance(3, 3) = _covariance(3, 3) + _input_var(0, 0) * _tmp29;
	_covariance(4, 3) = _covariance(4, 3) + _input_var(1, 0) * _tmp29;
	_covariance(5, 3) = _covariance(5, 3) + _input_var(2, 0) * _tmp29;
	// _covariance(6, 3) = _covariance(6, 3);
	// _covariance(7, 3) = _covariance(7, 3);
	// _covariance(8, 3) = _covariance(8, 3);

	_covariance(4, 4) = _covariance(4, 4) + _input_var(1, 1) * _tmp29;
	_covariance(5, 4) = _covariance(5, 4) + _input_var(2, 1) * _tmp29;
	// _covariance(6, 4) = _covariance(6, 4);
	// _covariance(7, 4) = _covariance(7, 4);
	// _covariance(8, 4) = _covariance(8, 4);

	_covariance(5, 5) = _bias_var(1, 1) + _covariance(5, 5) + _input_var(2, 2) * _tmp29;
	// _covariance(6, 5) = _covariance(6, 5);
	// _covariance(7, 5) = _covariance(7, 5);
	// _covariance(8, 5) = _covariance(8, 5);

	// _covariance(6, 6) = _covariance(6, 6);
	// _covariance(7, 6) = _covariance(7, 6);
	// _covariance(8, 6) = _covariance(8, 6);

	// _covariance(7, 7) = _covariance(7, 7);
	// _covariance(8, 7) = _covariance(8, 7);

	_covariance(8, 8) = _bias_var(2, 2) + _covariance(8, 8);

	// Symmetric matrix:
	_covariance(0, 1) = _covariance(1, 0);

	_covariance(0, 2) = _covariance(2, 0);
	_covariance(1, 2) = _covariance(2, 1);

	_covariance(0, 3) = _covariance(3, 0);
	_covariance(1, 3) = _covariance(3, 1);
	_covariance(2, 3) = _covariance(3, 2);

	_covariance(0, 4) = _covariance(4, 0);
	_covariance(1, 4) = _covariance(4, 1);
	_covariance(2, 4) = _covariance(4, 2);
	_covariance(3, 4) = _covariance(4, 3);

	_covariance(0, 5) = _covariance(5, 0);
	_covariance(1, 5) = _covariance(5, 1);
	_covariance(2, 5) = _covariance(5, 2);
	_covariance(3, 5) = _covariance(5, 3);
	_covariance(4, 5) = _covariance(5, 4);

	_covariance(0, 6) = _covariance(6, 0);
	_covariance(1, 6) = _covariance(6, 1);
	_covariance(2, 6) = _covariance(6, 2);
	// _covariance(3, 6) = _covariance(6, 3);
	// _covariance(4, 6) = _covariance(6, 4);
	// _covariance(5, 6) = _covariance(6, 5);

	_covariance(0, 7) = _covariance(7, 0);
	_covariance(1, 7) = _covariance(7, 1);
	_covariance(2, 7) = _covariance(7, 2);
	// _covariance(3, 7) = _covariance(7, 3);
	// _covariance(4, 7) = _covariance(7, 4);
	// _covariance(5, 7) = _covariance(7, 5);
	// _covariance(6, 7) = _covariance(7, 6);

	_covariance(0, 8) = _covariance(8, 0);
	_covariance(1, 8) = _covariance(8, 1);
	_covariance(2, 8) = _covariance(8, 2);
	// _covariance(3, 8) = _covariance(8, 3);
	// _covariance(4, 8) = _covariance(8, 4);
	// _covariance(5, 8) = _covariance(8, 5);
	// _covariance(6, 8) = _covariance(8, 6);
	// _covariance(7, 8) = _covariance(8, 7);
}


bool KFxyzCoupledStatic::update()
{
	// Avoid zero-division
	if (_innov_cov  <= 0.000001f && _innov_cov  >= -0.000001f) {
		return false;
	}

	float beta = _innov / _innov_cov * _innov;

	// 5% false alarm probability
	if (beta > 3.84f) {
		return false;
	}

	matrix::Matrix<float, 9, 1> kalmanGain = _covariance * _meas_matrix.transpose() / _innov_cov;

	_state = _state + kalmanGain * _innov;
	_covariance = _covariance - kalmanGain * _meas_matrix * _covariance;

	return true;
}

void KFxyzCoupledStatic::setH(matrix::Vector<float, 12> h_meas)
{
	// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]

	// state = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz]

	_meas_matrix(0, 0) = h_meas(0);
	_meas_matrix(0, 1) = h_meas(1);
	_meas_matrix(0, 2) = h_meas(2);
	_meas_matrix(0, 3) = h_meas(3);
	_meas_matrix(0, 4) = h_meas(4);
	_meas_matrix(0, 5) = h_meas(5);
	_meas_matrix(0, 6) = h_meas(6);
	_meas_matrix(0, 7) = h_meas(7);
	_meas_matrix(0, 8) = h_meas(8);
}

void KFxyzCoupledStatic::syncState(float dt, matrix::Vector<float, 3> acc)
{
	// Total ops: 20
	const float _tmp0 = 0.5f * dt * dt;

	_sync_state(0, 0) = _state(0, 0) - _state(3, 0) * dt - _tmp0 * acc(0);
	_sync_state(1, 0) = _state(1, 0) - _state(4, 0) * dt - _tmp0 * acc(1);
	_sync_state(2, 0) = _state(2, 0) - _state(5, 0) * dt - _tmp0 * acc(2);
	_sync_state(3, 0) = _state(3, 0) + acc(0) * dt;
	_sync_state(4, 0) = _state(4, 0) + acc(1) * dt;
	_sync_state(5, 0) = _state(5, 0) + acc(2) * dt;
	_sync_state(6, 0) = _state(6, 0);
	_sync_state(7, 0) = _state(7, 0);
	_sync_state(8, 0) = _state(8, 0);
}

float KFxyzCoupledStatic::computeInnovCov(float meas_unc)
{
	// Total ops: 171

	_innov_cov =
		_meas_matrix(0, 0) *
		(_covariance(0, 0) * _meas_matrix(0, 0) + _covariance(1, 0) * _meas_matrix(0, 1) +
		 _covariance(2, 0) * _meas_matrix(0, 2) + _covariance(3, 0) * _meas_matrix(0, 3) +
		 _covariance(4, 0) * _meas_matrix(0, 4) + _covariance(5, 0) * _meas_matrix(0, 5) +
		 _covariance(6, 0) * _meas_matrix(0, 6) + _covariance(7, 0) * _meas_matrix(0, 7) +
		 _covariance(8, 0) * _meas_matrix(0, 8)) +
		_meas_matrix(0, 1) *
		(_covariance(0, 1) * _meas_matrix(0, 0) + _covariance(1, 1) * _meas_matrix(0, 1) +
		 _covariance(2, 1) * _meas_matrix(0, 2) + _covariance(3, 1) * _meas_matrix(0, 3) +
		 _covariance(4, 1) * _meas_matrix(0, 4) + _covariance(5, 1) * _meas_matrix(0, 5) +
		 _covariance(6, 1) * _meas_matrix(0, 6) + _covariance(7, 1) * _meas_matrix(0, 7) +
		 _covariance(8, 1) * _meas_matrix(0, 8)) +
		_meas_matrix(0, 2) *
		(_covariance(0, 2) * _meas_matrix(0, 0) + _covariance(1, 2) * _meas_matrix(0, 1) +
		 _covariance(2, 2) * _meas_matrix(0, 2) + _covariance(3, 2) * _meas_matrix(0, 3) +
		 _covariance(4, 2) * _meas_matrix(0, 4) + _covariance(5, 2) * _meas_matrix(0, 5) +
		 _covariance(6, 2) * _meas_matrix(0, 6) + _covariance(7, 2) * _meas_matrix(0, 7) +
		 _covariance(8, 2) * _meas_matrix(0, 8)) +
		_meas_matrix(0, 3) *
		(_covariance(0, 3) * _meas_matrix(0, 0) + _covariance(1, 3) * _meas_matrix(0, 1) +
		 _covariance(2, 3) * _meas_matrix(0, 2) + _covariance(3, 3) * _meas_matrix(0, 3) +
		 _covariance(4, 3) * _meas_matrix(0, 4) + _covariance(5, 3) * _meas_matrix(0, 5) +
		 _covariance(6, 3) * _meas_matrix(0, 6) + _covariance(7, 3) * _meas_matrix(0, 7) +
		 _covariance(8, 3) * _meas_matrix(0, 8)) +
		_meas_matrix(0, 4) *
		(_covariance(0, 4) * _meas_matrix(0, 0) + _covariance(1, 4) * _meas_matrix(0, 1) +
		 _covariance(2, 4) * _meas_matrix(0, 2) + _covariance(3, 4) * _meas_matrix(0, 3) +
		 _covariance(4, 4) * _meas_matrix(0, 4) + _covariance(5, 4) * _meas_matrix(0, 5) +
		 _covariance(6, 4) * _meas_matrix(0, 6) + _covariance(7, 4) * _meas_matrix(0, 7) +
		 _covariance(8, 4) * _meas_matrix(0, 8)) +
		_meas_matrix(0, 5) *
		(_covariance(0, 5) * _meas_matrix(0, 0) + _covariance(1, 5) * _meas_matrix(0, 1) +
		 _covariance(2, 5) * _meas_matrix(0, 2) + _covariance(3, 5) * _meas_matrix(0, 3) +
		 _covariance(4, 5) * _meas_matrix(0, 4) + _covariance(5, 5) * _meas_matrix(0, 5) +
		 _covariance(6, 5) * _meas_matrix(0, 6) + _covariance(7, 5) * _meas_matrix(0, 7) +
		 _covariance(8, 5) * _meas_matrix(0, 8)) +
		_meas_matrix(0, 6) *
		(_covariance(0, 6) * _meas_matrix(0, 0) + _covariance(1, 6) * _meas_matrix(0, 1) +
		 _covariance(2, 6) * _meas_matrix(0, 2) + _covariance(3, 6) * _meas_matrix(0, 3) +
		 _covariance(4, 6) * _meas_matrix(0, 4) + _covariance(5, 6) * _meas_matrix(0, 5) +
		 _covariance(6, 6) * _meas_matrix(0, 6) + _covariance(7, 6) * _meas_matrix(0, 7) +
		 _covariance(8, 6) * _meas_matrix(0, 8)) +
		_meas_matrix(0, 7) *
		(_covariance(0, 7) * _meas_matrix(0, 0) + _covariance(1, 7) * _meas_matrix(0, 1) +
		 _covariance(2, 7) * _meas_matrix(0, 2) + _covariance(3, 7) * _meas_matrix(0, 3) +
		 _covariance(4, 7) * _meas_matrix(0, 4) + _covariance(5, 7) * _meas_matrix(0, 5) +
		 _covariance(6, 7) * _meas_matrix(0, 6) + _covariance(7, 7) * _meas_matrix(0, 7) +
		 _covariance(8, 7) * _meas_matrix(0, 8)) +
		_meas_matrix(0, 8) *
		(_covariance(0, 8) * _meas_matrix(0, 0) + _covariance(1, 8) * _meas_matrix(0, 1) +
		 _covariance(2, 8) * _meas_matrix(0, 2) + _covariance(3, 8) * _meas_matrix(0, 3) +
		 _covariance(4, 8) * _meas_matrix(0, 4) + _covariance(5, 8) * _meas_matrix(0, 5) +
		 _covariance(6, 8) * _meas_matrix(0, 6) + _covariance(7, 8) * _meas_matrix(0, 7) +
		 _covariance(8, 8) * _meas_matrix(0, 8)) +
		meas_unc;

	return _innov_cov;
}

float KFxyzCoupledStatic::computeInnov(float meas)
{
	/* z - H*x */
	_innov = meas - (_meas_matrix * _sync_state)(0, 0);
	return _innov;
}

} // namespace landing_target_estimator
