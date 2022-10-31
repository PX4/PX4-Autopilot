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
 * @file KFxyzCoupledMoving.h
 * Simple Kalman Filter for static target
 *
 * @author Jonas Perolini <jonas.perolini@epfl.ch>
 *
 */

#include "KFxyzCoupledMoving.h"

namespace landing_target_estimator
{

void KFxyzCoupledMoving::predictState(float dt, matrix::Vector<float, 3> acc)
{
	// Total ops: 32

	const float tmp0 = 0.5f * dt * dt;

	_state(0, 0) = _state(0, 0) + _state(3, 0) * dt + _state(9, 0) * tmp0 - tmp0 * acc(0);
	_state(1, 0) = _state(1, 0) + _state(10, 0) * tmp0 + _state(4, 0) * dt - tmp0 * acc(1);
	_state(2, 0) = _state(11, 0) * tmp0 + _state(2, 0) + _state(5, 0) * dt - tmp0 * acc(2);
	_state(3, 0) = _state(3, 0) + _state(9, 0) * dt - acc(0) * dt;
	_state(4, 0) = _state(10, 0) * dt + _state(4, 0) - acc(1) * dt;
	_state(5, 0) = _state(11, 0) * dt + _state(5, 0) - acc(2) * dt;
	// _state(6, 0) = _state(6, 0);
	// _state(7, 0) = _state(7, 0);
	// _state(8, 0) = _state(8, 0);
	// _state(9, 0) = _state(9, 0);
	// _state(10, 0) = _state(10, 0);
	// _state(11, 0) = _state(11, 0);
}

void KFxyzCoupledMoving::predictCov(float dt)
{
// Total ops: 480

	// Input arrays

	// Intermediate terms (76)
	const float _tmp0 = 0.25f * dt * dt * dt * dt;
	const float _tmp1 = dt * dt;
	const float _tmp2 = 0.5f * _tmp1;
	const float _tmp3 = _covariance(0, 3) + _covariance(3, 3) * dt + _covariance(9, 3) * _tmp2;
	const float _tmp4 = _covariance(9, 9) * _tmp2;
	const float _tmp5 = _covariance(0, 9) + _covariance(3, 9) * dt + _tmp4;
	const float _tmp6 = _covariance(1, 3) + _covariance(10, 3) * _tmp2 + _covariance(4, 3) * dt;
	const float _tmp7 = _covariance(10, 9) * _tmp2;
	const float _tmp8 = _covariance(1, 9) + _covariance(4, 9) * dt + _tmp7;
	const float _tmp9 = _covariance(11, 9) * _tmp2;
	const float _tmp10 = _covariance(2, 9) + _covariance(5, 9) * dt + _tmp9;
	const float _tmp11 = _covariance(11, 3) * _tmp2 + _covariance(2, 3) + _covariance(5, 3) * dt;
	const float _tmp12 = 0.5f * dt * dt * dt;
	const float _tmp13 = _input_var(0, 0) * _tmp12;
	const float _tmp14 = _covariance(9, 3) * dt;
	const float _tmp15 = _covariance(3, 3) + _tmp14;
	const float _tmp16 = _covariance(9, 9) * dt;
	const float _tmp17 = _covariance(3, 9) + _tmp16;
	const float _tmp18 = _input_var(1, 0) * _tmp12;
	const float _tmp19 = _covariance(10, 3) * dt;
	const float _tmp20 = _covariance(4, 3) + _tmp19;
	const float _tmp21 = _covariance(10, 9) * dt;
	const float _tmp22 = _covariance(4, 9) + _tmp21;
	const float _tmp23 = _input_var(2, 0) * _tmp12;
	const float _tmp24 = _covariance(11, 3) * dt;
	const float _tmp25 = _covariance(5, 3) + _tmp24;
	const float _tmp26 = _covariance(11, 9) * dt;
	const float _tmp27 = _covariance(5, 9) + _tmp26;
	// const float _tmp28 = _covariance(0, 4) + _covariance(3, 4) * dt + _covariance(9, 4) * _tmp2;
	const float _tmp29 = _covariance(9, 10) * _tmp2;
	// const float _tmp30 = _covariance(0, 10) + _covariance(3, 10) * dt + _tmp29;
	const float _tmp31 = _covariance(10, 10) * _tmp2;
	const float _tmp32 = _covariance(1, 10) + _covariance(4, 10) * dt + _tmp31;
	const float _tmp33 = _covariance(1, 4) + _covariance(10, 4) * _tmp2 + _covariance(4, 4) * dt;
	const float _tmp34 = _covariance(11, 10) * _tmp2;
	const float _tmp35 = _covariance(2, 10) + _covariance(5, 10) * dt + _tmp34;
	const float _tmp36 = _covariance(11, 4) * _tmp2 + _covariance(2, 4) + _covariance(5, 4) * dt;
	const float _tmp37 = _input_var(0, 1) * _tmp12;
	const float _tmp38 = _covariance(9, 10) * dt;
	const float _tmp39 = _covariance(3, 10) + _tmp38;
	const float _tmp40 = _covariance(9, 4) * dt;
	const float _tmp41 = _covariance(3, 4) + _tmp40;
	const float _tmp42 = _input_var(1, 1) * _tmp12;
	const float _tmp43 = _covariance(10, 10) * dt;
	const float _tmp44 = _covariance(4, 10) + _tmp43;
	const float _tmp45 = _covariance(10, 4) * dt;
	const float _tmp46 = _covariance(4, 4) + _tmp45;
	const float _tmp47 = _input_var(2, 1) * _tmp12;
	const float _tmp48 = _covariance(11, 10) * dt;
	const float _tmp49 = _covariance(5, 10) + _tmp48;
	const float _tmp50 = _covariance(11, 4) * dt;
	const float _tmp51 = _covariance(5, 4) + _tmp50;
	// const float _tmp52 = _covariance(0, 5) + _covariance(3, 5) * dt + _covariance(9, 5) * _tmp2;
	const float _tmp53 = _covariance(9, 11) * _tmp2;
	// const float _tmp54 = _covariance(0, 11) + _covariance(3, 11) * dt + _tmp53;
	const float _tmp55 = _covariance(10, 11) * _tmp2;
	// const float _tmp56 = _covariance(1, 11) + _covariance(4, 11) * dt + _tmp55;
	// const float _tmp57 = _covariance(1, 5) + _covariance(10, 5) * _tmp2 + _covariance(4, 5) * dt;
	const float _tmp58 = _covariance(11, 11) * _tmp2;
	const float _tmp59 = _covariance(2, 11) + _covariance(5, 11) * dt + _tmp58;
	const float _tmp60 = _covariance(11, 5) * _tmp2 + _covariance(2, 5) + _covariance(5, 5) * dt;
	const float _tmp61 = _input_var(0, 2) * _tmp12;
	const float _tmp62 = _covariance(9, 11) * dt;
	const float _tmp63 = _covariance(3, 11) + _tmp62;
	const float _tmp64 = _covariance(9, 5) * dt;
	const float _tmp65 = _covariance(3, 5) + _tmp64;
	const float _tmp66 = _input_var(1, 2) * _tmp12;
	const float _tmp67 = _covariance(10, 11) * dt;
	const float _tmp68 = _covariance(4, 11) + _tmp67;
	const float _tmp69 = _covariance(10, 5) * dt;
	const float _tmp70 = _covariance(4, 5) + _tmp69;
	const float _tmp71 = _input_var(2, 2) * _tmp12;
	const float _tmp72 = _covariance(11, 11) * dt;
	const float _tmp73 = _covariance(5, 11) + _tmp72;
	const float _tmp74 = _covariance(11, 5) * dt;
	const float _tmp75 = _covariance(5, 5) + _tmp74;

	_covariance(0, 0) = _covariance(0, 0) + _covariance(3, 0) * dt + _covariance(9, 0) * _tmp2 + _input_var(0,
			    0) * _tmp0 + _tmp2 * _tmp5 + _tmp3 * dt;
	_covariance(1, 0) = _covariance(1, 0) + _covariance(10, 0) * _tmp2 + _covariance(4, 0) * dt + _input_var(1,
			    0) * _tmp0 + _tmp2 * _tmp8 + _tmp6 * dt;
	_covariance(2, 0) = _covariance(11, 0) * _tmp2 + _covariance(2, 0) + _covariance(5, 0) * dt + _input_var(2,
			    0) * _tmp0 + _tmp10 * _tmp2 + _tmp11 * dt;
	_covariance(3, 0) = _covariance(3, 0) + _covariance(9, 0) * dt + _tmp13 + _tmp15 * dt + _tmp17 * _tmp2;
	_covariance(4, 0) = _covariance(10, 0) * dt + _covariance(4, 0) + _tmp18 + _tmp2 * _tmp22 + _tmp20 * dt;
	_covariance(5, 0) = _covariance(11, 0) * dt + _covariance(5, 0) + _tmp2 * _tmp27 + _tmp23 + _tmp25 * dt;
	_covariance(6, 0) = _covariance(6, 0) + _covariance(6, 3) * dt + _covariance(6, 9) * _tmp2;
	_covariance(7, 0) = _covariance(7, 0) + _covariance(7, 3) * dt + _covariance(7, 9) * _tmp2;
	_covariance(8, 0) = _covariance(8, 0) + _covariance(8, 3) * dt + _covariance(8, 9) * _tmp2;
	_covariance(9, 0) = _covariance(9, 0) + _tmp14 + _tmp4;
	_covariance(10, 0) = _covariance(10, 0) + _tmp19 + _tmp7;
	_covariance(11, 0) = _covariance(11, 0) + _tmp24 + _tmp9;

	_covariance(1, 1) = _covariance(1, 1) + _covariance(10, 1) * _tmp2 + _covariance(4, 1) * dt + _input_var(1,
			    1) * _tmp0 + _tmp2 * _tmp32 + _tmp33 * dt;
	_covariance(2, 1) = _covariance(11, 1) * _tmp2 + _covariance(2, 1) + _covariance(5, 1) * dt + _input_var(2,
			    1) * _tmp0 + _tmp2 * _tmp35 + _tmp36 * dt;
	_covariance(3, 1) = _covariance(3, 1) + _covariance(9, 1) * dt + _tmp2 * _tmp39 + _tmp37 + _tmp41 * dt;
	_covariance(4, 1) = _covariance(10, 1) * dt + _covariance(4, 1) + _tmp2 * _tmp44 + _tmp42 + _tmp46 * dt;
	_covariance(5, 1) = _covariance(11, 1) * dt + _covariance(5, 1) + _tmp2 * _tmp49 + _tmp47 + _tmp51 * dt;
	_covariance(6, 1) = _covariance(6, 1) + _covariance(6, 10) * _tmp2 + _covariance(6, 4) * dt;
	_covariance(7, 1) = _covariance(7, 1) + _covariance(7, 10) * _tmp2 + _covariance(7, 4) * dt;
	_covariance(8, 1) = _covariance(8, 1) + _covariance(8, 10) * _tmp2 + _covariance(8, 4) * dt;
	_covariance(9, 1) = _covariance(9, 1) + _tmp29 + _tmp40;
	_covariance(10, 1) = _covariance(10, 1) + _tmp31 + _tmp45;
	_covariance(11, 1) = _covariance(11, 1) + _tmp34 + _tmp50;

	_covariance(2, 2) = _bias_var(0, 0) + _covariance(11, 2) * _tmp2 + _covariance(2, 2) + _covariance(5,
			    2) * dt + _input_var(2, 2) * _tmp0 + _tmp2 * _tmp59 + _tmp60 * dt;
	_covariance(3, 2) = _covariance(3, 2) + _covariance(9, 2) * dt + _tmp2 * _tmp63 + _tmp61 + _tmp65 * dt;
	_covariance(4, 2) = _covariance(10, 2) * dt + _covariance(4, 2) + _tmp2 * _tmp68 + _tmp66 + _tmp70 * dt;
	_covariance(5, 2) = _covariance(11, 2) * dt + _covariance(5, 2) + _tmp2 * _tmp73 + _tmp71 + _tmp75 * dt;
	_covariance(6, 2) = _covariance(6, 11) * _tmp2 + _covariance(6, 2) + _covariance(6, 5) * dt;
	_covariance(7, 2) = _covariance(7, 11) * _tmp2 + _covariance(7, 2) + _covariance(7, 5) * dt;
	_covariance(8, 2) = _covariance(8, 11) * _tmp2 + _covariance(8, 2) + _covariance(8, 5) * dt;
	_covariance(9, 2) = _covariance(9, 2) + _tmp53 + _tmp64;
	_covariance(10, 2) = _covariance(10, 2) + _tmp55 + _tmp69;
	_covariance(11, 2) = _covariance(11, 2) + _tmp58 + _tmp74;

	_covariance(3, 3) = _acc_var(0, 0) + _input_var(0, 0) * _tmp1 + _tmp15 + _tmp17 * dt;
	_covariance(4, 3) = _input_var(1, 0) * _tmp1 + _tmp20 + _tmp22 * dt;
	_covariance(5, 3) = _input_var(2, 0) * _tmp1 + _tmp25 + _tmp27 * dt;
	_covariance(6, 3) = _covariance(6, 3) + _covariance(6, 9) * dt;
	_covariance(7, 3) = _covariance(7, 3) + _covariance(7, 9) * dt;
	_covariance(8, 3) = _covariance(8, 3) + _covariance(8, 9) * dt;
	_covariance(9, 3) = _covariance(9, 3) + _tmp16;
	_covariance(10, 3) = _covariance(10, 3) + _tmp21;
	_covariance(11, 3) = _covariance(11, 3) + _tmp26;

	_covariance(4, 4) = _input_var(1, 1) * _tmp1 + _tmp44 * dt + _tmp46;
	_covariance(5, 4) = _input_var(2, 1) * _tmp1 + _tmp49 * dt + _tmp51;
	_covariance(6, 4) = _covariance(6, 10) * dt + _covariance(6, 4);
	_covariance(7, 4) = _covariance(7, 10) * dt + _covariance(7, 4);
	_covariance(8, 4) = _covariance(8, 10) * dt + _covariance(8, 4);
	_covariance(9, 4) = _covariance(9, 4) + _tmp38;
	_covariance(10, 4) = _covariance(10, 4) + _tmp43;
	_covariance(11, 4) = _covariance(11, 4) + _tmp48;

	_covariance(5, 5) = _input_var(2, 2) * _tmp1 + _tmp73 * dt + _tmp75;
	_covariance(6, 5) = _covariance(6, 11) * dt + _covariance(6, 5);
	_covariance(7, 5) = _covariance(7, 11) * dt + _covariance(7, 5);
	_covariance(8, 5) = _covariance(8, 11) * dt + _covariance(8, 5);
	_covariance(9, 5) = _covariance(9, 5) + _tmp62;
	_covariance(10, 5) = _covariance(10, 5) + _tmp67;
	_covariance(11, 5) = _covariance(11, 5) + _tmp72;

	_covariance(6, 6) = _bias_var(1, 1) + _covariance(6, 6);
	// _covariance(7, 6) = _covariance(7, 6);
	// _covariance(8, 6) = _covariance(8, 6);
	// _covariance(9, 6) = _covariance(9, 6);
	// _covariance(10, 6) = _covariance(10, 6);
	// _covariance(11, 6) = _covariance(11, 6);

	_covariance(7, 7) = _acc_var(1, 1) + _covariance(7, 7);
	// _covariance(8, 7) = _covariance(8, 7);
	// _covariance(9, 7) = _covariance(9, 7);
	// _covariance(10, 7) = _covariance(10, 7);
	// _covariance(11, 7) = _covariance(11, 7);

	// _covariance(8, 8) = _covariance(8, 8);
	// _covariance(9, 8) = _covariance(9, 8);
	// _covariance(10, 8) = _covariance(10, 8);
	// _covariance(11, 8) = _covariance(11, 8);

	// _covariance(9, 9) = _covariance(9, 9);
	// _covariance(10, 9) = _covariance(10, 9);
	// _covariance(11, 9) = _covariance(11, 9);

	_covariance(10, 10) = _bias_var(2, 2) + _covariance(10, 10);
	// _covariance(11, 10) = _covariance(11, 10);

	_covariance(11, 11) = _acc_var(2, 2) + _covariance(11, 11);

	// Symmetric matrix: _covariance = upper_tri + upper_tri.transpose() - diag(upper_tri.diag())
	_covariance(0, 1) = _covariance(1, 0);

	_covariance(0, 2) = _covariance(2, 0);
	_covariance(1, 2) = _covariance(2, 1);

	_covariance(0, 3) = _covariance(3, 0);
	_covariance(1, 3) = _covariance(3, 1);
	_covariance(2, 3) = _covariance(3, 2);

	_covariance(0, 4) = _covariance(0, 4);
	_covariance(1, 4) = _covariance(0, 4);
	_covariance(2, 4) = _covariance(0, 4);
	_covariance(3, 4) = _covariance(0, 4);

	_covariance(0, 5) = _covariance(5, 0);
	_covariance(1, 5) = _covariance(5, 1);
	_covariance(2, 5) = _covariance(5, 2);
	_covariance(3, 5) = _covariance(5, 3);
	_covariance(4, 5) = _covariance(5, 4);

	_covariance(0, 6) = _covariance(6, 0);
	_covariance(1, 6) = _covariance(6, 1);
	_covariance(2, 6) = _covariance(6, 2);
	_covariance(3, 6) = _covariance(6, 3);
	_covariance(4, 6) = _covariance(6, 4);
	_covariance(5, 6) = _covariance(6, 5);

	_covariance(0, 7) = _covariance(7, 0);
	_covariance(1, 7) = _covariance(7, 1);
	_covariance(2, 7) = _covariance(7, 2);
	_covariance(3, 7) = _covariance(7, 3);
	_covariance(4, 7) = _covariance(7, 4);
	_covariance(5, 7) = _covariance(7, 5);
	//_covariance(6, 7) = _covariance(7, 6);

	_covariance(0, 8) = _covariance(8, 0);
	_covariance(1, 8) = _covariance(8, 1);
	_covariance(2, 8) = _covariance(8, 2);
	_covariance(3, 8) = _covariance(8, 3);
	_covariance(4, 8) = _covariance(8, 4);
	_covariance(5, 8) = _covariance(8, 5);
	// _covariance(6, 8) = _covariance(8, 6);
	// _covariance(7, 8) = _covariance(8, 7);

	_covariance(0, 9) = _covariance(9, 0);
	_covariance(1, 9) = _covariance(9, 1);
	_covariance(2, 9) = _covariance(9, 2);
	_covariance(3, 9) = _covariance(9, 3);
	_covariance(4, 9) = _covariance(9, 4);
	_covariance(5, 9) = _covariance(9, 5);
	// _covariance(6, 9) = _covariance(9, 6);
	// _covariance(7, 9) = _covariance(9, 7);
	// _covariance(8, 9) = _covariance(9, 8);

	_covariance(0, 10) = _covariance(10, 0);
	_covariance(1, 10) = _covariance(10, 1);
	_covariance(2, 10) = _covariance(10, 2);
	_covariance(3, 10) = _covariance(10, 3);
	_covariance(4, 10) = _covariance(10, 4);
	_covariance(5, 10) = _covariance(10, 5);
	// _covariance(6, 10) = _covariance(0, 6);
	// _covariance(7, 10) = _covariance(10, 7);
	// _covariance(8, 10) = _covariance(10, 8);
	// _covariance(9, 10) = _covariance(10, 9);

	_covariance(0, 11) = _covariance(11, 0);
	_covariance(1, 11) = _covariance(11, 1);
	_covariance(2, 11) = _covariance(11, 2);
	_covariance(3, 11) = _covariance(11, 3);
	_covariance(4, 11) = _covariance(11, 4);
	_covariance(5, 11) = _covariance(11, 5);
	// _covariance(6, 11) = _covariance(11, 6);
	// _covariance(7, 11) = _covariance(11, 7);
	// _covariance(8, 11) = _covariance(11, 8);
	// _covariance(9, 11) = _covariance(11, 9);
	// _covariance(10, 11) = _covariance(11, 10);
}


bool KFxyzCoupledMoving::update()
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

	matrix::Matrix<float, 12, 1> kalmanGain = _covariance * _meas_matrix.transpose() / _innov_cov;

	_state = _state + kalmanGain * _innov;
	_covariance = _covariance - kalmanGain * _meas_matrix * _covariance;

	return true;
}

void KFxyzCoupledMoving::setH(matrix::Vector<float, 12> h_meas)
{
	// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]
	_meas_matrix.row(0) = h_meas;
}

void KFxyzCoupledMoving::syncState(float dt, matrix::Vector<float, 3> acc)
{
	// Total ops: 45

	const float _tmp0 = 0.5f * dt * dt;
	const float _tmp1 = _state(3, 0);
	const float _tmp2 = _state(4, 0);
	const float _tmp3 = _state(5, 0);
	const float _tmp4 = dt;
	const float _tmp5 = _state(9, 0);
	const float _tmp6 = _state(10, 0);
	const float _tmp7 = _state(11, 0);

	_sync_state(0, 0) = _state(0, 0) + _state(9, 0) * _tmp0 - _tmp0 * acc(0) - _tmp1 * dt;
	_sync_state(1, 0) = _state(1, 0) + _state(10, 0) * _tmp0 - _tmp0 * acc(1) - _tmp2 * dt;
	_sync_state(2, 0) = _state(11, 0) * _tmp0 +  _state(2, 0) - _tmp0 * acc(2) - _tmp3 * dt;
	_sync_state(3, 0) = _tmp1 + _tmp4 * acc(0) - _tmp5 * dt;
	_sync_state(4, 0) = _tmp2 + _tmp4 * acc(1) - _tmp6 * dt;
	_sync_state(5, 0) = _tmp3 + _tmp4 * acc(2) - _tmp7 * dt;
	_sync_state(6, 0) =  _state(6, 0);
	_sync_state(7, 0) =  _state(7, 0);
	_sync_state(8, 0) =  _state(8, 0);
	_sync_state(9, 0) = _tmp5;
	_sync_state(10, 0) = _tmp6;
	_sync_state(11, 0) = _tmp7;
}

float KFxyzCoupledMoving::computeInnovCov(float meas_unc)
{
	// Total ops: 300

	_innov_cov =
		_meas_matrix(0, 0) *
		(_covariance(0, 0) * _meas_matrix(0, 0) + _covariance(1, 0) * _meas_matrix(0, 1) +
		 _covariance(10, 0) * _meas_matrix(0, 10) + _covariance(11, 0) * _meas_matrix(0, 11) +
		 _covariance(2, 0) * _meas_matrix(0, 2) + _covariance(3, 0) * _meas_matrix(0, 3) +
		 _covariance(4, 0) * _meas_matrix(0, 4) + _covariance(5, 0) * _meas_matrix(0, 5) +
		 _covariance(6, 0) * _meas_matrix(0, 6) + _covariance(7, 0) * _meas_matrix(0, 7) +
		 _covariance(8, 0) * _meas_matrix(0, 8) + _covariance(9, 0) * _meas_matrix(0, 9)) +
		_meas_matrix(0, 1) *
		(_covariance(0, 1) * _meas_matrix(0, 0) + _covariance(1, 1) * _meas_matrix(0, 1) +
		 _covariance(10, 1) * _meas_matrix(0, 10) + _covariance(11, 1) * _meas_matrix(0, 11) +
		 _covariance(2, 1) * _meas_matrix(0, 2) + _covariance(3, 1) * _meas_matrix(0, 3) +
		 _covariance(4, 1) * _meas_matrix(0, 4) + _covariance(5, 1) * _meas_matrix(0, 5) +
		 _covariance(6, 1) * _meas_matrix(0, 6) + _covariance(7, 1) * _meas_matrix(0, 7) +
		 _covariance(8, 1) * _meas_matrix(0, 8) + _covariance(9, 1) * _meas_matrix(0, 9)) +
		_meas_matrix(0, 10) *
		(_covariance(0, 10) * _meas_matrix(0, 0) + _covariance(1, 10) * _meas_matrix(0, 1) +
		 _covariance(10, 10) * _meas_matrix(0, 10) + _covariance(11, 10) * _meas_matrix(0, 11) +
		 _covariance(2, 10) * _meas_matrix(0, 2) + _covariance(3, 10) * _meas_matrix(0, 3) +
		 _covariance(4, 10) * _meas_matrix(0, 4) + _covariance(5, 10) * _meas_matrix(0, 5) +
		 _covariance(6, 10) * _meas_matrix(0, 6) + _covariance(7, 10) * _meas_matrix(0, 7) +
		 _covariance(8, 10) * _meas_matrix(0, 8) + _covariance(9, 10) * _meas_matrix(0, 9)) +
		_meas_matrix(0, 11) *
		(_covariance(0, 11) * _meas_matrix(0, 0) + _covariance(1, 11) * _meas_matrix(0, 1) +
		 _covariance(10, 11) * _meas_matrix(0, 10) + _covariance(11, 11) * _meas_matrix(0, 11) +
		 _covariance(2, 11) * _meas_matrix(0, 2) + _covariance(3, 11) * _meas_matrix(0, 3) +
		 _covariance(4, 11) * _meas_matrix(0, 4) + _covariance(5, 11) * _meas_matrix(0, 5) +
		 _covariance(6, 11) * _meas_matrix(0, 6) + _covariance(7, 11) * _meas_matrix(0, 7) +
		 _covariance(8, 11) * _meas_matrix(0, 8) + _covariance(9, 11) * _meas_matrix(0, 9)) +
		_meas_matrix(0, 2) *
		(_covariance(0, 2) * _meas_matrix(0, 0) + _covariance(1, 2) * _meas_matrix(0, 1) +
		 _covariance(10, 2) * _meas_matrix(0, 10) + _covariance(11, 2) * _meas_matrix(0, 11) +
		 _covariance(2, 2) * _meas_matrix(0, 2) + _covariance(3, 2) * _meas_matrix(0, 3) +
		 _covariance(4, 2) * _meas_matrix(0, 4) + _covariance(5, 2) * _meas_matrix(0, 5) +
		 _covariance(6, 2) * _meas_matrix(0, 6) + _covariance(7, 2) * _meas_matrix(0, 7) +
		 _covariance(8, 2) * _meas_matrix(0, 8) + _covariance(9, 2) * _meas_matrix(0, 9)) +
		_meas_matrix(0, 3) *
		(_covariance(0, 3) * _meas_matrix(0, 0) + _covariance(1, 3) * _meas_matrix(0, 1) +
		 _covariance(10, 3) * _meas_matrix(0, 10) + _covariance(11, 3) * _meas_matrix(0, 11) +
		 _covariance(2, 3) * _meas_matrix(0, 2) + _covariance(3, 3) * _meas_matrix(0, 3) +
		 _covariance(4, 3) * _meas_matrix(0, 4) + _covariance(5, 3) * _meas_matrix(0, 5) +
		 _covariance(6, 3) * _meas_matrix(0, 6) + _covariance(7, 3) * _meas_matrix(0, 7) +
		 _covariance(8, 3) * _meas_matrix(0, 8) + _covariance(9, 3) * _meas_matrix(0, 9)) +
		_meas_matrix(0, 4) *
		(_covariance(0, 4) * _meas_matrix(0, 0) + _covariance(1, 4) * _meas_matrix(0, 1) +
		 _covariance(10, 4) * _meas_matrix(0, 10) + _covariance(11, 4) * _meas_matrix(0, 11) +
		 _covariance(2, 4) * _meas_matrix(0, 2) + _covariance(3, 4) * _meas_matrix(0, 3) +
		 _covariance(4, 4) * _meas_matrix(0, 4) + _covariance(5, 4) * _meas_matrix(0, 5) +
		 _covariance(6, 4) * _meas_matrix(0, 6) + _covariance(7, 4) * _meas_matrix(0, 7) +
		 _covariance(8, 4) * _meas_matrix(0, 8) + _covariance(9, 4) * _meas_matrix(0, 9)) +
		_meas_matrix(0, 5) *
		(_covariance(0, 5) * _meas_matrix(0, 0) + _covariance(1, 5) * _meas_matrix(0, 1) +
		 _covariance(10, 5) * _meas_matrix(0, 10) + _covariance(11, 5) * _meas_matrix(0, 11) +
		 _covariance(2, 5) * _meas_matrix(0, 2) + _covariance(3, 5) * _meas_matrix(0, 3) +
		 _covariance(4, 5) * _meas_matrix(0, 4) + _covariance(5, 5) * _meas_matrix(0, 5) +
		 _covariance(6, 5) * _meas_matrix(0, 6) + _covariance(7, 5) * _meas_matrix(0, 7) +
		 _covariance(8, 5) * _meas_matrix(0, 8) + _covariance(9, 5) * _meas_matrix(0, 9)) +
		_meas_matrix(0, 6) *
		(_covariance(0, 6) * _meas_matrix(0, 0) + _covariance(1, 6) * _meas_matrix(0, 1) +
		 _covariance(10, 6) * _meas_matrix(0, 10) + _covariance(11, 6) * _meas_matrix(0, 11) +
		 _covariance(2, 6) * _meas_matrix(0, 2) + _covariance(3, 6) * _meas_matrix(0, 3) +
		 _covariance(4, 6) * _meas_matrix(0, 4) + _covariance(5, 6) * _meas_matrix(0, 5) +
		 _covariance(6, 6) * _meas_matrix(0, 6) + _covariance(7, 6) * _meas_matrix(0, 7) +
		 _covariance(8, 6) * _meas_matrix(0, 8) + _covariance(9, 6) * _meas_matrix(0, 9)) +
		_meas_matrix(0, 7) *
		(_covariance(0, 7) * _meas_matrix(0, 0) + _covariance(1, 7) * _meas_matrix(0, 1) +
		 _covariance(10, 7) * _meas_matrix(0, 10) + _covariance(11, 7) * _meas_matrix(0, 11) +
		 _covariance(2, 7) * _meas_matrix(0, 2) + _covariance(3, 7) * _meas_matrix(0, 3) +
		 _covariance(4, 7) * _meas_matrix(0, 4) + _covariance(5, 7) * _meas_matrix(0, 5) +
		 _covariance(6, 7) * _meas_matrix(0, 6) + _covariance(7, 7) * _meas_matrix(0, 7) +
		 _covariance(8, 7) * _meas_matrix(0, 8) + _covariance(9, 7) * _meas_matrix(0, 9)) +
		_meas_matrix(0, 8) *
		(_covariance(0, 8) * _meas_matrix(0, 0) + _covariance(1, 8) * _meas_matrix(0, 1) +
		 _covariance(10, 8) * _meas_matrix(0, 10) + _covariance(11, 8) * _meas_matrix(0, 11) +
		 _covariance(2, 8) * _meas_matrix(0, 2) + _covariance(3, 8) * _meas_matrix(0, 3) +
		 _covariance(4, 8) * _meas_matrix(0, 4) + _covariance(5, 8) * _meas_matrix(0, 5) +
		 _covariance(6, 8) * _meas_matrix(0, 6) + _covariance(7, 8) * _meas_matrix(0, 7) +
		 _covariance(8, 8) * _meas_matrix(0, 8) + _covariance(9, 8) * _meas_matrix(0, 9)) +
		_meas_matrix(0, 9) *
		(_covariance(0, 9) * _meas_matrix(0, 0) + _covariance(1, 9) * _meas_matrix(0, 1) +
		 _covariance(10, 9) * _meas_matrix(0, 10) + _covariance(11, 9) * _meas_matrix(0, 11) +
		 _covariance(2, 9) * _meas_matrix(0, 2) + _covariance(3, 9) * _meas_matrix(0, 3) +
		 _covariance(4, 9) * _meas_matrix(0, 4) + _covariance(5, 9) * _meas_matrix(0, 5) +
		 _covariance(6, 9) * _meas_matrix(0, 6) + _covariance(7, 9) * _meas_matrix(0, 7) +
		 _covariance(8, 9) * _meas_matrix(0, 8) + _covariance(9, 9) * _meas_matrix(0, 9)) +
		meas_unc;

	return _innov_cov;
}

float KFxyzCoupledMoving::computeInnov(float meas)
{
	/* z - H*x */
	_innov = meas - (_meas_matrix * _sync_state)(0, 0);
	return _innov;
}

} // namespace landing_target_estimator
