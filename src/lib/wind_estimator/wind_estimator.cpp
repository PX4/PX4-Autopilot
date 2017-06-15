/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file wind_estimator.cpp
 * A wind and airspeed scale estimator.
 */

#include "wind_estimator.h"
#include <mathlib/mathlib.h>

using namespace matrix;

WindEstimator::WindEstimator() :
	_state(),
	_P(),
	_tas_innov(0.0f),
	_tas_innov_var(0.0f),
	_beta_innov(0.0f),
	_beta_innov_var(0.0f),
	_initialised(false)
{

}

WindEstimator::~WindEstimator()
{

}

bool WindEstimator::initialise(float velI[3], float velIvar[2], float tas_meas)
{
	// do no initialise if ground velocity is low
	// this should prevent the filter from initialising on the ground
	if (sqrtf(velI[0] * velI[0] + velI[1] * velI[1]) < 3.0f) {
		return false;
	}

	float v_n = velI[0];
	float v_e = velI[1];

	// estimate heading from ground velocity
	float heading_est = atan2f(v_e, v_n);

	// initilaise wind states assuming zero side slip and horizontal flight
	_state(w_n) = velI[w_n] - tas_meas * cosf(heading_est);
	_state(w_e) = velI[w_e] - tas_meas * sinf(heading_est);
	_state(tas) = 1.0f;

	// compute jacobian of states wrt north/each earth velocity states and true airspeed measurement
	float L0 = v_e * v_e;
	float L1 = v_n * v_n;
	float L2 = L0 + L1;
	float L3 = tas_meas / (L2 * sqrtf(L2));
	float L4 = L3 * v_e * v_n + 1.0f;
	float L5 = 1.0f / sqrtf(L2);
	float L6 = -L5 * tas_meas;

	Matrix<float, 3, 3> L;
	L.setZero();
	L(0, 0) = L4;
	L(0, 1) = L0 * L3 + L6;
	L(1, 0) = L1 * L3 + L6;
	L(1, 1) = L4;
	L(2, 2) = 1.0f;

	// get an estimate of the state covariance matrix given the estimated variance of ground velocity
	// and measured airspeed
	_P(w_n, w_n) = velIvar[0];
	_P(w_e, w_e) = velIvar[1];
	_P(tas, tas) = 0.0001f;

	_P = L * _P * L.transpose();

	return true;
}

void WindEstimator::update(float dt)
{
	if (!_initialised) {
		return;
	}

	float q_w = _wind_p_var;
	float q_k_tas = _tas_scale_p_var;

	float SPP0 = dt * dt;
	float SPP1 = SPP0 * q_w;
	float SPP2 = SPP1 + _P(0, 1);

	Matrix<float, 3, 3> P_next;

	P_next(0, 0) = SPP1 + _P(0, 0);
	P_next(0, 1) = SPP2;
	P_next(0, 2) = _P(0, 2);
	P_next(1, 0) = SPP2;
	P_next(1, 1) = SPP1 + _P(1, 1);
	P_next(1, 2) = _P(1, 2);
	P_next(2, 0) = _P(0, 2);
	P_next(2, 1) = _P(1, 2);
	P_next(2, 2) = SPP0 * q_k_tas + _P(2, 2);

	_P = P_next;
}

void WindEstimator::fuse_airspeed(float true_airspeed, float velI[3], float velIvar[2])
{
	velIvar[0] = velIvar[0] < 0.01f ? 0.01f : velIvar[0];
	velIvar[1] = velIvar[1] < 0.01f ? 0.01f : velIvar[1];


	if (!_initialised) {
		// try to initialise
		_initialised =	initialise(velI, velIvar, true_airspeed);
		return;
	}

	// assign helper variables
	float v_n = velI[0];
	float v_e = velI[1];
	float v_d = velI[2];

	float k_tas = _state(tas);

	// compute kalman gain K
	float HH0 = sqrtf(v_d * v_d + (v_e - w_e) * (v_e - w_e) + (v_n - w_n) * (v_n - w_n));
	float HH1 = k_tas / HH0;

	Matrix<float, 1, 3> H_tas;
	H_tas(0, 0) = HH1 * (-v_n + w_n);
	H_tas(0, 1) = HH1 * (-v_e + w_e);
	H_tas(0, 2) = HH0;

	Matrix<float, 3, 1> K = _P * H_tas.transpose();

	Matrix<float, 1, 1> S = H_tas * _P * H_tas.transpose() + _tas_var;

	K /= (S._data[0][0]);

	// compute innovation
	float airspeed_pred = _state(tas) * sqrtf((v_n - _state(w_n)) * (v_n - _state(w_n)) + (v_e - _state(w_e)) *
			      (v_e - _state(w_e)) + v_d * v_d);
	_tas_innov = true_airspeed - airspeed_pred;

	// innovation variance
	_tas_innov_var = S._data[0][0];

	// apply correction to state
	_state(w_n) += _tas_innov * K(0, 0);
	_state(w_e) += _tas_innov * K(1, 0);
	_state(tas) += _tas_innov * K(2, 0);

	// update covariance matrix
	_P = _P - K * H_tas * _P;

	run_sanity_checks();
}

void WindEstimator::fuse_beta(float velI[3], float q_att[4])
{
	if (!_initialised) {return;}

	float v_n = velI[0];
	float v_e = velI[1];
	float v_d = velI[2];

	// compute sideslip observation vector
	float HB0 = 2.0f * q_att[0];
	float HB1 = HB0 * q_att[3];
	float HB2 = 2.0f * q_att[1];
	float HB3 = HB2 * q_att[2];
	float HB4 = v_e - w_e;
	float HB5 = HB1 + HB3;
	float HB6 = v_n - w_n;
	float HB7 = q_att[0] * q_att[0];
	float HB8 = q_att[3] * q_att[3];
	float HB9 = HB7 - HB8;
	float HB10 = q_att[1] * q_att[1];
	float HB11 = q_att[2] * q_att[2];
	float HB12 = HB10 - HB11;
	float HB13 = HB12 + HB9;
	float HB14 = HB13 * HB6 + HB4 * HB5 + v_d * (-HB0 * q_att[2] + HB2 * q_att[3]);
	float HB15 = 1.0f / HB14;
	float HB16 = (HB4 * (-HB10 + HB11 + HB9) + HB6 * (-HB1 + HB3) + v_d * (HB0 * q_att[1] + 2.0f * q_att[2] * q_att[3])) /
		     (HB14 * HB14);

	Matrix<float, 1, 3> H_beta;
	H_beta(0, 0) = HB13 * HB16 + HB15 * (HB1 - HB3);
	H_beta(0, 1) = HB15 * (HB12 - HB7 + HB8) + HB16 * HB5;
	H_beta(0, 2) = 0;

	// compute kalman gain
	Matrix<float, 3, 1> K = _P * H_beta.transpose();

	Matrix<float, 1, 1> S = H_beta * _P * H_beta.transpose() + _beta_var;

	K /= (S._data[0][0]);

	// compute predicted side slip angle
	Vector3f rel_wind = Vector3f(velI[0] - _state(w_n), velI[1] - _state(w_e), velI[2]);
	Dcmf R_body_to_earth = Quatf(q_att);
	rel_wind = R_body_to_earth.transpose() * rel_wind;

	if (fabsf(rel_wind(0)) < 0.1f) {return;}

	// use small angle approximation, sin(x) = x for small x
	float beta_pred = rel_wind(1) / rel_wind(0);

	_beta_innov = 0.0f - beta_pred;
	_beta_innov_var = S._data[0][0];

	// apply correction to state
	_state(w_n) += _beta_innov * K(0, 0);
	_state(w_e) += _beta_innov * K(1, 0);
	_state(tas) += _beta_innov * K(2, 0);

	// update covariance matrix
	_P = _P - K * H_beta * _P;

	run_sanity_checks();
}

void WindEstimator::run_sanity_checks()
{
	for (unsigned i = 0; i < 3; i++) {
		if (_P(i, i) < 0.0f) {
			// ill-conditioned covariance matrix, reset filter
			_initialised = false;
			return;
		}

		// limit covariance diagonals if they grow too large
		if (i < 2) {
			_P(i, i) = _P(i, i) > 25.0f ? 25.0f : _P(i, i);

		} else if (i == 2) {
			_P(i, i) = _P(i, i) > 0.1f ? 0.1f : _P(i, i);
		}
	}

	if (!PX4_ISFINITE(_state(w_n)) || !PX4_ISFINITE(_state(w_e)) || !PX4_ISFINITE(_state(tas))) {
		_initialised = false;
		return;
	}

	// constrain airspeed scale factor
	_state(tas) = math::constrain(_state(tas), 0.7f, 1.0f);

	// attain symmetry
	for (unsigned row = 0; row < 3; row++) {
		for (unsigned column = 0; column < row; column++) {
			float tmp = (_P(row, column) + _P(column, row)) / 2;
			_P(row, column) = tmp;
			_P(column, row) = tmp;
		}
	}
}
