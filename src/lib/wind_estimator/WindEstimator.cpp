/****************************************************************************
 *
 *   Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
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
 * @file WindEstimator.cpp
 * A wind and airspeed scale estimator.
 */

#include "WindEstimator.hpp"

bool
WindEstimator::initialise(const matrix::Vector3f &velI, const matrix::Vector2f &velIvar, const float tas_meas)
{
	// do no initialise if ground velocity is low
	// this should prevent the filter from initialising on the ground
	if (sqrtf(velI(0) * velI(0) + velI(1) * velI(1)) < 3.0f) {
		return false;
	}

	const float v_n = velI(0);
	const float v_e = velI(1);

	// estimate heading from ground velocity
	const float heading_est = atan2f(v_e, v_n);

	// initilaise wind states assuming zero side slip and horizontal flight
	_state(INDEX_W_N) = velI(INDEX_W_N) - tas_meas * cosf(heading_est);
	_state(INDEX_W_E) = velI(INDEX_W_E) - tas_meas * sinf(heading_est);
	_state(INDEX_TAS_SCALE) = _scale_init;

	// compute jacobian of states wrt north/each earth velocity states and true airspeed measurement
	float L0 = v_e * v_e;
	float L1 = v_n * v_n;
	float L2 = L0 + L1;
	float L3 = tas_meas / (L2 * sqrtf(L2));
	float L4 = L3 * v_e * v_n + 1.0f;
	float L5 = 1.0f / sqrtf(L2);
	float L6 = -L5 * tas_meas;

	matrix::Matrix3f L;
	L.setZero();
	L(0, 0) = L4;
	L(0, 1) = L0 * L3 + L6;
	L(1, 0) = L1 * L3 + L6;
	L(1, 1) = L4;
	L(2, 2) = 1.0f;

	// get an estimate of the state covariance matrix given the estimated variance of ground velocity
	// and measured airspeed
	_P.setZero();
	_P(INDEX_W_N, INDEX_W_N) = velIvar(0);
	_P(INDEX_W_E, INDEX_W_E) = velIvar(1);
	_P(INDEX_TAS_SCALE, INDEX_TAS_SCALE) = 0.0001f;

	_P = L * _P * L.transpose();

	// reset the timestamp for measurement rejection
	_time_rejected_tas = 0;
	_time_rejected_beta = 0;

	_wind_estimator_reset = true;

	return true;
}

void
WindEstimator::update(uint64_t time_now)
{
	if (!_initialised) {
		return;
	}

	// set reset state to false (is set to true when initialise fuction is called later)
	_wind_estimator_reset = false;

	// run covariance prediction at 1Hz
	if (time_now - _time_last_update < 1000 * 1000 || _time_last_update == 0) {
		if (_time_last_update == 0) {
			_time_last_update = time_now;
		}

		return;
	}

	float dt = (float)(time_now - _time_last_update) * 1e-6f;
	_time_last_update = time_now;

	float q_w = _wind_p_var;
	float q_k_tas = _disable_tas_scale_estimate ? 0.f : _tas_scale_p_var;

	float SPP0 = dt * dt;
	float SPP1 = SPP0 * q_w;
	float SPP2 = SPP1 + _P(0, 1);

	matrix::Matrix3f P_next;

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

void
WindEstimator::fuse_airspeed(uint64_t time_now, const float true_airspeed, const matrix::Vector3f &velI,
			     const matrix::Vector2f &velIvar)
{
	matrix::Vector2f velIvar_constrained = { math::max(0.01f, velIvar(0)), math::max(0.01f, velIvar(1)) };

	if (!_initialised) {
		// try to initialise
		_initialised =	initialise(velI, velIvar_constrained, true_airspeed);
		return;
	}

	// don't fuse faster than 10Hz
	if (time_now - _time_last_airspeed_fuse < 100 * 1000) {
		return;
	}

	_time_last_airspeed_fuse = time_now;

	// assign helper variables
	const float v_n = velI(0);
	const float v_e = velI(1);
	const float v_d = velI(2);

	// calculate airspeed from ground speed and wind states (without scale)
	const float airspeed_predicted_raw = sqrtf((v_n - _state(INDEX_W_N)) * (v_n - _state(INDEX_W_N)) +
					     (v_e - _state(INDEX_W_E)) * (v_e - _state(INDEX_W_E)) + v_d * v_d);

	// compute state observation matrix H
	const float HH0 = airspeed_predicted_raw;
	const float HH1 = _state(INDEX_TAS_SCALE) / HH0;

	matrix::Matrix<float, 1, 3> H_tas;
	H_tas(0, 0) = HH1 * (-v_n + _state(INDEX_W_N));
	H_tas(0, 1) = HH1 * (-v_e + _state(INDEX_W_E));
	H_tas(0, 2) = HH0;

	// compute innovation covariance S
	const matrix::Matrix<float, 1, 1> S = H_tas * _P * H_tas.transpose() + _tas_var;

	// compute Kalman gain
	matrix::Matrix<float, 3, 1> K = _P * H_tas.transpose();
	K /= S(0, 0);

	if (_disable_tas_scale_estimate) {
		K(2, 0) = 0.f;
	}

	// compute innovation
	const float airspeed_pred = _state(INDEX_TAS_SCALE) * airspeed_predicted_raw;
	_tas_innov = true_airspeed - airspeed_pred;

	// innovation variance
	_tas_innov_var = S(0, 0);

	bool reinit_filter = false;
	bool meas_is_rejected = false;

	meas_is_rejected = check_if_meas_is_rejected(time_now, _tas_innov, _tas_innov_var, _tas_gate, _time_rejected_tas,
			   reinit_filter);

	reinit_filter |= _tas_innov_var < 0.0f;

	if (meas_is_rejected || reinit_filter) {
		if (reinit_filter) {
			_initialised =	initialise(velI, matrix::Vector2f(0.1f, 0.1f), true_airspeed);
		}

		// we either did a filter reset or the current measurement was rejected so do not fuse
		return;
	}

	// apply correction to state
	_state(INDEX_W_N) += _tas_innov * K(0, 0);
	_state(INDEX_W_E) += _tas_innov * K(1, 0);
	_state(INDEX_TAS_SCALE) += _tas_innov * K(2, 0);

	// update covariance matrix
	_P = _P - K * H_tas * _P;

	run_sanity_checks();
}

void
WindEstimator::fuse_beta(uint64_t time_now, const matrix::Vector3f &velI, const matrix::Quatf &q_att)
{
	if (!_initialised) {
		_initialised =	initialise(velI, matrix::Vector2f(0.1f, 0.1f), velI.length());
		return;
	}

	// don't fuse faster than 10Hz
	if (time_now - _time_last_beta_fuse < 100 * 1000) {
		return;
	}

	_time_last_beta_fuse = time_now;

	const float v_n = velI(0);
	const float v_e = velI(1);
	const float v_d = velI(2);

	// compute sideslip observation vector
	float HB0 = 2.0f * q_att(0);
	float HB1 = HB0 * q_att(3);
	float HB2 = 2.0f * q_att(1);
	float HB3 = HB2 * q_att(2);
	float HB4 = v_e - _state(INDEX_W_E);
	float HB5 = HB1 + HB3;
	float HB6 = v_n - _state(INDEX_W_N);
	float HB7 = q_att(0) * q_att(0);
	float HB8 = q_att(3) * q_att(3);
	float HB9 = HB7 - HB8;
	float HB10 = q_att(1) * q_att(1);
	float HB11 = q_att(2) * q_att(2);
	float HB12 = HB10 - HB11;
	float HB13 = HB12 + HB9;
	float HB14 = HB13 * HB6 + HB4 * HB5 + v_d * (-HB0 * q_att(2) + HB2 * q_att(3));
	float HB15 = 1.0f / HB14;
	float HB16 = (HB4 * (-HB10 + HB11 + HB9) + HB6 * (-HB1 + HB3) + v_d * (HB0 * q_att(1) + 2.0f * q_att(2) * q_att(3))) /
		     (HB14 * HB14);

	matrix::Matrix<float, 1, 3> H_beta;
	H_beta(0, 0) = HB13 * HB16 + HB15 * (HB1 - HB3);
	H_beta(0, 1) = HB15 * (HB12 - HB7 + HB8) + HB16 * HB5;
	H_beta(0, 2) = 0;

	// compute innovation covariance S
	const matrix::Matrix<float, 1, 1> S = H_beta * _P * H_beta.transpose() + _beta_var;

	// compute Kalman gain
	matrix::Matrix<float, 3, 1> K = _P * H_beta.transpose();
	K /= S(0, 0);

	if (_disable_tas_scale_estimate) {
		K(2, 0) = 0.f;
	}

	// compute predicted side slip angle
	matrix::Vector3f rel_wind(velI(0) - _state(INDEX_W_N), velI(1) - _state(INDEX_W_E), velI(2));
	matrix::Dcmf R_body_to_earth(q_att);
	rel_wind = R_body_to_earth.transpose() * rel_wind;

	if (fabsf(rel_wind(0)) < 0.1f) {
		return;
	}

	// use small angle approximation, sin(x) = x for small x
	const float beta_pred = rel_wind(1) / rel_wind(0);

	_beta_innov = 0.0f - beta_pred;
	_beta_innov_var = S(0, 0);

	bool reinit_filter = false;
	bool meas_is_rejected = false;

	meas_is_rejected = check_if_meas_is_rejected(time_now, _beta_innov, _beta_innov_var, _beta_gate, _time_rejected_beta,
			   reinit_filter);

	reinit_filter |= _beta_innov_var < 0.0f;

	if (meas_is_rejected || reinit_filter) {
		if (reinit_filter) {
			_initialised =	initialise(velI, matrix::Vector2f(0.1f, 0.1f), velI.length());
		}

		// we either did a filter reset or the current measurement was rejected so do not fuse
		return;
	}

	// apply correction to state
	_state(INDEX_W_N) += _beta_innov * K(0, 0);
	_state(INDEX_W_E) += _beta_innov * K(1, 0);
	_state(INDEX_TAS_SCALE) += _beta_innov * K(2, 0);

	// update covariance matrix
	_P = _P - K * H_beta * _P;

	run_sanity_checks();
}

void
WindEstimator::run_sanity_checks()
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

	if (!PX4_ISFINITE(_state(INDEX_W_N)) || !PX4_ISFINITE(_state(INDEX_W_E)) || !PX4_ISFINITE(_state(INDEX_TAS_SCALE))) {
		_initialised = false;
		return;
	}

	// attain symmetry
	for (unsigned row = 0; row < 3; row++) {
		for (unsigned column = 0; column < row; column++) {
			float tmp = (_P(row, column) + _P(column, row)) / 2;
			_P(row, column) = tmp;
			_P(column, row) = tmp;
		}
	}
}

bool
WindEstimator::check_if_meas_is_rejected(uint64_t time_now, float innov, float innov_var, uint8_t gate_size,
		uint64_t &time_meas_rejected, bool &reinit_filter)
{
	if (innov * innov > gate_size * gate_size * innov_var) {
		time_meas_rejected = time_meas_rejected == 0 ? time_now : time_meas_rejected;

	} else {
		time_meas_rejected = 0;
	}

	reinit_filter = time_now - time_meas_rejected > 5 * 1000 * 1000 && time_meas_rejected != 0;

	return time_meas_rejected != 0;
}
