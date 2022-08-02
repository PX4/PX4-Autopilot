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
WindEstimator::initialise(const matrix::Vector3f &velI, const float hor_vel_variance, const float heading_rad,
			  const float tas_meas, const float tas_variance)
{
	if (PX4_ISFINITE(tas_meas) && PX4_ISFINITE(tas_variance)) {

		const float cos_heading = cosf(heading_rad);
		const float sin_heading = sinf(heading_rad);

		// initialise wind states assuming zero side slip and horizontal flight
		_state(INDEX_W_N) = velI(0) - tas_meas * cos_heading;
		_state(INDEX_W_E) = velI(1) - tas_meas * sin_heading;
		_state(INDEX_TAS_SCALE) = _scale_init;

		constexpr float initial_sideslip_uncertainty = math::radians(INITIAL_BETA_ERROR_DEG);
		const float initial_wind_var_body_y = sq(tas_meas * sinf(initial_sideslip_uncertainty));
		constexpr float heading_variance = sq(math::radians(INITIAL_HEADING_ERROR_DEG));

		// rotate wind velocity into earth frame aligned with vehicle yaw
		const float Wx = _state(INDEX_W_N) * cos_heading + _state(INDEX_W_E) * sin_heading;
		const float Wy = -_state(INDEX_W_N) * sin_heading + _state(INDEX_W_E) * cos_heading;

		_P(INDEX_W_N, INDEX_W_N) = tas_variance * sq(cos_heading) + heading_variance * sq(-Wx * sin_heading - Wy * cos_heading)
					   + initial_wind_var_body_y * sq(sin_heading);
		_P(INDEX_W_N, INDEX_W_E) = tas_variance * sin_heading * cos_heading + heading_variance *
					   (-Wx * sin_heading - Wy * cos_heading) * (Wx * cos_heading - Wy * sin_heading) -
					   initial_wind_var_body_y * sin_heading * cos_heading;
		_P(INDEX_W_E, INDEX_W_N) = _P(INDEX_W_N, INDEX_W_E);
		_P(INDEX_W_E, INDEX_W_E) = tas_variance * sq(sin_heading) + heading_variance * sq(Wx * cos_heading - Wy * sin_heading) +
					   initial_wind_var_body_y * sq(cos_heading);

		// Now add the variance due to uncertainty in vehicle velocity that was used to calculate the initial wind speed
		_P(INDEX_W_N, INDEX_W_N) += hor_vel_variance;
		_P(INDEX_W_E, INDEX_W_E) += hor_vel_variance;

	} else {
		// no airspeed available
		_state.setZero();
		_state(INDEX_TAS_SCALE) = 1.0f;
		_P.setZero();
		_P(INDEX_W_N, INDEX_W_N) = _P(INDEX_W_E, INDEX_W_E) = sq(INITIAL_WIND_ERROR);
	}

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
	if (time_now - _time_last_update < 1_s || _time_last_update == 0) {
		if (_time_last_update == 0) {
			_time_last_update = time_now;
		}

		return;
	}

	float dt = (float)(time_now - _time_last_update) * 1e-6f;
	_time_last_update = time_now;

	matrix::Matrix3f Qk;
	Qk(0, 0) = _wind_psd * dt;
	Qk(1, 1) = Qk(0, 0);
	Qk(2, 2) = _tas_scale_psd * dt;
	_P += Qk;
}

void
WindEstimator::fuse_airspeed(uint64_t time_now, const float true_airspeed, const matrix::Vector3f &velI,
			     const float hor_vel_variance, const matrix::Quatf &q_att)
{

	if (!_initialised) {
		// try to initialise
		_initialised = initialise(velI, hor_vel_variance, matrix::Eulerf(q_att).psi(), true_airspeed, _tas_var);
		return;
	}

	// don't fuse faster than 10Hz
	if (time_now - _time_last_airspeed_fuse < 100_ms) {
		return;
	}

	_time_last_airspeed_fuse = time_now;

	matrix::Matrix<float, 1, 3> H_tas;
	matrix::Matrix<float, 3, 1> K;

	sym::FuseAirspeed(velI, _state, _P, true_airspeed, _tas_var, FLT_EPSILON,
			  &H_tas, &K, &_tas_innov_var, &_tas_innov);

	bool reinit_filter = false;
	bool meas_is_rejected = false;

	// note: _time_rejected_tas and reinit_filter are not used anymore as filter can't get reset due to tas rejection
	meas_is_rejected = check_if_meas_is_rejected(time_now, _tas_innov, _tas_innov_var, _tas_gate, _time_rejected_tas,
			   reinit_filter);

	if (meas_is_rejected || _tas_innov_var < 0.f) {
		// only reset filter if _tas_innov_var gets unfeasible
		if (_tas_innov_var < 0.0f) {
			_initialised = initialise(velI, hor_vel_variance, matrix::Eulerf(q_att).psi(), true_airspeed, _tas_var);
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
WindEstimator::fuse_beta(uint64_t time_now, const matrix::Vector3f &velI, const float hor_vel_variance,
			 const matrix::Quatf &q_att)
{
	if (!_initialised) {
		_initialised = initialise(velI, hor_vel_variance, matrix::Eulerf(q_att).psi());
		return;
	}

	// don't fuse faster than 10Hz
	if (time_now - _time_last_beta_fuse < 100_ms) {
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
			_initialised = initialise(velI, hor_vel_variance, matrix::Eulerf(q_att).psi());
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

	reinit_filter = time_now - time_meas_rejected > 5_s && time_meas_rejected != 0;

	return time_meas_rejected != 0;
}
