/****************************************************************************
 *
 *   Copyright (c) 2021 PX4. All rights reserved.
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
 * @file height_fusion.cpp
 * Function for fusing height (range, baro, GNSS alt, ...) measurements
 */

#include "ekf.h"

void Ekf::fuseBaroHgt()
{
	Vector2f baro_hgt_innov_gate;
	Vector3f baro_hgt_obs_var;

	// vertical position innovation - baro measurement has opposite sign to earth z axis
	const float unbiased_baro = _baro_sample_delayed.hgt - _baro_b_est.getBias();
	_baro_hgt_innov(2) = _state.pos(2) + unbiased_baro - _baro_hgt_offset;
	// observation variance - user parameter defined
	baro_hgt_obs_var(2) = sq(fmaxf(_params.baro_noise, 0.01f));
	// innovation gate size
	baro_hgt_innov_gate(1) = fmaxf(_params.baro_innov_gate, 1.0f);

	// Compensate for positive static pressure transients (negative vertical position innovations)
	// caused by rotor wash ground interaction by applying a temporary deadzone to baro innovations.
	if (_control_status.flags.gnd_effect && _control_status.flags.in_air && _params.gnd_effect_deadzone > 0.f) {

		const float deadzone_start = 0.0f;
		const float deadzone_end = deadzone_start + _params.gnd_effect_deadzone;

		if (_baro_hgt_innov(2) < -deadzone_start) {
			if (_baro_hgt_innov(2) <= -deadzone_end) {
				_baro_hgt_innov(2) += deadzone_end;

			} else {
				_baro_hgt_innov(2) = -deadzone_start;
			}
		}
	}

	fuseVerticalPosition(_baro_hgt_innov, baro_hgt_innov_gate,
			     baro_hgt_obs_var, _baro_hgt_innov_var, _baro_hgt_test_ratio);
}

void Ekf::fuseGpsHgt()
{
	Vector2f gps_hgt_innov_gate;
	Vector3f gps_hgt_obs_var;
	// vertical position innovation - gps measurement has opposite sign to earth z axis
	_gps_pos_innov(2) = _state.pos(2) + _gps_sample_delayed.hgt - _gps_alt_ref - _hgt_sensor_offset;
	gps_hgt_obs_var(2) = getGpsHeightVariance();
	// innovation gate size
	gps_hgt_innov_gate(1) = fmaxf(_params.baro_innov_gate, 1.0f);

	fuseVerticalPosition(_gps_pos_innov, gps_hgt_innov_gate,
			     gps_hgt_obs_var, _gps_pos_innov_var, _gps_pos_test_ratio);
}

void Ekf::fuseRngHgt()
{
	Vector2f rng_hgt_innov_gate;
	Vector3f rng_hgt_obs_var;
	// use range finder with tilt correction
	_rng_hgt_innov(2) = _state.pos(2) - (-math::max(_range_sensor.getDistBottom(),
					     _params.rng_gnd_clearance)) - _hgt_sensor_offset;
	// observation variance - user parameter defined
	rng_hgt_obs_var(2) = fmaxf(sq(_params.range_noise)
				   + sq(_params.range_noise_scaler * _range_sensor.getDistBottom()), 0.01f);
	// innovation gate size
	rng_hgt_innov_gate(1) = fmaxf(_params.range_innov_gate, 1.0f);

	fuseVerticalPosition(_rng_hgt_innov, rng_hgt_innov_gate,
			     rng_hgt_obs_var, _rng_hgt_innov_var, _rng_hgt_test_ratio);
}

void Ekf::fuseEvHgt()
{
	Vector2f ev_hgt_innov_gate;
	Vector3f ev_hgt_obs_var;
	// calculate the innovation assuming the external vision observation is in local NED frame
	_ev_pos_innov(2) = _state.pos(2) - _ev_sample_delayed.pos(2);
	// observation variance - defined externally
	ev_hgt_obs_var(2) = fmaxf(_ev_sample_delayed.posVar(2), sq(0.01f));
	// innovation gate size
	ev_hgt_innov_gate(1) = fmaxf(_params.ev_pos_innov_gate, 1.0f);

	fuseVerticalPosition(_ev_pos_innov, ev_hgt_innov_gate,
			     ev_hgt_obs_var, _ev_pos_innov_var, _ev_pos_test_ratio);
}
