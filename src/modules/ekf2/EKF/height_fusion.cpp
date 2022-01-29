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
	// vertical position innovation - baro measurement has opposite sign to earth z axis
	const float unbiased_baro = _baro_sample_delayed.hgt - _baro_b_est.getBias();

	_baro_hgt_innov = _state.pos(2) + unbiased_baro - _baro_hgt_offset;

	// Compensate for positive static pressure transients (negative vertical position innovations)
	// caused by rotor wash ground interaction by applying a temporary deadzone to baro innovations.
	const float deadzone_start = 0.0f;
	const float deadzone_end = deadzone_start + _params.gnd_effect_deadzone;

	if (_control_status.flags.gnd_effect) {
		if (_baro_hgt_innov < -deadzone_start) {
			if (_baro_hgt_innov <= -deadzone_end) {
				_baro_hgt_innov += deadzone_end;

			} else {
				_baro_hgt_innov = -deadzone_start;
			}
		}
	}

	// innovation gate size
	float innov_gate = fmaxf(_params.baro_innov_gate, 1.f);

	// observation variance - user parameter defined
	float obs_var = sq(fmaxf(_params.baro_noise, 0.01f));

	fuseVerticalPosition(_baro_hgt_innov, innov_gate, obs_var,
			     _baro_hgt_innov_var, _baro_hgt_test_ratio);
}

void Ekf::fuseGpsHgt()
{
	// vertical position innovation - gps measurement has opposite sign to earth z axis
	_gps_pos_innov(2) = _state.pos(2) + _gps_sample_delayed.hgt - _gps_alt_ref - _hgt_sensor_offset;

	// innovation gate size
	float innov_gate = fmaxf(_params.baro_innov_gate, 1.f);

	float obs_var = getGpsHeightVariance();

	// _gps_pos_test_ratio(1) is the vertical test ratio
	fuseVerticalPosition(_gps_pos_innov(2), innov_gate, obs_var,
			     _gps_pos_innov_var(2), _gps_pos_test_ratio(1));
}

void Ekf::fuseRngHgt()
{
	// use range finder with tilt correction
	_rng_hgt_innov = _state.pos(2) - (-math::max(_range_sensor.getDistBottom(),
					  _params.rng_gnd_clearance)) - _hgt_sensor_offset;

	// innovation gate size
	float innov_gate = fmaxf(_params.range_innov_gate, 1.f);

	// observation variance - user parameter defined
	float obs_var = fmaxf(sq(_params.range_noise) + sq(_params.range_noise_scaler * _range_sensor.getDistBottom()), 0.01f);

	fuseVerticalPosition(_rng_hgt_innov, innov_gate, obs_var,
			     _rng_hgt_innov_var, _rng_hgt_test_ratio);
}

void Ekf::fuseEvHgt()
{
	// calculate the innovation assuming the external vision observation is in local NED frame
	_ev_pos_innov(2) = _state.pos(2) - _ev_sample_delayed.pos(2);

	// innovation gate size
	float innov_gate = fmaxf(_params.ev_pos_innov_gate, 1.f);

	// observation variance - defined externally
	float obs_var = fmaxf(_ev_sample_delayed.posVar(2), sq(0.01f));

	// _ev_pos_test_ratio(1) is the vertical test ratio
	fuseVerticalPosition(_ev_pos_innov(2), innov_gate, obs_var,
			     _ev_pos_innov_var(2), _ev_pos_test_ratio(1));
}
