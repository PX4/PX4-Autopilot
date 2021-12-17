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

	if (!PX4_ISFINITE(_baro_hgt_offset)) {
		_baro_hgt_offset = _state.pos(2) + unbiased_baro;
	}

	_baro_hgt_innov = _state.pos(2) + unbiased_baro - _baro_hgt_offset;

	if (!_control_status.flags.baro_hgt) {
		// apply a 10 second first order low pass filter to baro offset
		const float local_time_step = math::constrain(1e-6f * _delta_time_baro_us, 0.f, 1.f);
		const float offset_rate_correction = 0.1f * _baro_hgt_innov;
		_baro_hgt_offset += local_time_step * math::constrain(offset_rate_correction, -0.1f, 0.1f);
	}

	// Compensate for positive static pressure transients (negative vertical position innovations)
	// caused by rotor wash ground interaction by applying a temporary deadzone to baro innovations.
	if (_control_status.flags.gnd_effect && _control_status.flags.in_air && _params.gnd_effect_deadzone > 0.f) {

		const float deadzone_start = 0.0f;
		const float deadzone_end = deadzone_start + _params.gnd_effect_deadzone;

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

	_baro_hgt_innov_var = P(9, 9) + obs_var;
	_baro_hgt_test_ratio = sq(_baro_hgt_innov) / (sq(innov_gate) * _baro_hgt_innov_var);

	_vert_pos_innov_ratio_baro = _baro_hgt_innov / sqrtf(_baro_hgt_innov_var);
	_vert_pos_baro_fuse_attempt_time_us = _time_last_imu;

	if (_control_status.flags.baro_hgt) {
		if (fuseVerticalPosition(_baro_hgt_innov, innov_gate, obs_var, _baro_hgt_innov_var, _baro_hgt_test_ratio)) {
			_time_last_baro_hgt_fuse = _time_last_imu;
		}
	}

	if (_baro_hgt_test_ratio < 1.f) {
		_innov_check_fail_status.flags.reject_ver_pos_baro = false;

	} else {
		_innov_check_fail_status.flags.reject_ver_pos_baro = true;
	}
}

void Ekf::fuseGpsHgt()
{
	if (!PX4_ISFINITE(_gps_hgt_offset)) {
		_gps_hgt_offset = _state.pos(2) + _gps_sample_delayed.hgt - _gps_alt_ref;
	}

	// vertical position innovation - gps measurement has opposite sign to earth z axis
	float innov = _state.pos(2) + _gps_sample_delayed.hgt - _gps_alt_ref - _gps_hgt_offset;

	if (!_control_status.flags.gps_hgt) {
		// apply a 10 second first order low pass filter to offset
		const float local_time_step = math::constrain(1e-6f * _delta_time_gps_us, 0.f, 1.f);
		const float offset_rate_correction = 0.1f * innov;
		_gps_hgt_offset += local_time_step * math::constrain(offset_rate_correction, -0.1f, 0.1f);
	}

	// innovation gate size
	float innov_gate = fmaxf(_params.baro_innov_gate, 1.f);

	float obs_var = getGpsHeightVariance();
	float innov_var = P(9, 9) + obs_var;
	float test_ratio = sq(innov) / (sq(innov_gate) * innov_var);

	_vert_pos_innov_ratio_gps = innov / sqrtf(innov_var);
	_vert_pos_gps_fuse_attempt_time_us = _time_last_imu;

	if (_control_status.flags.baro_hgt) {
		if (fuseVerticalPosition(innov, innov_gate, obs_var, innov_var, test_ratio)) {
			_time_last_gps_hgt_fuse = _time_last_imu;
		}
	}

	_gps_pos_innov(2) = innov;
	_gps_pos_innov_var(2) = innov_var;
	_gps_pos_test_ratio(2) = test_ratio;

	if (test_ratio < 1.f) {
		_innov_check_fail_status.flags.reject_ver_pos_gps = false;

	} else {
		_innov_check_fail_status.flags.reject_ver_pos_gps = true;
	}
}

void Ekf::fuseRngHgt()
{
	if (!PX4_ISFINITE(_rng_hgt_offset)) {
		_rng_hgt_offset = _state.pos(2) - (-math::max(_range_sensor.getDistBottom(), _params.rng_gnd_clearance));
	}

	// use range finder with tilt correction
	_rng_hgt_innov = _state.pos(2) - (-math::max(_range_sensor.getDistBottom(),
					     _params.rng_gnd_clearance)) - _rng_hgt_offset;

	if (!_control_status.flags.rng_hgt) {
		// apply a 10 second first order low pass filter to offset
		const float local_time_step = math::constrain(1e-6f * _delta_time_rng_us, 0.f, 1.f);
		const float offset_rate_correction = 0.1f * _rng_hgt_innov;
		_rng_hgt_offset += local_time_step * math::constrain(offset_rate_correction, -0.1f, 0.1f);
	}

	// innovation gate size
	float innov_gate = fmaxf(_params.range_innov_gate, 1.f);

	// observation variance - user parameter defined
	float obs_var = fmaxf(sq(_params.range_noise)
				   + sq(_params.range_noise_scaler * _range_sensor.getDistBottom()), 0.01f);

	_rng_hgt_innov_var = P(9, 9) + obs_var;

	_rng_hgt_test_ratio = sq(_rng_hgt_innov) / (sq(innov_gate) * _rng_hgt_innov_var);

	_vert_pos_innov_ratio_rng = _rng_hgt_innov / sqrtf(_rng_hgt_innov_var);
	_vert_pos_rng_fuse_attempt_time_us = _time_last_imu;

	if (_control_status.flags.rng_hgt) {
		if (fuseVerticalPosition(_rng_hgt_innov, innov_gate, obs_var, _rng_hgt_innov_var, _rng_hgt_test_ratio)) {
			_time_last_rng_hgt_fuse = _time_last_imu;
		}
	}

	if (_rng_hgt_test_ratio < 1.f) {
		_innov_check_fail_status.flags.reject_ver_pos_rng = false;

	} else {
		_innov_check_fail_status.flags.reject_ver_pos_rng = true;
	}
}

void Ekf::fuseEvHgt()
{
	if (!PX4_ISFINITE(_ev_hgt_offset)) {
		_ev_hgt_offset = _state.pos(2) - _ev_sample_delayed.pos(2);
	}

	// calculate the innovation assuming the external vision observation is in local NED frame
	float innov = _state.pos(2) - _ev_sample_delayed.pos(2) - _ev_hgt_offset;

	if (!_control_status.flags.ev_hgt) {
		// apply a 10 second first order low pass filter to offset
		const float local_time_step = math::constrain(1e-6f * _delta_time_ev_us, 0.f, 1.f);
		const float offset_rate_correction = 0.1f * innov;
		_ev_hgt_offset += local_time_step * math::constrain(offset_rate_correction, -0.1f, 0.1f);
	}

	// innovation gate size
	float innov_gate = fmaxf(_params.ev_pos_innov_gate, 1.f);

	// observation variance - defined externally
	float obs_var = fmaxf(_ev_sample_delayed.posVar(2), sq(0.01f));
	float innov_var = P(9, 9) + obs_var;
	float test_ratio = sq(innov) / (sq(innov_gate) * innov_var);

	_vert_pos_innov_ratio_ev = innov / sqrtf(innov_var);
	_vert_pos_ev_fuse_attempt_time_us = _time_last_imu;

	if (_control_status.flags.ev_hgt) {
		if (fuseVerticalPosition(innov, innov_gate, obs_var, innov_var, test_ratio)) {
			_time_last_ev_hgt_fuse = _time_last_imu;
		}
	}

	_ev_pos_innov(2) = innov;
	_ev_pos_innov_var(2) = innov_var;
	_ev_pos_test_ratio(2) = test_ratio;

	if (test_ratio < 1.f) {
		_innov_check_fail_status.flags.reject_ver_pos_ev = false;

	} else {
		_innov_check_fail_status.flags.reject_ver_pos_ev = true;
	}
}
