/****************************************************************************
 *
 *   Copyright (c) 2019 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file range_finder_checks.cpp
 * Perform checks on range finder data in order to evaluate validity.
 *
 *
 */

#include "ekf.h"

// check that the range finder data is continuous
void Ekf::updateRangeDataContinuity()
{
	// update range data continuous flag (1Hz ie 2000 ms)
	/* Timing in micro seconds */

	/* Apply a 2.0 sec low pass filter to the time delta from the last range finder updates */
	float alpha = 0.5f * _dt_update;
	_dt_last_range_update_filt_us = _dt_last_range_update_filt_us * (1.0f - alpha) + alpha *
					(_imu_sample_delayed.time_us - _range_sample_delayed.time_us);

	_dt_last_range_update_filt_us = fminf(_dt_last_range_update_filt_us, 4e6f);
}

void Ekf::updateRangeDataValidity()
{
	updateRangeDataContinuity();

	// check if out of date
	if ((_imu_sample_delayed.time_us - _range_sample_delayed.time_us) > 2 * RNG_MAX_INTERVAL) {
		_rng_hgt_valid = false;
		return;
	}

	// Don't allow faulty flag to clear unless range data is continuous
	if (!_rng_hgt_valid && !isRangeDataContinuous()) {
		return;
	}

	// Don't run the checks after this unless we have retrieved new data from the buffer
	if (!_range_data_ready) {
		return;
	}

	if (_range_sample_delayed.quality == 0) {
		_time_bad_rng_signal_quality = _imu_sample_delayed.time_us;
		_rng_hgt_valid = false;
	} else if (_imu_sample_delayed.time_us - _time_bad_rng_signal_quality > (unsigned)_params.range_signal_hysteresis_ms) {
		_rng_hgt_valid = true;
	}

	// Check if excessively tilted
	if (_R_rng_to_earth_2_2 < _params.range_cos_max_tilt) {
		_rng_hgt_valid = false;
		return;
	}

	// Check if out of range
	if ((_range_sample_delayed.rng > _rng_valid_max_val)
	|| (_range_sample_delayed.rng < _rng_valid_min_val)) {
		if (_control_status.flags.in_air) {
			_rng_hgt_valid = false;
			return;
		} else {
			// Range finders can fail to provide valid readings when resting on the ground
			// or being handled by the user, which prevents use of as a primary height sensor.
			// To work around this issue, we replace out of range data with the expected on ground value.
			_range_sample_delayed.rng = _params.rng_gnd_clearance;
			return;
		}
	}

	updateRangeDataStuck();

	_rng_hgt_valid = _rng_hgt_valid && !_control_status.flags.rng_stuck;
}

void Ekf::updateRangeDataStuck()
{
	// Check for "stuck" range finder measurements when range was not valid for certain period
	// This handles a failure mode observed with some lidar sensors
	if (((_range_sample_delayed.time_us - _time_last_rng_ready) > (uint64_t)10e6) &&
	    _control_status.flags.in_air) {

		// require a variance of rangefinder values to check for "stuck" measurements
		if (_rng_stuck_max_val - _rng_stuck_min_val > _params.range_stuck_threshold) {
			_time_last_rng_ready = _range_sample_delayed.time_us;
			_rng_stuck_min_val = 0.0f;
			_rng_stuck_max_val = 0.0f;
			_control_status.flags.rng_stuck = false;

		} else {
			if (_range_sample_delayed.rng > _rng_stuck_max_val) {
				_rng_stuck_max_val = _range_sample_delayed.rng;
			}

			if (_rng_stuck_min_val < 0.1f || _range_sample_delayed.rng < _rng_stuck_min_val) {
				_rng_stuck_min_val = _range_sample_delayed.rng;
			}

			_control_status.flags.rng_stuck = true;
		}

	} else {
		_time_last_rng_ready = _range_sample_delayed.time_us;
	}
}
