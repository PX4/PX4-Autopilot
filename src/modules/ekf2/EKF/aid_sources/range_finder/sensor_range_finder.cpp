/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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
 * @file sensor_range_finder.cpp
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 *
 */

#include <aid_sources/range_finder/sensor_range_finder.hpp>

#include <lib/matrix/matrix/math.hpp>

namespace estimator
{
namespace sensor
{

void SensorRangeFinder::runChecks(const uint64_t current_time_us, const matrix::Dcmf &R_to_earth)
{
	updateSensorToEarthRotation(R_to_earth);
	updateValidity(current_time_us);
}

void SensorRangeFinder::updateSensorToEarthRotation(const matrix::Dcmf &R_to_earth)
{
	// calculate 2,2 element of rotation matrix from sensor frame to earth frame
	// this is required for use of range finder and flow data
	_cos_tilt_rng_to_earth = R_to_earth(2, 0) * _sin_pitch_offset + R_to_earth(2, 2) * _cos_pitch_offset;
}

void SensorRangeFinder::updateValidity(uint64_t current_time_us)
{
	updateDtDataLpf(current_time_us);

	if (_is_faulty || isSampleOutOfDate(current_time_us) || !isDataContinuous()) {
		_is_sample_valid = false;
		_is_regularly_sending_data = false;
		return;
	}

	_is_regularly_sending_data = true;

	// Don't run the checks unless we have retrieved new data from the buffer
	if (_is_sample_ready) {
		_is_sample_valid = false;

		_time_bad_quality_us = _sample.quality == 0 ? current_time_us : _time_bad_quality_us;

		if (!isQualityOk(current_time_us) || !isTiltOk() || !isDataInRange()) {
			return;
		}

		updateStuckCheck();
		updateFogCheck(getDistBottom(), _sample.time_us);

		if (!_is_stuck && !_is_blocked) {
			_is_sample_valid = true;
			_time_last_valid_us = _sample.time_us;
		}
	}
}

bool SensorRangeFinder::isQualityOk(uint64_t current_time_us) const
{
	return current_time_us - _time_bad_quality_us > _quality_hyst_us;
}

void SensorRangeFinder::updateDtDataLpf(uint64_t current_time_us)
{
	// Calculate a first order IIR low-pass filtered time of arrival between samples using a 2 second time constant.
	float alpha = 0.5f * _dt_update;
	_dt_data_lpf = _dt_data_lpf * (1.0f - alpha) + alpha * (current_time_us - _sample.time_us);

	// Apply spike protection to the filter state.
	_dt_data_lpf = fminf(_dt_data_lpf, 4e6f);
}

inline bool SensorRangeFinder::isSampleOutOfDate(uint64_t current_time_us) const
{
	return (current_time_us - _sample.time_us) > 2 * RNG_MAX_INTERVAL;
}

inline bool SensorRangeFinder::isDataInRange() const
{
	return (_sample.rng >= _rng_valid_min_val) && (_sample.rng <= _rng_valid_max_val);
}

void SensorRangeFinder::updateStuckCheck()
{
	if (!isStuckDetectorEnabled()) {
		// Stuck detector disabled
		_is_stuck = false;
		return;
	}

	// Check for "stuck" range finder measurements when range was not valid for certain period
	// This handles a failure mode observed with some lidar sensors
	if (((_sample.time_us - _time_last_valid_us) > (uint64_t)10e6)) {

		// require a variance of rangefinder values to check for "stuck" measurements
		if (_stuck_max_val - _stuck_min_val > _stuck_threshold) {
			_stuck_min_val = 0.0f;
			_stuck_max_val = 0.0f;
			_is_stuck = false;

		} else {
			if (_sample.rng > _stuck_max_val) {
				_stuck_max_val = _sample.rng;
			}

			if (_stuck_min_val < 0.1f || _sample.rng < _stuck_min_val) {
				_stuck_min_val = _sample.rng;
			}

			_is_stuck = true;
		}
	}
}

void SensorRangeFinder::updateFogCheck(const float dist_bottom, const uint64_t time_us)
{
	if (_max_fog_dist > 0.f) {

		const float median_dist = _median_dist.apply(dist_bottom);
		const float factor = 2.f; // magic hardcoded factor

		if (!_is_blocked && median_dist < _max_fog_dist && _prev_median_dist - median_dist > factor * _max_fog_dist) {
			_is_blocked = true;

		} else if (_is_blocked && median_dist > factor * _max_fog_dist) {
			_is_blocked = false;
		}

		_prev_median_dist = median_dist;
	}
}

} // namespace sensor
} // namespace estimator
