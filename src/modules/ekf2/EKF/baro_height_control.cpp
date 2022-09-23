/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file baro_height_control.cpp
 * Control functions for ekf barometric height fusion
 */

#include "ekf.h"

void Ekf::controlBaroHeightFusion()
{
	if (!_baro_buffer) {
		return;
	}

	baroSample baro_sample;
	const bool baro_data_ready = _baro_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &baro_sample);

	if (baro_data_ready) {
		if (_baro_counter == 0) {
			_baro_lpf.reset(baro_sample.hgt);

		} else {
			_baro_lpf.update(baro_sample.hgt);
		}

		if (_baro_counter < _obs_buffer_length) {
			// Initialize the pressure offset (included in the baro bias)
			_baro_b_est.setBias(_state.pos(2) + _baro_lpf.getState());
			_baro_counter++;
		}
	}

	if (!(_params.baro_ctrl == 1)) {
		stopBaroHgtFusion();
		return;
	}

	_baro_b_est.predict(_dt_ekf_avg);

	// check for intermittent data
	const bool baro_hgt_intermittent = !isNewestSampleRecent(_time_last_baro_buffer_push, 2 * BARO_MAX_INTERVAL);

	if (baro_data_ready) {
		updateBaroHgt(baro_sample, _aid_src_baro_hgt);

		const bool continuing_conditions_passing = !_baro_hgt_faulty && !baro_hgt_intermittent;
		const bool starting_conditions_passing = continuing_conditions_passing && (_baro_counter >= _obs_buffer_length);

		if (_control_status.flags.baro_hgt) {
			if (continuing_conditions_passing) {
				fuseBaroHgt(_aid_src_baro_hgt);

				const bool is_fusion_failing = isTimedOut(_aid_src_baro_hgt.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()) {
					// All height sources are failing
					resetHeightToBaro(baro_sample);
					resetVerticalVelocityToZero();

				} else if (is_fusion_failing) {
					// Some other height source is still working
					stopBaroHgtFusion();
					_baro_hgt_faulty = true;
				}

			} else {
				stopBaroHgtFusion();
			}
		} else {
			if (starting_conditions_passing) {
				startBaroHgtFusion(baro_sample);
			}
		}

	} else if (_control_status.flags.baro_hgt && baro_hgt_intermittent) {
		// No baro data anymore. Stop until it comes back.
		stopBaroHgtFusion();
	}
}

void Ekf::startBaroHgtFusion(const baroSample &baro_sample)
{
	if (!_control_status.flags.baro_hgt) {
		if (_params.height_sensor_ref == HeightSensor::BARO) {
			_height_sensor_ref = HeightSensor::BARO;
			resetHeightToBaro(baro_sample);

		} else {
			_baro_b_est.setBias(_state.pos(2) + _baro_lpf.getState());
		}

		_control_status.flags.baro_hgt = true;
		_baro_b_est.setFusionActive();
		ECL_INFO("starting baro height fusion");
	}
}

void Ekf::resetHeightToBaro(const baroSample &baro_sample)
{
	ECL_INFO("reset height to baro");
	_information_events.flags.reset_hgt_to_baro = true;

	resetVerticalPositionTo(-(baro_sample.hgt - _baro_b_est.getBias()));

	// the state variance is the same as the observation
	P.uncorrelateCovarianceSetVariance<1>(9, sq(_params.baro_noise));

	_gps_hgt_b_est.setBias(_gps_hgt_b_est.getBias() + _state_reset_status.posD_change);
	_rng_hgt_b_est.setBias(_rng_hgt_b_est.getBias() + _state_reset_status.posD_change);
	_ev_hgt_b_est.setBias(_ev_hgt_b_est.getBias() - _state_reset_status.posD_change);
}

void Ekf::stopBaroHgtFusion()
{
	if (_control_status.flags.baro_hgt) {
		if (_height_sensor_ref == HeightSensor::BARO) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_control_status.flags.baro_hgt = false;
		_baro_b_est.setFusionInactive();
		ECL_INFO("stopping baro height fusion");
	}
}
