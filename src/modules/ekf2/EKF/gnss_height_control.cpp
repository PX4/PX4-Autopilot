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
 * @file gnss_height_control.cpp
 * Control functions for ekf GNSS height fusion
 */

#include "ekf.h"

void Ekf::controlGnssHeightFusion(const gpsSample &gps_sample)
{
	if (!(_params.gnss_ctrl & GnssCtrl::VPOS)) {
		stopGpsHgtFusion();
		return;
	}

	auto &aid_src = _aid_src_gnss_hgt;
	HeightBiasEstimator &bias_est = _gps_hgt_b_est;

	bias_est.predict(_dt_ekf_avg);

	if (_gps_data_ready) {

		const bool gps_checks_passing = isTimedOut(_last_gps_fail_us, (uint64_t)5e6);
		const bool gps_checks_failing = isTimedOut(_last_gps_pass_us, (uint64_t)5e6);

		const float innov_gate = fmaxf(_params.gps_pos_innov_gate, 1.f);

		const float measurement = gps_sample.hgt - getEkfGlobalOriginAltitude();
		const float measurement_var = getGpsHeightVariance(gps_sample);

		// GNSS position, vertical position GNSS measurement has opposite sign to earth z axis
		updateVerticalPositionAidSrcStatus(gps_sample.time_us,
						   -(measurement - bias_est.getBias()),
						   measurement_var + bias_est.getBiasVar(),
						   innov_gate,
						   aid_src);

		// update the bias estimator before updating the main filter but after
		// using its current state to compute the vertical position innovation
		if (gps_checks_passing && !gps_checks_failing
		    && PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var)
		   ) {
			const float noise = sqrtf(measurement_var);
			bias_est.setMaxStateNoise(noise);
			bias_est.setProcessNoiseSpectralDensity(_params.gps_hgt_bias_nsd);
			bias_est.fuseBias(measurement - (-_state.pos(2)), measurement_var + P(9, 9));
		}

		// determine if we should use GNSS height aiding
		const bool continuing_conditions_passing = (_params.gnss_ctrl & GnssCtrl::VPOS)
				&& PX4_ISFINITE(gps_sample.hgt)
				&& _NED_origin_initialised
				&& _gps_checks_passed;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& _gps_checks_passed
				&& gps_checks_passing
				&& !gps_checks_failing
				&& isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GPS_MAX_INTERVAL);

		if (_control_status.flags.gps_hgt) {
			aid_src.fusion_enabled = true;

			if (continuing_conditions_passing) {

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()) {
					// All height sources are failing
					resetHeightToGps(gps_sample);
					resetVerticalVelocityToGps(gps_sample);

				} else if (is_fusion_failing) {
					// Some other height source is still working
					stopGpsHgtFusion();
				}

			} else {
				stopGpsHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				startGpsHgtFusion(gps_sample);
			}
		}

	} else if (_control_status.flags.gps_hgt && _gps_intermittent) {
		stopGpsHgtFusion();
	}
}

void Ekf::startGpsHgtFusion(const gpsSample &gps_sample)
{
	if (!_control_status.flags.gps_hgt) {

		if (_params.height_sensor_ref == HeightSensor::GNSS) {
			_gps_hgt_b_est.reset();
			_height_sensor_ref = HeightSensor::GNSS;
			resetHeightToGps(gps_sample);

		} else {
			_gps_hgt_b_est.setBias(_state.pos(2) + (gps_sample.hgt - getEkfGlobalOriginAltitude()));

			// Reset the timeout value here because the fusion isn't done at the same place and would immediately trigger a timeout
			_aid_src_gnss_hgt.time_last_fuse = _imu_sample_delayed.time_us;
		}

		_control_status.flags.gps_hgt = true;
		_gps_hgt_b_est.setFusionActive();
		ECL_INFO("starting GPS height fusion");
	}
}

void Ekf::resetHeightToGps(const gpsSample &gps_sample)
{
	ECL_INFO("reset height to GPS");
	_information_events.flags.reset_hgt_to_gps = true;

	resetVerticalPositionTo(-(gps_sample.hgt - getEkfGlobalOriginAltitude() - _gps_hgt_b_est.getBias()));

	// the state variance is the same as the observation
	P.uncorrelateCovarianceSetVariance<1>(9, getGpsHeightVariance(gps_sample));

	_baro_b_est.setBias(_baro_b_est.getBias() + _state_reset_status.posD_change);
	_rng_hgt_b_est.setBias(_rng_hgt_b_est.getBias() + _state_reset_status.posD_change);
	_ev_hgt_b_est.setBias(_ev_hgt_b_est.getBias() - _state_reset_status.posD_change);

	_aid_src_gnss_hgt.time_last_fuse = _imu_sample_delayed.time_us;
}

void Ekf::stopGpsHgtFusion()
{
	if (_control_status.flags.gps_hgt) {

		if (_height_sensor_ref == HeightSensor::GNSS) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_control_status.flags.gps_hgt = false;
		_gps_hgt_b_est.setFusionInactive();
		ECL_INFO("stopping GPS height fusion");
	}
}
