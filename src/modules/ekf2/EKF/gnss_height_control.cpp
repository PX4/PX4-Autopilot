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

	_gps_hgt_b_est.predict(_dt_ekf_avg);

	if (_gps_data_ready) {
		const bool continuing_conditions_passing = !_gps_intermittent && _gps_checks_passed && _NED_origin_initialised;
		const bool starting_conditions_passing = continuing_conditions_passing;

		if (_control_status.flags.gps_hgt) {
			if (continuing_conditions_passing) {
				/* fuseGpsHgt(); */ // Done in fuseGpsPos

				const bool is_fusion_failing = isTimedOut(_aid_src_gnss_pos.time_last_fuse[2], _params.hgt_fusion_timeout_max);

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
			_aid_src_gnss_pos.time_last_fuse[2] = _imu_sample_delayed.time_us;
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

	_aid_src_gnss_pos.time_last_fuse[2] = _imu_sample_delayed.time_us;
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
