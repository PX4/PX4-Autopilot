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
 * @file gps_control.cpp
 * Control functions for ekf range finder height fusion
 */

#include "ekf.h"

void Ekf::controlRangeHeightFusion()
{
	if (!((_params.rng_ctrl == RngCtrl::CONDITIONAL) || (_params.rng_ctrl == RngCtrl::ENABLED))) {
		stopRngHgtFusion();
		return;
	}

	_rng_hgt_b_est.predict(_dt_ekf_avg);

	const bool rng_intermittent = !isNewestSampleRecent(_time_last_range_buffer_push, 2 * RNG_MAX_INTERVAL);

	// If we are supposed to be using range finder data as the primary height sensor, have bad range measurements
	// and are on the ground, then synthesise a measurement at the expected on ground value
	if (!_control_status.flags.in_air
	    && !_range_sensor.isDataHealthy()
	    && _range_sensor.isRegularlySendingData()
	    && _range_sensor.isDataReady()) {

		_range_sensor.setRange(_params.rng_gnd_clearance);
		_range_sensor.setValidity(true); // bypass the checks
	}

	if (_rng_data_ready) {
		updateRngHgt(_aid_src_rng_hgt);

		const bool do_conditional_range_aid = (_params.rng_ctrl == RngCtrl::CONDITIONAL) && isConditionalRangeAidSuitable();

		const bool continuing_conditions_passing = _range_sensor.isDataHealthy() && !rng_intermittent
							   && ((_params.rng_ctrl == RngCtrl::ENABLED) || do_conditional_range_aid);
		const bool starting_conditions_passing = continuing_conditions_passing
							 && _range_sensor.isRegularlySendingData();

		if (_control_status.flags.rng_hgt) {
			if (continuing_conditions_passing) {
				fuseRngHgt(_aid_src_rng_hgt);

				const bool is_fusion_failing = isTimedOut(_aid_src_rng_hgt.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()) {
					// All height sources are failing
					resetHeightToRng();
					resetVerticalVelocityToZero();

				} else if (is_fusion_failing) {
					// Some other height source is still working
					stopRngHgtFusion();
					_control_status.flags.rng_fault = true;
					_range_sensor.setFaulty();
				}

			} else {
				stopRngHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				startRngHgtFusion();
			}
		}

	} else if (_control_status.flags.rng_hgt && rng_intermittent) {
		stopRngHgtFusion();
	}
}

bool Ekf::isConditionalRangeAidSuitable()
{
	bool is_range_aid_suitable = false;

	if (_control_status.flags.in_air
	    && _range_sensor.isHealthy()
	    && isTerrainEstimateValid()) {
		// check if we can use range finder measurements to estimate height, use hysteresis to avoid rapid switching
		// Note that the 0.7 coefficients and the innovation check are arbitrary values but work well in practice
		const float range_hagl = _terrain_vpos - _state.pos(2);
		const float range_hagl_max = _control_status.flags.rng_hgt ? _params.max_hagl_for_range_aid : (_params.max_hagl_for_range_aid * 0.7f);
		const bool is_in_range = range_hagl < range_hagl_max;

		const float hagl_test_ratio = (_hagl_innov * _hagl_innov / (sq(_params.range_aid_innov_gate) * _hagl_innov_var));
		const bool is_hagl_stable = _control_status.flags.rng_hgt ? (hagl_test_ratio < 1.f) : (hagl_test_ratio < 0.01f);

		if (isHorizontalAidingActive()) {
			const float max_vel = _control_status.flags.rng_hgt ? _params.max_vel_for_range_aid : (_params.max_vel_for_range_aid * 0.7f);
			const bool is_below_max_speed = !_state.vel.xy().longerThan(max_vel);

			is_range_aid_suitable = is_in_range && is_hagl_stable && is_below_max_speed;

		} else {
			is_range_aid_suitable = is_in_range && is_hagl_stable;
		}

	}

	return is_range_aid_suitable;
}

void Ekf::startRngHgtFusion()
{
	if (!_control_status.flags.rng_hgt) {
		if ((_params.height_sensor_ref == HeightSensor::RANGE) && (_params.rng_ctrl == RngCtrl::CONDITIONAL)) {
			// Range finder is used while hovering to stabilize the height estimate. Don't reset but use it as height reference.
			_rng_hgt_b_est.setBias(_state.pos(2) + _range_sensor.getDistBottom());
			_height_sensor_ref = HeightSensor::RANGE;

		} else if ((_params.height_sensor_ref == HeightSensor::RANGE) && (_params.rng_ctrl != RngCtrl::CONDITIONAL)) {
			// Range finder is the primary height source, the ground is now the datum used
			// to compute the local vertical position
			_rng_hgt_b_est.reset();
			_height_sensor_ref = HeightSensor::RANGE;
			resetHeightToRng();

		} else {
			_rng_hgt_b_est.setBias(_state.pos(2) + _range_sensor.getDistBottom());
		}

		_control_status.flags.rng_hgt = true;
		_rng_hgt_b_est.setFusionActive();
		ECL_INFO("starting RNG height fusion");
	}
}

void Ekf::resetHeightToRng()
{
	ECL_INFO("reset height to RNG");
	_information_events.flags.reset_hgt_to_rng = true;

	float dist_bottom;

	if (_control_status.flags.in_air) {
		dist_bottom = _range_sensor.getDistBottom();

	} else {
		// use the parameter rng_gnd_clearance if on ground to avoid a noisy offset initialization (e.g. sonar)
		dist_bottom = _params.rng_gnd_clearance;
	}

	// update the state and associated variance
	resetVerticalPositionTo(-(dist_bottom - _rng_hgt_b_est.getBias()));

	// the state variance is the same as the observation
	P.uncorrelateCovarianceSetVariance<1>(9, sq(_params.range_noise));

	_baro_b_est.setBias(_baro_b_est.getBias() + _state_reset_status.posD_change);
	_gps_hgt_b_est.setBias(_gps_hgt_b_est.getBias() + _state_reset_status.posD_change);
	_ev_hgt_b_est.setBias(_ev_hgt_b_est.getBias() - _state_reset_status.posD_change);
}

void Ekf::stopRngHgtFusion()
{
	if (_control_status.flags.rng_hgt) {
		if (_height_sensor_ref == HeightSensor::RANGE) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_control_status.flags.rng_hgt = false;
		_rng_hgt_b_est.setFusionInactive();
		ECL_INFO("stopping range height fusion");
	}
}
