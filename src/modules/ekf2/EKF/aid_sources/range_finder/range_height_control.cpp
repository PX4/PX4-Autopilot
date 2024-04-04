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
 * @file range_height_control.cpp
 * Control functions for ekf range finder height fusion
 */

#include "ekf.h"

void Ekf::controlRangeHeightFusion()
{
	static constexpr const char *HGT_SRC_NAME = "RNG";

	bool rng_data_ready = false;

	if (_range_buffer) {
		// Get range data from buffer and check validity
		rng_data_ready = _range_buffer->pop_first_older_than(_time_delayed_us, _range_sensor.getSampleAddress());
		_range_sensor.setDataReadiness(rng_data_ready);

		// update range sensor angle parameters in case they have changed
		_range_sensor.setPitchOffset(_params.rng_sens_pitch);
		_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
		_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);

		_range_sensor.runChecks(_time_delayed_us, _R_to_earth);

		if (_range_sensor.isDataHealthy()) {
			// correct the range data for position offset relative to the IMU
			const Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_range_sensor.setRange(_range_sensor.getRange() + pos_offset_earth(2) / _range_sensor.getCosTilt());

			if (_control_status.flags.in_air) {
				const bool horizontal_motion = _control_status.flags.fixed_wing
							       || (sq(_state.vel(0)) + sq(_state.vel(1)) > fmaxf(P.trace<2>(State::vel.idx), 0.1f));

				const float dist_dependant_var = sq(_params.range_noise_scaler * _range_sensor.getDistBottom());
				const float var = sq(_params.range_noise) + dist_dependant_var;

				_rng_consistency_check.setGate(_params.range_kin_consistency_gate);
				_rng_consistency_check.update(_range_sensor.getDistBottom(), math::max(var, 0.001f), _state.vel(2),
							      P(State::vel.idx + 2, State::vel.idx + 2), horizontal_motion, _time_delayed_us);
			}

		} else {
			// If we are supposed to be using range finder data as the primary height sensor, have bad range measurements
			// and are on the ground, then synthesise a measurement at the expected on ground value
			if (!_control_status.flags.in_air
			    && _range_sensor.isRegularlySendingData()
			    && _range_sensor.isDataReady()) {

				_range_sensor.setRange(_params.rng_gnd_clearance);
				_range_sensor.setValidity(true); // bypass the checks
			}
		}

		_control_status.flags.rng_kin_consistent = _rng_consistency_check.isKinematicallyConsistent();

	} else {
		return;
	}

	auto &aid_src = _aid_src_rng_hgt;
	HeightBiasEstimator &bias_est = _rng_hgt_b_est;

	bias_est.predict(_dt_ekf_avg);

	if (rng_data_ready && _range_sensor.getSampleAddress()) {

		const float measurement = math::max(_range_sensor.getDistBottom(), _params.rng_gnd_clearance);
		const float measurement_var = sq(_params.range_noise) + sq(_params.range_noise_scaler * _range_sensor.getDistBottom());

		const float innov_gate = math::max(_params.range_innov_gate, 1.f);

		const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var);

		// vertical position innovation - baro measurement has opposite sign to earth z axis
		updateVerticalPositionAidSrcStatus(_range_sensor.getSampleAddress()->time_us,
						   -(measurement - bias_est.getBias()),
						   measurement_var + bias_est.getBiasVar(),
						   innov_gate,
						   aid_src);

		// update the bias estimator before updating the main filter but after
		// using its current state to compute the vertical position innovation
		if (measurement_valid && _range_sensor.isDataHealthy()) {
			bias_est.setMaxStateNoise(sqrtf(measurement_var));
			bias_est.setProcessNoiseSpectralDensity(_params.rng_hgt_bias_nsd);
			bias_est.fuseBias(measurement - (-_state.pos(2)), measurement_var + P(State::pos.idx + 2, State::pos.idx + 2));
		}

		// determine if we should use height aiding
		const bool do_conditional_range_aid = (_params.rng_ctrl == static_cast<int32_t>(RngCtrl::CONDITIONAL))
						      && isConditionalRangeAidSuitable();

		const bool continuing_conditions_passing = ((_params.rng_ctrl == static_cast<int32_t>(RngCtrl::ENABLED)) || do_conditional_range_aid)
				&& measurement_valid
				&& _range_sensor.isDataHealthy();

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_range_buffer_push, 2 * estimator::sensor::RNG_MAX_INTERVAL)
				&& _range_sensor.isRegularlySendingData();

		if (_control_status.flags.rng_hgt) {
			if (continuing_conditions_passing) {

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()) {
					// All height sources are failing
					ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);

					_information_events.flags.reset_hgt_to_rng = true;
					resetVerticalPositionTo(-(measurement - bias_est.getBias()));
					bias_est.setBias(_state.pos(2) + measurement);

					// reset vertical velocity
					resetVerticalVelocityToZero();

					aid_src.time_last_fuse = _time_delayed_us;

				} else if (is_fusion_failing) {
					// Some other height source is still working
					ECL_WARN("stopping %s height fusion, fusion failing", HGT_SRC_NAME);
					stopRngHgtFusion();
					_control_status.flags.rng_fault = true;
					_range_sensor.setFaulty();
				}

			} else {
				ECL_WARN("stopping %s height fusion, continuing conditions failing", HGT_SRC_NAME);
				stopRngHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if ((_params.height_sensor_ref == static_cast<int32_t>(HeightSensor::RANGE))
				    && (_params.rng_ctrl == static_cast<int32_t>(RngCtrl::CONDITIONAL))
				   ) {
					// Range finder is used while hovering to stabilize the height estimate. Don't reset but use it as height reference.
					ECL_INFO("starting conditional %s height fusion", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::RANGE;
					bias_est.setBias(_state.pos(2) + measurement);

				} else if ((_params.height_sensor_ref == static_cast<int32_t>(HeightSensor::RANGE))
					   && (_params.rng_ctrl != static_cast<int32_t>(RngCtrl::CONDITIONAL))
					  ) {
					// Range finder is the primary height source, the ground is now the datum used
					// to compute the local vertical position
					ECL_INFO("starting %s height fusion, resetting height", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::RANGE;

					_information_events.flags.reset_hgt_to_rng = true;
					resetVerticalPositionTo(-measurement, measurement_var);
					bias_est.reset();

				} else {
					ECL_INFO("starting %s height fusion", HGT_SRC_NAME);
					bias_est.setBias(_state.pos(2) + measurement);
				}

				aid_src.time_last_fuse = _time_delayed_us;
				bias_est.setFusionActive();
				_control_status.flags.rng_hgt = true;
			}
		}

	} else if (_control_status.flags.rng_hgt
		   && !isNewestSampleRecent(_time_last_range_buffer_push, 2 * estimator::sensor::RNG_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_WARN("stopping %s height fusion, no data", HGT_SRC_NAME);
		stopRngHgtFusion();
	}
}

bool Ekf::isConditionalRangeAidSuitable()
{
#if defined(CONFIG_EKF2_TERRAIN)

	if (_control_status.flags.in_air
	    && _range_sensor.isHealthy()
	    && isTerrainEstimateValid()) {
		// check if we can use range finder measurements to estimate height, use hysteresis to avoid rapid switching
		// Note that the 0.7 coefficients and the innovation check are arbitrary values but work well in practice
		float range_hagl_max = _params.max_hagl_for_range_aid;
		float max_vel_xy = _params.max_vel_for_range_aid;

		const float hagl_innov = _aid_src_terrain_range_finder.innovation;
		const float hagl_innov_var = _aid_src_terrain_range_finder.innovation_variance;

		const float hagl_test_ratio = (hagl_innov * hagl_innov / (sq(_params.range_aid_innov_gate) * hagl_innov_var));

		bool is_hagl_stable = (hagl_test_ratio < 1.f);

		if (!_control_status.flags.rng_hgt) {
			range_hagl_max = 0.7f * _params.max_hagl_for_range_aid;
			max_vel_xy = 0.7f * _params.max_vel_for_range_aid;
			is_hagl_stable = (hagl_test_ratio < 0.01f);
		}

		const float range_hagl = _terrain_vpos - _state.pos(2);

		const bool is_in_range = (range_hagl < range_hagl_max);

		bool is_below_max_speed = true;

		if (isHorizontalAidingActive()) {
			is_below_max_speed = !_state.vel.xy().longerThan(max_vel_xy);
		}

		return is_in_range && is_hagl_stable && is_below_max_speed;
	}

#endif // CONFIG_EKF2_TERRAIN

	return false;
}

void Ekf::stopRngHgtFusion()
{
	if (_control_status.flags.rng_hgt) {

		if (_height_sensor_ref == HeightSensor::RANGE) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_rng_hgt_b_est.setFusionInactive();
		resetEstimatorAidStatus(_aid_src_rng_hgt);

		_control_status.flags.rng_hgt = false;
	}
}
