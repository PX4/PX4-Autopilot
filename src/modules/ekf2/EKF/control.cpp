/****************************************************************************
 *
 *   Copyright (c) 2015-2020 Estimation and Control Library (ECL). All rights reserved.
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
 * @file control.cpp
 * Control functions for ekf attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */


#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlFusionModes(const imuSample &imu_delayed)
{
	// Store the status to enable change detection
	_control_status_prev.value = _control_status.value;
	_state_reset_count_prev = _state_reset_status.reset_count;

	if (_system_flag_buffer) {
		systemFlagUpdate system_flags_delayed;

		if (_system_flag_buffer->pop_first_older_than(imu_delayed.time_us, &system_flags_delayed)) {

			set_vehicle_at_rest(system_flags_delayed.at_rest);
			set_in_air_status(system_flags_delayed.in_air);

			set_is_fixed_wing(system_flags_delayed.is_fixed_wing);

			if (system_flags_delayed.gnd_effect) {
				set_gnd_effect();
			}
		}
	}

	// monitor the tilt alignment
	if (!_control_status.flags.tilt_align) {
		// whilst we are aligning the tilt, monitor the variances
		const Vector3f angle_err_var_vec = calcRotVecVariances();

		// Once the tilt variances have reduced to equivalent of 3deg uncertainty
		// and declare the tilt alignment complete
		if ((angle_err_var_vec(0) + angle_err_var_vec(1)) < sq(math::radians(3.0f))) {
			_control_status.flags.tilt_align = true;

			// send alignment status message to the console
			const char *height_source = nullptr;

			if (_control_status.flags.baro_hgt) {
				height_source = "baro";

			} else if (_control_status.flags.ev_hgt) {
				height_source = "ev";

			} else if (_control_status.flags.gps_hgt) {
				height_source = "gps";

			} else if (_control_status.flags.rng_hgt) {
				height_source = "rng";

			} else {
				height_source = "unknown";

			}

			if (height_source) {
				ECL_INFO("%llu: EKF aligned, (%s hgt, IMU buf: %i, OBS buf: %i)",
					 (unsigned long long)imu_delayed.time_us, height_source, (int)_imu_buffer_length, (int)_obs_buffer_length);
			}
		}
	}

	if (_gps_buffer) {
		_gps_intermittent = !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL);

		// check for arrival of new sensor data at the fusion time horizon
		_time_prev_gps_us = _gps_sample_delayed.time_us;
		_gps_data_ready = _gps_buffer->pop_first_older_than(imu_delayed.time_us, &_gps_sample_delayed);

		if (_gps_data_ready) {
			// correct velocity for offset relative to IMU
			const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
			const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
			const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
			_gps_sample_delayed.vel -= vel_offset_earth;

			// correct position and height for offset relative to IMU
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_gps_sample_delayed.pos -= pos_offset_earth.xy();
			_gps_sample_delayed.hgt += pos_offset_earth(2);

			// update GSF yaw estimator velocity (basic sanity check on GNSS velocity data)
			if ((_gps_sample_delayed.sacc > 0.f) && (_gps_sample_delayed.sacc < _params.req_sacc)
			    && _gps_sample_delayed.vel.isAllFinite()
			   ) {
				_yawEstimator.setVelocity(_gps_sample_delayed.vel.xy(), math::max(_gps_sample_delayed.sacc, _params.gps_vel_noise));
			}
		}
	}

	if (_range_buffer) {
		// Get range data from buffer and check validity
		_rng_data_ready = _range_buffer->pop_first_older_than(imu_delayed.time_us, _range_sensor.getSampleAddress());
		_range_sensor.setDataReadiness(_rng_data_ready);

		// update range sensor angle parameters in case they have changed
		_range_sensor.setPitchOffset(_params.rng_sens_pitch);
		_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
		_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);

		_range_sensor.runChecks(imu_delayed.time_us, _R_to_earth);

		if (_range_sensor.isDataHealthy()) {
			// correct the range data for position offset relative to the IMU
			const Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_range_sensor.setRange(_range_sensor.getRange() + pos_offset_earth(2) / _range_sensor.getCosTilt());

			// Run the kinematic consistency check when not moving horizontally
			if (_control_status.flags.in_air && !_control_status.flags.fixed_wing
			    && (sq(_state.vel(0)) + sq(_state.vel(1)) < fmaxf(P(4, 4) + P(5, 5), 0.1f))) {

				const float dist_dependant_var = sq(_params.range_noise_scaler * _range_sensor.getDistBottom());
				const float var = sq(_params.range_noise) + dist_dependant_var;

				_rng_consistency_check.setGate(_params.range_kin_consistency_gate);
				_rng_consistency_check.update(_range_sensor.getDistBottom(), math::max(var, 0.001f), _state.vel(2), P(6, 6), imu_delayed.time_us);
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
	}

	if (_flow_buffer) {
		// We don't fuse flow data immediately because we have to wait for the mid integration point to fall behind the fusion time horizon.
		// This means we stop looking for new data until the old data has been fused, unless we are not fusing optical flow,
		// in this case we need to empty the buffer
		if (!_flow_data_ready || (!_control_status.flags.opt_flow && !_hagl_sensor_status.flags.flow)) {
			_flow_data_ready = _flow_buffer->pop_first_older_than(imu_delayed.time_us, &_flow_sample_delayed);
		}
	}

	if (_airspeed_buffer) {
		_tas_data_ready = _airspeed_buffer->pop_first_older_than(imu_delayed.time_us, &_airspeed_sample_delayed);
	}

	// run EKF-GSF yaw estimator once per imu_delayed update after all main EKF data samples available
	runYawEKFGSF(imu_delayed);

	// control use of observations for aiding
	controlMagFusion();
	controlOpticalFlowFusion(imu_delayed);
	controlGpsFusion();
	controlAirDataFusion();
	controlBetaFusion(imu_delayed);
	controlDragFusion();
	controlHeightFusion(imu_delayed);
	controlGravityFusion(imu_delayed);

	// Additional data odometry data from an external estimator can be fused.
	controlExternalVisionFusion();

	// Additional horizontal velocity data from an auxiliary sensor can be fused
	controlAuxVelFusion();

	controlZeroInnovationHeadingUpdate();

	controlZeroVelocityUpdate();

	// Fake position measurement for constraining drift when no other velocity or position measurements
	controlFakePosFusion();
	controlFakeHgtFusion();

	// check if we are no longer fusing measurements that directly constrain velocity drift
	updateDeadReckoningStatus();
}

void Ekf::controlAuxVelFusion()
{
	if (_auxvel_buffer) {
		auxVelSample auxvel_sample_delayed;

		if (_auxvel_buffer->pop_first_older_than(_time_delayed_us, &auxvel_sample_delayed)) {

			resetEstimatorAidStatus(_aid_src_aux_vel);

			updateVelocityAidSrcStatus(auxvel_sample_delayed.time_us, auxvel_sample_delayed.vel, auxvel_sample_delayed.velVar, fmaxf(_params.auxvel_gate, 1.f), _aid_src_aux_vel);

			if (isHorizontalAidingActive()) {
				_aid_src_aux_vel.fusion_enabled = true;
				fuseVelocity(_aid_src_aux_vel);
			}
		}
	}
}
