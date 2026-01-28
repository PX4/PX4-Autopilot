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

void Ekf::controlGnssHeightFusion(const gnssSample &gps_sample)
{
	static constexpr const char *HGT_SRC_NAME = "GNSS";

	auto &aid_src = _aid_src_gnss_hgt;
	HeightBiasEstimator &bias_est = _gps_hgt_b_est;

	bias_est.predict(_dt_ekf_avg);

	if (_gps_data_ready) {

		// relax the upper observation noise limit which prevents bad GPS perturbing the position estimate
		float noise = math::max(gps_sample.vacc, 1.5f * _params.ekf2_gps_p_noise); // use 1.5 as a typical ratio of vacc/hacc

		if (!isOnlyActiveSourceOfVerticalPositionAiding(_control_status.flags.gps_hgt)) {
			// if we are not using another source of aiding, then we are reliant on the GPS
			// observations to constrain attitude errors and must limit the observation noise value.
			if (noise > _params.ekf2_noaid_noise) {
				noise = _params.ekf2_noaid_noise;
			}
		}

		const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
		const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
		const float gnss_alt = gps_sample.alt + pos_offset_earth(2);

		const float measurement = gnss_alt;
		const float measurement_var = sq(noise);

		const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var);

		// GNSS position, vertical position GNSS measurement has opposite sign to earth z axis
		updateVerticalPositionAidStatus(aid_src,
						gps_sample.time_us,
						-(measurement - bias_est.getBias()),
						measurement_var + bias_est.getBiasVar(),
						math::max(_params.ekf2_gps_p_gate, 1.f));

		// determine if we should use height aiding
		const bool common_conditions_passing = measurement_valid
						       && _local_origin_lat_lon.isInitialized()
						       && _gnss_checks.passed()
						       && !_control_status.flags.gnss_fault;

		const bool continuing_conditions_passing = (_params.ekf2_gps_ctrl & static_cast<int32_t>(GnssCtrl::VPOS))
				&& common_conditions_passing;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL);

		const bool altitude_initialisation_conditions_passing = common_conditions_passing
				&& !PX4_ISFINITE(_local_origin_alt)
				&& _params.ekf2_hgt_ref == static_cast<int32_t>(HeightSensor::GNSS)
				&& isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL);

		if (_control_status.flags.gps_hgt) {
			if (continuing_conditions_passing) {

				// update the bias estimator before updating the main filter but after
				// using its current state to compute the vertical position innovation
				bias_est.setMaxStateNoise(sqrtf(measurement_var));
				bias_est.setProcessNoiseSpectralDensity(_params.gps_hgt_bias_nsd);
				bias_est.fuseBias(measurement - _gpos.altitude(), measurement_var + P(State::pos.idx + 2, State::pos.idx + 2));

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired() && (_height_sensor_ref == HeightSensor::GNSS) && isGnssHgtResetAllowed()) {
					// All height sources are failing
					ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);

					_information_events.flags.reset_hgt_to_gps = true;
					resetAltitudeTo(measurement, measurement_var);
					bias_est.setBias(-_gpos.altitude() + measurement);
					resetAidSourceStatusZeroInnovation(aid_src);

					aid_src.time_last_fuse = _time_delayed_us;

				} else if (is_fusion_failing) {
					// Some other height source is still working
					ECL_WARN("stopping %s height fusion, fusion failing", HGT_SRC_NAME);
					stopGpsHgtFusion();

					if (!isGnssHgtResetAllowed()) {
						_control_status.flags.gnss_hgt_fault = true;
						_time_last_gnss_hgt_rejected = _time_delayed_us;
					}
				}

			} else {
				ECL_WARN("stopping %s height fusion, continuing conditions failing", HGT_SRC_NAME);
				stopGpsHgtFusion();
			}

		} else {
			if (altitude_initialisation_conditions_passing) {
				// Altitude not initialized, GNSS is the configured height reference
				_information_events.flags.reset_hgt_to_gps = true;
				initialiseAltitudeTo(measurement, measurement_var);
				bias_est.reset();

				// Start fusion if GPS vertical position control is also enabled
				if (starting_conditions_passing) {
					_height_sensor_ref = HeightSensor::GNSS;
					resetAidSourceStatusZeroInnovation(aid_src);
					aid_src.time_last_fuse = _time_delayed_us;
					bias_est.setFusionActive();
					_control_status.flags.gps_hgt = true;
				}

			} else if (starting_conditions_passing) {
				if (_params.ekf2_hgt_ref == static_cast<int32_t>(HeightSensor::GNSS) && isGnssHgtResetAllowed()) {
					_height_sensor_ref = HeightSensor::GNSS;
					_information_events.flags.reset_hgt_to_gps = true;

					resetAltitudeTo(measurement, measurement_var);
					bias_est.reset();
					resetAidSourceStatusZeroInnovation(aid_src);

					aid_src.time_last_fuse = _time_delayed_us;
					bias_est.setFusionActive();
					_control_status.flags.gps_hgt = true;
					_control_status.flags.gnss_hgt_fault = false;

				} else {
					bool is_gnss_hgt_consistent = true;

					if (_control_status.flags.gnss_hgt_fault) {
						if (aid_src.innovation_rejected) {
							_time_last_gnss_hgt_rejected = _time_delayed_us;
						}

						is_gnss_hgt_consistent = isTimedOut(_time_last_gnss_hgt_rejected, _params.hgt_fusion_timeout_max);
					}

					if (is_gnss_hgt_consistent) {
						if (_params.ekf2_hgt_ref != static_cast<int32_t>(HeightSensor::GNSS)) {
							bias_est.setBias(-_gpos.altitude() + measurement);
						}

						aid_src.time_last_fuse = _time_delayed_us;
						bias_est.setFusionActive();
						_control_status.flags.gps_hgt = true;
						_control_status.flags.gnss_hgt_fault = false;
					}
				}
			}
		}

	} else if (_control_status.flags.gps_hgt
		   && !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_WARN("stopping %s height fusion, no data", HGT_SRC_NAME);
		stopGpsHgtFusion();
	}
}

void Ekf::stopGpsHgtFusion()
{
	if (_control_status.flags.gps_hgt) {

		if (_height_sensor_ref == HeightSensor::GNSS) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_gps_hgt_b_est.setFusionInactive();

		_control_status.flags.gps_hgt = false;
	}
}

bool Ekf::isGnssHgtResetAllowed()
{
	const bool allowed = !(static_cast<GnssMode>(_params.ekf2_gps_mode) == GnssMode::kDeadReckoning
			       && isOtherSourceOfVerticalPositionAidingThan(_control_status.flags.gps_hgt))
			     || !PX4_ISFINITE(_local_origin_alt);

	return allowed;
}
