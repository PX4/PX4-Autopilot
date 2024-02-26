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
 * @file ev_height_control.cpp
 * Control functions for ekf external vision height fusion
 */

#include "ekf.h"

void Ekf::controlEvHeightFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing,
				const bool ev_reset, const bool quality_sufficient, estimator_aid_source1d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "EV height";

	HeightBiasEstimator &bias_est = _ev_hgt_b_est;

	// bias_est.predict(_dt_ekf_avg) called by controlExternalVisionFusion()

	// correct position for offset relative to IMU
	const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	// rotate measurement into correct earth frame if required
	Vector3f pos{ev_sample.pos};
	Matrix3f pos_cov{matrix::diag(ev_sample.position_var)};

	// rotate EV to the EKF reference frame unless we're operating entirely in vision frame
	if (!(_control_status.flags.ev_yaw && _control_status.flags.ev_pos)) {

		const Quatf q_error(_ev_q_error_filt.getState());

		if (q_error.isAllFinite()) {
			const Dcmf R_ev_to_ekf(q_error);

			pos = R_ev_to_ekf * ev_sample.pos;
			pos_cov = R_ev_to_ekf * matrix::diag(ev_sample.position_var) * R_ev_to_ekf.transpose();

			// increase minimum variance to include EV orientation variance
			// TODO: do this properly
			const float orientation_var_max = math::max(ev_sample.orientation_var(0), ev_sample.orientation_var(1));
			pos_cov(2, 2) = math::max(pos_cov(2, 2), orientation_var_max);
		}
	}

	const float measurement = pos(2) - pos_offset_earth(2);
	float measurement_var = math::max(pos_cov(2, 2), sq(_params.ev_pos_noise), sq(0.01f));

#if defined(CONFIG_EKF2_GNSS)
	// increase minimum variance if GPS active
	if (_control_status.flags.gps_hgt) {
		measurement_var = math::max(measurement_var, sq(_params.gps_pos_noise));
	}
#endif // CONFIG_EKF2_GNSS

	const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var);

	updateVerticalPositionAidSrcStatus(ev_sample.time_us,
					   measurement - bias_est.getBias(),
					   measurement_var + bias_est.getBiasVar(),
					   math::max(_params.ev_pos_innov_gate, 1.f),
					   aid_src);

	// update the bias estimator before updating the main filter but after
	// using its current state to compute the vertical position innovation
	if (measurement_valid && quality_sufficient) {
		bias_est.setMaxStateNoise(sqrtf(measurement_var));
		bias_est.setProcessNoiseSpectralDensity(_params.ev_hgt_bias_nsd);
		bias_est.fuseBias(measurement - _state.pos(2), measurement_var + P(State::pos.idx + 2, State::pos.idx + 2));
	}

	const bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VPOS))
			&& measurement_valid;

	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing;

	if (_control_status.flags.ev_hgt) {
		if (continuing_conditions_passing) {
			if (ev_reset) {

				if (quality_sufficient) {
					ECL_INFO("reset to %s", AID_SRC_NAME);

					if (_height_sensor_ref == HeightSensor::EV) {
						_information_events.flags.reset_hgt_to_ev = true;
						resetVerticalPositionTo(measurement, measurement_var);
						bias_est.reset();

					} else {
						bias_est.setBias(-_state.pos(2) + measurement);
					}

					aid_src.time_last_fuse = _time_delayed_us;

				} else {
					// EV has reset, but quality isn't sufficient
					// we have no choice but to stop EV and try to resume once quality is acceptable
					stopEvHgtFusion();
					return;
				}

			} else if (quality_sufficient) {
				fuseVerticalPosition(aid_src);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

			if (isHeightResetRequired() && quality_sufficient) {
				// All height sources are failing
				ECL_WARN("%s fusion reset required, all height sources failing", AID_SRC_NAME);
				_information_events.flags.reset_hgt_to_ev = true;
				resetVerticalPositionTo(measurement - bias_est.getBias(), measurement_var);
				bias_est.setBias(-_state.pos(2) + measurement);

				// reset vertical velocity
				if (ev_sample.vel.isAllFinite() && (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VEL))) {

					// correct velocity for offset relative to IMU
					const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
					const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;

					switch (ev_sample.vel_frame) {
					case VelocityFrame::LOCAL_FRAME_NED:
					case VelocityFrame::LOCAL_FRAME_FRD: {
							const Vector3f reset_vel = ev_sample.vel - vel_offset_earth;
							resetVerticalVelocityTo(reset_vel(2), math::max(ev_sample.velocity_var(2), sq(_params.ev_vel_noise)));
						}
						break;

					case VelocityFrame::BODY_FRAME_FRD: {
							const Vector3f reset_vel = _R_to_earth * (ev_sample.vel - vel_offset_body);
							const Matrix3f reset_vel_cov = _R_to_earth * matrix::diag(ev_sample.velocity_var) * _R_to_earth.transpose();
							resetVerticalVelocityTo(reset_vel(2), math::max(reset_vel_cov(2, 2), sq(_params.ev_vel_noise)));
						}
						break;
					}

				} else {
					resetVerticalVelocityToZero();
				}

				aid_src.time_last_fuse = _time_delayed_us;

			} else if (is_fusion_failing) {
				// A reset did not fix the issue but all the starting checks are not passing
				// This could be a temporary issue, stop the fusion without declaring the sensor faulty
				ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
				stopEvHgtFusion();
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", AID_SRC_NAME);
			stopEvHgtFusion();
		}

	} else {
		if (starting_conditions_passing) {
			// activate fusion, only reset if necessary
			if (_params.height_sensor_ref == static_cast<int32_t>(HeightSensor::EV)) {
				ECL_INFO("starting %s fusion, resetting state", AID_SRC_NAME);
				_information_events.flags.reset_hgt_to_ev = true;
				resetVerticalPositionTo(measurement, measurement_var);

				_height_sensor_ref = HeightSensor::EV;
				bias_est.reset();

			} else {
				ECL_INFO("starting %s fusion", AID_SRC_NAME);
				bias_est.setBias(-_state.pos(2) + measurement);
			}

			aid_src.time_last_fuse = _time_delayed_us;
			bias_est.setFusionActive();
			_control_status.flags.ev_hgt = true;
		}
	}
}

void Ekf::stopEvHgtFusion()
{
	if (_control_status.flags.ev_hgt) {

		if (_height_sensor_ref == HeightSensor::EV) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_ev_hgt_b_est.setFusionInactive();
		resetEstimatorAidStatus(_aid_src_ev_hgt);

		_control_status.flags.ev_hgt = false;
	}
}
