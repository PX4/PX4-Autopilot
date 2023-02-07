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
 * @file ev_vel_control.cpp
 * Control functions for ekf external vision velocity fusion
 */

#include "ekf.h"

void Ekf::controlEvVelFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing,
			     const bool ev_reset, const bool quality_sufficient, estimator_aid_source3d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "EV velocity";

	const bool yaw_alignment_changed = (!_control_status_prev.flags.ev_yaw && _control_status.flags.ev_yaw)
					   || (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align);

	// determine if we should use EV velocity aiding
	bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VEL))
					     && _control_status.flags.tilt_align
					     && ev_sample.vel.isAllFinite();

	// correct velocity for offset relative to IMU
	const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
	const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
	const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;

	// rotate measurement into correct earth frame if required
	Vector3f vel{NAN, NAN, NAN};
	Matrix3f vel_cov{};

	switch (ev_sample.vel_frame) {
	case VelocityFrame::LOCAL_FRAME_NED:
		if (_control_status.flags.yaw_align) {
			vel = ev_sample.vel - vel_offset_earth;
			vel_cov = matrix::diag(ev_sample.velocity_var);

		} else {
			continuing_conditions_passing = false;
		}

		break;

	case VelocityFrame::LOCAL_FRAME_FRD:
		if (_control_status.flags.ev_yaw) {
			// using EV frame
			vel = ev_sample.vel - vel_offset_earth;
			vel_cov = matrix::diag(ev_sample.velocity_var);

		} else {
			// rotate EV to the EKF reference frame
			const Quatf q_error((_state.quat_nominal * ev_sample.quat.inversed()).normalized());
			const Dcmf R_ev_to_ekf = Dcmf(q_error);

			vel = R_ev_to_ekf * ev_sample.vel - vel_offset_earth;
			vel_cov = R_ev_to_ekf * matrix::diag(ev_sample.velocity_var) * R_ev_to_ekf.transpose();

			// increase minimum variance to include EV orientation variance
			// TODO: do this properly
			const float orientation_var_max = ev_sample.orientation_var.max();

			for (int i = 0; i < 2; i++) {
				vel_cov(i, i) = math::max(vel_cov(i, i), orientation_var_max);
			}
		}

		break;

	case VelocityFrame::BODY_FRAME_FRD:
		vel = _R_to_earth * (ev_sample.vel - vel_offset_body);
		vel_cov = _R_to_earth * matrix::diag(ev_sample.velocity_var) * _R_to_earth.transpose();
		break;

	default:
		continuing_conditions_passing = false;
		break;
	}

	// increase minimum variance if GPS active (position reference)
	if (_control_status.flags.gps) {
		for (int i = 0; i < 2; i++) {
			vel_cov(i, i) = math::max(vel_cov(i, i), sq(_params.gps_vel_noise));
		}
	}

	const Vector3f measurement{vel};

	const Vector3f measurement_var{
		math::max(vel_cov(0, 0), sq(_params.ev_vel_noise), sq(0.01f)),
		math::max(vel_cov(1, 1), sq(_params.ev_vel_noise), sq(0.01f)),
		math::max(vel_cov(2, 2), sq(_params.ev_vel_noise), sq(0.01f))
	};

	const bool measurement_valid = measurement.isAllFinite() && measurement_var.isAllFinite();

	updateVelocityAidSrcStatus(ev_sample.time_us,
				   measurement,                               // observation
				   measurement_var,                           // observation variance
				   math::max(_params.ev_vel_innov_gate, 1.f), // innovation gate
				   aid_src);

	if (!measurement_valid) {
		continuing_conditions_passing = false;
	}

	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing
			&& ((Vector3f(aid_src.test_ratio).max() < 0.1f) || !isHorizontalAidingActive());

	if (_control_status.flags.ev_vel) {
		aid_src.fusion_enabled = true;

		if (continuing_conditions_passing) {

			if ((ev_reset && isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.ev_vel)) || yaw_alignment_changed) {

				if (quality_sufficient) {
					ECL_INFO("reset to %s", AID_SRC_NAME);
					_information_events.flags.reset_vel_to_vision = true;
					resetVelocityTo(measurement, measurement_var);
					aid_src.time_last_fuse = _time_delayed_us;

				} else {
					// EV has reset, but quality isn't sufficient
					// we have no choice but to stop EV and try to resume once quality is acceptable
					stopEvVelFusion();
					return;
				}

			} else if (quality_sufficient) {
				fuseVelocity(aid_src);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max); // 1 second

			if (is_fusion_failing) {

				if ((_nb_ev_vel_reset_available > 0) && quality_sufficient) {
					// Data seems good, attempt a reset
					_information_events.flags.reset_vel_to_vision = true;
					ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);
					resetVelocityTo(measurement, measurement_var);
					aid_src.time_last_fuse = _time_delayed_us;

					if (_control_status.flags.in_air) {
						_nb_ev_vel_reset_available--;
					}

				} else if (starting_conditions_passing) {
					// Data seems good, but previous reset did not fix the issue
					// something else must be wrong, declare the sensor faulty and stop the fusion
					//_control_status.flags.ev_vel_fault = true;
					ECL_WARN("stopping %s fusion, starting conditions failing", AID_SRC_NAME);
					stopEvVelFusion();

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
					stopEvVelFusion();
				}
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", AID_SRC_NAME);
			stopEvVelFusion();
		}

	} else {
		if (starting_conditions_passing) {
			// activate fusion, only reset if necessary
			if (!isHorizontalAidingActive() || yaw_alignment_changed) {
				ECL_INFO("starting %s fusion, resetting velocity to (%.3f, %.3f, %.3f)", AID_SRC_NAME, (double)measurement(0), (double)measurement(1), (double)measurement(2));
				_information_events.flags.reset_vel_to_vision = true;
				resetVelocityTo(measurement, measurement_var);

			} else {
				ECL_INFO("starting %s fusion", AID_SRC_NAME);
			}

			aid_src.time_last_fuse = _time_delayed_us;

			_nb_ev_vel_reset_available = 5;
			_information_events.flags.starting_vision_vel_fusion = true;
			_control_status.flags.ev_vel = true;
		}
	}
}

void Ekf::stopEvVelFusion()
{
	if (_control_status.flags.ev_vel) {
		resetEstimatorAidStatus(_aid_src_ev_vel);

		_control_status.flags.ev_vel = false;
	}
}
