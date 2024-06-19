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
#include <ekf_derivation/generated/compute_ev_body_vel_hx.h>
#include <ekf_derivation/generated/compute_ev_body_vel_hy.h>
#include <ekf_derivation/generated/compute_ev_body_vel_hz.h>

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
	Vector3f measurement{};
	Vector3f measurement_var{};

	float minimum_variance = math::max(sq(0.01f), sq(_params.ev_vel_noise));

	switch (ev_sample.vel_frame) {
	case VelocityFrame::LOCAL_FRAME_NED:
		if (_control_status.flags.yaw_align) {
			measurement = ev_sample.vel - vel_offset_earth;
			measurement_var = ev_sample.velocity_var;

		} else {
			continuing_conditions_passing = false;
		}

		break;

	case VelocityFrame::LOCAL_FRAME_FRD:
		if (_control_status.flags.ev_yaw) {
			// using EV frame
			measurement = ev_sample.vel - vel_offset_earth;
			measurement_var = ev_sample.velocity_var;

		} else {
			// rotate EV to the EKF reference frame
			const Dcmf R_ev_to_ekf = Dcmf(_ev_q_error_filt.getState());

			measurement = R_ev_to_ekf * ev_sample.vel - vel_offset_earth;
			measurement_var = matrix::SquareMatrix3f(R_ev_to_ekf * matrix::diag(ev_sample.velocity_var) *
					  R_ev_to_ekf.transpose()).diag();
			minimum_variance = math::max(minimum_variance, ev_sample.orientation_var.max());
		}

		break;

	case VelocityFrame::BODY_FRAME_FRD: {

			// currently it is assumed that the orientation of the EV frame and the body frame are the same
			measurement = ev_sample.vel - vel_offset_body;
			measurement_var = ev_sample.velocity_var;
			break;
		}

	default:
		continuing_conditions_passing = false;
		break;
	}

#if defined(CONFIG_EKF2_GNSS)

	// increase minimum variance if GPS active (position reference)
	if (_control_status.flags.gps) {
		for (int i = 0; i < 2; i++) {
			measurement_var(i) = math::max(measurement_var(i), sq(_params.gps_vel_noise));
		}
	}

#endif // CONFIG_EKF2_GNSS

	measurement_var = Vector3f{
		math::max(measurement_var(0), minimum_variance),
		math::max(measurement_var(1), minimum_variance),
		math::max(measurement_var(2), minimum_variance)
	};
	continuing_conditions_passing &= measurement.isAllFinite() && measurement_var.isAllFinite();

	if (ev_sample.vel_frame == VelocityFrame::BODY_FRAME_FRD) {
		const Vector3f measurement_var_ekf_frame = rotateVarianceToEkf(measurement_var);
		const Vector3f measurement_ekf_frame = _R_to_earth * measurement;
		const uint64_t t = aid_src.timestamp_sample;
		updateAidSourceStatus(aid_src,
				      ev_sample.time_us,					// sample timestamp
				      measurement_ekf_frame,					// observation
				      measurement_var_ekf_frame,				// observation variance
				      _state.vel - measurement_ekf_frame,			// innovation
				      getVelocityVariance() + measurement_var_ekf_frame,	// innovation variance
				      math::max(_params.ev_vel_innov_gate, 1.f));		// innovation gate
		aid_src.timestamp_sample = t;
		measurement.copyTo(aid_src.observation);
		measurement_var.copyTo(aid_src.observation_variance);

	} else {
		updateAidSourceStatus(aid_src,
				      ev_sample.time_us,				// sample timestamp
				      measurement,					// observation
				      measurement_var,					// observation variance
				      _state.vel - measurement,				// innovation
				      getVelocityVariance() + measurement_var,		// innovation variance
				      math::max(_params.ev_vel_innov_gate, 1.f));	// innovation gate
	}


	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing
			&& ((Vector3f(aid_src.test_ratio).max() < 0.1f) || !isHorizontalAidingActive());

	if (_control_status.flags.ev_vel) {
		if (continuing_conditions_passing) {
			if ((ev_reset && isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.ev_vel)) || yaw_alignment_changed) {
				if (quality_sufficient) {
					ECL_INFO("reset to %s", AID_SRC_NAME);
					_information_events.flags.reset_vel_to_vision = true;
					resetVelocityToEV(measurement, measurement_var, ev_sample.vel_frame);
					resetAidSourceStatusZeroInnovation(aid_src);

				} else {
					// EV has reset, but quality isn't sufficient
					// we have no choice but to stop EV and try to resume once quality is acceptable
					stopEvVelFusion();
					return;
				}

			} else if (quality_sufficient) {
				fuseEvVelocity(aid_src, ev_sample);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max); // 1 second

			if (is_fusion_failing) {

				if ((_nb_ev_vel_reset_available > 0) && quality_sufficient) {
					// Data seems good, attempt a reset
					_information_events.flags.reset_vel_to_vision = true;
					ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);
					resetVelocityToEV(measurement, measurement_var, ev_sample.vel_frame);
					resetAidSourceStatusZeroInnovation(aid_src);

					if (_control_status.flags.in_air) {
						_nb_ev_vel_reset_available--;
					}

				} else {
					// differ warning message based on whether the starting conditions are passing
					if (starting_conditions_passing) {
						// Data seems good, but previous reset did not fix the issue
						// something else must be wrong, declare the sensor faulty and stop the fusion
						//_control_status.flags.ev_vel_fault = true;
						ECL_WARN("stopping %s fusion, starting conditions failing", AID_SRC_NAME);

					} else {
						// A reset did not fix the issue but all the starting checks are not passing
						// This could be a temporary issue, stop the fusion without declaring the sensor faulty
						ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
					}

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
				ECL_INFO("starting %s fusion, resetting velocity to (%.3f, %.3f, %.3f)", AID_SRC_NAME,
					 (double)measurement(0), (double)measurement(1), (double)measurement(2));
				_information_events.flags.reset_vel_to_vision = true;
				resetVelocityToEV(measurement, measurement_var, ev_sample.vel_frame);
				resetAidSourceStatusZeroInnovation(aid_src);

				_control_status.flags.ev_vel = true;

			} else if (fuseEvVelocity(aid_src, ev_sample)) {
				ECL_INFO("starting %s fusion", AID_SRC_NAME);
				_control_status.flags.ev_vel = true;
			}

			if (_control_status.flags.ev_vel) {
				_nb_ev_vel_reset_available = 5;
				_information_events.flags.starting_vision_vel_fusion = true;
			}
		}
	}
}

bool Ekf::fuseEvVelocity(estimator_aid_source3d_s &aid_src, const extVisionSample &ev_sample)
{
	if (ev_sample.vel_frame == VelocityFrame::BODY_FRAME_FRD) {

		VectorState H;
		estimator_aid_source1d_s current_aid_src;

		for (uint8_t index = 0; index <= 2; index++) {
			current_aid_src.timestamp_sample = aid_src.timestamp_sample;

			if (index == 0) {
				sym::ComputeEvBodyVelHx(_state.vector(), &H);

			} else if (index == 1) {
				sym::ComputeEvBodyVelHy(_state.vector(), &H);

			} else {
				sym::ComputeEvBodyVelHz(_state.vector(), &H);
			}

			const float innov_var = (H.T() * P * H)(0, 0) + aid_src.observation_variance[index];
			const float innov = (_R_to_earth.transpose() * _state.vel - Vector3f(aid_src.observation))(index, 0);

			updateAidSourceStatus(current_aid_src,
					      ev_sample.time_us,				// sample timestamp
					      aid_src.observation[index],			// observation
					      aid_src.observation_variance[index],		// observation variance
					      innov,						// innovation
					      innov_var,    					// innovation variance
					      math::max(_params.ev_vel_innov_gate, 1.f));	// innovation gate

			if (!current_aid_src.innovation_rejected) {
				fuseBodyVelocity(current_aid_src, current_aid_src.innovation_variance, H);

			}

			aid_src.innovation[index] = current_aid_src.innovation;
			aid_src.innovation_variance[index] = current_aid_src.innovation_variance;
			aid_src.test_ratio[index] = current_aid_src.test_ratio;
			aid_src.fused = current_aid_src.fused;
			aid_src.innovation_rejected |= current_aid_src.innovation_rejected;

			if (aid_src.fused) {
				aid_src.time_last_fuse = _time_delayed_us;
			}

		}

		aid_src.timestamp_sample = current_aid_src.timestamp_sample;
		return !aid_src.innovation_rejected;

	} else {
		return fuseVelocity(aid_src);
	}
}

void Ekf::stopEvVelFusion()
{
	if (_control_status.flags.ev_vel) {

		_control_status.flags.ev_vel = false;
	}
}

void Ekf::resetVelocityToEV(const Vector3f &measurement, const Vector3f &measurement_var,
			    const VelocityFrame &vel_frame)
{
	if (vel_frame == VelocityFrame::BODY_FRAME_FRD) {
		const Vector3f measurement_var_ekf_frame = rotateVarianceToEkf(measurement_var);
		resetVelocityTo(_R_to_earth * measurement, measurement_var_ekf_frame);

	} else {
		resetVelocityTo(measurement, measurement_var);
	}

}

Vector3f Ekf::rotateVarianceToEkf(const Vector3f &measurement_var)
{
	// rotate the covariance matrix into the EKF frame
	const matrix::SquareMatrix<float, 3> R_cov = _R_to_earth * matrix::diag(measurement_var) * _R_to_earth.transpose();
	return R_cov.diag();
}
