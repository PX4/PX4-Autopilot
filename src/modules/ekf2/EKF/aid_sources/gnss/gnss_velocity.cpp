/****************************************************************************
 *
 *   Copyright (c) 2021-2025 PX4 Development Team. All rights reserved.
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

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlGnssVelocityFusion(const imuSample &imu_sample, const gnssSample &gnss_sample, bool reset)
{
	static constexpr const char *AID_SRC_NAME = "GNSS velocity";

	auto &aid_src = _aid_src_gnss_vel;

	// correct velocity for offset relative to IMU
	const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;

	const Vector3f angular_velocity = imu_sample.delta_ang / imu_sample.delta_ang_dt - _state.gyro_bias;
	const Vector3f vel_offset_body = angular_velocity % pos_offset_body;
	const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
	const Vector3f velocity = gnss_sample.vel - vel_offset_earth;

	const float vel_var = sq(math::max(gnss_sample.sacc, _params.gps_vel_noise, 0.01f));
	const Vector3f vel_obs_var(vel_var, vel_var, vel_var * sq(1.5f));

	const float innovation_gate = math::max(_params.gps_vel_innov_gate, 1.f);

	updateAidSourceStatus(aid_src,
			      gnss_sample.time_us,                  // sample timestamp
			      velocity,                             // observation
			      vel_obs_var,                          // observation variance
			      _state.vel - velocity,                // innovation
			      getVelocityVariance() + vel_obs_var,  // innovation variance
			      innovation_gate);                     // innovation gate

	// vz special case if there is bad vertical acceleration data, then don't reject measurement if GNSS reports velocity accuracy is acceptable,
	// but limit innovation to prevent spikes that could destabilise the filter
	bool bad_acc_vz_rejected = _fault_status.flags.bad_acc_vertical
				   && (aid_src.test_ratio[2] > 1.f)                                   // vz rejected
				   && (aid_src.test_ratio[0] < 1.f) && (aid_src.test_ratio[1] < 1.f); // vx & vy accepted

	if (bad_acc_vz_rejected
	    && (gnss_sample.sacc < _params.req_sacc)
	   ) {
		const float innov_limit = innovation_gate * sqrtf(aid_src.innovation_variance[2]);
		aid_src.innovation[2] = math::constrain(aid_src.innovation[2], -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}

	if (_control_status.flags.gnss_vel) {

		if (_control_status.flags.in_air
		    && isYawFailure()
		    && isTimedOut(_time_last_hor_vel_fuse, _params.EKFGSF_reset_delay)
		    && (_time_last_hor_vel_fuse > _time_last_on_ground_us)) {

			if (tryYawEmergencyReset()) {
				reset = true;
			}
		}
	}

	const bool continuing_conditions_passing = (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::VEL))
			&& _control_status.flags.tilt_align
			&& _control_status.flags.yaw_align
			&& velocity.isAllFinite()
			&& (gnss_sample.sacc < _params.req_sacc)
			&& !_gps_check_fail_status.flags.hspeed;

	const bool starting_conditions_passing = continuing_conditions_passing
				&& _gnss_common_checks_passed
				&& velocity.longerThan(0.f);

	if (_control_status.flags.gnss_vel) {
		if (continuing_conditions_passing) {
			if (reset) {
				ECL_WARN("%s fusion timeout, resetting", AID_SRC_NAME);

				_information_events.flags.reset_vel_to_gps = true;
				resetVelocityTo(Vector3f(aid_src.observation), Vector3f(aid_src.observation_variance));

				resetAidSourceStatusZeroInnovation(aid_src);

			} else {
				fuseVelocity(aid_src);

				if (isHeightResetRequired()) {
					// reset vertical velocity if height is failing
					resetVerticalVelocityTo(aid_src.observation[2], aid_src.observation_variance[2]);
				}
			}

		} else {
			stopGnssVelFusion();
		}

	} else {
		if (starting_conditions_passing) {
			ECL_INFO("%s starting fusion", AID_SRC_NAME);
			_information_events.flags.starting_gps_fusion = true;

			// when already using another velocity source velocity reset is not necessary
			if (!isHorizontalAidingActive()
			    || isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
			    || !_control_status_prev.flags.yaw_align
			   ) {
				// reset velocity
				_information_events.flags.reset_vel_to_gps = true;
				resetVelocityTo(Vector3f(aid_src.observation), Vector3f(aid_src.observation_variance));

				resetAidSourceStatusZeroInnovation(aid_src);

			}

			_control_status.flags.gnss_vel = true;
		}
	}
}

void Ekf::controlGnssYawEstimator(estimator_aid_source3d_s &aid_src_vel)
{
	// update yaw estimator velocity (basic sanity check on GNSS velocity data)
	const float vel_var = aid_src_vel.observation_variance[0];
	const Vector2f vel_xy(aid_src_vel.observation);

	if ((vel_var > 0.f)
	    && (vel_var < _params.req_sacc)
	    && vel_xy.isAllFinite()
	   ) {
		_yawEstimator.fuseVelocity(vel_xy, vel_var, _control_status.flags.in_air);

		// Try to align yaw using estimate if available
		if ((_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::VEL))
		    && !_control_status.flags.yaw_align
		    && _control_status.flags.tilt_align
		   ) {

			if (resetYawToEKFGSF()) {
				ECL_INFO("GNSS yaw aligned using IMU");
			}
		}
	}
}

bool Ekf::tryYawEmergencyReset()
{
	bool success = false;

	/* A rapid reset to the yaw emergency estimate is performed if horizontal velocity innovation checks continuously
	 * fails while the difference between the yaw emergency estimator and the yaw estimate is large.
	 * This enables recovery from a bad yaw estimate. A reset is not performed if the fault condition was
	 * present before flight to prevent triggering due to GPS glitches or other sensor errors.
	 */
	if (resetYawToEKFGSF()) {
		ECL_WARN("GNSS emergency yaw reset");

		if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
			// stop using the magnetometer in the main EKF otherwise its fusion could drag the yaw around
			// and cause another navigation failure
			_control_status.flags.mag_fault = true;
		}

#if defined(CONFIG_EKF2_GNSS_YAW)

		if (_control_status.flags.gnss_yaw) {
			_control_status.flags.gnss_yaw_fault = true;
		}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

		if (_control_status.flags.ev_yaw) {
			_control_status.flags.ev_yaw_fault = true;
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION

		success = true;
	}

	return success;
}

void Ekf::stopGnssVelFusion()
{
	if (_control_status.flags.gnss_vel) {
		ECL_INFO("stopping GNSS velocity fusion");
		_control_status.flags.gnss_vel = false;

		//TODO: what if gnss yaw or height is used?
		if (!_control_status.flags.gnss_pos) {
			_last_gps_fail_us = 0;
			_last_gps_pass_us = 0;
		}
	}
}

bool Ekf::isYawEmergencyEstimateAvailable() const
{
	// don't allow reet using the EKF-GSF estimate until the filter has started fusing velocity
	// data and the yaw estimate has converged
	if (!_yawEstimator.isActive()) {
		return false;
	}

	const float yaw_var = _yawEstimator.getYawVar();

	return (yaw_var > 0.f)
	       && (yaw_var < sq(_params.EKFGSF_yaw_err_max))
	       && PX4_ISFINITE(yaw_var);
}

bool Ekf::isYawFailure() const
{
	if (!isYawEmergencyEstimateAvailable()) {
		return false;
	}

	const float euler_yaw = getEulerYaw(_R_to_earth);
	const float yaw_error = wrap_pi(euler_yaw - _yawEstimator.getYaw());

	return fabsf(yaw_error) > math::radians(25.f);
}

bool Ekf::resetYawToEKFGSF()
{
	if (!isYawEmergencyEstimateAvailable()) {
		return false;
	}

	// don't allow reset if there's just been a yaw reset
	const bool yaw_alignment_changed = (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align);
	const bool quat_reset = (_state_reset_status.reset_count.quat != _state_reset_count_prev.quat);

	if (yaw_alignment_changed || quat_reset) {
		return false;
	}

	ECL_INFO("yaw estimator reset heading %.3f -> %.3f rad",
		 (double)getEulerYaw(_R_to_earth), (double)_yawEstimator.getYaw());

	resetQuatStateYaw(_yawEstimator.getYaw(), _yawEstimator.getYawVar());

	_control_status.flags.yaw_align = true;
	_information_events.flags.yaw_aligned_to_imu_gps = true;

	return true;
}

bool Ekf::getDataEKFGSF(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF])
{
	return _yawEstimator.getLogData(yaw_composite, yaw_variance, yaw, innov_VN, innov_VE, weight);
}
