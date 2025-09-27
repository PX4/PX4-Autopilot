/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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
 * Control functions for ekf GNSS fusion
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlGpsFusion(const imuSample &imu_delayed)
{
	if (!_gps_buffer || (_params.ekf2_gps_ctrl == 0)) {
		stopGnssFusion();
		return;
	}

	if (!gyro_bias_inhibited()) {
		_yawEstimator.setGyroBias(getGyroBias(), _control_status.flags.vehicle_at_rest);
	}

	// run EKF-GSF yaw estimator once per imu_delayed update
	_yawEstimator.predict(imu_delayed.delta_ang, imu_delayed.delta_ang_dt,
			      imu_delayed.delta_vel, imu_delayed.delta_vel_dt,
			      (_control_status.flags.in_air && !_control_status.flags.vehicle_at_rest));

	_gps_intermittent = !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL);

	// check for arrival of new sensor data at the fusion time horizon
	_gps_data_ready = _gps_buffer->pop_first_older_than(imu_delayed.time_us, &_gps_sample_delayed);

	if (_gps_data_ready) {
		const gnssSample &gnss_sample = _gps_sample_delayed;

		const bool initial_checks_passed_prev = _gnss_checks.initialChecksPassed();

		if (_gnss_checks.run(gnss_sample, _time_delayed_us)) {
			if (_gnss_checks.initialChecksPassed() && !initial_checks_passed_prev) {
				// First time checks are passing, latching.
				_information_events.flags.gps_checks_passed = true;
			}

		} else {
			// Skip this sample
			_gps_data_ready = false;

			const bool using_gnss = _control_status.flags.gnss_vel || _control_status.flags.gnss_pos;
			const bool gnss_checks_pass_timeout = isTimedOut(_gnss_checks.getLastPassUs(), _params.reset_timeout_max);

			if (using_gnss && gnss_checks_pass_timeout) {
				stopGnssFusion();
				ECL_WARN("GNSS quality poor - stopping use");
			}
		}

		updateGnssPos(gnss_sample, _aid_src_gnss_pos);
		updateGnssVel(imu_delayed, gnss_sample, _aid_src_gnss_vel);

	} else if (_control_status.flags.gnss_vel || _control_status.flags.gnss_pos) {
		if (!isNewestSampleRecent(_time_last_gps_buffer_push, _params.reset_timeout_max)) {
			stopGnssFusion();
			ECL_WARN("GNSS data stopped");
		}
	}

	if (_gps_data_ready) {
#if defined(CONFIG_EKF2_GNSS_YAW)
		const gnssSample &gnss_sample = _gps_sample_delayed;
		controlGnssYawFusion(gnss_sample);
#endif // CONFIG_EKF2_GNSS_YAW

		controlGnssYawEstimator(_aid_src_gnss_vel);

		bool do_vel_pos_reset = false;

		if (!_control_status.flags.gnss_fault && (_control_status.flags.gnss_vel || _control_status.flags.gnss_pos)) {

			if (_control_status.flags.in_air
			    && isYawFailure()
			    && isTimedOut(_time_last_hor_vel_fuse, _params.EKFGSF_reset_delay)
			    && (_time_last_hor_vel_fuse > _time_last_on_ground_us)) {
				do_vel_pos_reset = tryYawEmergencyReset();
			}
		}

		controlGnssVelFusion(_aid_src_gnss_vel, do_vel_pos_reset);
		controlGnssPosFusion(_aid_src_gnss_pos, do_vel_pos_reset);
	}
}

void Ekf::controlGnssVelFusion(estimator_aid_source3d_s &aid_src, const bool force_reset)
{
	const bool continuing_conditions_passing = (_params.ekf2_gps_ctrl & static_cast<int32_t>(GnssCtrl::VEL))
			&& _control_status.flags.tilt_align
			&& _control_status.flags.yaw_align
			&& !_control_status.flags.gnss_fault
			&& !_control_status.flags.gnss_hgt_fault;
	const bool starting_conditions_passing = continuing_conditions_passing && _gnss_checks.passed();

	if (_control_status.flags.gnss_vel) {
		if (continuing_conditions_passing) {
			fuseVelocity(aid_src);

			const bool fusion_timeout = isTimedOut(aid_src.time_last_fuse, _params.reset_timeout_max);

			if (fusion_timeout || force_reset) {
				if (isGnssVelResetAllowed() || force_reset) {
					ECL_WARN("GNSS fusion timeout, resetting");
					resetVelocityToGnss(aid_src);

				} else {
					stopGnssVelFusion();
				}
			}

		} else {
			stopGnssVelFusion();
		}

	} else {
		if (starting_conditions_passing) {
			bool fused = false;

			const bool do_reset = force_reset || !_control_status_prev.flags.yaw_align;

			// Start fusing the data without reset if possible to avoid disturbing the filter
			if (!do_reset && ((aid_src.test_ratio[0] + aid_src.test_ratio[1]) < sq(0.5f))) {
				fused = fuseVelocity(aid_src);
			}

			bool reset = false;

			if (!fused && (isGnssVelResetAllowed() || force_reset)) {
				resetVelocityToGnss(aid_src);
				reset = true;
			}

			if (fused || reset) {
				ECL_INFO("starting GNSS velocity fusion");
				_information_events.flags.starting_gps_fusion = true;
				_control_status.flags.gnss_vel = true;
			}
		}
	}
}

void Ekf::controlGnssPosFusion(estimator_aid_source2d_s &aid_src, const bool force_reset)
{
	const bool gnss_pos_enabled = (_params.ekf2_gps_ctrl & static_cast<int32_t>(GnssCtrl::HPOS));

	const bool continuing_conditions_passing = gnss_pos_enabled
			&& _control_status.flags.tilt_align
			&& _control_status.flags.yaw_align
			&& !_control_status.flags.gnss_hgt_fault;
	const bool starting_conditions_passing = continuing_conditions_passing && _gnss_checks.passed();
	const bool gpos_init_conditions_passing = gnss_pos_enabled && _gnss_checks.passed();

	if (_control_status.flags.gnss_pos) {
		if (continuing_conditions_passing) {
			fuseHorizontalPosition(aid_src);

			const bool fusion_timeout = isTimedOut(aid_src.time_last_fuse, _params.reset_timeout_max);

			if (fusion_timeout || force_reset) {
				if (isGnssPosResetAllowed()) {
					ECL_WARN("GNSS fusion timeout, resetting");
					resetHorizontalPositionToGnss(aid_src);

				} else {
					stopGnssPosFusion();
					_control_status.flags.gnss_fault = true;
				}
			}

		} else {
			stopGnssPosFusion();
		}

	} else {
		if (starting_conditions_passing) {
			bool fused = false;

			const bool do_reset = force_reset || !_control_status_prev.flags.yaw_align;

			// Start fusing the data without reset if possible to avoid disturbing the filter
			if (_local_origin_lat_lon.isInitialized()
			    && !do_reset
			    && ((aid_src.test_ratio[0] + aid_src.test_ratio[1]) < sq(0.5f))) {
				fused = fuseHorizontalPosition(aid_src);
			}

			bool reset = false;

			if ((!fused && isGnssPosResetAllowed())
			    || (gpos_init_conditions_passing && !_local_origin_lat_lon.isInitialized())) {
				resetHorizontalPositionToGnss(aid_src);
				reset = true;
			}

			if (fused || reset) {
				ECL_INFO("starting GNSS position fusion");
				_information_events.flags.starting_gps_fusion = true;
				_control_status.flags.gnss_pos = true;
				_control_status.flags.gnss_fault = false;
			}

		} else if (gpos_init_conditions_passing && !_local_origin_lat_lon.isInitialized()) {
			resetHorizontalPositionToGnss(aid_src);
		}
	}
}

bool Ekf::isGnssVelResetAllowed() const
{
	if (_control_status.flags.gnss_fault) {
		return false;
	}

	bool allowed = true;

	switch (static_cast<GnssMode>(_params.ekf2_gps_mode)) {
	case GnssMode::kAuto:
		if (isOtherSourceOfHorizontalVelocityAidingThan(_control_status.flags.gnss_vel)
		    && !_control_status.flags.wind_dead_reckoning) {
			allowed = false;
		}

		break;

	case GnssMode::kDeadReckoning:
		if (isOtherSourceOfHorizontalAidingThan(_control_status.flags.gnss_vel)) {
			allowed = false;
		}

		break;
	}

	return allowed;
}

bool Ekf::isGnssPosResetAllowed() const
{
	if (_control_status.flags.gnss_fault) {
		return false;
	}

	bool allowed = true;

	switch (static_cast<GnssMode>(_params.ekf2_gps_mode)) {
	case GnssMode::kAuto:
		if (isOtherSourceOfHorizontalPositionAidingThan(_control_status.flags.gnss_pos)) {
			allowed = false;
		}

		break;

	case GnssMode::kDeadReckoning:
		if (isOtherSourceOfHorizontalAidingThan(_control_status.flags.gnss_pos)) {
			allowed = false;
		}

		break;
	}

	return allowed;
}

void Ekf::updateGnssVel(const imuSample &imu_sample, const gnssSample &gnss_sample, estimator_aid_source3d_s &aid_src)
{
	// correct velocity for offset relative to IMU
	const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;

	const Vector3f angular_velocity = imu_sample.delta_ang / imu_sample.delta_ang_dt - _state.gyro_bias;
	const Vector3f vel_offset_body = angular_velocity % pos_offset_body;
	const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
	const Vector3f velocity = gnss_sample.vel - vel_offset_earth;

	const float vel_var = sq(math::max(gnss_sample.sacc, _params.ekf2_gps_v_noise, 0.01f));
	const Vector3f vel_obs_var(vel_var, vel_var, vel_var * sq(1.5f));

	const float innovation_gate = math::max(_params.ekf2_gps_v_gate, 1.f);

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
	    && (gnss_sample.sacc < _params.ekf2_req_sacc)
	   ) {
		const float innov_limit = innovation_gate * sqrtf(aid_src.innovation_variance[2]);
		aid_src.innovation[2] = math::constrain(aid_src.innovation[2], -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}
}

void Ekf::updateGnssPos(const gnssSample &gnss_sample, estimator_aid_source2d_s &aid_src)
{
	// correct position and height for offset relative to IMU
	const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = Vector3f(_R_to_earth * pos_offset_body);
	const LatLonAlt measurement(gnss_sample.lat, gnss_sample.lon, gnss_sample.alt);
	const LatLonAlt measurement_corrected = measurement + (-pos_offset_earth);
	const Vector2f innovation = (_gpos - measurement_corrected).xy();

	// relax the upper observation noise limit which prevents bad GPS perturbing the position estimate
	float pos_noise = math::max(gnss_sample.hacc, _params.ekf2_gps_p_noise);

	if (!isOtherSourceOfHorizontalAidingThan(_control_status.flags.gnss_pos)) {
		// if we are not using another source of aiding, then we are reliant on the GNSS
		// observations to constrain attitude errors and must limit the observation noise value.
		if (pos_noise > _params.ekf2_noaid_noise) {
			pos_noise = _params.ekf2_noaid_noise;
		}
	}

	const float pos_var = math::max(sq(pos_noise), sq(0.01f));
	const Vector2f pos_obs_var(pos_var, pos_var);
	const matrix::Vector2d observation(measurement_corrected.latitude_deg(), measurement_corrected.longitude_deg());

	updateAidSourceStatus(aid_src,
			      gnss_sample.time_us,                                    // sample timestamp
			      observation,                                            // observation
			      pos_obs_var,                                            // observation variance
			      innovation,                                             // innovation
			      Vector2f(getStateVariance<State::pos>()) + pos_obs_var, // innovation variance
			      math::max(_params.ekf2_gps_p_gate, 1.f));            // innovation gate
}

void Ekf::controlGnssYawEstimator(estimator_aid_source3d_s &aid_src_vel)
{
	// update yaw estimator velocity (basic sanity check on GNSS velocity data)
	const float vel_var = aid_src_vel.observation_variance[0];
	const Vector2f vel_xy(aid_src_vel.observation);

	if ((vel_var > 0.f)
	    && (vel_var < _params.ekf2_req_sacc)
	    && vel_xy.isAllFinite()) {

		_yawEstimator.fuseVelocity(vel_xy, vel_var, _control_status.flags.in_air);

		// Try to align yaw using estimate if available
		if (((_params.ekf2_gps_ctrl & static_cast<int32_t>(GnssCtrl::VEL))
		     || (_params.ekf2_gps_ctrl & static_cast<int32_t>(GnssCtrl::HPOS)))
		    && !_control_status.flags.yaw_align
		    && _control_status.flags.tilt_align) {
			if (resetYawToEKFGSF()) {
				ECL_INFO("GPS yaw aligned using IMU");
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
		ECL_WARN("GPS emergency yaw reset");

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

void Ekf::resetVelocityToGnss(estimator_aid_source3d_s &aid_src)
{
	_information_events.flags.reset_vel_to_gps = true;
	resetVelocityTo(Vector3f(aid_src.observation), Vector3f(aid_src.observation_variance));

	resetAidSourceStatusZeroInnovation(aid_src);
}

void Ekf::resetHorizontalPositionToGnss(estimator_aid_source2d_s &aid_src)
{
	_information_events.flags.reset_pos_to_gps = true;
	resetLatLonTo(aid_src.observation[0], aid_src.observation[1],
		      aid_src.observation_variance[0] +
		      aid_src.observation_variance[1]);

	resetAidSourceStatusZeroInnovation(aid_src);
}

void Ekf::stopGnssFusion()
{
	if (_control_status.flags.gnss_vel || _control_status.flags.gnss_pos) {
		_gnss_checks.reset();
	}

	stopGnssVelFusion();
	stopGnssPosFusion();
	stopGpsHgtFusion();
#if defined(CONFIG_EKF2_GNSS_YAW)
	stopGnssYawFusion();
#endif // CONFIG_EKF2_GNSS_YAW

	_yawEstimator.reset();
}

void Ekf::stopGnssVelFusion()
{
	if (_control_status.flags.gnss_vel) {
		ECL_INFO("stopping GNSS velocity fusion");
		_control_status.flags.gnss_vel = false;

		//TODO: what if gnss yaw or height is used?
		if (!_control_status.flags.gnss_pos) {
			_gnss_checks.reset();
		}
	}
}

void Ekf::stopGnssPosFusion()
{
	if (_control_status.flags.gnss_pos) {
		ECL_INFO("stopping GNSS position fusion");
		_control_status.flags.gnss_pos = false;

		//TODO: what if gnss yaw or height is used?
		if (!_control_status.flags.gnss_vel) {
			_gnss_checks.reset();
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
