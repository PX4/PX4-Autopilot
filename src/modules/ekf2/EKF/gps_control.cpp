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
	if (!_gps_buffer || (_params.gnss_ctrl == 0)) {
		stopGpsFusion();
		return;
	}

	if (!gyro_bias_inhibited()) {
		_yawEstimator.setGyroBias(getGyroBias());
	}

	// run EKF-GSF yaw estimator once per imu_delayed update
	_yawEstimator.predict(imu_delayed, _control_status.flags.in_air && !_control_status.flags.vehicle_at_rest);

	_gps_intermittent = !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL);

	// check for arrival of new sensor data at the fusion time horizon
	_gps_data_ready = _gps_buffer->pop_first_older_than(imu_delayed.time_us, &_gps_sample_delayed);

	if (_gps_data_ready) {
		const gnssSample &gnss_sample = _gps_sample_delayed;

		if (runGnssChecks(gnss_sample) && isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us / 2)) {
			if (isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us)) {
				// First time checks are passing, latching.
				_gps_checks_passed = true;
			}

			collect_gps(gnss_sample);

		} else {
			// Skip this sample
			_gps_data_ready = false;

			if (_control_status.flags.gps && isTimedOut(_last_gps_pass_us, _params.reset_timeout_max)) {
				stopGpsFusion();
				_warning_events.flags.gps_quality_poor = true;
				ECL_WARN("GPS quality poor - stopping use");
			}
		}

		if (_pos_ref.isInitialized()) {
			updateGnssPos(gnss_sample, _aid_src_gnss_pos);
		}

		updateGnssVel(gnss_sample, _aid_src_gnss_vel);

	} else if (_control_status.flags.gps) {
		if (!isNewestSampleRecent(_time_last_gps_buffer_push, _params.reset_timeout_max)) {
			stopGpsFusion();
			_warning_events.flags.gps_data_stopped = true;
			ECL_WARN("GPS data stopped");
		}
	}

	if (_gps_data_ready) {
#if defined(CONFIG_EKF2_GNSS_YAW)
		const gnssSample &gnss_sample = _gps_sample_delayed;
		controlGpsYawFusion(gnss_sample);
#endif // CONFIG_EKF2_GNSS_YAW

		controlGnssYawEstimator(_aid_src_gnss_vel);

		const bool gnss_vel_enabled = (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::VEL));
		const bool gnss_pos_enabled = (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::HPOS));

		const bool continuing_conditions_passing = (gnss_vel_enabled || gnss_pos_enabled)
				&& _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align
				&& _NED_origin_initialised;
		const bool starting_conditions_passing = continuing_conditions_passing && _gps_checks_passed;

		if (_control_status.flags.gps) {
			if (continuing_conditions_passing) {
				if (gnss_vel_enabled) {
					fuseVelocity(_aid_src_gnss_vel);
				}

				if (gnss_pos_enabled) {
					fuseHorizontalPosition(_aid_src_gnss_pos);
				}

				bool do_vel_pos_reset = shouldResetGpsFusion();

				if (isYawFailure()
				    && _control_status.flags.in_air
				    && isTimedOut(_time_last_hor_vel_fuse, _params.EKFGSF_reset_delay)
				    && (_time_last_hor_vel_fuse > _time_last_on_ground_us)) {
					do_vel_pos_reset = tryYawEmergencyReset();
				}

				if (do_vel_pos_reset) {
					ECL_WARN("GPS fusion timeout, resetting velocity / position");

					if (gnss_vel_enabled) {
						resetVelocityToGnss(_aid_src_gnss_vel);
					}

					if (gnss_pos_enabled) {
						resetHorizontalPositionToGnss(_aid_src_gnss_pos);
					}
				}

			} else {
				stopGpsFusion();
			}

		} else {
			if (starting_conditions_passing) {
				ECL_INFO("starting GPS fusion");
				_information_events.flags.starting_gps_fusion = true;

				// when already using another velocity source velocity reset is not necessary
				if (!isHorizontalAidingActive()
				    || isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
				    || !_control_status_prev.flags.yaw_align
				   ) {
					// reset velocity
					if (gnss_vel_enabled) {
						resetVelocityToGnss(_aid_src_gnss_vel);
					}
				}

				if (gnss_pos_enabled) {
					resetHorizontalPositionToGnss(_aid_src_gnss_pos);
				}

				_control_status.flags.gps = true;
			}
		}
	}
}

void Ekf::updateGnssVel(const gnssSample &gnss_sample, estimator_aid_source3d_s &aid_src)
{
	// correct velocity for offset relative to IMU
	const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;

	const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
	const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
	const Vector3f velocity = gnss_sample.vel - vel_offset_earth;

	const float vel_var = sq(math::max(gnss_sample.sacc, _params.gps_vel_noise));
	const Vector3f vel_obs_var(vel_var, vel_var, vel_var * sq(1.5f));
	updateVelocityAidSrcStatus(gnss_sample.time_us,
				   velocity,                                                   // observation
				   vel_obs_var,                                                // observation variance
				   math::max(_params.gps_vel_innov_gate, 1.f),                 // innovation gate
				   aid_src);
}

void Ekf::updateGnssPos(const gnssSample &gnss_sample, estimator_aid_source2d_s &aid_src)
{
	// correct position and height for offset relative to IMU
	const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
	const Vector2f position = _pos_ref.project(gnss_sample.lat, gnss_sample.lon) - pos_offset_earth.xy();

	// relax the upper observation noise limit which prevents bad GPS perturbing the position estimate
	float pos_noise = math::max(gnss_sample.hacc, _params.gps_pos_noise);

	if (!isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
		// if we are not using another source of aiding, then we are reliant on the GPS
		// observations to constrain attitude errors and must limit the observation noise value.
		if (pos_noise > _params.pos_noaid_noise) {
			pos_noise = _params.pos_noaid_noise;
		}
	}

	const float pos_var = sq(pos_noise);
	const Vector2f pos_obs_var(pos_var, pos_var);
	updateHorizontalPositionAidSrcStatus(gnss_sample.time_us,
					     position,                                   // observation
					     pos_obs_var,                                // observation variance
					     math::max(_params.gps_pos_innov_gate, 1.f), // innovation gate
					     aid_src);
}

void Ekf::controlGnssYawEstimator(estimator_aid_source3d_s &aid_src_vel)
{
	// update yaw estimator velocity (basic sanity check on GNSS velocity data)
	const float vel_var = aid_src_vel.observation_variance[0];
	const Vector2f vel_xy(aid_src_vel.observation);

	if ((vel_var > 0.f)
	    && (vel_var < _params.req_sacc)
	    && vel_xy.isAllFinite()) {

		_yawEstimator.fuseVelocity(vel_xy, vel_var, _control_status.flags.in_air);

		// Try to align yaw using estimate if available
		if (((_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::VEL))
		     || (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::HPOS)))
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
	 * fails while the difference between the yaw emergency estimator and the yas estimate is large.
	 * This enables recovery from a bad yaw estimate. A reset is not performed if the fault condition was
	 * present before flight to prevent triggering due to GPS glitches or other sensor errors.
	 */
	if (resetYawToEKFGSF()) {
		ECL_WARN("GPS emergency yaw reset");

		if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
			// stop using the magnetometer in the main EKF otherwise its fusion could drag the yaw around
			// and cause another navigation failure
			_control_status.flags.mag_fault = true;
			_warning_events.flags.emergency_yaw_reset_mag_stopped = true;
		}

#if defined(CONFIG_EKF2_GNSS_YAW)

		if (_control_status.flags.gps_yaw) {
			_control_status.flags.gps_yaw_fault = true;
			_warning_events.flags.emergency_yaw_reset_gps_yaw_stopped = true;
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
	aid_src.time_last_fuse = _time_delayed_us;
}

void Ekf::resetHorizontalPositionToGnss(estimator_aid_source2d_s &aid_src)
{
	_information_events.flags.reset_pos_to_gps = true;
	resetHorizontalPositionTo(Vector2f(aid_src.observation), Vector2f(aid_src.observation_variance));
	_gpos_origin_eph = 0.f; // The uncertainty of the global origin is now contained in the local position uncertainty
	aid_src.time_last_fuse = _time_delayed_us;
}

bool Ekf::shouldResetGpsFusion() const
{
	/* We are relying on aiding to constrain drift so after a specified time
	 * with no aiding we need to do something
	 */
	bool has_horizontal_aiding_timed_out = isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
					       && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max);

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (has_horizontal_aiding_timed_out) {
		// horizontal aiding hasn't timed out if optical flow still active
		if (_control_status.flags.opt_flow && isRecent(_aid_src_optical_flow.time_last_fuse, _params.reset_timeout_max)) {
			has_horizontal_aiding_timed_out = false;
		}
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

	const bool is_reset_required = has_horizontal_aiding_timed_out
				       || isTimedOut(_time_last_hor_pos_fuse, 2 * _params.reset_timeout_max);

	const bool is_inflight_nav_failure = _control_status.flags.in_air
					     && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
					     && isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
					     && (_time_last_hor_vel_fuse > _time_last_on_ground_us)
					     && (_time_last_hor_pos_fuse > _time_last_on_ground_us);

	return (is_reset_required || is_inflight_nav_failure);
}

#if defined(CONFIG_EKF2_GNSS_YAW)
void Ekf::controlGpsYawFusion(const gnssSample &gps_sample)
{
	if (!(_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::YAW))
	    || _control_status.flags.gps_yaw_fault) {

		stopGpsYawFusion();
		return;
	}

	updateGpsYaw(gps_sample);

	const bool is_new_data_available = PX4_ISFINITE(gps_sample.yaw);

	if (is_new_data_available) {

		const bool continuing_conditions_passing = _control_status.flags.tilt_align;

		const bool is_gps_yaw_data_intermittent = !isNewestSampleRecent(_time_last_gps_yaw_buffer_push,
				2 * GNSS_YAW_MAX_INTERVAL);

		const bool starting_conditions_passing = continuing_conditions_passing
				&& _gps_checks_passed
				&& !is_gps_yaw_data_intermittent
				&& !_gps_intermittent;

		if (_control_status.flags.gps_yaw) {

			if (continuing_conditions_passing) {

				fuseGpsYaw(gps_sample.yaw_offset);

				const bool is_fusion_failing = isTimedOut(_aid_src_gnss_yaw.time_last_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					if (_nb_gps_yaw_reset_available > 0) {
						// Data seems good, attempt a reset
						resetYawToGps(gps_sample.yaw, gps_sample.yaw_offset);

						if (_control_status.flags.in_air) {
							_nb_gps_yaw_reset_available--;
						}

					} else if (starting_conditions_passing) {
						// Data seems good, but previous reset did not fix the issue
						// something else must be wrong, declare the sensor faulty and stop the fusion
						_control_status.flags.gps_yaw_fault = true;
						stopGpsYawFusion();

					} else {
						// A reset did not fix the issue but all the starting checks are not passing
						// This could be a temporary issue, stop the fusion without declaring the sensor faulty
						stopGpsYawFusion();
					}

					// TODO: should we give a new reset credit when the fusion does not fail for some time?
				}

			} else {
				// Stop GPS yaw fusion but do not declare it faulty
				stopGpsYawFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate GPS yaw fusion
				if (resetYawToGps(gps_sample.yaw, gps_sample.yaw_offset)) {
					ECL_INFO("starting GPS yaw fusion");

					_aid_src_gnss_yaw.time_last_fuse = _time_delayed_us;
					_control_status.flags.gps_yaw = true;
					_control_status.flags.yaw_align = true;

					_nb_gps_yaw_reset_available = 1;
				}
			}
		}

	} else if (_control_status.flags.gps_yaw
		   && !isNewestSampleRecent(_time_last_gps_yaw_buffer_push, _params.reset_timeout_max)) {

		// No yaw data in the message anymore. Stop until it comes back.
		stopGpsYawFusion();
	}
}

void Ekf::stopGpsYawFusion()
{
	if (_control_status.flags.gps_yaw) {

		_control_status.flags.gps_yaw = false;
		resetEstimatorAidStatus(_aid_src_gnss_yaw);

		// Before takeoff, we do not want to continue to rely on the current heading
		// if we had to stop the fusion
		if (!_control_status.flags.in_air) {
			ECL_INFO("stopping GPS yaw fusion, clearing yaw alignment");
			_control_status.flags.yaw_align = false;

		} else {
			ECL_INFO("stopping GPS yaw fusion");
		}
	}
}
#endif // CONFIG_EKF2_GNSS_YAW

void Ekf::stopGpsFusion()
{
	if (_control_status.flags.gps) {
		ECL_INFO("stopping GPS position and velocity fusion");
		resetEstimatorAidStatus(_aid_src_gnss_pos);
		resetEstimatorAidStatus(_aid_src_gnss_vel);
		_last_gps_fail_us = 0;
		_last_gps_pass_us = 0;

		_control_status.flags.gps = false;
	}

	stopGpsHgtFusion();
#if defined(CONFIG_EKF2_GNSS_YAW)
	stopGpsYawFusion();
#endif // CONFIG_EKF2_GNSS_YAW

	_yawEstimator.reset();
}

bool Ekf::isYawEmergencyEstimateAvailable() const
{
	// don't allow reet using the EKF-GSF estimate until the filter has started fusing velocity
	// data and the yaw estimate has converged
	if (!_yawEstimator.isActive()) {
		return false;
	}

	return _yawEstimator.getYawVar() < sq(_params.EKFGSF_yaw_err_max);
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
