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

void Ekf::controlGnssFusion(const imuSample &imu_sample)
{
	if (!_gps_buffer || (_params.gnss_ctrl == 0)) {
		stopGnssFusion();
		return;
	}

	if (!gyro_bias_inhibited()) {
		_yawEstimator.setGyroBias(getGyroBias(), _control_status.flags.vehicle_at_rest);
	}

	// run EKF-GSF yaw estimator once per IMU update
	_yawEstimator.predict(imu_sample.delta_ang, imu_sample.delta_ang_dt,
			      imu_sample.delta_vel, imu_sample.delta_vel_dt,
			      (_control_status.flags.in_air && !_control_status.flags.vehicle_at_rest));


	// check for arrival of new sensor data at the fusion time horizon
	gnssSample gnss_sample;

	if (_gps_buffer->pop_first_older_than(imu_sample.time_us, &gnss_sample)) {

		if (runGnssChecks(gnss_sample)
		    && isTimedOut(_last_gps_fail_us, max((uint64_t)1e6, (uint64_t)_min_gps_health_time_us / 10))) {
			if (isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us)) {
				// First time checks are passing, latching.
				if (!_gps_checks_passed) {
					_information_events.flags.gps_checks_passed = true;
				}

				_gps_checks_passed = true;
			}

		} else {
			// Skip this sample
			_gps_data_ready = false;

			if ((_control_status.flags.gnss_vel || _control_status.flags.gnss_pos)
			    && isTimedOut(_last_gps_pass_us, _params.reset_timeout_max)) {
				stopGnssFusion();
				ECL_WARN("GNSS quality poor - stopping use");
			}
		}

		controlGnssHeightFusion(gnss_sample);

#if defined(CONFIG_EKF2_GNSS_YAW)
		controlGnssYawFusion(gnss_sample);
#endif // CONFIG_EKF2_GNSS_YAW

		controlGnssYawEstimator(_aid_src_gnss_vel);

		bool do_vel_pos_reset = false;

		if (_control_status.flags.gnss_vel || _control_status.flags.gnss_pos) {
			// Reset checks need to run together, before each control function runs because the result would change
			// after the first one runs
			do_vel_pos_reset = shouldResetGpsFusion();

			if (_control_status.flags.in_air
			    && isYawFailure()
			    && isTimedOut(_time_last_hor_vel_fuse, _params.EKFGSF_reset_delay)
			    && (_time_last_hor_vel_fuse > _time_last_on_ground_us)) {
				do_vel_pos_reset = tryYawEmergencyReset();
			}
		}

		controlGnssVelocityFusion(imu_sample, gnss_sample, do_vel_pos_reset);
		controlGnssPositionFusion(imu_sample, gnss_sample, do_vel_pos_reset);


	} else if (_control_status.flags.gnss_vel || _control_status.flags.gnss_pos) {
		if (!isNewestSampleRecent(_time_last_gps_buffer_push, _params.reset_timeout_max)) {
			stopGnssFusion();
			ECL_WARN("GNSS data stopped");
		}
	}
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
				       || (isTimedOut(_time_last_hor_pos_fuse, 2 * _params.reset_timeout_max)
					   && (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::HPOS)));

	const bool is_inflight_nav_failure = _control_status.flags.in_air
					     && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
					     && isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
					     && (_time_last_hor_vel_fuse > _time_last_on_ground_us)
					     && (_time_last_hor_pos_fuse > _time_last_on_ground_us);

	return (is_reset_required || is_inflight_nav_failure);
}

void Ekf::stopGnssFusion()
{
	if (_control_status.flags.gnss_vel || _control_status.flags.gnss_pos) {
		_last_gps_fail_us = 0;
		_last_gps_pass_us = 0;
	}

	stopGnssVelFusion();
	stopGnssPosFusion();
	stopGpsHgtFusion();
#if defined(CONFIG_EKF2_GNSS_YAW)
	stopGnssYawFusion();
#endif // CONFIG_EKF2_GNSS_YAW

	_yawEstimator.reset();
}
