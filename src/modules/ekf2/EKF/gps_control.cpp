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
	if (!_gps_buffer || !((_params.gnss_ctrl & GnssCtrl::HPOS) || (_params.gnss_ctrl & GnssCtrl::VEL))) {
		stopGpsFusion();
		return;
	}

	_gps_intermittent = !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL);

	// check for arrival of new sensor data at the fusion time horizon
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

	// run EKF-GSF yaw estimator once per imu_delayed update after all main EKF data samples available
	_yawEstimator.update(imu_delayed, _control_status.flags.in_air, getGyroBias());

	// Check for new GPS data that has fallen behind the fusion time horizon
	if (_gps_data_ready) {

		const gpsSample &gps_sample{_gps_sample_delayed};

		const bool gps_checks_passing = isTimedOut(_last_gps_fail_us, (uint64_t)5e6);
		const bool gps_checks_failing = isTimedOut(_last_gps_pass_us, (uint64_t)5e6);

#if defined(CONFIG_EKF2_GNSS_YAW)
		controlGpsYawFusion(gps_sample, gps_checks_passing, gps_checks_failing);
#endif // CONFIG_EKF2_GNSS_YAW

		// GNSS velocity
		const Vector3f velocity{gps_sample.vel};
		const float vel_var = sq(math::max(gps_sample.sacc, _params.gps_vel_noise));
		const Vector3f vel_obs_var(vel_var, vel_var, vel_var * sq(1.5f));
		updateVelocityAidSrcStatus(gps_sample.time_us,
					   velocity,                                                   // observation
					   vel_obs_var,                                                // observation variance
					   math::max(_params.gps_vel_innov_gate, 1.f),                 // innovation gate
					   _aid_src_gnss_vel);
		_aid_src_gnss_vel.fusion_enabled = (_params.gnss_ctrl & GnssCtrl::VEL);

		// GNSS position
		const Vector2f position{gps_sample.pos};
		// relax the upper observation noise limit which prevents bad GPS perturbing the position estimate
		float pos_noise = math::max(gps_sample.hacc, _params.gps_pos_noise);

		if (!isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
			// if we are not using another source of aiding, then we are reliant on the GPS
			// observations to constrain attitude errors and must limit the observation noise value.
			if (pos_noise > _params.pos_noaid_noise) {
				pos_noise = _params.pos_noaid_noise;
			}
		}

		const float pos_var = sq(pos_noise);
		const Vector2f pos_obs_var(pos_var, pos_var);
		updateHorizontalPositionAidSrcStatus(gps_sample.time_us,
						     position,                                   // observation
						     pos_obs_var,                                // observation variance
						     math::max(_params.gps_pos_innov_gate, 1.f), // innovation gate
						     _aid_src_gnss_pos);
		_aid_src_gnss_pos.fusion_enabled = (_params.gnss_ctrl & GnssCtrl::HPOS);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		// if GPS is otherwise ready to go, but yaw_align is blocked by EV give mag a chance to start
		if (_control_status.flags.tilt_align && _NED_origin_initialised
		    && gps_checks_passing && !gps_checks_failing) {

			if (!_control_status.flags.yaw_align) {
				if (_control_status.flags.ev_yaw && !_control_status.flags.yaw_align) {

					// give mag a chance to start and yaw align if currently blocked by EV yaw
					const bool mag_enabled = (_params.mag_fusion_type <= MagFuseType::MAG_3D);
					const bool mag_available = (_mag_counter != 0);

					if (mag_enabled && mag_available
					    && !_control_status.flags.mag_field_disturbed
					    && !_control_status.flags.mag_fault) {

						stopEvYawFusion();
					}
				}
			}
		}
#endif // CONFIG_EKF2_EXTERNAL_VISION

		// Determine if we should use GPS aiding for velocity and horizontal position
		// To start using GPS we need angular alignment completed, the local NED origin set and GPS data that has not failed checks recently
		const bool mandatory_conditions_passing = ((_params.gnss_ctrl & GnssCtrl::HPOS) || (_params.gnss_ctrl & GnssCtrl::VEL))
				&& _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align
				&& _NED_origin_initialised;

		const bool continuing_conditions_passing = mandatory_conditions_passing && !gps_checks_failing;
		const bool starting_conditions_passing = continuing_conditions_passing && gps_checks_passing;

		if (_control_status.flags.gps) {
			if (mandatory_conditions_passing) {
				if (continuing_conditions_passing
				    || !isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {

					fuseVelocity(_aid_src_gnss_vel);
					fuseHorizontalPosition(_aid_src_gnss_pos);

					bool do_vel_pos_reset = shouldResetGpsFusion();

					if (isYawFailure()
					    && _control_status.flags.in_air
					    && isTimedOut(_time_last_hor_vel_fuse, _params.EKFGSF_reset_delay)
					    && (_time_last_hor_vel_fuse > _time_last_on_ground_us)) {
						/* A rapid reset to the yaw emergency estimate is performed if horizontal velocity innovation checks continuously
						 * fails while the difference between the yaw emergency estimator and the yas estimate is large.
						 * This enables recovery from a bad yaw estimate. A reset is not performed if the fault condition was
						 * present before flight to prevent triggering due to GPS glitches or other sensor errors.
						 */
						if (resetYawToEKFGSF()) {
							ECL_WARN("GPS emergency yaw reset");
							do_vel_pos_reset = true;
						}
					}

					if (do_vel_pos_reset) {
						ECL_WARN("GPS fusion timeout, resetting velocity and position");

						// reset velocity
						_information_events.flags.reset_vel_to_gps = true;
						resetVelocityTo(velocity, vel_obs_var);
						_aid_src_gnss_vel.time_last_fuse = _time_delayed_us;

						// reset position
						_information_events.flags.reset_pos_to_gps = true;
						resetHorizontalPositionTo(position, pos_obs_var);
						_aid_src_gnss_pos.time_last_fuse = _time_delayed_us;
					}

				} else {
					stopGpsFusion();
					_warning_events.flags.gps_quality_poor = true;
					ECL_WARN("GPS quality poor - stopping use");
				}

			} else { // mandatory conditions are not passing
				stopGpsFusion();
			}

		} else {
			if (starting_conditions_passing) {
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
				// Do not use external vision for yaw if using GPS because yaw needs to be
				// defined relative to an NED reference frame
				if (_control_status.flags.ev_yaw) {
					// Stop the vision for yaw fusion and do not allow it to start again
					stopEvYawFusion();
					_inhibit_ev_yaw_use = true;
				}
#endif // CONFIG_EKF2_EXTERNAL_VISION

				ECL_INFO("starting GPS fusion");
				_information_events.flags.starting_gps_fusion = true;

				// when already using another velocity source velocity reset is not necessary
				if (!isHorizontalAidingActive()
				    || isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
				    || !_control_status_prev.flags.yaw_align
				   ) {
					// reset velocity
					_information_events.flags.reset_vel_to_gps = true;
					resetVelocityTo(velocity, vel_obs_var);
					_aid_src_gnss_vel.time_last_fuse = _time_delayed_us;
				}

				// reset position
				_information_events.flags.reset_pos_to_gps = true;
				resetHorizontalPositionTo(position, pos_obs_var);
				_aid_src_gnss_pos.time_last_fuse = _time_delayed_us;

				_control_status.flags.gps = true;

			} else if (gps_checks_passing && !_control_status.flags.yaw_align && (_params.mag_fusion_type == MagFuseType::NONE)) {
				// If no mag is used, align using the yaw estimator (if available)
				if (resetYawToEKFGSF()) {
					_information_events.flags.yaw_aligned_to_imu_gps = true;
					ECL_INFO("GPS yaw aligned using IMU, resetting vel and pos");

					// reset velocity
					_information_events.flags.reset_vel_to_gps = true;
					resetVelocityTo(velocity, vel_obs_var);
					_aid_src_gnss_vel.time_last_fuse = _time_delayed_us;

					// reset position
					_information_events.flags.reset_pos_to_gps = true;
					resetHorizontalPositionTo(position, pos_obs_var);
					_aid_src_gnss_pos.time_last_fuse = _time_delayed_us;
				}
			}
		}

	} else if (_control_status.flags.gps && !isNewestSampleRecent(_time_last_gps_buffer_push, (uint64_t)10e6)) {
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped = true;
		ECL_WARN("GPS data stopped");

	}  else if (_control_status.flags.gps && !isNewestSampleRecent(_time_last_gps_buffer_push, (uint64_t)1e6)
		    && isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {

		// Handle the case where we are fusing another position source along GPS,
		// stop waiting for GPS after 1 s of lost signal
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped_using_alternate = true;
		ECL_WARN("GPS data stopped, using only EV, OF or air data");
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
				       || isTimedOut(_time_last_hor_pos_fuse, 2 * _params.reset_timeout_max);

	const bool is_inflight_nav_failure = _control_status.flags.in_air
					     && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
					     && isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
					     && (_time_last_hor_vel_fuse > _time_last_on_ground_us)
					     && (_time_last_hor_pos_fuse > _time_last_on_ground_us);

	return (is_reset_required || is_inflight_nav_failure);
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

#if defined(CONFIG_EKF2_GNSS_YAW)
void Ekf::controlGpsYawFusion(const gpsSample &gps_sample, bool gps_checks_passing, bool gps_checks_failing)
{
	if (!(_params.gnss_ctrl & GnssCtrl::YAW)
	    || _control_status.flags.gps_yaw_fault) {

		stopGpsYawFusion();
		return;
	}

	updateGpsYaw(gps_sample);

	const bool is_new_data_available = PX4_ISFINITE(gps_sample.yaw);

	if (is_new_data_available) {

		const bool continuing_conditions_passing = !gps_checks_failing;

		const bool is_gps_yaw_data_intermittent = !isNewestSampleRecent(_time_last_gps_yaw_buffer_push,
				2 * GNSS_YAW_MAX_INTERVAL);

		const bool starting_conditions_passing = continuing_conditions_passing
				&& _control_status.flags.tilt_align
				&& gps_checks_passing
				&& !is_gps_yaw_data_intermittent
				&& !_gps_intermittent;

		if (_control_status.flags.gps_yaw) {

			if (continuing_conditions_passing) {

				fuseGpsYaw();

				const bool is_fusion_failing = isTimedOut(_aid_src_gnss_yaw.time_last_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					if (_nb_gps_yaw_reset_available > 0) {
						// Data seems good, attempt a reset
						resetYawToGps(gps_sample.yaw);

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
				startGpsYawFusion(gps_sample);

				if (_control_status.flags.gps_yaw) {
					_nb_gps_yaw_reset_available = 1;
				}
			}
		}

	} else if (_control_status.flags.gps_yaw
		   && !isNewestSampleRecent(_time_last_gps_yaw_buffer_push, _params.reset_timeout_max)) {

		// No yaw data in the message anymore. Stop until it comes back.
		stopGpsYawFusion();
	}

	// Before takeoff, we do not want to continue to rely on the current heading
	// if we had to stop the fusion
	if (!_control_status.flags.in_air
	    && !_control_status.flags.gps_yaw
	    && _control_status_prev.flags.gps_yaw) {
		_control_status.flags.yaw_align = false;
	}
}

void Ekf::startGpsYawFusion(const gpsSample &gps_sample)
{
	if (!_control_status.flags.gps_yaw && resetYawToGps(gps_sample.yaw)) {
		ECL_INFO("starting GPS yaw fusion");
		_control_status.flags.yaw_align = true;
		_control_status.flags.mag_dec = false;

		stopMagHdgFusion();
		stopMag3DFusion();
		_control_status.flags.gps_yaw = true;
	}
}
#endif // CONFIG_EKF2_GNSS_YAW

void Ekf::stopGpsYawFusion()
{
#if defined(CONFIG_EKF2_GNSS_YAW)
	if (_control_status.flags.gps_yaw) {
		ECL_INFO("stopping GPS yaw fusion");
		_control_status.flags.gps_yaw = false;
		resetEstimatorAidStatus(_aid_src_gnss_yaw);
	}
#endif // CONFIG_EKF2_GNSS_YAW
}

void Ekf::stopGpsFusion()
{
	if (_control_status.flags.gps) {
		stopGpsPosFusion();
		stopGpsVelFusion();

		_control_status.flags.gps = false;
	}

	if (_control_status.flags.gps_yaw) {
		stopGpsYawFusion();
	}

	// We do not need to know the true North anymore
	// EV yaw can start again
	_inhibit_ev_yaw_use = false;
}

void Ekf::stopGpsPosFusion()
{
	if (_control_status.flags.gps) {
		ECL_INFO("stopping GPS position fusion");
		_control_status.flags.gps = false;

		resetEstimatorAidStatus(_aid_src_gnss_pos);
	}
}

void Ekf::stopGpsVelFusion()
{
	ECL_INFO("stopping GPS velocity fusion");

	resetEstimatorAidStatus(_aid_src_gnss_vel);
}
