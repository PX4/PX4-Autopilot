/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

void Ekf::controlGpsFusion()
{
	if (!(_params.fusion_mode & SensorFusionMask::USE_GPS)) {
		stopGpsFusion();
		return;
	}

	// Check for new GPS data that has fallen behind the fusion time horizon
	if (_gps_data_ready) {
		const gpsSample gps_sample{_gps_sample_delayed};

		// reset flags
		resetEstimatorAidStatusFlags(_aid_src_gnss_vel);
		resetEstimatorAidStatusFlags(_aid_src_gnss_pos);


		// correct velocity for offset relative to IMU
		const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
		const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
		const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;

		// correct position and height for offset relative to IMU
		const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

		const Vector3f gnss_velocity = gps_sample.vel - vel_offset_earth;

		Vector3f gnss_position;
		gnss_position.xy() = _pos_ref.project(gps_sample.lat, gps_sample.lon) - pos_offset_earth.xy();

		// GPS height: update offset
		if (!_control_status.flags.gps_hgt) {
			// calculate a filtered offset between the GNSS origin and local NED origin if we are not
			// using the GNSS as a height reference
			if (_delta_time_gnss_us != 0) {
				const float local_time_step = math::constrain(1e-6f * _delta_time_gnss_us, 0.0f, 1.0f);

				// apply a 10 second first order low pass filter to offset
				const float hgt_offset = gps_sample.alt + pos_offset_earth(2) - getEkfGlobalOriginAltitude() + _state.pos(2);

				const float offset_rate_correction = 0.1f * (hgt_offset + _state.pos(2) - _gps_hgt_offset);
				_gps_hgt_offset += local_time_step * math::constrain(offset_rate_correction, -0.1f, 0.1f);
			}
		}

		// vertical position - gps measurement has opposite sign to earth z axis
		gnss_position(2) = -((gps_sample.alt + pos_offset_earth(2)) - getEkfGlobalOriginAltitude() - _gps_hgt_offset);

		updateGpsVel(gps_sample.time_us, gnss_velocity, gps_sample.sacc);
		updateGpsPos(gps_sample.time_us, gnss_position, gps_sample.hacc, gps_sample.vacc);

		const bool gps_checks_passing = isTimedOut(_last_gps_fail_us, (uint64_t)5e6);
		const bool gps_checks_failing = isTimedOut(_last_gps_pass_us, (uint64_t)5e6);


		// Yaw EKFGSF update
		if (gps_checks_passing && !gps_checks_failing) {
			_yawEstimator.setVelocity(gnss_velocity.xy(), gps_sample.vacc);
		}

		// GPS yaw
		controlGpsYawFusion(gps_sample, gps_checks_passing, gps_checks_failing);


		// Determine if we should use GPS aiding for velocity and horizontal position
		// To start using GPS we need angular alignment completed, the local NED origin set and GPS data that has not failed checks recently
		const bool mandatory_conditions_passing = _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align
				&& _NED_origin_initialised;

		const bool continuing_conditions_passing = mandatory_conditions_passing && !gps_checks_failing;
		const bool starting_conditions_passing = continuing_conditions_passing && gps_checks_passing;

		if (_control_status.flags.gps) {
			if (mandatory_conditions_passing) {
				if (continuing_conditions_passing
				    || !isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {

					fuseGpsVel();
					fuseGpsPos();

					if (shouldResetGpsFusion()) {
						const bool was_gps_signal_lost = isTimedOut(_time_prev_gps_us, 1'000'000);

						/* A reset is not performed when getting GPS back after a significant period of no data
						 * because the timeout could have been caused by bad GPS.
						 * The total number of resets allowed per boot cycle is limited.
						 */
						if (isYawFailure()
						    && _control_status.flags.in_air
						    && !was_gps_signal_lost
						    && _ekfgsf_yaw_reset_count < _params.EKFGSF_reset_count_limit
						    && isTimedOut(_ekfgsf_yaw_reset_time, 5'000'000)) {
							// The minimum time interval between resets to the EKF-GSF estimate is limited to allow the EKF-GSF time
							// to improve its estimate if the previous reset was not successful.
							if (resetYawToEKFGSF()) {
								ECL_WARN("GPS emergency yaw reset");
							}

						} else {
							// use GPS velocity data to check and correct yaw angle if a FW vehicle
							if (_control_status.flags.fixed_wing && _control_status.flags.in_air) {
								// if flying a fixed wing aircraft, do a complete reset that includes yaw
								_mag_yaw_reset_req = true;
							}

							_warning_events.flags.gps_fusion_timout = true;
							ECL_WARN("GPS fusion timeout - resetting");
						}

						ECL_INFO("reset velocity & position to GNSS");
						_information_events.flags.reset_vel_to_gps = true;
						_information_events.flags.reset_pos_to_gps = true;

						resetVelocityTo(gnss_velocity);
						resetHorizontalPositionTo(gnss_position.xy());

						P.uncorrelateCovarianceSetVariance<3>(4, sq(gps_sample.sacc));
						P.uncorrelateCovarianceSetVariance<2>(7, sq(gps_sample.hacc));
					}

				} else {
					stopGpsFusion();
					_warning_events.flags.gps_quality_poor = true;
					ECL_WARN("GPS quality poor - stopping use");

					// TODO: move this to EV control logic
					// Reset position state to external vision if we are going to use absolute values
					if (_control_status.flags.ev_pos && !(_params.fusion_mode & SensorFusionMask::ROTATE_EXT_VIS)) {
						resetHorizontalPositionToVision();
					}
				}

			} else { // mandatory conditions are not passing
				stopGpsFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Do not use external vision for yaw if using GPS because yaw needs to be
				// defined relative to an NED reference frame
				if (_control_status.flags.ev_yaw
				    || _mag_inhibit_yaw_reset_req
				    || _mag_yaw_reset_req) {

					_mag_yaw_reset_req = true;

					// Stop the vision for yaw fusion and do not allow it to start again
					stopEvYawFusion();
					_inhibit_ev_yaw_use = true;

				} else {

					if (!_control_status.flags.gps) {
						_information_events.flags.starting_gps_fusion = true;
						ECL_INFO("starting GPS fusion");

						_information_events.flags.reset_pos_to_gps = true;
						resetHorizontalPositionTo(gnss_position.xy());
						P.uncorrelateCovarianceSetVariance<2>(7, sq(gps_sample.hacc));

						// when already using another velocity source velocity reset is not necessary
						if (!_control_status.flags.opt_flow && !_control_status.flags.ev_vel) {
							// resetVelocityToGps
							ECL_INFO("reset velocity to GPS");
							_information_events.flags.reset_vel_to_gps = true;
							resetVelocityTo(gnss_velocity);
							P.uncorrelateCovarianceSetVariance<3>(4, sq(gps_sample.sacc));
						}

						_control_status.flags.gps = true;
					}
				}

			} else if (gps_checks_passing && !_control_status.flags.yaw_align && (_params.mag_fusion_type == MagFuseType::NONE)) {
				// If no mag is used, align using the yaw estimator (if available)
				if (resetYawToEKFGSF()) {
					_information_events.flags.yaw_aligned_to_imu_gps = true;
					ECL_INFO("Yaw aligned using IMU and GPS");

					_information_events.flags.reset_vel_to_gps = true;
					_information_events.flags.reset_pos_to_gps = true;

					resetVelocityTo(gnss_velocity);
					resetHorizontalPositionTo(gnss_position.xy());
					P.uncorrelateCovarianceSetVariance<3>(4, sq(gps_sample.sacc));
					P.uncorrelateCovarianceSetVariance<2>(7, sq(gps_sample.hacc));
				}
			}
		}

	} else if (_control_status.flags.gps && (_imu_sample_delayed.time_us - _time_prev_gps_us > (uint64_t)10e6)) {
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped = true;
		ECL_WARN("GPS data stopped");

	}  else if (_control_status.flags.gps && (_imu_sample_delayed.time_us - _time_prev_gps_us > (uint64_t)1e6)
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
	const bool is_reset_required = hasHorizontalAidingTimedOut()
				       || isTimedOut(_time_last_hor_pos_fuse, 2 * _params.reset_timeout_max);

	/* Logic controlling the reset of navigation filter yaw to the EKF-GSF estimate to recover from loss of
	 * navigation casued by a bad yaw estimate.

	 * A rapid reset to the EKF-GSF estimate is performed after a recent takeoff if horizontal velocity
	 * innovation checks fail. This enables recovery from a bad yaw estimate. After 30 seconds from takeoff,
	 * different test criteria are used that take longer to trigger and reduce false positives. A reset is
	 * not performed if the fault condition was present before flight to prevent triggering due to GPS glitches
	 * or other sensor errors.
	 */
	const bool is_recent_takeoff_nav_failure = _control_status.flags.in_air
			&& isRecent(_time_last_on_ground_us, 30000000)
			&& isTimedOut(_time_last_hor_vel_fuse, _params.EKFGSF_reset_delay)
			&& (_time_last_hor_vel_fuse > _time_last_on_ground_us);

	const bool is_inflight_nav_failure = _control_status.flags.in_air
					     && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
					     && isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
					     && (_time_last_hor_vel_fuse > _time_last_on_ground_us)
					     && (_time_last_hor_pos_fuse > _time_last_on_ground_us);

	return (is_reset_required || is_recent_takeoff_nav_failure || is_inflight_nav_failure);
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
