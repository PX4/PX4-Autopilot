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
	if (!(_params.fusion_mode & MASK_USE_GPS)) {
		stopGpsFusion();
		return;
	}

	// Check for new GPS data that has fallen behind the fusion time horizon
	if (_gps_data_ready) {
		const bool gps_checks_passing = isTimedOut(_last_gps_fail_us, (uint64_t)5e6);
		const bool gps_checks_failing = isTimedOut(_last_gps_pass_us, (uint64_t)5e6);

		controlGpsYawFusion(gps_checks_passing, gps_checks_failing);

		// Determine if we should use GPS aiding for velocity and horizontal position
		// To start using GPS we need angular alignment completed, the local NED origin set and GPS data that has not failed checks recently
		const bool mandatory_conditions_passing = _control_status.flags.tilt_align
		                                          && _control_status.flags.yaw_align
							  && _NED_origin_initialised;
		const bool continuing_conditions_passing = mandatory_conditions_passing
		                                           && !gps_checks_failing;
		const bool starting_conditions_passing = continuing_conditions_passing
							 && gps_checks_passing;

		if (_control_status.flags.gps) {
			if (mandatory_conditions_passing) {
				if (continuing_conditions_passing
				    || !isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {

					fuseGpsVelPos();

					if (shouldResetGpsFusion()){
						const bool is_yaw_failure = !isVelStateAlignedWithObs();
						const bool was_gps_signal_lost = isTimedOut(_time_prev_gps_us, 1000000);

						/* A reset is not performed when getting GPS back after a significant period of no data
						 * because the timeout could have been caused by bad GPS.
						 * The total number of resets allowed per boot cycle is limited.
						 */
						if (is_yaw_failure
						    && _control_status.flags.in_air
						    && !was_gps_signal_lost
						    && _ekfgsf_yaw_reset_count < _params.EKFGSF_reset_count_limit) {

							_do_ekfgsf_yaw_reset = true;

						} else {
							// use GPS velocity data to check and correct yaw angle if a FW vehicle
							if (_control_status.flags.fixed_wing && _control_status.flags.in_air) {
								// if flying a fixed wing aircraft, do a complete reset that includes yaw
								_control_status.flags.mag_aligned_in_flight = realignYawGPS();
							}

							_warning_events.flags.gps_fusion_timout = true;
							ECL_WARN("GPS fusion timeout - resetting");
							_velpos_reset_request = true;
						}
					}

				} else {
					stopGpsFusion();
					_warning_events.flags.gps_quality_poor = true;
					ECL_WARN("GPS quality poor - stopping use");

					// TODO: move this to EV control logic
					// Reset position state to external vision if we are going to use absolute values
					if (_control_status.flags.ev_pos && !(_params.fusion_mode & MASK_ROTATE_EV)) {
						resetHorizontalPosition();
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
					startGpsFusion();
				}

			} else if(!_control_status.flags.yaw_align
		                  && (_params.mag_fusion_type == MAG_FUSE_TYPE_NONE)) {
				// If no mag is used, align using the yaw estimator
				_do_ekfgsf_yaw_reset = true;
			}
		}

		processYawEstimatorResetRequest();
		processVelPosResetRequest();

	} else if (_control_status.flags.gps && (_imu_sample_delayed.time_us - _gps_sample_delayed.time_us > (uint64_t)10e6)) {
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped = true;
		ECL_WARN("GPS data stopped");

	}  else if (_control_status.flags.gps && (_imu_sample_delayed.time_us - _gps_sample_delayed.time_us > (uint64_t)1e6)
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

bool Ekf::hasHorizontalAidingTimedOut() const
{
	return isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
	       && isTimedOut(_time_last_delpos_fuse, _params.reset_timeout_max)
	       && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
	       && isTimedOut(_time_last_of_fuse, _params.reset_timeout_max);
}

bool Ekf::isVelStateAlignedWithObs() const
{
	/* Do sanity check to see if the innovation failures is likely caused by a yaw angle error
	 * by measuring the angle between the velocity estimate and the last velocity observation
	 * Only use those vectors if their norm if they are larger than 4 times their noise standard deviation
	 */
	const float vel_obs_xy_norm_sq = _last_vel_obs.xy().norm_squared();
	const float vel_state_xy_norm_sq = _state.vel.xy().norm_squared();

	const float vel_obs_threshold_sq = fmaxf(sq(4.f) * (_last_vel_obs_var(0) + _last_vel_obs_var(1)), sq(0.4f));
	const float vel_state_threshold_sq = fmaxf(sq(4.f) * (P(4, 4) + P(5, 5)), sq(0.4f));

	if (vel_obs_xy_norm_sq > vel_obs_threshold_sq && vel_state_xy_norm_sq > vel_state_threshold_sq) {
		const float obs_dot_vel = Vector2f(_last_vel_obs).dot(_state.vel.xy());
		const float cos_sq = sq(obs_dot_vel) / (vel_state_xy_norm_sq * vel_obs_xy_norm_sq);

		if (cos_sq < sq(cosf(math::radians(25.f))) || obs_dot_vel < 0.f) {
			// The angle between the observation and the velocity estimate is greater than 25 degrees
			return false;
		}
	}

	return true;
}

void Ekf::processYawEstimatorResetRequest()
{
	/* The yaw reset to the EKF-GSF estimate can be requested externally at any time during flight.
	 * The minimum time interval between resets to the EKF-GSF estimate is limited to allow the EKF-GSF time
	 * to improve its estimate if the previous reset was not successful.
	 */
	if (_do_ekfgsf_yaw_reset
	    && isTimedOut(_ekfgsf_yaw_reset_time, 5000000)){
		if (resetYawToEKFGSF()) {
			_ekfgsf_yaw_reset_time = _time_last_imu;
			_time_last_hor_pos_fuse = _time_last_imu;
			_time_last_hor_vel_fuse = _time_last_imu;

			_do_ekfgsf_yaw_reset = false;
			_velpos_reset_request = false; // included in yaw reset
			_ekfgsf_yaw_reset_count++;
		}
	}
}

void Ekf::processVelPosResetRequest()
{
	if (_velpos_reset_request) {
		resetVelocity();
		resetHorizontalPosition();
		_velpos_reset_request = false;

		// Reset the timeout counters
		_time_last_hor_pos_fuse = _time_last_imu;
		_time_last_hor_vel_fuse = _time_last_imu;
	}
}
