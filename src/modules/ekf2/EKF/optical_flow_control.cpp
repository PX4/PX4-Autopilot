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
 * @file optical_flow_control.cpp
 * Control functions for optical flow fusion
 */

#include "ekf.h"

void Ekf::controlOpticalFlowFusion(const imuSample &imu_delayed)
{
	if (_flow_buffer) {
		// We don't fuse flow data immediately because we have to wait for the mid integration point to fall behind the fusion time horizon.
		// This means we stop looking for new data until the old data has been fused, unless we are not fusing optical flow,
		// in this case we need to empty the buffer
		if (!_flow_data_ready || (!_control_status.flags.opt_flow && !_hagl_sensor_status.flags.flow)) {
			_flow_data_ready = _flow_buffer->pop_first_older_than(imu_delayed.time_us, &_flow_sample_delayed);
		}
	}

	// Check if on ground motion is un-suitable for use of optical flow
	if (!_control_status.flags.in_air) {
		updateOnGroundMotionForOpticalFlowChecks();

	} else {
		resetOnGroundMotionForOpticalFlowChecks();
	}

	// Accumulate autopilot gyro data across the same time interval as the flow sensor
	const Vector3f delta_angle(imu_delayed.delta_ang - (getGyroBias() * imu_delayed.delta_ang_dt));
	if (_delta_time_of < 0.1f) {
		_imu_del_ang_of += delta_angle;
		_delta_time_of += imu_delayed.delta_ang_dt;

	} else {
		// reset the accumulators if the time interval is too large
		_imu_del_ang_of = delta_angle;
		_delta_time_of = imu_delayed.delta_ang_dt;
	}

	if (_flow_data_ready) {
		const bool is_quality_good = (_flow_sample_delayed.quality >= _params.flow_qual_min);
		const bool is_magnitude_good = !_flow_sample_delayed.flow_xy_rad.longerThan(_flow_sample_delayed.dt * _flow_max_rate);
		const bool is_tilt_good = (_R_to_earth(2, 2) > _params.range_cos_max_tilt);

		const float delta_time_min = fmaxf(0.7f * _delta_time_of, 0.001f);
		const float delta_time_max = fminf(1.3f * _delta_time_of, 0.2f);
		bool is_delta_time_good = _flow_sample_delayed.dt >= delta_time_min && _flow_sample_delayed.dt <= delta_time_max;

		if (!is_delta_time_good && (_flow_sample_delayed.dt > FLT_EPSILON)) {

			if (fabsf(imu_delayed.delta_ang_dt - _flow_sample_delayed.dt) < 0.1f) {
				// reset accumulators to current IMU
				_imu_del_ang_of = delta_angle;
				_delta_time_of = imu_delayed.delta_ang_dt;

				is_delta_time_good = true;
			}

			if (is_quality_good && !is_delta_time_good) {
				ECL_DEBUG("Optical flow: bad delta time: OF dt %.6f s (min: %.3f, max: %.3f), IMU dt %.6f s",
					  (double)_flow_sample_delayed.dt, (double)delta_time_min, (double)delta_time_max,
					  (double)imu_delayed.delta_ang_dt);
			}
		}

		const bool is_body_rate_comp_available = calcOptFlowBodyRateComp();

		// don't allow invalid flow gyro_xyz to propagate
		if (!_flow_sample_delayed.gyro_xyz.isAllFinite()) {
			_flow_sample_delayed.gyro_xyz.zero();
		}

		if (is_quality_good
		    && is_magnitude_good
		    && is_tilt_good
		    && is_body_rate_comp_available
		    && is_delta_time_good) {
			// compensate for body motion to give a LOS rate
			_flow_compensated_XY_rad = _flow_sample_delayed.flow_xy_rad - _flow_sample_delayed.gyro_xyz.xy();

		} else if (!_control_status.flags.in_air) {

			if (!is_delta_time_good) {
				// handle special case of SITL and PX4Flow where dt is forced to
				// zero when the quaity is 0
				_flow_sample_delayed.dt = delta_time_min;
			}

			// when on the ground with poor flow quality,
			// assume zero ground relative velocity and LOS rate
			_flow_compensated_XY_rad.setZero();

		} else {
			// don't use this flow data and wait for the next data to arrive
			_flow_data_ready = false;
			_flow_compensated_XY_rad.setZero();
		}

		updateOptFlow(_aid_src_optical_flow);

	} else {
		_flow_compensated_XY_rad.setZero();
	}

	// New optical flow data is available and is ready to be fused when the midpoint of the sample falls behind the fusion time horizon
	if (_flow_data_ready) {

		// Inhibit flow use if motion is un-suitable or we have good quality GPS
		// Apply hysteresis to prevent rapid mode switching
		const float gps_err_norm_lim = _control_status.flags.opt_flow ? 0.7f : 1.0f;

		// Check if we are in-air and require optical flow to control position drift
		const bool is_flow_required = _control_status.flags.in_air
					      && (_control_status.flags.inertial_dead_reckoning // is doing inertial dead-reckoning so must constrain drift urgently
						  || isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow)
						  || (_control_status.flags.gps && (_gps_error_norm > gps_err_norm_lim))); // is using GPS, but GPS is bad

		// inhibit use of optical flow if motion is unsuitable and we are not reliant on it for flight navigation
		const bool preflight_motion_not_ok = !_control_status.flags.in_air
						     && ((_time_delayed_us > (_time_good_motion_us + (uint64_t)1E5))
								     || (_time_delayed_us < (_time_bad_motion_us + (uint64_t)5E6)));
		const bool flight_condition_not_ok = _control_status.flags.in_air && !isTerrainEstimateValid();

		const bool inhibit_flow_use = ((preflight_motion_not_ok || flight_condition_not_ok) && !is_flow_required)
				    || !_control_status.flags.tilt_align;

		// Handle cases where we are using optical flow but we should not use it anymore
		if (_control_status.flags.opt_flow) {
			if (!(_params.flow_ctrl == 1)
			    || inhibit_flow_use) {

				stopFlowFusion();
				return;
			}
		}

		// optical flow fusion mode selection logic
		if ((_params.flow_ctrl == 1) // optical flow has been selected by the user
		    && !_control_status.flags.opt_flow // we are not yet using flow data
		    && !inhibit_flow_use) {

			// set the flag and reset the fusion timeout
			ECL_INFO("starting optical flow fusion");

			// if we are not using GPS or external vision aiding, then the velocity and position states and covariances need to be set
			if (!isHorizontalAidingActive()) {
				ECL_INFO("reset velocity to flow");
				_information_events.flags.reset_vel_to_flow = true;
				resetHorizontalVelocityTo(_flow_vel_ne, calcOptFlowMeasVar(_flow_sample_delayed));

				// reset position, estimate is relative to initial position in this mode, so we start with zero error
				if (!_control_status.flags.in_air) {
					ECL_INFO("reset position to zero");
					resetHorizontalPositionTo(Vector2f(0.f, 0.f), 0.f);
					_last_known_pos.xy() = _state.pos.xy();

				} else {
					_information_events.flags.reset_pos_to_last_known = true;
					ECL_INFO("reset position to last known (%.3f, %.3f)", (double)_last_known_pos(0), (double)_last_known_pos(1));
					resetHorizontalPositionTo(_last_known_pos.xy(), 0.f);
				}
			}

			_aid_src_optical_flow.time_last_fuse = _time_delayed_us;
			_control_status.flags.opt_flow = true;

			return;
		}

		if (_control_status.flags.opt_flow) {
			// Wait until the midpoint of the flow sample has fallen behind the fusion time horizon
			if (_time_delayed_us > (_flow_sample_delayed.time_us - uint32_t(1e6f * _flow_sample_delayed.dt) / 2)) {
				// Fuse optical flow LOS rate observations into the main filter only if height above ground has been updated recently
				// but use a relaxed time criteria to enable it to coast through bad range finder data
				if (isRecent(_time_last_hagl_fuse, (uint64_t)10e6)) {
					fuseOptFlow();
					_last_known_pos.xy() = _state.pos.xy();
				}

				_flow_data_ready = false;
			}

			// handle the case when we have optical flow, are reliant on it, but have not been using it for an extended period
			if (isTimedOut(_aid_src_optical_flow.time_last_fuse, _params.no_aid_timeout_max)
			    && !isOtherSourceOfHorizontalAidingThan(_control_status.flags.opt_flow)) {

				ECL_INFO("reset velocity to flow");
				_information_events.flags.reset_vel_to_flow = true;
				resetHorizontalVelocityTo(_flow_vel_ne, calcOptFlowMeasVar(_flow_sample_delayed));

				// reset position, estimate is relative to initial position in this mode, so we start with zero error
				ECL_INFO("reset position to last known (%.3f, %.3f)", (double)_last_known_pos(0), (double)_last_known_pos(1));
				_information_events.flags.reset_pos_to_last_known = true;
				resetHorizontalPositionTo(_last_known_pos.xy(), 0.f);

				_aid_src_optical_flow.time_last_fuse = _time_delayed_us;
			}
		}

	} else if (_control_status.flags.opt_flow && !isRecent(_flow_sample_delayed.time_us, (uint64_t)10e6)) {

		stopFlowFusion();
	}
}

void Ekf::updateOnGroundMotionForOpticalFlowChecks()
{
	// When on ground check if the vehicle is being shaken or moved in a way that could cause a loss of navigation
	const float accel_norm = _accel_vec_filt.norm();

	const bool motion_is_excessive = ((accel_norm > (CONSTANTS_ONE_G * 1.5f)) // upper g limit
					  || (accel_norm < (CONSTANTS_ONE_G * 0.5f)) // lower g limit
					  || (_ang_rate_magnitude_filt > _flow_max_rate) // angular rate exceeds flow sensor limit
					  || (_R_to_earth(2, 2) < cosf(math::radians(30.0f)))); // tilted excessively

	if (motion_is_excessive) {
		_time_bad_motion_us = _time_delayed_us;

	} else {
		_time_good_motion_us = _time_delayed_us;
	}
}

void Ekf::resetOnGroundMotionForOpticalFlowChecks()
{
	_time_bad_motion_us = 0;
	_time_good_motion_us = _time_delayed_us;
}

void Ekf::stopFlowFusion()
{
	if (_control_status.flags.opt_flow) {
		ECL_INFO("stopping optical flow fusion");
		_control_status.flags.opt_flow = false;

		resetEstimatorAidStatus(_aid_src_optical_flow);
	}
}
