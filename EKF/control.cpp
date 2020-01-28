/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file control.cpp
 * Control functions for ekf attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "../ecl.h"
#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlFusionModes()
{
	// Store the status to enable change detection
	_control_status_prev.value = _control_status.value;

	// monitor the tilt alignment
	if (!_control_status.flags.tilt_align) {
		// whilst we are aligning the tilt, monitor the variances
		const Vector3f angle_err_var_vec = calcRotVecVariances();

		// Once the tilt variances have reduced to equivalent of 3deg uncertainty, re-set the yaw and magnetic field states
		// and declare the tilt alignment complete
		if ((angle_err_var_vec(0) + angle_err_var_vec(1)) < sq(math::radians(3.0f))) {
			_control_status.flags.tilt_align = true;
			_control_status.flags.yaw_align = resetMagHeading(_mag_lpf.getState()); // TODO: is this needed?

			// send alignment status message to the console
			const char* height_source = nullptr;
			if (_control_status.flags.baro_hgt) {
				height_source = "baro";

			} else if (_control_status.flags.ev_hgt) {
				height_source = "ev";

			} else if (_control_status.flags.gps_hgt) {
				height_source = "gps";

			} else if (_control_status.flags.rng_hgt) {
				height_source = "range";

			} else {
				height_source = "unknown";

			}
			if(height_source){
				ECL_INFO("%llu: EKF aligned, (%s height, IMU buf: %i, OBS buf: %i)",
					(unsigned long long)_imu_sample_delayed.time_us, height_source, (int)_imu_buffer_length, (int)_obs_buffer_length);
			}
		}
	}

	// check for intermittent data (before pop_first_older_than)
	const baroSample &baro_init = _baro_buffer.get_newest();
	_baro_hgt_faulty = !isRecent(baro_init.time_us, 2 * BARO_MAX_INTERVAL);

	const gpsSample &gps_init = _gps_buffer.get_newest();
	_gps_hgt_intermittent = !isRecent(gps_init.time_us, 2 * GPS_MAX_INTERVAL);

	// check for arrival of new sensor data at the fusion time horizon
	_gps_data_ready = _gps_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_gps_sample_delayed);
	_mag_data_ready = _mag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_mag_sample_delayed);

	if (_mag_data_ready) {
		// if enabled, use knowledge of theoretical magnetic field vector to calculate a synthetic magnetomter Z component value.
		// this is useful if there is a lot of interference on the sensor measurement.
		if (_params.synthesize_mag_z && (_params.mag_declination_source & MASK_USE_GEO_DECL) &&_NED_origin_initialised) {
			const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps, _mag_declination_gps)) * Vector3f(_mag_strength_gps, 0, 0);
			_mag_sample_delayed.mag(2) = calculate_synthetic_mag_z_measurement(_mag_sample_delayed.mag, mag_earth_pred);
			_control_status.flags.synthetic_mag_z = true;
		} else {
			_control_status.flags.synthetic_mag_z = false;
		}
	}

	_delta_time_baro_us = _baro_sample_delayed.time_us;
	_baro_data_ready = _baro_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed);

	// if we have a new baro sample save the delta time between this sample and the last sample which is
	// used below for baro offset calculations
	if (_baro_data_ready) {
		_delta_time_baro_us = _baro_sample_delayed.time_us - _delta_time_baro_us;
	}

	// calculate 2,2 element of rotation matrix from sensor frame to earth frame
	// this is required for use of range finder and flow data
	_R_rng_to_earth_2_2 = _R_to_earth(2, 0) * _sin_tilt_rng + _R_to_earth(2, 2) * _cos_tilt_rng;

	// Get range data from buffer and check validity
	_range_data_ready = _range_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_range_sample_delayed);

	updateRangeDataValidity();

	if (_range_data_ready && _rng_hgt_valid) {
		// correct the range data for position offset relative to the IMU
		Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;
		Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
		_range_sample_delayed.rng += pos_offset_earth(2) / _R_rng_to_earth_2_2;
	}

	// We don't fuse flow data immediately because we have to wait for the mid integration point to fall behind the fusion time horizon.
	// This means we stop looking for new data until the old data has been fused, unless we are not fusing optical flow,
	// in this case we need to empty the buffer
	if (!_flow_data_ready || !_control_status.flags.opt_flow) {
		_flow_data_ready = _flow_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_flow_sample_delayed)
				   && (_R_to_earth(2, 2) > _params.range_cos_max_tilt);
	}

	// check if we should fuse flow data for terrain estimation
	if (!_flow_for_terrain_data_ready && _flow_data_ready && _control_status.flags.in_air) {
		// only fuse flow for terrain if range data hasn't been fused for 5 seconds
		_flow_for_terrain_data_ready = isTimedOut(_time_last_hagl_fuse, (uint64_t)5E6);
		// only fuse flow for terrain if the main filter is not fusing flow and we are using gps
		_flow_for_terrain_data_ready &= (!_control_status.flags.opt_flow && _control_status.flags.gps);
	}

	_ev_data_ready = _ext_vision_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_ev_sample_delayed);
	_tas_data_ready = _airspeed_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_airspeed_sample_delayed);

	// check for height sensor timeouts and reset and change sensor if necessary
	controlHeightSensorTimeouts();

	// control use of observations for aiding
	controlMagFusion();
	controlOpticalFlowFusion();
	controlGpsFusion();
	controlAirDataFusion();
	controlBetaFusion();
	controlDragFusion();
	controlHeightFusion();

	// Additional data odoemtery data from an external estimator can be fused.
	controlExternalVisionFusion();

	// Additional horizontal velocity data from an auxiliary sensor can be fused
	controlAuxVelFusion();

	// Fake position measurement for constraining drift when no other velocity or position measurements
	controlFakePosFusion();

	// check if we are no longer fusing measurements that directly constrain velocity drift
	update_deadreckoning_status();
}

void Ekf::controlExternalVisionFusion()
{
	// Check for new external vision data
	if (_ev_data_ready) {

		// if the ev data is not in a NED reference frame, then the transformation between EV and EKF navigation frames
		// needs to be calculated and the observations rotated into the EKF frame of reference
		if ((_params.fusion_mode & MASK_ROTATE_EV) && ((_params.fusion_mode & MASK_USE_EVPOS) || (_params.fusion_mode & MASK_USE_EVVEL)) && !_control_status.flags.ev_yaw) {
			// rotate EV measurements into the EKF Navigation frame
			calcExtVisRotMat();
		}

		// external vision aiding selection logic
		if (_control_status.flags.tilt_align && _control_status.flags.yaw_align) {

			// check for a external vision measurement that has fallen behind the fusion time horizon
			if (isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL)) {
				// turn on use of external vision measurements for position
				if (_params.fusion_mode & MASK_USE_EVPOS && !_control_status.flags.ev_pos) {
					_control_status.flags.ev_pos = true;
					resetPosition();
					ECL_INFO_TIMESTAMPED("commencing external vision position fusion");
				}

				// turn on use of external vision measurements for velocity
				if (_params.fusion_mode & MASK_USE_EVVEL && !_control_status.flags.ev_vel) {
					_control_status.flags.ev_vel = true;
					resetVelocity();
					ECL_INFO_TIMESTAMPED("commencing external vision velocity fusion");
				}
			}
		}

		// external vision yaw aiding selection logic
		if (!_control_status.flags.gps && (_params.fusion_mode & MASK_USE_EVYAW) && !_control_status.flags.ev_yaw && _control_status.flags.tilt_align) {
			// don't start using EV data unless daa is arriving frequently
			if (isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL)) {
				// reset the yaw angle to the value from the observation quaternion
				// get the roll, pitch, yaw estimates from the quaternion states
				Eulerf euler_init(_state.quat_nominal);

				// get initial yaw from the observation quaternion
				const extVisionSample &ev_newest = _ext_vision_buffer.get_newest();
				const Eulerf euler_obs(ev_newest.quat);
				euler_init(2) = euler_obs(2);

				// save a copy of the quaternion state for later use in calculating the amount of reset change
				const Quatf quat_before_reset = _state.quat_nominal;

				// calculate initial quaternion states for the ekf
				_state.quat_nominal = Quatf(euler_init);
				uncorrelateQuatStates();

				// adjust the quaternion covariances estimated yaw error
				increaseQuatYawErrVariance(fmaxf(_ev_sample_delayed.angVar, sq(1.0e-2f)));

				// calculate the amount that the quaternion has changed by
				_state_reset_status.quat_change = _state.quat_nominal * quat_before_reset.inversed();

				// add the reset amount to the output observer buffered data
				for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
					_output_buffer[i].quat_nominal = _state_reset_status.quat_change * _output_buffer[i].quat_nominal;
				}

				// apply the change in attitude quaternion to our newest quaternion estimate
				// which was already taken out from the output buffer
				_output_new.quat_nominal = _state_reset_status.quat_change * _output_new.quat_nominal;

				// capture the reset event
				_state_reset_status.quat_counter++;

				// flag the yaw as aligned
				_control_status.flags.yaw_align = true;

				// turn on fusion of external vision yaw measurements and disable all magnetometer fusion
				_control_status.flags.ev_yaw = true;
				_control_status.flags.mag_dec = false;

				stopMagHdgFusion();
				stopMag3DFusion();

				ECL_INFO_TIMESTAMPED("commencing external vision yaw fusion");
			}
		}



		// determine if we should use the horizontal position observations
		if (_control_status.flags.ev_pos) {

			Vector3f ev_pos_obs_var;
			Vector2f ev_pos_innov_gates;

			// correct position and height for offset relative to IMU
			const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_ev_sample_delayed.pos -= pos_offset_earth;

			// Use an incremental position fusion method for EV position data if GPS is also used
			if (_params.fusion_mode & MASK_USE_GPS) {
				_fuse_hpos_as_odom = true;
			} else {
				_fuse_hpos_as_odom = false;
			}

			if (_fuse_hpos_as_odom) {
				if (!_hpos_prev_available) {
					// no previous observation available to calculate position change
					_fuse_pos = false;
					_hpos_prev_available = true;

				} else {
					// calculate the change in position since the last measurement
					Vector3f ev_delta_pos = _ev_sample_delayed.pos - _pos_meas_prev;

					// rotate measurement into body frame is required when fusing with GPS
					ev_delta_pos = _R_ev_to_ekf * ev_delta_pos;

					// use the change in position since the last measurement
					_ev_pos_innov(0) = _state.pos(0) - _hpos_pred_prev(0) - ev_delta_pos(0);
					_ev_pos_innov(1) = _state.pos(1) - _hpos_pred_prev(1) - ev_delta_pos(1);

					// observation 1-STD error, incremental pos observation is expected to have more uncertainty
					Matrix3f ev_pos_var = matrix::diag(_ev_sample_delayed.posVar);
					ev_pos_var = _R_ev_to_ekf * ev_pos_var * _R_ev_to_ekf.transpose();
					ev_pos_obs_var(0) = fmaxf(ev_pos_var(0, 0), sq(0.5f));
					ev_pos_obs_var(1) = fmaxf(ev_pos_var(1, 1), sq(0.5f));
				}

				// record observation and estimate for use next time
				_pos_meas_prev = _ev_sample_delayed.pos;
				_hpos_pred_prev(0) = _state.pos(0);
				_hpos_pred_prev(1) = _state.pos(1);

			} else {
				// use the absolute position
				Vector3f ev_pos_meas = _ev_sample_delayed.pos;
				Matrix3f ev_pos_var = matrix::diag(_ev_sample_delayed.posVar);
				if (_params.fusion_mode & MASK_ROTATE_EV) {
					ev_pos_meas = _R_ev_to_ekf * ev_pos_meas;
					ev_pos_var = _R_ev_to_ekf * ev_pos_var * _R_ev_to_ekf.transpose();
				}
				_ev_pos_innov(0) = _state.pos(0) - ev_pos_meas(0);
				_ev_pos_innov(1) = _state.pos(1) - ev_pos_meas(1);

				ev_pos_obs_var(0) = fmaxf(ev_pos_var(0, 0), sq(0.01f));
				ev_pos_obs_var(1) = fmaxf(ev_pos_var(1, 0), sq(0.01f));

				// check if we have been deadreckoning too long
				if (isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)) {
					// only reset velocity if we have no another source of aiding constraining it
					if (isTimedOut(_time_last_of_fuse, (uint64_t)1E6) &&
					    isTimedOut(_time_last_hor_vel_fuse, (uint64_t)1E6)) {
						resetVelocity();
					}

					resetPosition();
				}
			}

			// innovation gate size
			ev_pos_innov_gates(0) = fmaxf(_params.ev_pos_innov_gate, 1.0f);

			fuseHorizontalPosition(_ev_pos_innov, ev_pos_innov_gates, ev_pos_obs_var, _ev_pos_innov_var, _ev_pos_test_ratio);
		}

		// determine if we should use the velocity observations
		if (_control_status.flags.ev_vel) {

			Vector3f ev_vel_obs_var;
			Vector2f ev_vel_innov_gates;

			Vector3f vel_aligned{_ev_sample_delayed.vel};
			Matrix3f ev_vel_var = matrix::diag(_ev_sample_delayed.velVar);

			// rotate measurement into correct earth frame if required
			if (_params.fusion_mode & MASK_ROTATE_EV) {
				vel_aligned = _R_ev_to_ekf * _ev_sample_delayed.vel;
				ev_vel_var = _R_ev_to_ekf * ev_vel_var * _R_ev_to_ekf.transpose();
			}

			// correct velocity for offset relative to IMU
			const Vector3f ang_rate = _imu_sample_delayed.delta_ang * (1.0f / _imu_sample_delayed.delta_ang_dt);
			const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
			const Vector3f vel_offset_body = ang_rate % pos_offset_body;
			const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
			vel_aligned -= vel_offset_earth;

			_ev_vel_innov = _state.vel - vel_aligned;

			// check if we have been deadreckoning too long
			if (isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)) {
				// only reset velocity if we have no another source of aiding constraining it
				if (isTimedOut(_time_last_of_fuse, (uint64_t)1E6) &&
				    isTimedOut(_time_last_hor_pos_fuse, (uint64_t)1E6)) {
					resetVelocity();
				}
			}

			ev_vel_obs_var = matrix::max(ev_vel_var.diag(), sq(0.01f));

			ev_vel_innov_gates(0) = ev_vel_innov_gates(1) = fmaxf(_params.ev_vel_innov_gate, 1.0f);

			fuseHorizontalVelocity(_ev_vel_innov, ev_vel_innov_gates,ev_vel_obs_var, _ev_vel_innov_var, _ev_vel_test_ratio);
			fuseVerticalVelocity(_ev_vel_innov, ev_vel_innov_gates, ev_vel_obs_var, _ev_vel_innov_var, _ev_vel_test_ratio);
		}

		// determine if we should use the yaw observation
		if (_control_status.flags.ev_yaw) {
			fuseHeading();
		}

	} else if ((_control_status.flags.ev_pos || _control_status.flags.ev_vel)
		   && isTimedOut(_time_last_ext_vision, (uint64_t)_params.reset_timeout_max)) {

		// Turn off EV fusion mode if no data has been received
		stopEvFusion();
		ECL_INFO_TIMESTAMPED("External Vision Data Stopped");

	}
}

void Ekf::controlOpticalFlowFusion()
{
	// TODO: These motion checks run all the time. Pull them out of this function
	// Check if on ground motion is un-suitable for use of optical flow
	if (!_control_status.flags.in_air) {
		// When on ground check if the vehicle is being shaken or moved in a way that could cause a loss of navigation
		const float accel_norm = _accel_vec_filt.norm();

		const bool motion_is_excessive = ((accel_norm > (CONSTANTS_ONE_G * 1.5f)) // upper g limit
					    || (accel_norm < (CONSTANTS_ONE_G * 0.5f)) // lower g limit
					    || (_ang_rate_magnitude_filt > _flow_max_rate) // angular rate exceeds flow sensor limit
					    || (_R_to_earth(2,2) < cosf(math::radians(30.0f)))); // tilted excessively

		if (motion_is_excessive) {
			_time_bad_motion_us = _imu_sample_delayed.time_us;

		} else {
			_time_good_motion_us = _imu_sample_delayed.time_us;
		}

	} else {
		_time_bad_motion_us = 0;
		_time_good_motion_us = _imu_sample_delayed.time_us;
	}

	// Accumulate autopilot gyro data across the same time interval as the flow sensor
	_imu_del_ang_of += _imu_sample_delayed.delta_ang - _state.delta_ang_bias;
	_delta_time_of += _imu_sample_delayed.delta_ang_dt;

	// New optical flow data is available and is ready to be fused when the midpoint of the sample falls behind the fusion time horizon
	if (_flow_data_ready) {
		// Inhibit flow use if motion is un-suitable or we have good quality GPS
		// Apply hysteresis to prevent rapid mode switching
		float gps_err_norm_lim;
		if (_control_status.flags.opt_flow) {
			gps_err_norm_lim = 0.7f;
		} else {
			gps_err_norm_lim = 1.0f;
		}

		// Check if we are in-air and require optical flow to control position drift
		bool flow_required = _control_status.flags.in_air &&
				(_is_dead_reckoning // is doing inertial dead-reckoning so must constrain drift urgently
				  || (isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow))
				  || (_control_status.flags.gps && (_gps_error_norm > gps_err_norm_lim))); // is using GPS, but GPS is bad

		if (!_inhibit_flow_use && _control_status.flags.opt_flow) {
			// inhibit use of optical flow if motion is unsuitable and we are not reliant on it for flight navigation
			bool preflight_motion_not_ok = !_control_status.flags.in_air && ((_imu_sample_delayed.time_us - _time_good_motion_us) > (uint64_t)1E5);
			bool flight_motion_not_ok = _control_status.flags.in_air && !isTerrainEstimateValid();
			if ((preflight_motion_not_ok || flight_motion_not_ok) && !flow_required) {
				_inhibit_flow_use = true;
			}
		} else if (_inhibit_flow_use && !_control_status.flags.opt_flow){
			// allow use of optical flow if motion is suitable or we are reliant on it for flight navigation
			bool preflight_motion_ok = !_control_status.flags.in_air && ((_imu_sample_delayed.time_us - _time_bad_motion_us) > (uint64_t)5E6);
			bool flight_motion_ok = _control_status.flags.in_air && isRangeAidSuitable();
			if (preflight_motion_ok || flight_motion_ok || flow_required) {
				_inhibit_flow_use = false;
			}
		}

		// Handle cases where we are using optical flow but are no longer able to because data is old
		// or its use has been inhibited.
		if (_control_status.flags.opt_flow) {
			if (_inhibit_flow_use) {
				stopFlowFusion();
				_time_last_of_fuse = 0;

			} else if (isTimedOut(_time_last_of_fuse, (uint64_t)_params.reset_timeout_max)) {
				stopFlowFusion();

			}
		}

		// optical flow fusion mode selection logic
		if ((_params.fusion_mode & MASK_USE_OF) // optical flow has been selected by the user
			&& !_control_status.flags.opt_flow // we are not yet using flow data
			&& _control_status.flags.tilt_align // we know our tilt attitude
			&& !_inhibit_flow_use
			&& isTerrainEstimateValid())
		{
			// If the heading is not aligned, reset the yaw and magnetic field states
			if (!_control_status.flags.yaw_align) {
				_control_status.flags.yaw_align = resetMagHeading(_mag_lpf.getState());
			}

			// If the heading is valid and use is not inhibited , start using optical flow aiding
			if (_control_status.flags.yaw_align) {
				// set the flag and reset the fusion timeout
				_control_status.flags.opt_flow = true;
				_time_last_of_fuse = _time_last_imu;

				// if we are not using GPS or external vision aiding, then the velocity and position states and covariances need to be set
				const bool flow_aid_only = !isOtherSourceOfHorizontalAidingThan(_control_status.flags.opt_flow);
				if (flow_aid_only) {
					resetVelocity();
					resetPosition();

					// align the output observer to the EKF states
					alignOutputFilter();
				}
			}

		} else if (!(_params.fusion_mode & MASK_USE_OF)) {
			_control_status.flags.opt_flow = false;
		}

		// handle the case when we have optical flow, are reliant on it, but have not been using it for an extended period
		if (isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow)) {

			bool do_reset = isTimedOut(_time_last_of_fuse, _params.reset_timeout_max);

			if (do_reset) {
				resetVelocity();
				resetPosition();
			}
		}

		// Only fuse optical flow if valid body rate compensation data is available
		if (calcOptFlowBodyRateComp()) {

			bool flow_quality_good = (_flow_sample_delayed.quality >= _params.flow_qual_min);

			if (!flow_quality_good && !_control_status.flags.in_air) {
				// when on the ground with poor flow quality, assume zero ground relative velocity and LOS rate
				_flow_compensated_XY_rad.zero();
			} else {
				// compensate for body motion to give a LOS rate
				_flow_compensated_XY_rad(0) = _flow_sample_delayed.flow_xy_rad(0) - _flow_sample_delayed.gyro_xyz(0);
				_flow_compensated_XY_rad(1) = _flow_sample_delayed.flow_xy_rad(1) - _flow_sample_delayed.gyro_xyz(1);
			}
		} else {
			// don't use this flow data and wait for the next data to arrive
			_flow_data_ready = false;
		}
	}

	// Wait until the midpoint of the flow sample has fallen behind the fusion time horizon
	if (_flow_data_ready && (_imu_sample_delayed.time_us > _flow_sample_delayed.time_us - uint32_t(1e6f * _flow_sample_delayed.dt) / 2)) {
		// Fuse optical flow LOS rate observations into the main filter only if height above ground has been updated recently
		// but use a relaxed time criteria to enable it to coast through bad range finder data
		if (_control_status.flags.opt_flow && isRecent(_time_last_hagl_fuse, (uint64_t)10e6)) {
			fuseOptFlow();
			_last_known_posNE(0) = _state.pos(0);
			_last_known_posNE(1) = _state.pos(1);
		}

		_flow_data_ready = false;
	}
}

void Ekf::controlGpsFusion()
{
	// Check for new GPS data that has fallen behind the fusion time horizon
	if (_gps_data_ready) {

		// GPS yaw aiding selection logic
		if ((_params.fusion_mode & MASK_USE_GPSYAW)
				&& ISFINITE(_gps_sample_delayed.yaw)
				&& _control_status.flags.tilt_align
				&& (!_control_status.flags.gps_yaw || !_control_status.flags.yaw_align)
				&& isRecent(_time_last_gps, 2 * GPS_MAX_INTERVAL)) {

			if (resetGpsAntYaw()) {
				// flag the yaw as aligned
				_control_status.flags.yaw_align = true;

				// turn on fusion of external vision yaw measurements and disable all other yaw fusion
				_control_status.flags.gps_yaw = true;
				_control_status.flags.ev_yaw = false;
				_control_status.flags.mag_dec = false;

				stopMagHdgFusion();
				stopMag3DFusion();

				ECL_INFO_TIMESTAMPED("commencing GPS yaw fusion");
			}
		}

		// fuse the yaw observation
		if (_control_status.flags.gps_yaw) {
			fuseGpsAntYaw();
		}

		// Determine if we should use GPS aiding for velocity and horizontal position
		// To start using GPS we need angular alignment completed, the local NED origin set and GPS data that has not failed checks recently
		bool gps_checks_passing = isTimedOut(_last_gps_fail_us, (uint64_t)5e6);
		bool gps_checks_failing = isTimedOut(_last_gps_pass_us, (uint64_t)5e6);
		if ((_params.fusion_mode & MASK_USE_GPS) && !_control_status.flags.gps) {
			if (_control_status.flags.tilt_align && _NED_origin_initialised && gps_checks_passing) {
				// If the heading is not aligned, reset the yaw and magnetic field states
				// Do not use external vision for yaw if using GPS because yaw needs to be
				// defined relative to an NED reference frame
				const bool want_to_reset_mag_heading = !_control_status.flags.yaw_align ||
								       _control_status.flags.ev_yaw ||
								       _mag_inhibit_yaw_reset_req;
				if (want_to_reset_mag_heading && canResetMagHeading()) {
					_control_status.flags.ev_yaw = false;
					_control_status.flags.yaw_align = resetMagHeading(_mag_lpf.getState());
					// Handle the special case where we have not been constraining yaw drift or learning yaw bias due
					// to assumed invalid mag field associated with indoor operation with a downwards looking flow sensor.
					if (_mag_inhibit_yaw_reset_req) {
						_mag_inhibit_yaw_reset_req = false;
						// Zero the yaw bias covariance and set the variance to the initial alignment uncertainty
						P.uncorrelateCovarianceSetVariance<1>(12, sq(_params.switch_on_gyro_bias * FILTER_UPDATE_PERIOD_S));
					}
				}

				// If the heading is valid start using gps aiding
				if (_control_status.flags.yaw_align) {
					// if we are not already aiding with optical flow, then we need to reset the position and velocity
					// otherwise we only need to reset the position
					_control_status.flags.gps = true;

					if (!_control_status.flags.opt_flow) {
						if (!resetPosition() || !resetVelocity()) {
							_control_status.flags.gps = false;

						}

					} else if (!resetPosition()) {
						_control_status.flags.gps = false;

					}

					if (_control_status.flags.gps) {
						ECL_INFO_TIMESTAMPED("commencing GPS fusion");
						_time_last_gps = _time_last_imu;
					}
				}
			}

		}  else if (!(_params.fusion_mode & MASK_USE_GPS)) {
			_control_status.flags.gps = false;

		}

		// Handle the case where we are using GPS and another source of aiding and GPS is failing checks
		if (_control_status.flags.gps && gps_checks_failing && isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
			stopGpsFusion();
			// Reset position state to external vision if we are going to use absolute values
			if (_control_status.flags.ev_pos && !(_params.fusion_mode & MASK_ROTATE_EV)) {
				resetPosition();
			}
			ECL_WARN_TIMESTAMPED("GPS data quality poor - stopping use");
		}

		// handle the case when we now have GPS, but have not been using it for an extended period
		if (_control_status.flags.gps) {
			// We are relying on aiding to constrain drift so after a specified time
			// with no aiding we need to do something
			bool do_reset = isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
					&& isTimedOut(_time_last_delpos_fuse, _params.reset_timeout_max)
					&& isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
					&& isTimedOut(_time_last_of_fuse, _params.reset_timeout_max);

			// We haven't had an absolute position fix for a longer time so need to do something
			do_reset = do_reset || isTimedOut(_time_last_hor_pos_fuse, 2 * _params.reset_timeout_max);

			if (do_reset) {
				// use GPS velocity data to check and correct yaw angle if a FW vehicle
				if (_control_status.flags.fixed_wing && _control_status.flags.in_air) {
					// if flying a fixed wing aircraft, do a complete reset that includes yaw
					_control_status.flags.mag_aligned_in_flight = realignYawGPS();
				}

				resetVelocity();
				resetPosition();
				_velpos_reset_request = false;
				ECL_WARN_TIMESTAMPED("GPS fusion timeout - reset to GPS");

				// Reset the timeout counters
				_time_last_hor_pos_fuse = _time_last_imu;
				_time_last_hor_vel_fuse = _time_last_imu;

			}
		}

		// Only use GPS data for position and velocity aiding if enabled
		if (_control_status.flags.gps) {


			Vector2f gps_vel_innov_gates; // [horizontal vertical]
			Vector2f gps_pos_innov_gates; // [horizontal vertical]
			Vector3f gps_vel_obs_var;
			Vector3f gps_pos_obs_var;

			// correct velocity for offset relative to IMU
			const Vector3f ang_rate = _imu_sample_delayed.delta_ang * (1.0f / _imu_sample_delayed.delta_ang_dt);
			const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
			const Vector3f vel_offset_body = ang_rate % pos_offset_body;
			const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
			_gps_sample_delayed.vel -= vel_offset_earth;

			// correct position and height for offset relative to IMU
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_gps_sample_delayed.pos(0) -= pos_offset_earth(0);
			_gps_sample_delayed.pos(1) -= pos_offset_earth(1);
			_gps_sample_delayed.hgt += pos_offset_earth(2);

			const float lower_limit = fmaxf(_params.gps_pos_noise, 0.01f);

			if (isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
				// if we are using other sources of aiding, then relax the upper observation
				// noise limit which prevents bad GPS perturbing the position estimate
				gps_pos_obs_var(0) = gps_pos_obs_var(1) = sq(fmaxf(_gps_sample_delayed.hacc, lower_limit));

			} else {
				// if we are not using another source of aiding, then we are reliant on the GPS
				// observations to constrain attitude errors and must limit the observation noise value.
				float upper_limit = fmaxf(_params.pos_noaid_noise, lower_limit);
				gps_pos_obs_var(0) = gps_pos_obs_var(1) = sq(math::constrain(_gps_sample_delayed.hacc, lower_limit, upper_limit));
			}

			gps_vel_obs_var(0) = gps_vel_obs_var(1) = gps_vel_obs_var(2) = sq(fmaxf(_gps_sample_delayed.sacc, _params.gps_vel_noise));
			gps_vel_obs_var(2) = sq(1.5f) * gps_vel_obs_var(2);

			// calculate innovations
			_gps_vel_innov(0) = _state.vel(0) - _gps_sample_delayed.vel(0);
			_gps_vel_innov(1) = _state.vel(1) - _gps_sample_delayed.vel(1);
			_gps_vel_innov(2) = _state.vel(2) - _gps_sample_delayed.vel(2);
			_gps_pos_innov(0) = _state.pos(0) - _gps_sample_delayed.pos(0);
			_gps_pos_innov(1) = _state.pos(1) - _gps_sample_delayed.pos(1);

			// set innovation gate size
			gps_pos_innov_gates(0) = fmaxf(_params.gps_pos_innov_gate, 1.0f);
			gps_vel_innov_gates(0) = gps_vel_innov_gates(1) = fmaxf(_params.gps_vel_innov_gate, 1.0f);

			// fuse GPS measurement
			fuseHorizontalVelocity(_gps_vel_innov, gps_vel_innov_gates,gps_vel_obs_var, _gps_vel_innov_var, _gps_vel_test_ratio);
			fuseVerticalVelocity(_gps_vel_innov, gps_vel_innov_gates, gps_vel_obs_var, _gps_vel_innov_var, _gps_vel_test_ratio);
			fuseHorizontalPosition(_gps_pos_innov, gps_pos_innov_gates, gps_pos_obs_var, _gps_pos_innov_var, _gps_pos_test_ratio);
		}

	} else if (_control_status.flags.gps && (_imu_sample_delayed.time_us - _gps_sample_delayed.time_us > (uint64_t)10e6)) {
		stopGpsFusion();
		ECL_WARN_TIMESTAMPED("GPS data stopped");
	}  else if (_control_status.flags.gps && (_imu_sample_delayed.time_us - _gps_sample_delayed.time_us > (uint64_t)1e6) && isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
		// Handle the case where we are fusing another position source along GPS,
		// stop waiting for GPS after 1 s of lost signal
		stopGpsFusion();
		ECL_WARN_TIMESTAMPED("GPS data stopped, using only EV or OF");
	}
}

void Ekf::controlHeightSensorTimeouts()
{
	/*
	 * Handle the case where we have not fused height measurements recently and
	 * uncertainty exceeds the max allowable. Reset using the best available height
	 * measurement source, continue using it after the reset and declare the current
	 * source failed if we have switched.
	*/

	// Check for IMU accelerometer vibration induced clipping as evidenced by the vertical innovations being positive and not stale.
	// Clipping causes the average accel reading to move towards zero which makes the INS think it is falling and produces positive vertical innovations
	float var_product_lim = sq(_params.vert_innov_test_lim) * sq(_params.vert_innov_test_lim);
	bool bad_vert_accel = (_control_status.flags.baro_hgt && // we can only run this check if vertical position and velocity observations are independent
			(sq(_gps_pos_innov(2) * fmaxf(fabsf(_gps_vel_innov(2)),fabsf(_ev_vel_innov(2)))) > var_product_lim * (_gps_pos_innov_var(2) * fmaxf(fabsf(_gps_vel_innov_var(2)),fabsf(_ev_vel_innov_var(2))))) && // vertical position and velocity sensors are in agreement that we have a significant error
			(_gps_vel_innov(2) > 0.0f || _ev_vel_innov(2) > 0.0f) && // positive innovation indicates that the inertial nav thinks it is falling
			((_imu_sample_delayed.time_us - _baro_sample_delayed.time_us) < 2 * BARO_MAX_INTERVAL) && // vertical position data is fresh
			((_imu_sample_delayed.time_us - _gps_sample_delayed.time_us) < 2 * GPS_MAX_INTERVAL)); // vertical velocity data is fresh

	// record time of last bad vert accel
	if (bad_vert_accel) {
		_time_bad_vert_accel =  _time_last_imu;

	} else {
		_time_good_vert_accel = _time_last_imu;
	}

	// declare a bad vertical acceleration measurement and make the declaration persist
	// for a minimum of 10 seconds
	if (_bad_vert_accel_detected) {
		_bad_vert_accel_detected = isRecent(_time_bad_vert_accel, BADACC_PROBATION);

	} else {
		_bad_vert_accel_detected = bad_vert_accel;
	}

	// check if height is continuously failing because of accel errors
	bool continuous_bad_accel_hgt = isTimedOut(_time_good_vert_accel, (uint64_t)_params.bad_acc_reset_delay_us);

	// check if height has been inertial deadreckoning for too long
	bool hgt_fusion_timeout = isTimedOut(_time_last_hgt_fuse, (uint64_t)5e6);

	if (hgt_fusion_timeout || continuous_bad_accel_hgt) {

		bool request_height_reset = false;

		// handle the case where we are using baro for height
		if (_control_status.flags.baro_hgt) {
			// check if GPS height is available
			const gpsSample &gps_init = _gps_buffer.get_newest();
			const bool gps_hgt_accurate = (gps_init.vacc < _params.req_vacc);

			const baroSample &baro_init = _baro_buffer.get_newest();
			const bool baro_data_available = isRecent(baro_init.time_us, 2 * BARO_MAX_INTERVAL);

			// check for inertial sensing errors in the last 10 seconds
			const bool prev_bad_vert_accel = isRecent(_time_bad_vert_accel, BADACC_PROBATION);

			// reset to GPS if adequate GPS data is available and the timeout cannot be blamed on IMU data
			const bool reset_to_gps = !_gps_hgt_intermittent &&
					    ((gps_hgt_accurate && !prev_bad_vert_accel) || !baro_data_available);

			if (reset_to_gps) {
				// set height sensor health
				_baro_hgt_faulty = true;

				setControlGPSHeight();

				request_height_reset = true;
				ECL_WARN_TIMESTAMPED("baro hgt timeout - reset to GPS");

			} else if (baro_data_available) {
				// set height sensor health
				_baro_hgt_faulty = false;

				setControlBaroHeight();

				request_height_reset = true;
				ECL_WARN_TIMESTAMPED("baro hgt timeout - reset to baro");

			}
		}

		// handle the case we are using GPS for height
		if (_control_status.flags.gps_hgt) {
			// check if GPS height is available
			const gpsSample &gps_init = _gps_buffer.get_newest();
			const bool gps_hgt_accurate = (gps_init.vacc < _params.req_vacc);

			// check the baro height source for consistency and freshness
			const baroSample &baro_init = _baro_buffer.get_newest();
			const bool baro_data_fresh = isRecent(baro_init.time_us, 2 * BARO_MAX_INTERVAL);
			const float baro_innov = _state.pos(2) - (_hgt_sensor_offset - baro_init.hgt + _baro_hgt_offset);
			const bool baro_data_consistent = fabsf(baro_innov) < (sq(_params.baro_noise) + P(9,9)) * sq(_params.baro_innov_gate);

			// if baro data is acceptable and GPS data is inaccurate, reset height to baro
			const bool reset_to_baro = baro_data_fresh &&
						   ((baro_data_consistent && !_baro_hgt_faulty && !gps_hgt_accurate) ||
						     _gps_hgt_intermittent);

			if (reset_to_baro) {
				// set height sensor health
				_baro_hgt_faulty = false;

				setControlBaroHeight();

				request_height_reset = true;
				ECL_WARN_TIMESTAMPED("gps hgt timeout - reset to baro");

			} else if (!_gps_hgt_intermittent) {
				setControlGPSHeight();

				request_height_reset = true;
				ECL_WARN_TIMESTAMPED("gps hgt timeout - reset to GPS");

			}
		}

		// handle the case we are using range finder for height
		if (_control_status.flags.rng_hgt) {

			// check if baro data is available
			const baroSample &baro_init = _baro_buffer.get_newest();
			const bool baro_data_available = isRecent(baro_init.time_us, 2 * BARO_MAX_INTERVAL);

			if (_rng_hgt_valid) {

				setControlRangeHeight();

				request_height_reset = true;
				ECL_WARN_TIMESTAMPED("rng hgt timeout - reset to rng hgt");

			} else if (baro_data_available) {
				// set height sensor health
				_baro_hgt_faulty = false;

				setControlBaroHeight();

				request_height_reset = true;
				ECL_WARN_TIMESTAMPED("rng hgt timeout - reset to baro");

			}
		}

		// handle the case where we are using external vision data for height
		if (_control_status.flags.ev_hgt) {
			// check if vision data is available
			const extVisionSample &ev_init = _ext_vision_buffer.get_newest();
			const bool ev_data_available = isRecent(ev_init.time_us, 2 * EV_MAX_INTERVAL);

			// check if baro data is available
			const baroSample &baro_init = _baro_buffer.get_newest();
			const bool baro_data_available = isRecent(baro_init.time_us, 2 * BARO_MAX_INTERVAL);

			if (ev_data_available) {
				setControlEVHeight();

				request_height_reset = true;
				ECL_WARN_TIMESTAMPED("ev hgt timeout - reset to ev hgt");

			} else if (baro_data_available) {
				// set height sensor health
				_baro_hgt_faulty = false;

				setControlBaroHeight();

				request_height_reset = true;
				ECL_WARN_TIMESTAMPED("ev hgt timeout - reset to baro");

			}
		}

		// Reset vertical position and velocity states to the last measurement
		if (request_height_reset) {
			resetHeight();
			// Reset the timout timer
			_time_last_hgt_fuse = _time_last_imu;

		}

	}
}

void Ekf::controlHeightFusion()
{

	checkRangeAidSuitability();
	_range_aid_mode_selected = (_params.range_aid == 1) && isRangeAidSuitable();

	if (_params.vdist_sensor_type == VDIST_SENSOR_BARO) {

		if (_range_aid_mode_selected && _range_data_ready && _rng_hgt_valid) {
			setControlRangeHeight();
			_fuse_height = true;

			// we have just switched to using range finder, calculate height sensor offset such that current
			// measurement matches our current height estimate
			if (_control_status_prev.flags.rng_hgt != _control_status.flags.rng_hgt) {
				if (isTerrainEstimateValid()) {
					_hgt_sensor_offset = _terrain_vpos;

				} else {
					_hgt_sensor_offset = _R_rng_to_earth_2_2 * _range_sample_delayed.rng + _state.pos(2);
				}
			}

		} else if (!_range_aid_mode_selected && _baro_data_ready && !_baro_hgt_faulty) {
			setControlBaroHeight();
			_fuse_height = true;

			// we have just switched to using baro height, we don't need to set a height sensor offset
			// since we track a separate _baro_hgt_offset
			if (_control_status_prev.flags.baro_hgt != _control_status.flags.baro_hgt) {
				_hgt_sensor_offset = 0.0f;
			}

			// Turn off ground effect compensation if it times out
			if (_control_status.flags.gnd_effect) {
				if (isTimedOut(_time_last_gnd_effect_on, GNDEFFECT_TIMEOUT)) {

					_control_status.flags.gnd_effect = false;
				}
			}

		} else if (_control_status.flags.gps_hgt && _gps_data_ready && !_gps_hgt_intermittent) {
			// switch to gps if there was a reset to gps
			_fuse_height = true;

			// we have just switched to using gps height, calculate height sensor offset such that current
			// measurement matches our current height estimate
			if (_control_status_prev.flags.gps_hgt != _control_status.flags.gps_hgt) {
				_hgt_sensor_offset = _gps_sample_delayed.hgt - _gps_alt_ref + _state.pos(2);
			}
		}
	}

	// set the height data source to range if requested
	if ((_params.vdist_sensor_type == VDIST_SENSOR_RANGE) && _rng_hgt_valid) {
		setControlRangeHeight();
		_fuse_height = _range_data_ready;

		// we have just switched to using range finder, calculate height sensor offset such that current
		// measurement matches our current height estimate
		if (_control_status_prev.flags.rng_hgt != _control_status.flags.rng_hgt) {
			// use the parameter rng_gnd_clearance if on ground to avoid a noisy offset initialization (e.g. sonar)
			if (_control_status.flags.in_air && isTerrainEstimateValid()) {

				_hgt_sensor_offset = _terrain_vpos;

			} else if (_control_status.flags.in_air) {

				_hgt_sensor_offset = _R_rng_to_earth_2_2 * _range_sample_delayed.rng + _state.pos(2);

			} else {

				_hgt_sensor_offset = _params.rng_gnd_clearance;
			}
		}

	} else if ((_params.vdist_sensor_type == VDIST_SENSOR_RANGE) && _baro_data_ready && !_baro_hgt_faulty) {
		setControlBaroHeight();
		_fuse_height = true;

		// we have just switched to using baro height, we don't need to set a height sensor offset
		// since we track a separate _baro_hgt_offset
		if (_control_status_prev.flags.baro_hgt != _control_status.flags.baro_hgt) {
			_hgt_sensor_offset = 0.0f;
		}
	}

	// Determine if GPS should be used as the height source
	if (_params.vdist_sensor_type == VDIST_SENSOR_GPS) {

		if (_range_aid_mode_selected && _range_data_ready && _rng_hgt_valid) {
			setControlRangeHeight();
			_fuse_height = true;

			// we have just switched to using range finder, calculate height sensor offset such that current
			// measurement matches our current height estimate
			if (_control_status_prev.flags.rng_hgt != _control_status.flags.rng_hgt) {
				if (isTerrainEstimateValid()) {
					_hgt_sensor_offset = _terrain_vpos;

				} else {
					_hgt_sensor_offset = _R_rng_to_earth_2_2 * _range_sample_delayed.rng + _state.pos(2);
				}
			}

		} else if (!_range_aid_mode_selected && _gps_data_ready && !_gps_hgt_intermittent && _gps_checks_passed) {
			setControlGPSHeight();
			_fuse_height = true;

			// we have just switched to using gps height, calculate height sensor offset such that current
			// measurement matches our current height estimate
			if (_control_status_prev.flags.gps_hgt != _control_status.flags.gps_hgt) {
				_hgt_sensor_offset = _gps_sample_delayed.hgt - _gps_alt_ref + _state.pos(2);
			}

		} else if (_control_status.flags.baro_hgt && _baro_data_ready && !_baro_hgt_faulty) {
			// switch to baro if there was a reset to baro
			_fuse_height = true;

			// we have just switched to using baro height, we don't need to set a height sensor offset
			// since we track a separate _baro_hgt_offset
			if (_control_status_prev.flags.baro_hgt != _control_status.flags.baro_hgt) {
				_hgt_sensor_offset = 0.0f;
			}
		}
	}

	// Determine if we rely on EV height but switched to baro
	if (_params.vdist_sensor_type == VDIST_SENSOR_EV) {

		// don't start using EV data unless data is arriving frequently
		if (!_control_status.flags.ev_hgt && isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL)) {
			_fuse_height = true;
			setControlEVHeight();
			resetHeight();
		}

		if (_control_status.flags.baro_hgt && _baro_data_ready && !_baro_hgt_faulty) {
			// switch to baro if there was a reset to baro
			_fuse_height = true;

			// we have just switched to using baro height, we don't need to set a height sensor offset
			// since we track a separate _baro_hgt_offset
			if (_control_status_prev.flags.baro_hgt != _control_status.flags.baro_hgt) {
				_hgt_sensor_offset = 0.0f;
			}
		}

		// determine if we should use the vertical position observation
		if (_control_status.flags.ev_hgt) {
			_fuse_height = true;
		}
	}

	// calculate a filtered offset between the baro origin and local NED origin if we are not using the baro as a height reference
	if (!_control_status.flags.baro_hgt && _baro_data_ready) {
		float local_time_step = 1e-6f * _delta_time_baro_us;
		local_time_step = math::constrain(local_time_step, 0.0f, 1.0f);

		// apply a 10 second first order low pass filter to baro offset
		float offset_rate_correction =  0.1f * (_baro_sample_delayed.hgt + _state.pos(
				2) - _baro_hgt_offset);
		_baro_hgt_offset += local_time_step * math::constrain(offset_rate_correction, -0.1f, 0.1f);
	}

	if (isTimedOut(_time_last_hgt_fuse, 2 * RNG_MAX_INTERVAL) && _control_status.flags.rng_hgt
	    && (!_range_data_ready || !_rng_hgt_valid)) {

		// If we are supposed to be using range finder data as the primary height sensor, have missed or rejected measurements
		// and are on the ground, then synthesise a measurement at the expected on ground value
		if (!_control_status.flags.in_air) {
			_range_sample_delayed.rng = _params.rng_gnd_clearance;
			_range_sample_delayed.time_us = _imu_sample_delayed.time_us;

		}

		_fuse_height = true;
	}

	if (_fuse_height) {

		if (_control_status.flags.baro_hgt) {
			Vector2f baro_hgt_innov_gate;
			Vector3f baro_hgt_obs_var;

			// vertical position innovation - baro measurement has opposite sign to earth z axis
			_baro_hgt_innov(2) = _state.pos(2) + _baro_sample_delayed.hgt - _baro_hgt_offset - _hgt_sensor_offset;
			// observation variance - user parameter defined
			baro_hgt_obs_var(2) = sq(fmaxf(_params.baro_noise, 0.01f));
			// innovation gate size
			baro_hgt_innov_gate(1) = fmaxf(_params.baro_innov_gate, 1.0f);

			// Compensate for positive static pressure transients (negative vertical position innovations)
			// caused by rotor wash ground interaction by applying a temporary deadzone to baro innovations.
			float deadzone_start = 0.0f;
			float deadzone_end = deadzone_start + _params.gnd_effect_deadzone;

			if (_control_status.flags.gnd_effect) {
				if (_baro_hgt_innov(2) < -deadzone_start) {
					if (_baro_hgt_innov(2) <= -deadzone_end) {
						_baro_hgt_innov(2) += deadzone_end;

					} else {
						_baro_hgt_innov(2) = -deadzone_start;
					}
				}
			}
			// fuse height information
			fuseVerticalPosition(_baro_hgt_innov,baro_hgt_innov_gate,
				baro_hgt_obs_var, _baro_hgt_innov_var,_baro_hgt_test_ratio);

		} else if (_control_status.flags.gps_hgt) {
			Vector2f gps_hgt_innov_gate;
			Vector3f gps_hgt_obs_var;
			// vertical position innovation - gps measurement has opposite sign to earth z axis
			_gps_pos_innov(2) = _state.pos(2) + _gps_sample_delayed.hgt - _gps_alt_ref - _hgt_sensor_offset;
			// observation variance - receiver defined and parameter limited
			// use scaled horizontal position accuracy assuming typical ratio of VDOP/HDOP
			const float lower_limit = fmaxf(_params.gps_pos_noise, 0.01f);
			const float upper_limit = fmaxf(_params.pos_noaid_noise, lower_limit);
			gps_hgt_obs_var(2) = sq(1.5f * math::constrain(_gps_sample_delayed.vacc, lower_limit, upper_limit));
			// innovation gate size
			gps_hgt_innov_gate(1) = fmaxf(_params.baro_innov_gate, 1.0f);
			// fuse height information
			fuseVerticalPosition(_gps_pos_innov,gps_hgt_innov_gate,
				gps_hgt_obs_var, _gps_pos_innov_var,_gps_pos_test_ratio);

		} else if (_control_status.flags.rng_hgt && (_R_rng_to_earth_2_2 > _params.range_cos_max_tilt)) {
			// TODO: Tilt check does not belong here, should not set fuse height to true if tilted
			Vector2f rng_hgt_innov_gate;
			Vector3f rng_hgt_obs_var;
			// use range finder with tilt correction
			_rng_hgt_innov(2) = _state.pos(2) - (-math::max(_range_sample_delayed.rng * _R_rng_to_earth_2_2,
							 _params.rng_gnd_clearance)) - _hgt_sensor_offset;
			// observation variance - user parameter defined
			rng_hgt_obs_var(2) = fmaxf((sq(_params.range_noise) + sq(_params.range_noise_scaler * _range_sample_delayed.rng)) * sq(_R_rng_to_earth_2_2), 0.01f);
			// innovation gate size
			rng_hgt_innov_gate(1) = fmaxf(_params.range_innov_gate, 1.0f);
			// fuse height information
			fuseVerticalPosition(_rng_hgt_innov,rng_hgt_innov_gate,
				rng_hgt_obs_var, _rng_hgt_innov_var,_rng_hgt_test_ratio);

		} else if (_control_status.flags.ev_hgt) {
			Vector2f ev_hgt_innov_gate;
			Vector3f ev_hgt_obs_var;
			// calculate the innovation assuming the external vision observation is in local NED frame
			_ev_pos_innov(2) = _state.pos(2) - _ev_sample_delayed.pos(2);
			// observation variance - defined externally
			ev_hgt_obs_var(2) = fmaxf(_ev_sample_delayed.posVar(2), sq(0.01f));
			// innovation gate size
			ev_hgt_innov_gate(1) = fmaxf(_params.ev_pos_innov_gate, 1.0f);
			// fuse height information
			fuseVerticalPosition(_ev_pos_innov,ev_hgt_innov_gate,
				ev_hgt_obs_var, _ev_pos_innov_var,_ev_pos_test_ratio);
		}

	}

}

void Ekf::checkRangeAidSuitability()
{
	if (_control_status.flags.in_air
	    && _rng_hgt_valid
	    && isTerrainEstimateValid()
	    && isHorizontalAidingActive()) {
		// check if we can use range finder measurements to estimate height, use hysteresis to avoid rapid switching
		// Note that the 0.7 coefficients and the innovation check are arbitrary values but work well in practice
		const bool is_in_range = _is_range_aid_suitable
					 ? (_terrain_vpos - _state.pos(2) < _params.max_hagl_for_range_aid)
					 : (_terrain_vpos - _state.pos(2) < _params.max_hagl_for_range_aid * 0.7f);

		const float ground_vel = sqrtf(_state.vel(0) * _state.vel(0) + _state.vel(1) * _state.vel(1));
		const bool is_below_max_speed = _is_range_aid_suitable
						? ground_vel < _params.max_vel_for_range_aid
						: ground_vel < _params.max_vel_for_range_aid * 0.7f;

		const bool is_hagl_stable = _is_range_aid_suitable
					    ? ((_hagl_innov * _hagl_innov / (sq(_params.range_aid_innov_gate) * _hagl_innov_var)) < 1.0f)
					    : ((_hagl_innov * _hagl_innov / (sq(_params.range_aid_innov_gate) * _hagl_innov_var)) < 0.01f);

		_is_range_aid_suitable = is_in_range && is_below_max_speed && is_hagl_stable;

	} else {
		_is_range_aid_suitable = false;
	}
}

void Ekf::controlAirDataFusion()
{
	// control activation and initialisation/reset of wind states required for airspeed fusion

	// If both airspeed and sideslip fusion have timed out and we are not using a drag observation model then we no longer have valid wind estimates
	const bool airspeed_timed_out = isTimedOut(_time_last_arsp_fuse, (uint64_t)10e6);
	const bool sideslip_timed_out = isTimedOut(_time_last_beta_fuse, (uint64_t)10e6);

	if (_control_status.flags.wind && airspeed_timed_out && sideslip_timed_out && !(_params.fusion_mode & MASK_USE_DRAG)) {
		_control_status.flags.wind = false;

	}

	if (_control_status.flags.fuse_aspd && airspeed_timed_out) {
		_control_status.flags.fuse_aspd = false;

	}

	// Always try to fuse airspeed data if available and we are in flight
	if (_tas_data_ready && _control_status.flags.in_air) {
		// always fuse airsped data if we are flying and data is present
		if (!_control_status.flags.fuse_aspd) {
			_control_status.flags.fuse_aspd = true;
		}

		// If starting wind state estimation, reset the wind states and covariances before fusing any data
		if (!_control_status.flags.wind) {
			// activate the wind states
			_control_status.flags.wind = true;
			// reset the timout timer to prevent repeated resets
			_time_last_arsp_fuse = _time_last_imu;
			_time_last_beta_fuse = _time_last_imu;
			// reset the wind speed states and corresponding covariances
			resetWindStates();
			resetWindCovariance();

		}

		fuseAirspeed();

	}
}

void Ekf::controlBetaFusion()
{
	// control activation and initialisation/reset of wind states required for synthetic sideslip fusion fusion

	// If both airspeed and sideslip fusion have timed out and we are not using a drag observation model then we no longer have valid wind estimates
	const bool sideslip_timed_out = isTimedOut(_time_last_beta_fuse, (uint64_t)10e6);
	const bool airspeed_timed_out = isTimedOut(_time_last_arsp_fuse, (uint64_t)10e6);

	if (_control_status.flags.wind && airspeed_timed_out && sideslip_timed_out && !(_params.fusion_mode & MASK_USE_DRAG)) {
		_control_status.flags.wind = false;
	}

	// Perform synthetic sideslip fusion when in-air and sideslip fuson had been enabled externally in addition to the following criteria:

	// Sufficient time has lapsed sice the last fusion
	bool beta_fusion_time_triggered = isTimedOut(_time_last_beta_fuse, _params.beta_avg_ft_us);

	if (beta_fusion_time_triggered && _control_status.flags.fuse_beta && _control_status.flags.in_air) {
		// If starting wind state estimation, reset the wind states and covariances before fusing any data
		if (!_control_status.flags.wind) {
			// activate the wind states
			_control_status.flags.wind = true;
			// reset the timeout timers to prevent repeated resets
			_time_last_beta_fuse = _time_last_imu;
			_time_last_arsp_fuse = _time_last_imu;
			// reset the wind speed states and corresponding covariances
			resetWindStates();
			resetWindCovariance();
		}

		fuseSideslip();
	}
}

void Ekf::controlDragFusion()
{
	if (_params.fusion_mode & MASK_USE_DRAG) {
		if (_control_status.flags.in_air
				&& !_mag_inhibit_yaw_reset_req) {
			if (!_control_status.flags.wind) {
				// reset the wind states and covariances when starting drag accel fusion
				_control_status.flags.wind = true;
				resetWindStates();
				resetWindCovariance();

			} else if (_drag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_drag_sample_delayed)) {
				fuseDrag();

			}

		} else {
			_control_status.flags.wind = false;

		}
	}
}

void Ekf::controlFakePosFusion()
{
	// if we aren't doing any aiding, fake position measurements at the last known position to constrain drift
	// Coincide fake measurements with baro data for efficiency with a minimum fusion rate of 5Hz

	if (!isHorizontalAidingActive()
	    && !(_control_status.flags.fuse_aspd && _control_status.flags.fuse_beta)) {

		// We now need to use a synthetic position observation to prevent unconstrained drift of the INS states.
		_using_synthetic_position = true;

		// Fuse synthetic position observations every 200msec
		if (isTimedOut(_time_last_fake_pos, (uint64_t)2e5) || _fuse_height) {

			Vector3f fake_pos_obs_var;
			Vector2f fake_pos_innov_gate;


			// Reset position and velocity states if we re-commence this aiding method
			if (isTimedOut(_time_last_fake_pos, (uint64_t)4e5)) {
				resetPosition();
				resetVelocity();
				_fuse_hpos_as_odom = false;

				if (_time_last_fake_pos != 0) {
					ECL_WARN_TIMESTAMPED("stopping navigation");
				}

			}
			_time_last_fake_pos = _time_last_imu;

			if (_control_status.flags.in_air && _control_status.flags.tilt_align) {
				fake_pos_obs_var(0) = fake_pos_obs_var(1) = sq(fmaxf(_params.pos_noaid_noise, _params.gps_pos_noise));

			} else {
				fake_pos_obs_var(0) = fake_pos_obs_var(1) = sq(0.5f);
			}

			_gps_pos_innov(0) = _state.pos(0) - _last_known_posNE(0);
			_gps_pos_innov(1) = _state.pos(1) - _last_known_posNE(1);

			// glitch protection is not required so set gate to a large value
			fake_pos_innov_gate(0) = 100.0f;

			fuseHorizontalPosition(_gps_pos_innov, fake_pos_innov_gate, fake_pos_obs_var,
						_gps_pos_innov_var, _gps_pos_test_ratio);
		}

	} else {
		_using_synthetic_position = false;
	}

}

void Ekf::controlAuxVelFusion()
{
	bool data_ready = _auxvel_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_auxvel_sample_delayed);

	if (data_ready && isHorizontalAidingActive()) {

		Vector2f aux_vel_innov_gate;
		Vector3f aux_vel_obs_var;

		_aux_vel_innov = _state.vel - _auxvel_sample_delayed.vel;
		aux_vel_obs_var = _auxvel_sample_delayed.velVar;
		aux_vel_innov_gate(0) = _params.auxvel_gate;

		fuseHorizontalVelocity(_aux_vel_innov, aux_vel_innov_gate, aux_vel_obs_var,
				_aux_vel_innov_var, _aux_vel_test_ratio);

		// Can be enabled after bit for this is added to EKF_AID_MASK
		// fuseVerticalVelocity(_aux_vel_innov, aux_vel_innov_gate, aux_vel_obs_var,
		//		_aux_vel_innov_var, _aux_vel_test_ratio);

	}
}
