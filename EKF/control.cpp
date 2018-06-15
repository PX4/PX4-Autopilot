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

	// Get the magnetic declination
	calcMagDeclination();

	// monitor the tilt alignment
	if (!_control_status.flags.tilt_align) {
		// whilst we are aligning the tilt, monitor the variances
		Vector3f angle_err_var_vec = calcRotVecVariances();

		// Once the tilt variances have reduced to equivalent of 3deg uncertainty, re-set the yaw and magnetic field states
		// and declare the tilt alignment complete
		if ((angle_err_var_vec(0) + angle_err_var_vec(1)) < sq(0.05235f)) {
			_control_status.flags.tilt_align = true;
			_control_status.flags.yaw_align = resetMagHeading(_mag_sample_delayed.mag);

			// send alignment status message to the console
			if (_control_status.flags.baro_hgt) {
				ECL_INFO("EKF aligned, (pressure height, IMU buf: %i, OBS buf: %i)", (int)_imu_buffer_length, (int)_obs_buffer_length);

			} else if (_control_status.flags.ev_hgt) {
				ECL_INFO("EKF aligned, (EV height, IMU buf: %i, OBS buf: %i)", (int)_imu_buffer_length, (int)_obs_buffer_length);

			} else if (_control_status.flags.gps_hgt) {
				ECL_INFO("EKF aligned, (GPS height, IMU buf: %i, OBS buf: %i)", (int)_imu_buffer_length, (int)_obs_buffer_length);

			} else if (_control_status.flags.rng_hgt) {
				ECL_INFO("EKF aligned, (range height, IMU buf: %i, OBS buf: %i)", (int)_imu_buffer_length, (int)_obs_buffer_length);

			} else {
				ECL_ERR("EKF aligned, (unknown height, IMU buf: %i, OBS buf: %i)", (int)_imu_buffer_length, (int)_obs_buffer_length);
			}

		}

	}

	// check faultiness (before pop_first_older_than) to see if we can change back to original height sensor
	const baroSample &baro_init = _baro_buffer.get_newest();
	_baro_hgt_faulty = !((_time_last_imu - baro_init.time_us) < 2 * BARO_MAX_INTERVAL);

	const gpsSample &gps_init = _gps_buffer.get_newest();
	_gps_hgt_faulty = !((_time_last_imu - gps_init.time_us) < 2 * GPS_MAX_INTERVAL);

	const rangeSample &rng_init = _range_buffer.get_newest();
	_rng_hgt_faulty = !((_time_last_imu - rng_init.time_us) < 2 * RNG_MAX_INTERVAL);

	// check for arrival of new sensor data at the fusion time horizon
	_gps_data_ready = _gps_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_gps_sample_delayed);
	_mag_data_ready = _mag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_mag_sample_delayed);

	_delta_time_baro_us = _baro_sample_delayed.time_us;
	_baro_data_ready = _baro_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed);

	// if we have a new baro sample save the delta time between this sample and the last sample which is
	// used below for baro offset calculations
	if (_baro_data_ready) {
		_delta_time_baro_us = _baro_sample_delayed.time_us - _delta_time_baro_us;
	}

	// calculate 2,2 element of rotation matrix from sensor frame to earth frame
	_R_rng_to_earth_2_2 = _R_to_earth(2, 0) * _sin_tilt_rng + _R_to_earth(2, 2) * _cos_tilt_rng;
	_range_data_ready = _range_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_range_sample_delayed)
			    && (_R_rng_to_earth_2_2 > _params.range_cos_max_tilt);

	checkForStuckRange();

	_flow_data_ready = _flow_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_flow_sample_delayed)
			   && (_R_to_earth(2, 2) > 0.7071f);
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

	// For efficiency, fusion of direct state observations for position and velocity is performed sequentially
	// in a single function using sensor data from multiple sources (GPS, baro, range finder, etc)
	controlVelPosFusion();

	// Additional data from an external vision pose estimator can be fused.
	controlExternalVisionFusion();

	// Additional NE velocity data from an auxiliary sensor can be fused
	controlAuxVelFusion();

	// check if we are no longer fusing measurements that directly constrain velocity drift
	update_deadreckoning_status();
}

void Ekf::controlExternalVisionFusion()
{
	// Check for new exernal vision data
	if (_ev_data_ready) {

		// if the ev data is not in a NED reference frame, then the transformation between EV and EKF navigation frames
		// needs to be calculated and the observations rotated into the EKF frame of reference
		if ((_params.fusion_mode & MASK_ROTATE_EV) && (_params.fusion_mode & MASK_USE_EVPOS) && !_control_status.flags.ev_yaw) {
			// rotate EV measurements into the EKF Navigation frame
			calcExtVisRotMat();
		}

		// external vision position aiding selection logic
		if ((_params.fusion_mode & MASK_USE_EVPOS) && !_control_status.flags.ev_pos && _control_status.flags.tilt_align
		    && _control_status.flags.yaw_align) {

			// check for a exernal vision measurement that has fallen behind the fusion time horizon
			if (_time_last_imu - _time_last_ext_vision < 2 * EV_MAX_INTERVAL) {
				// turn on use of external vision measurements for position
				_control_status.flags.ev_pos = true;
				ECL_INFO("EKF commencing external vision position fusion");

				// reset the position if we are not already aiding using GPS, else use a relative position
				// method for fusing the position data
				if (_control_status.flags.gps) {
					_fuse_hpos_as_odom = true;

				} else {
					resetPosition();
					resetVelocity();

				}
			}
		}

		// external vision yaw aiding selection logic
		if (!_control_status.flags.gps && (_params.fusion_mode & MASK_USE_EVYAW) && !_control_status.flags.ev_yaw && _control_status.flags.tilt_align) {
			// don't start using EV data unless daa is arriving frequently
			if (_time_last_imu - _time_last_ext_vision < 2 * EV_MAX_INTERVAL) {
				// reset the yaw angle to the value from the observaton quaternion
				// get the roll, pitch, yaw estimates from the quaternion states
				Quatf q_init(_state.quat_nominal);
				Eulerf euler_init(q_init);

				// get initial yaw from the observation quaternion
				const extVisionSample &ev_newest = _ext_vision_buffer.get_newest();
				Quatf q_obs(ev_newest.quat);
				Eulerf euler_obs(q_obs);
				euler_init(2) = euler_obs(2);

				// save a copy of the quaternion state for later use in calculating the amount of reset change
				Quatf quat_before_reset = _state.quat_nominal;

				// calculate initial quaternion states for the ekf
				_state.quat_nominal = Quatf(euler_init);

				// calculate the amount that the quaternion has changed by
				_state_reset_status.quat_change = quat_before_reset.inversed() * _state.quat_nominal;

				// add the reset amount to the output observer buffered data
				// Note q1 *= q2 is equivalent to q1 = q2 * q1
				for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
					_output_buffer[i].quat_nominal *= _state_reset_status.quat_change;
				}

				// apply the change in attitude quaternion to our newest quaternion estimate
				// which was already taken out from the output buffer
				_output_new.quat_nominal = _state_reset_status.quat_change * _output_new.quat_nominal;

				// capture the reset event
				_state_reset_status.quat_counter++;

				// flag the yaw as aligned
				_control_status.flags.yaw_align = true;

				// turn on fusion of external vision yaw measurements and disable all magnetoemter fusion
				_control_status.flags.ev_yaw = true;
				_control_status.flags.mag_hdg = false;
				_control_status.flags.mag_3D = false;
				_control_status.flags.mag_dec = false;

				ECL_INFO("EKF commencing external vision yaw fusion");
			}
		}

		// determine if we should start using the height observations
		if (_params.vdist_sensor_type == VDIST_SENSOR_EV) {
			// don't start using EV data unless data is arriving frequently
			if (!_control_status.flags.ev_hgt && (_time_last_imu - _time_last_ext_vision < 2 * EV_MAX_INTERVAL)) {
				setControlEVHeight();
				resetHeight();
			}
		}

		// determine if we should use the vertical position observation
		if (_control_status.flags.ev_hgt) {
			_fuse_height = true;
		}

		// determine if we should use the horizontal position observations
		if (_control_status.flags.ev_pos) {
			_fuse_pos = true;

			// correct position and height for offset relative to IMU
			Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
			Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_ev_sample_delayed.posNED(0) -= pos_offset_earth(0);
			_ev_sample_delayed.posNED(1) -= pos_offset_earth(1);
			_ev_sample_delayed.posNED(2) -= pos_offset_earth(2);

			// Use an incremental position fusion method for EV data if using GPS or if the observations are not in NED
			if (_control_status.flags.gps || (_params.fusion_mode & MASK_ROTATE_EV)) {
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
					Vector3f ev_delta_pos = _ev_sample_delayed.posNED - _pos_meas_prev;

					// rotate measurement into body frame if required
					if (_params.fusion_mode & MASK_ROTATE_EV) {
						ev_delta_pos = _ev_rot_mat * ev_delta_pos;
					}

					// use the change in position since the last measurement
					_vel_pos_innov[3] = _state.pos(0) - _hpos_pred_prev(0) - ev_delta_pos(0);
					_vel_pos_innov[4] = _state.pos(1) - _hpos_pred_prev(1) - ev_delta_pos(1);

				}

				// record observation and estimate for use next time
				_pos_meas_prev = _ev_sample_delayed.posNED;
				_hpos_pred_prev(0) = _state.pos(0);
				_hpos_pred_prev(1) = _state.pos(1);

			} else {
				// use the absolute position
				_vel_pos_innov[3] = _state.pos(0) - _ev_sample_delayed.posNED(0);
				_vel_pos_innov[4] = _state.pos(1) - _ev_sample_delayed.posNED(1);

				// check if we have been deadreckoning too long
				if (_time_last_imu - _time_last_pos_fuse > _params.no_gps_timeout_max) {
					// don't reset velocity if we have another source of aiding constraining it
					if (_time_last_imu - _time_last_of_fuse > (uint64_t)1E6) {
						resetVelocity();
					}

					resetPosition();
				}
			}

			// observation 1-STD error
			_posObsNoiseNE = fmaxf(_ev_sample_delayed.posErr, 0.01f);

			// innovation gate size
			_posInnovGateNE = fmaxf(_params.ev_innov_gate, 1.0f);
		}

		// Fuse available NED position data into the main filter
		if (_fuse_height || _fuse_pos) {
			fuseVelPosHeight();
			_fuse_pos = _fuse_height = false;
			_fuse_hpos_as_odom = false;

		}

		// determine if we should use the yaw observation
		if (_control_status.flags.ev_yaw) {
			fuseHeading();

		}

	} else if (_control_status.flags.ev_pos
		   && (_time_last_imu >= _time_last_ext_vision)
		   && (_time_last_imu - _time_last_ext_vision > (uint64_t)_params.no_gps_timeout_max)) {

		// Turn off EV fusion mode if no data has been received
		_control_status.flags.ev_pos = false;
		ECL_INFO("EKF External Vision Data Stopped");

	}
}

void Ekf::controlOpticalFlowFusion()
{
	// Check if on ground motion is un-suitable for use of optical flow
	if (!_control_status.flags.in_air) {
		// When on ground check if the vehicle is being shaken or moved in a way that could cause a loss of navigation
		float accel_norm = _accel_vec_filt.norm();
		bool motion_is_excessive = ((accel_norm > 14.7f) // accel greater than 1.5g
					    || (accel_norm < 4.9f) // accel less than 0.5g
					    || (_ang_rate_mag_filt > _flow_max_rate) // angular rate exceeds flow sensor limit
					    || (_R_to_earth(2,2) < 0.866f)); // tilted more than 30 degrees
		if (motion_is_excessive) {
			_time_bad_motion_us = _imu_sample_delayed.time_us;

		} else {
			_time_good_motion_us = _imu_sample_delayed.time_us;
		}

	} else {
		_time_bad_motion_us = 0;
		_time_good_motion_us = _imu_sample_delayed.time_us;
	}

	// New optical flow data has fallen behind the fusion time horizon and is ready to be fused
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
				  || (_control_status.flags.opt_flow && !_control_status.flags.gps && !_control_status.flags.ev_pos) // is completely reliant on optical flow
				  || (_control_status.flags.gps && (_gps_error_norm > gps_err_norm_lim))); // is using GPS, but GPS is bad

		if (!_inhibit_flow_use && _control_status.flags.opt_flow) {
			// inhibit use of optical flow if motion is unsuitable and we are not reliant on it for flight navigation
			bool preflight_motion_not_ok = !_control_status.flags.in_air && ((_imu_sample_delayed.time_us - _time_good_motion_us) > (uint64_t)1E5);
			bool flight_motion_not_ok = _control_status.flags.in_air && !_range_aid_mode_enabled;
			if ((preflight_motion_not_ok || flight_motion_not_ok) && !flow_required) {
				_inhibit_flow_use = true;
			}
		} else if (_inhibit_flow_use && !_control_status.flags.opt_flow){
			// allow use of optical flow if motion is suitable or we are reliant on it for flight navigation
			bool preflight_motion_ok = !_control_status.flags.in_air && ((_imu_sample_delayed.time_us - _time_bad_motion_us) > (uint64_t)5E6);
			bool flight_motion_ok = _control_status.flags.in_air && _range_aid_mode_enabled;
			if (preflight_motion_ok || flight_motion_ok || flow_required) {
				_inhibit_flow_use = false;
			}
		}

		// Handle cases where we are using optical flow but are no longer able to because data is old
		// or its use has been inhibited.
		if (_control_status.flags.opt_flow) {
		       if (_inhibit_flow_use) {
			       _control_status.flags.opt_flow = false;
			       _time_last_of_fuse = 0;

			} else if (_time_last_imu - _flow_sample_delayed.time_us > (uint64_t)_params.no_gps_timeout_max) {
				_control_status.flags.opt_flow = false;

			}
		}

		// Accumulate autopilot gyro data across the same time interval as the flow sensor
		_imu_del_ang_of += _imu_sample_delayed.delta_ang - _state.gyro_bias;
		_delta_time_of += _imu_sample_delayed.delta_ang_dt;

		// optical flow fusion mode selection logic
		if ((_params.fusion_mode & MASK_USE_OF) // optical flow has been selected by the user
			&& !_control_status.flags.opt_flow // we are not yet using flow data
			&& _control_status.flags.tilt_align // we know our tilt attitude
			&& !_inhibit_flow_use
			&& get_terrain_valid()) // we have a valid distance to ground estimate
		{
			// If the heading is not aligned, reset the yaw and magnetic field states
			if (!_control_status.flags.yaw_align) {
				_control_status.flags.yaw_align = resetMagHeading(_mag_sample_delayed.mag);
			}

			// If the heading is valid and use is not inhibited , start using optical flow aiding
			if (_control_status.flags.yaw_align) {
				// set the flag and reset the fusion timeout
				_control_status.flags.opt_flow = true;
				_time_last_of_fuse = _time_last_imu;

				// if we are not using GPS then the velocity and position states and covariances need to be set
				if (!_control_status.flags.gps || !_control_status.flags.ev_pos) {
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
		if (_control_status.flags.opt_flow
		    && !_control_status.flags.gps
		    && !_control_status.flags.ev_pos) {

			bool do_reset = _time_last_imu - _time_last_of_fuse > _params.no_gps_timeout_max;

			if (do_reset) {
				resetVelocity();
				resetPosition();
			}
		}

		// fuse the data if the terrain/distance to bottom is valid but use a more relaxed check to enable it to survive bad range finder data
		if (_control_status.flags.opt_flow && (_time_last_imu - _time_last_hagl_fuse < (uint64_t)10e6)) {
			// Update optical flow bias estimates
			calcOptFlowBias();

			// Fuse optical flow LOS rate observations into the main filter
			fuseOptFlow();
			_last_known_posNE(0) = _state.pos(0);
			_last_known_posNE(1) = _state.pos(1);

		}
	}

}

void Ekf::controlGpsFusion()
{
	// Check for new GPS data that has fallen behind the fusion time horizon
	if (_gps_data_ready) {

		// Determine if we should use GPS aiding for velocity and horizontal position
		// To start using GPS we need angular alignment completed, the local NED origin set and GPS data that has not failed checks recently
		bool gps_checks_passing = (_time_last_imu - _last_gps_fail_us > (uint64_t)5e6);
		bool gps_checks_failing = (_time_last_imu - _last_gps_pass_us > (uint64_t)5e6);
		if ((_params.fusion_mode & MASK_USE_GPS) && !_control_status.flags.gps) {
			if (_control_status.flags.tilt_align && _NED_origin_initialised && gps_checks_passing) {
				// If the heading is not aligned, reset the yaw and magnetic field states
				// Do not use external vision for yaw if using GPS because yaw needs to be
				// defined relative to an NED reference frame
				if (!_control_status.flags.yaw_align || _control_status.flags.ev_yaw || _mag_inhibit_yaw_reset_req) {
					_control_status.flags.yaw_align = false;
					_control_status.flags.ev_yaw = false;
					_control_status.flags.yaw_align = resetMagHeading(_mag_sample_delayed.mag);
					// Handle the special case where we have not been constraining yaw drift or learning yaw bias due
					// to assumed invalid mag field associated with indoor operation with a downwards looking flow sensor.
					if (_mag_inhibit_yaw_reset_req) {
						_mag_inhibit_yaw_reset_req = false;
						// Zero the yaw bias covariance and set the variance to the initial alignment uncertainty
						float dt = 0.001f * (float)FILTER_UPDATE_PERIOD_MS;
						setDiag(P, 12, 12, sq(_params.switch_on_gyro_bias * dt));
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
						ECL_INFO("EKF commencing GPS fusion");
						_time_last_gps = _time_last_imu;
					}
				}
			}

		}  else if (!(_params.fusion_mode & MASK_USE_GPS)) {
			_control_status.flags.gps = false;

		}

		// Handle the case where we are using GPS and another source of aiding and GPS is failing checks
		if (_control_status.flags.gps  && gps_checks_failing && (_control_status.flags.opt_flow || _control_status.flags.ev_pos)) {
			_control_status.flags.gps = false;
			ECL_WARN("EKF GPS data quality poor - stopping use");
		}

		// handle the case when we now have GPS, but have not been using it for an extended period
		if (_control_status.flags.gps) {
			// We are relying on aiding to constrain drift so after a specified time
			// with no aiding we need to do something
			bool do_reset = (_time_last_imu - _time_last_pos_fuse > _params.no_gps_timeout_max)
					&& (_time_last_imu - _time_last_delpos_fuse > _params.no_gps_timeout_max)
					&& (_time_last_imu - _time_last_vel_fuse > _params.no_gps_timeout_max)
					&& (_time_last_imu - _time_last_of_fuse > _params.no_gps_timeout_max);

			// We haven't had an absolute position fix for a longer time so need to do something
			do_reset = do_reset || (_time_last_imu - _time_last_pos_fuse > 2 * _params.no_gps_timeout_max);

			if (do_reset) {
				// use GPS velocity data to check and correct yaw angle if a FW vehicle
				if (_control_status.flags.fixed_wing && _control_status.flags.in_air) {
					// if flying a fixed wing aircraft, do a complete reset that includes yaw
					realignYawGPS();
				}

				resetVelocity();
				resetPosition();
				_velpos_reset_request = false;
				ECL_WARN("EKF GPS fusion timeout - reset to GPS");

				// Reset the timeout counters
				_time_last_pos_fuse = _time_last_imu;
				_time_last_vel_fuse = _time_last_imu;

			}
		}

		// Only use GPS data for position and velocity aiding if enabled
		if (_control_status.flags.gps) {
			_fuse_pos = true;
			_fuse_vert_vel = true;
			_fuse_hor_vel = true;

			// correct velocity for offset relative to IMU
			Vector3f ang_rate = _imu_sample_delayed.delta_ang * (1.0f / _imu_sample_delayed.delta_ang_dt);
			Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
			Vector3f vel_offset_body = cross_product(ang_rate, pos_offset_body);
			Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
			_gps_sample_delayed.vel -= vel_offset_earth;

			// correct position and height for offset relative to IMU
			Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_gps_sample_delayed.pos(0) -= pos_offset_earth(0);
			_gps_sample_delayed.pos(1) -= pos_offset_earth(1);
			_gps_sample_delayed.hgt += pos_offset_earth(2);

			// calculate observation process noise
			float lower_limit = fmaxf(_params.gps_pos_noise, 0.01f);

			if (_control_status.flags.opt_flow || _control_status.flags.ev_pos) {
				// if we are using other sources of aiding, then relax the upper observation
				// noise limit which prevents bad GPS perturbing the position estimate
				_posObsNoiseNE = fmaxf(_gps_sample_delayed.hacc, lower_limit);

			} else {
				// if we are not using another source of aiding, then we are reliant on the GPS
				// observations to constrain attitude errors and must limit the observation noise value.
				float upper_limit = fmaxf(_params.pos_noaid_noise, lower_limit);
				_posObsNoiseNE = math::constrain(_gps_sample_delayed.hacc, lower_limit, upper_limit);
			}

			_velObsVarNE(1) = _velObsVarNE(0) = sq(fmaxf(_gps_sample_delayed.sacc, _params.gps_vel_noise));

			// calculate innovations
			_vel_pos_innov[0] = _state.vel(0) - _gps_sample_delayed.vel(0);
			_vel_pos_innov[1] = _state.vel(1) - _gps_sample_delayed.vel(1);
			_vel_pos_innov[2] = _state.vel(2) - _gps_sample_delayed.vel(2);
			_vel_pos_innov[3] = _state.pos(0) - _gps_sample_delayed.pos(0);
			_vel_pos_innov[4] = _state.pos(1) - _gps_sample_delayed.pos(1);

			// set innovation gate size
			_posInnovGateNE = fmaxf(_params.posNE_innov_gate, 1.0f);
			_hvelInnovGate = fmaxf(_params.vel_innov_gate, 1.0f);
		}

	} else if (_control_status.flags.gps && (_imu_sample_delayed.time_us - _gps_sample_delayed.time_us > (uint64_t)10e6)) {
		_control_status.flags.gps = false;
		ECL_WARN("EKF GPS data stopped");
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
	bool bad_vert_accel = (_control_status.flags.baro_hgt && // we can only run this check if vertical position and velocity observations are indepedant
			(sq(_vel_pos_innov[5] * _vel_pos_innov[2]) > var_product_lim * (_vel_pos_innov_var[5] * _vel_pos_innov_var[2])) && // vertical position and velocity sensors are in agreement that we have a significant error
			(_vel_pos_innov[2] > 0.0f) && // positive innovation indicates that the inertial nav thinks it is falling
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
		_bad_vert_accel_detected = (_time_last_imu - _time_bad_vert_accel < BADACC_PROBATION);

	} else {
		_bad_vert_accel_detected = bad_vert_accel;
	}

	// check if height is continuously failing becasue of accel errors
	bool continuous_bad_accel_hgt = ((_time_last_imu - _time_good_vert_accel) > (unsigned)_params.bad_acc_reset_delay_us);

	// check if height has been inertial deadreckoning for too long
	bool hgt_fusion_timeout = ((_time_last_imu - _time_last_hgt_fuse) > (uint64_t)5e6);

	// reset the vertical position and velocity states
	if (hgt_fusion_timeout || continuous_bad_accel_hgt) {
		// boolean that indicates we will do a height reset
		bool reset_height = false;

		// handle the case where we are using baro for height
		if (_control_status.flags.baro_hgt) {
			// check if GPS height is available
			const gpsSample &gps_init = _gps_buffer.get_newest();
			bool gps_hgt_available = ((_time_last_imu - gps_init.time_us) < 2 * GPS_MAX_INTERVAL);
			bool gps_hgt_accurate = (gps_init.vacc < _params.req_vacc);

			const baroSample &baro_init = _baro_buffer.get_newest();
			bool baro_hgt_available = ((_time_last_imu - baro_init.time_us) < 2 * BARO_MAX_INTERVAL);

			// check for inertial sensing errors in the last 10 seconds
			bool prev_bad_vert_accel = (_time_last_imu - _time_bad_vert_accel < BADACC_PROBATION);

			// reset to GPS if adequate GPS data is available and the timeout cannot be blamed on IMU data
			bool reset_to_gps = gps_hgt_available && gps_hgt_accurate && !_gps_hgt_faulty && !prev_bad_vert_accel;

			// reset to GPS if GPS data is available and there is no Baro data
			reset_to_gps = reset_to_gps || (gps_hgt_available && !baro_hgt_available);

			// reset to Baro if we are not doing a GPS reset and baro data is available
			bool reset_to_baro = !reset_to_gps && baro_hgt_available;

			if (reset_to_gps) {
				// set height sensor health
				_baro_hgt_faulty = true;

				// declare the GPS height healthy
				_gps_hgt_faulty = false;

				// reset the height mode
				setControlGPSHeight();

				// request a reset
				reset_height = true;
				ECL_WARN("EKF baro hgt timeout - reset to GPS");

			} else if (reset_to_baro) {
				// set height sensor health
				_baro_hgt_faulty = false;

				// reset the height mode
				setControlBaroHeight();

				// request a reset
				reset_height = true;
				ECL_WARN("EKF baro hgt timeout - reset to baro");

			} else {
				// we have nothing we can reset to
				// deny a reset
				reset_height = false;

			}
		}

		// handle the case we are using GPS for height
		if (_control_status.flags.gps_hgt) {
			// check if GPS height is available
			const gpsSample &gps_init = _gps_buffer.get_newest();
			bool gps_hgt_available = ((_time_last_imu - gps_init.time_us) < 2 * GPS_MAX_INTERVAL);
			bool gps_hgt_accurate = (gps_init.vacc < _params.req_vacc);

			// check the baro height source for consistency and freshness
			const baroSample &baro_init = _baro_buffer.get_newest();
			bool baro_data_fresh = ((_time_last_imu - baro_init.time_us) < 2 * BARO_MAX_INTERVAL);
			float baro_innov = _state.pos(2) - (_hgt_sensor_offset - baro_init.hgt + _baro_hgt_offset);
			bool baro_data_consistent = fabsf(baro_innov) < (sq(_params.baro_noise) + P[9][9]) * sq(_params.baro_innov_gate);

			// if baro data is acceptable and GPS data is inaccurate, reset height to baro
			bool reset_to_baro = baro_data_consistent && baro_data_fresh && !_baro_hgt_faulty && !gps_hgt_accurate;

			// if GPS height is unavailable and baro data is available, reset height to baro
			reset_to_baro = reset_to_baro || (!gps_hgt_available && baro_data_fresh);

			// if we cannot switch to baro and GPS data is available, reset height to GPS
			bool reset_to_gps = !reset_to_baro && gps_hgt_available;

			if (reset_to_baro) {
				// set height sensor health
				_gps_hgt_faulty = true;
				_baro_hgt_faulty = false;

				// reset the height mode
				setControlBaroHeight();

				// request a reset
				reset_height = true;
				ECL_WARN("EKF gps hgt timeout - reset to baro");

			} else if (reset_to_gps) {
				// set height sensor health
				_gps_hgt_faulty = false;

				// reset the height mode
				setControlGPSHeight();

				// request a reset
				reset_height = true;
				ECL_WARN("EKF gps hgt timeout - reset to GPS");

			} else {
				// we have nothing to reset to
				reset_height = false;

			}
		}

		// handle the case we are using range finder for height
		if (_control_status.flags.rng_hgt) {
			// check if range finder data is available
			const rangeSample &rng_init = _range_buffer.get_newest();
			bool rng_data_available = ((_time_last_imu - rng_init.time_us) < 2 * RNG_MAX_INTERVAL);

			// check if baro data is available
			const baroSample &baro_init = _baro_buffer.get_newest();
			bool baro_data_available = ((_time_last_imu - baro_init.time_us) < 2 * BARO_MAX_INTERVAL);

			// reset to baro if we have no range data and baro data is available
			bool reset_to_baro = !rng_data_available && baro_data_available;

			// reset to range data if it is available
			bool reset_to_rng = rng_data_available;

			if (reset_to_baro) {
				// set height sensor health
				_rng_hgt_faulty = true;
				_baro_hgt_faulty = false;

				// reset the height mode
				setControlBaroHeight();

				// request a reset
				reset_height = true;
				ECL_WARN("EKF rng hgt timeout - reset to baro");

			} else if (reset_to_rng) {
				// set height sensor health
				_rng_hgt_faulty = false;

				// reset the height mode
				setControlRangeHeight();

				// request a reset
				reset_height = true;
				ECL_WARN("EKF rng hgt timeout - reset to rng hgt");

			} else {
				// we have nothing to reset to
				reset_height = false;

			}
		}

		// handle the case where we are using external vision data for height
		if (_control_status.flags.ev_hgt) {
			// check if vision data is available
			const extVisionSample &ev_init = _ext_vision_buffer.get_newest();
			bool ev_data_available = ((_time_last_imu - ev_init.time_us) < 2 * EV_MAX_INTERVAL);

			// check if baro data is available
			const baroSample &baro_init = _baro_buffer.get_newest();
			bool baro_data_available = ((_time_last_imu - baro_init.time_us) < 2 * BARO_MAX_INTERVAL);

			// reset to baro if we have no vision data and baro data is available
			bool reset_to_baro = !ev_data_available && baro_data_available;

			// reset to ev data if it is available
			bool reset_to_ev = ev_data_available;

			if (reset_to_baro) {
				// set height sensor health
				_baro_hgt_faulty = false;

				// reset the height mode
				setControlBaroHeight();

				// request a reset
				reset_height = true;
				ECL_WARN("EKF ev hgt timeout - reset to baro");

			} else if (reset_to_ev) {
				// reset the height mode
				setControlEVHeight();

				// request a reset
				reset_height = true;
				ECL_WARN("EKF ev hgt timeout - reset to ev hgt");

			} else {
				// we have nothing to reset to
				reset_height = false;

			}
		}

		// Reset vertical position and velocity states to the last measurement
		if (reset_height) {
			resetHeight();
			// Reset the timout timer
			_time_last_hgt_fuse = _time_last_imu;

		}

	}
}

void Ekf::controlHeightFusion()
{
	// set control flags for the desired primary height source

	if (_range_data_ready) {
		// correct the range data for position offset relative to the IMU
		Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;
		Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
		_range_sample_delayed.rng += pos_offset_earth(2) / _R_rng_to_earth_2_2;
	}

	rangeAidConditionsMet();

	_range_aid_mode_selected = (_params.range_aid == 1) && _range_aid_mode_enabled;

	if (_params.vdist_sensor_type == VDIST_SENSOR_BARO) {

		if (_range_aid_mode_selected && _range_data_ready && !_rng_hgt_faulty) {
			setControlRangeHeight();
			_fuse_height = true;

			// we have just switched to using range finder, calculate height sensor offset such that current
			// measurment matches our current height estimate
			if (_control_status_prev.flags.rng_hgt != _control_status.flags.rng_hgt) {
				if (get_terrain_valid()) {
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

			// Turn off ground effect compensation if it times out or sufficient height has been gained
			// since takeoff.
			if (_control_status.flags.gnd_effect) {
				if ((_time_last_imu - _time_last_gnd_effect_on > GNDEFFECT_TIMEOUT) ||
				    (((_last_on_ground_posD - _state.pos(2)) > _params.gnd_effect_max_hgt) &&
				     _control_status.flags.in_air)) {

					_control_status.flags.gnd_effect = false;
				}
			}

		} else if (_control_status.flags.gps_hgt && _gps_data_ready && !_gps_hgt_faulty) {
			// switch to gps if there was a reset to gps
			_fuse_height = true;

			// we have just switched to using gps height, calculate height sensor offset such that current
			// measurment matches our current height estimate
			if (_control_status_prev.flags.gps_hgt != _control_status.flags.gps_hgt) {
				_hgt_sensor_offset = _gps_sample_delayed.hgt - _gps_alt_ref + _state.pos(2);
			}
		}
	}

	// set the height data source to range if requested
	if ((_params.vdist_sensor_type == VDIST_SENSOR_RANGE) && !_rng_hgt_faulty) {
		setControlRangeHeight();
		_fuse_height = _range_data_ready;

		// we have just switched to using range finder, calculate height sensor offset such that current
		// measurment matches our current height estimate
		if (_control_status_prev.flags.rng_hgt != _control_status.flags.rng_hgt) {
			// use the parameter rng_gnd_clearance if on ground to avoid a noisy offset initialization (e.g. sonar)
			if (_control_status.flags.in_air && get_terrain_valid()) {

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

		if (_range_aid_mode_selected && _range_data_ready && !_rng_hgt_faulty) {
			setControlRangeHeight();
			_fuse_height = true;

			// we have just switched to using range finder, calculate height sensor offset such that current
			// measurment matches our current height estimate
			if (_control_status_prev.flags.rng_hgt != _control_status.flags.rng_hgt) {
				if (get_terrain_valid()) {
					_hgt_sensor_offset = _terrain_vpos;

				} else {
					_hgt_sensor_offset = _R_rng_to_earth_2_2 * _range_sample_delayed.rng + _state.pos(2);
				}
			}

		} else if (!_range_aid_mode_selected && _gps_data_ready && !_gps_hgt_faulty) {
			setControlGPSHeight();
			_fuse_height = true;

			// we have just switched to using gps height, calculate height sensor offset such that current
			// measurment matches our current height estimate
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
		if (_control_status.flags.baro_hgt && _baro_data_ready && !_baro_hgt_faulty) {
			// switch to baro if there was a reset to baro
			_fuse_height = true;

			// we have just switched to using baro height, we don't need to set a height sensor offset
			// since we track a separate _baro_hgt_offset
			if (_control_status_prev.flags.baro_hgt != _control_status.flags.baro_hgt) {
				_hgt_sensor_offset = 0.0f;
			}
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

	if ((_time_last_imu - _time_last_hgt_fuse) > 2 * RNG_MAX_INTERVAL && _control_status.flags.rng_hgt
	    && !_range_data_ready) {

		// If we are supposed to be using range finder data as the primary height sensor, have missed or rejected measurements
		// and are on the ground, then synthesise a measurement at the expected on ground value
		if (!_control_status.flags.in_air) {
			_range_sample_delayed.rng = _params.rng_gnd_clearance;
			_range_sample_delayed.time_us = _imu_sample_delayed.time_us;

		}

		_fuse_height = true;
	}


}

void Ekf::rangeAidConditionsMet()
{
	// if the parameter for range aid is enabled we allow to switch from using the primary height source to using range finder as height source
	// under the following conditions
	// 1) we are not further than max_hagl_for_range_aid away from the ground
	// 2) our ground speed is not higher than max_vel_for_range_aid
	// 3) Our terrain estimate is stable (needs better checks)
	// 4) We are in-air
	if (_control_status.flags.in_air) {
		// check if we can use range finder measurements to estimate height, use hysteresis to avoid rapid switching
		bool can_use_range_finder;
		if (_range_aid_mode_enabled) {
			can_use_range_finder = (_terrain_vpos - _state.pos(2) < _params.max_hagl_for_range_aid) && get_terrain_valid();

		} else {
			// if we were not using range aid in the previous iteration then require the current height above terrain to be
			// smaller than 70 % of the maximum allowed ground distance for range aid
			can_use_range_finder = (_terrain_vpos - _state.pos(2) < 0.7f * _params.max_hagl_for_range_aid) && get_terrain_valid();
		}

		bool horz_vel_valid = (_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.opt_flow)
				      && (_fault_status.value == 0);

		if (horz_vel_valid) {
			float ground_vel = sqrtf(_state.vel(0) * _state.vel(0) + _state.vel(1) * _state.vel(1));

			if (_range_aid_mode_enabled) {
				can_use_range_finder &= ground_vel < _params.max_vel_for_range_aid;

			} else {
				// if we were not using range aid in the previous iteration then require the ground velocity to be
				// smaller than 70 % of the maximum allowed ground velocity for range aid
				can_use_range_finder &= ground_vel < 0.7f * _params.max_vel_for_range_aid;
			}

		} else {
			can_use_range_finder = false;
		}

		// use hysteresis to check for hagl
		if (_range_aid_mode_enabled) {
			can_use_range_finder &= ((_hagl_innov * _hagl_innov / (sq(_params.range_aid_innov_gate) * _hagl_innov_var)) < 1.0f);

		} else {
			// if we were not using range aid in the previous iteration then use a much lower (1/100) threshold to avoid
			// switching to range finder too soon (wait for terrain to update).
			can_use_range_finder &= ((_hagl_innov * _hagl_innov / (sq(_params.range_aid_innov_gate) * _hagl_innov_var)) < 0.01f);
		}

		_range_aid_mode_enabled = can_use_range_finder;

	} else {
		_range_aid_mode_enabled = false;
	}
}

void Ekf::checkForStuckRange()
{
	if (_range_data_ready && _range_sample_delayed.time_us - _time_last_rng_ready > (uint64_t)10e6 &&
	    _control_status.flags.in_air) {

		_control_status.flags.rng_stuck = true;

		//require a variance of rangefinder values to check for "stuck" measurements
		if (_rng_check_max_val - _rng_check_min_val > 1.0f) {
			_time_last_rng_ready = _range_sample_delayed.time_us;
			_rng_check_min_val = 0.0f;
			_rng_check_max_val = 0.0f;
			_control_status.flags.rng_stuck = false;

		} else {
			if (_range_sample_delayed.rng > _rng_check_max_val) {
				_rng_check_max_val = _range_sample_delayed.rng;
			}

			if (_rng_check_min_val < 0.1f || _range_sample_delayed.rng < _rng_check_min_val) {
				_rng_check_min_val = _range_sample_delayed.rng;
			}

			_range_data_ready = false;
		}

	} else if (_range_data_ready) {
		_time_last_rng_ready = _range_sample_delayed.time_us;
	}
}

void Ekf::controlAirDataFusion()
{
	// control activation and initialisation/reset of wind states required for airspeed fusion

	// If both airspeed and sideslip fusion have timed out and we are not using a drag observation model then we no longer have valid wind estimates
	bool airspeed_timed_out = _time_last_imu - _time_last_arsp_fuse > (uint64_t)10e6;
	bool sideslip_timed_out = _time_last_imu - _time_last_beta_fuse > (uint64_t)10e6;

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
	bool sideslip_timed_out = _time_last_imu - _time_last_beta_fuse > (uint64_t)10e6;
	bool airspeed_timed_out = _time_last_imu - _time_last_arsp_fuse > (uint64_t)10e6;

	if (_control_status.flags.wind && airspeed_timed_out && sideslip_timed_out && !(_params.fusion_mode & MASK_USE_DRAG)) {
		_control_status.flags.wind = false;
	}

	// Perform synthetic sideslip fusion when in-air and sideslip fuson had been enabled externally in addition to the following criteria:

	// Suffient time has lapsed sice the last fusion
	bool beta_fusion_time_triggered = _time_last_imu - _time_last_beta_fuse > _params.beta_avg_ft_us;

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

void Ekf::controlMagFusion()
{
	// If we are on ground, store the local position and time to use as a reference
	// Also reset the flight alignment flag so that the mag fields will be re-initialised next time we achieve flight altitude
	if (!_control_status.flags.in_air) {
		_last_on_ground_posD = _state.pos(2);
		_flt_mag_align_complete = false;
		_num_bad_flight_yaw_events = 0;
	}

	// check for new magnetometer data that has fallen behind the fusion time horizon
	// If we are using external vision data for heading then no magnetometer fusion is used
	if (!_control_status.flags.ev_yaw && _mag_data_ready) {

		// Determine if we should use simple magnetic heading fusion which works better when there are large external disturbances
		// or the more accurate 3-axis fusion
		if (_control_status.flags.mag_fault) {
			// do no magnetometer fusion at all
			_control_status.flags.mag_hdg = false;
			_control_status.flags.mag_3D = false;

		} else if (_params.mag_fusion_type == MAG_FUSE_TYPE_AUTO || _params.mag_fusion_type == MAG_FUSE_TYPE_AUTOFW) {
			// Check if height has increased sufficiently to be away from ground magnetic anomalies
			bool height_achieved = (_last_on_ground_posD - _state.pos(2)) > 1.5f;

			// Check if there has been enough change in horizontal velocity to make yaw observable
			// Apply hysteresis to check to avoid rapid toggling
			if (_yaw_angle_observable) {
				_yaw_angle_observable = _accel_lpf_NE.norm() > _params.mag_acc_gate;

			} else {
				_yaw_angle_observable = _accel_lpf_NE.norm() > 2.0f * _params.mag_acc_gate;
			}

			_yaw_angle_observable = _yaw_angle_observable && (_control_status.flags.gps || _control_status.flags.ev_pos);

			// check if there is enough yaw rotation to make the mag bias states observable
			if (!_mag_bias_observable && (fabsf(_yaw_rate_lpf_ef) > _params.mag_yaw_rate_gate)) {
				// initial yaw motion is detected
				_mag_bias_observable = true;
				_yaw_delta_ef = 0.0f;
				_time_yaw_started = _imu_sample_delayed.time_us;

			} else if (_mag_bias_observable) {
				// monitor yaw rotation in 45 deg sections.
				// a rotation of 45 deg is sufficient to make the mag bias observable
				if (fabsf(_yaw_delta_ef) > 0.7854f) {
					_time_yaw_started = _imu_sample_delayed.time_us;
					_yaw_delta_ef = 0.0f;
				}

				// require sustained yaw motion of 50% the initial yaw rate threshold
				float min_yaw_change_req =  0.5f * _params.mag_yaw_rate_gate * (1e-6f * (float)(_imu_sample_delayed.time_us - _time_yaw_started));
				_mag_bias_observable = fabsf(_yaw_delta_ef) > min_yaw_change_req;

			} else {
				_mag_bias_observable = false;
			}

			// record the last time that movement was suitable for use of 3-axis magnetometer fusion
			if (_mag_bias_observable || _yaw_angle_observable) {
				_time_last_movement = _imu_sample_delayed.time_us;
			}

			// decide whether 3-axis magnetomer fusion can be used
			bool use_3D_fusion = _control_status.flags.tilt_align && // Use of 3D fusion requires valid tilt estimates
					_control_status.flags.in_air && // don't use when on the ground becasue of magnetic anomalies
					(_flt_mag_align_complete || height_achieved) && // once in-flight field alignment has been performed, ignore relative height
					((_imu_sample_delayed.time_us - _time_last_movement) < 2 * 1000 * 1000); // Using 3-axis fusion for a minimum period after to allow for false negatives

			// perform switch-over
			if (use_3D_fusion) {
				if (!_control_status.flags.mag_3D) {
					if (!_flt_mag_align_complete) {
						// If we are flying a vehicle that flies forward, eg plane, then we can use the GPS course to check and correct the heading
						if (_control_status.flags.fixed_wing && _control_status.flags.in_air) {
							_flt_mag_align_complete = realignYawGPS();

							if (_velpos_reset_request) {
								resetVelocity();
								resetPosition();
								_velpos_reset_request = false;
							}

						} else {
							_flt_mag_align_complete = resetMagHeading(_mag_sample_delayed.mag);
						}

						_control_status.flags.yaw_align = _control_status.flags.yaw_align || _flt_mag_align_complete;

					} else {
						// reset the mag field covariances
						zeroRows(P, 16, 21);
						zeroCols(P, 16, 21);

						// re-instate the last used variances
						for (uint8_t index = 0; index <= 5; index ++) {
							P[index + 16][index + 16] = _saved_mag_variance[index];
						}
					}
				}

				// only use one type of mag fusion at the same time
				_control_status.flags.mag_3D = _flt_mag_align_complete;
				_control_status.flags.mag_hdg = !_control_status.flags.mag_3D;

			} else {
				// save magnetic field state variances for next time
				if (_control_status.flags.mag_3D) {
					for (uint8_t index = 0; index <= 5; index ++) {
						_saved_mag_variance[index] = P[index + 16][index + 16];
					}

					_control_status.flags.mag_3D = false;
				}

				_control_status.flags.mag_hdg = true;
			}

			/*
			Control switch-over between only updating the mag states to updating all states
			When flying as a fixed wing aircraft, a misaligned magnetometer can cause an error in pitch/roll and accel bias estimates.
			When MAG_FUSE_TYPE_AUTOFW is selected and the vehicle is flying as a fixed wing, then magnetometer fusion is only allowed
			to access the magnetic field states.
			*/
			_control_status.flags.update_mag_states_only = (_params.mag_fusion_type == MAG_FUSE_TYPE_AUTOFW)
					&& _control_status.flags.fixed_wing;

			// For the first 5 seconds after switching to 3-axis fusion we allow the magnetic field state estimates to stabilise
			// before they are used to constrain heading drift
			_flt_mag_align_converging = ((_imu_sample_delayed.time_us - _flt_mag_align_start_time) < (uint64_t)5e6);

			if (!_control_status.flags.update_mag_states_only && _control_status_prev.flags.update_mag_states_only) {
				// When re-commencing use of magnetometer to correct vehicle states
				// set the field state variance to the observation variance and zero
				// the covariance terms to allow the field states re-learn rapidly
				zeroRows(P, 16, 21);
				zeroCols(P, 16, 21);

				for (uint8_t index = 0; index <= 5; index ++) {
					P[index + 16][index + 16] = sq(_params.mag_noise);
				}
			}

		} else if (_params.mag_fusion_type == MAG_FUSE_TYPE_HEADING) {
			// always use heading fusion
			_control_status.flags.mag_hdg = true;
			_control_status.flags.mag_3D = false;

		} else if (_params.mag_fusion_type == MAG_FUSE_TYPE_3D) {
			// if transitioning into 3-axis fusion mode, we need to initialise the yaw angle and field states
			if (!_control_status.flags.mag_3D) {
				_control_status.flags.yaw_align = resetMagHeading(_mag_sample_delayed.mag) || _control_status.flags.yaw_align;
			}

			// always use 3-axis mag fusion
			_control_status.flags.mag_hdg = false;
			_control_status.flags.mag_3D = true;

		} else {
			// do no magnetometer fusion at all
			_control_status.flags.mag_hdg = false;
			_control_status.flags.mag_3D = false;
		}

		// if we are using 3-axis magnetometer fusion, but without external aiding, then the declination must be fused as an observation to prevent long term heading drift
		// fusing declination when gps aiding is available is optional, but recommended to prevent problem if the vehicle is static for extended periods of time
		if (_control_status.flags.mag_3D && (!_control_status.flags.gps || (_params.mag_declination_source & MASK_FUSE_DECL))) {
			_control_status.flags.mag_dec = true;

		} else {
			_control_status.flags.mag_dec = false;
		}

		// If the user has selected auto protection against indoor magnetic field errors, only use the magnetomer
		// if a yaw angle relative to true North is required for navigation. If no GPS or other earth frame aiding
		// is available, assume that we are operating indoors and the magnetometer should not be used.
		bool user_selected = (_params.mag_fusion_type == MAG_FUSE_TYPE_INDOOR);
		bool not_using_gps = !(_params.fusion_mode & MASK_USE_GPS) || !_control_status.flags.gps;
		bool not_using_evpos = !(_params.fusion_mode & MASK_USE_EVPOS) || !_control_status.flags.ev_pos;
		bool not_selected_evyaw =  !(_params.fusion_mode & MASK_USE_EVYAW);
		if (user_selected && not_using_gps && not_using_evpos && not_selected_evyaw) {
			_mag_use_inhibit = true;
		} else {
			_mag_use_inhibit = false;
			_mag_use_not_inhibit_us = _imu_sample_delayed.time_us;
		}

		// If magnetomer use has been inhibited continuously then a yaw reset is required for a valid heading
		if (uint32_t(_imu_sample_delayed.time_us - _mag_use_not_inhibit_us) > (uint32_t)5e6) {
			_mag_inhibit_yaw_reset_req = true;
		}

		// fuse magnetometer data using the selected methods
		if (_control_status.flags.mag_3D && _control_status.flags.yaw_align) {
			fuseMag();

			if (_control_status.flags.mag_dec) {
				fuseDeclination();
			}

		} else if (_control_status.flags.mag_hdg && _control_status.flags.yaw_align) {
			// fusion of an Euler yaw angle from either a 321 or 312 rotation sequence
			fuseHeading();

		} else {
			// do no fusion at all
		}
	}
}

void Ekf::controlVelPosFusion()
{
	// if we aren't doing any aiding, fake GPS measurements at the last known position to constrain drift
	// Coincide fake measurements with baro data for efficiency with a minimum fusion rate of 5Hz
	if (!(_params.fusion_mode & MASK_USE_GPS)) {
		_control_status.flags.gps = false;
	}

	if (!_control_status.flags.gps &&
	    !_control_status.flags.opt_flow &&
	    !_control_status.flags.ev_pos &&
	    !(_control_status.flags.fuse_aspd && _control_status.flags.fuse_beta)) {

		// We now need to use a synthetic positon observation to prevent unconstrained drift of the INS states.
		_using_synthetic_position = true;

		// Fuse synthetic position observations every 200msec
		if ((_time_last_imu - _time_last_fake_gps > (uint64_t)2e5) || _fuse_height) {
			// Reset position and velocity states if we re-commence this aiding method
			if ((_time_last_imu - _time_last_fake_gps) > (uint64_t)4e5) {
				resetPosition();
				resetVelocity();
				_fuse_hpos_as_odom = false;

				if (_time_last_fake_gps != 0) {
					ECL_WARN("EKF stopping navigation");
				}

			}

			_fuse_pos = true;
			_fuse_hor_vel = false;
			_fuse_vert_vel = false;
			_time_last_fake_gps = _time_last_imu;

			if (_control_status.flags.in_air && _control_status.flags.tilt_align) {
				_posObsNoiseNE = fmaxf(_params.pos_noaid_noise, _params.gps_pos_noise);

			} else {
				_posObsNoiseNE = 0.5f;
			}

			_vel_pos_innov[0] = 0.0f;
			_vel_pos_innov[1] = 0.0f;
			_vel_pos_innov[2] = 0.0f;
			_vel_pos_innov[3] = _state.pos(0) - _last_known_posNE(0);
			_vel_pos_innov[4] = _state.pos(1) - _last_known_posNE(1);

			// glitch protection is not required so set gate to a large value
			_posInnovGateNE = 100.0f;

		}

	} else {
		_using_synthetic_position = false;
	}

	// Fuse available NED velocity and position data into the main filter
	if (_fuse_height || _fuse_pos || _fuse_hor_vel || _fuse_vert_vel) {
		fuseVelPosHeight();

	}
}

void Ekf::controlAuxVelFusion()
{
	bool data_ready = _auxvel_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_auxvel_sample_delayed);
	bool primary_aiding = _control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.opt_flow;

	if (data_ready && primary_aiding) {
		_fuse_hor_vel = _fuse_vert_vel = _fuse_pos = _fuse_height = false;
		_fuse_hor_vel_aux = true;
		_aux_vel_innov[0] = _state.vel(0) - _auxvel_sample_delayed.velNE(0);
		_aux_vel_innov[1] = _state.vel(1) - _auxvel_sample_delayed.velNE(1);
		_velObsVarNE = _auxvel_sample_delayed.velVarNE;
		_hvelInnovGate = _params.auxvel_gate;
		fuseVelPosHeight();
	}
}
