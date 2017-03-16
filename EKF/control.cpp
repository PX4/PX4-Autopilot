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
#include "mathlib.h"

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
				ECL_INFO("EKF aligned, (pressure height, IMU buf: %i, OBS buf: %i)",(int)_imu_buffer_length,(int)_obs_buffer_length);
			} else if (_control_status.flags.ev_hgt) {
				ECL_INFO("EKF aligned, (EV height, IMU buf: %i, OBS buf: %i)",(int)_imu_buffer_length,(int)_obs_buffer_length);
			} else if (_control_status.flags.gps_hgt) {
				ECL_INFO("EKF aligned, (GPS height, IMU buf: %i, OBS buf: %i)",(int)_imu_buffer_length,(int)_obs_buffer_length);
			} else if (_control_status.flags.rng_hgt) {
				ECL_INFO("EKF aligned, (range height, IMU buf: %i, OBS buf: %i)",(int)_imu_buffer_length,(int)_obs_buffer_length);
			} else {
				ECL_ERR("EKF aligned, (unknown height, IMU buf: %i, OBS buf: %i)",(int)_imu_buffer_length,(int)_obs_buffer_length);
			}

		}

	}

	// check for arrival of new sensor data at the fusion time horizon
	_gps_data_ready = _gps_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_gps_sample_delayed);
	_mag_data_ready = _mag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_mag_sample_delayed);
	_baro_data_ready = _baro_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed);

	// calculate 2,2 element of rotation matrix from sensor frame to earth frame
	_R_rng_to_earth_2_2 = _R_to_earth(2, 0) * _sin_tilt_rng + _R_to_earth(2, 2) * _cos_tilt_rng;
	_range_data_ready = _range_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_range_sample_delayed)
			&& (_R_rng_to_earth_2_2 > 0.7071f);

	_flow_data_ready = _flow_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_flow_sample_delayed)
			&&  (_R_to_earth(2, 2) > 0.7071f);
	_ev_data_ready = _ext_vision_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_ev_sample_delayed);
	_tas_data_ready = _airspeed_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_airspeed_sample_delayed);

	// check for height sensor timeouts and reset and change sensor if necessary
	controlHeightSensorTimeouts();

	// control use of observations for aiding
	controlMagFusion();
	controlExternalVisionFusion();
	controlOpticalFlowFusion();
	controlGpsFusion();
	controlBaroFusion();
	controlRangeFinderFusion();
	controlAirDataFusion();
	controlBetaFusion();

	// for efficiency, fusion of direct state observations for position and velocity is performed sequentially
	// in a single function using sensor data from multiple sources (GPS, external vision, baro, range finder, etc)
	controlVelPosFusion();

	// report dead reckoning if we are no longer fusing measurements that constrain velocity drift
	_is_dead_reckoning = (_time_last_imu - _time_last_pos_fuse > _params.no_aid_timeout_max)
			&& (_time_last_imu - _time_last_vel_fuse > _params.no_aid_timeout_max)
			&& (_time_last_imu - _time_last_of_fuse > _params.no_aid_timeout_max);

}

void Ekf::controlExternalVisionFusion()
{
	// Check for new exernal vision data
	if (_ev_data_ready) {

		// external vision position aiding selection logic
		if ((_params.fusion_mode & MASK_USE_EVPOS) && !_control_status.flags.ev_pos && _control_status.flags.tilt_align && _control_status.flags.yaw_align) {
			// check for a exernal vision measurement that has fallen behind the fusion time horizon
			if (_time_last_imu - _time_last_ext_vision < 2 * EV_MAX_INTERVAL) {
				// turn on use of external vision measurements for position and height
				_control_status.flags.ev_pos = true;
				ECL_INFO("EKF commencing external vision position fusion");
				// turn off other forms of height aiding
				_control_status.flags.baro_hgt = false;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = false;
				// reset the position, height and velocity
				resetPosition();
				resetVelocity();
				resetHeight();
			}
		}

		// external vision yaw aiding selection logic
		if ((_params.fusion_mode & MASK_USE_EVYAW) && !_control_status.flags.ev_yaw && _control_status.flags.tilt_align) {
			// check for a exernal vision measurement that has fallen behind the fusion time horizon
			if (_time_last_imu - _time_last_ext_vision < 2 * EV_MAX_INTERVAL) {
				// reset the yaw angle to the value from the observaton quaternion
				// get the roll, pitch, yaw estimates from the quaternion states
				matrix::Quaternion<float> q_init(_state.quat_nominal(0), _state.quat_nominal(1), _state.quat_nominal(2),
							    _state.quat_nominal(3));
				matrix::Euler<float> euler_init(q_init);

				// get initial yaw from the observation quaternion
				extVisionSample ev_newest = _ext_vision_buffer.get_newest();
				matrix::Quaternion<float> q_obs(ev_newest.quat(0), ev_newest.quat(1), ev_newest.quat(2), ev_newest.quat(3));
				matrix::Euler<float> euler_obs(q_obs);
				euler_init(2) = euler_obs(2);

				// save a copy of the quaternion state for later use in calculating the amount of reset change
				Quaternion quat_before_reset = _state.quat_nominal;

				// calculate initial quaternion states for the ekf
				_state.quat_nominal = Quaternion(euler_init);

				// calculate the amount that the quaternion has changed by
				_state_reset_status.quat_change = _state.quat_nominal * quat_before_reset.inversed();

				// add the reset amount to the output observer buffered data
				outputSample output_states;
				unsigned output_length = _output_buffer.get_length();
				for (unsigned i=0; i < output_length; i++) {
					output_states = _output_buffer.get_from_index(i);
					output_states.quat_nominal *= _state_reset_status.quat_change;
					_output_buffer.push_to_index(i,output_states);
				}

				// apply the change in attitude quaternion to our newest quaternion estimate
				// which was already taken out from the output buffer
				_output_new.quat_nominal *= _state_reset_status.quat_change;

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

		// determine if we should use the height observation
		if (_params.vdist_sensor_type == VDIST_SENSOR_EV) {
			_control_status.flags.baro_hgt = false;
			_control_status.flags.gps_hgt = false;
			_control_status.flags.rng_hgt = false;
			_control_status.flags.ev_hgt = true;
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
		}

		// determine if we should use the yaw observation
		if (_control_status.flags.ev_yaw) {
			fuseHeading();
		}
	}
}

void Ekf::controlOpticalFlowFusion()
{
	// Check for new optical flow data that has fallen behind the fusion time horizon
	if (_flow_data_ready) {

		// optical flow fusion mode selection logic
		if ((_params.fusion_mode & MASK_USE_OF) // optical flow has been selected by the user
				&& !_control_status.flags.opt_flow // we are not yet using flow data
				&& _control_status.flags.tilt_align // we know our tilt attitude
				&& (_time_last_imu - _time_last_hagl_fuse) < 5e5) // we have a valid distance to ground estimate
		{

			// If the heading is not aligned, reset the yaw and magnetic field states
			if (!_control_status.flags.yaw_align) {
				_control_status.flags.yaw_align = resetMagHeading(_mag_sample_delayed.mag);
			}

			// If the heading is valid, start using optical flow aiding
			if (_control_status.flags.yaw_align) {
				// set the flag and reset the fusion timeout
				_control_status.flags.opt_flow = true;
				_time_last_of_fuse = _time_last_imu;

				// if we are not using GPS then the velocity and position states and covariances need to be set
				if (!_control_status.flags.gps) {
					// constrain height above ground to be above minimum possible
					float heightAboveGndEst = fmaxf((_terrain_vpos - _state.pos(2)), _params.rng_gnd_clearance);

					// calculate absolute distance from focal point to centre of frame assuming a flat earth
					float range = heightAboveGndEst / _R_rng_to_earth_2_2;

					if ((range - _params.rng_gnd_clearance) > 0.3f && _flow_sample_delayed.dt > 0.05f) {
						// we should have reliable OF measurements so
						// calculate X and Y body relative velocities from OF measurements
						Vector3f vel_optflow_body;
						vel_optflow_body(0) = - range * _flow_sample_delayed.flowRadXYcomp(1) / _flow_sample_delayed.dt;
						vel_optflow_body(1) =   range * _flow_sample_delayed.flowRadXYcomp(0) / _flow_sample_delayed.dt;
						vel_optflow_body(2) = 0.0f;

						// rotate from body to earth frame
						Vector3f vel_optflow_earth;
						vel_optflow_earth = _R_to_earth * vel_optflow_body;

						// take x and Y components
						_state.vel(0) = vel_optflow_earth(0);
						_state.vel(1) = vel_optflow_earth(1);

					} else {
						_state.vel(0) = 0.0f;
						_state.vel(1) = 0.0f;
					}

					// reset the velocity covariance terms
					zeroRows(P,4,5);
					zeroCols(P,4,5);

					// reset the horizontal velocity variance using the optical flow noise variance
					P[5][5] = P[4][4] = sq(range) * calcOptFlowMeasVar();

					if (!_control_status.flags.in_air) {
						// we are likely starting OF for the first time so reset the horizontal position and vertical velocity states
						_state.pos(0) = 0.0f;
						_state.pos(1) = 0.0f;

					} else {
						// set to the last known position
						_state.pos(0) = _last_known_posNE(0);
						_state.pos(1) = _last_known_posNE(1);

					}

					// reset the corresponding covariances
					// we are by definition at the origin at commencement so variances are also zeroed
					zeroRows(P,7,8);
					zeroCols(P,7,8);

					// align the output observer to the EKF states
					alignOutputFilter();

				}
			}

		} else if (!(_params.fusion_mode & MASK_USE_OF)) {
			_control_status.flags.opt_flow = false;

		}

		// handle the case when we are relying on optical flow fusion and lose it
		if (_control_status.flags.opt_flow && !_control_status.flags.gps) {
			// We are relying on flow aiding to constrain attitude drift so after 5s without aiding we need to do something
			if ((_time_last_imu - _time_last_of_fuse > 5e6)) {
				// Switch to the non-aiding mode, zero the velocity states
				// and set the synthetic position to the current estimate
				_control_status.flags.opt_flow = false;
				_last_known_posNE(0) = _state.pos(0);
				_last_known_posNE(1) = _state.pos(1);
				_state.vel.setZero();

			}
		}

		// fuse the data
		if (_control_status.flags.opt_flow) {
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
		if ((_params.fusion_mode & MASK_USE_GPS) && !_control_status.flags.gps) {
			if (_control_status.flags.tilt_align && _NED_origin_initialised && (_time_last_imu - _last_gps_fail_us > 5e6)) {
				// If the heading is not aligned, reset the yaw and magnetic field states
				if (!_control_status.flags.yaw_align) {
					_control_status.flags.yaw_align = resetMagHeading(_mag_sample_delayed.mag);

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

		// handle the case when we now have GPS, but have not been using it for an extended period
		if (_control_status.flags.gps && !_control_status.flags.opt_flow) {
			// We are relying on GPS aiding to constrain attitude drift so after 7 seconds without aiding we need to do something
			bool do_reset = (_time_last_imu - _time_last_pos_fuse > _params.no_gps_timeout_max) && (_time_last_imu - _time_last_vel_fuse > _params.no_gps_timeout_max);

			// Our position measurments have been rejected for more than 14 seconds
			do_reset |= _time_last_imu - _time_last_pos_fuse > 2 * _params.no_gps_timeout_max;

			if (do_reset) {
				// Reset states to the last GPS measurement
				resetPosition();
				resetVelocity();
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
			Vector3f ang_rate = _imu_sample_delayed.delta_ang * (1.0f/_imu_sample_delayed.delta_ang_dt);
			Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
			Vector3f vel_offset_body = cross_product(ang_rate,pos_offset_body);
			Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
			_gps_sample_delayed.vel -= vel_offset_earth;

			// correct position and height for offset relative to IMU
			Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_gps_sample_delayed.pos(0) -= pos_offset_earth(0);
			_gps_sample_delayed.pos(1) -= pos_offset_earth(1);
			_gps_sample_delayed.hgt += pos_offset_earth(2);

		}

		// Determine if GPS should be used as the height source
		if (((_params.vdist_sensor_type == VDIST_SENSOR_GPS)) && !_gps_hgt_faulty) {
			_control_status.flags.baro_hgt = false;
			_control_status.flags.gps_hgt = true;
			_control_status.flags.rng_hgt = false;
			_control_status.flags.ev_hgt = false;
			_fuse_height = true;

		}
	} else {
		// handle the case where we do not have GPS and have not been using it for an extended period, but are still relying on it
		if ((_time_last_imu - _time_last_gps > 10e6) && (_time_last_imu - _time_last_airspeed > 1e6) && (_time_last_imu - _time_last_optflow > 1e6) && _control_status.flags.gps) {
			// if we don't have a source of aiding to constrain attitude drift,
			// then we need to switch to the non-aiding mode, zero the velocity states
			// and set the synthetic GPS position to the current estimate
			_control_status.flags.gps = false;
			_last_known_posNE(0) = _state.pos(0);
			_last_known_posNE(1) = _state.pos(1);
			_state.vel.setZero();
			ECL_WARN("EKF measurement timeout - stopping navigation");

		}
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

	// check for inertial sensing errors as evidenced by the vertical innovations having the same sign and not stale
	bool bad_vert_accel = (_control_status.flags.baro_hgt && // we can only run this check if vertical position and velocity observations are indepedant
			(_vel_pos_innov[5] * _vel_pos_innov[2] > 0.0f) && // vertical position and velocity sensors are in agreement
			((_imu_sample_delayed.time_us - _baro_sample_delayed.time_us) < 2 * BARO_MAX_INTERVAL) && // vertical position data is fresh
			((_imu_sample_delayed.time_us - _gps_sample_delayed.time_us) < 2 * GPS_MAX_INTERVAL) &&  // vertical velocity data is freshs
			_vel_pos_test_ratio[2] > 1.0f && // vertical velocty innovations have failed innovation consistency checks
			_vel_pos_test_ratio[5] > 1.0f); // vertical position innovations have failed innovation consistency checks

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
	bool continuous_bad_accel_hgt = ((_time_last_imu - _time_good_vert_accel) > BADACC_HGT_RESET);

	// check if height has been inertial deadreckoning for too long
	bool hgt_fusion_timeout = ((_time_last_imu - _time_last_hgt_fuse) > 5e6);

	// reset the vertical position and velocity states
	if ((P[9][9] > sq(_params.hgt_reset_lim)) && (hgt_fusion_timeout || continuous_bad_accel_hgt)) {
		// boolean that indicates we will do a height reset
		bool reset_height = false;

		// handle the case where we are using baro for height
		if (_control_status.flags.baro_hgt) {
			// check if GPS height is available
			gpsSample gps_init = _gps_buffer.get_newest();
			bool gps_hgt_available = ((_time_last_imu - gps_init.time_us) < 2 * GPS_MAX_INTERVAL);
			bool gps_hgt_accurate = (gps_init.vacc < _params.req_vacc);
			baroSample baro_init = _baro_buffer.get_newest();
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
				_gps_hgt_faulty = false;

				// declare the GPS height healthy
				_gps_hgt_faulty = false;

				// reset the height mode
				_control_status.flags.baro_hgt = false;
				_control_status.flags.gps_hgt = true;
				_control_status.flags.rng_hgt = false;
				_control_status.flags.ev_hgt = false;

				// request a reset
				reset_height = true;
				ECL_WARN("EKF baro hgt timeout - reset to GPS");

			} else if (reset_to_baro){
				// set height sensor health
				_baro_hgt_faulty = false;

				// reset the height mode
				_control_status.flags.baro_hgt = true;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = false;
				_control_status.flags.ev_hgt = false;

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
			gpsSample gps_init = _gps_buffer.get_newest();
			bool gps_hgt_available = ((_time_last_imu - gps_init.time_us) < 2 * GPS_MAX_INTERVAL);
			bool gps_hgt_accurate = (gps_init.vacc < _params.req_vacc);

			// check the baro height source for consistency and freshness
			baroSample baro_init = _baro_buffer.get_newest();
			bool baro_data_fresh = ((_time_last_imu - baro_init.time_us) < 2 * BARO_MAX_INTERVAL);
			float baro_innov = _state.pos(2) - (_hgt_sensor_offset - baro_init.hgt + _baro_hgt_offset);
			bool baro_data_consistent = fabsf(baro_innov) < (sq(_params.baro_noise) + P[8][8]) * sq(_params.baro_innov_gate);

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
				_control_status.flags.baro_hgt = true;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = false;
				_control_status.flags.ev_hgt = false;

				// request a reset
				reset_height = true;
				ECL_WARN("EKF gps hgt timeout - reset to baro");

			} else if (reset_to_gps) {
				// set height sensor health
				_gps_hgt_faulty = false;

				// reset the height mode
				_control_status.flags.baro_hgt = false;
				_control_status.flags.gps_hgt = true;
				_control_status.flags.rng_hgt = false;
				_control_status.flags.ev_hgt = false;

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
			rangeSample rng_init = _range_buffer.get_newest();
			bool rng_data_available = ((_time_last_imu - rng_init.time_us) < 2 * RNG_MAX_INTERVAL);

			// check if baro data is available
			baroSample baro_init = _baro_buffer.get_newest();
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
				_control_status.flags.baro_hgt = true;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = false;
				_control_status.flags.ev_hgt = false;

				// request a reset
				reset_height = true;
				ECL_WARN("EKF rng hgt timeout - reset to baro");

			} else if (reset_to_rng) {
				// set height sensor health
				_rng_hgt_faulty = false;

				// reset the height mode
				_control_status.flags.baro_hgt = false;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = true;
				_control_status.flags.ev_hgt = false;

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
			extVisionSample ev_init = _ext_vision_buffer.get_newest();
			bool ev_data_available = ((_time_last_imu - ev_init.time_us) < 2 * EV_MAX_INTERVAL);

			// check if baro data is available
			baroSample baro_init = _baro_buffer.get_newest();
			bool baro_data_available = ((_time_last_imu - baro_init.time_us) < 2 * BARO_MAX_INTERVAL);

			// reset to baro if we have no vision data and baro data is available
			bool reset_to_baro = !ev_data_available && baro_data_available;

			// reset to ev data if it is available
			bool reset_to_ev = ev_data_available;

			if (reset_to_baro) {
				// set height sensor health
				_rng_hgt_faulty = true;
				_baro_hgt_faulty = false;

				// reset the height mode
				_control_status.flags.baro_hgt = true;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = false;
				_control_status.flags.ev_hgt = false;

				// request a reset
				reset_height = true;
				ECL_WARN("EKF ev hgt timeout - reset to baro");

			} else if (reset_to_ev) {
				// reset the height mode
				_control_status.flags.baro_hgt = false;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = false;
				_control_status.flags.ev_hgt = true;

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

void Ekf::controlBaroFusion()
{
	if (_baro_data_ready) {
		// determine if we should use the baro as our height source
		uint64_t last_baro_time_us = _baro_sample_delayed.time_us;
		if (((_params.vdist_sensor_type == VDIST_SENSOR_BARO) || _control_status.flags.baro_hgt) && !_baro_hgt_faulty) {
			_control_status.flags.baro_hgt = true;
			_control_status.flags.gps_hgt = false;
			_control_status.flags.rng_hgt = false;
			_control_status.flags.ev_hgt = false;
			_fuse_height = true;

		}

		// calculate a filtered offset between the baro origin and local NED origin if we are not using the baro as a height reference
		if (!_control_status.flags.baro_hgt) {
			float local_time_step = 1e-6f*(float)(_baro_sample_delayed.time_us - last_baro_time_us);
			local_time_step = math::constrain(local_time_step,0.0f,1.0f);
			last_baro_time_us = _baro_sample_delayed.time_us;
			float offset_rate_correction =  0.1f * (_baro_sample_delayed.hgt - _hgt_sensor_offset) + _state.pos(2) - _baro_hgt_offset;
			_baro_hgt_offset += local_time_step * math::constrain(offset_rate_correction, -0.1f, 0.1f);

		}
	}
}

void Ekf::controlRangeFinderFusion()
{
	// determine if we should use range finder data for height
	if (_range_data_ready) {
		// set the height data source to range if requested
		if ((_params.vdist_sensor_type == VDIST_SENSOR_RANGE) && !_rng_hgt_faulty) {
			_control_status.flags.baro_hgt = false;
			_control_status.flags.gps_hgt = false;
			_control_status.flags.rng_hgt = true;
			_control_status.flags.ev_hgt = false;

		}

		// correct the range data for position offset relative to the IMU
		Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;
		Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
		_range_sample_delayed.rng += pos_offset_earth(2) / _R_rng_to_earth_2_2;

		// only use range finder as a height observation in the main filter if specifically enabled
		if (_control_status.flags.rng_hgt) {
			_fuse_height = true;

		}

	} else if ((_time_last_imu - _time_last_hgt_fuse) > 2 * RNG_MAX_INTERVAL && _control_status.flags.rng_hgt) {
		// If we are supposed to be using range finder data as the primary height sensor, have missed or rejected measurements
		// and are on the ground, then synthesise a measurement at the expected on ground value
		if (!_control_status.flags.in_air) {
			_range_sample_delayed.rng = _params.rng_gnd_clearance;
			_range_sample_delayed.time_us = _imu_sample_delayed.time_us;

		}

		_fuse_height = true;

	}
}

void Ekf::controlAirDataFusion()
{
        // control activation and initialisation/reset of wind states required for airspeed fusion

        // If both airspeed and sideslip fusion have timed out then we no longer have valid wind estimates
        bool airspeed_timed_out = _time_last_imu - _time_last_arsp_fuse > 10e6;
        bool sideslip_timed_out = _time_last_imu - _time_last_beta_fuse > 10e6;
        if (_control_status.flags.wind && airspeed_timed_out && sideslip_timed_out) {
                // if the airspeed or sideslip measurements have timed out for 10 seconds we declare the wind estimate to be invalid
		_control_status.flags.wind = false;

	}

	// Always try to fuse airspeed data if available and we are in flight and the filter is operating in a normal aiding mode
	bool is_aiding = _control_status.flags.gps || _control_status.flags.opt_flow || _control_status.flags.ev_pos;
	if (_tas_data_ready && _control_status.flags.in_air && is_aiding) {
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

        // If both airspeed and sideslip fusion have timed out then we no longer have valid wind estimates
        bool sideslip_timed_out = _time_last_imu - _time_last_beta_fuse > 10e6;
        bool airspeed_timed_out = _time_last_imu - _time_last_arsp_fuse > 10e6;
        if(_control_status.flags.wind && airspeed_timed_out && sideslip_timed_out){
                _control_status.flags.wind = false;

        }

	// Perform synthetic sideslip fusion when in-air and sideslip fuson had been enabled externally in addition to the following criteria:

	// Suffient time has lapsed sice the last fusion
	bool beta_fusion_time_triggered = _time_last_imu - _time_last_beta_fuse > _params.beta_avg_ft_us;

	// The filter is operating in a mode where velocity states can be used
	bool vel_states_active = _control_status.flags.gps || _control_status.flags.opt_flow || _control_status.flags.ev_pos;

	if(beta_fusion_time_triggered && _control_status.flags.fuse_beta && _control_status.flags.in_air && vel_states_active) {
		// If starting wind state estimation, reset the wind states and covariances before fusing any data
		if (!_control_status.flags.wind) {
			// activate the wind states
			_control_status.flags.wind = true;
			// reset the timout timers to prevent repeated resets
			_time_last_beta_fuse = _time_last_imu;
			_time_last_arsp_fuse = _time_last_imu;
			// reset the wind speed states and corresponding covariances
			resetWindStates();
			resetWindCovariance();
		}

                fuseSideslip();
 	}

 	

}

void Ekf::controlMagFusion()
{
	// If we are using external vision data for heading then no magnetometer fusion is used
	if (_control_status.flags.ev_yaw) {
		return;
	}

	// If we are on ground, store the local position and time to use as a reference
	if (!_control_status.flags.in_air) {
		_last_on_ground_posD = _state.pos(2);

	}

	// checs for new magnetometer data tath has fallen beind the fusion time horizon
	if (_mag_data_ready) {

		// Determine if we should use simple magnetic heading fusion which works better when there are large external disturbances
		// or the more accurate 3-axis fusion
		if (_params.mag_fusion_type == MAG_FUSE_TYPE_AUTO) {
			// start 3D fusion if in-flight and height has increased sufficiently
			// to be away from ground magnetic anomalies
			// don't switch back to heading fusion until we are back on the ground
			bool height_achieved = (_last_on_ground_posD - _state.pos(2)) > 1.5f;
			bool use_3D_fusion = _control_status.flags.in_air && (_control_status.flags.mag_3D || height_achieved);

			if (use_3D_fusion && _control_status.flags.tilt_align) {
				// if transitioning into 3-axis fusion mode, we need to initialise the yaw angle and field states
				if (!_control_status.flags.mag_3D) {
					_control_status.flags.yaw_align = resetMagHeading(_mag_sample_delayed.mag);
				}

				// use 3D mag fusion when airborne
				_control_status.flags.mag_hdg = false;
				_control_status.flags.mag_3D = true;

			} else {
				// use heading fusion when on the ground
				_control_status.flags.mag_hdg = true;
				_control_status.flags.mag_3D = false;
			}

		} else if (_params.mag_fusion_type == MAG_FUSE_TYPE_HEADING) {
			// always use heading fusion
			_control_status.flags.mag_hdg = true;
			_control_status.flags.mag_3D = false;

		} else if (_params.mag_fusion_type == MAG_FUSE_TYPE_3D) {
			// if transitioning into 3-axis fusion mode, we need to initialise the yaw angle and field states
			if (!_control_status.flags.mag_3D) {
				_control_status.flags.yaw_align = resetMagHeading(_mag_sample_delayed.mag);
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
		// fusing declination when gps aiding is available is optional, but recommneded to prevent problem if the vehicle is static for extended periods of time
		if (_control_status.flags.mag_3D && (!_control_status.flags.gps || (_params.mag_declination_source & MASK_FUSE_DECL))) {
			_control_status.flags.mag_dec = true;

		} else {
			_control_status.flags.mag_dec = false;
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
	if (!_control_status.flags.gps && !_control_status.flags.opt_flow && !_control_status.flags.ev_pos
	    && ((_time_last_imu - _time_last_fake_gps > 2e5) || _fuse_height)) {
		_fuse_pos = true;
		_time_last_fake_gps = _time_last_imu;

	}

	// Fuse available NED velocity and position data into the main filter
	if (_fuse_height || _fuse_pos || _fuse_hor_vel || _fuse_vert_vel) {
		fuseVelPosHeight();
		_fuse_hor_vel = _fuse_vert_vel = _fuse_pos = _fuse_height = false;

	}
}
