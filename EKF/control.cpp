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

#include "ekf.h"

void Ekf::controlFusionModes()
{
	// Store the status to enable change detection
	_control_status_prev.value = _control_status.value;

	// Get the magnetic declination
	calcMagDeclination();

	// Once the angular uncertainty has reduced sufficiently, initialise the yaw and magnetic field states
	float total_angle_variance = P[0][0] + P[1][1] + P[2][2] + P[3][3];
	if (total_angle_variance < 0.002f && !_control_status.flags.tilt_align) {
		_control_status.flags.tilt_align = true;
		_control_status.flags.yaw_align = resetMagHeading(_mag_sample_delayed.mag);
	}

	// external vision position aiding selection logic
	if ((_params.fusion_mode & MASK_USE_EVPOS) && !_control_status.flags.ev_pos && _control_status.flags.tilt_align && _control_status.flags.yaw_align) {
		// check for a exernal vision measurement that has fallen behind the fusion time horizon
		if (_time_last_imu - _time_last_ext_vision < 2 * EV_MAX_INTERVAL) {
			// turn on use of external vision measurements for position
			_control_status.flags.ev_pos = true;
			// reset the position, height and velocity
			resetPosition();
			resetVelocity();
			resetHeight();
		}
	}

	// external vision yaw aiding selection logic
	if ((_params.fusion_mode & MASK_USE_EVYAW) && !_control_status.flags.ev_yaw && _control_status.flags.tilt_align
			&& (_time_last_imu - _time_last_ext_vision) < 5e5) {
		//TODO
	}

	// optical flow fusion mode selection logic
	// to start using optical flow data we need angular alignment complete, and fresh optical flow and height above terrain data
	if ((_params.fusion_mode & MASK_USE_OF) && !_control_status.flags.opt_flow && _control_status.flags.tilt_align
	    && (_time_last_imu - _time_last_optflow) < 5e5 && (_time_last_imu - _time_last_hagl_fuse) < 5e5) {
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
				float range = heightAboveGndEst / _R_to_earth(2, 2);

				if ((range - _params.rng_gnd_clearance) > 0.3f && _flow_sample_delayed.dt > 0.05f) {
					// we should ahve reliable OF measurements so
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

					// reset the coresponding covariances
					// we are by definition at the origin at commencement so variances are also zeroed
					zeroRows(P,7,8);
					zeroCols(P,7,8);

					// align the output observer to the EKF states
					alignOutputFilter();
				}
			}
		}

	} else if (!(_params.fusion_mode & MASK_USE_OF)) {
		_control_status.flags.opt_flow = false;
	}

	// GPS fusion mode selection logic
	// To start use GPS we need angular alignment completed, the local NED origin set and fresh GPS data
	if ((_params.fusion_mode & MASK_USE_GPS) && !_control_status.flags.gps) {
		if (_control_status.flags.tilt_align && (_time_last_imu - _time_last_gps) < 5e5 && _NED_origin_initialised
		    && (_time_last_imu - _last_gps_fail_us > 5e6)) {
			// If the heading is not aligned, reset the yaw and magnetic field states
			if (!_control_status.flags.yaw_align) {
				_control_status.flags.yaw_align = resetMagHeading(_mag_sample_delayed.mag);
			}

			// If the heading is valid start using gps aiding
			if (_control_status.flags.yaw_align) {
				_control_status.flags.gps = true;
				_time_last_gps = _time_last_imu;

				// if we are not already aiding with optical flow, then we need to reset the position and velocity
				if (!_control_status.flags.opt_flow) {
					_control_status.flags.gps = resetPosition();
					_control_status.flags.gps = resetVelocity();
				}
			}
		}

	}  else if (!(_params.fusion_mode & MASK_USE_GPS)) {
		_control_status.flags.gps = false;
	}

	// handle the case when we are relying on GPS fusion and lose it
	if (_control_status.flags.gps && !_control_status.flags.opt_flow) {
		// We are relying on GPS aiding to constrain attitude drift so after 10 seconds without aiding we need to do something
		if ((_time_last_imu - _time_last_pos_fuse > 10e6) && (_time_last_imu - _time_last_vel_fuse > 10e6)) {
			if (_time_last_imu - _time_last_gps > 5e5) {
				// if we don't have gps then we need to switch to the non-aiding mode, zero the veloity states
				// and set the synthetic GPS position to the current estimate
				_control_status.flags.gps = false;
				_last_known_posNE(0) = _state.pos(0);
				_last_known_posNE(1) = _state.pos(1);
				_state.vel.setZero();

			} else {
				// Reset states to the last GPS measurement
				resetPosition();
				resetVelocity();

				// Reset the timeout counters
				_time_last_pos_fuse = _time_last_imu;
				_time_last_vel_fuse = _time_last_imu;
			}
		}
	}

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
	}

	if ((P[8][8] > sq(_params.hgt_reset_lim)) && ((_time_last_imu - _time_last_hgt_fuse) > 5e6)) {
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
			bool prev_bad_vert_accel = (_time_last_imu - _time_bad_vert_accel < 10E6);

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
				// request a reset
				reset_height = true;
				printf("EKF baro hgt timeout - reset to GPS\n");
			} else if (reset_to_baro){
				// set height sensor health
				_baro_hgt_faulty = false;
				// reset the height mode
				_control_status.flags.baro_hgt = true;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = false;
				// request a reset
				reset_height = true;
				printf("EKF baro hgt timeout - reset to baro\n");
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

			// if we cannot switch to baro and GPs data is available, reset height to GPS
			bool reset_to_gps = !reset_to_baro && gps_hgt_available;

			if (reset_to_baro) {
				// set height sensor health
				_gps_hgt_faulty = true;
				_baro_hgt_faulty = false;
				// reset the height mode
				_control_status.flags.baro_hgt = true;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = false;
				// request a reset
				reset_height = true;
				printf("EKF gps hgt timeout - reset to baro\n");
			} else if (reset_to_gps) {
				// set height sensor health
				_gps_hgt_faulty = false;
				// reset the height mode
				_control_status.flags.baro_hgt = false;
				_control_status.flags.gps_hgt = true;
				_control_status.flags.rng_hgt = false;
				// request a reset
				reset_height = true;
				printf("EKF gps hgt timeout - reset to GPS\n");
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
				// request a reset
				reset_height = true;
				printf("EKF rng hgt timeout - reset to baro\n");
			} else if (reset_to_rng) {
				// set height sensor health
				_rng_hgt_faulty = false;
				// reset the height mode
				_control_status.flags.baro_hgt = false;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = true;
				// request a reset
				reset_height = true;
				printf("EKF rng hgt timeout - reset to rng hgt\n");
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

	// handle the case when we are relying on optical flow fusion and lose it
	if (_control_status.flags.opt_flow && !_control_status.flags.gps) {
		// We are relying on flow aiding to constrain attitude drift so after 5s without aiding we need to do something
		if ((_time_last_imu - _time_last_of_fuse > 5e6)) {
			// Switch to the non-aiding mode, zero the veloity states
			// and set the synthetic position to the current estimate
			_control_status.flags.opt_flow = false;
			_last_known_posNE(0) = _state.pos(0);
			_last_known_posNE(1) = _state.pos(1);
			_state.vel.setZero();
		}
	}

	// Determine if we should use simple magnetic heading fusion which works better when there are large external disturbances
	// or the more accurate 3-axis fusion
	if (_params.mag_fusion_type == MAG_FUSE_TYPE_AUTO) {

	if (_control_status.flags.in_air) {
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

	// Control the soure of height measurements for the main filter
	if ((_params.vdist_sensor_type == VDIST_SENSOR_BARO && !_baro_hgt_faulty) || _control_status.flags.baro_hgt) {
		_control_status.flags.baro_hgt = true;
		_control_status.flags.gps_hgt = false;
		_control_status.flags.rng_hgt = false;

	} else if ((_params.vdist_sensor_type == VDIST_SENSOR_GPS && !_gps_hgt_faulty) || _control_status.flags.gps_hgt) {
		_control_status.flags.baro_hgt = false;
		_control_status.flags.gps_hgt = true;
		_control_status.flags.rng_hgt = false;

	} else if (_params.vdist_sensor_type == VDIST_SENSOR_RANGE && !_rng_hgt_faulty) {
		_control_status.flags.baro_hgt = false;
		_control_status.flags.gps_hgt = false;
		_control_status.flags.rng_hgt = true;
	}

	// if the airspeed measurements have timed out for 10 seconds we declare the wind estimate to be invalid
	if (_time_last_imu - _time_last_arsp_fuse > 10e6 || _time_last_arsp_fuse == 0) {
		_control_status.flags.wind = false;
	} else {
		_control_status.flags.wind = true;
	}

}
