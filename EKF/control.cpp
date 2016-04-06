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
	// Determine the vehicle status
	calculateVehicleStatus();

	// Get the magnetic declination
	calcMagDeclination();

	// Check for tilt convergence during initial alignment
	// filter the tilt error vector using a 1 sec time constant LPF
	float filt_coef = 1.0f * _imu_sample_delayed.delta_ang_dt;
	_tilt_err_length_filt = filt_coef * _tilt_err_vec.norm() + (1.0f - filt_coef) * _tilt_err_length_filt;

	// Once the tilt error has reduced sufficiently, initialise the yaw and magnetic field states
	if (_tilt_err_length_filt < 0.005f && !_control_status.flags.tilt_align) {
		_control_status.flags.tilt_align = true;
		_control_status.flags.yaw_align = resetMagHeading(_mag_sample_delayed.mag);
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

			// if we are not using GPS and are in air, then we need to reset the velocity to be consistent with the optical flow reading
			if (!_control_status.flags.gps) {
				// calculate the rotation matrix from body to earth frame
				matrix::Dcm<float> body_to_earth(_state.quat_nominal);

				// constrain height above ground to be above minimum possible
				float heightAboveGndEst = fmaxf((_terrain_vpos - _state.pos(2)), _params.rng_gnd_clearance);

				// calculate absolute distance from focal point to centre of frame assuming a flat earth
				float range = heightAboveGndEst / body_to_earth(2, 2);

				if (_in_air && (range - _params.rng_gnd_clearance) > 0.3f && _flow_sample_delayed.dt > 0.05f) {
					// calculate X and Y body relative velocities from OF measurements
					Vector3f vel_optflow_body;
					vel_optflow_body(0) = - range * _flow_sample_delayed.flowRadXYcomp(1) / _flow_sample_delayed.dt;
					vel_optflow_body(1) =   range * _flow_sample_delayed.flowRadXYcomp(0) / _flow_sample_delayed.dt;
					vel_optflow_body(2) = 0.0f;

					// rotate from body to earth frame
					Vector3f vel_optflow_earth;
					vel_optflow_earth = body_to_earth * vel_optflow_body;

					// take x and Y components
					_state.vel(0) = vel_optflow_earth(0);
					_state.vel(1) = vel_optflow_earth(1);

				} else {
					_state.vel.setZero();
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
			}
		}
	}

	/*
	 * Handle the case where we have not fused height measurements recently and
	 * uncertainty exceeds the max allowable. Reset using the best available height
	 * measurement source, continue using it after the reset and declare the current
	 * source failed if we have switched.
	*/
	if ((P[8][8] > sq(_params.hgt_reset_lim)) && ((_time_last_imu - _time_last_hgt_fuse) > 5e6)) {
		// handle the case where we are using baro for height
		if (_control_status.flags.baro_hgt) {
			// check if GPS height is available
			gpsSample gps_init = _gps_buffer.get_newest();
			bool gps_hgt_available = ((_time_last_imu - gps_init.time_us) < 2 * GPS_MAX_INTERVAL);
			bool gps_hgt_accurate = (gps_init.vacc < _params.req_vacc);
			baroSample baro_init = _baro_buffer.get_newest();
			bool baro_hgt_available = ((_time_last_imu - baro_init.time_us) < 2 * BARO_MAX_INTERVAL);

			// use the gps if it is accurate or there is no baro data available
			if (gps_hgt_available && (gps_hgt_accurate || !baro_hgt_available)) {
				// declare the baro as unhealthy
				_baro_hgt_faulty = true;
				// set the height mode to the GPS
				_control_status.flags.baro_hgt = false;
				_control_status.flags.gps_hgt = true;
				_control_status.flags.rng_hgt = false;
				// adjust the height offset so we can use the GPS
				_hgt_sensor_offset = _state.pos(2) + gps_init.hgt - _gps_alt_ref;
				if (!baro_hgt_available) {
					printf("EKF baro hgt timeout - switching to gps\n");
				}
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

			// if baro data is consistent and fresh or GPS height is unavailable or inaccurate, we switch to baro for height
			if ((baro_data_consistent && baro_data_fresh) || !gps_hgt_available || !gps_hgt_accurate) {
				// declare the GPS height unhealthy
				_gps_hgt_faulty = true;
				// set the height mode to the baro
				_control_status.flags.baro_hgt = true;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = false;
				printf("EKF gps hgt timeout - switching to baro\n");
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
			// check if baro data is consistent
			float baro_innov = _state.pos(2) - (_hgt_sensor_offset - baro_init.hgt + _baro_hgt_offset);
			bool baro_data_consistent = sq(baro_innov) < (sq(_params.baro_noise) + P[8][8]) * sq(_params.baro_innov_gate);
			// switch to baro if necessary or preferable
			bool switch_to_baro = (!rng_data_available && baro_data_available) || (baro_data_consistent && baro_data_available);

			if (switch_to_baro) {
				// declare the range finder height unhealthy
				_rng_hgt_faulty = true;
				// set the height mode to the baro
				_control_status.flags.baro_hgt = true;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = false;
				printf("EKF rng hgt timeout - switching to baro\n");
			}
		}

		// Reset vertical position and velocity states to the last measurement
		resetHeight();
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
		if (!_control_status.flags.armed) {
			// use heading fusion for initial startup
			_control_status.flags.mag_hdg = true;
			_control_status.flags.mag_2D = false;
			_control_status.flags.mag_3D = false;

		} else {
			if (_control_status.flags.in_air) {
				// if transitioning into 3-axis fusion mode, we need to initialise the yaw angle and field states
				if (!_control_status.flags.mag_3D) {
					_control_status.flags.yaw_align = resetMagHeading(_mag_sample_delayed.mag);
				}

				// use 3D mag fusion when airborne
				_control_status.flags.mag_hdg = false;
				_control_status.flags.mag_2D = false;
				_control_status.flags.mag_3D = true;

			} else {
				// use heading fusion when on the ground
				_control_status.flags.mag_hdg = true;
				_control_status.flags.mag_2D = false;
				_control_status.flags.mag_3D = false;
			}
		}

	} else if (_params.mag_fusion_type == MAG_FUSE_TYPE_HEADING) {
		// always use heading fusion
		_control_status.flags.mag_hdg = true;
		_control_status.flags.mag_2D = false;
		_control_status.flags.mag_3D = false;

	} else if (_params.mag_fusion_type == MAG_FUSE_TYPE_2D) {
		// always use 2D mag fusion
		_control_status.flags.mag_hdg = false;
		_control_status.flags.mag_2D = true;
		_control_status.flags.mag_3D = false;

	} else if (_params.mag_fusion_type == MAG_FUSE_TYPE_3D) {
		// if transitioning into 3-axis fusion mode, we need to initialise the yaw angle and field states
		if (!_control_status.flags.mag_3D) {
			_control_status.flags.yaw_align = resetMagHeading(_mag_sample_delayed.mag);
		}

		// always use 3-axis mag fusion
		_control_status.flags.mag_hdg = false;
		_control_status.flags.mag_2D = false;
		_control_status.flags.mag_3D = true;

	} else {
		// do no magnetometer fusion at all
		_control_status.flags.mag_hdg = false;
		_control_status.flags.mag_2D = false;
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

	// Placeholder for control of wind velocity states estimation
	// TODO add methods for true airspeed and/or sidelsip fusion or some type of drag force measurement
	if (false) {
		_control_status.flags.wind = false;
	}

	// Store the status to enable change detection
	_control_status_prev.value = _control_status.value;
}

void Ekf::calculateVehicleStatus()
{
	// determine if the vehicle is armed
	_control_status.flags.armed = _vehicle_armed;

	// record vertical position whilst disarmed to use as a height change reference
	if (!_control_status.flags.armed) {
		_last_disarmed_posD = _state.pos(2);
	}

	// Transition to in-air occurs when armed and when altitude has increased sufficiently from the altitude at arming
	bool in_air = _control_status.flags.armed && (_state.pos(2) - _last_disarmed_posD) < -1.0f;

	if (!_control_status.flags.in_air && in_air) {
		_control_status.flags.in_air = true;
	}

	// Transition to on-ground occurs when disarmed or if the land detector indicated landed state
	if (_control_status.flags.in_air && (!_control_status.flags.armed || !_in_air)) {
		_control_status.flags.in_air = false;
	}
}
