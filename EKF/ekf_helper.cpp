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
 * @file ekf_helper.cpp
 * Definition of ekf helper functions.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#include "ekf.h"

#include <ecl.h>
#include <mathlib/mathlib.h>
#include <cstdlib>

// Reset the velocity states. If we have a recent and valid
// gps measurement then use for velocity initialisation
bool Ekf::resetVelocity()
{
	// used to calculate the velocity change due to the reset
	Vector3f vel_before_reset = _state.vel;

	// reset EKF states
	if (_control_status.flags.gps) {
		// this reset is only called if we have new gps data at the fusion time horizon
		_state.vel = _gps_sample_delayed.vel;

		// use GPS accuracy to reset variances
		setDiag(P, 4, 6, sq(_gps_sample_delayed.sacc));

	} else if (_control_status.flags.opt_flow) {
		// constrain height above ground to be above minimum possible
		float heightAboveGndEst = fmaxf((_terrain_vpos - _state.pos(2)), _params.rng_gnd_clearance);

		// calculate absolute distance from focal point to centre of frame assuming a flat earth
		float range = heightAboveGndEst / _R_rng_to_earth_2_2;

		if ((range - _params.rng_gnd_clearance) > 0.3f && _flow_sample_delayed.dt > 0.05f) {
			// we should have reliable OF measurements so
			// calculate X and Y body relative velocities from OF measurements
			Vector3f vel_optflow_body;
			vel_optflow_body(0) = - range * _flowRadXYcomp(1) / _flow_sample_delayed.dt;
			vel_optflow_body(1) =   range * _flowRadXYcomp(0) / _flow_sample_delayed.dt;
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
		zeroRows(P, 4, 5);
		zeroCols(P, 4, 5);

		// reset the horizontal velocity variance using the optical flow noise variance
		P[5][5] = P[4][4] = sq(range) * calcOptFlowMeasVar();

	} else if (_control_status.flags.ev_pos) {
		_state.vel.setZero();
		zeroOffDiag(P, 4, 6);

	} else {
		// Used when falling back to non-aiding mode of operation
		_state.vel(0) = 0.0f;
		_state.vel(1) = 0.0f;
		setDiag(P, 4, 5, 25.0f);
	}

	// calculate the change in velocity and apply to the output predictor state history
	const Vector3f velocity_change = _state.vel - vel_before_reset;

	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].vel += velocity_change;
	}

	// apply the change in velocity to our newest velocity estimate
	// which was already taken out from the output buffer
	_output_new.vel += velocity_change;

	// capture the reset event
	_state_reset_status.velNE_change(0) = velocity_change(0);
	_state_reset_status.velNE_change(1) = velocity_change(1);
	_state_reset_status.velD_change = velocity_change(2);
	_state_reset_status.velNE_counter++;
	_state_reset_status.velD_counter++;

	return true;
}

// Reset position states. If we have a recent and valid
// gps measurement then use for position initialisation
bool Ekf::resetPosition()
{
	// used to calculate the position change due to the reset
	Vector2f posNE_before_reset;
	posNE_before_reset(0) = _state.pos(0);
	posNE_before_reset(1) = _state.pos(1);

	// let the next odometry update know that the previous value of states cannot be used to calculate the change in position
	_hpos_prev_available = false;

	if (_control_status.flags.gps) {
		// this reset is only called if we have new gps data at the fusion time horizon
		_state.pos(0) = _gps_sample_delayed.pos(0);
		_state.pos(1) = _gps_sample_delayed.pos(1);

		// use GPS accuracy to reset variances
		setDiag(P, 7, 8, sq(_gps_sample_delayed.hacc));

	} else if (_control_status.flags.ev_pos) {
		// this reset is only called if we have new ev data at the fusion time horizon
		_state.pos(0) = _ev_sample_delayed.posNED(0);
		_state.pos(1) = _ev_sample_delayed.posNED(1);

		// use EV accuracy to reset variances
		setDiag(P, 7, 8, sq(_ev_sample_delayed.posErr));

	} else if (_control_status.flags.opt_flow) {
		if (!_control_status.flags.in_air) {
			// we are likely starting OF for the first time so reset the horizontal position
			_state.pos(0) = 0.0f;
			_state.pos(1) = 0.0f;

		} else {
			// set to the last known position
			_state.pos(0) = _last_known_posNE(0);
			_state.pos(1) = _last_known_posNE(1);

		}

		// estimate is relative to initial positon in this mode, so we start with zero error.
		zeroCols(P,7,8);
		zeroRows(P,7,8);

	} else {
		// Used when falling back to non-aiding mode of operation
		_state.pos(0) = _last_known_posNE(0);
		_state.pos(1) = _last_known_posNE(1);
		setDiag(P, 7, 8, sq(_params.pos_noaid_noise));
	}

	// calculate the change in position and apply to the output predictor state history
	const Vector2f posNE_change{_state.pos(0) - posNE_before_reset(0), _state.pos(1) - posNE_before_reset(1)};

	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].pos(0) += posNE_change(0);
		_output_buffer[index].pos(1) += posNE_change(1);
	}

	// apply the change in position to our newest position estimate
	// which was already taken out from the output buffer
	_output_new.pos(0) += posNE_change(0);
	_output_new.pos(1) += posNE_change(1);

	// capture the reset event
	_state_reset_status.posNE_change = posNE_change;
	_state_reset_status.posNE_counter++;

	return true;
}

// Reset height state using the last height measurement
void Ekf::resetHeight()
{
	// Get the most recent GPS data
	const gpsSample &gps_newest = _gps_buffer.get_newest();

	// store the current vertical position and velocity for reference so we can calculate and publish the reset amount
	float old_vert_pos = _state.pos(2);
	bool vert_pos_reset = false;
	float old_vert_vel = _state.vel(2);
	bool vert_vel_reset = false;

	// reset the vertical position
	if (_control_status.flags.rng_hgt) {
		rangeSample range_newest = _range_buffer.get_newest();

		if (_time_last_imu - range_newest.time_us < 2 * RNG_MAX_INTERVAL) {
			// correct the range data for position offset relative to the IMU
			Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;
			Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			range_newest.rng += pos_offset_earth(2) / _R_rng_to_earth_2_2;
			// calculate the new vertical position using range sensor
			float new_pos_down = _hgt_sensor_offset - range_newest.rng * _R_rng_to_earth_2_2;

			// update the state and assoicated variance
			_state.pos(2) = new_pos_down;

			// reset the associated covariance values
			zeroRows(P, 9, 9);
			zeroCols(P, 9, 9);

			// the state variance is the same as the observation
			P[9][9] = sq(_params.range_noise);

			vert_pos_reset = true;

			// reset the baro offset which is subtracted from the baro reading if we need to use it as a backup
			const baroSample &baro_newest = _baro_buffer.get_newest();
			_baro_hgt_offset = baro_newest.hgt + _state.pos(2);

		} else {
			// TODO: reset to last known range based estimate
		}


	} else if (_control_status.flags.baro_hgt) {
		// initialize vertical position with newest baro measurement
		const baroSample &baro_newest = _baro_buffer.get_newest();

		if (_time_last_imu - baro_newest.time_us < 2 * BARO_MAX_INTERVAL) {
			_state.pos(2) = _hgt_sensor_offset - baro_newest.hgt + _baro_hgt_offset;

			// reset the associated covariance values
			zeroRows(P, 9, 9);
			zeroCols(P, 9, 9);

			// the state variance is the same as the observation
			P[9][9] = sq(_params.baro_noise);

			vert_pos_reset = true;

		} else {
			// TODO: reset to last known baro based estimate
		}

	} else if (_control_status.flags.gps_hgt) {
		// initialize vertical position and velocity with newest gps measurement
		if (_time_last_imu - gps_newest.time_us < 2 * GPS_MAX_INTERVAL) {
			_state.pos(2) = _hgt_sensor_offset - gps_newest.hgt + _gps_alt_ref;

			// reset the associated covarince values
			zeroRows(P, 9, 9);
			zeroCols(P, 9, 9);

			// the state variance is the same as the observation
			P[9][9] = sq(gps_newest.hacc);

			vert_pos_reset = true;

			// reset the baro offset which is subtracted from the baro reading if we need to use it as a backup
			const baroSample &baro_newest = _baro_buffer.get_newest();
			_baro_hgt_offset = baro_newest.hgt + _state.pos(2);

		} else {
			// TODO: reset to last known gps based estimate
		}

	} else if (_control_status.flags.ev_hgt) {
		// initialize vertical position with newest measurement
		const extVisionSample &ev_newest = _ext_vision_buffer.get_newest();

		// use the most recent data if it's time offset from the fusion time horizon is smaller
		int32_t dt_newest = ev_newest.time_us - _imu_sample_delayed.time_us;
		int32_t dt_delayed = _ev_sample_delayed.time_us - _imu_sample_delayed.time_us;

		vert_pos_reset = true;

		if (std::abs(dt_newest) < std::abs(dt_delayed)) {
			_state.pos(2) = ev_newest.posNED(2);

		} else {
			_state.pos(2) = _ev_sample_delayed.posNED(2);
		}

	}

	// reset the vertical velocity covariance values
	zeroRows(P, 6, 6);
	zeroCols(P, 6, 6);

	// reset the vertical velocity state
	if (_control_status.flags.gps && (_time_last_imu - gps_newest.time_us < 2 * GPS_MAX_INTERVAL)) {
		// If we are using GPS, then use it to reset the vertical velocity
		_state.vel(2) = gps_newest.vel(2);

		// the state variance is the same as the observation
		P[6][6] = sq(1.5f * gps_newest.sacc);

	} else {
		// we don't know what the vertical velocity is, so set it to zero
		_state.vel(2) = 0.0f;

		// Set the variance to a value large enough to allow the state to converge quickly
		// that does not destabilise the filter
		P[6][6] = 10.0f;

	}

	vert_vel_reset = true;

	// store the reset amount and time to be published
	if (vert_pos_reset) {
		_state_reset_status.posD_change = _state.pos(2) - old_vert_pos;
		_state_reset_status.posD_counter++;
	}

	if (vert_vel_reset) {
		_state_reset_status.velD_change = _state.vel(2) - old_vert_vel;
		_state_reset_status.velD_counter++;
	}

	// apply the change in height / height rate to our newest height / height rate estimate
	// which have already been taken out from the output buffer
	if (vert_pos_reset) {
		_output_new.pos(2) += _state_reset_status.posD_change;
	}

	if (vert_vel_reset) {
		_output_new.vel(2) += _state_reset_status.velD_change;
	}

	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		if (vert_pos_reset) {
			_output_buffer[i].pos(2) += _state_reset_status.posD_change;
			_output_vert_buffer[i].vel_d_integ += _state_reset_status.posD_change;
		}

		if (vert_vel_reset) {
			_output_buffer[i].vel(2) += _state_reset_status.velD_change;
			_output_vert_buffer[i].vel_d += _state_reset_status.velD_change;
		}
	}

	// add the reset amount to the output observer vertical position state
	if (vert_pos_reset) {
		_output_vert_delayed.vel_d_integ = _state.pos(2);
		_output_vert_new.vel_d_integ = _state.pos(2);
	}

	if (vert_vel_reset) {
		_output_vert_delayed.vel_d = _state.vel(2);
		_output_vert_new.vel_d = _state.vel(2);
	}
}

// align output filter states to match EKF states at the fusion time horizon
void Ekf::alignOutputFilter()
{
	// calculate the quaternion delta between the output and EKF quaternions at the EKF fusion time horizon
	Quatf q_delta = _state.quat_nominal.inversed() * _output_sample_delayed.quat_nominal;
	q_delta.normalize();

	// calculate the velocity and posiiton deltas between the output and EKF at the EKF fusion time horizon
	const Vector3f vel_delta = _state.vel - _output_sample_delayed.vel;
	const Vector3f pos_delta = _state.pos - _output_sample_delayed.pos;

	// loop through the output filter state history and add the deltas
	// Note q1 *= q2 is equivalent to q1 = q2 * q1
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].quat_nominal *= q_delta;
		_output_buffer[i].quat_nominal.normalize();
		_output_buffer[i].vel += vel_delta;
		_output_buffer[i].pos += pos_delta;
	}
}

// Do a forced re-alignment of the yaw angle to align with the horizontal velocity vector from the GPS.
// It is used to align the yaw angle after launch or takeoff for fixed wing vehicle only.
bool Ekf::realignYawGPS()
{
	// Need at least 5 m/s of GPS horizontal speed and ratio of velocity error to velocity < 0.15  for a reliable alignment
	float gpsSpeed = sqrtf(sq(_gps_sample_delayed.vel(0)) + sq(_gps_sample_delayed.vel(1)));

	if ((gpsSpeed > 5.0f) && (_gps_sample_delayed.sacc < (0.15f * gpsSpeed))) {
		// check for excessive GPS velocity innovations
		bool badVelInnov = ((_vel_pos_test_ratio[0] > 1.0f) || (_vel_pos_test_ratio[1] > 1.0f)) && _control_status.flags.gps;

		// calculate GPS course over ground angle
		float gpsCOG = atan2f(_gps_sample_delayed.vel(1), _gps_sample_delayed.vel(0));

		// calculate course yaw angle
		float ekfGOG = atan2f(_state.vel(1), _state.vel(0));

		// Check the EKF and GPS course over ground for consistency
		float courseYawError = gpsCOG - ekfGOG;

		// If the angles disagree and horizontal GPS velocity innovations are large or no previous yaw alignment, we declare the magnetic yaw as bad
		bool badYawErr = fabsf(courseYawError) > 0.5f;
		bool badMagYaw = (badYawErr && badVelInnov);

		if (badMagYaw) {
			_num_bad_flight_yaw_events ++;
		}

		// correct yaw angle using GPS ground course if compass yaw bad or yaw is previously not aligned
		if (badMagYaw || !_control_status.flags.yaw_align) {
			ECL_WARN("EKF bad yaw corrected using GPS course");

			// declare the magnetomer as failed if a bad yaw has occurred more than once
			if (_flt_mag_align_complete && (_num_bad_flight_yaw_events >= 2) && !_control_status.flags.mag_fault) {
				ECL_WARN("EKF stopping magnetometer use");
				_control_status.flags.mag_fault = true;
			}

			// save a copy of the quaternion state for later use in calculating the amount of reset change
			Quatf quat_before_reset = _state.quat_nominal;

			// calculate the variance for the rotation estimate expressed as a rotation vector
			// this will be used later to reset the quaternion state covariances
			Vector3f angle_err_var_vec = calcRotVecVariances();

			// update transformation matrix from body to world frame using the current state estimate
			_R_to_earth = quat_to_invrotmat(_state.quat_nominal);

			// get quaternion from existing filter states and calculate roll, pitch and yaw angles
			Eulerf euler321(_state.quat_nominal);

			// apply yaw correction
			if (!_flt_mag_align_complete) {
				// This is our first flight aligment so we can assume that the recent change in velocity has occurred due to a
				// forward direction takeoff or launch and therefore the inertial and GPS ground course discrepancy is due to yaw error
				euler321(2) += courseYawError;
				_flt_mag_align_complete = true;

			} else if (_control_status.flags.wind) {
				// we have previously aligned yaw in-flight and have wind estimates so set the yaw such that the vehicle nose is
				// aligned with the wind relative GPS velocity vector
				euler321(2) = atan2f((_gps_sample_delayed.vel(1) - _state.wind_vel(1)),
						     (_gps_sample_delayed.vel(0) - _state.wind_vel(0)));

			} else {
				// we don't have wind estimates, so align yaw to the GPS velocity vector
				euler321(2) = atan2f(_gps_sample_delayed.vel(1), _gps_sample_delayed.vel(0));

			}

			// calculate new filter quaternion states using corected yaw angle
			_state.quat_nominal = Quatf(euler321);

			// If heading was bad, then we alos need to reset the velocity and position states
			_velpos_reset_request = badMagYaw;

			// update transformation matrix from body to world frame using the current state estimate
			_R_to_earth = quat_to_invrotmat(_state.quat_nominal);

			// Use the last magnetometer measurements to reset the field states
			_state.mag_B.zero();
			_state.mag_I = _R_to_earth * _mag_sample_delayed.mag;

			// use the combined EKF and GPS speed variance to calculate a rough estimate of the yaw error after alignment
			float SpdErrorVariance = sq(_gps_sample_delayed.sacc) + P[4][4] + P[5][5];
			float sineYawError = math::constrain(sqrtf(SpdErrorVariance) / gpsSpeed, 0.0f, 1.0f);
			angle_err_var_vec(2) = sq(asinf(sineYawError));

			// reset the quaternion covariances using the rotation vector variances
			initialiseQuatCovariances(angle_err_var_vec);

			// reset the corresponding rows and columns in the covariance matrix and set the variances on the magnetic field states to the measurement variance
			zeroRows(P, 16, 21);
			zeroCols(P, 16, 21);

			for (uint8_t index = 16; index <= 21; index ++) {
				P[index][index] = sq(_params.mag_noise);
			}

			// record the start time for the magnetic field alignment
			_flt_mag_align_start_time = _imu_sample_delayed.time_us;
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

			return true;

		} else {
			// align mag states only

			// calculate initial earth magnetic field states
			_state.mag_I = _R_to_earth * _mag_sample_delayed.mag;

			// reset the corresponding rows and columns in the covariance matrix and set the variances on the magnetic field states to the measurement variance
			zeroRows(P, 16, 21);
			zeroCols(P, 16, 21);

			for (uint8_t index = 16; index <= 21; index ++) {
				P[index][index] = sq(_params.mag_noise);
			}

			// record the start time for the magnetic field alignment
			_flt_mag_align_start_time = _imu_sample_delayed.time_us;

			return true;
		}

	} else {
		// attempt a normal alignment using the magnetometer
		return resetMagHeading(_mag_sample_delayed.mag);

	}
}

// Reset heading and magnetic field states
bool Ekf::resetMagHeading(Vector3f &mag_init)
{
	// prevent a reset being performed more than once on the same frame
	if (_imu_sample_delayed.time_us == _flt_mag_align_start_time) {
		return true;
	}

	if (_params.mag_fusion_type >= MAG_FUSE_TYPE_NONE) {
		// do not use the magnetomer and deactivate magnetic field states
		zeroRows(P, 16, 21);
		zeroCols(P, 16, 21);
		_control_status.flags.mag_hdg = false;
		_control_status.flags.mag_3D = false;
		return false;
	}

	// save a copy of the quaternion state for later use in calculating the amount of reset change
	Quatf quat_before_reset = _state.quat_nominal;
	Quatf quat_after_reset = _state.quat_nominal;

	// calculate the variance for the rotation estimate expressed as a rotation vector
	// this will be used later to reset the quaternion state covariances
	Vector3f angle_err_var_vec = calcRotVecVariances();

	// update transformation matrix from body to world frame using the current estimate
	_R_to_earth = quat_to_invrotmat(_state.quat_nominal);

	// calculate the initial quaternion
	// determine if a 321 or 312 Euler sequence is best
	if (fabsf(_R_to_earth(2, 0)) < fabsf(_R_to_earth(2, 1))) {
		// use a 321 sequence

		// rotate the magnetometer measurement into earth frame
		Eulerf euler321(_state.quat_nominal);

		// Set the yaw angle to zero and calculate the rotation matrix from body to earth frame
		euler321(2) = 0.0f;
		Dcmf R_to_earth(euler321);

		// calculate the observed yaw angle
		if (_control_status.flags.ev_yaw) {
			// convert the observed quaternion to a rotation matrix
			Dcmf R_to_earth_ev(_ev_sample_delayed.quat);	// transformation matrix from body to world frame
			// calculate the yaw angle for a 312 sequence
			euler321(2) = atan2f(R_to_earth_ev(1, 0), R_to_earth_ev(0, 0));

		} else if (_params.mag_fusion_type <= MAG_FUSE_TYPE_AUTOFW) {
			// rotate the magnetometer measurements into earth frame using a zero yaw angle
			Vector3f mag_earth_pred = R_to_earth * _mag_sample_delayed.mag;
			// the angle of the projection onto the horizontal gives the yaw angle
			euler321(2) = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + _mag_declination;

		} else {
			// there is no yaw observation
			return false;
		}

		// calculate initial quaternion states for the ekf
		// we don't change the output attitude to avoid jumps
		quat_after_reset = Quatf(euler321);

	} else {
		// use a 312 sequence

		// Calculate the 312 sequence euler angles that rotate from earth to body frame
		// See http://www.atacolorado.com/eulersequences.doc
		Vector3f euler312;
		euler312(0) = atan2f(-_R_to_earth(0, 1), _R_to_earth(1, 1));  // first rotation (yaw)
		euler312(1) = asinf(_R_to_earth(2, 1)); // second rotation (roll)
		euler312(2) = atan2f(-_R_to_earth(2, 0), _R_to_earth(2, 2));  // third rotation (pitch)

		// Set the first rotation (yaw) to zero and calculate the rotation matrix from body to earth frame
		euler312(0) = 0.0f;

		// Calculate the body to earth frame rotation matrix from the euler angles using a 312 rotation sequence
		float c2 = cosf(euler312(2));
		float s2 = sinf(euler312(2));
		float s1 = sinf(euler312(1));
		float c1 = cosf(euler312(1));
		float s0 = sinf(euler312(0));
		float c0 = cosf(euler312(0));

		Dcmf R_to_earth;
		R_to_earth(0, 0) = c0 * c2 - s0 * s1 * s2;
		R_to_earth(1, 1) = c0 * c1;
		R_to_earth(2, 2) = c2 * c1;
		R_to_earth(0, 1) = -c1 * s0;
		R_to_earth(0, 2) = s2 * c0 + c2 * s1 * s0;
		R_to_earth(1, 0) = c2 * s0 + s2 * s1 * c0;
		R_to_earth(1, 2) = s0 * s2 - s1 * c0 * c2;
		R_to_earth(2, 0) = -s2 * c1;
		R_to_earth(2, 1) = s1;

		// calculate the observed yaw angle
		if (_control_status.flags.ev_yaw) {
			// convert the observed quaternion to a rotation matrix
			Dcmf R_to_earth_ev(_ev_sample_delayed.quat);	// transformation matrix from body to world frame
			// calculate the yaw angle for a 312 sequence
			euler312(0) = atan2f(-R_to_earth_ev(0, 1), R_to_earth_ev(1, 1));

		} else if (_params.mag_fusion_type <= MAG_FUSE_TYPE_AUTOFW) {
			// rotate the magnetometer measurements into earth frame using a zero yaw angle
			Vector3f mag_earth_pred = R_to_earth * _mag_sample_delayed.mag;
			// the angle of the projection onto the horizontal gives the yaw angle
			euler312(0) = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + _mag_declination;

		} else {
			// there is no yaw observation
			return false;
		}

		// re-calculate the rotation matrix using the updated yaw angle
		s0 = sinf(euler312(0));
		c0 = cosf(euler312(0));
		R_to_earth(0, 0) = c0 * c2 - s0 * s1 * s2;
		R_to_earth(1, 1) = c0 * c1;
		R_to_earth(2, 2) = c2 * c1;
		R_to_earth(0, 1) = -c1 * s0;
		R_to_earth(0, 2) = s2 * c0 + c2 * s1 * s0;
		R_to_earth(1, 0) = c2 * s0 + s2 * s1 * c0;
		R_to_earth(1, 2) = s0 * s2 - s1 * c0 * c2;
		R_to_earth(2, 0) = -s2 * c1;
		R_to_earth(2, 1) = s1;

		// calculate initial quaternion states for the ekf
		// we don't change the output attitude to avoid jumps
		quat_after_reset = Quatf(R_to_earth);
	}

	// set the earth magnetic field states using the updated rotation
	Dcmf _R_to_earth_after = quat_to_invrotmat(quat_after_reset);
	_state.mag_I = _R_to_earth_after * mag_init;

	// reset the corresponding rows and columns in the covariance matrix and set the variances on the magnetic field states to the measurement variance
	zeroRows(P, 16, 21);
	zeroCols(P, 16, 21);

	for (uint8_t index = 16; index <= 21; index ++) {
		P[index][index] = sq(_params.mag_noise);
	}

	// record the time for the magnetic field alignment event
	_flt_mag_align_start_time = _imu_sample_delayed.time_us;

	// calculate the amount that the quaternion has changed by
	Quatf q_error =  quat_before_reset.inversed() * quat_after_reset;
	q_error.normalize();

	// convert the quaternion delta to a delta angle
	Vector3f delta_ang_error;
	float scalar;

	if (q_error(0) >= 0.0f) {
		scalar = -2.0f;

	} else {
		scalar = 2.0f;
	}

	delta_ang_error(0) = scalar * q_error(1);
	delta_ang_error(1) = scalar * q_error(2);
	delta_ang_error(2) = scalar * q_error(3);

	// update quaternion states
	_state.quat_nominal = quat_after_reset;

	// record the state change
	_state_reset_status.quat_change = q_error;

	// update transformation matrix from body to world frame using the current estimate
	_R_to_earth = quat_to_invrotmat(_state.quat_nominal);

	// reset the rotation from the EV to EKF frame of reference if it is being used
	if ((_params.fusion_mode & MASK_ROTATE_EV) && (_params.fusion_mode & MASK_USE_EVPOS) && !_control_status.flags.ev_yaw) {
		resetExtVisRotMat();
	}

	// update the yaw angle variance using the variance of the measurement
	if (!_control_status.flags.ev_yaw) {
		// using error estimate from external vision data
		angle_err_var_vec(2) = sq(fmaxf(_ev_sample_delayed.angErr, 1.0e-2f));

	} else if (_params.mag_fusion_type <= MAG_FUSE_TYPE_AUTOFW) {
		// using magnetic heading tuning parameter
		angle_err_var_vec(2) = sq(fmaxf(_params.mag_heading_noise, 1.0e-2f));
	}

	// update the covariances only if the change in angle has been large to avoid unnecessary
	// manipulation of the covariance matrix that can temporarily reduce filter performance
	if (delta_ang_error.norm() > math::radians(15.0f)) {
		// reset the quaternion covariances using the rotation vector variances
		initialiseQuatCovariances(angle_err_var_vec);
	}

	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		// Note q1 *= q2 is equivalent to q1 = q2 * q1
		_output_buffer[i].quat_nominal *= _state_reset_status.quat_change;
	}

	// apply the change in attitude quaternion to our newest quaternion estimate
	// which was already taken out from the output buffer
	_output_new.quat_nominal = _state_reset_status.quat_change * _output_new.quat_nominal;

	// capture the reset event
	_state_reset_status.quat_counter++;

	return true;
}

// Calculate the magnetic declination to be used by the alignment and fusion processing
void Ekf::calcMagDeclination()
{
	// set source of magnetic declination for internal use
	if (_flt_mag_align_complete) {
		// Use value consistent with earth field state
		_mag_declination = atan2f(_state.mag_I(1), _state.mag_I(0));

	} else if (_params.mag_declination_source & MASK_USE_GEO_DECL) {
		// use parameter value until GPS is available, then use value returned by geo library
		if (_NED_origin_initialised) {
			_mag_declination = _mag_declination_gps;
			_mag_declination_to_save_deg = math::degrees(_mag_declination);

		} else {
			_mag_declination = math::radians(_params.mag_declination_deg);
			_mag_declination_to_save_deg = _params.mag_declination_deg;
		}

	} else {
		// always use the parameter value
		_mag_declination = math::radians(_params.mag_declination_deg);
		_mag_declination_to_save_deg = _params.mag_declination_deg;
	}
}

// This function forces the covariance matrix to be symmetric
void Ekf::makeSymmetrical(float (&cov_mat)[_k_num_states][_k_num_states], uint8_t first, uint8_t last)
{
	for (unsigned row = first; row <= last; row++) {
		for (unsigned column = 0; column < row; column++) {
			float tmp = (cov_mat[row][column] + cov_mat[column][row]) / 2;
			cov_mat[row][column] = tmp;
			cov_mat[column][row] = tmp;
		}
	}
}

void Ekf::constrainStates()
{
	for (int i = 0; i < 4; i++) {
		_state.quat_nominal(i) = math::constrain(_state.quat_nominal(i), -1.0f, 1.0f);
	}

	for (int i = 0; i < 3; i++) {
		_state.vel(i) = math::constrain(_state.vel(i), -1000.0f, 1000.0f);
	}

	for (int i = 0; i < 3; i++) {
		_state.pos(i) = math::constrain(_state.pos(i), -1.e6f, 1.e6f);
	}

	for (int i = 0; i < 3; i++) {
		_state.gyro_bias(i) = math::constrain(_state.gyro_bias(i), -math::radians(20.f) * _dt_ekf_avg, math::radians(20.f) * _dt_ekf_avg);
	}

	for (int i = 0; i < 3; i++) {
		_state.accel_bias(i) = math::constrain(_state.accel_bias(i), -_params.acc_bias_lim * _dt_ekf_avg, _params.acc_bias_lim * _dt_ekf_avg);
	}

	for (int i = 0; i < 3; i++) {
		_state.mag_I(i) = math::constrain(_state.mag_I(i), -1.0f, 1.0f);
	}

	for (int i = 0; i < 3; i++) {
		_state.mag_B(i) = math::constrain(_state.mag_B(i), -0.5f, 0.5f);
	}

	for (int i = 0; i < 2; i++) {
		_state.wind_vel(i) = math::constrain(_state.wind_vel(i), -100.0f, 100.0f);
	}
}

// calculate the earth rotation vector
void Ekf::calcEarthRateNED(Vector3f &omega, float lat_rad) const
{
	omega(0) = CONSTANTS_EARTH_SPIN_RATE * cosf(lat_rad);
	omega(1) = 0.0f;
	omega(2) = -CONSTANTS_EARTH_SPIN_RATE * sinf(lat_rad);
}

// gets the innovations of velocity and position measurements
// 0-2 vel, 3-5 pos
void Ekf::get_vel_pos_innov(float vel_pos_innov[6])
{
	memcpy(vel_pos_innov, _vel_pos_innov, sizeof(float) * 6);
}

// gets the innovations of the earth magnetic field measurements
void Ekf::get_aux_vel_innov(float aux_vel_innov[2])
{
	memcpy(aux_vel_innov, _aux_vel_innov, sizeof(float) * 2);
}

// writes the innovations of the earth magnetic field measurements
void Ekf::get_mag_innov(float mag_innov[3])
{
	memcpy(mag_innov, _mag_innov, 3 * sizeof(float));
}

// gets the innovations of the airspeed measnurement
void Ekf::get_airspeed_innov(float *airspeed_innov)
{
	memcpy(airspeed_innov, &_airspeed_innov, sizeof(float));
}

// gets the innovations of the synthetic sideslip measurements
void Ekf::get_beta_innov(float *beta_innov)
{
	memcpy(beta_innov, &_beta_innov, sizeof(float));
}

// gets the innovations of the heading measurement
void Ekf::get_heading_innov(float *heading_innov)
{
	memcpy(heading_innov, &_heading_innov, sizeof(float));
}

// gets the innovation variances of velocity and position measurements
// 0-2 vel, 3-5 pos
void Ekf::get_vel_pos_innov_var(float vel_pos_innov_var[6])
{
	memcpy(vel_pos_innov_var, _vel_pos_innov_var, sizeof(float) * 6);
}

// gets the innovation variances of the earth magnetic field measurements
void Ekf::get_mag_innov_var(float mag_innov_var[3])
{
	memcpy(mag_innov_var, _mag_innov_var, sizeof(float) * 3);
}

// gest the innovation variance of the airspeed measurement
void Ekf::get_airspeed_innov_var(float *airspeed_innov_var)
{
	memcpy(airspeed_innov_var, &_airspeed_innov_var, sizeof(float));
}

// gets the innovation variance of the synthetic sideslip measurement
void Ekf::get_beta_innov_var(float *beta_innov_var)
{
	memcpy(beta_innov_var, &_beta_innov_var, sizeof(float));
}

// gets the innovation variance of the heading measurement
void Ekf::get_heading_innov_var(float *heading_innov_var)
{
	memcpy(heading_innov_var, &_heading_innov_var, sizeof(float));
}

// get GPS check status
void Ekf::get_gps_check_status(uint16_t *val)
{
	*val = _gps_check_fail_status.value;
}

// get the state vector at the delayed time horizon
void Ekf::get_state_delayed(float *state)
{
	for (int i = 0; i < 4; i++) {
		state[i] = _state.quat_nominal(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 4] = _state.vel(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 7] = _state.pos(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 10] = _state.gyro_bias(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 13] = _state.accel_bias(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 16] = _state.mag_I(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 19] = _state.mag_B(i);
	}

	for (int i = 0; i < 2; i++) {
		state[i + 22] = _state.wind_vel(i);
	}
}

// get the accelerometer bias
void Ekf::get_accel_bias(float bias[3])
{
	float temp[3];
	temp[0] = _state.accel_bias(0) / _dt_ekf_avg;
	temp[1] = _state.accel_bias(1) / _dt_ekf_avg;
	temp[2] = _state.accel_bias(2) / _dt_ekf_avg;
	memcpy(bias, temp, 3 * sizeof(float));
}

// get the gyroscope bias in rad/s
void Ekf::get_gyro_bias(float bias[3])
{
	float temp[3];
	temp[0] = _state.gyro_bias(0) / _dt_ekf_avg;
	temp[1] = _state.gyro_bias(1) / _dt_ekf_avg;
	temp[2] = _state.gyro_bias(2) / _dt_ekf_avg;
	memcpy(bias, temp, 3 * sizeof(float));
}

// get the diagonal elements of the covariance matrix
void Ekf::get_covariances(float *covariances)
{
	for (unsigned i = 0; i < _k_num_states; i++) {
		covariances[i] = P[i][i];
	}
}

// get the position and height of the ekf origin in WGS-84 coordinates and time the origin was set
// return true if the origin is valid
bool Ekf::get_ekf_origin(uint64_t *origin_time, map_projection_reference_s *origin_pos, float *origin_alt)
{
	memcpy(origin_time, &_last_gps_origin_time_us, sizeof(uint64_t));
	memcpy(origin_pos, &_pos_ref, sizeof(map_projection_reference_s));
	memcpy(origin_alt, &_gps_alt_ref, sizeof(float));
	return _NED_origin_initialised;
}

// return an array containing the output predictor angular, velocity and position tracking
// error magnitudes (rad), (m/s), (m)
void Ekf::get_output_tracking_error(float error[3])
{
	memcpy(error, _output_tracking_error, 3 * sizeof(float));
}

/*
Returns  following IMU vibration metrics in the following array locations
0 : Gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
1 : Gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
2 : Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)
*/
void Ekf::get_imu_vibe_metrics(float vibe[3])
{
	memcpy(vibe, _vibe_metrics, 3 * sizeof(float));
}

/*
	First argument returns GPS drift  metrics in the following array locations
	0 : Horizontal position drift rate (m/s)
	1 : Vertical position drift rate (m/s)
	2 : Filtered horizontal velocity (m/s)
	Second argument returns true when IMU movement is blocking the drift calculation
	Function returns true if the metrics have been updated and not returned previously by this function
*/
bool Ekf::get_gps_drift_metrics(float drift[3], bool *blocked)
{
	memcpy(drift, _gps_drift_metrics, 3 * sizeof(float));
	*blocked = !_vehicle_at_rest;
	if (_gps_drift_updated) {
		_gps_drift_updated = false;
		return true;
	}
	return false;
}

// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
void Ekf::get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv)
{
	// report absolute accuracy taking into account the uncertainty in location of the origin
	// If not aiding, return 0 for horizontal position estimate as no estimate is available
	// TODO - allow for baro drift in vertical position error
	float hpos_err = sqrtf(P[7][7] + P[8][8] + sq(_gps_origin_eph));

	// If we are dead-reckoning, use the innovations as a conservative alternate measure of the horizontal position error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_is_dead_reckoning && (_control_status.flags.gps || _control_status.flags.ev_pos)) {
		hpos_err = math::max(hpos_err, sqrtf(sq(_vel_pos_innov[3]) + sq(_vel_pos_innov[4])));
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P[9][9] + sq(_gps_origin_epv));
}

// get the 1-sigma horizontal and vertical position uncertainty of the ekf local position
void Ekf::get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv)
{
	// TODO - allow for baro drift in vertical position error
	float hpos_err = sqrtf(P[7][7] + P[8][8]);

	// If we are dead-reckoning, use the innovations as a conservative alternate measure of the horizontal position error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_is_dead_reckoning && (_control_status.flags.gps || _control_status.flags.ev_pos)) {
		hpos_err = math::max(hpos_err, sqrtf(sq(_vel_pos_innov[3]) + sq(_vel_pos_innov[4])));
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P[9][9]);
}

// get the 1-sigma horizontal and vertical velocity uncertainty
void Ekf::get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv)
{
	float hvel_err = sqrtf(P[4][4] + P[5][5]);

	// If we are dead-reckoning, use the innovations as a conservative alternate measure of the horizontal velocity error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_is_dead_reckoning) {
		float vel_err_conservative = 0.0f;

		if (_control_status.flags.opt_flow) {
			float gndclearance = math::max(_params.rng_gnd_clearance, 0.1f);
			vel_err_conservative = math::max((_terrain_vpos - _state.pos(2)), gndclearance) * sqrtf(sq(_flow_innov[0]) + sq(_flow_innov[1]));
		}

		if (_control_status.flags.gps || _control_status.flags.ev_pos) {
			vel_err_conservative = math::max(vel_err_conservative, sqrtf(sq(_vel_pos_innov[0]) + sq(_vel_pos_innov[1])));
		}

		hvel_err = math::max(hvel_err, vel_err_conservative);
	}

	*ekf_evh = hvel_err;
	*ekf_evv = sqrtf(P[6][6]);
}

/*
Returns the following vehicle control limits required by the estimator to keep within sensor limitations.
vxy_max : Maximum ground relative horizontal speed (meters/sec). NaN when limiting is not needed.
vz_max : Maximum ground relative vertical speed (meters/sec). NaN when limiting is not needed.
hagl_min : Minimum height above ground (meters). NaN when limiting is not needed.
hagl_max : Maximum height above ground (meters). NaN when limiting is not needed.
*/
void Ekf::get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max)
{
	// Calculate range finder limits
	float rangefinder_hagl_min = _rng_valid_min_val;
	// Allow use of 75% of rangefinder maximum range to allow for angular motion
	float rangefinder_hagl_max = 0.75f * _rng_valid_max_val;

	// Calculate optical flow limits
	// Allow ground relative velocity to use 50% of available flow sensor range to allow for angular motion
	float flow_vxy_max = fmaxf(0.5f * _flow_max_rate * (_terrain_vpos - _state.pos(2)), 0.0f);
	float flow_hagl_min = _flow_min_distance;
	float flow_hagl_max = _flow_max_distance;

	// TODO : calculate visual odometry limits

	bool relying_on_rangefinder = _control_status.flags.rng_hgt && !_params.range_aid;

	bool relying_on_optical_flow = _control_status.flags.opt_flow && !(_control_status.flags.gps || _control_status.flags.ev_pos);

	// Do not require limiting by default
	*vxy_max = NAN;
	*vz_max = NAN;
	*hagl_min = NAN;
	*hagl_max = NAN;

	// Keep within range sensor limit when using rangefinder as primary height source
	if (relying_on_rangefinder) {
		*vxy_max = NAN;
		*vz_max = NAN;
		*hagl_min = rangefinder_hagl_min;
		*hagl_max = rangefinder_hagl_max;
	}

	// Keep within flow AND range sensor limits when exclusively using optical flow
	if (relying_on_optical_flow) {
		*vxy_max = flow_vxy_max;
		*vz_max = NAN;
		*hagl_min = fmaxf(rangefinder_hagl_min, flow_hagl_min);
		*hagl_max = fminf(rangefinder_hagl_max, flow_hagl_max);
	}

}

bool Ekf::reset_imu_bias()
{
	if (_imu_sample_delayed.time_us - _last_imu_bias_cov_reset_us < (uint64_t)10e6) {
		return false;

	}

	// Zero the delta angle and delta velocity bias states
	_state.gyro_bias.zero();
	_state.accel_bias.zero();

	// Zero the corresponding covariances
	zeroCols(P, 10, 15);
	zeroRows(P, 10, 15);

	// Set the corresponding variances to the values use for initial alignment
	float dt = FILTER_UPDATE_PERIOD_S;
	P[12][12] = P[11][11] = P[10][10] = sq(_params.switch_on_gyro_bias * dt);
	P[15][15] = P[14][14] = P[13][13] = sq(_params.switch_on_accel_bias * dt);
	_last_imu_bias_cov_reset_us = _imu_sample_delayed.time_us;

	// Set previous frame values
	_prev_dvel_bias_var(0) = P[13][13];
	_prev_dvel_bias_var(1) = P[14][14];
	_prev_dvel_bias_var(2) = P[15][15];

	return true;

}

// get EKF innovation consistency check status information comprising of:
// status - a bitmask integer containing the pass/fail status for each EKF measurement innovation consistency check
// Innovation Test Ratios - these are the ratio of the innovation to the acceptance threshold.
// A value > 1 indicates that the sensor measurement has exceeded the maximum acceptable level and has been rejected by the EKF
// Where a measurement type is a vector quantity, eg magnetoemter, GPS position, etc, the maximum value is returned.
void Ekf::get_innovation_test_status(uint16_t *status, float *mag, float *vel, float *pos, float *hgt, float *tas, float *hagl, float *beta)
{
	// return the integer bitmask containing the consistency check pass/fail satus
	*status = _innov_check_fail_status.value;
	// return the largest magnetometer innovation test ratio
	*mag = sqrtf(math::max(_yaw_test_ratio, math::max(math::max(_mag_test_ratio[0], _mag_test_ratio[1]), _mag_test_ratio[2])));
	// return the largest NED velocity innovation test ratio
	*vel = sqrtf(math::max(math::max(_vel_pos_test_ratio[0], _vel_pos_test_ratio[1]), _vel_pos_test_ratio[2]));
	// return the largest NE position innovation test ratio
	*pos = sqrtf(math::max(_vel_pos_test_ratio[3], _vel_pos_test_ratio[4]));
	// return the vertical position innovation test ratio
	*hgt = sqrtf(_vel_pos_test_ratio[5]);
	// return the airspeed fusion innovation test ratio
	*tas = sqrtf(_tas_test_ratio);
	// return the terrain height innovation test ratio
	*hagl = sqrtf(_terr_test_ratio);
	// return the synthetic sideslip innovation test ratio
	*beta = sqrtf(_beta_test_ratio);
}

// return a bitmask integer that describes which state estimates are valid
void Ekf::get_ekf_soln_status(uint16_t *status)
{
	ekf_solution_status soln_status;

	soln_status.flags.attitude = _control_status.flags.tilt_align && _control_status.flags.yaw_align && (_fault_status.value == 0);
	soln_status.flags.velocity_horiz = (_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.opt_flow || (_control_status.flags.fuse_beta && _control_status.flags.fuse_aspd)) && (_fault_status.value == 0);
	soln_status.flags.velocity_vert = (_control_status.flags.baro_hgt || _control_status.flags.ev_hgt || _control_status.flags.gps_hgt || _control_status.flags.rng_hgt) && (_fault_status.value == 0);
	soln_status.flags.pos_horiz_rel = (_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.opt_flow) && (_fault_status.value == 0);
	soln_status.flags.pos_horiz_abs = (_control_status.flags.gps || _control_status.flags.ev_pos) && (_fault_status.value == 0);
	soln_status.flags.pos_vert_abs = soln_status.flags.velocity_vert;
	soln_status.flags.pos_vert_agl = get_terrain_valid();
	soln_status.flags.const_pos_mode = !soln_status.flags.velocity_horiz;
	soln_status.flags.pred_pos_horiz_rel = soln_status.flags.pos_horiz_rel;
	soln_status.flags.pred_pos_horiz_abs = soln_status.flags.pos_horiz_abs;
	bool gps_vel_innov_bad = (_vel_pos_test_ratio[0] > 1.0f) || (_vel_pos_test_ratio[1] > 1.0f);
	bool gps_pos_innov_bad = (_vel_pos_test_ratio[3] > 1.0f) || (_vel_pos_test_ratio[4] > 1.0f);
	bool mag_innov_good = (_mag_test_ratio[0] < 1.0f) && (_mag_test_ratio[1] < 1.0f) && (_mag_test_ratio[2] < 1.0f) && (_yaw_test_ratio < 1.0f);
	soln_status.flags.gps_glitch = (gps_vel_innov_bad || gps_pos_innov_bad) && mag_innov_good;
	soln_status.flags.accel_error = _bad_vert_accel_detected;
	*status = soln_status.value;
}

// fuse measurement
void Ekf::fuse(float *K, float innovation)
{
	for (unsigned i = 0; i < 4; i++) {
		_state.quat_nominal(i) = _state.quat_nominal(i) - K[i] * innovation;
	}

	_state.quat_nominal.normalize();

	for (unsigned i = 0; i < 3; i++) {
		_state.vel(i) = _state.vel(i) - K[i + 4] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.pos(i) = _state.pos(i) - K[i + 7] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.gyro_bias(i) = _state.gyro_bias(i) - K[i + 10] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.accel_bias(i) = _state.accel_bias(i) - K[i + 13] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.mag_I(i) = _state.mag_I(i) - K[i + 16] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.mag_B(i) = _state.mag_B(i) - K[i + 19] * innovation;
	}

	for (unsigned i = 0; i < 2; i++) {
		_state.wind_vel(i) = _state.wind_vel(i) - K[i + 22] * innovation;
	}
}

// zero specified range of rows in the state covariance matrix
void Ekf::zeroRows(float (&cov_mat)[_k_num_states][_k_num_states], uint8_t first, uint8_t last)
{
	uint8_t row;

	for (row = first; row <= last; row++) {
		memset(&cov_mat[row][0], 0, sizeof(cov_mat[0][0]) * 24);
	}
}

// zero specified range of columns in the state covariance matrix
void Ekf::zeroCols(float (&cov_mat)[_k_num_states][_k_num_states], uint8_t first, uint8_t last)
{
	uint8_t row;

	for (row = 0; row <= 23; row++) {
		memset(&cov_mat[row][first], 0, sizeof(cov_mat[0][0]) * (1 + last - first));
	}
}

void Ekf::zeroOffDiag(float (&cov_mat)[_k_num_states][_k_num_states], uint8_t first, uint8_t last)
{
	// save diagonal elements
	uint8_t row;
	float variances[_k_num_states];

	for (row = first; row <= last; row++) {
		variances[row] = cov_mat[row][row];
	}

	// zero rows and columns
	zeroRows(cov_mat, first, last);
	zeroCols(cov_mat, first, last);

	// restore diagonals
	for (row = first; row <= last; row++) {
		cov_mat[row][row] = variances[row];
	}
}

void Ekf::setDiag(float (&cov_mat)[_k_num_states][_k_num_states], uint8_t first, uint8_t last, float variance)
{
	// zero rows and columns
	zeroRows(cov_mat, first, last);
	zeroCols(cov_mat, first, last);

	// set diagonals
	uint8_t row;

	for (row = first; row <= last; row++) {
		cov_mat[row][row] = variance;
	}

}

bool Ekf::global_position_is_valid()
{
	// return true if the origin is set we are not doing unconstrained free inertial navigation
	// and have not started using synthetic position observations to constrain drift
	return (_NED_origin_initialised && !_deadreckon_time_exceeded && !_using_synthetic_position);
}

// return true if we are totally reliant on inertial dead-reckoning for position
void Ekf::update_deadreckoning_status()
{
	bool velPosAiding = (_control_status.flags.gps || _control_status.flags.ev_pos)
			    && ((_time_last_imu - _time_last_pos_fuse <= _params.no_aid_timeout_max)
				|| (_time_last_imu - _time_last_vel_fuse <= _params.no_aid_timeout_max)
				|| (_time_last_imu - _time_last_delpos_fuse <= _params.no_aid_timeout_max));
	bool optFlowAiding = _control_status.flags.opt_flow && (_time_last_imu - _time_last_of_fuse <= _params.no_aid_timeout_max);
	bool airDataAiding = _control_status.flags.wind && (_time_last_imu - _time_last_arsp_fuse <= _params.no_aid_timeout_max) && (_time_last_imu - _time_last_beta_fuse <= _params.no_aid_timeout_max);

	_is_wind_dead_reckoning = !velPosAiding && !optFlowAiding && airDataAiding;
	_is_dead_reckoning = !velPosAiding && !optFlowAiding && !airDataAiding;

	// record the time we start inertial dead reckoning
	if (!_is_dead_reckoning) {
		_time_ins_deadreckon_start = _time_last_imu - _params.no_aid_timeout_max;
	}

	// report if we have been deadreckoning for too long
	_deadreckon_time_exceeded = ((_time_last_imu - _time_ins_deadreckon_start) > (unsigned)_params.valid_timeout_max);
}

// perform a vector cross product
Vector3f EstimatorInterface::cross_product(const Vector3f &vecIn1, const Vector3f &vecIn2)
{
	Vector3f vecOut;
	vecOut(0) = vecIn1(1) * vecIn2(2) - vecIn1(2) * vecIn2(1);
	vecOut(1) = vecIn1(2) * vecIn2(0) - vecIn1(0) * vecIn2(2);
	vecOut(2) = vecIn1(0) * vecIn2(1) - vecIn1(1) * vecIn2(0);
	return vecOut;
}

// calculate the inverse rotation matrix from a quaternion rotation
Matrix3f EstimatorInterface::quat_to_invrotmat(const Quatf &quat)
{
	float q00 = quat(0) * quat(0);
	float q11 = quat(1) * quat(1);
	float q22 = quat(2) * quat(2);
	float q33 = quat(3) * quat(3);
	float q01 = quat(0) * quat(1);
	float q02 = quat(0) * quat(2);
	float q03 = quat(0) * quat(3);
	float q12 = quat(1) * quat(2);
	float q13 = quat(1) * quat(3);
	float q23 = quat(2) * quat(3);

	Matrix3f dcm;
	dcm(0, 0) = q00 + q11 - q22 - q33;
	dcm(1, 1) = q00 - q11 + q22 - q33;
	dcm(2, 2) = q00 - q11 - q22 + q33;
	dcm(0, 1) = 2.0f * (q12 - q03);
	dcm(0, 2) = 2.0f * (q13 + q02);
	dcm(1, 0) = 2.0f * (q12 + q03);
	dcm(1, 2) = 2.0f * (q23 - q01);
	dcm(2, 0) = 2.0f * (q13 - q02);
	dcm(2, 1) = 2.0f * (q23 + q01);

	return dcm;
}

// calculate the variances for the rotation vector equivalent
Vector3f Ekf::calcRotVecVariances()
{
	Vector3f rot_var_vec;
	float q0, q1, q2, q3;

	if (_state.quat_nominal(0) >= 0.0f) {
		q0 = _state.quat_nominal(0);
		q1 = _state.quat_nominal(1);
		q2 = _state.quat_nominal(2);
		q3 = _state.quat_nominal(3);

	} else {
		q0 = -_state.quat_nominal(0);
		q1 = -_state.quat_nominal(1);
		q2 = -_state.quat_nominal(2);
		q3 = -_state.quat_nominal(3);
	}
	float t2 = q0*q0;
	float t3 = acosf(q0);
	float t4 = -t2+1.0f;
	float t5 = t2-1.0f;
	if ((t4 > 1e-9f) && (t5 < -1e-9f)) {
		float t6 = 1.0f/t5;
		float t7 = q1*t6*2.0f;
		float t8 = 1.0f/powf(t4,1.5f);
		float t9 = q0*q1*t3*t8*2.0f;
		float t10 = t7+t9;
		float t11 = 1.0f/sqrtf(t4);
		float t12 = q2*t6*2.0f;
		float t13 = q0*q2*t3*t8*2.0f;
		float t14 = t12+t13;
		float t15 = q3*t6*2.0f;
		float t16 = q0*q3*t3*t8*2.0f;
		float t17 = t15+t16;
		rot_var_vec(0) = t10*(P[0][0]*t10+P[1][0]*t3*t11*2.0f)+t3*t11*(P[0][1]*t10+P[1][1]*t3*t11*2.0f)*2.0f;
		rot_var_vec(1) = t14*(P[0][0]*t14+P[2][0]*t3*t11*2.0f)+t3*t11*(P[0][2]*t14+P[2][2]*t3*t11*2.0f)*2.0f;
		rot_var_vec(2) = t17*(P[0][0]*t17+P[3][0]*t3*t11*2.0f)+t3*t11*(P[0][3]*t17+P[3][3]*t3*t11*2.0f)*2.0f;
	} else {
		rot_var_vec(0) = 4.0f * P[1][1];
		rot_var_vec(1) = 4.0f * P[2][2];
		rot_var_vec(2) = 4.0f * P[3][3];
	}

	return rot_var_vec;
}

// initialise the quaternion covariances using rotation vector variances
void Ekf::initialiseQuatCovariances(Vector3f &rot_vec_var)
{
	// calculate an equivalent rotation vector from the quaternion
	float q0,q1,q2,q3;
	if (_state.quat_nominal(0) >= 0.0f) {
		q0 = _state.quat_nominal(0);
		q1 = _state.quat_nominal(1);
		q2 = _state.quat_nominal(2);
		q3 = _state.quat_nominal(3);

	} else {
		q0 = -_state.quat_nominal(0);
		q1 = -_state.quat_nominal(1);
		q2 = -_state.quat_nominal(2);
		q3 = -_state.quat_nominal(3);
	}
	float delta = 2.0f*acosf(q0);
	float scaler = (delta/sinf(delta*0.5f));
	float rotX = scaler*q1;
	float rotY = scaler*q2;
	float rotZ = scaler*q3;

	// autocode generated using matlab symbolic toolbox
	float t2 = rotX*rotX;
	float t4 = rotY*rotY;
	float t5 = rotZ*rotZ;
	float t6 = t2+t4+t5;
	if (t6 > 1e-9f) {
		float t7 = sqrtf(t6);
		float t8 = t7*0.5f;
		float t3 = sinf(t8);
		float t9 = t3*t3;
		float t10 = 1.0f/t6;
		float t11 = 1.0f/sqrtf(t6);
		float t12 = cosf(t8);
		float t13 = 1.0f/powf(t6,1.5f);
		float t14 = t3*t11;
		float t15 = rotX*rotY*t3*t13;
		float t16 = rotX*rotZ*t3*t13;
		float t17 = rotY*rotZ*t3*t13;
		float t18 = t2*t10*t12*0.5f;
		float t27 = t2*t3*t13;
		float t19 = t14+t18-t27;
		float t23 = rotX*rotY*t10*t12*0.5f;
		float t28 = t15-t23;
		float t20 = rotY*rot_vec_var(1)*t3*t11*t28*0.5f;
		float t25 = rotX*rotZ*t10*t12*0.5f;
		float t31 = t16-t25;
		float t21 = rotZ*rot_vec_var(2)*t3*t11*t31*0.5f;
		float t22 = t20+t21-rotX*rot_vec_var(0)*t3*t11*t19*0.5f;
		float t24 = t15-t23;
		float t26 = t16-t25;
		float t29 = t4*t10*t12*0.5f;
		float t34 = t3*t4*t13;
		float t30 = t14+t29-t34;
		float t32 = t5*t10*t12*0.5f;
		float t40 = t3*t5*t13;
		float t33 = t14+t32-t40;
		float t36 = rotY*rotZ*t10*t12*0.5f;
		float t39 = t17-t36;
		float t35 = rotZ*rot_vec_var(2)*t3*t11*t39*0.5f;
		float t37 = t15-t23;
		float t38 = t17-t36;
		float t41 = rot_vec_var(0)*(t15-t23)*(t16-t25);
		float t42 = t41-rot_vec_var(1)*t30*t39-rot_vec_var(2)*t33*t39;
		float t43 = t16-t25;
		float t44 = t17-t36;

		// zero all the quaternion covariances
		zeroRows(P, 0, 3);
		zeroCols(P, 0, 3);

		// Update the quaternion internal covariances using auto-code generated using matlab symbolic toolbox
		P[0][0] = rot_vec_var(0)*t2*t9*t10*0.25f+rot_vec_var(1)*t4*t9*t10*0.25f+rot_vec_var(2)*t5*t9*t10*0.25f;
		P[0][1] = t22;
		P[0][2] = t35+rotX*rot_vec_var(0)*t3*t11*(t15-rotX*rotY*t10*t12*0.5f)*0.5f-rotY*rot_vec_var(1)*t3*t11*t30*0.5f;
		P[0][3] = rotX*rot_vec_var(0)*t3*t11*(t16-rotX*rotZ*t10*t12*0.5f)*0.5f+rotY*rot_vec_var(1)*t3*t11*(t17-rotY*rotZ*t10*t12*0.5f)*0.5f-rotZ*rot_vec_var(2)*t3*t11*t33*0.5f;
		P[1][0] = t22;
		P[1][1] = rot_vec_var(0)*(t19*t19)+rot_vec_var(1)*(t24*t24)+rot_vec_var(2)*(t26*t26);
		P[1][2] = rot_vec_var(2)*(t16-t25)*(t17-rotY*rotZ*t10*t12*0.5f)-rot_vec_var(0)*t19*t28-rot_vec_var(1)*t28*t30;
		P[1][3] = rot_vec_var(1)*(t15-t23)*(t17-rotY*rotZ*t10*t12*0.5f)-rot_vec_var(0)*t19*t31-rot_vec_var(2)*t31*t33;
		P[2][0] = t35-rotY*rot_vec_var(1)*t3*t11*t30*0.5f+rotX*rot_vec_var(0)*t3*t11*(t15-t23)*0.5f;
		P[2][1] = rot_vec_var(2)*(t16-t25)*(t17-t36)-rot_vec_var(0)*t19*t28-rot_vec_var(1)*t28*t30;
		P[2][2] = rot_vec_var(1)*(t30*t30)+rot_vec_var(0)*(t37*t37)+rot_vec_var(2)*(t38*t38);
		P[2][3] = t42;
		P[3][0] = rotZ*rot_vec_var(2)*t3*t11*t33*(-0.5f)+rotX*rot_vec_var(0)*t3*t11*(t16-t25)*0.5f+rotY*rot_vec_var(1)*t3*t11*(t17-t36)*0.5f;
		P[3][1] = rot_vec_var(1)*(t15-t23)*(t17-t36)-rot_vec_var(0)*t19*t31-rot_vec_var(2)*t31*t33;
		P[3][2] = t42;
		P[3][3] = rot_vec_var(2)*(t33*t33)+rot_vec_var(0)*(t43*t43)+rot_vec_var(1)*(t44*t44);

	} else {
		// the equations are badly conditioned so use a small angle approximation
		P[0][0] = 0.0f;
		P[0][1] = 0.0f;
		P[0][2] = 0.0f;
		P[0][3] = 0.0f;
		P[1][0] = 0.0f;
		P[1][1] = 0.25f * rot_vec_var(0);
		P[1][2] = 0.0f;
		P[1][3] = 0.0f;
		P[2][0] = 0.0f;
		P[2][1] = 0.0f;
		P[2][2] = 0.25f * rot_vec_var(1);
		P[2][3] = 0.0f;
		P[3][0] = 0.0f;
		P[3][1] = 0.0f;
		P[3][2] = 0.0f;
		P[3][3] = 0.25f * rot_vec_var(2);

	}
}

void Ekf::setControlBaroHeight()
{
	_control_status.flags.baro_hgt = true;

	_control_status.flags.gps_hgt = false;
	_control_status.flags.rng_hgt = false;
	_control_status.flags.ev_hgt = false;
}

void Ekf::setControlRangeHeight()
{
	_control_status.flags.rng_hgt = true;

	_control_status.flags.baro_hgt = false;
	_control_status.flags.gps_hgt = false;
	_control_status.flags.ev_hgt = false;
}

void Ekf::setControlGPSHeight()
{
	_control_status.flags.gps_hgt = true;

	_control_status.flags.baro_hgt = false;
	_control_status.flags.rng_hgt = false;
	_control_status.flags.ev_hgt = false;
}

void Ekf::setControlEVHeight()
{
	_control_status.flags.ev_hgt = true;

	_control_status.flags.baro_hgt = false;
	_control_status.flags.gps_hgt = false;
	_control_status.flags.rng_hgt = false;
}

// update the estimated misalignment between the EV navigation frame and the EKF navigation frame
// and calculate a rotation matrix which rotates EV measurements into the EKF's navigatin frame
void Ekf::calcExtVisRotMat()
{
	// calculate the quaternion delta between the EKF and EV reference frames at the EKF fusion time horizon
	Quatf quat_inv = _ev_sample_delayed.quat.inversed();
	Quatf q_error =   quat_inv * _state.quat_nominal;
	q_error.normalize();

	// convert to a delta angle and apply a spike and low pass filter
	Vector3f rot_vec = q_error.to_axis_angle();

	float rot_vec_norm = rot_vec.norm();

	if (rot_vec_norm > 1e-6f) {

		// apply an input limiter to protect from spikes
		Vector3f _input_delta_vec = rot_vec - _ev_rot_vec_filt;
		float input_delta_mag = _input_delta_vec.norm();

		if (input_delta_mag > 0.1f) {
			rot_vec = _ev_rot_vec_filt + _input_delta_vec * (0.1f / input_delta_mag);
		}

		// Apply a first order IIR low pass filter
		const float omega_lpf_us = 0.2e-6f; // cutoff frequency in rad/uSec
		float alpha = math::constrain(omega_lpf_us * (float)(_time_last_imu - _ev_rot_last_time_us), 0.0f, 1.0f);
		_ev_rot_last_time_us = _time_last_imu;
		_ev_rot_vec_filt = _ev_rot_vec_filt * (1.0f - alpha) + rot_vec * alpha;

	}

	// convert filtered vector to a quaternion and then to a rotation matrix
	q_error.from_axis_angle(_ev_rot_vec_filt);
	_ev_rot_mat = quat_to_invrotmat(q_error);

}

// reset the estimated misalignment between the EV navigation frame and the EKF navigation frame
// and update the rotation matrix which rotates EV measurements into the EKF's navigation frame
void Ekf::resetExtVisRotMat()
{
	// calculate the quaternion delta between the EKF and EV reference frames at the EKF fusion time horizon
	Quatf quat_inv = _ev_sample_delayed.quat.inversed();
	Quatf q_error =   quat_inv * _state.quat_nominal;
	q_error.normalize();

	// convert to a delta angle and reset
	Vector3f rot_vec = q_error.to_axis_angle();

	float rot_vec_norm = rot_vec.norm();

	if (rot_vec_norm > 1e-9f) {
		_ev_rot_vec_filt = rot_vec;

	} else {
		_ev_rot_vec_filt.zero();
	}

	// reset the rotation matrix
	_ev_rot_mat = quat_to_invrotmat(q_error);
}

// return the quaternions for the rotation from the EKF to the External Vision system frame of reference
void Ekf::get_ekf2ev_quaternion(float *quat)
{
	Quatf quat_ekf2ev;
	quat_ekf2ev.from_axis_angle(_ev_rot_vec_filt);

	for (unsigned i = 0; i < 4; i++) {
		quat[i] = quat_ekf2ev(i);
	}
}
