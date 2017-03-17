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
#include "mathlib.h"
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

	} else if (_control_status.flags.opt_flow || _control_status.flags.ev_pos) {
		_state.vel.setZero();

	} else {
		return false;

	}

	// calculate the change in velocity and apply to the output predictor state history
	Vector3f velocity_change = _state.vel - vel_before_reset;
	outputSample output_states;
	unsigned max_index = _output_buffer.get_length() - 1;
	for (unsigned index=0; index <= max_index; index++) {
		output_states = _output_buffer.get_from_index(index);
		output_states.vel += velocity_change;
		_output_buffer.push_to_index(index,output_states);

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

	if (_control_status.flags.gps) {
		// this reset is only called if we have new gps data at the fusion time horizon
		_state.pos(0) = _gps_sample_delayed.pos(0);
		_state.pos(1) = _gps_sample_delayed.pos(1);

	} else if (_control_status.flags.opt_flow) {
		_state.pos(0) = 0.0f;
		_state.pos(1) = 0.0f;

	} else if (_control_status.flags.ev_pos) {
		// this reset is only called if we have new ev data at the fusion time horizon
		_state.pos(0) = _ev_sample_delayed.posNED(0);
		_state.pos(1) = _ev_sample_delayed.posNED(1);

	} else {
		return false;
	}

	// calculate the change in position and apply to the output predictor state history
	Vector2f posNE_change;
	posNE_change(0) = _state.pos(0) - posNE_before_reset(0);
	posNE_change(1) = _state.pos(1) - posNE_before_reset(1);
	outputSample output_states;
	unsigned max_index = _output_buffer.get_length() - 1;
	for (unsigned index=0; index <= max_index; index++) {
		output_states = _output_buffer.get_from_index(index);
		output_states.pos(0) += posNE_change(0);
		output_states.pos(1) += posNE_change(1);
		_output_buffer.push_to_index(index,output_states);
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
	gpsSample gps_newest = _gps_buffer.get_newest();

	// store the current vertical position and velocity for reference so we can calculate and publish the reset amount
	float old_vert_pos = _state.pos(2);
	bool vert_pos_reset = false;
	float old_vert_vel = _state.vel(2);
	bool vert_vel_reset = false;

	// reset the vertical position
	if (_control_status.flags.rng_hgt) {
		rangeSample range_newest = _range_buffer.get_newest();

		if (_time_last_imu - range_newest.time_us < 2 * RNG_MAX_INTERVAL) {
			// calculate the new vertical position using range sensor
			float new_pos_down = _hgt_sensor_offset - range_newest.rng;

			// update the state and assoicated variance
			_state.pos(2) = new_pos_down;

			// reset the associated covariance values
			zeroRows(P, 9, 9);
			zeroCols(P, 9, 9);

			// the state variance is the same as the observation
			P[9][9] = sq(_params.range_noise);

			vert_pos_reset = true;

			// reset the baro offset which is subtracted from the baro reading if we need to use it as a backup
			baroSample baro_newest = _baro_buffer.get_newest();
			_baro_hgt_offset = baro_newest.hgt + _state.pos(2);

		} else {
			// TODO: reset to last known range based estimate
		}


	} else if (_control_status.flags.baro_hgt) {
		// initialize vertical position with newest baro measurement
		baroSample baro_newest = _baro_buffer.get_newest();

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
			baroSample baro_newest = _baro_buffer.get_newest();
			_baro_hgt_offset = baro_newest.hgt + _state.pos(2);

		} else {
			// TODO: reset to last known gps based estimate
		}

	} else if (_control_status.flags.ev_hgt) {
		// initialize vertical position with newest measurement
		extVisionSample ev_newest = _ext_vision_buffer.get_newest();

		// use the most recent data if it's time offset from the fusion time horizon is smaller
		int32_t dt_newest = ev_newest.time_us - _imu_sample_delayed.time_us;
		int32_t dt_delayed = _ev_sample_delayed.time_us - _imu_sample_delayed.time_us;
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
	outputSample output_states;
	unsigned output_length = _output_buffer.get_length();
	for (unsigned i=0; i < output_length; i++) {
		output_states = _output_buffer.get_from_index(i);

		if (vert_pos_reset) {
			output_states.pos(2) += _state_reset_status.posD_change;
		}

		if (vert_vel_reset) {
			output_states.vel(2) += _state_reset_status.velD_change;
		}

		_output_buffer.push_to_index(i,output_states);

	}
}

// align output filter states to match EKF states at the fusion time horizon
void Ekf::alignOutputFilter()
{
	// calculate the quaternion delta between the output and EKF quaternions at the EKF fusion time horizon
	Quaternion quat_inv = _state.quat_nominal.inversed();
	Quaternion q_delta =  _output_sample_delayed.quat_nominal * quat_inv;
	q_delta.normalize();

	// calculate the velocity and posiiton deltas between the output and EKF at the EKF fusion time horizon
	Vector3f vel_delta = _state.vel - _output_sample_delayed.vel;
	Vector3f pos_delta = _state.pos - _output_sample_delayed.pos;

	// loop through the output filter state history and add the deltas
	outputSample output_states;
	unsigned output_length = _output_buffer.get_length();
	for (unsigned i=0; i < output_length; i++) {
		output_states = _output_buffer.get_from_index(i);
		output_states.quat_nominal *= q_delta;
		output_states.quat_nominal.normalize();
		output_states.vel += vel_delta;
		output_states.pos += pos_delta;
		_output_buffer.push_to_index(i,output_states);
	}
}

// Reset heading and magnetic field states
bool Ekf::resetMagHeading(Vector3f &mag_init)
{
	// save a copy of the quaternion state for later use in calculating the amount of reset change
	Quaternion quat_before_reset = _state.quat_nominal;

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
		matrix::Euler<float> euler321(_state.quat_nominal);

		// Set the yaw angle to zero and calculate the rotation matrix from body to earth frame
		euler321(2) = 0.0f;
		matrix::Dcm<float> R_to_earth(euler321);

		// calculate the observed yaw angle
		if (_params.fusion_mode & MASK_USE_EVYAW) {
			// convert the observed quaternion to a rotation matrix
			matrix::Dcm<float> R_to_earth_ev(_ev_sample_delayed.quat);	// transformation matrix from body to world frame
			// calculate the yaw angle for a 312 sequence
			euler321(2) = atan2f(R_to_earth_ev(1, 0) , R_to_earth_ev(0, 0));
		} else if (_params.mag_fusion_type <= MAG_FUSE_TYPE_3D) {
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
		_state.quat_nominal = Quaternion(euler321);

	} else {
		// use a 312 sequence

		// Calculate the 312 sequence euler angles that rotate from earth to body frame
		// See http://www.atacolorado.com/eulersequences.doc
		Vector3f euler312;
		euler312(0) = atan2f(-_R_to_earth(0, 1) , _R_to_earth(1, 1)); // first rotation (yaw)
		euler312(1) = asinf(_R_to_earth(2, 1)); // second rotation (roll)
		euler312(2) = atan2f(-_R_to_earth(2, 0) , _R_to_earth(2, 2)); // third rotation (pitch)

		// Set the first rotation (yaw) to zero and calculate the rotation matrix from body to earth frame
		euler312(0) = 0.0f;

		// Calculate the body to earth frame rotation matrix from the euler angles using a 312 rotation sequence
		float c2 = cosf(euler312(2));
		float s2 = sinf(euler312(2));
		float s1 = sinf(euler312(1));
		float c1 = cosf(euler312(1));
		float s0 = sinf(euler312(0));
		float c0 = cosf(euler312(0));

		matrix::Dcm<float> R_to_earth;
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
		if (_params.fusion_mode & MASK_USE_EVYAW) {
			// convert the observed quaternion to a rotation matrix
			matrix::Dcm<float> R_to_earth_ev(_ev_sample_delayed.quat);	// transformation matrix from body to world frame
			// calculate the yaw angle for a 312 sequence
			euler312(0) = atan2f(-R_to_earth_ev(0, 1) , R_to_earth_ev(1, 1));
		} else if (_params.mag_fusion_type <= MAG_FUSE_TYPE_3D) {
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
		_state.quat_nominal = Quaternion(R_to_earth);
	}

	// update transformation matrix from body to world frame using the current estimate
	_R_to_earth = quat_to_invrotmat(_state.quat_nominal);

	// update the yaw angle variance using the variance of the measurement
	if (_params.fusion_mode & MASK_USE_EVYAW) {
		// using error estimate from external vision data
		angle_err_var_vec(2) = sq(fmaxf(_ev_sample_delayed.angErr, 1.0e-2f));
	} else if (_params.mag_fusion_type <= MAG_FUSE_TYPE_3D) {
		// using magnetic heading tuning parameter
		angle_err_var_vec(2) = sq(fmaxf(_params.mag_heading_noise, 1.0e-2f));
	}

	// reset the quaternion covariances using the rotation vector variances
	initialiseQuatCovariances(angle_err_var_vec);

	// calculate initial earth magnetic field states
	_state.mag_I = _R_to_earth * mag_init;

	// reset the corresponding rows and columns in the covariance matrix and set the variances on the magnetic field states to the measurement variance
	zeroRows(P, 16, 21);
	zeroCols(P, 16, 21);

	for (uint8_t index = 16; index <= 21; index ++) {
		P[index][index] = sq(_params.mag_noise);
	}

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

	return true;
}

// Calculate the magnetic declination to be used by the alignment and fusion processing
void Ekf::calcMagDeclination()
{
	// set source of magnetic declination for internal use
	if (_params.mag_declination_source & MASK_USE_GEO_DECL) {
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
		_state.gyro_bias(i) = math::constrain(_state.gyro_bias(i), -0.349066f * _dt_ekf_avg, 0.349066f * _dt_ekf_avg);
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
void Ekf::calcEarthRateNED(Vector3f &omega, double lat_rad) const
{
	omega(0) = _k_earth_rate * cosf((float)lat_rad);
	omega(1) = 0.0f;
	omega(2) = -_k_earth_rate * sinf((float)lat_rad);
}

// gets the innovations of velocity and position measurements
// 0-2 vel, 3-5 pos
void Ekf::get_vel_pos_innov(float vel_pos_innov[6])
{
	memcpy(vel_pos_innov, _vel_pos_innov, sizeof(float) * 6);
}

// writes the innovations of the earth magnetic field measurements
void Ekf::get_mag_innov(float mag_innov[3])
{
	memcpy(mag_innov, _mag_innov, 3 * sizeof(float));
}

// gets the innovations of the airspeed measnurement
void Ekf::get_airspeed_innov(float *airspeed_innov)
{
	memcpy(airspeed_innov,&_airspeed_innov, sizeof(float));
}

// gets the innovations of the synthetic sideslip measurements
void Ekf::get_beta_innov(float *beta_innov)
{
	memcpy(beta_innov,&_beta_innov, sizeof(float));
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
	temp[0] = _state.accel_bias(0) /_dt_ekf_avg;
	temp[1] = _state.accel_bias(1) /_dt_ekf_avg;
	temp[2] = _state.accel_bias(2) /_dt_ekf_avg;
	memcpy(bias, temp, 3 * sizeof(float));
}

// get the gyroscope bias in rad/s
void Ekf::get_gyro_bias(float bias[3])
{
	float temp[3];
	temp[0] = _state.gyro_bias(0) /_dt_ekf_avg;
	temp[1] = _state.gyro_bias(1) /_dt_ekf_avg;
	temp[2] = _state.gyro_bias(2) /_dt_ekf_avg;
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

// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
void Ekf::get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv, bool *dead_reckoning)
{
	// report absolute accuracy taking into account the uncertainty in location of the origin
	// If not aiding, return 0 for horizontal position estimate as no estimate is available
	// TODO - allow for baro drift in vertical position error
	float hpos_err;
	float vpos_err;
	bool vel_pos_aiding = (_control_status.flags.gps || _control_status.flags.opt_flow || _control_status.flags.ev_pos);
	if (vel_pos_aiding && _NED_origin_initialised) {
		hpos_err = sqrtf(P[7][7] + P[8][8] + sq(_gps_origin_eph));
		vpos_err = sqrtf(P[9][9] + sq(_gps_origin_epv));

	} else {
		hpos_err = 0.0f;
		vpos_err = 0.0f;

	}

	// If we are dead-reckoning, use the innovations as a conservative alternate measure of the horizontal position error
	// The reason is that complete rejection of measurements is often casued by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_is_dead_reckoning && (_control_status.flags.gps || _control_status.flags.ev_pos)) {
		hpos_err = math::max(hpos_err, sqrtf(_vel_pos_innov[3]*_vel_pos_innov[3] + _vel_pos_innov[4]*_vel_pos_innov[4]));

	}

	memcpy(ekf_eph, &hpos_err, sizeof(float));
	memcpy(ekf_epv, &vpos_err, sizeof(float));
	memcpy(dead_reckoning, &_is_dead_reckoning, sizeof(bool));
}

// get the 1-sigma horizontal and vertical position uncertainty of the ekf local position
void Ekf::get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv, bool *dead_reckoning)
{
	// TODO - allow for baro drift in vertical position error
	float hpos_err;
	float vpos_err;
	bool vel_pos_aiding = (_control_status.flags.gps || _control_status.flags.opt_flow || _control_status.flags.ev_pos);
	if (vel_pos_aiding && _NED_origin_initialised) {
		hpos_err = sqrtf(P[7][7] + P[8][8]);
		vpos_err = sqrtf(P[9][9]);

	} else {
		hpos_err = 0.0f;
		vpos_err = 0.0f;

	}

	// If we are dead-reckoning, use the innovations as a conservative alternate measure of the horizontal position error
	// The reason is that complete rejection of measurements is often casued by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_is_dead_reckoning && (_control_status.flags.gps || _control_status.flags.ev_pos)) {
		hpos_err = math::max(hpos_err, sqrtf(_vel_pos_innov[3]*_vel_pos_innov[3] + _vel_pos_innov[4]*_vel_pos_innov[4]));

	}

	memcpy(ekf_eph, &hpos_err, sizeof(float));
	memcpy(ekf_epv, &vpos_err, sizeof(float));
	memcpy(dead_reckoning, &_is_dead_reckoning, sizeof(bool));
}

// get the 1-sigma horizontal and vertical velocity uncertainty
void Ekf::get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv, bool *dead_reckoning)
{
	float hvel_err;
	float vvel_err;
	bool vel_pos_aiding = (_control_status.flags.gps || _control_status.flags.opt_flow || _control_status.flags.ev_pos);
	if (vel_pos_aiding && _NED_origin_initialised) {
		hvel_err = sqrtf(P[4][4] + P[5][5]);
		vvel_err = sqrtf(P[6][6]);

	} else {
		hvel_err = 0.0f;
		vvel_err = 0.0f;

	}

	// If we are dead-reckoning, use the innovations as a conservative alternate measure of the horizontal velocity error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	float vel_err_conservative = 0.0f;
	if (_is_dead_reckoning) {
		if (_control_status.flags.opt_flow) {
			float gndclearance = math::max(_params.rng_gnd_clearance, 0.1f);
			vel_err_conservative = math::max((_terrain_vpos - _state.pos(2)), gndclearance) * sqrtf(_flow_innov[0]*_flow_innov[0] + _flow_innov[1]*_flow_innov[1]);
		}
		if (_control_status.flags.gps || _control_status.flags.ev_pos) {
			vel_err_conservative = math::max(vel_err_conservative, sqrtf(_vel_pos_innov[0]*_vel_pos_innov[0] + _vel_pos_innov[1]*_vel_pos_innov[1]));
		}
		hvel_err = math::max(hvel_err, vel_err_conservative);
	}

	memcpy(ekf_evh, &hvel_err, sizeof(float));
	memcpy(ekf_evv, &vvel_err, sizeof(float));
	memcpy(dead_reckoning, &_is_dead_reckoning, sizeof(bool));
}

// get EKF innovation consistency check status information comprising of:
// status - a bitmask integer containing the pass/fail status for each EKF measurement innovation consistency check
// Innovation Test Ratios - these are the ratio of the innovation to the acceptance threshold.
// A value > 1 indicates that the sensor measurement has exceeded the maximum acceptable level and has been rejected by the EKF
// Where a measurement type is a vector quantity, eg magnetoemter, GPS position, etc, the maximum value is returned.
void Ekf::get_innovation_test_status(uint16_t *status, float *mag, float *vel, float *pos, float *hgt, float *tas, float *hagl)
{
	// return the integer bitmask containing the consistency check pass/fail satus
	*status = _innov_check_fail_status.value;
	// return the largest magnetometer innovation test ratio
	*mag = sqrtf(math::max(_yaw_test_ratio,math::max(math::max(_mag_test_ratio[0],_mag_test_ratio[1]),_mag_test_ratio[2])));
	// return the largest NED velocity innovation test ratio
	*vel = sqrtf(math::max(math::max(_vel_pos_test_ratio[0],_vel_pos_test_ratio[1]),_vel_pos_test_ratio[2]));
	// return the largest NE position innovation test ratio
	*pos = sqrtf(math::max(_vel_pos_test_ratio[3],_vel_pos_test_ratio[4]));
	// return the vertical position innovation test ratio
	*hgt = sqrtf(_vel_pos_test_ratio[5]);
	// return the airspeed fusion innovation test ratio
	*tas = sqrtf(_tas_test_ratio);
	// return the terrain height innovation test ratio
	*hagl = sqrtf(_terr_test_ratio);
}

// return a bitmask integer that describes which state estimates are valid
void Ekf::get_ekf_soln_status(uint16_t *status)
{
	ekf_solution_status soln_status{};
	soln_status.flags.attitude = _control_status.flags.tilt_align && _control_status.flags.yaw_align && (_fault_status.value == 0);
	soln_status.flags.velocity_horiz = (_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.opt_flow) && (_fault_status.value == 0);
	soln_status.flags.velocity_vert = (_control_status.flags.baro_hgt || _control_status.flags.ev_hgt || _control_status.flags.gps_hgt || _control_status.flags.rng_hgt) && (_fault_status.value == 0);
	soln_status.flags.pos_horiz_rel = (_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.opt_flow) && (_fault_status.value == 0);
	soln_status.flags.pos_horiz_abs = (_control_status.flags.gps || _control_status.flags.ev_pos) && (_fault_status.value == 0);
	soln_status.flags.pos_vert_abs = soln_status.flags.velocity_vert;
	float dummy_var;
	soln_status.flags.pos_vert_agl = get_terrain_vert_pos(&dummy_var);
	soln_status.flags.const_pos_mode = !soln_status.flags.velocity_horiz;
	soln_status.flags.pred_pos_horiz_rel = soln_status.flags.pos_horiz_rel;
	soln_status.flags.pred_pos_horiz_abs = soln_status.flags.pos_vert_abs;
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

bool Ekf::global_position_is_valid()
{
	// return true if the position estimate is valid
	// TODO implement proper check based on published GPS accuracy, innovation consistency checks and timeout status
	return (_NED_origin_initialised && ((_time_last_imu - _time_last_gps) < 5e6) && _control_status.flags.gps);
}

// return true if we are totally reliant on inertial dead-reckoning for position
bool Ekf::inertial_dead_reckoning()
{
	bool velPosAiding = (_control_status.flags.gps || _control_status.flags.ev_pos) && ((_time_last_imu - _time_last_pos_fuse <= 1E6) || (_time_last_imu - _time_last_vel_fuse <= 1E6));
	bool optFlowAiding = _control_status.flags.opt_flow && (_time_last_imu - _time_last_of_fuse <= 1E6);
	bool airDataAiding = _control_status.flags.wind && (_time_last_imu - _time_last_arsp_fuse <= 1E6);

	return !velPosAiding && !optFlowAiding && !airDataAiding;
}

// perform a vector cross product
Vector3f EstimatorInterface::cross_product(const Vector3f &vecIn1, const Vector3f &vecIn2)
{
	Vector3f vecOut;
	vecOut(0) = vecIn1(1)*vecIn2(2) - vecIn1(2)*vecIn2(1);
	vecOut(1) = vecIn1(2)*vecIn2(0) - vecIn1(0)*vecIn2(2);
	vecOut(2) = vecIn1(0)*vecIn2(1) - vecIn1(1)*vecIn2(0);
	return vecOut;
}

// calculate the inverse rotation matrix from a quaternion rotation
Matrix3f EstimatorInterface::quat_to_invrotmat(const Quaternion& quat)
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
	dcm(0,0) = q00 + q11 - q22 - q33;
	dcm(1,1) = q00 - q11 + q22 - q33;
	dcm(2,2) = q00 - q11 - q22 + q33;
	dcm(0,1) = 2.0f * (q12 - q03);
	dcm(0,2) = 2.0f * (q13 + q02);
	dcm(1,0) = 2.0f * (q12 + q03);
	dcm(1,2) = 2.0f * (q23 - q01);
	dcm(2,0) = 2.0f * (q13 - q02);
	dcm(2,1) = 2.0f * (q23 + q01);

	return dcm;
}

// calculate the variances for the rotation vector equivalent
Vector3f Ekf::calcRotVecVariances()
{
	Vector3f rot_var_vec;
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
		zeroRows(P,0,3);
		zeroCols(P,0,3);

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
		P[1][1] = 0.25f*rot_vec_var(0);
		P[1][2] = 0.0f;
		P[1][3] = 0.0f;
		P[2][0] = 0.0f;
		P[2][1] = 0.0f;
		P[2][2] = 0.25f*rot_vec_var(1);
		P[2][3] = 0.0f;
		P[3][0] = 0.0f;
		P[3][1] = 0.0f;
		P[3][2] = 0.0f;
		P[3][3] = 0.25f*rot_vec_var(2);

	}
}
