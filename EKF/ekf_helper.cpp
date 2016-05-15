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
#ifdef __PX4_POSIX
#include <iostream>
#include <fstream>
#endif
#include <iomanip>
#include "mathlib.h"

// Reset the velocity states. If we have a recent and valid
// gps measurement then use for velocity initialisation
bool Ekf::resetVelocity()
{
	if (_control_status.flags.gps) {
		// if we have a valid GPS measurement use it to initialise velocity states
		gpsSample gps_newest = _gps_buffer.get_newest();

		if (_time_last_imu - gps_newest.time_us < 2*GPS_MAX_INTERVAL) {
			_state.vel = gps_newest.vel;
			return true;

		} else {
			// XXX use the value of the last known velocity
			return false;
		}
	} else if (_control_status.flags.opt_flow || _control_status.flags.ev_pos) {
		_state.vel.setZero();
		return true;
	} else {
		return false;
	}
}

// Reset position states. If we have a recent and valid
// gps measurement then use for position initialisation
bool Ekf::resetPosition()
{
	if (_control_status.flags.gps) {
		// if we have a fresh GPS measurement, use it to initialise position states and correct the position for the measurement delay
		gpsSample gps_newest = _gps_buffer.get_newest();

		float time_delay = 1e-6f * (float)(_imu_sample_delayed.time_us - gps_newest.time_us);
		float max_time_delay = 1e-6f * (float)GPS_MAX_INTERVAL;

		if (time_delay < max_time_delay) {
			_state.pos(0) = gps_newest.pos(0) + gps_newest.vel(0) * time_delay;
			_state.pos(1) = gps_newest.pos(1) + gps_newest.vel(1) * time_delay;
			return true;

		} else {
			// XXX use the value of the last known position
			return false;
		}
	} else if (_control_status.flags.opt_flow) {
		_state.pos(0) = 0.0f;
		_state.pos(1) = 0.0f;
		return true;
	} else if (_control_status.flags.ev_pos) {
		// if we have fresh data, reset the position to the measurement
		extVisionSample ev_newest = _ext_vision_buffer.get_newest();
		if (_time_last_imu - ev_newest.time_us < 2*EV_MAX_INTERVAL) {
			// use the most recent data if it's time offset from the fusion time horizon is smaller
			int32_t dt_newest = ev_newest.time_us - _imu_sample_delayed.time_us;
			int32_t dt_delayed = _ev_sample_delayed.time_us - _imu_sample_delayed.time_us;
			if (abs(dt_newest) < abs(dt_delayed)) {
				_state.pos(0) = ev_newest.posNED(0);
				_state.pos(1) = ev_newest.posNED(1);
			} else {
				_state.pos(0) = _ev_sample_delayed.posNED(0);
				_state.pos(1) = _ev_sample_delayed.posNED(1);
			}
			return true;

		} else {
			// XXX use the value of the last known position
			return false;
		}

	} else {
		return false;
	}
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

	} else if (_control_status.flags.ev_pos) {
		// initialize vertical position with newest measurement
		extVisionSample ev_newest = _ext_vision_buffer.get_newest();

		if (_time_last_imu - ev_newest.time_us < 2 * EV_MAX_INTERVAL) {
			_state.pos(2) = ev_newest.posNED(2);

		} else {
			// TODO: reset to last known baro based estimate
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
		_vert_pos_reset_delta = _state.pos(2) - old_vert_pos;
		_time_vert_pos_reset = _time_last_imu;
	}

	if (vert_vel_reset) {
		_vert_vel_reset_delta = _state.vel(2) - old_vert_vel;
		_time_vert_vel_reset = _time_last_imu;
	}

	// add the reset amount to the output observer states
	_output_new.pos(2) += _vert_pos_reset_delta;
	_output_new.vel(2) += _vert_vel_reset_delta;

	// add the reset amount to the output observer buffered data
	outputSample output_states;
	unsigned output_length = _output_buffer.get_length();
	for (unsigned i=0; i < output_length; i++) {
		output_states = _output_buffer.get_from_index(i);

		if (vert_pos_reset) {
			output_states.pos(2) += _vert_pos_reset_delta;
		}

		if (vert_vel_reset) {
			output_states.vel(2) += _vert_vel_reset_delta;
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
	// If we don't a tilt estimate then we cannot initialise the yaw
	if (!_control_status.flags.tilt_align) {
		return false;
	}

	// get the roll, pitch, yaw estimates and set the yaw to zero
	matrix::Quaternion<float> q(_state.quat_nominal(0), _state.quat_nominal(1), _state.quat_nominal(2),
				    _state.quat_nominal(3));
	matrix::Euler<float> euler_init(q);
	euler_init(2) = 0.0f;

	// rotate the magnetometer measurements into earth axes
	matrix::Dcm<float> R_to_earth_zeroyaw(euler_init);
	Vector3f mag_ef_zeroyaw = R_to_earth_zeroyaw * mag_init;
	euler_init(2) = _mag_declination - atan2f(mag_ef_zeroyaw(1), mag_ef_zeroyaw(0));

	// calculate initial quaternion states for the ekf
	// we don't change the output attitude to avoid jumps
	_state.quat_nominal = Quaternion(euler_init);

	// reset the quaternion variances because the yaw angle could have changed by a significant amount
	// by setting them to zero we avoid 'kicks' in angle when 3-D fusion starts and the imu process noise
	// will grow them again.
	zeroRows(P, 0, 3);
	zeroCols(P, 0, 3);

	// calculate initial earth magnetic field states
	matrix::Dcm<float> R_to_earth(euler_init);
	_state.mag_I = R_to_earth * mag_init;

	// reset the corresponding rows and columns in the covariance matrix and set the variances on the magnetic field states to the measurement variance
	zeroRows(P, 16, 21);
	zeroCols(P, 16, 21);

	for (uint8_t index = 16; index <= 21; index ++) {
		P[index][index] = sq(_params.mag_noise);
	}

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
		_state.gyro_bias(i) = math::constrain(_state.gyro_bias(i), -0.349066f * _dt_imu_avg, 0.349066f * _dt_imu_avg);
	}

	for (int i = 0; i < 3; i++) {
		_state.accel_bias(i) = math::constrain(_state.accel_bias(i), -1.0f * _dt_imu_avg, 1.0f * _dt_imu_avg);
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

// get the diagonal elements of the covariance matrix
void Ekf::get_covariances(float *covariances)
{
	for (unsigned i = 0; i < _k_num_states; i++) {
		covariances[i] = P[i][i];
	}
}

// get the position and height of the ekf origin in WGS-84 coordinates and time the origin was set
void Ekf::get_ekf_origin(uint64_t *origin_time, map_projection_reference_s *origin_pos, float *origin_alt)
{
	memcpy(origin_time, &_last_gps_origin_time_us, sizeof(uint64_t));
	memcpy(origin_pos, &_pos_ref, sizeof(map_projection_reference_s));
	memcpy(origin_alt, &_gps_alt_ref, sizeof(float));
}

// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
void Ekf::get_ekf_accuracy(float *ekf_eph, float *ekf_epv, bool *dead_reckoning)
{
	// report absolute accuracy taking into account the uncertainty in location of the origin
	// TODO we a need a way to allow for baro drift error
	float temp1 = sqrtf(P[7][7] + P[8][8] + sq(_gps_origin_eph));
	float temp2 = sqrtf(P[9][9] + sq(_gps_origin_epv));
	memcpy(ekf_eph, &temp1, sizeof(float));
	memcpy(ekf_epv, &temp2, sizeof(float));

	// report dead reckoning if it is more than a second since we fused in GPS
	bool temp3 = (_time_last_imu - _time_last_pos_fuse > 1e6);
	memcpy(dead_reckoning, &temp3, sizeof(bool));
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
Matrix3f EstimatorInterface::quat_to_invrotmat(const Quaternion quat)
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
