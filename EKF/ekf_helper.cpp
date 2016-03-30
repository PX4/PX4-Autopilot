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
	// if we have a valid GPS measurement use it to initialise velocity states
	gpsSample gps_newest = _gps_buffer.get_newest();

	if (_time_last_imu - gps_newest.time_us < 400000) {
		_state.vel = gps_newest.vel;
		return true;

	} else {
		// XXX use the value of the last known velocity
		return false;
	}
}

// Reset position states. If we have a recent and valid
// gps measurement then use for position initialisation
bool Ekf::resetPosition()
{
	// if we have a fresh GPS measurement, use it to initialise position states and correct the position for the measurement delay
	gpsSample gps_newest = _gps_buffer.get_newest();

	float time_delay = 1e-6f * (float)(_time_last_imu - gps_newest.time_us);

	if (time_delay < 0.4f) {
		_state.pos(0) = gps_newest.pos(0) + gps_newest.vel(0) * time_delay;
		_state.pos(1) = gps_newest.pos(1) + gps_newest.vel(1) * time_delay;
		return true;

	} else {
		// XXX use the value of the last known position
		return false;
	}
}

// Reset height state using the last height measurement
void Ekf::resetHeight()
{
	if (_control_status.flags.rng_hgt) {
		rangeSample range_newest = _range_buffer.get_newest();

		if (_time_last_imu - range_newest.time_us < 2 * RNG_MAX_INTERVAL) {
			_state.pos(2) = _hgt_sensor_offset - range_newest.rng;

		} else {
			// TODO: reset to last known range based estimate
		}

		// reset the baro offset which is subtracted from the baro reading if we need to use it as a backup
		baroSample baro_newest = _baro_buffer.get_newest();
		_baro_hgt_offset = baro_newest.hgt + _state.pos(2);

	} else if (_control_status.flags.baro_hgt) {
		// initialize vertical position with newest baro measurement
		baroSample baro_newest = _baro_buffer.get_newest();

		if (_time_last_imu - baro_newest.time_us < 2 * BARO_MAX_INTERVAL) {
			_state.pos(2) = _hgt_sensor_offset - baro_newest.hgt + _baro_hgt_offset;

		} else {
			// TODO: reset to last known baro based estimate
		}

	} else if (_control_status.flags.gps_hgt) {
		// initialize vertical position and velocity with newest gps measurement
		gpsSample gps_newest = _gps_buffer.get_newest();

		if (_time_last_imu - gps_newest.time_us < 2 * GPS_MAX_INTERVAL) {
			_state.pos(2) = _hgt_sensor_offset - gps_newest.hgt + _gps_alt_ref;
			_state.vel(2) = gps_newest.vel(2);

		} else {
			// TODO: reset to last known gps based estimate
		}

		// reset the baro offset which is subtracted from the baro reading if we need to use it as a backup
		baroSample baro_newest = _baro_buffer.get_newest();
		_baro_hgt_offset = baro_newest.hgt + _state.pos(2);
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

	// reset the angle error variances because the yaw angle could have changed by a significant amount
	// by setting them to zero we avoid 'kicks' in angle when 3-D fusion starts and the imu process noise
	// will grow them again.
	zeroRows(P, 0, 2);
	zeroCols(P, 0, 2);

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
void Ekf::makeSymmetrical()
{
	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column < row; column++) {
			float tmp = (P[row][column] + P[column][row]) / 2;
			P[row][column] = tmp;
			P[column][row] = tmp;
		}
	}
}

void Ekf::constrainStates()
{
	for (int i = 0; i < 3; i++) {
		_state.ang_error(i) = math::constrain(_state.ang_error(i), -1.0f, 1.0f);
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
		_state.gyro_scale(i) = math::constrain(_state.gyro_scale(i), 0.95f, 1.05f);
	}

	_state.accel_z_bias = math::constrain(_state.accel_z_bias, -1.0f * _dt_imu_avg, 1.0f * _dt_imu_avg);

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

// get the state vector at the delayed time horizon
void Ekf::get_state_delayed(float *state)
{
	for (int i = 0; i < 3; i++) {
		state[i] = _state.ang_error(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 3] = _state.vel(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 6] = _state.pos(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 9] = _state.gyro_bias(i);
	}

	for (int i = 0; i < 3; i++) {
		state[i + 12] = _state.gyro_scale(i);
	}

	state[15] = _state.accel_z_bias;

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
	float temp1 = sqrtf(P[6][6] + P[7][7] + sq(_gps_origin_eph));
	float temp2 = sqrtf(P[8][8] + sq(_gps_origin_epv));
	memcpy(ekf_eph, &temp1, sizeof(float));
	memcpy(ekf_epv, &temp2, sizeof(float));

	// report dead reckoning if it is more than a second since we fused in GPS
	bool temp3 = (_time_last_imu - _time_last_pos_fuse > 1e6);
	memcpy(dead_reckoning, &temp3, sizeof(bool));
}

// fuse measurement
void Ekf::fuse(float *K, float innovation)
{
	for (unsigned i = 0; i < 3; i++) {
		_state.ang_error(i) = _state.ang_error(i) - K[i] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.vel(i) = _state.vel(i) - K[i + 3] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.pos(i) = _state.pos(i) - K[i + 6] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.gyro_bias(i) = _state.gyro_bias(i) - K[i + 9] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.gyro_scale(i) = _state.gyro_scale(i) - K[i + 12] * innovation;
	}

	_state.accel_z_bias -= K[15] * innovation;

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
