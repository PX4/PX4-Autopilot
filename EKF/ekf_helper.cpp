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
#include <mathlib/mathlib.h>

// Reset the velocity states. If we have a recent and valid
// gps measurement then use for velocity initialisation
void Ekf::resetVelocity()
{
	// if we have a valid GPS measurement use it to initialise velocity states
	gpsSample gps_newest = _gps_buffer.get_newest();

	if (_time_last_imu - gps_newest.time_us < 100000) {
		_state.vel = gps_newest.vel;

	} else {
		_state.vel.setZero();
	}
}

// Reset position states. If we have a recent and valid
// gps measurement then use for position initialisation
void Ekf::resetPosition()
{
	// if we have a valid GPS measurement use it to initialise position states
	gpsSample gps_newest = _gps_buffer.get_newest();

	if (_time_last_imu - gps_newest.time_us < 100000) {
		_state.pos(0) = gps_newest.pos(0);
		_state.pos(1) = gps_newest.pos(1);

	} else {
		// XXX use the value of the last known position
	}

	baroSample baro_newest = _baro_buffer.get_newest();
	_state.pos(2) = -baro_newest.hgt;
}

#if defined(__PX4_POSIX) && !defined(__PX4_QURT)
void Ekf::printCovToFile(char const *filename)
{
	std::ofstream myfile;
	myfile.open(filename);
	myfile << "Covariance matrix\n";
	myfile << std::setprecision(1);

	for (int i = 0; i < _k_num_states; i++) {
		for (int j = 0; j < _k_num_states; j++) {
			myfile << std::to_string(P[i][j]) << std::setprecision(1) << "          ";
		}

		myfile << "\n\n\n\n\n\n\n\n\n\n";
	}
}
#endif

// This checks if the diagonal of the covariance matrix is non-negative
// and that the matrix is symmetric
void Ekf::assertCovNiceness()
{
	for (int row = 0; row < _k_num_states; row++) {
		for (int column = 0; column < row; column++) {
			assert(fabsf(P[row][column] - P[column][row]) < 0.00001f);
		}
	}

	for (int i = 0; i < _k_num_states; i++) {
		assert(P[i][i] > -0.000001f);
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
