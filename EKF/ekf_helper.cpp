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
#include <iostream>
#include <fstream>
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
	_state.pos(2) = baro_newest.hgt;
}

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

	for (int i = 0; i < 3; i++) { 
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
