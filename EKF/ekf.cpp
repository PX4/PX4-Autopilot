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
 * @file ekf.cpp
 * Core functions for ekf attitude and position estimator.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#include "ekf.h"
#include <drivers/drv_hrt.h>

Ekf::Ekf()
{

}


Ekf::~Ekf()
{

}

void Ekf::update()
{
	if (!_filter_initialised) {
		_filter_initialised = initialiseFilter();
	}

	// prediction
	if (_imu_updated) {
		predictState();
		predictCovariance();
		_imu_updated = false;
	}

	// measurement updates
	if (_mag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_mag_sample_delayed)) {
		fuseMag();
	}

	if (_baro_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed)) {
		_fuse_height = true;
	}

	if (_gps_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_gps_sample_delayed)) {
		_fuse_pos = true;
		_fuse_vel = true;
	}

	if (_fuse_height || _fuse_pos || _fuse_vel) {
		fusePosVel();
	}

	if (_range_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_range_sample_delayed)) {
		fuseRange();
	}

	if (_airspeed_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_airspeed_sample_delayed)) {
		fuseAirspeed();
	}

}

bool Ekf::initialiseFilter(void)
{

	_state.ang_error.setZero();
	_state.vel.setZero();
	_state.pos.setZero();
	_state.gyro_bias.setZero();
	_state.gyro_scale(0) = _state.gyro_scale(1) = _state.gyro_scale(2) = 1.0f;
	_state.accel_z_bias = 0.0f;
	_state.mag_I.setZero();
	_state.mag_B.setZero();
	_state.wind_vel.setZero();

	// get initial attitude estimate from accel vector, assuming vehicle is static
	Vector3f accel_init = _imu_down_sampled.delta_vel / _imu_down_sampled.delta_vel_dt;

	float pitch = 0.0f;
	float roll = 0.0f;

	if (accel_init.norm() > 0.001f) {
		accel_init.normalize();

		pitch = asinf(accel_init(0));
		roll = -asinf(accel_init(1) / cosf(pitch));
	}

	matrix::Euler<float> euler_init(0, pitch, roll);
	_state.quat_nominal = Quaternion(euler_init);

	resetVelocity();
	resetPosition();

	initialiseCovariance();

	return true;
}

void Ekf::predictState()
{
	// compute transformation matrix from body to world frame
	matrix::Dcm<float> R(_state.quat_nominal);
	R.transpose();

	// attitude error state prediciton
	Quaternion dq;
	dq.from_axis_angle(_imu_sample_delayed.delta_ang);
	_state.quat_nominal = dq * _state.quat_nominal;
	_state.quat_nominal.normalize();

	Vector3f vel_last = _state.vel;

	// predict velocity states
	_state.vel += R * _imu_sample_delayed.delta_vel;
	_state.vel(2) += 9.81f * _imu_sample_delayed.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_state.pos += (vel_last + _state.vel) * _imu_sample_delayed.delta_vel_dt * 0.5f;

	//matrix::Euler<float> euler(_state.quat_nominal);
	//printf("roll pitch yaw %.5f %.5f %.5f\n", (double)euler(2), (double)euler(1), (double)euler(0));
}


void Ekf::fusePosVel()
{

}

void Ekf::fuseMag()
{
}

void Ekf::fuseAirspeed()
{

}

void Ekf::fuseRange()
{

}
