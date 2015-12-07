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

Ekf::Ekf():
	_filter_initialised(false),
	_earth_rate_initialised(false),
	_fuse_height(false),
	_fuse_pos(false),
	_fuse_vel(false)
{
	_earth_rate_NED.setZero();
	_R_prev = matrix::Dcm<float>();
}


Ekf::~Ekf()
{

}

bool Ekf::update()
{
	bool ret = false;	// indicates if there has been an update
	if (!_filter_initialised) {
		_filter_initialised = initialiseFilter();
	}
	printStates();
	// prediction
	if (_imu_updated) {
		ret = true;
		predictState();
		predictCovariance();
	}

	// measurement updates

	if (_mag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_mag_sample_delayed)) {
		fuseHeading();
	}

	if (_baro_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed)) {
		_fuse_height = true;
	}

	if (_gps_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_gps_sample_delayed)) {
		_fuse_pos = true;
		_fuse_vel = true;
	}


	if (_fuse_height || _fuse_pos || _fuse_vel) {
		fuseVelPosHeight();
		_fuse_vel = _fuse_pos = _fuse_height = false;
	}

	if (_range_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_range_sample_delayed)) {
		fuseRange();
	}

	if (_airspeed_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_airspeed_sample_delayed)) {
		fuseAirspeed();
	}

	// write to output if this has been a prediction step
	if (_imu_updated) {
		_output_delayed.vel = _state.vel;
		_output_delayed.pos = _state.pos;
		_output_delayed.quat_nominal = _state.quat_nominal;
		_output_delayed.time_us = _imu_time_last;
		_imu_updated = false;
	}

	return ret;
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
	if (!_earth_rate_initialised) {
		if (_gps_initialised) {
			calcEarthRateNED(_earth_rate_NED, _posRef.lat_rad );
			_earth_rate_initialised = true;
		}
	}

	// attitude error state prediciton
	matrix::Dcm<float> R_to_earth(_state.quat_nominal);	// transformation matrix from body to world frame
	Vector3f corrected_delta_ang = _imu_sample_delayed.delta_ang - _R_prev * _earth_rate_NED * _imu_sample_delayed.delta_ang_dt;
	Quaternion dq;	// delta quaternion since last update
	dq.from_axis_angle(corrected_delta_ang);
	_state.quat_nominal = dq * _state.quat_nominal;
	_state.quat_nominal.normalize();

	_R_prev = R_to_earth.transpose();

	Vector3f vel_last = _state.vel;

	// predict velocity states
	_state.vel += R_to_earth * _imu_sample_delayed.delta_vel;
	_state.vel(2) += 9.81f * _imu_sample_delayed.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_state.pos += (vel_last + _state.vel) * _imu_sample_delayed.delta_vel_dt * 0.5f;

	constrainStates();
}


void Ekf::fuseAirspeed()
{

}

void Ekf::fuseRange()
{

}

void Ekf::printStates()
{
	static int counter = 0;

	if (counter % 50 == 0) {
		printf("ang error\n");
		for(int i = 0; i < 3; i++) {
			printf("ang_e %i %.5f\n", i, (double)_state.ang_error(i));
		}

		matrix::Euler<float> euler(_state.quat_nominal);
		printf("yaw pitch roll %.5f %.5f %.5f\n", (double)euler(2), (double)euler(1), (double)euler(0));

		printf("vel\n");
		for(int i = 0; i < 3; i++) {
			printf("v %i %.5f\n", i, (double)_state.vel(i));
		}

		printf("pos\n");
		for(int i = 0; i < 3; i++) {
			printf("p %i %.5f\n", i, (double)_state.pos(i));
		}

		printf("g gyro_bias\n");
		for(int i = 0; i < 3; i++) {
			printf("gb %i %.5f\n", i, (double)_state.gyro_bias(i));
		}

		printf("gyro_scale\n");
		for(int i = 0; i < 3; i++) {
			printf("gs %i %.5f\n", i, (double)_state.gyro_scale(i));
		}

		printf("mag_I\n");
		for(int i = 0; i < 3; i++) {
			printf("mI %i %.5f\n", i, (double)_state.mag_I(i));
		}
		counter = 0;
	}
	counter++;

}
