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
 * @file heading_fusion.cpp
 * Magnetometer heading fusion.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */
#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::fuseHeading()
{
	// assign intermediate state variables
	float q0 = _state.quat_nominal(0);
	float q1 = _state.quat_nominal(1);
	float q2 = _state.quat_nominal(2);
	float q3 = _state.quat_nominal(3);

	float magX = _mag_sample_delayed.mag(0);
	float magY = _mag_sample_delayed.mag(1);
	float magZ = _mag_sample_delayed.mag(2);

	float R_mag = _params.mag_heading_noise;	

	float t2 = q0*q0;
	float t3 = q1*q1;
	float t4 = q2*q2;
	float t5 = q3*q3;
	float t6 = q0*q2*2.0f;
	float t7 = q1*q3*2.0f;
	float t8 = t6+t7;
	float t9 = q0*q3*2.0f;
	float t13 = q1*q2*2.0f;
	float t10 = t9-t13;
	float t11 = t2+t3-t4-t5;
	float t12 = magX*t11;
	float t14 = magZ*t8;
	float t19 = magY*t10;
	float t15 = t12+t14-t19;
	float t16 = t2-t3+t4-t5;
	float t17 = q0*q1*2.0f;
	float t24 = q2*q3*2.0f;
	float t18 = t17-t24;
	float t20 = 1.0f/t15;
	float t21 = magY*t16;
	float t22 = t9+t13;
	float t23 = magX*t22;
	float t28 = magZ*t18;
	float t25 = t21+t23-t28;
	float t29 = t20*t25;
	float t26 = tan(t29);
	float t27 = 1.0f/(t15*t15);
	float t30 = t26*t26;
	float t31 = t30+1.0f;

	float H_MAG[3] = {};
	H_MAG[0] = -t31*(t20*(magZ*t16+magY*t18)+t25*t27*(magY*t8+magZ*t10));
	H_MAG[1] = t31*(t20*(magX*t18+magZ*t22)+t25*t27*(magX*t8-magZ*t11));
	H_MAG[2] = t31*(t20*(magX*t16-magY*t22)+t25*t27*(magX*t10+magY*t11));

	// calculate innovation
	matrix::Dcm<float> R_to_earth(_state.quat_nominal);
	matrix::Vector3f mag_earth_pred = R_to_earth * _mag_sample_delayed.mag;

	float innovation = atan2f(mag_earth_pred(1), mag_earth_pred(0)) - math::radians(_params.mag_declination_deg);

	innovation = math::constrain(innovation, -0.5f, 0.5f);

	float innovation_var = R_mag;

	// calculate innovation variance
	float PH[3] = {};
	for (unsigned row = 0; row < 3; row++) {
		for (unsigned column = 0; column < 3; column++) {
			PH[row] += P[row][column]*H_MAG[column];
		}
		innovation_var += H_MAG[row] * PH[row];
	}

	if (innovation_var >= R_mag) {
		// variance has increased, no failure
		_fault_status.bad_mag_x = false;
		_fault_status.bad_mag_y = false;
		_fault_status.bad_mag_z = false;
	} else {
		// our innovation variance has decreased, our calculation is thus badly conditioned
		_fault_status.bad_mag_x = true;
		_fault_status.bad_mag_y = true;
		_fault_status.bad_mag_z = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		return;
	}

	float innovation_var_inv = 1 / innovation_var;

	// calculate kalman gain
	float Kfusion[_k_num_states] = {};
	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column < 3; column++) {
			Kfusion[row] += P[row][column] * H_MAG[column];
		}
		Kfusion[row] *= innovation_var_inv;
	}

	// innovation test ratio
	float yaw_test_ratio = sq(innovation) / (sq(math::max(0.01f * (float)_params.heading_innov_gate, 1.0f)) * innovation_var);

	// set the magnetometer unhealthy if the test fails
	if (yaw_test_ratio > 1.0f) {
		_mag_healthy = false;

		// if we are in air we don't want to fuse the measurement
		// we allow to use it when on the ground because the large innovation could be caused
		// by interference or a large initial gyro bias
		if (_in_air) {
			return;
		}
	} else {
		_mag_healthy = true;
	}

	_state.ang_error.setZero();
	fuse(Kfusion, innovation);

	// correct the nominal quaternion
 	Quaternion dq;
	dq.from_axis_angle(_state.ang_error);
	_state.quat_nominal = dq * _state.quat_nominal;
	_state.quat_nominal.normalize();

	float HP[_k_num_states] = {};

	for (unsigned column = 0; column < _k_num_states; column++) {
		for (unsigned row = 0; row < 3; row++) {
			HP[column] += H_MAG[row] * P[row][column];
		}
	}

	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column < _k_num_states; column++) {
			P[row][column] -= Kfusion[row] * HP[column];
		}
	}

	makeSymmetrical();
	limitCov();
}