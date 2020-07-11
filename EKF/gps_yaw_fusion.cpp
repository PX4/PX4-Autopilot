/****************************************************************************
 *
 *   Copyright (c) 2018 Estimation and Control Library (ECL). All rights reserved.
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
 * @file gps_yaw_fusion.cpp
 * Definition of functions required to use yaw obtained from GPS dual antenna measurements.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <ecl.h>
#include <mathlib/mathlib.h>
#include <cstdlib>

void Ekf::fuseGpsYaw()
{
	// assign intermediate state variables
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	// calculate the observed yaw angle of antenna array, converting a from body to antenna yaw measurement
	const float measured_hdg = wrap_pi(_gps_sample_delayed.yaw + _gps_yaw_offset);

	// define the predicted antenna array vector and rotate into earth frame
	const Vector3f ant_vec_bf = {cosf(_gps_yaw_offset), sinf(_gps_yaw_offset), 0.0f};
	const Vector3f ant_vec_ef = _R_to_earth * ant_vec_bf;

	// check if antenna array vector is within 30 degrees of vertical and therefore unable to provide a reliable heading
	if (fabsf(ant_vec_ef(2)) > cosf(math::radians(30.0f)))  {
		return;
	}

	// calculate predicted antenna yaw angle
	const float predicted_hdg =  atan2f(ant_vec_ef(1),ant_vec_ef(0));

	// calculate observation jacobian
	float t2 = sinf(_gps_yaw_offset);
	float t3 = cosf(_gps_yaw_offset);
	float t4 = q0*q3*2.0f;
	float t5 = q0*q0;
	float t6 = q1*q1;
	float t7 = q2*q2;
	float t8 = q3*q3;
	float t9 = q1*q2*2.0f;
	float t10 = t5+t6-t7-t8;
	float t11 = t3*t10;
	float t12 = t4+t9;
	float t13 = t3*t12;
	float t14 = t5-t6+t7-t8;
	float t15 = t2*t14;
	float t16 = t13+t15;
	float t17 = t4-t9;
	float t19 = t2*t17;
	float t20 = t11-t19;
	float t18 = (t20*t20);
	if (t18 < 1e-6f) {
		return;
	}
	t18 = 1.0f / t18;
	float t21 = t16*t16;
	float t22 = sq(t11-t19);
	if (t22 < 1e-6f) {
		return;
	}
	t22 = 1.0f/t22;
	float t23 = q1*t3*2.0f;
	float t24 = q2*t2*2.0f;
	float t25 = t23+t24;
	float t26 = 1.0f/t20;
	float t27 = q1*t2*2.0f;
	float t28 = t21*t22;
	float t29 = t28+1.0f;
	if (fabsf(t29) < 1e-6f) {
		return;
	}
	float t30 = 1.0f/t29;
	float t31 = q0*t3*2.0f;
	float t32 = t31-q3*t2*2.0f;
	float t33 = q3*t3*2.0f;
	float t34 = q0*t2*2.0f;
	float t35 = t33+t34;

	float H_YAW[4];
	H_YAW[0] = (t35/(t11-t2*(t4-q1*q2*2.0f))-t16*t18*t32)/(t18*t21+1.0f);
	H_YAW[1] = -t30*(t26*(t27-q2*t3*2.0f)+t16*t22*t25);
	H_YAW[2] = t30*(t25*t26-t16*t22*(t27-q2*t3*2.0f));
	H_YAW[3] = t30*(t26*t32+t16*t22*t35);

	// using magnetic heading tuning parameter
	const float R_YAW = sq(fmaxf(_params.mag_heading_noise, 1.0e-2f));

	// calculate the innovation and define the innovation gate
	const float innov_gate = math::max(_params.heading_innov_gate, 1.0f);
	_heading_innov = predicted_hdg - measured_hdg;

	// wrap the innovation to the interval between +-pi
	_heading_innov = wrap_pi(_heading_innov);

	// Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 3 elements in H are non zero
	// calculate the innovation variance
	float PH[4];
	_heading_innov_var = R_YAW;

	for (unsigned row = 0; row <= 3; row++) {
		PH[row] = 0.0f;

		for (uint8_t col = 0; col <= 3; col++) {
			PH[row] += P(row,col) * H_YAW[col];
		}

		_heading_innov_var += H_YAW[row] * PH[row];
	}

	// check if the innovation variance calculation is badly conditioned
	if (_heading_innov_var >= R_YAW) {
		// the innovation variance contribution from the state covariances is not negative, no fault
		_fault_status.flags.bad_hdg = false;

	} else {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		ECL_ERR_TIMESTAMPED("GPS yaw numerical error - covariance reset");
		return;
	}

	const float heading_innov_var_inv = 1.f / _heading_innov_var;

	// calculate the Kalman gains
	// only calculate gains for states we are using
	Vector24f Kfusion;

	for (uint8_t row = 0; row <= 15; row++) {
		for (uint8_t col = 0; col <= 3; col++) {
			Kfusion(row) += P(row,col) * H_YAW[col];
		}

		Kfusion(row) *= heading_innov_var_inv;
	}

	if (_control_status.flags.wind) {
		for (uint8_t row = 22; row <= 23; row++) {
			for (uint8_t col = 0; col <= 3; col++) {
				Kfusion(row) += P(row,col) * H_YAW[col];
			}

			Kfusion(row) *= heading_innov_var_inv;
		}
	}

	// innovation test ratio
	_yaw_test_ratio = sq(_heading_innov) / (sq(innov_gate) * _heading_innov_var);

	// we are no longer using 3-axis fusion so set the reported test levels to zero
	_mag_test_ratio.setZero();

	if (_yaw_test_ratio > 1.0f) {
		_innov_check_fail_status.flags.reject_yaw = true;

		// if we are in air we don't want to fuse the measurement
		// we allow to use it when on the ground because the large innovation could be caused
		// by interference or a large initial gyro bias
		if (_control_status.flags.in_air) {
			return;

		} else {
			// constrain the innovation to the maximum set by the gate
			const float gate_limit = sqrtf((sq(innov_gate) * _heading_innov_var));
			_heading_innov = math::constrain(_heading_innov, -gate_limit, gate_limit);
		}

	} else {
		_innov_check_fail_status.flags.reject_yaw = false;
	}

	// apply covariance correction via P_new = (I -K*H)*P
	// first calculate expression for KHP
	// then calculate P - KHP
	SquareMatrix24f KHP;
	float KH[4];

	for (unsigned row = 0; row < _k_num_states; row++) {

		KH[0] = Kfusion(row) * H_YAW[0];
		KH[1] = Kfusion(row) * H_YAW[1];
		KH[2] = Kfusion(row) * H_YAW[2];
		KH[3] = Kfusion(row) * H_YAW[3];

		for (unsigned column = 0; column < _k_num_states; column++) {
			float tmp = KH[0] * P(0,column);
			tmp += KH[1] * P(1,column);
			tmp += KH[2] * P(2,column);
			tmp += KH[3] * P(3,column);
			KHP(row,column) = tmp;
		}
	}

	const bool healthy = checkAndFixCovarianceUpdate(KHP);

	_fault_status.flags.bad_hdg = !healthy;

	if (healthy) {
		// apply the covariance corrections
		P -= KHP;

		fixCovarianceErrors(true);

		// apply the state corrections
		fuse(Kfusion, _heading_innov);

		_time_last_gps_yaw_fuse = _time_last_imu;
	}
}

bool Ekf::resetYawToGps()
{
	// define the predicted antenna array vector and rotate into earth frame
	const Vector3f ant_vec_bf = {cosf(_gps_yaw_offset), sinf(_gps_yaw_offset), 0.0f};
	const Vector3f ant_vec_ef = _R_to_earth * ant_vec_bf;

	// check if antenna array vector is within 30 degrees of vertical and therefore unable to provide a reliable heading
	if (fabsf(ant_vec_ef(2)) > cosf(math::radians(30.0f)))  {
		return false;
	}

	// GPS yaw measurement is alreday compensated for antenna offset in the driver
	const float measured_yaw = _gps_sample_delayed.yaw;

	const float yaw_variance = sq(fmaxf(_params.mag_heading_noise, 1.0e-2f));
	resetQuatStateYaw(measured_yaw, yaw_variance, true);

	_time_last_gps_yaw_fuse = _time_last_imu;

	return true;
}
