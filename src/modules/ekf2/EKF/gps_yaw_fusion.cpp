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
 * Equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <cstdlib>

void Ekf::fuseGpsYaw()
{
	// assign intermediate state variables
	const float &q0 = _state.quat_nominal(0);
	const float &q1 = _state.quat_nominal(1);
	const float &q2 = _state.quat_nominal(2);
	const float &q3 = _state.quat_nominal(3);

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
	const float predicted_hdg = atan2f(ant_vec_ef(1), ant_vec_ef(0));

	// using magnetic heading process noise
	// TODO extend interface to use yaw uncertainty provided by GPS if available
	const float R_YAW = sq(fmaxf(_params.gps_heading_noise, 1.0e-2f));

	// calculate intermediate variables
	const float HK0 = sinf(_gps_yaw_offset);
	const float HK1 = q0*q3;
	const float HK2 = q1*q2;
	const float HK3 = 2*HK0*(HK1 - HK2);
	const float HK4 = cosf(_gps_yaw_offset);
	const float HK5 = ecl::powf(q1, 2);
	const float HK6 = ecl::powf(q2, 2);
	const float HK7 = ecl::powf(q0, 2) - ecl::powf(q3, 2);
	const float HK8 = HK4*(HK5 - HK6 + HK7);
	const float HK9 = HK3 - HK8;

	if (fabsf(HK9) < 1e-3f) {
		return;
	}
	const float HK10 = 1.0F/HK9;
	const float HK11 = HK4*q0;
	const float HK12 = HK0*q3;
	const float HK13 = HK0*(-HK5 + HK6 + HK7) + 2*HK4*(HK1 + HK2);
	const float HK14 = HK10*HK13;
	const float HK15 = HK0*q0 + HK4*q3;
	const float HK16 = HK10*(HK14*(HK11 - HK12) + HK15);
	const float HK17 = ecl::powf(HK13, 2)/ecl::powf(HK9, 2) + 1;
	if (fabsf(HK17) < 1e-3f) {
		return;
	}
	const float HK18 = 2/HK17;
	// const float HK19 = 1.0F/(-HK3 + HK8);
	const float HK19_inverse = -HK3 + HK8;

	if (fabsf(HK19_inverse) < 1e-6f) {
		return;
	}
	const float HK19 = 1.0F/HK19_inverse;
	const float HK20 = HK4*q1;
	const float HK21 = HK0*q2;
	const float HK22 = HK13*HK19;
	const float HK23 = HK0*q1 - HK4*q2;
	const float HK24 = HK19*(HK22*(HK20 + HK21) + HK23);
	const float HK25 = HK19*(-HK20 - HK21 + HK22*HK23);
	const float HK26 = HK10*(-HK11 + HK12 + HK14*HK15);
	const float HK27 = -HK16*P(0,0) - HK24*P(0,1) - HK25*P(0,2) + HK26*P(0,3);
	const float HK28 = -HK16*P(0,1) - HK24*P(1,1) - HK25*P(1,2) + HK26*P(1,3);
	const float HK29 = 4/ecl::powf(HK17, 2);
	const float HK30 = -HK16*P(0,2) - HK24*P(1,2) - HK25*P(2,2) + HK26*P(2,3);
	const float HK31 = -HK16*P(0,3) - HK24*P(1,3) - HK25*P(2,3) + HK26*P(3,3);
	// const float HK32 = HK18/(-HK16*HK27*HK29 - HK24*HK28*HK29 - HK25*HK29*HK30 + HK26*HK29*HK31 + R_YAW);

	// check if the innovation variance calculation is badly conditioned
	_heading_innov_var = (-HK16*HK27*HK29 - HK24*HK28*HK29 - HK25*HK29*HK30 + HK26*HK29*HK31 + R_YAW);

	if (_heading_innov_var < R_YAW) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		ECL_ERR("GPS yaw numerical error - covariance reset");
		return;
	}

	_fault_status.flags.bad_hdg = false;
	const float HK32 = HK18 / _heading_innov_var;

	// calculate the innovation and define the innovation gate
	const float innov_gate = math::max(_params.heading_innov_gate, 1.0f);
	_heading_innov = predicted_hdg - measured_hdg;

	// wrap the innovation to the interval between +-pi
	_heading_innov = wrap_pi(_heading_innov);

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

	// calculate observation jacobian
	// Observation jacobian and Kalman gain vectors
	SparseVector24f<0,1,2,3> Hfusion;
	Hfusion.at<0>() = -HK16*HK18;
	Hfusion.at<1>() = -HK18*HK24;
	Hfusion.at<2>() = -HK18*HK25;
	Hfusion.at<3>() = HK18*HK26;

	// calculate the Kalman gains
	// only calculate gains for states we are using
	Vector24f Kfusion;
	Kfusion(0) = HK27*HK32;
	Kfusion(1) = HK28*HK32;
	Kfusion(2) = HK30*HK32;
	Kfusion(3) = HK31*HK32;
	for (unsigned row = 4; row <= 23; row++) {
		Kfusion(row) = HK32*(-HK16*P(0,row) - HK24*P(1,row) - HK25*P(2,row) + HK26*P(3,row));
	}

	const bool is_fused = measurementUpdate(Kfusion, Hfusion, _heading_innov);
	_fault_status.flags.bad_hdg = !is_fused;

	if (is_fused) {
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

	const float yaw_variance = sq(fmaxf(_params.gps_heading_noise, 1.0e-2f));
	resetQuatStateYaw(measured_yaw, yaw_variance, true);

	_time_last_gps_yaw_fuse = _time_last_imu;

	return true;
}
