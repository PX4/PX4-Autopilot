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

void Ekf::fuseGpsAntYaw()
{
	// assign intermediate state variables
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	float R_YAW = 1.0f;
	float predicted_hdg;
	float H_YAW[4];
	float measured_hdg;

	// check if data has been set to NAN indicating no measurement
	if (ISFINITE(_gps_sample_delayed.yaw)) {
		// calculate the observed yaw angle of antenna array, converting a from body to antenna yaw measurement
		measured_hdg = _gps_sample_delayed.yaw + _gps_yaw_offset;

		// define the predicted antenna array vector and rotate into earth frame
		Vector3f ant_vec_bf = {cosf(_gps_yaw_offset), sinf(_gps_yaw_offset), 0.0f};
		Vector3f ant_vec_ef = _R_to_earth * ant_vec_bf;

		// check if antenna array vector is within 30 degrees of vertical and therefore unable to provide a reliable heading
		if (fabsf(ant_vec_ef(2)) > cosf(math::radians(30.0f)))  {
			return;
		}

		// calculate predicted antenna yaw angle
		predicted_hdg =  atan2f(ant_vec_ef(1),ant_vec_ef(0));

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

		H_YAW[0] = (t35/(t11-t2*(t4-q1*q2*2.0f))-t16*t18*t32)/(t18*t21+1.0f);
		H_YAW[1] = -t30*(t26*(t27-q2*t3*2.0f)+t16*t22*t25);
		H_YAW[2] = t30*(t25*t26-t16*t22*(t27-q2*t3*2.0f));
		H_YAW[3] = t30*(t26*t32+t16*t22*t35);

		// using magnetic heading tuning parameter
		R_YAW = sq(fmaxf(_params.mag_heading_noise, 1.0e-2f));

	} else {
		// there is nothing to fuse
		return;
	}

	// wrap the heading to the interval between +-pi
	measured_hdg = wrap_pi(measured_hdg);

	// calculate the innovation and define the innovation gate
	float innov_gate = math::max(_params.heading_innov_gate, 1.0f);
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

	float heading_innov_var_inv;

	// check if the innovation variance calculation is badly conditioned
	if (_heading_innov_var >= R_YAW) {
		// the innovation variance contribution from the state covariances is not negative, no fault
		_fault_status.flags.bad_hdg = false;
		heading_innov_var_inv = 1.0f / _heading_innov_var;

	} else {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		ECL_ERR_TIMESTAMPED("GPS yaw fusion numerical error - covariance reset");
		return;
	}

	// calculate the Kalman gains
	// only calculate gains for states we are using
	float Kfusion[_k_num_states] = {};

	for (uint8_t row = 0; row <= 15; row++) {
		Kfusion[row] = 0.0f;

		for (uint8_t col = 0; col <= 3; col++) {
			Kfusion[row] += P(row,col) * H_YAW[col];
		}

		Kfusion[row] *= heading_innov_var_inv;
	}

	if (_control_status.flags.wind) {
		for (uint8_t row = 22; row <= 23; row++) {
			Kfusion[row] = 0.0f;

			for (uint8_t col = 0; col <= 3; col++) {
				Kfusion[row] += P(row,col) * H_YAW[col];
			}

			Kfusion[row] *= heading_innov_var_inv;
		}
	}

	// innovation test ratio
	_yaw_test_ratio = sq(_heading_innov) / (sq(innov_gate) * _heading_innov_var);

	// we are no longer using 3-axis fusion so set the reported test levels to zero
	memset(_mag_test_ratio, 0, sizeof(_mag_test_ratio));

	// set the magnetometer unhealthy if the test fails
	if (_yaw_test_ratio > 1.0f) {
		_innov_check_fail_status.flags.reject_yaw = true;

		// if we are in air we don't want to fuse the measurement
		// we allow to use it when on the ground because the large innovation could be caused
		// by interference or a large initial gyro bias
		if (_control_status.flags.in_air) {
			return;

		} else {
			// constrain the innovation to the maximum set by the gate
			float gate_limit = sqrtf((sq(innov_gate) * _heading_innov_var));
			_heading_innov = math::constrain(_heading_innov, -gate_limit, gate_limit);
		}

	} else {
		_innov_check_fail_status.flags.reject_yaw = false;
	}

	// apply covariance correction via P_new = (I -K*H)*P
	// first calculate expression for KHP
	// then calculate P - KHP
	matrix::SquareMatrix<float, _k_num_states> KHP;
	float KH[4];

	for (unsigned row = 0; row < _k_num_states; row++) {

		KH[0] = Kfusion[row] * H_YAW[0];
		KH[1] = Kfusion[row] * H_YAW[1];
		KH[2] = Kfusion[row] * H_YAW[2];
		KH[3] = Kfusion[row] * H_YAW[3];

		for (unsigned column = 0; column < _k_num_states; column++) {
			float tmp = KH[0] * P(0,column);
			tmp += KH[1] * P(1,column);
			tmp += KH[2] * P(2,column);
			tmp += KH[3] * P(3,column);
			KHP(row,column) = tmp;
		}
	}

	// if the covariance correction will result in a negative variance, then
	// the covariance matrix is unhealthy and must be corrected
	bool healthy = true;
	_fault_status.flags.bad_hdg = false;

	for (int i = 0; i < _k_num_states; i++) {
		if (P(i,i) < KHP(i,i)) {
			// zero rows and columns
			P.uncorrelateCovarianceSetVariance<1>(i, 0.0f);

			//flag as unhealthy
			healthy = false;

			// update individual measurement health status
			_fault_status.flags.bad_hdg = true;

		}
	}

	// only apply covariance and state corrections if healthy
	if (healthy) {
		// apply the covariance corrections
		for (unsigned row = 0; row < _k_num_states; row++) {
			for (unsigned column = 0; column < _k_num_states; column++) {
				P(row,column) = P(row,column) - KHP(row,column);
			}
		}

		// correct the covariance matrix for gross errors
		fixCovarianceErrors(true);

		// apply the state corrections
		fuse(Kfusion, _heading_innov);

	}
}

bool Ekf::resetGpsAntYaw()
{
	// check if data has been set to NAN indicating no measurement
	if (ISFINITE(_gps_sample_delayed.yaw)) {

		// define the predicted antenna array vector and rotate into earth frame
		const Vector3f ant_vec_bf = {cosf(_gps_yaw_offset), sinf(_gps_yaw_offset), 0.0f};
		const Vector3f ant_vec_ef = _R_to_earth * ant_vec_bf;

		// check if antenna array vector is within 30 degrees of vertical and therefore unable to provide a reliable heading
		if (fabsf(ant_vec_ef(2)) > cosf(math::radians(30.0f)))  {
			return false;
		}

		const float predicted_yaw =  atan2f(ant_vec_ef(1),ant_vec_ef(0));

		// get measurement and correct for antenna array yaw offset
		const float measured_yaw = _gps_sample_delayed.yaw + _gps_yaw_offset;

		// calculate the amount the yaw needs to be rotated by
		float yaw_delta = wrap_pi(measured_yaw - predicted_yaw);

		// save a copy of the quaternion state for later use in calculating the amount of reset change
		const Quatf quat_before_reset = _state.quat_nominal;
		Quatf quat_after_reset = _state.quat_nominal;

		// obtain the yaw angle using the best conditioned from either a Tait-Bryan 321 or 312 sequence
		// to avoid gimbal lock
		if (fabsf(_R_to_earth(2, 0)) < fabsf(_R_to_earth(2, 1))) {
			// get the roll, pitch, yaw estimates from the quaternion states using a 321 Tait-Bryan rotation sequence
			Eulerf euler_init(_state.quat_nominal);

			// correct the yaw angle
			euler_init(2) += yaw_delta;
			euler_init(2) = wrap_pi(euler_init(2));

			quat_after_reset = Quatf(euler_init);

		} else {
			// Calculate the 312 Tait-Bryan sequence euler angles that rotate from earth to body frame
			// PX4 math library does not support this so are using equations from
			// http://www.atacolorado.com/eulersequences.doc
			Vector3f euler312;
			euler312(0) = atan2f(-_R_to_earth(0, 1), _R_to_earth(1, 1));  // first rotation (yaw)
			euler312(1) = asinf(_R_to_earth(2, 1)); // second rotation (roll)
			euler312(2) = atan2f(-_R_to_earth(2, 0), _R_to_earth(2, 2));  // third rotation (pitch)

			// correct the yaw angle
			euler312(0) += yaw_delta;
			euler312(0) = wrap_pi(euler312(0));

			// Calculate the body to earth frame rotation matrix from the corrected euler angles
			float c2 = cosf(euler312(2));
			float s2 = sinf(euler312(2));
			float s1 = sinf(euler312(1));
			float c1 = cosf(euler312(1));
			float s0 = sinf(euler312(0));
			float c0 = cosf(euler312(0));

			Dcmf R_to_earth;
			R_to_earth(0, 0) = c0 * c2 - s0 * s1 * s2;
			R_to_earth(1, 1) = c0 * c1;
			R_to_earth(2, 2) = c2 * c1;
			R_to_earth(0, 1) = -c1 * s0;
			R_to_earth(0, 2) = s2 * c0 + c2 * s1 * s0;
			R_to_earth(1, 0) = c2 * s0 + s2 * s1 * c0;
			R_to_earth(1, 2) = s0 * s2 - s1 * c0 * c2;
			R_to_earth(2, 0) = -s2 * c1;
			R_to_earth(2, 1) = s1;

			// update the quaternions
			quat_after_reset = Quatf(R_to_earth);
		}

		// calculate the amount that the quaternion has changed by
		const Quatf q_error( (_state.quat_nominal * quat_before_reset.inversed()).normalized() );

		// convert the quaternion delta to a delta angle
		Vector3f delta_ang_error;
		float scalar;

		if (q_error(0) >= 0.0f) {
			scalar = -2.0f;

		} else {
			scalar = 2.0f;
		}

		delta_ang_error(0) = scalar * q_error(1);
		delta_ang_error(1) = scalar * q_error(2);
		delta_ang_error(2) = scalar * q_error(3);

		// update the quaternion state estimates and corresponding covariances only if the change in angle has been large or the yaw is not yet aligned
		if (delta_ang_error.norm() > math::radians(15.0f) || !_control_status.flags.yaw_align) {
			// update quaternion states
			_state.quat_nominal = quat_after_reset;
			uncorrelateQuatStates();

			// record the state change
			_state_reset_status.quat_change = q_error;

			// update transformation matrix from body to world frame using the current estimate
			_R_to_earth = Dcmf(_state.quat_nominal);

			// update the yaw angle variance using the variance of the measurement
			increaseQuatYawErrVariance(sq(fmaxf(_params.mag_heading_noise, 1.0e-2f)));

			// add the reset amount to the output observer buffered data
			for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
				_output_buffer[i].quat_nominal = _state_reset_status.quat_change * _output_buffer[i].quat_nominal;
			}

			// apply the change in attitude quaternion to our newest quaternion estimate
			// which was already taken out from the output buffer
			_output_new.quat_nominal = _state_reset_status.quat_change * _output_new.quat_nominal;

			// capture the reset event
			_state_reset_status.quat_counter++;

		}

		return true;
	}

	return false;
}
