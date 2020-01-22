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
 * Magnetometer fusion methods.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include <ecl.h>
#include <mathlib/mathlib.h>

void Ekf::fuseMag()
{
	// assign intermediate variables
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	const float magN = _state.mag_I(0);
	const float magE = _state.mag_I(1);
	const float magD = _state.mag_I(2);

	// XYZ Measurement uncertainty. Need to consider timing errors for fast rotations
	const float R_MAG = sq(fmaxf(_params.mag_noise, 0.0f));

	// intermediate variables from algebraic optimisation
	float SH_MAG[9];
	SH_MAG[0] = 2.0f*magD*q3 + 2.0f*magE*q2 + 2.0f*magN*q1;
	SH_MAG[1] = 2.0f*magD*q0 - 2.0f*magE*q1 + 2.0f*magN*q2;
	SH_MAG[2] = 2.0f*magD*q1 + 2.0f*magE*q0 - 2.0f*magN*q3;
	SH_MAG[3] = sq(q3);
	SH_MAG[4] = sq(q2);
	SH_MAG[5] = sq(q1);
	SH_MAG[6] = sq(q0);
	SH_MAG[7] = 2.0f*magN*q0;
	SH_MAG[8] = 2.0f*magE*q3;

	// rotate magnetometer earth field state into body frame
	const Dcmf R_to_body = quat_to_invrotmat(_state.quat_nominal);

	const Vector3f mag_I_rot = R_to_body * _state.mag_I;

	// compute magnetometer innovations
	_mag_innov[0] = (mag_I_rot(0) + _state.mag_B(0)) - _mag_sample_delayed.mag(0);
	_mag_innov[1] = (mag_I_rot(1) + _state.mag_B(1)) - _mag_sample_delayed.mag(1);

	// do not use the synthesized measurement for the magnetomter Z component for 3D fusion
	if (_control_status.flags.synthetic_mag_z) {
		_mag_innov[2] = 0.0f;
	} else {
		_mag_innov[2] = (mag_I_rot(2) + _state.mag_B(2)) - _mag_sample_delayed.mag(2);
	}
	// Observation jacobian and Kalman gain vectors
	float H_MAG[24];
	float Kfusion[24];

	// X axis innovation variance
	_mag_innov_var[0] = (P(19,19) + R_MAG + P(1,19)*SH_MAG[0] - P(2,19)*SH_MAG[1] + P(3,19)*SH_MAG[2] - P(16,19)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + (2.0f*q0*q3 + 2.0f*q1*q2)*(P(19,17) + P(1,17)*SH_MAG[0] - P(2,17)*SH_MAG[1] + P(3,17)*SH_MAG[2] - P(16,17)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P(17,17)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,17)*(2.0f*q0*q2 - 2.0f*q1*q3) + P(0,17)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (2.0f*q0*q2 - 2.0f*q1*q3)*(P(19,18) + P(1,18)*SH_MAG[0] - P(2,18)*SH_MAG[1] + P(3,18)*SH_MAG[2] - P(16,18)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P(17,18)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,18)*(2.0f*q0*q2 - 2.0f*q1*q3) + P(0,18)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)*(P(19,0) + P(1,0)*SH_MAG[0] - P(2,0)*SH_MAG[1] + P(3,0)*SH_MAG[2] - P(16,0)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P(17,0)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,0)*(2.0f*q0*q2 - 2.0f*q1*q3) + P(0,0)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P(17,19)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,19)*(2.0f*q0*q2 - 2.0f*q1*q3) + SH_MAG[0]*(P(19,1) + P(1,1)*SH_MAG[0] - P(2,1)*SH_MAG[1] + P(3,1)*SH_MAG[2] - P(16,1)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P(17,1)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,1)*(2.0f*q0*q2 - 2.0f*q1*q3) + P(0,1)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - SH_MAG[1]*(P(19,2) + P(1,2)*SH_MAG[0] - P(2,2)*SH_MAG[1] + P(3,2)*SH_MAG[2] - P(16,2)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P(17,2)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,2)*(2.0f*q0*q2 - 2.0f*q1*q3) + P(0,2)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[2]*(P(19,3) + P(1,3)*SH_MAG[0] - P(2,3)*SH_MAG[1] + P(3,3)*SH_MAG[2] - P(16,3)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P(17,3)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,3)*(2.0f*q0*q2 - 2.0f*q1*q3) + P(0,3)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6])*(P(19,16) + P(1,16)*SH_MAG[0] - P(2,16)*SH_MAG[1] + P(3,16)*SH_MAG[2] - P(16,16)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P(17,16)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,16)*(2.0f*q0*q2 - 2.0f*q1*q3) + P(0,16)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P(0,19)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2));
	// check for a badly conditioned covariance matrix

	if (_mag_innov_var[0] >= R_MAG) {
		// the innovation variance contribution from the state covariances is non-negative - no fault
		_fault_status.flags.bad_mag_x = false;

	} else {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_mag_x = true;

		// we need to re-initialise covariances and abort this fusion step
		resetMagRelatedCovariances();
		ECL_ERR_TIMESTAMPED("magX fusion numerical error - covariance reset");
		return;

	}

	// Y axis innovation variance
	_mag_innov_var[1] = (P(20,20) + R_MAG + P(0,20)*SH_MAG[2] + P(1,20)*SH_MAG[1] + P(2,20)*SH_MAG[0] - P(17,20)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - (2.0f*q0*q3 - 2.0f*q1*q2)*(P(20,16) + P(0,16)*SH_MAG[2] + P(1,16)*SH_MAG[1] + P(2,16)*SH_MAG[0] - P(17,16)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P(16,16)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,16)*(2.0f*q0*q1 + 2.0f*q2*q3) - P(3,16)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (2.0f*q0*q1 + 2.0f*q2*q3)*(P(20,18) + P(0,18)*SH_MAG[2] + P(1,18)*SH_MAG[1] + P(2,18)*SH_MAG[0] - P(17,18)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P(16,18)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,18)*(2.0f*q0*q1 + 2.0f*q2*q3) - P(3,18)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)*(P(20,3) + P(0,3)*SH_MAG[2] + P(1,3)*SH_MAG[1] + P(2,3)*SH_MAG[0] - P(17,3)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P(16,3)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,3)*(2.0f*q0*q1 + 2.0f*q2*q3) - P(3,3)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - P(16,20)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,20)*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_MAG[2]*(P(20,0) + P(0,0)*SH_MAG[2] + P(1,0)*SH_MAG[1] + P(2,0)*SH_MAG[0] - P(17,0)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P(16,0)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,0)*(2.0f*q0*q1 + 2.0f*q2*q3) - P(3,0)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[1]*(P(20,1) + P(0,1)*SH_MAG[2] + P(1,1)*SH_MAG[1] + P(2,1)*SH_MAG[0] - P(17,1)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P(16,1)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,1)*(2.0f*q0*q1 + 2.0f*q2*q3) - P(3,1)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[0]*(P(20,2) + P(0,2)*SH_MAG[2] + P(1,2)*SH_MAG[1] + P(2,2)*SH_MAG[0] - P(17,2)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P(16,2)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,2)*(2.0f*q0*q1 + 2.0f*q2*q3) - P(3,2)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6])*(P(20,17) + P(0,17)*SH_MAG[2] + P(1,17)*SH_MAG[1] + P(2,17)*SH_MAG[0] - P(17,17)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P(16,17)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,17)*(2.0f*q0*q1 + 2.0f*q2*q3) - P(3,17)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - P(3,20)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2));

	// check for a badly conditioned covariance matrix
	if (_mag_innov_var[1] >= R_MAG) {
		// the innovation variance contribution from the state covariances is non-negative - no fault
		_fault_status.flags.bad_mag_y = false;

	} else {
		// the innovation variance contribution from the state covariances is negtive which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_mag_y = true;

		// we need to re-initialise covariances and abort this fusion step
		resetMagRelatedCovariances();
		ECL_ERR_TIMESTAMPED("magY fusion numerical error - covariance reset");
		return;

	}

	// Z axis innovation variance
	_mag_innov_var[2] = (P(21,21) + R_MAG + P(0,21)*SH_MAG[1] - P(1,21)*SH_MAG[2] + P(3,21)*SH_MAG[0] + P(18,21)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + (2.0f*q0*q2 + 2.0f*q1*q3)*(P(21,16) + P(0,16)*SH_MAG[1] - P(1,16)*SH_MAG[2] + P(3,16)*SH_MAG[0] + P(18,16)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P(16,16)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,16)*(2.0f*q0*q1 - 2.0f*q2*q3) + P(2,16)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (2.0f*q0*q1 - 2.0f*q2*q3)*(P(21,17) + P(0,17)*SH_MAG[1] - P(1,17)*SH_MAG[2] + P(3,17)*SH_MAG[0] + P(18,17)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P(16,17)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,17)*(2.0f*q0*q1 - 2.0f*q2*q3) + P(2,17)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)*(P(21,2) + P(0,2)*SH_MAG[1] - P(1,2)*SH_MAG[2] + P(3,2)*SH_MAG[0] + P(18,2)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P(16,2)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,2)*(2.0f*q0*q1 - 2.0f*q2*q3) + P(2,2)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P(16,21)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,21)*(2.0f*q0*q1 - 2.0f*q2*q3) + SH_MAG[1]*(P(21,0) + P(0,0)*SH_MAG[1] - P(1,0)*SH_MAG[2] + P(3,0)*SH_MAG[0] + P(18,0)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P(16,0)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,0)*(2.0f*q0*q1 - 2.0f*q2*q3) + P(2,0)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - SH_MAG[2]*(P(21,1) + P(0,1)*SH_MAG[1] - P(1,1)*SH_MAG[2] + P(3,1)*SH_MAG[0] + P(18,1)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P(16,1)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,1)*(2.0f*q0*q1 - 2.0f*q2*q3) + P(2,1)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[0]*(P(21,3) + P(0,3)*SH_MAG[1] - P(1,3)*SH_MAG[2] + P(3,3)*SH_MAG[0] + P(18,3)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P(16,3)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,3)*(2.0f*q0*q1 - 2.0f*q2*q3) + P(2,3)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6])*(P(21,18) + P(0,18)*SH_MAG[1] - P(1,18)*SH_MAG[2] + P(3,18)*SH_MAG[0] + P(18,18)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P(16,18)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,18)*(2.0f*q0*q1 - 2.0f*q2*q3) + P(2,18)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P(2,21)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2));

	// check for a badly conditioned covariance matrix
	if (_mag_innov_var[2] >= R_MAG) {
		// the innovation variance contribution from the state covariances is non-negative - no fault
		_fault_status.flags.bad_mag_z = false;

	} else if (_mag_innov_var[2] > 0.0f) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_mag_z = true;

		// we need to re-initialise covariances and abort this fusion step
		resetMagRelatedCovariances();
		ECL_ERR_TIMESTAMPED("magZ fusion numerical error - covariance reset");
		return;

	}

	// Perform an innovation consistency check and report the result
	bool healthy = true;

	for (uint8_t index = 0; index <= 2; index++) {
		_mag_test_ratio[index] = sq(_mag_innov[index]) / (sq(math::max(_params.mag_innov_gate, 1.0f)) * _mag_innov_var[index]);

		if (_mag_test_ratio[index] > 1.0f) {
			healthy = false;
			_innov_check_fail_status.value |= (1 << (index + 3));

		} else {
			_innov_check_fail_status.value &= ~(1 << (index + 3));
		}
	}

	// we are no longer using heading fusion so set the reported test level to zero
	_yaw_test_ratio = 0.0f;

	// if any axis fails, abort the mag fusion
	if (!healthy) {
		return;
	}

	// For the first few seconds after in-flight alignment we allow the magnetic field state estimates to stabilise
	// before they are used to constrain heading drift
	const bool update_all_states = ((_imu_sample_delayed.time_us - _flt_mag_align_start_time) > (uint64_t)5e6);

	// update the states and covariance using sequential fusion of the magnetometer components
	for (uint8_t index = 0; index <= 2; index++) {

		// Calculate Kalman gains and observation jacobians
		if (index == 0) {
			// Calculate X axis observation jacobians
			memset(H_MAG, 0, sizeof(H_MAG));
			H_MAG[0] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
			H_MAG[1] = SH_MAG[0];
			H_MAG[2] = -SH_MAG[1];
			H_MAG[3] = SH_MAG[2];
			H_MAG[16] = SH_MAG[5] - SH_MAG[4] - SH_MAG[3] + SH_MAG[6];
			H_MAG[17] = 2.0f*q0*q3 + 2.0f*q1*q2;
			H_MAG[18] = 2.0f*q1*q3 - 2.0f*q0*q2;
			H_MAG[19] = 1.0f;

			// Calculate X axis Kalman gains
			float SK_MX[5];
			SK_MX[0] = 1.0f / _mag_innov_var[0];
			SK_MX[1] = SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6];
			SK_MX[2] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
			SK_MX[3] = 2.0f*q0*q2 - 2.0f*q1*q3;
			SK_MX[4] = 2.0f*q0*q3 + 2.0f*q1*q2;

			if (update_all_states) {
				Kfusion[0] = SK_MX[0]*(P(0,19) + P(0,1)*SH_MAG[0] - P(0,2)*SH_MAG[1] + P(0,3)*SH_MAG[2] + P(0,0)*SK_MX[2] - P(0,16)*SK_MX[1] + P(0,17)*SK_MX[4] - P(0,18)*SK_MX[3]);
				Kfusion[1] = SK_MX[0]*(P(1,19) + P(1,1)*SH_MAG[0] - P(1,2)*SH_MAG[1] + P(1,3)*SH_MAG[2] + P(1,0)*SK_MX[2] - P(1,16)*SK_MX[1] + P(1,17)*SK_MX[4] - P(1,18)*SK_MX[3]);
				Kfusion[2] = SK_MX[0]*(P(2,19) + P(2,1)*SH_MAG[0] - P(2,2)*SH_MAG[1] + P(2,3)*SH_MAG[2] + P(2,0)*SK_MX[2] - P(2,16)*SK_MX[1] + P(2,17)*SK_MX[4] - P(2,18)*SK_MX[3]);
				Kfusion[3] = SK_MX[0]*(P(3,19) + P(3,1)*SH_MAG[0] - P(3,2)*SH_MAG[1] + P(3,3)*SH_MAG[2] + P(3,0)*SK_MX[2] - P(3,16)*SK_MX[1] + P(3,17)*SK_MX[4] - P(3,18)*SK_MX[3]);
				Kfusion[4] = SK_MX[0]*(P(4,19) + P(4,1)*SH_MAG[0] - P(4,2)*SH_MAG[1] + P(4,3)*SH_MAG[2] + P(4,0)*SK_MX[2] - P(4,16)*SK_MX[1] + P(4,17)*SK_MX[4] - P(4,18)*SK_MX[3]);
				Kfusion[5] = SK_MX[0]*(P(5,19) + P(5,1)*SH_MAG[0] - P(5,2)*SH_MAG[1] + P(5,3)*SH_MAG[2] + P(5,0)*SK_MX[2] - P(5,16)*SK_MX[1] + P(5,17)*SK_MX[4] - P(5,18)*SK_MX[3]);
				Kfusion[6] = SK_MX[0]*(P(6,19) + P(6,1)*SH_MAG[0] - P(6,2)*SH_MAG[1] + P(6,3)*SH_MAG[2] + P(6,0)*SK_MX[2] - P(6,16)*SK_MX[1] + P(6,17)*SK_MX[4] - P(6,18)*SK_MX[3]);
				Kfusion[7] = SK_MX[0]*(P(7,19) + P(7,1)*SH_MAG[0] - P(7,2)*SH_MAG[1] + P(7,3)*SH_MAG[2] + P(7,0)*SK_MX[2] - P(7,16)*SK_MX[1] + P(7,17)*SK_MX[4] - P(7,18)*SK_MX[3]);
				Kfusion[8] = SK_MX[0]*(P(8,19) + P(8,1)*SH_MAG[0] - P(8,2)*SH_MAG[1] + P(8,3)*SH_MAG[2] + P(8,0)*SK_MX[2] - P(8,16)*SK_MX[1] + P(8,17)*SK_MX[4] - P(8,18)*SK_MX[3]);
				Kfusion[9] = SK_MX[0]*(P(9,19) + P(9,1)*SH_MAG[0] - P(9,2)*SH_MAG[1] + P(9,3)*SH_MAG[2] + P(9,0)*SK_MX[2] - P(9,16)*SK_MX[1] + P(9,17)*SK_MX[4] - P(9,18)*SK_MX[3]);
				Kfusion[10] = SK_MX[0]*(P(10,19) + P(10,1)*SH_MAG[0] - P(10,2)*SH_MAG[1] + P(10,3)*SH_MAG[2] + P(10,0)*SK_MX[2] - P(10,16)*SK_MX[1] + P(10,17)*SK_MX[4] - P(10,18)*SK_MX[3]);
				Kfusion[11] = SK_MX[0]*(P(11,19) + P(11,1)*SH_MAG[0] - P(11,2)*SH_MAG[1] + P(11,3)*SH_MAG[2] + P(11,0)*SK_MX[2] - P(11,16)*SK_MX[1] + P(11,17)*SK_MX[4] - P(11,18)*SK_MX[3]);
				Kfusion[12] = SK_MX[0]*(P(12,19) + P(12,1)*SH_MAG[0] - P(12,2)*SH_MAG[1] + P(12,3)*SH_MAG[2] + P(12,0)*SK_MX[2] - P(12,16)*SK_MX[1] + P(12,17)*SK_MX[4] - P(12,18)*SK_MX[3]);
				Kfusion[13] = SK_MX[0]*(P(13,19) + P(13,1)*SH_MAG[0] - P(13,2)*SH_MAG[1] + P(13,3)*SH_MAG[2] + P(13,0)*SK_MX[2] - P(13,16)*SK_MX[1] + P(13,17)*SK_MX[4] - P(13,18)*SK_MX[3]);
				Kfusion[14] = SK_MX[0]*(P(14,19) + P(14,1)*SH_MAG[0] - P(14,2)*SH_MAG[1] + P(14,3)*SH_MAG[2] + P(14,0)*SK_MX[2] - P(14,16)*SK_MX[1] + P(14,17)*SK_MX[4] - P(14,18)*SK_MX[3]);
				Kfusion[15] = SK_MX[0]*(P(15,19) + P(15,1)*SH_MAG[0] - P(15,2)*SH_MAG[1] + P(15,3)*SH_MAG[2] + P(15,0)*SK_MX[2] - P(15,16)*SK_MX[1] + P(15,17)*SK_MX[4] - P(15,18)*SK_MX[3]);
				Kfusion[22] = SK_MX[0]*(P(22,19) + P(22,1)*SH_MAG[0] - P(22,2)*SH_MAG[1] + P(22,3)*SH_MAG[2] + P(22,0)*SK_MX[2] - P(22,16)*SK_MX[1] + P(22,17)*SK_MX[4] - P(22,18)*SK_MX[3]);
				Kfusion[23] = SK_MX[0]*(P(23,19) + P(23,1)*SH_MAG[0] - P(23,2)*SH_MAG[1] + P(23,3)*SH_MAG[2] + P(23,0)*SK_MX[2] - P(23,16)*SK_MX[1] + P(23,17)*SK_MX[4] - P(23,18)*SK_MX[3]);
			} else {
				for (uint8_t i = 0; i < 16; i++) {
					Kfusion[i] = 0.0f;
				}

				Kfusion[22] = 0.0f;
				Kfusion[23] = 0.0f;
			}

			Kfusion[16] = SK_MX[0]*(P(16,19) + P(16,1)*SH_MAG[0] - P(16,2)*SH_MAG[1] + P(16,3)*SH_MAG[2] + P(16,0)*SK_MX[2] - P(16,16)*SK_MX[1] + P(16,17)*SK_MX[4] - P(16,18)*SK_MX[3]);
			Kfusion[17] = SK_MX[0]*(P(17,19) + P(17,1)*SH_MAG[0] - P(17,2)*SH_MAG[1] + P(17,3)*SH_MAG[2] + P(17,0)*SK_MX[2] - P(17,16)*SK_MX[1] + P(17,17)*SK_MX[4] - P(17,18)*SK_MX[3]);
			Kfusion[18] = SK_MX[0]*(P(18,19) + P(18,1)*SH_MAG[0] - P(18,2)*SH_MAG[1] + P(18,3)*SH_MAG[2] + P(18,0)*SK_MX[2] - P(18,16)*SK_MX[1] + P(18,17)*SK_MX[4] - P(18,18)*SK_MX[3]);
			Kfusion[19] = SK_MX[0]*(P(19,19) + P(19,1)*SH_MAG[0] - P(19,2)*SH_MAG[1] + P(19,3)*SH_MAG[2] + P(19,0)*SK_MX[2] - P(19,16)*SK_MX[1] + P(19,17)*SK_MX[4] - P(19,18)*SK_MX[3]);
			Kfusion[20] = SK_MX[0]*(P(20,19) + P(20,1)*SH_MAG[0] - P(20,2)*SH_MAG[1] + P(20,3)*SH_MAG[2] + P(20,0)*SK_MX[2] - P(20,16)*SK_MX[1] + P(20,17)*SK_MX[4] - P(20,18)*SK_MX[3]);
			Kfusion[21] = SK_MX[0]*(P(21,19) + P(21,1)*SH_MAG[0] - P(21,2)*SH_MAG[1] + P(21,3)*SH_MAG[2] + P(21,0)*SK_MX[2] - P(21,16)*SK_MX[1] + P(21,17)*SK_MX[4] - P(21,18)*SK_MX[3]);

		} else if (index == 1) {
			// Calculate Y axis observation jacobians
			memset(H_MAG, 0, sizeof(H_MAG));
			H_MAG[0] = SH_MAG[2];
			H_MAG[1] = SH_MAG[1];
			H_MAG[2] = SH_MAG[0];
			H_MAG[3] = 2.0f*magD*q2 - SH_MAG[8] - SH_MAG[7];
			H_MAG[16] = 2.0f*q1*q2 - 2.0f*q0*q3;
			H_MAG[17] = SH_MAG[4] - SH_MAG[3] - SH_MAG[5] + SH_MAG[6];
			H_MAG[18] = 2.0f*q0*q1 + 2.0f*q2*q3;
			H_MAG[20] = 1.0f;

			// Calculate Y axis Kalman gains
			float SK_MY[5];
			SK_MY[0] = 1.0f / _mag_innov_var[1];
			SK_MY[1] = SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6];
			SK_MY[2] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
			SK_MY[3] = 2.0f*q0*q3 - 2.0f*q1*q2;
			SK_MY[4] = 2.0f*q0*q1 + 2.0f*q2*q3;

			if (update_all_states) {
				Kfusion[0] = SK_MY[0]*(P(0,20) + P(0,0)*SH_MAG[2] + P(0,1)*SH_MAG[1] + P(0,2)*SH_MAG[0] - P(0,3)*SK_MY[2] - P(0,17)*SK_MY[1] - P(0,16)*SK_MY[3] + P(0,18)*SK_MY[4]);
				Kfusion[1] = SK_MY[0]*(P(1,20) + P(1,0)*SH_MAG[2] + P(1,1)*SH_MAG[1] + P(1,2)*SH_MAG[0] - P(1,3)*SK_MY[2] - P(1,17)*SK_MY[1] - P(1,16)*SK_MY[3] + P(1,18)*SK_MY[4]);
				Kfusion[2] = SK_MY[0]*(P(2,20) + P(2,0)*SH_MAG[2] + P(2,1)*SH_MAG[1] + P(2,2)*SH_MAG[0] - P(2,3)*SK_MY[2] - P(2,17)*SK_MY[1] - P(2,16)*SK_MY[3] + P(2,18)*SK_MY[4]);
				Kfusion[3] = SK_MY[0]*(P(3,20) + P(3,0)*SH_MAG[2] + P(3,1)*SH_MAG[1] + P(3,2)*SH_MAG[0] - P(3,3)*SK_MY[2] - P(3,17)*SK_MY[1] - P(3,16)*SK_MY[3] + P(3,18)*SK_MY[4]);
				Kfusion[4] = SK_MY[0]*(P(4,20) + P(4,0)*SH_MAG[2] + P(4,1)*SH_MAG[1] + P(4,2)*SH_MAG[0] - P(4,3)*SK_MY[2] - P(4,17)*SK_MY[1] - P(4,16)*SK_MY[3] + P(4,18)*SK_MY[4]);
				Kfusion[5] = SK_MY[0]*(P(5,20) + P(5,0)*SH_MAG[2] + P(5,1)*SH_MAG[1] + P(5,2)*SH_MAG[0] - P(5,3)*SK_MY[2] - P(5,17)*SK_MY[1] - P(5,16)*SK_MY[3] + P(5,18)*SK_MY[4]);
				Kfusion[6] = SK_MY[0]*(P(6,20) + P(6,0)*SH_MAG[2] + P(6,1)*SH_MAG[1] + P(6,2)*SH_MAG[0] - P(6,3)*SK_MY[2] - P(6,17)*SK_MY[1] - P(6,16)*SK_MY[3] + P(6,18)*SK_MY[4]);
				Kfusion[7] = SK_MY[0]*(P(7,20) + P(7,0)*SH_MAG[2] + P(7,1)*SH_MAG[1] + P(7,2)*SH_MAG[0] - P(7,3)*SK_MY[2] - P(7,17)*SK_MY[1] - P(7,16)*SK_MY[3] + P(7,18)*SK_MY[4]);
				Kfusion[8] = SK_MY[0]*(P(8,20) + P(8,0)*SH_MAG[2] + P(8,1)*SH_MAG[1] + P(8,2)*SH_MAG[0] - P(8,3)*SK_MY[2] - P(8,17)*SK_MY[1] - P(8,16)*SK_MY[3] + P(8,18)*SK_MY[4]);
				Kfusion[9] = SK_MY[0]*(P(9,20) + P(9,0)*SH_MAG[2] + P(9,1)*SH_MAG[1] + P(9,2)*SH_MAG[0] - P(9,3)*SK_MY[2] - P(9,17)*SK_MY[1] - P(9,16)*SK_MY[3] + P(9,18)*SK_MY[4]);
				Kfusion[10] = SK_MY[0]*(P(10,20) + P(10,0)*SH_MAG[2] + P(10,1)*SH_MAG[1] + P(10,2)*SH_MAG[0] - P(10,3)*SK_MY[2] - P(10,17)*SK_MY[1] - P(10,16)*SK_MY[3] + P(10,18)*SK_MY[4]);
				Kfusion[11] = SK_MY[0]*(P(11,20) + P(11,0)*SH_MAG[2] + P(11,1)*SH_MAG[1] + P(11,2)*SH_MAG[0] - P(11,3)*SK_MY[2] - P(11,17)*SK_MY[1] - P(11,16)*SK_MY[3] + P(11,18)*SK_MY[4]);
				Kfusion[12] = SK_MY[0]*(P(12,20) + P(12,0)*SH_MAG[2] + P(12,1)*SH_MAG[1] + P(12,2)*SH_MAG[0] - P(12,3)*SK_MY[2] - P(12,17)*SK_MY[1] - P(12,16)*SK_MY[3] + P(12,18)*SK_MY[4]);
				Kfusion[13] = SK_MY[0]*(P(13,20) + P(13,0)*SH_MAG[2] + P(13,1)*SH_MAG[1] + P(13,2)*SH_MAG[0] - P(13,3)*SK_MY[2] - P(13,17)*SK_MY[1] - P(13,16)*SK_MY[3] + P(13,18)*SK_MY[4]);
				Kfusion[14] = SK_MY[0]*(P(14,20) + P(14,0)*SH_MAG[2] + P(14,1)*SH_MAG[1] + P(14,2)*SH_MAG[0] - P(14,3)*SK_MY[2] - P(14,17)*SK_MY[1] - P(14,16)*SK_MY[3] + P(14,18)*SK_MY[4]);
				Kfusion[15] = SK_MY[0]*(P(15,20) + P(15,0)*SH_MAG[2] + P(15,1)*SH_MAG[1] + P(15,2)*SH_MAG[0] - P(15,3)*SK_MY[2] - P(15,17)*SK_MY[1] - P(15,16)*SK_MY[3] + P(15,18)*SK_MY[4]);
				Kfusion[22] = SK_MY[0]*(P(22,20) + P(22,0)*SH_MAG[2] + P(22,1)*SH_MAG[1] + P(22,2)*SH_MAG[0] - P(22,3)*SK_MY[2] - P(22,17)*SK_MY[1] - P(22,16)*SK_MY[3] + P(22,18)*SK_MY[4]);
				Kfusion[23] = SK_MY[0]*(P(23,20) + P(23,0)*SH_MAG[2] + P(23,1)*SH_MAG[1] + P(23,2)*SH_MAG[0] - P(23,3)*SK_MY[2] - P(23,17)*SK_MY[1] - P(23,16)*SK_MY[3] + P(23,18)*SK_MY[4]);
			} else {
				for (uint8_t i = 0; i < 16; i++) {
					Kfusion[i] = 0.0f;
				}

				Kfusion[22] = 0.0f;
				Kfusion[23] = 0.0f;
			}

			Kfusion[16] = SK_MY[0]*(P(16,20) + P(16,0)*SH_MAG[2] + P(16,1)*SH_MAG[1] + P(16,2)*SH_MAG[0] - P(16,3)*SK_MY[2] - P(16,17)*SK_MY[1] - P(16,16)*SK_MY[3] + P(16,18)*SK_MY[4]);
			Kfusion[17] = SK_MY[0]*(P(17,20) + P(17,0)*SH_MAG[2] + P(17,1)*SH_MAG[1] + P(17,2)*SH_MAG[0] - P(17,3)*SK_MY[2] - P(17,17)*SK_MY[1] - P(17,16)*SK_MY[3] + P(17,18)*SK_MY[4]);
			Kfusion[18] = SK_MY[0]*(P(18,20) + P(18,0)*SH_MAG[2] + P(18,1)*SH_MAG[1] + P(18,2)*SH_MAG[0] - P(18,3)*SK_MY[2] - P(18,17)*SK_MY[1] - P(18,16)*SK_MY[3] + P(18,18)*SK_MY[4]);
			Kfusion[19] = SK_MY[0]*(P(19,20) + P(19,0)*SH_MAG[2] + P(19,1)*SH_MAG[1] + P(19,2)*SH_MAG[0] - P(19,3)*SK_MY[2] - P(19,17)*SK_MY[1] - P(19,16)*SK_MY[3] + P(19,18)*SK_MY[4]);
			Kfusion[20] = SK_MY[0]*(P(20,20) + P(20,0)*SH_MAG[2] + P(20,1)*SH_MAG[1] + P(20,2)*SH_MAG[0] - P(20,3)*SK_MY[2] - P(20,17)*SK_MY[1] - P(20,16)*SK_MY[3] + P(20,18)*SK_MY[4]);
			Kfusion[21] = SK_MY[0]*(P(21,20) + P(21,0)*SH_MAG[2] + P(21,1)*SH_MAG[1] + P(21,2)*SH_MAG[0] - P(21,3)*SK_MY[2] - P(21,17)*SK_MY[1] - P(21,16)*SK_MY[3] + P(21,18)*SK_MY[4]);

		} else if (index == 2) {

			// we do not fuse synthesized magnetomter measurements when doing 3D fusion
			if (_control_status.flags.synthetic_mag_z) {
				continue;
			}

			// calculate Z axis observation jacobians
			memset(H_MAG, 0, sizeof(H_MAG));
			H_MAG[0] = SH_MAG[1];
			H_MAG[1] = -SH_MAG[2];
			H_MAG[2] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
			H_MAG[3] = SH_MAG[0];
			H_MAG[16] = 2.0f*q0*q2 + 2.0f*q1*q3;
			H_MAG[17] = 2.0f*q2*q3 - 2.0f*q0*q1;
			H_MAG[18] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
			H_MAG[21] = 1.0f;

			// Calculate Z axis Kalman gains
			float SK_MZ[5];
			SK_MZ[0] = 1.0f / _mag_innov_var[2];
			SK_MZ[1] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
			SK_MZ[2] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
			SK_MZ[3] = 2.0f*q0*q1 - 2.0f*q2*q3;
			SK_MZ[4] = 2.0f*q0*q2 + 2.0f*q1*q3;

			if (update_all_states) {
				Kfusion[0] = SK_MZ[0]*(P(0,21) + P(0,0)*SH_MAG[1] - P(0,1)*SH_MAG[2] + P(0,3)*SH_MAG[0] + P(0,2)*SK_MZ[2] + P(0,18)*SK_MZ[1] + P(0,16)*SK_MZ[4] - P(0,17)*SK_MZ[3]);
				Kfusion[1] = SK_MZ[0]*(P(1,21) + P(1,0)*SH_MAG[1] - P(1,1)*SH_MAG[2] + P(1,3)*SH_MAG[0] + P(1,2)*SK_MZ[2] + P(1,18)*SK_MZ[1] + P(1,16)*SK_MZ[4] - P(1,17)*SK_MZ[3]);
				Kfusion[2] = SK_MZ[0]*(P(2,21) + P(2,0)*SH_MAG[1] - P(2,1)*SH_MAG[2] + P(2,3)*SH_MAG[0] + P(2,2)*SK_MZ[2] + P(2,18)*SK_MZ[1] + P(2,16)*SK_MZ[4] - P(2,17)*SK_MZ[3]);
				Kfusion[3] = SK_MZ[0]*(P(3,21) + P(3,0)*SH_MAG[1] - P(3,1)*SH_MAG[2] + P(3,3)*SH_MAG[0] + P(3,2)*SK_MZ[2] + P(3,18)*SK_MZ[1] + P(3,16)*SK_MZ[4] - P(3,17)*SK_MZ[3]);
				Kfusion[4] = SK_MZ[0]*(P(4,21) + P(4,0)*SH_MAG[1] - P(4,1)*SH_MAG[2] + P(4,3)*SH_MAG[0] + P(4,2)*SK_MZ[2] + P(4,18)*SK_MZ[1] + P(4,16)*SK_MZ[4] - P(4,17)*SK_MZ[3]);
				Kfusion[5] = SK_MZ[0]*(P(5,21) + P(5,0)*SH_MAG[1] - P(5,1)*SH_MAG[2] + P(5,3)*SH_MAG[0] + P(5,2)*SK_MZ[2] + P(5,18)*SK_MZ[1] + P(5,16)*SK_MZ[4] - P(5,17)*SK_MZ[3]);
				Kfusion[6] = SK_MZ[0]*(P(6,21) + P(6,0)*SH_MAG[1] - P(6,1)*SH_MAG[2] + P(6,3)*SH_MAG[0] + P(6,2)*SK_MZ[2] + P(6,18)*SK_MZ[1] + P(6,16)*SK_MZ[4] - P(6,17)*SK_MZ[3]);
				Kfusion[7] = SK_MZ[0]*(P(7,21) + P(7,0)*SH_MAG[1] - P(7,1)*SH_MAG[2] + P(7,3)*SH_MAG[0] + P(7,2)*SK_MZ[2] + P(7,18)*SK_MZ[1] + P(7,16)*SK_MZ[4] - P(7,17)*SK_MZ[3]);
				Kfusion[8] = SK_MZ[0]*(P(8,21) + P(8,0)*SH_MAG[1] - P(8,1)*SH_MAG[2] + P(8,3)*SH_MAG[0] + P(8,2)*SK_MZ[2] + P(8,18)*SK_MZ[1] + P(8,16)*SK_MZ[4] - P(8,17)*SK_MZ[3]);
				Kfusion[9] = SK_MZ[0]*(P(9,21) + P(9,0)*SH_MAG[1] - P(9,1)*SH_MAG[2] + P(9,3)*SH_MAG[0] + P(9,2)*SK_MZ[2] + P(9,18)*SK_MZ[1] + P(9,16)*SK_MZ[4] - P(9,17)*SK_MZ[3]);
				Kfusion[10] = SK_MZ[0]*(P(10,21) + P(10,0)*SH_MAG[1] - P(10,1)*SH_MAG[2] + P(10,3)*SH_MAG[0] + P(10,2)*SK_MZ[2] + P(10,18)*SK_MZ[1] + P(10,16)*SK_MZ[4] - P(10,17)*SK_MZ[3]);
				Kfusion[11] = SK_MZ[0]*(P(11,21) + P(11,0)*SH_MAG[1] - P(11,1)*SH_MAG[2] + P(11,3)*SH_MAG[0] + P(11,2)*SK_MZ[2] + P(11,18)*SK_MZ[1] + P(11,16)*SK_MZ[4] - P(11,17)*SK_MZ[3]);
				Kfusion[12] = SK_MZ[0]*(P(12,21) + P(12,0)*SH_MAG[1] - P(12,1)*SH_MAG[2] + P(12,3)*SH_MAG[0] + P(12,2)*SK_MZ[2] + P(12,18)*SK_MZ[1] + P(12,16)*SK_MZ[4] - P(12,17)*SK_MZ[3]);
				Kfusion[13] = SK_MZ[0]*(P(13,21) + P(13,0)*SH_MAG[1] - P(13,1)*SH_MAG[2] + P(13,3)*SH_MAG[0] + P(13,2)*SK_MZ[2] + P(13,18)*SK_MZ[1] + P(13,16)*SK_MZ[4] - P(13,17)*SK_MZ[3]);
				Kfusion[14] = SK_MZ[0]*(P(14,21) + P(14,0)*SH_MAG[1] - P(14,1)*SH_MAG[2] + P(14,3)*SH_MAG[0] + P(14,2)*SK_MZ[2] + P(14,18)*SK_MZ[1] + P(14,16)*SK_MZ[4] - P(14,17)*SK_MZ[3]);
				Kfusion[15] = SK_MZ[0]*(P(15,21) + P(15,0)*SH_MAG[1] - P(15,1)*SH_MAG[2] + P(15,3)*SH_MAG[0] + P(15,2)*SK_MZ[2] + P(15,18)*SK_MZ[1] + P(15,16)*SK_MZ[4] - P(15,17)*SK_MZ[3]);
				Kfusion[22] = SK_MZ[0]*(P(22,21) + P(22,0)*SH_MAG[1] - P(22,1)*SH_MAG[2] + P(22,3)*SH_MAG[0] + P(22,2)*SK_MZ[2] + P(22,18)*SK_MZ[1] + P(22,16)*SK_MZ[4] - P(22,17)*SK_MZ[3]);
				Kfusion[23] = SK_MZ[0]*(P(23,21) + P(23,0)*SH_MAG[1] - P(23,1)*SH_MAG[2] + P(23,3)*SH_MAG[0] + P(23,2)*SK_MZ[2] + P(23,18)*SK_MZ[1] + P(23,16)*SK_MZ[4] - P(23,17)*SK_MZ[3]);
			} else {
				for (uint8_t i = 0; i < 16; i++) {
					Kfusion[i] = 0.0f;
				}

				Kfusion[22] = 0.0f;
				Kfusion[23] = 0.0f;
			}

			Kfusion[16] = SK_MZ[0]*(P(16,21) + P(16,0)*SH_MAG[1] - P(16,1)*SH_MAG[2] + P(16,3)*SH_MAG[0] + P(16,2)*SK_MZ[2] + P(16,18)*SK_MZ[1] + P(16,16)*SK_MZ[4] - P(16,17)*SK_MZ[3]);
			Kfusion[17] = SK_MZ[0]*(P(17,21) + P(17,0)*SH_MAG[1] - P(17,1)*SH_MAG[2] + P(17,3)*SH_MAG[0] + P(17,2)*SK_MZ[2] + P(17,18)*SK_MZ[1] + P(17,16)*SK_MZ[4] - P(17,17)*SK_MZ[3]);
			Kfusion[18] = SK_MZ[0]*(P(18,21) + P(18,0)*SH_MAG[1] - P(18,1)*SH_MAG[2] + P(18,3)*SH_MAG[0] + P(18,2)*SK_MZ[2] + P(18,18)*SK_MZ[1] + P(18,16)*SK_MZ[4] - P(18,17)*SK_MZ[3]);
			Kfusion[19] = SK_MZ[0]*(P(19,21) + P(19,0)*SH_MAG[1] - P(19,1)*SH_MAG[2] + P(19,3)*SH_MAG[0] + P(19,2)*SK_MZ[2] + P(19,18)*SK_MZ[1] + P(19,16)*SK_MZ[4] - P(19,17)*SK_MZ[3]);
			Kfusion[20] = SK_MZ[0]*(P(20,21) + P(20,0)*SH_MAG[1] - P(20,1)*SH_MAG[2] + P(20,3)*SH_MAG[0] + P(20,2)*SK_MZ[2] + P(20,18)*SK_MZ[1] + P(20,16)*SK_MZ[4] - P(20,17)*SK_MZ[3]);
			Kfusion[21] = SK_MZ[0]*(P(21,21) + P(21,0)*SH_MAG[1] - P(21,1)*SH_MAG[2] + P(21,3)*SH_MAG[0] + P(21,2)*SK_MZ[2] + P(21,18)*SK_MZ[1] + P(21,16)*SK_MZ[4] - P(21,17)*SK_MZ[3]);

		} else {
			return;
		}

		// apply covariance correction via P_new = (I -K*H)*P
		// first calculate expression for KHP
		// then calculate P - KHP
		matrix::SquareMatrix<float, _k_num_states> KHP {};
		float KH[10];

		for (unsigned row = 0; row < _k_num_states; row++) {

			KH[0] = Kfusion[row] * H_MAG[0];
			KH[1] = Kfusion[row] * H_MAG[1];
			KH[2] = Kfusion[row] * H_MAG[2];
			KH[3] = Kfusion[row] * H_MAG[3];
			KH[4] = Kfusion[row] * H_MAG[16];
			KH[5] = Kfusion[row] * H_MAG[17];
			KH[6] = Kfusion[row] * H_MAG[18];
			KH[7] = Kfusion[row] * H_MAG[19];
			KH[8] = Kfusion[row] * H_MAG[20];
			KH[9] = Kfusion[row] * H_MAG[21];

			for (unsigned column = 0; column < _k_num_states; column++) {
				float tmp = KH[0] * P(0,column);
				tmp += KH[1] * P(1,column);
				tmp += KH[2] * P(2,column);
				tmp += KH[3] * P(3,column);
				tmp += KH[4] * P(16,column);
				tmp += KH[5] * P(17,column);
				tmp += KH[6] * P(18,column);
				tmp += KH[7] * P(19,column);
				tmp += KH[8] * P(20,column);
				tmp += KH[9] * P(21,column);
				KHP(row,column) = tmp;
			}
		}

		// if the covariance correction will result in a negative variance, then
		// the covariance matrix is unhealthy and must be corrected
		_fault_status.flags.bad_mag_x = false;
		_fault_status.flags.bad_mag_y = false;
		_fault_status.flags.bad_mag_z = false;

		for (int i = 0; i < _k_num_states; i++) {
			if (P(i,i) < KHP(i,i)) {
				// zero rows and columns
				P.uncorrelateCovarianceSetVariance<1>(i, 0.0f);

				//flag as unhealthy
				healthy = false;

				// update individual measurement health status
				if (index == 0) {
					_fault_status.flags.bad_mag_x = true;

				} else if (index == 1) {
					_fault_status.flags.bad_mag_y = true;

				} else if (index == 2) {
					_fault_status.flags.bad_mag_z = true;
				}
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
			fuse(Kfusion, _mag_innov[index]);

			// constrain the declination of the earth field states
			limitDeclination();
		}
	}
}

void Ekf::fuseHeading()
{
	// assign intermediate state variables
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	float R_YAW = 1.0f;
	float predicted_hdg;
	float H_YAW[4];
	Vector3f mag_earth_pred;
	float measured_hdg;

	// determine if a 321 or 312 Euler sequence is best
	if (fabsf(_R_to_earth(2, 0)) < fabsf(_R_to_earth(2, 1))) {
		// calculate observation jacobian when we are observing the first rotation in a 321 sequence
		float t9 = q0*q3;
		float t10 = q1*q2;
		float t2 = t9+t10;
		float t3 = q0*q0;
		float t4 = q1*q1;
		float t5 = q2*q2;
		float t6 = q3*q3;
		float t7 = t3+t4-t5-t6;

		float t16 = q3*t3;
		float t17 = q3*t5;
		float t18 = q0*q1*q2*2.0f;
		float t19 = t16+t17+t18-q3*t4+q3*t6;

		float t24 = q2*t4;
		float t25 = q2*t6;
		float t26 = q0*q1*q3*2.0f;
		float t27 = t24+t25+t26-q2*t3+q2*t5;
		float t28 = q1*t3;
		float t29 = q1*t5;
		float t30 = q0*q2*q3*2.0f;
		float t31 = t28+t29+t30+q1*t4-q1*t6;
		float t32 = q0*t4;
		float t33 = q0*t6;
		float t34 = q1*q2*q3*2.0f;
		float t35 = t32+t33+t34+q0*t3-q0*t5;

		// two computational paths are provided to work around singularities in calculation of the Jacobians
		float t8 = t7*t7;
		float t15 = t2*t2;
		if (t8 > t15 && t8 > 1E-6f) {
			// this path has a singularities at yaw = +-90 degrees
			t8 = 1.0f/t8;
			float t11 = t2*t2;
			float t12 = t8*t11*4.0f;
			float t13 = t12+1.0f;
			float t14 = 1.0f/t13;

			H_YAW[0] = t8*t14*t19*(-2.0f);
			H_YAW[1] = t8*t14*t27*(-2.0f);
			H_YAW[2] = t8*t14*t31*2.0f;
			H_YAW[3] = t8*t14*t35*2.0f;

		} else if (t15 > 1E-6f) {
			// this path has singularities at yaw = 0 and +-180 deg
			t15 = 1.0f/t15;
			float t20 = t7*t7;
			float t21 = t15*t20*0.25f;
			float t22 = t21+1.0f;

			if (fabsf(t22) > 1E-6f) {
				float t23 = 1.0f/t22;

				H_YAW[0] = t15*t19*t23*(-0.5f);
				H_YAW[1] = t15*t23*t27*(-0.5f);
				H_YAW[2] = t15*t23*t31*0.5f;
				H_YAW[3] = t15*t23*t35*0.5f;

			} else {
				return;

			}
		} else {
			return;

		}

		// rotate the magnetometer measurement into earth frame
		Eulerf euler321(_state.quat_nominal);
		predicted_hdg = euler321(2); // we will need the predicted heading to calculate the innovation

		// calculate the observed yaw angle
		if (_control_status.flags.mag_hdg) {
			// Set the yaw angle to zero and rotate the measurements into earth frame using the zero yaw angle
			euler321(2) = 0.0f;
			const Dcmf R_to_earth(euler321);

			// rotate the magnetometer measurements into earth frame using a zero yaw angle
			if (_control_status.flags.mag_3D) {
				// don't apply bias corrections if we are learning them
				mag_earth_pred = R_to_earth * _mag_sample_delayed.mag;

			} else {
				mag_earth_pred = R_to_earth * (_mag_sample_delayed.mag - _state.mag_B);
			}

			// the angle of the projection onto the horizontal gives the yaw angle
			measured_hdg = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();

		} else if (_control_status.flags.ev_yaw) {
			// calculate the yaw angle for a 321 sequence
			// Expressions obtained from yaw_input_321.c produced by https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw321.m
			const float Tbn_1_0 = 2.0f*(_ev_sample_delayed.quat(0)*_ev_sample_delayed.quat(3)+_ev_sample_delayed.quat(1)*_ev_sample_delayed.quat(2));
			const float Tbn_0_0 = sq(_ev_sample_delayed.quat(0))+sq(_ev_sample_delayed.quat(1))-sq(_ev_sample_delayed.quat(2))-sq(_ev_sample_delayed.quat(3));
			measured_hdg = atan2f(Tbn_1_0,Tbn_0_0);

		} else if (_mag_use_inhibit) {
			// Special case where we either use the current or last known stationary value
			// so set to current value as a safe default
			measured_hdg = predicted_hdg;

		} else {
			// Should not be doing yaw fusion
			return;

		}

	} else {
		// calculate observation jacobian when we are observing a rotation in a 312 sequence
		float t9 = q0*q3;
		float t10 = q1*q2;
		float t2 = t9-t10;
		float t3 = q0*q0;
		float t4 = q1*q1;
		float t5 = q2*q2;
		float t6 = q3*q3;
		float t7 = t3-t4+t5-t6;

		float t16 = q3*t3;
		float t17 = q3*t4;
		float t18 = t16+t17-q3*t5+q3*t6-q0*q1*q2*2.0f;
		float t23 = q2*t3;
		float t24 = q2*t4;
		float t25 = t23+t24+q2*t5-q2*t6-q0*q1*q3*2.0f;
		float t26 = q1*t5;
		float t27 = q1*t6;
		float t28 = t26+t27-q1*t3+q1*t4-q0*q2*q3*2.0f;
		float t29 = q0*t5;
		float t30 = q0*t6;
		float t31 = t29+t30+q0*t3-q0*t4-q1*q2*q3*2.0f;

		// two computational paths are provided to work around singularities in calculation of the Jacobians
		float t8 = t7*t7;
		float t15 = t2*t2;
		if (t8 > t15 && t8 > 1E-6f) {
			// this path has a singularities at yaw = +-90 degrees
			t8 = 1.0f/t8;
			float t11 = t2*t2;
			float t12 = t8*t11*4.0f;
			float t13 = t12+1.0f;
			float t14 = 1.0f/t13;

			H_YAW[0] = t8*t14*t18*(-2.0f);
			H_YAW[1] = t8*t14*t25*(-2.0f);
			H_YAW[2] = t8*t14*t28*2.0f;
			H_YAW[3] = t8*t14*t31*2.0f;

		} else if (t15 > 1E-6f) {
			// this path has singularities at yaw = 0 and +-180 deg
			t15 = 1.0f/t15;
			float t19 = t7*t7;
			float t20 = t15*t19*0.25f;
			float t21 = t20+1.0f;

			if (fabsf(t21) > 1E-6f) {
				float t22 = 1.0f/t21;

				H_YAW[0] = t15*t18*t22*(-0.5f);
				H_YAW[1] = t15*t22*t25*(-0.5f);
				H_YAW[2] = t15*t22*t28*0.5f;
				H_YAW[3] = t15*t22*t31*0.5f;

			} else {
				return;

			}

		} else {
			return;

		}



		/* Calculate the 312 sequence euler angles that rotate from earth to body frame
		 * Derived from https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw312.m
		 * Body to nav frame transformation using a yaw-roll-pitch rotation sequence is given by:
		 *
		[ cos(pitch)*cos(yaw) - sin(pitch)*sin(roll)*sin(yaw), -cos(roll)*sin(yaw), cos(yaw)*sin(pitch) + cos(pitch)*sin(roll)*sin(yaw)]
		[ cos(pitch)*sin(yaw) + cos(yaw)*sin(pitch)*sin(roll),  cos(roll)*cos(yaw), sin(pitch)*sin(yaw) - cos(pitch)*cos(yaw)*sin(roll)]
		[                               -cos(roll)*sin(pitch),           sin(roll),                                cos(pitch)*cos(roll)]
		*/
		float yaw = atan2f(-_R_to_earth(0, 1), _R_to_earth(1, 1)); // first rotation (yaw)
		const float roll = asinf(_R_to_earth(2, 1)); // second rotation (roll)
		const float pitch = atan2f(-_R_to_earth(2, 0), _R_to_earth(2, 2)); // third rotation (pitch)

		predicted_hdg = yaw; // we will need the predicted heading to calculate the innovation

		// calculate the observed yaw angle
		if (_control_status.flags.mag_hdg) {
			// Set the first rotation (yaw) to zero and rotate the measurements into earth frame
			yaw = 0.0f;

			// Calculate the body to earth frame rotation matrix from the euler angles using a 312 rotation sequence
			// Equations from Tbn_312.c produced by https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw312.m
			Dcmf R_to_earth;
			float sy = sinf(yaw);
			float cy = cosf(yaw);
			float sp = sinf(pitch);
			float cp = cosf(pitch);
			float sr = sinf(roll);
			float cr = cosf(roll);
			R_to_earth(0,0) = cy*cp-sy*sp*sr;
			R_to_earth(0,1) = -sy*cr;
			R_to_earth(0,2) = cy*sp+sy*cp*sr;
			R_to_earth(1,0) = sy*cp+cy*sp*sr;
			R_to_earth(1,1) = cy*cr;
			R_to_earth(1,2) = sy*sp-cy*cp*sr;
			R_to_earth(2,0) = -sp*cr;
			R_to_earth(2,1) = sr;
			R_to_earth(2,2) = cp*cr;

			// rotate the magnetometer measurements into earth frame using a zero yaw angle
			if (_control_status.flags.mag_3D) {
				// don't apply bias corrections if we are learning them
				mag_earth_pred = R_to_earth * _mag_sample_delayed.mag;
			} else {
				mag_earth_pred = R_to_earth * (_mag_sample_delayed.mag - _state.mag_B);
			}

			// the angle of the projection onto the horizontal gives the yaw angle
			measured_hdg = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();

		} else if (_control_status.flags.ev_yaw) {
			// calculate the yaw angle for a 312 sequence
			// Values from yaw_input_312.c file produced by https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw312.m
			float Tbn_0_1_neg = 2.0f*(_ev_sample_delayed.quat(0)*_ev_sample_delayed.quat(3)-_ev_sample_delayed.quat(1)*_ev_sample_delayed.quat(2));
			float Tbn_1_1 = sq(_ev_sample_delayed.quat(0))-sq(_ev_sample_delayed.quat(1))+sq(_ev_sample_delayed.quat(2))-sq(_ev_sample_delayed.quat(3));
			measured_hdg = atan2f(Tbn_0_1_neg,Tbn_1_1);

		} else if (_mag_use_inhibit) {
			// Special case where we either use the current or last known stationary value
			// so set to current value as a safe default
			measured_hdg = predicted_hdg;

		} else {
			// Should not be doing yaw fusion
			return;

		}
	}

	// Calculate the observation variance
	if (_control_status.flags.mag_hdg) {
		// using magnetic heading tuning parameter
		R_YAW = sq(fmaxf(_params.mag_heading_noise, 1.0e-2f));

	} else if (_control_status.flags.ev_yaw) {
		// using error estimate from external vision data
		R_YAW = fmaxf(_ev_sample_delayed.angVar, sq(1.0e-2f));

	} else {
		// default value
		R_YAW = 0.01f;
	}

	// wrap the heading to the interval between +-pi
	measured_hdg = wrap_pi(measured_hdg);

	// calculate the innovation and define the innovation gate
	float innov_gate = math::max(_params.heading_innov_gate, 1.0f);
	if (_mag_use_inhibit) {
		// The magnetometer cannot be trusted but we need to fuse a heading to prevent a badly
		// conditioned covariance matrix developing over time.
		if (!_vehicle_at_rest) {
			// Vehicle is not at rest so fuse a zero innovation and record the
			// predicted heading to use as an observation when movement ceases.
			_heading_innov = 0.0f;
			_last_static_yaw = predicted_hdg;

		} else {
			// Vehicle is at rest so use the last moving prediction as an observation
			// to prevent the heading from drifting and to enable yaw gyro bias learning
			// before takeoff.
			_heading_innov = predicted_hdg - _last_static_yaw;
			innov_gate = 5.0f;

		}
	} else {
		_heading_innov = predicted_hdg - measured_hdg;
		_last_static_yaw = predicted_hdg;

	}
	_mag_use_inhibit_prev = _mag_use_inhibit;

	// wrap the innovation to the interval between +-pi
	_heading_innov = wrap_pi(_heading_innov);

	// Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 4 elements in H are non zero
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
		ECL_ERR_TIMESTAMPED("mag yaw fusion numerical error - covariance reset");
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

void Ekf::fuseDeclination(float decl_sigma)
{
	// assign intermediate state variables
	const float magN = _state.mag_I(0);
	const float magE = _state.mag_I(1);

	// minimum horizontal field strength before calculation becomes badly conditioned (T)
	const float h_field_min = 0.001f;

	// observation variance (rad**2)
	const float R_DECL = sq(decl_sigma);

	// Calculate intermediate variables
	float t2 = magE*magE;
	float t3 = magN*magN;
	float t4 = t2+t3;
	// if the horizontal magnetic field is too small, this calculation will be badly conditioned
	if (t4 < h_field_min*h_field_min) {
		return;
	}
	float t5 = P(16,16)*t2;
	float t6 = P(17,17)*t3;
	float t7 = t2*t2;
	float t8 = R_DECL*t7;
	float t9 = t3*t3;
	float t10 = R_DECL*t9;
	float t11 = R_DECL*t2*t3*2.0f;
	float t14 = P(16,17)*magE*magN;
	float t15 = P(17,16)*magE*magN;
	float t12 = t5+t6+t8+t10+t11-t14-t15;
	float t13;
	if (fabsf(t12) > 1e-6f) {
		t13 = 1.0f / t12;
	} else {
		return;
	}
	float t18 = magE*magE;
	float t19 = magN*magN;
	float t20 = t18+t19;
	float t21;
	if (fabsf(t20) > 1e-6f) {
		t21 = 1.0f/t20;
	} else {
		return;
	}

	// Calculate the observation Jacobian
	// Note only 2 terms are non-zero which can be used in matrix operations for calculation of Kalman gains and covariance update to significantly reduce cost
	float H_DECL[24] = {};
	H_DECL[16] = -magE*t21;
	H_DECL[17] = magN*t21;

	// Calculate the Kalman gains
	float Kfusion[_k_num_states] = {};
	Kfusion[0] = -t4*t13*(P(0,16)*magE-P(0,17)*magN);
	Kfusion[1] = -t4*t13*(P(1,16)*magE-P(1,17)*magN);
	Kfusion[2] = -t4*t13*(P(2,16)*magE-P(2,17)*magN);
	Kfusion[3] = -t4*t13*(P(3,16)*magE-P(3,17)*magN);
	Kfusion[4] = -t4*t13*(P(4,16)*magE-P(4,17)*magN);
	Kfusion[5] = -t4*t13*(P(5,16)*magE-P(5,17)*magN);
	Kfusion[6] = -t4*t13*(P(6,16)*magE-P(6,17)*magN);
	Kfusion[7] = -t4*t13*(P(7,16)*magE-P(7,17)*magN);
	Kfusion[8] = -t4*t13*(P(8,16)*magE-P(8,17)*magN);
	Kfusion[9] = -t4*t13*(P(9,16)*magE-P(9,17)*magN);
	Kfusion[10] = -t4*t13*(P(10,16)*magE-P(10,17)*magN);
	Kfusion[11] = -t4*t13*(P(11,16)*magE-P(11,17)*magN);
	Kfusion[12] = -t4*t13*(P(12,16)*magE-P(12,17)*magN);
	Kfusion[13] = -t4*t13*(P(13,16)*magE-P(13,17)*magN);
	Kfusion[14] = -t4*t13*(P(14,16)*magE-P(14,17)*magN);
	Kfusion[15] = -t4*t13*(P(15,16)*magE-P(15,17)*magN);
	Kfusion[16] = -t4*t13*(P(16,16)*magE-P(16,17)*magN);
	Kfusion[17] = -t4*t13*(P(17,16)*magE-P(17,17)*magN);
	Kfusion[18] = -t4*t13*(P(18,16)*magE-P(18,17)*magN);
	Kfusion[19] = -t4*t13*(P(19,16)*magE-P(19,17)*magN);
	Kfusion[20] = -t4*t13*(P(20,16)*magE-P(20,17)*magN);
	Kfusion[21] = -t4*t13*(P(21,16)*magE-P(21,17)*magN);
	Kfusion[22] = -t4*t13*(P(22,16)*magE-P(22,17)*magN);
	Kfusion[23] = -t4*t13*(P(23,16)*magE-P(23,17)*magN);

	const float innovation = math::constrain(atan2f(magE, magN) - getMagDeclination(), -0.5f, 0.5f);

	// apply covariance correction via P_new = (I -K*H)*P
	// first calculate expression for KHP
	// then calculate P - KHP
	// take advantage of the empty columns in KH to reduce the number of operations
	matrix::SquareMatrix<float, _k_num_states> KHP;
	float KH[2];
	for (unsigned row = 0; row < _k_num_states; row++) {

		KH[0] = Kfusion[row] * H_DECL[16];
		KH[1] = Kfusion[row] * H_DECL[17];

		for (unsigned column = 0; column < _k_num_states; column++) {
			float tmp = KH[0] * P(16,column);
			tmp += KH[1] * P(17,column);
			KHP(row,column) = tmp;
		}
	}

	// if the covariance correction will result in a negative variance, then
	// the covariance matrix is unhealthy and must be corrected
	bool healthy = true;
	_fault_status.flags.bad_mag_decl = false;
	for (int i = 0; i < _k_num_states; i++) {
		if (P(i,i) < KHP(i,i)) {
			// zero rows and columns
			P.uncorrelateCovarianceSetVariance<1>(i, 0.0f);

			//flag as unhealthy
			healthy = false;

			// update individual measurement health status
			_fault_status.flags.bad_mag_decl = true;

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
		fuse(Kfusion, innovation);

		// constrain the declination of the earth field states
		limitDeclination();
	}
}

void Ekf::limitDeclination()
{
	// get a reference value for the earth field declinaton and minimum plausible horizontal field strength
	// set to 50% of the horizontal strength from geo tables if location is known
	float decl_reference;
	float h_field_min = 0.001f;
	if (_params.mag_declination_source & MASK_USE_GEO_DECL) {
		// use parameter value until GPS is available, then use value returned by geo library
		if (_NED_origin_initialised) {
			decl_reference = _mag_declination_gps;
			h_field_min = fmaxf(h_field_min , 0.5f * _mag_strength_gps * cosf(_mag_inclination_gps));
		} else {
			decl_reference = math::radians(_params.mag_declination_deg);
		}
	} else {
		// always use the parameter value
		decl_reference = math::radians(_params.mag_declination_deg);
	}

	// do not allow the horizontal field length to collapse - this will make the declination fusion badly conditioned
	// and can result in a reversal of the NE field states which the filter cannot recover from
	// apply a circular limit
	float h_field = sqrtf(_state.mag_I(0)*_state.mag_I(0) + _state.mag_I(1)*_state.mag_I(1));
	if (h_field < h_field_min) {
		if (h_field > 0.001f * h_field_min) {
			float h_scaler = h_field_min / h_field;
			_state.mag_I(0) *= h_scaler;
			_state.mag_I(1) *= h_scaler;
		} else {
			// too small to scale radially so set to expected value
			float mag_declination = getMagDeclination();
			_state.mag_I(0) = 2.0f * h_field_min * cosf(mag_declination);
			_state.mag_I(1) = 2.0f * h_field_min * sinf(mag_declination);
		}
		h_field = h_field_min;
	}

	// do not allow the declination estimate to vary too much relative to the reference value
	const float decl_tolerance = 0.5f;
	const float decl_max = decl_reference + decl_tolerance;
	const float decl_min = decl_reference - decl_tolerance;
	const float decl_estimate = atan2f(_state.mag_I(1) , _state.mag_I(0));
	if (decl_estimate > decl_max)  {
		_state.mag_I(0) = h_field * cosf(decl_max);
		_state.mag_I(1) = h_field * sinf(decl_max);
	} else if (decl_estimate < decl_min)  {
		_state.mag_I(0) = h_field * cosf(decl_min);
		_state.mag_I(1) = h_field * sinf(decl_min);
	}
}

float Ekf::calculate_synthetic_mag_z_measurement(const Vector3f& mag_meas, const Vector3f& mag_earth_predicted)
{
	// theoretical magnitude of the magnetometer Z component value given X and Y sensor measurement and our knowledge
	// of the earth magnetic field vector at the current location
	const float mag_z_abs = sqrtf(math::max(sq(mag_earth_predicted.length()) - sq(mag_meas(0)) - sq(mag_meas(1)), 0.0f));

	// calculate sign of synthetic magnetomter Z component based on the sign of the predicted magnetomer Z component
	const float mag_z_body_pred = mag_earth_predicted.dot(_R_to_earth.slice<3,1>(0,2));

	return mag_z_body_pred < 0 ? -mag_z_abs : mag_z_abs;
}
