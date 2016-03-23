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
#include "mathlib.h"

void Ekf::fuseMag()
{
	// assign intermediate variables
	float q0 = _state.quat_nominal(0);
	float q1 = _state.quat_nominal(1);
	float q2 = _state.quat_nominal(2);
	float q3 = _state.quat_nominal(3);

	float magN = _state.mag_I(0);
	float magE = _state.mag_I(1);
	float magD = _state.mag_I(2);

	// XYZ Measurement uncertainty. Need to consider timing errors for fast rotations
	float R_MAG = fmaxf(_params.mag_noise, 1.0e-3f);
	R_MAG = R_MAG * R_MAG;

	// intermediate variables from algebraic optimisation
	float SH_MAG[9];
	SH_MAG[0] = sq(q0) - sq(q1) + sq(q2) - sq(q3);
	SH_MAG[1] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
	SH_MAG[2] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
	SH_MAG[3] = 2 * q0 * q1 + 2 * q2 * q3;
	SH_MAG[4] = 2 * q0 * q3 + 2 * q1 * q2;
	SH_MAG[5] = 2 * q0 * q2 + 2 * q1 * q3;
	SH_MAG[6] = magE * (2 * q0 * q1 - 2 * q2 * q3);
	SH_MAG[7] = 2 * q1 * q3 - 2 * q0 * q2;
	SH_MAG[8] = 2 * q0 * q3;

	// rotate magnetometer earth field state into body frame
	matrix::Dcm<float> R_to_body(_state.quat_nominal);
	R_to_body = R_to_body.transpose();

	Vector3f mag_I_rot = R_to_body * _state.mag_I;

	// compute magnetometer innovations
	_mag_innov[0] = (mag_I_rot(0) + _state.mag_B(0)) - _mag_sample_delayed.mag(0);
	_mag_innov[1] = (mag_I_rot(1) + _state.mag_B(1)) - _mag_sample_delayed.mag(1);
	_mag_innov[2] = (mag_I_rot(2) + _state.mag_B(2)) - _mag_sample_delayed.mag(2);

	// Note that although the observation jacobians and kalman gains are decalred as arrays
	// sequential fusion of the X,Y and Z components is used.
	float H_MAG[3][24] = {};
	float Kfusion[24] = {};

	// Calculate observation Jacobians and kalman gains for each magentoemter axis
	// X Axis
	H_MAG[0][1] = SH_MAG[6] - magD * SH_MAG[2] - magN * SH_MAG[5];
	H_MAG[0][2] = magE * SH_MAG[0] + magD * SH_MAG[3] - magN * (SH_MAG[8] - 2 * q1 * q2);
	H_MAG[0][16] = SH_MAG[1];
	H_MAG[0][17] = SH_MAG[4];
	H_MAG[0][18] = SH_MAG[7];
	H_MAG[0][19] = 1;

	// intermediate variables
	float SK_MX[4] = {};
	// innovation variance
	_mag_innov_var[0] = (P[19][19] + R_MAG - P[1][19] * (magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5]) + P[16][19] *
			     SH_MAG[1]
			     + P[17][19] * SH_MAG[4] + P[18][19] * SH_MAG[7] + P[2][19] * (magE * SH_MAG[0] + magD * SH_MAG[3] - magN *
					     (SH_MAG[8] - 2 * q1 * q2)) - (magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5]) * (P[19][1] - P[1][1] *
							     (magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5]) + P[16][1] * SH_MAG[1] + P[17][1] * SH_MAG[4] + P[18][1] * SH_MAG[7] +
							     P[2][1] * (magE * SH_MAG[0] + magD * SH_MAG[3] - magN * (SH_MAG[8] - 2 * q1 * q2))) + SH_MAG[1] *
			     (P[19][16] - P[1][16] * (magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5]) + P[16][16] * SH_MAG[1] + P[17][16] *
			      SH_MAG[4] + P[18][16] * SH_MAG[7] + P[2][16] * (magE * SH_MAG[0] + magD * SH_MAG[3] - magN *
					      (SH_MAG[8] - 2 * q1 * q2))) + SH_MAG[4] * (P[19][17] - P[1][17] * (magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5]) +
							      P[16][17] * SH_MAG[1] + P[17][17] * SH_MAG[4] + P[18][17] * SH_MAG[7] + P[2][17] * (magE * SH_MAG[0] + magD * SH_MAG[3]
									      - magN * (SH_MAG[8] - 2 * q1 * q2))) + SH_MAG[7] * (P[19][18] - P[1][18] * (magD * SH_MAG[2] - SH_MAG[6] + magN *
											      SH_MAG[5]) + P[16][18] * SH_MAG[1] + P[17][18] * SH_MAG[4] + P[18][18] * SH_MAG[7] + P[2][18] *
											      (magE * SH_MAG[0] + magD * SH_MAG[3] - magN * (SH_MAG[8] - 2 * q1 * q2))) + (magE * SH_MAG[0] + magD * SH_MAG[3] -
													      magN * (SH_MAG[8] - 2 * q1 * q2)) * (P[19][2] - P[1][2] * (magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5]) + P[16][2] *
															      SH_MAG[1] + P[17][2] * SH_MAG[4] + P[18][2] * SH_MAG[7] + P[2][2] * (magE * SH_MAG[0] + magD * SH_MAG[3] - magN *
																	      (SH_MAG[8] - 2 * q1 * q2))));

	// check for a badly conditioned covariance matrix
	if (_mag_innov_var[0] >= R_MAG) {
		// the innovation variance contribution from the state covariances is non-negative - no fault
		_fault_status.bad_mag_x = false;

	} else {
		// the innovation variance contribution from the state covariances is negtive which means the covariance matrix is badly conditioned
		_fault_status.bad_mag_x = true;
		// we need to reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		return;
	}


	// Y axis

	H_MAG[1][0] = magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5];
	H_MAG[1][2] = - magE * SH_MAG[4] - magD * SH_MAG[7] - magN * SH_MAG[1];
	H_MAG[1][16] = 2 * q1 * q2 - SH_MAG[8];
	H_MAG[1][17] = SH_MAG[0];
	H_MAG[1][18] = SH_MAG[3];
	H_MAG[1][20] = 1;

	// intermediate variables - note SK_MY[0] is 1/(innovation variance)
	float SK_MY[4];
	_mag_innov_var[1] = (P[20][20] + R_MAG + P[0][20] * (magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5]) + P[17][20] *
			     SH_MAG[0]
			     + P[18][20] * SH_MAG[3] - (SH_MAG[8] - 2 * q1 * q2) * (P[20][16] + P[0][16] * (magD * SH_MAG[2] - SH_MAG[6] + magN *
					     SH_MAG[5]) + P[17][16] * SH_MAG[0] + P[18][16] * SH_MAG[3] - P[2][16] * (magE * SH_MAG[4] + magD * SH_MAG[7] + magN *
							     SH_MAG[1]) - P[16][16] * (SH_MAG[8] - 2 * q1 * q2)) - P[2][20] * (magE * SH_MAG[4] + magD * SH_MAG[7] + magN *
									     SH_MAG[1]) + (magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5]) * (P[20][0] + P[0][0] *
											     (magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5]) + P[17][0] * SH_MAG[0] + P[18][0] * SH_MAG[3] - P[2][0] *
											     (magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1]) - P[16][0] * (SH_MAG[8] - 2 * q1 * q2)) + SH_MAG[0] *
			     (P[20][17] + P[0][17] * (magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5]) + P[17][17] * SH_MAG[0] + P[18][17] *
			      SH_MAG[3] - P[2][17] * (magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1]) - P[16][17] *
			      (SH_MAG[8] - 2 * q1 * q2)) + SH_MAG[3] * (P[20][18] + P[0][18] * (magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5]) +
					      P[17][18] * SH_MAG[0] + P[18][18] * SH_MAG[3] - P[2][18] * (magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1]) -
					      P[16][18] * (SH_MAG[8] - 2 * q1 * q2)) - P[16][20] * (SH_MAG[8] - 2 * q1 * q2) - (magE * SH_MAG[4] + magD * SH_MAG[7] +
							      magN * SH_MAG[1]) * (P[20][2] + P[0][2] * (magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5]) + P[17][2] * SH_MAG[0] +
									      P[18][2] * SH_MAG[3] - P[2][2] * (magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1]) - P[16][2] *
									      (SH_MAG[8] - 2 * q1 * q2)));

	// check for a badly conditioned covariance matrix
	if (_mag_innov_var[1] >= R_MAG) {
		// the innovation variance contribution from the state covariances is non-negative - no fault
		_fault_status.bad_mag_y = false;

	} else {
		// the innovation variance contribution from the state covariances is negtive which means the covariance matrix is badly conditioned
		_fault_status.bad_mag_y = true;
		// we need to reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		return;
	}


	// Z axis

	H_MAG[2][0] = magN * (SH_MAG[8] - 2 * q1 * q2) - magD * SH_MAG[3] - magE * SH_MAG[0];
	H_MAG[2][1] = magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1];
	H_MAG[2][16] = SH_MAG[5];
	H_MAG[2][17] = 2 * q2 * q3 - 2 * q0 * q1;
	H_MAG[2][18] = SH_MAG[2];
	H_MAG[2][21] = 1;

	// intermediate variables
	float SK_MZ[4];
	_mag_innov_var[2] = (P[21][21] + R_MAG + P[16][21] * SH_MAG[5] + P[18][21] * SH_MAG[2] - (2 * q0 * q1 - 2 * q2 * q3) *
			     (P[21][17] + P[16][17] * SH_MAG[5] + P[18][17] * SH_MAG[2] - P[0][17] * (magE * SH_MAG[0] + magD * SH_MAG[3] - magN *
					     (SH_MAG[8] - 2 * q1 * q2)) + P[1][17] * (magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1]) - P[17][17] *
			      (2 * q0 * q1 - 2 * q2 * q3)) - P[0][21] * (magE * SH_MAG[0] + magD * SH_MAG[3] - magN *
					      (SH_MAG[8] - 2 * q1 * q2)) + P[1][21] * (magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1]) + SH_MAG[5] *
			     (P[21][16] + P[16][16] * SH_MAG[5] + P[18][16] * SH_MAG[2] - P[0][16] * (magE * SH_MAG[0] + magD * SH_MAG[3] - magN *
					     (SH_MAG[8] - 2 * q1 * q2)) + P[1][16] * (magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1]) - P[17][16] *
			      (2 * q0 * q1 - 2 * q2 * q3)) + SH_MAG[2] * (P[21][18] + P[16][18] * SH_MAG[5] + P[18][18] * SH_MAG[2] - P[0][18] *
					      (magE * SH_MAG[0] + magD * SH_MAG[3] - magN * (SH_MAG[8] - 2 * q1 * q2)) + P[1][18] *
					      (magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1]) - P[17][18] * (2 * q0 * q1 - 2 * q2 * q3)) -
			     (magE * SH_MAG[0] + magD * SH_MAG[3] - magN * (SH_MAG[8] - 2 * q1 * q2)) * (P[21][0] + P[16][0] * SH_MAG[5] + P[18][0] *
					     SH_MAG[2] - P[0][0] * (magE * SH_MAG[0] + magD * SH_MAG[3] - magN * (SH_MAG[8] - 2 * q1 * q2)) + P[1][0] *
					     (magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1]) - P[17][0] * (2 * q0 * q1 - 2 * q2 * q3)) - P[17][21] *
			     (2 * q0 * q1 - 2 * q2 * q3) + (magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1]) *
			     (P[21][1] + P[16][1] * SH_MAG[5] + P[18][1] * SH_MAG[2] - P[0][1] * (magE * SH_MAG[0] + magD * SH_MAG[3] - magN *
					     (SH_MAG[8] - 2 * q1 * q2)) + P[1][1] * (magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1]) - P[17][1] *
			      (2 * q0 * q1 - 2 * q2 * q3)));

	// check for a badly conditioned covariance matrix
	if (_mag_innov_var[2] >= R_MAG) {
		// the innovation variance contribution from the state covariances is non-negative - no fault
		_fault_status.bad_mag_z = false;

	} else {
		// the innovation variance contribution from the state covariances is negtive which means the covariance matrix is badly conditioned
		_fault_status.bad_mag_z = true;
		// we need to reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		return;
	}


	// Perform an innovation consistency check on each measurement and if one axis fails
	// do not fuse any data from the sensor because the most common errors affect multiple axes.
	_mag_healthy = true;

	for (uint8_t index = 0; index <= 2; index++) {
		_mag_test_ratio[index] = sq(_mag_innov[index]) / (sq(math::max(_params.mag_innov_gate, 1.0f)) * _mag_innov_var[index]);

		if (_mag_test_ratio[index] > 1.0f) {
			_mag_healthy = false;
		}
	}

	if (!_mag_healthy) {
		return;
	}

	// update the states and covariance usinng sequential fusion of the magnetometer components
	for (uint8_t index = 0; index <= 2; index++) {
		// Calculate Kalman gains
		if (index == 0) {
			// Calculate X axis Kalman gains
			SK_MX[0] = 1.0f / _mag_innov_var[0];
			SK_MX[1] = magE * SH_MAG[0] + magD * SH_MAG[3] - magN * (SH_MAG[8] - 2 * q1 * q2);
			SK_MX[2] = magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5];
			SK_MX[3] = SH_MAG[7];

			Kfusion[0] = SK_MX[0] * (P[0][19] + P[0][16] * SH_MAG[1] + P[0][17] * SH_MAG[4] - P[0][1] * SK_MX[2] + P[0][2] *
						 SK_MX[1] + P[0][18] * SK_MX[3]);
			Kfusion[1] = SK_MX[0] * (P[1][19] + P[1][16] * SH_MAG[1] + P[1][17] * SH_MAG[4] - P[1][1] * SK_MX[2] + P[1][2] *
						 SK_MX[1] + P[1][18] * SK_MX[3]);
			Kfusion[2] = SK_MX[0] * (P[2][19] + P[2][16] * SH_MAG[1] + P[2][17] * SH_MAG[4] - P[2][1] * SK_MX[2] + P[2][2] *
						 SK_MX[1] + P[2][18] * SK_MX[3]);
			Kfusion[3] = SK_MX[0] * (P[3][19] + P[3][16] * SH_MAG[1] + P[3][17] * SH_MAG[4] - P[3][1] * SK_MX[2] + P[3][2] *
						 SK_MX[1] + P[3][18] * SK_MX[3]);
			Kfusion[4] = SK_MX[0] * (P[4][19] + P[4][16] * SH_MAG[1] + P[4][17] * SH_MAG[4] - P[4][1] * SK_MX[2] + P[4][2] *
						 SK_MX[1] + P[4][18] * SK_MX[3]);
			Kfusion[5] = SK_MX[0] * (P[5][19] + P[5][16] * SH_MAG[1] + P[5][17] * SH_MAG[4] - P[5][1] * SK_MX[2] + P[5][2] *
						 SK_MX[1] + P[5][18] * SK_MX[3]);
			Kfusion[6] = SK_MX[0] * (P[6][19] + P[6][16] * SH_MAG[1] + P[6][17] * SH_MAG[4] - P[6][1] * SK_MX[2] + P[6][2] *
						 SK_MX[1] + P[6][18] * SK_MX[3]);
			Kfusion[7] = SK_MX[0] * (P[7][19] + P[7][16] * SH_MAG[1] + P[7][17] * SH_MAG[4] - P[7][1] * SK_MX[2] + P[7][2] *
						 SK_MX[1] + P[7][18] * SK_MX[3]);
			Kfusion[8] = SK_MX[0] * (P[8][19] + P[8][16] * SH_MAG[1] + P[8][17] * SH_MAG[4] - P[8][1] * SK_MX[2] + P[8][2] *
						 SK_MX[1] + P[8][18] * SK_MX[3]);
			Kfusion[9] = SK_MX[0] * (P[9][19] + P[9][16] * SH_MAG[1] + P[9][17] * SH_MAG[4] - P[9][1] * SK_MX[2] + P[9][2] *
						 SK_MX[1] + P[9][18] * SK_MX[3]);
			Kfusion[10] = SK_MX[0] * (P[10][19] + P[10][16] * SH_MAG[1] + P[10][17] * SH_MAG[4] - P[10][1] * SK_MX[2] + P[10][2] *
						  SK_MX[1] + P[10][18] * SK_MX[3]);
			Kfusion[11] = SK_MX[0] * (P[11][19] + P[11][16] * SH_MAG[1] + P[11][17] * SH_MAG[4] - P[11][1] * SK_MX[2] + P[11][2] *
						  SK_MX[1] + P[11][18] * SK_MX[3]);
			Kfusion[12] = SK_MX[0] * (P[12][19] + P[12][16] * SH_MAG[1] + P[12][17] * SH_MAG[4] - P[12][1] * SK_MX[2] + P[12][2] *
						  SK_MX[1] + P[12][18] * SK_MX[3]);
			Kfusion[13] = SK_MX[0] * (P[13][19] + P[13][16] * SH_MAG[1] + P[13][17] * SH_MAG[4] - P[13][1] * SK_MX[2] + P[13][2] *
						  SK_MX[1] + P[13][18] * SK_MX[3]);
			Kfusion[14] = SK_MX[0] * (P[14][19] + P[14][16] * SH_MAG[1] + P[14][17] * SH_MAG[4] - P[14][1] * SK_MX[2] + P[14][2] *
						  SK_MX[1] + P[14][18] * SK_MX[3]);
			Kfusion[15] = SK_MX[0] * (P[15][19] + P[15][16] * SH_MAG[1] + P[15][17] * SH_MAG[4] - P[15][1] * SK_MX[2] + P[15][2] *
						  SK_MX[1] + P[15][18] * SK_MX[3]);
			Kfusion[16] = SK_MX[0] * (P[16][19] + P[16][16] * SH_MAG[1] + P[16][17] * SH_MAG[4] - P[16][1] * SK_MX[2] + P[16][2] *
						  SK_MX[1] + P[16][18] * SK_MX[3]);
			Kfusion[17] = SK_MX[0] * (P[17][19] + P[17][16] * SH_MAG[1] + P[17][17] * SH_MAG[4] - P[17][1] * SK_MX[2] + P[17][2] *
						  SK_MX[1] + P[17][18] * SK_MX[3]);
			Kfusion[18] = SK_MX[0] * (P[18][19] + P[18][16] * SH_MAG[1] + P[18][17] * SH_MAG[4] - P[18][1] * SK_MX[2] + P[18][2] *
						  SK_MX[1] + P[18][18] * SK_MX[3]);
			Kfusion[19] = SK_MX[0] * (P[19][19] + P[19][16] * SH_MAG[1] + P[19][17] * SH_MAG[4] - P[19][1] * SK_MX[2] + P[19][2] *
						  SK_MX[1] + P[19][18] * SK_MX[3]);
			Kfusion[20] = SK_MX[0] * (P[20][19] + P[20][16] * SH_MAG[1] + P[20][17] * SH_MAG[4] - P[20][1] * SK_MX[2] + P[20][2] *
						  SK_MX[1] + P[20][18] * SK_MX[3]);
			Kfusion[21] = SK_MX[0] * (P[21][19] + P[21][16] * SH_MAG[1] + P[21][17] * SH_MAG[4] - P[21][1] * SK_MX[2] + P[21][2] *
						  SK_MX[1] + P[21][18] * SK_MX[3]);

			// Don't update wind states unless we are doing wind estimation
			if (_control_status.flags.wind) {
				Kfusion[22] = SK_MX[0] * (P[22][19] + P[22][16] * SH_MAG[1] + P[22][17] * SH_MAG[4] - P[22][1] * SK_MX[2] + P[22][2] *
							  SK_MX[1] + P[22][18] * SK_MX[3]);
				Kfusion[23] = SK_MX[0] * (P[23][19] + P[23][16] * SH_MAG[1] + P[23][17] * SH_MAG[4] - P[23][1] * SK_MX[2] + P[23][2] *
							  SK_MX[1] + P[23][18] * SK_MX[3]);

			} else {
				Kfusion[22] = 0.0f;
				Kfusion[23] = 0.0f;
			}

		} else if (index == 1) {
			// Calculate Y axis Kalman gains
			SK_MY[0] = 1.0f / _mag_innov_var[1];
			SK_MY[1] = magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1];
			SK_MY[2] = magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5];
			SK_MY[3] = SH_MAG[8] - 2 * q1 * q2;

			Kfusion[0] = SK_MY[0] * (P[0][20] + P[0][17] * SH_MAG[0] + P[0][18] * SH_MAG[3] + P[0][0] * SK_MY[2] - P[0][2] *
						 SK_MY[1] - P[0][16] * SK_MY[3]);
			Kfusion[1] = SK_MY[0] * (P[1][20] + P[1][17] * SH_MAG[0] + P[1][18] * SH_MAG[3] + P[1][0] * SK_MY[2] - P[1][2] *
						 SK_MY[1] - P[1][16] * SK_MY[3]);
			Kfusion[2] = SK_MY[0] * (P[2][20] + P[2][17] * SH_MAG[0] + P[2][18] * SH_MAG[3] + P[2][0] * SK_MY[2] - P[2][2] *
						 SK_MY[1] - P[2][16] * SK_MY[3]);
			Kfusion[3] = SK_MY[0] * (P[3][20] + P[3][17] * SH_MAG[0] + P[3][18] * SH_MAG[3] + P[3][0] * SK_MY[2] - P[3][2] *
						 SK_MY[1] - P[3][16] * SK_MY[3]);
			Kfusion[4] = SK_MY[0] * (P[4][20] + P[4][17] * SH_MAG[0] + P[4][18] * SH_MAG[3] + P[4][0] * SK_MY[2] - P[4][2] *
						 SK_MY[1] - P[4][16] * SK_MY[3]);
			Kfusion[5] = SK_MY[0] * (P[5][20] + P[5][17] * SH_MAG[0] + P[5][18] * SH_MAG[3] + P[5][0] * SK_MY[2] - P[5][2] *
						 SK_MY[1] - P[5][16] * SK_MY[3]);
			Kfusion[6] = SK_MY[0] * (P[6][20] + P[6][17] * SH_MAG[0] + P[6][18] * SH_MAG[3] + P[6][0] * SK_MY[2] - P[6][2] *
						 SK_MY[1] - P[6][16] * SK_MY[3]);
			Kfusion[7] = SK_MY[0] * (P[7][20] + P[7][17] * SH_MAG[0] + P[7][18] * SH_MAG[3] + P[7][0] * SK_MY[2] - P[7][2] *
						 SK_MY[1] - P[7][16] * SK_MY[3]);
			Kfusion[8] = SK_MY[0] * (P[8][20] + P[8][17] * SH_MAG[0] + P[8][18] * SH_MAG[3] + P[8][0] * SK_MY[2] - P[8][2] *
						 SK_MY[1] - P[8][16] * SK_MY[3]);
			Kfusion[9] = SK_MY[0] * (P[9][20] + P[9][17] * SH_MAG[0] + P[9][18] * SH_MAG[3] + P[9][0] * SK_MY[2] - P[9][2] *
						 SK_MY[1] - P[9][16] * SK_MY[3]);
			Kfusion[10] = SK_MY[0] * (P[10][20] + P[10][17] * SH_MAG[0] + P[10][18] * SH_MAG[3] + P[10][0] * SK_MY[2] - P[10][2] *
						  SK_MY[1] - P[10][16] * SK_MY[3]);
			Kfusion[11] = SK_MY[0] * (P[11][20] + P[11][17] * SH_MAG[0] + P[11][18] * SH_MAG[3] + P[11][0] * SK_MY[2] - P[11][2] *
						  SK_MY[1] - P[11][16] * SK_MY[3]);
			Kfusion[12] = SK_MY[0] * (P[12][20] + P[12][17] * SH_MAG[0] + P[12][18] * SH_MAG[3] + P[12][0] * SK_MY[2] - P[12][2] *
						  SK_MY[1] - P[12][16] * SK_MY[3]);
			Kfusion[13] = SK_MY[0] * (P[13][20] + P[13][17] * SH_MAG[0] + P[13][18] * SH_MAG[3] + P[13][0] * SK_MY[2] - P[13][2] *
						  SK_MY[1] - P[13][16] * SK_MY[3]);
			Kfusion[14] = SK_MY[0] * (P[14][20] + P[14][17] * SH_MAG[0] + P[14][18] * SH_MAG[3] + P[14][0] * SK_MY[2] - P[14][2] *
						  SK_MY[1] - P[14][16] * SK_MY[3]);
			Kfusion[15] = SK_MY[0] * (P[15][20] + P[15][17] * SH_MAG[0] + P[15][18] * SH_MAG[3] + P[15][0] * SK_MY[2] - P[15][2] *
						  SK_MY[1] - P[15][16] * SK_MY[3]);
			Kfusion[16] = SK_MY[0] * (P[16][20] + P[16][17] * SH_MAG[0] + P[16][18] * SH_MAG[3] + P[16][0] * SK_MY[2] - P[16][2] *
						  SK_MY[1] - P[16][16] * SK_MY[3]);
			Kfusion[17] = SK_MY[0] * (P[17][20] + P[17][17] * SH_MAG[0] + P[17][18] * SH_MAG[3] + P[17][0] * SK_MY[2] - P[17][2] *
						  SK_MY[1] - P[17][16] * SK_MY[3]);
			Kfusion[18] = SK_MY[0] * (P[18][20] + P[18][17] * SH_MAG[0] + P[18][18] * SH_MAG[3] + P[18][0] * SK_MY[2] - P[18][2] *
						  SK_MY[1] - P[18][16] * SK_MY[3]);
			Kfusion[19] = SK_MY[0] * (P[19][20] + P[19][17] * SH_MAG[0] + P[19][18] * SH_MAG[3] + P[19][0] * SK_MY[2] - P[19][2] *
						  SK_MY[1] - P[19][16] * SK_MY[3]);
			Kfusion[20] = SK_MY[0] * (P[20][20] + P[20][17] * SH_MAG[0] + P[20][18] * SH_MAG[3] + P[20][0] * SK_MY[2] - P[20][2] *
						  SK_MY[1] - P[20][16] * SK_MY[3]);
			Kfusion[21] = SK_MY[0] * (P[21][20] + P[21][17] * SH_MAG[0] + P[21][18] * SH_MAG[3] + P[21][0] * SK_MY[2] - P[21][2] *
						  SK_MY[1] - P[21][16] * SK_MY[3]);

			// Don't update wind states unless we are doing wind estimation
			if (_control_status.flags.wind) {
				Kfusion[22] = SK_MY[0] * (P[22][20] + P[22][17] * SH_MAG[0] + P[22][18] * SH_MAG[3] + P[22][0] * SK_MY[2] - P[22][2] *
							  SK_MY[1] - P[22][16] * SK_MY[3]);
				Kfusion[23] = SK_MY[0] * (P[23][20] + P[23][17] * SH_MAG[0] + P[23][18] * SH_MAG[3] + P[23][0] * SK_MY[2] - P[23][2] *
							  SK_MY[1] - P[23][16] * SK_MY[3]);

			} else {
				Kfusion[22] = 0.0f;
				Kfusion[23] = 0.0f;
			}

		} else if (index == 2) {
			// Calculate Z axis Kalman gains
			SK_MZ[0] = 1.0f / _mag_innov_var[2];
			SK_MZ[1] = magE * SH_MAG[0] + magD * SH_MAG[3] - magN * (SH_MAG[8] - 2 * q1 * q2);
			SK_MZ[2] = magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1];
			SK_MZ[3] = 2 * q0 * q1 - 2 * q2 * q3;

			Kfusion[0] = SK_MZ[0] * (P[0][21] + P[0][18] * SH_MAG[2] + P[0][16] * SH_MAG[5] - P[0][0] * SK_MZ[1] + P[0][1] *
						 SK_MZ[2] - P[0][17] * SK_MZ[3]);
			Kfusion[1] = SK_MZ[0] * (P[1][21] + P[1][18] * SH_MAG[2] + P[1][16] * SH_MAG[5] - P[1][0] * SK_MZ[1] + P[1][1] *
						 SK_MZ[2] - P[1][17] * SK_MZ[3]);
			Kfusion[2] = SK_MZ[0] * (P[2][21] + P[2][18] * SH_MAG[2] + P[2][16] * SH_MAG[5] - P[2][0] * SK_MZ[1] + P[2][1] *
						 SK_MZ[2] - P[2][17] * SK_MZ[3]);
			Kfusion[3] = SK_MZ[0] * (P[3][21] + P[3][18] * SH_MAG[2] + P[3][16] * SH_MAG[5] - P[3][0] * SK_MZ[1] + P[3][1] *
						 SK_MZ[2] - P[3][17] * SK_MZ[3]);
			Kfusion[4] = SK_MZ[0] * (P[4][21] + P[4][18] * SH_MAG[2] + P[4][16] * SH_MAG[5] - P[4][0] * SK_MZ[1] + P[4][1] *
						 SK_MZ[2] - P[4][17] * SK_MZ[3]);
			Kfusion[5] = SK_MZ[0] * (P[5][21] + P[5][18] * SH_MAG[2] + P[5][16] * SH_MAG[5] - P[5][0] * SK_MZ[1] + P[5][1] *
						 SK_MZ[2] - P[5][17] * SK_MZ[3]);
			Kfusion[6] = SK_MZ[0] * (P[6][21] + P[6][18] * SH_MAG[2] + P[6][16] * SH_MAG[5] - P[6][0] * SK_MZ[1] + P[6][1] *
						 SK_MZ[2] - P[6][17] * SK_MZ[3]);
			Kfusion[7] = SK_MZ[0] * (P[7][21] + P[7][18] * SH_MAG[2] + P[7][16] * SH_MAG[5] - P[7][0] * SK_MZ[1] + P[7][1] *
						 SK_MZ[2] - P[7][17] * SK_MZ[3]);
			Kfusion[8] = SK_MZ[0] * (P[8][21] + P[8][18] * SH_MAG[2] + P[8][16] * SH_MAG[5] - P[8][0] * SK_MZ[1] + P[8][1] *
						 SK_MZ[2] - P[8][17] * SK_MZ[3]);
			Kfusion[9] = SK_MZ[0] * (P[9][21] + P[9][18] * SH_MAG[2] + P[9][16] * SH_MAG[5] - P[9][0] * SK_MZ[1] + P[9][1] *
						 SK_MZ[2] - P[9][17] * SK_MZ[3]);
			Kfusion[10] = SK_MZ[0] * (P[10][21] + P[10][18] * SH_MAG[2] + P[10][16] * SH_MAG[5] - P[10][0] * SK_MZ[1] + P[10][1] *
						  SK_MZ[2] - P[10][17] * SK_MZ[3]);
			Kfusion[11] = SK_MZ[0] * (P[11][21] + P[11][18] * SH_MAG[2] + P[11][16] * SH_MAG[5] - P[11][0] * SK_MZ[1] + P[11][1] *
						  SK_MZ[2] - P[11][17] * SK_MZ[3]);
			Kfusion[12] = SK_MZ[0] * (P[12][21] + P[12][18] * SH_MAG[2] + P[12][16] * SH_MAG[5] - P[12][0] * SK_MZ[1] + P[12][1] *
						  SK_MZ[2] - P[12][17] * SK_MZ[3]);
			Kfusion[13] = SK_MZ[0] * (P[13][21] + P[13][18] * SH_MAG[2] + P[13][16] * SH_MAG[5] - P[13][0] * SK_MZ[1] + P[13][1] *
						  SK_MZ[2] - P[13][17] * SK_MZ[3]);
			Kfusion[14] = SK_MZ[0] * (P[14][21] + P[14][18] * SH_MAG[2] + P[14][16] * SH_MAG[5] - P[14][0] * SK_MZ[1] + P[14][1] *
						  SK_MZ[2] - P[14][17] * SK_MZ[3]);
			Kfusion[15] = SK_MZ[0] * (P[15][21] + P[15][18] * SH_MAG[2] + P[15][16] * SH_MAG[5] - P[15][0] * SK_MZ[1] + P[15][1] *
						  SK_MZ[2] - P[15][17] * SK_MZ[3]);
			Kfusion[16] = SK_MZ[0] * (P[16][21] + P[16][18] * SH_MAG[2] + P[16][16] * SH_MAG[5] - P[16][0] * SK_MZ[1] + P[16][1] *
						  SK_MZ[2] - P[16][17] * SK_MZ[3]);
			Kfusion[17] = SK_MZ[0] * (P[17][21] + P[17][18] * SH_MAG[2] + P[17][16] * SH_MAG[5] - P[17][0] * SK_MZ[1] + P[17][1] *
						  SK_MZ[2] - P[17][17] * SK_MZ[3]);
			Kfusion[18] = SK_MZ[0] * (P[18][21] + P[18][18] * SH_MAG[2] + P[18][16] * SH_MAG[5] - P[18][0] * SK_MZ[1] + P[18][1] *
						  SK_MZ[2] - P[18][17] * SK_MZ[3]);
			Kfusion[19] = SK_MZ[0] * (P[19][21] + P[19][18] * SH_MAG[2] + P[19][16] * SH_MAG[5] - P[19][0] * SK_MZ[1] + P[19][1] *
						  SK_MZ[2] - P[19][17] * SK_MZ[3]);
			Kfusion[20] = SK_MZ[0] * (P[20][21] + P[20][18] * SH_MAG[2] + P[20][16] * SH_MAG[5] - P[20][0] * SK_MZ[1] + P[20][1] *
						  SK_MZ[2] - P[20][17] * SK_MZ[3]);
			Kfusion[21] = SK_MZ[0] * (P[21][21] + P[21][18] * SH_MAG[2] + P[21][16] * SH_MAG[5] - P[21][0] * SK_MZ[1] + P[21][1] *
						  SK_MZ[2] - P[21][17] * SK_MZ[3]);

			// Don't update wind states unless we are doing wind estimation
			if (_control_status.flags.wind) {
				Kfusion[22] = SK_MZ[0] * (P[22][21] + P[22][18] * SH_MAG[2] + P[22][16] * SH_MAG[5] - P[22][0] * SK_MZ[1] + P[22][1] *
							  SK_MZ[2] - P[22][17] * SK_MZ[3]);
				Kfusion[23] = SK_MZ[0] * (P[23][21] + P[23][18] * SH_MAG[2] + P[23][16] * SH_MAG[5] - P[23][0] * SK_MZ[1] + P[23][1] *
							  SK_MZ[2] - P[23][17] * SK_MZ[3]);

			} else {
				Kfusion[22] = 0.0f;
				Kfusion[23] = 0.0f;
			}

		} else {
			return;
		}

		// by definition our error state is zero at the time of fusion
		_state.ang_error.setZero();

		fuse(Kfusion, _mag_innov[index]);

		Quaternion q_correction;
		q_correction.from_axis_angle(_state.ang_error);
		_state.quat_nominal = q_correction * _state.quat_nominal;
		_state.quat_nominal.normalize();
		_state.ang_error.setZero();

		// apply covariance correction via P_new = (I -K*H)*P
		// first calculate expression for KHP
		// then calculate P - KHP
		for (unsigned row = 0; row < _k_num_states; row++) {
			for (unsigned column = 0; column <= 2; column++) {
				KH[row][column] = Kfusion[row] * H_MAG[index][column];
			}

			for (unsigned column = 16; column <= 21; column++) {
				KH[row][column] = Kfusion[row] * H_MAG[index][column];
			}

		}

		for (unsigned row = 0; row < _k_num_states; row++) {
			for (unsigned column = 0; column < _k_num_states; column++) {
				float tmp = KH[row][0] * P[0][column];
				tmp += KH[row][1] * P[1][column];
				tmp += KH[row][2] * P[2][column];
				tmp += KH[row][16] * P[16][column];
				tmp += KH[row][17] * P[17][column];
				tmp += KH[row][18] * P[18][column];
				tmp += KH[row][19] * P[19][column];
				tmp += KH[row][20] * P[20][column];
				tmp += KH[row][21] * P[21][column];
				KHP[row][column] = tmp;
			}
		}

		for (unsigned row = 0; row < _k_num_states; row++) {
			for (unsigned column = 0; column < _k_num_states; column++) {
				P[row][column] -= KHP[row][column];
			}
		}

		makeSymmetrical();
		limitCov();
	}
}

void Ekf::fuseHeading()
{
	// assign intermediate state variables
	float q0 = _state.quat_nominal(0);
	float q1 = _state.quat_nominal(1);
	float q2 = _state.quat_nominal(2);
	float q3 = _state.quat_nominal(3);

	float R_YAW = fmaxf(_params.mag_heading_noise, 1.0e-2f);
	R_YAW = R_YAW * R_YAW;

	float predicted_hdg;
	float H_YAW[3];
	matrix::Vector3f mag_earth_pred;

	// determine if a 321 or 312 Euler sequence is best
	if (fabsf(_R_prev(0, 2)) < fabsf(_R_prev(1, 2))) {
		// calculate observation jacobian when we are observing the first rotation in a 321 sequence
		float t2 = q0 * q0;
		float t3 = q1 * q1;
		float t4 = q2 * q2;
		float t5 = q3 * q3;
		float t6 = t2 + t3 - t4 - t5;
		float t7 = q0 * q3 * 2.0f;
		float t8 = q1 * q2 * 2.0f;
		float t9 = t7 + t8;
		float t10 = sq(t6);

		if (t10 > 1e-6f) {
			t10 = 1.0f / t10;

		} else {
			return;
		}

		float t11 = t9 * t9;
		float t12 = t10 * t11;
		float t13 = t12 + 1.0f;
		float t14;

		if (fabsf(t13) > 1e-3f) {
			t14 = 1.0f / t13;

		} else {
			return;
		}

		float t15 = 1.0f / t6;

		H_YAW[0] = 0.0f;
		H_YAW[1] = t14 * (t15 * (q0 * q1 * 2.0f - q2 * q3 * 2.0f) + t9 * t10 * (q0 * q2 * 2.0f + q1 * q3 * 2.0f));
		H_YAW[2] = t14 * (t15 * (t2 - t3 + t4 - t5) + t9 * t10 * (t7 - t8));

		// rotate the magnetometer measurement into earth frame
		matrix::Euler<float> euler321(_state.quat_nominal);
		predicted_hdg = euler321(2); // we will need the predicted heading to calculate the innovation

		// Set the yaw angle to zero and rotate the measurements into earth frame using the zero yaw angle
		euler321(2) = 0.0f;
		matrix::Dcm<float> R_to_earth(euler321);

		// rotate the magnetometer measurements into earth frame using a zero yaw angle
		mag_earth_pred = R_to_earth * _mag_sample_delayed.mag;

	} else {
		// calculate observaton jacobian when we are observing a rotation in a 312 sequence
		float t2 = q0 * q0;
		float t3 = q1 * q1;
		float t4 = q2 * q2;
		float t5 = q3 * q3;
		float t6 = t2 - t3 + t4 - t5;
		float t7 = q0 * q3 * 2.0f;
		float t10 = q1 * q2 * 2.0f;
		float t8 = t7 - t10;
		float t9 = sq(t6);

		if (t9 > 1e-6f) {
			t9 = 1.0f / t9;

		} else {
			return;
		}

		float t11 = t8 * t8;
		float t12 = t9 * t11;
		float t13 = t12 + 1.0f;
		float t14;

		if (fabsf(t13) > 1e-3f) {
			t14 = 1.0f / t13;

		} else {
			return;
		}

		float t15 = 1.0f / t6;

		H_YAW[0] = -t14 * (t15 * (q0 * q2 * 2.0f + q1 * q3 * 2.0f) - t8 * t9 * (q0 * q1 * 2.0f - q2 * q3 * 2.0f));
		H_YAW[1] = 0.0f;
		H_YAW[2] = t14 * (t15 * (t2 + t3 - t4 - t5) + t8 * t9 * (t7 + t10));

		// Calculate the 312 sequence euler angles that rotate from earth to body frame
		// See http://www.atacolorado.com/eulersequences.doc
		Vector3f euler312;
		euler312(0) = atan2f(-_R_prev(1, 0) , _R_prev(1, 1)); // first rotation (yaw)
		euler312(1) = asinf(_R_prev(1, 2)); // second rotation (roll)
		euler312(2) = atan2f(-_R_prev(0, 2) , _R_prev(2, 2)); // third rotation (pitch)

		predicted_hdg = euler312(0); // we will need the predicted heading to calculate the innovation

		// Set the first rotation (yaw) to zero and rotate the measurements into earth frame
		euler312(0) = 0.0f;

		// Calculate the body to earth frame rotation matrix from the euler angles using a 312 rotation sequence
		float c2 = cosf(euler312(2));
		float s2 = sinf(euler312(2));
		float s1 = sinf(euler312(1));
		float c1 = cosf(euler312(1));
		float s0 = sinf(euler312(0));
		float c0 = cosf(euler312(0));

		matrix::Dcm<float> R_to_earth;
		R_to_earth(0, 0) = c0 * c2 - s0 * s1 * s2;
		R_to_earth(1, 1) = c0 * c1;
		R_to_earth(2, 2) = c2 * c1;
		R_to_earth(0, 1) = -c1 * s0;
		R_to_earth(0, 2) = s2 * c0 + c2 * s1 * s0;
		R_to_earth(1, 0) = c2 * s0 + s2 * s1 * c0;
		R_to_earth(1, 2) = s0 * s2 - s1 * c0 * c2;
		R_to_earth(2, 0) = -s2 * c1;
		R_to_earth(2, 1) = s1;

		// rotate the magnetometer measurements into earth frame using a zero yaw angle
		mag_earth_pred = R_to_earth * _mag_sample_delayed.mag;
	}

	// Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 3 elements in H are non zero
	// calculate the innovaton variance
	float PH[3];
	_heading_innov_var = R_YAW;

	for (unsigned row = 0; row <= 2; row++) {
		PH[row] = 0.0f;

		for (uint8_t col = 0; col <= 2; col++) {
			PH[row] += P[row][col] * H_YAW[col];
		}

		_heading_innov_var += H_YAW[row] * PH[row];
	}

	float heading_innov_var_inv;

	// check if the innovation variance calculation is badly conditioned
	if (_heading_innov_var >= R_YAW) {
		// the innovation variance contribution from the state covariances is not negative, no fault
		_fault_status.bad_mag_hdg = false;
		heading_innov_var_inv = 1.0f / _heading_innov_var;

	} else {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.bad_mag_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		return;
	}

	// calculate the Kalman gains
	// only calculate gains for states we are using
	float Kfusion[_k_num_states] = {};

	for (uint8_t row = 0; row <= 15; row++) {
		Kfusion[row] = 0.0f;

		for (uint8_t col = 0; col <= 2; col++) {
			Kfusion[row] += P[row][col] * H_YAW[col];
		}

		Kfusion[row] *= heading_innov_var_inv;
	}

	if (_control_status.flags.wind) {
		for (uint8_t row = 22; row <= 23; row++) {
			Kfusion[row] = 0.0f;

			for (uint8_t col = 0; col <= 2; col++) {
				Kfusion[row] += P[row][col] * H_YAW[col];
			}

			Kfusion[row] *= heading_innov_var_inv;
		}
	}

	// Use the difference between the horizontal projection of the mag field and declination to give the measured heading
	float measured_hdg = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + _mag_declination;

	// wrap the heading to the interval between +-pi
	measured_hdg = matrix::wrap_pi(measured_hdg);

	// calculate the innovation
	_heading_innov = predicted_hdg - measured_hdg;

	// wrap the innovation to the interval between +-pi
	_heading_innov = matrix::wrap_pi(_heading_innov);

	// innovation test ratio
	_yaw_test_ratio = sq(_heading_innov) / (sq(math::max(_params.heading_innov_gate, 1.0f)) * _heading_innov_var);

	// set the magnetometer unhealthy if the test fails
	if (_yaw_test_ratio > 1.0f) {
		_mag_healthy = false;

		// if we are in air we don't want to fuse the measurement
		// we allow to use it when on the ground because the large innovation could be caused
		// by interference or a large initial gyro bias
		if (_control_status.flags.in_air) {
			return;

		} else {
			// constrain the innovation to the maximum set by the gate
			float gate_limit = sqrtf((sq(math::max(_params.heading_innov_gate, 1.0f)) * _heading_innov_var));
			_heading_innov = math::constrain(_heading_innov, -gate_limit, gate_limit);
		}

	} else {
		_mag_healthy = true;
	}

	// zero the attitude error states and use the kalman gain vector and innovation to update the states
	_state.ang_error.setZero();
	fuse(Kfusion, _heading_innov);

	// correct the nominal quaternion
	Quaternion dq;
	dq.from_axis_angle(_state.ang_error);
	_state.quat_nominal = dq * _state.quat_nominal;
	_state.quat_nominal.normalize();

	// apply covariance correction via P_new = (I -K*H)*P
	// first calculate expression for KHP
	// then calculate P - KHP
	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column <= 2; column++) {
			KH[row][column] = Kfusion[row] * H_YAW[column];
		}
	}

	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column < _k_num_states; column++) {
			float tmp = KH[row][0] * P[0][column];
			tmp += KH[row][1] * P[1][column];
			tmp += KH[row][2] * P[2][column];
			KHP[row][column] = tmp;
		}
	}

	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column < _k_num_states; column++) {
			P[row][column] -= KHP[row][column];
		}
	}

	makeSymmetrical();
	limitCov();
}

void Ekf::fuseDeclination()
{
	// assign intermediate state variables
	float magN = _state.mag_I(0);
	float magE = _state.mag_I(1);

	float R_DECL = sq(0.5f);

	// Calculate intermediate variables
	// if the horizontal magnetic field is too small, this calculation will be badly conditioned
	if (magN < 0.001f) {
		return;
	}

	float t2 = magE * magE;
	float t3 = magN * magN;
	float t4 = t2 + t3;
	float t5 = 1.0f / t4;
	float t22 = magE * t5;
	float t23 = magN * t5;
	float t6 = P[16][16] * t22;
	float t13 = P[17][16] * t23;
	float t7 = t6 - t13;
	float t8 = t22 * t7;
	float t9 = P[16][17] * t22;
	float t14 = P[17][17] * t23;
	float t10 = t9 - t14;
	float t15 = t23 * t10;
	float t11 = R_DECL + t8 - t15; // innovation variance

	// check the innovation variance calculation for a badly conditioned covariance matrix
	if (t11 >= R_DECL) {
		// the innovation variance contribution from the state covariances is not negative, no fault
		_fault_status.bad_mag_decl = false;

	} else {
		// the innovation variance contribution from the state covariances is negtive which means the covariance matrix is badly conditioned
		_fault_status.bad_mag_decl = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		return;
	}

	float t12 = 1.0f / t11;

	// Calculate the observation Jacobian
	// Note only 2 terms are non-zero which can be used in matrix operations for calculation of Kalman gains and covariance update to significantly reduce cost
	float H_DECL[24] = {};
	H_DECL[16] = -magE * t5;
	H_DECL[17] = magN * t5;

	// Calculate the Kalman gains
	float Kfusion[_k_num_states] = {};
	Kfusion[0] = -t12 * (P[0][16] * t22 - P[0][17] * t23);
	Kfusion[1] = -t12 * (P[1][16] * t22 - P[1][17] * t23);
	Kfusion[2] = -t12 * (P[2][16] * t22 - P[2][17] * t23);
	Kfusion[3] = -t12 * (P[3][16] * t22 - P[3][17] * t23);
	Kfusion[4] = -t12 * (P[4][16] * t22 - P[4][17] * t23);
	Kfusion[5] = -t12 * (P[5][16] * t22 - P[5][17] * t23);
	Kfusion[6] = -t12 * (P[6][16] * t22 - P[6][17] * t23);
	Kfusion[7] = -t12 * (P[7][16] * t22 - P[7][17] * t23);
	Kfusion[8] = -t12 * (P[8][16] * t22 - P[8][17] * t23);
	Kfusion[9] = -t12 * (P[9][16] * t22 - P[9][17] * t23);
	Kfusion[10] = -t12 * (P[10][16] * t22 - P[10][17] * t23);
	Kfusion[11] = -t12 * (P[11][16] * t22 - P[11][17] * t23);
	Kfusion[12] = -t12 * (P[12][16] * t22 - P[12][17] * t23);
	Kfusion[13] = -t12 * (P[13][16] * t22 - P[13][17] * t23);
	Kfusion[14] = -t12 * (P[14][16] * t22 - P[14][17] * t23);
	Kfusion[15] = -t12 * (P[15][16] * t22 - P[15][17] * t23);

	// We only do declination fusion when we are using all the field states, so no logic required here
	Kfusion[16] = -t12 * (t6 - P[16][17] * t23);
	Kfusion[17] = t12 * (t14 - P[17][16] * t22);
	Kfusion[18] = -t12 * (P[18][16] * t22 - P[18][17] * t23);
	Kfusion[19] = -t12 * (P[19][16] * t22 - P[19][17] * t23);
	Kfusion[20] = -t12 * (P[20][16] * t22 - P[20][17] * t23);
	Kfusion[21] = -t12 * (P[21][16] * t22 - P[21][17] * t23);

	// Don't update wind states unless we are doing wind estimation
	if (_control_status.flags.wind) {
		Kfusion[22] = -t12 * (P[22][16] * t22 - P[22][17] * t23);
		Kfusion[23] = -t12 * (P[23][16] * t22 - P[23][17] * t23);

	} else {
		Kfusion[22] = 0.0f;
		Kfusion[23] = 0.0f;
	}

	// calculate innovation and constrain
	float innovation = atanf(magE / magN) - _mag_declination;
	innovation = math::constrain(innovation, -0.5f, 0.5f);

	// zero attitude error states and perform the state correction
	_state.ang_error.setZero();
	fuse(Kfusion, innovation);

	// use the attitude error estimate to correct the quaternion
	Quaternion dq;
	dq.from_axis_angle(_state.ang_error);
	_state.quat_nominal = dq * _state.quat_nominal;
	_state.quat_nominal.normalize();

	// apply covariance correction via P_new = (I -K*H)*P
	// first calculate expression for KHP
	// then calculate P - KHP
	// take advantage of the empty columns in KH to reduce the number of operations
	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 16; column <= 17; column++) {
			KH[row][column] = Kfusion[row] * H_DECL[column];
		}
	}

	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column < _k_num_states; column++) {
			float tmp = KH[row][16] * P[16][column];
			tmp += KH[row][17] * P[17][column];
			KHP[row][column] = tmp;
		}
	}

	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column < _k_num_states; column++) {
			P[row][column] -= KHP[row][column];
		}
	}

	// force the covariance matrix to be symmetrical and don't allow the variances to be negative.
	makeSymmetrical();
	limitCov();
}

void Ekf::fuseMag2D()
{
	// assign intermediate state variables
	float q0 = _state.quat_nominal(0);
	float q1 = _state.quat_nominal(1);
	float q2 = _state.quat_nominal(2);
	float q3 = _state.quat_nominal(3);

	float magX = _mag_sample_delayed.mag(0);
	float magY = _mag_sample_delayed.mag(1);
	float magZ = _mag_sample_delayed.mag(2);

	float R_DECL = fmaxf(_params.mag_heading_noise, 1.0e-2f);
	R_DECL = R_DECL * R_DECL;

	// calculate intermediate variables for observation jacobian
	float t2 = q0 * q0;
	float t3 = q1 * q1;
	float t4 = q2 * q2;
	float t5 = q3 * q3;
	float t6 = q0 * q3 * 2.0f;
	float t8 = t2 - t3 + t4 - t5;
	float t9 = q0 * q1 * 2.0f;
	float t10 = q2 * q3 * 2.0f;
	float t11 = t9 - t10;
	float t14 = q1 * q2 * 2.0f;
	float t21 = magY * t8;
	float t22 = t6 + t14;
	float t23 = magX * t22;
	float t24 = magZ * t11;
	float t7 = t21 + t23 - t24;
	float t12 = t2 + t3 - t4 - t5;
	float t13 = magX * t12;
	float t15 = q0 * q2 * 2.0f;
	float t16 = q1 * q3 * 2.0f;
	float t17 = t15 + t16;
	float t18 = magZ * t17;
	float t19 = t6 - t14;
	float t25 = magY * t19;
	float t20 = t13 + t18 - t25;

	if (fabsf(t20) < 1e-6f) {
		return;
	}

	float t26 = 1.0f / (t20 * t20);
	float t27 = t7 * t7;
	float t28 = t26 * t27;
	float t29 = t28 + 1.0f;

	if (fabsf(t29) < 1e-12f) {
		return;
	}

	float t30 = 1.0f / t29;

	if (fabsf(t20) < 1e-12f) {
		return;
	}

	float t31 = 1.0f / t20;

	// calculate observation jacobian
	float H_DECL[3] = {};
	H_DECL[0] = -t30 * (t31 * (magZ * t8 + magY * t11) + t7 * t26 * (magY * t17 + magZ * t19));
	H_DECL[1] = t30 * (t31 * (magX * t11 + magZ * t22) - t7 * t26 * (magZ * t12 - magX * t17));
	H_DECL[2] = t30 * (t31 * (magX * t8 - magY * t22) + t7 * t26 * (magY * t12 + magX * t19));

	// rotate the magnetometer measurement into earth frame
	matrix::Dcm<float> R_to_earth(_state.quat_nominal);
	matrix::Vector3f mag_earth_pred = R_to_earth * _mag_sample_delayed.mag;

	// check if there is enough magnetic field length to use and exit if too small
	float magLength2 = sq(mag_earth_pred(0) + mag_earth_pred(1));

	if (magLength2 < sq(_params.mag_noise)) {
		return;
	}

	// Adjust the measurement variance upwards if thehorizontal strength to magnetometer noise ratio make the value unrealistic
	R_DECL = fmaxf(R_DECL, sq(_params.mag_noise) / magLength2);

	// Calculate the innovation, using the declination angle of the projection onto the horizontal as the measurement
	_heading_innov = atan2f(mag_earth_pred(1), mag_earth_pred(0)) - _mag_declination;

	// wrap the innovation to the interval between +-pi
	_heading_innov = matrix::wrap_pi(_heading_innov);

	// Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 3 elements in H are non zero
	float PH[3];
	_heading_innov_var = R_DECL;

	for (unsigned row = 0; row <= 2; row++) {
		PH[row] = 0.0f;

		for (unsigned col = 0; col <= 2; col++) {
			PH[row] += P[row][col] * H_DECL[col];
		}

		_heading_innov_var += H_DECL[row] * PH[row];
	}

	float varInnovInv;

	if (_heading_innov_var >= R_DECL) {
		// the innovation variance contribution from the state covariances is not negative, no fault
		_fault_status.bad_mag_hdg = false;

	} else {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.bad_mag_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		return;
	}

	// innovation test ratio
	_yaw_test_ratio = sq(_heading_innov) / (sq(math::max(_params.heading_innov_gate, 1.0f)) * _heading_innov_var);

	// set the magnetometer unhealthy if the test fails
	if (_yaw_test_ratio > 1.0f) {
		_mag_healthy = false;

		// if we are in air we don't want to fuse the measurement
		// we allow to use it when on the ground because the large innovation could be caused
		// by interference or a large initial gyro bias
		if (_control_status.flags.in_air) {
			return;

		} else {
			// constrain the innovation to the maximum set by the gate
			float gate_limit = sqrtf((sq(math::max(_params.heading_innov_gate, 1.0f)) * _heading_innov_var));
			_heading_innov = math::constrain(_heading_innov, -gate_limit, gate_limit);
		}

	} else {
		_mag_healthy = true;
	}

	varInnovInv = 1.0f / _heading_innov_var;

	// calculate the Kalman gains
	float Kfusion[24] = {};

	for (unsigned row = 0; row < 16; row++) {
		Kfusion[row] = 0.0f;

		for (unsigned col = 0; col <= 2; col++) {
			Kfusion[row] += P[row][col] * H_DECL[col];
		}

		Kfusion[row] *= varInnovInv;
	}

	// by definition our error state is zero at the time of fusion
	_state.ang_error.setZero();

	// correct the states
	fuse(Kfusion, _heading_innov);

	// correct the quaternon using the attitude error estimate
	Quaternion q_correction;
	q_correction.from_axis_angle(_state.ang_error);
	_state.quat_nominal = q_correction * _state.quat_nominal;
	_state.quat_nominal.normalize();
	_state.ang_error.setZero();

	// apply covariance correction via P_new = (I -K*H)*P
	// first calculate expression for KHP
	// then calculate P - KHP
	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column <= 2; column++) {
			KH[row][column] = Kfusion[row] * H_DECL[column];
		}
	}

	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column < _k_num_states; column++) {
			float tmp = KH[row][0] * P[0][column];
			tmp += KH[row][1] * P[1][column];
			tmp += KH[row][2] * P[2][column];
			KHP[row][column] = tmp;
		}
	}

	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column < _k_num_states; column++) {
			P[row][column] -= KHP[row][column];
		}
	}

	makeSymmetrical();
	limitCov();
}
