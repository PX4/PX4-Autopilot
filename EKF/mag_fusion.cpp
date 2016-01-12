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
 *
 */
#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::fuseMag(uint8_t index)
{
	// assign intermediate variables
	float q0 = _state.quat_nominal(0);
	float q1 = _state.quat_nominal(1);
	float q2 = _state.quat_nominal(2);
	float q3 = _state.quat_nominal(3);

	float magN = _state.mag_I(0);
	float magE = _state.mag_I(1);
	float magD = _state.mag_I(2);

	// XXX Measurement uncertainty. Need to consider timing errors for fast rotations
	float R_MAG = 1e-3f;

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

	// XXX Do mag checks here!


	float H_MAG[24] = {};
	float Kfusion[24] = {};

	// index corresponds to a mag axis (x, y, z)
	if (index == 0) {
		H_MAG[1] = SH_MAG[6] - magD * SH_MAG[2] - magN * SH_MAG[5];
		H_MAG[2] = magE * SH_MAG[0] + magD * SH_MAG[3] - magN * (SH_MAG[8] - 2 * q1 * q2);
		H_MAG[16] = SH_MAG[1];
		H_MAG[17] = SH_MAG[4];
		H_MAG[18] = SH_MAG[7];
		H_MAG[19] = 1;

		// intermediate variables
		float SK_MX[4] = {};
		SK_MX[0] = 1 / (P[19][19] + R_MAG - P[1][19] * (magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5]) + P[16][19] * SH_MAG[1]
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
		SK_MX[1] = magE * SH_MAG[0] + magD * SH_MAG[3] - magN * (SH_MAG[8] - 2 * q1 * q2);
		SK_MX[2] = magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5];
		SK_MX[3] = SH_MAG[7];

		_mag_innov_var[0] = 1 / SK_MX[0];

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
		Kfusion[22] = SK_MX[0] * (P[22][19] + P[22][16] * SH_MAG[1] + P[22][17] * SH_MAG[4] - P[22][1] * SK_MX[2] + P[22][2] *
					  SK_MX[1] + P[22][18] * SK_MX[3]);
		Kfusion[23] = SK_MX[0] * (P[23][19] + P[23][16] * SH_MAG[1] + P[23][17] * SH_MAG[4] - P[23][1] * SK_MX[2] + P[23][2] *
					  SK_MX[1] + P[23][18] * SK_MX[3]);

	} else if (index == 1) {

		H_MAG[0] = magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5];
		H_MAG[2] = - magE * SH_MAG[4] - magD * SH_MAG[7] - magN * SH_MAG[1];
		H_MAG[16] = 2 * q1 * q2 - SH_MAG[8];
		H_MAG[17] = SH_MAG[0];
		H_MAG[18] = SH_MAG[3];
		H_MAG[20] = 1;

		// intermediate variables - note SK_MY[0] is 1/(innovation variance)
		float SK_MY[4];
		SK_MY[0] = 1 / (P[20][20] + R_MAG + P[0][20] * (magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5]) + P[17][20] * SH_MAG[0]
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
		SK_MY[1] = magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1];
		SK_MY[2] = magD * SH_MAG[2] - SH_MAG[6] + magN * SH_MAG[5];
		SK_MY[3] = SH_MAG[8] - 2 * q1 * q2;

		_mag_innov_var[1] = 1 / SK_MY[0];

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
		Kfusion[22] = SK_MY[0] * (P[22][20] + P[22][17] * SH_MAG[0] + P[22][18] * SH_MAG[3] + P[22][0] * SK_MY[2] - P[22][2] *
					  SK_MY[1] - P[22][16] * SK_MY[3]);
		Kfusion[23] = SK_MY[0] * (P[23][20] + P[23][17] * SH_MAG[0] + P[23][18] * SH_MAG[3] + P[23][0] * SK_MY[2] - P[23][2] *
					  SK_MY[1] - P[23][16] * SK_MY[3]);


	} else if (index == 2)

	{
		H_MAG[0] = magN * (SH_MAG[8] - 2 * q1 * q2) - magD * SH_MAG[3] - magE * SH_MAG[0];
		H_MAG[1] = magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1];
		H_MAG[16] = SH_MAG[5];
		H_MAG[17] = 2 * q2 * q3 - 2 * q0 * q1;
		H_MAG[18] = SH_MAG[2];
		H_MAG[21] = 1;

		// intermediate variables
		float SK_MZ[4];
		SK_MZ[0] = 1 / (P[21][21] + R_MAG + P[16][21] * SH_MAG[5] + P[18][21] * SH_MAG[2] - (2 * q0 * q1 - 2 * q2 * q3) *
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
		SK_MZ[1] = magE * SH_MAG[0] + magD * SH_MAG[3] - magN * (SH_MAG[8] - 2 * q1 * q2);
		SK_MZ[2] = magE * SH_MAG[4] + magD * SH_MAG[7] + magN * SH_MAG[1];
		SK_MZ[3] = 2 * q0 * q1 - 2 * q2 * q3;

		_mag_innov_var[2] = 1 / SK_MZ[0];

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
		Kfusion[22] = SK_MZ[0] * (P[22][21] + P[22][18] * SH_MAG[2] + P[22][16] * SH_MAG[5] - P[22][0] * SK_MZ[1] + P[22][1] *
					  SK_MZ[2] - P[22][17] * SK_MZ[3]);
		Kfusion[23] = SK_MZ[0] * (P[23][21] + P[23][18] * SH_MAG[2] + P[23][16] * SH_MAG[5] - P[23][0] * SK_MZ[1] + P[23][1] *
					  SK_MZ[2] - P[23][17] * SK_MZ[3]);

	} else {
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
	float KH[_k_num_states][_k_num_states] = {};

	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column < 3; column++) {
			KH[row][column] = Kfusion[row] * H_MAG[column];
		}

		for (unsigned column = 16; column < 22; column++) {
			KH[row][column] = Kfusion[row] * H_MAG[column];
		}

	}

	float KHP[_k_num_states][_k_num_states] = {};

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

	float t2 = q0 * q0;
	float t3 = q1 * q1;
	float t4 = q2 * q2;
	float t5 = q3 * q3;
	float t6 = q0 * q2 * 2.0f;
	float t7 = q1 * q3 * 2.0f;
	float t8 = t6 + t7;
	float t9 = q0 * q3 * 2.0f;
	float t13 = q1 * q2 * 2.0f;
	float t10 = t9 - t13;
	float t11 = t2 + t3 - t4 - t5;
	float t12 = magX * t11;
	float t14 = magZ * t8;
	float t19 = magY * t10;
	float t15 = t12 + t14 - t19;
	float t16 = t2 - t3 + t4 - t5;
	float t17 = q0 * q1 * 2.0f;
	float t24 = q2 * q3 * 2.0f;
	float t18 = t17 - t24;
	float t20 = 1.0f / t15;
	float t21 = magY * t16;
	float t22 = t9 + t13;
	float t23 = magX * t22;
	float t28 = magZ * t18;
	float t25 = t21 + t23 - t28;
	float t29 = t20 * t25;
	float t26 = tan(t29);
	float t27 = 1.0f / (t15 * t15);
	float t30 = t26 * t26;
	float t31 = t30 + 1.0f;

	float H_MAG[3] = {};
	H_MAG[0] = -t31 * (t20 * (magZ * t16 + magY * t18) + t25 * t27 * (magY * t8 + magZ * t10));
	H_MAG[1] = t31 * (t20 * (magX * t18 + magZ * t22) + t25 * t27 * (magX * t8 - magZ * t11));
	H_MAG[2] = t31 * (t20 * (magX * t16 - magY * t22) + t25 * t27 * (magX * t10 + magY * t11));

	// calculate innovation
	matrix::Dcm<float> R_to_earth(_state.quat_nominal);
	matrix::Vector3f mag_earth_pred = R_to_earth * _mag_sample_delayed.mag;

	float innovation = atan2f(mag_earth_pred(1), mag_earth_pred(0)) - math::radians(_params.mag_declination_deg);

	innovation = math::constrain(innovation, -0.5f, 0.5f);
	_heading_innov = innovation;

	float innovation_var = R_mag;
	_heading_innov_var = innovation_var;

	// calculate innovation variance
	float PH[3] = {};

	for (unsigned row = 0; row < 3; row++) {
		for (unsigned column = 0; column < 3; column++) {
			PH[row] += P[row][column] * H_MAG[column];
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
	float yaw_test_ratio = sq(innovation) / (sq(math::max(0.01f * (float)_params.heading_innov_gate,
			       1.0f)) * innovation_var);

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