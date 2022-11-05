/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "test_helper/comparison_helper.h"

#include "../EKF/python/ekf_derivation/generated/compute_mag_innov_innov_var_and_hx.h"
#include "../EKF/python/ekf_derivation/generated/compute_mag_y_innov_var_and_h.h"
#include "../EKF/python/ekf_derivation/generated/compute_mag_z_innov_var_and_h.h"

using namespace matrix;

void sympyMagXInnovVarHxAndKx(float q0, float q1, float q2, float q3, float magN, float magE, float magD,
			      const SquareMatrix24f &P, float R_MAG, float &innov_var, Vector24f &Kfusion, Vector24f &H)
{
	// common expressions used by sympy generated equations
	// calculate intermediate variables used for X axis innovation variance, observation Jacobians and Kalman gainss
	const float HKX0 = -magD * q2 + magE * q3 + magN * q0;
	const float HKX1 = magD * q3 + magE * q2 + magN * q1;
	const float HKX2 = magE * q1;
	const float HKX3 = magD * q0;
	const float HKX4 = magN * q2;
	const float HKX5 = magD * q1 + magE * q0 - magN * q3;
	const float HKX6 = powf(q0, 2) + powf(q1, 2) - powf(q2, 2) - powf(q3, 2);
	const float HKX7 = q0 * q3 + q1 * q2;
	const float HKX8 = q1 * q3;
	const float HKX9 = q0 * q2;
	const float HKX10 = 2 * HKX7;
	const float HKX11 = -2 * HKX8 + 2 * HKX9;
	const float HKX12 = 2 * HKX1;
	const float HKX13 = 2 * HKX0;
	const float HKX14 = -2 * HKX2 + 2 * HKX3 + 2 * HKX4;
	const float HKX15 = 2 * HKX5;
	const float HKX16 = HKX10 * P(0, 17) - HKX11 * P(0, 18) + HKX12 * P(0, 1) + HKX13 * P(0, 0) - HKX14 * P(0,
			    2) + HKX15 * P(0, 3) + HKX6 * P(0, 16) + P(0, 19);
	const float HKX17 = HKX10 * P(16, 17) - HKX11 * P(16, 18) + HKX12 * P(1, 16) + HKX13 * P(0, 16) - HKX14 * P(2,
			    16) + HKX15 * P(3, 16) + HKX6 * P(16, 16) + P(16, 19);
	const float HKX18 = HKX10 * P(17, 18) - HKX11 * P(18, 18) + HKX12 * P(1, 18) + HKX13 * P(0, 18) - HKX14 * P(2,
			    18) + HKX15 * P(3, 18) + HKX6 * P(16, 18) + P(18, 19);
	const float HKX19 = HKX10 * P(2, 17) - HKX11 * P(2, 18) + HKX12 * P(1, 2) + HKX13 * P(0, 2) - HKX14 * P(2,
			    2) + HKX15 * P(2, 3) + HKX6 * P(2, 16) + P(2, 19);
	const float HKX20 = HKX10 * P(17, 17) - HKX11 * P(17, 18) + HKX12 * P(1, 17) + HKX13 * P(0, 17) - HKX14 * P(2,
			    17) + HKX15 * P(3, 17) + HKX6 * P(16, 17) + P(17, 19);
	const float HKX21 = HKX10 * P(3, 17) - HKX11 * P(3, 18) + HKX12 * P(1, 3) + HKX13 * P(0, 3) - HKX14 * P(2,
			    3) + HKX15 * P(3, 3) + HKX6 * P(3, 16) + P(3, 19);
	const float HKX22 = HKX10 * P(1, 17) - HKX11 * P(1, 18) + HKX12 * P(1, 1) + HKX13 * P(0, 1) - HKX14 * P(1,
			    2) + HKX15 * P(1, 3) + HKX6 * P(1, 16) + P(1, 19);
	const float HKX23 = HKX10 * P(17, 19) - HKX11 * P(18, 19) + HKX12 * P(1, 19) + HKX13 * P(0, 19) - HKX14 * P(2,
			    19) + HKX15 * P(3, 19) + HKX6 * P(16, 19) + P(19, 19);
	const float HKX24 = 1.0F / (HKX10 * HKX20 - HKX11 * HKX18 + HKX12 * HKX22 + HKX13 * HKX16 - HKX14 * HKX19 + HKX15 *
				    HKX21 + HKX17 * HKX6 + HKX23 + R_MAG);

	innov_var = (HKX10 * HKX20 - HKX11 * HKX18 + HKX12 * HKX22 + HKX13 * HKX16 - HKX14 * HKX19 + HKX15 * HKX21 + HKX17 *
		     HKX6 + HKX23 + R_MAG);

	// Calculate X axis observation jacobians
	float Hfusion[24] = {};
	Hfusion[0] = 2 * HKX0;
	Hfusion[1] = 2 * HKX1;
	Hfusion[2] = 2 * HKX2 - 2 * HKX3 - 2 * HKX4;
	Hfusion[3] = 2 * HKX5;
	Hfusion[16] = HKX6;
	Hfusion[17] = 2 * HKX7;
	Hfusion[18] = 2 * HKX8 - 2 * HKX9;
	Hfusion[19] = 1;

	// Calculate X axis Kalman gains
	if (true) {
		Kfusion(0) = HKX16 * HKX24;
		Kfusion(1) = HKX22 * HKX24;
		Kfusion(2) = HKX19 * HKX24;
		Kfusion(3) = HKX21 * HKX24;

		for (unsigned row = 4; row <= 15; row++) {
			Kfusion(row) = HKX24 * (HKX10 * P(row, 17) - HKX11 * P(row, 18) + HKX12 * P(1, row) + HKX13 * P(0, row) - HKX14 * P(2,
						row) + HKX15 * P(3, row) + HKX6 * P(row, 16) + P(row, 19));
		}

		for (unsigned row = 22; row <= 23; row++) {
			Kfusion(row) = HKX24 * (HKX10 * P(17, row) - HKX11 * P(18, row) + HKX12 * P(1, row) + HKX13 * P(0, row) - HKX14 * P(2,
						row) + HKX15 * P(3, row) + HKX6 * P(16, row) + P(19, row));
		}
	}

	Kfusion(16) = HKX17 * HKX24;
	Kfusion(17) = HKX20 * HKX24;
	Kfusion(18) = HKX18 * HKX24;
	Kfusion(19) = HKX23 * HKX24;

	for (unsigned row = 20; row <= 21; row++) {
		Kfusion(row) = HKX24 * (HKX10 * P(17, row) - HKX11 * P(18, row) + HKX12 * P(1, row) + HKX13 * P(0, row) - HKX14 * P(2,
					row) + HKX15 * P(3, row) + HKX6 * P(16, row) + P(19, row));
	}

	for (int row = 0; row < 24; row++) {
		H(row) = Hfusion[row];
	}
}

void sympyMagYInnovVarHyAndKy(float q0, float q1, float q2, float q3, float magN, float magE, float magD,
			      const SquareMatrix24f &P, float R_MAG, float &innov_var, Vector24f &Kfusion, Vector24f &H)
{
	const float HKY0 = magD * q1 + magE * q0 - magN * q3;
	const float HKY1 = magD * q0 - magE * q1 + magN * q2;
	const float HKY2 = magD * q3 + magE * q2 + magN * q1;
	const float HKY3 = magD * q2;
	const float HKY4 = magE * q3;
	const float HKY5 = magN * q0;
	const float HKY6 = q1 * q2;
	const float HKY7 = q0 * q3;
	const float HKY8 = powf(q0, 2) - powf(q1, 2) + powf(q2, 2) - powf(q3, 2);
	const float HKY9 = q0 * q1 + q2 * q3;
	const float HKY10 = 2 * HKY9;
	const float HKY11 = -2 * HKY6 + 2 * HKY7;
	const float HKY12 = 2 * HKY2;
	const float HKY13 = 2 * HKY0;
	const float HKY14 = 2 * HKY1;
	const float HKY15 = -2 * HKY3 + 2 * HKY4 + 2 * HKY5;
	const float HKY16 = HKY10 * P(0, 18) - HKY11 * P(0, 16) + HKY12 * P(0, 2) + HKY13 * P(0, 0) + HKY14 * P(0,
			    1) - HKY15 * P(0, 3) + HKY8 * P(0, 17) + P(0, 20);
	const float HKY17 = HKY10 * P(17, 18) - HKY11 * P(16, 17) + HKY12 * P(2, 17) + HKY13 * P(0, 17) + HKY14 * P(1,
			    17) - HKY15 * P(3, 17) + HKY8 * P(17, 17) + P(17, 20);
	const float HKY18 = HKY10 * P(16, 18) - HKY11 * P(16, 16) + HKY12 * P(2, 16) + HKY13 * P(0, 16) + HKY14 * P(1,
			    16) - HKY15 * P(3, 16) + HKY8 * P(16, 17) + P(16, 20);
	const float HKY19 = HKY10 * P(3, 18) - HKY11 * P(3, 16) + HKY12 * P(2, 3) + HKY13 * P(0, 3) + HKY14 * P(1,
			    3) - HKY15 * P(3, 3) + HKY8 * P(3, 17) + P(3, 20);
	const float HKY20 = HKY10 * P(18, 18) - HKY11 * P(16, 18) + HKY12 * P(2, 18) + HKY13 * P(0, 18) + HKY14 * P(1,
			    18) - HKY15 * P(3, 18) + HKY8 * P(17, 18) + P(18, 20);
	const float HKY21 = HKY10 * P(1, 18) - HKY11 * P(1, 16) + HKY12 * P(1, 2) + HKY13 * P(0, 1) + HKY14 * P(1,
			    1) - HKY15 * P(1, 3) + HKY8 * P(1, 17) + P(1, 20);
	const float HKY22 = HKY10 * P(2, 18) - HKY11 * P(2, 16) + HKY12 * P(2, 2) + HKY13 * P(0, 2) + HKY14 * P(1,
			    2) - HKY15 * P(2, 3) + HKY8 * P(2, 17) + P(2, 20);
	const float HKY23 = HKY10 * P(18, 20) - HKY11 * P(16, 20) + HKY12 * P(2, 20) + HKY13 * P(0, 20) + HKY14 * P(1,
			    20) - HKY15 * P(3, 20) + HKY8 * P(17, 20) + P(20, 20);
	innov_var = (HKY10 * HKY20 - HKY11 * HKY18 + HKY12 * HKY22 + HKY13 * HKY16 + HKY14 * HKY21 - HKY15 * HKY19 + HKY17 *
		     HKY8 + HKY23 + R_MAG);
	const float HKY24 = 1.0F / innov_var;

	// Calculate Y axis observation jacobians
	float Hfusion[24] = {};
	Hfusion[0] = 2 * HKY0;
	Hfusion[1] = 2 * HKY1;
	Hfusion[2] = 2 * HKY2;
	Hfusion[3] = 2 * HKY3 - 2 * HKY4 - 2 * HKY5;
	Hfusion[16] = 2 * HKY6 - 2 * HKY7;
	Hfusion[17] = HKY8;
	Hfusion[18] = 2 * HKY9;
	Hfusion[20] = 1;

	// Calculate Y axis Kalman gains
	if (true) {
		Kfusion(0) = HKY16 * HKY24;
		Kfusion(1) = HKY21 * HKY24;
		Kfusion(2) = HKY22 * HKY24;
		Kfusion(3) = HKY19 * HKY24;

		for (unsigned row = 4; row <= 15; row++) {
			Kfusion(row) = HKY24 * (HKY10 * P(row, 18) - HKY11 * P(row, 16) + HKY12 * P(2, row) + HKY13 * P(0, row) + HKY14 * P(1,
						row) - HKY15 * P(3, row) + HKY8 * P(row, 17) + P(row, 20));
		}

		for (unsigned row = 22; row <= 23; row++) {
			Kfusion(row) = HKY24 * (HKY10 * P(18, row) - HKY11 * P(16, row) + HKY12 * P(2, row) + HKY13 * P(0, row) + HKY14 * P(1,
						row) - HKY15 * P(3, row) + HKY8 * P(17, row) + P(20, row));
		}
	}

	Kfusion(16) = HKY18 * HKY24;
	Kfusion(17) = HKY17 * HKY24;
	Kfusion(18) = HKY20 * HKY24;
	Kfusion(19) = HKY24 * (HKY10 * P(18, 19) - HKY11 * P(16, 19) + HKY12 * P(2, 19) + HKY13 * P(0, 19) + HKY14 * P(1,
			       19) - HKY15 * P(3, 19) + HKY8 * P(17, 19) + P(19, 20));
	Kfusion(20) = HKY23 * HKY24;
	Kfusion(21) = HKY24 * (HKY10 * P(18, 21) - HKY11 * P(16, 21) + HKY12 * P(2, 21) + HKY13 * P(0, 21) + HKY14 * P(1,
			       21) - HKY15 * P(3, 21) + HKY8 * P(17, 21) + P(20, 21));

	// save output and repeat calculation using legacy matlab generated code
	for (int row = 0; row < 24; row++) {
		H(row) = Hfusion[row];
	}
}

void sympyMagZInnovVarHzAndKz(float q0, float q1, float q2, float q3, float magN, float magE, float magD,
			      const SquareMatrix24f &P, float R_MAG, float &innov_var, Vector24f &Kfusion, Vector24f &H)
{
	const float HKZ0 = magD * q0 - magE * q1 + magN * q2;
	const float HKZ1 = magN * q3;
	const float HKZ2 = magD * q1;
	const float HKZ3 = magE * q0;
	const float HKZ4 = -magD * q2 + magE * q3 + magN * q0;
	const float HKZ5 = magD * q3 + magE * q2 + magN * q1;
	const float HKZ6 = q0 * q2 + q1 * q3;
	const float HKZ7 = q2 * q3;
	const float HKZ8 = q0 * q1;
	const float HKZ9 = powf(q0, 2) - powf(q1, 2) - powf(q2, 2) + powf(q3, 2);
	const float HKZ10 = 2 * HKZ6;
	const float HKZ11 = -2 * HKZ7 + 2 * HKZ8;
	const float HKZ12 = 2 * HKZ5;
	const float HKZ13 = 2 * HKZ0;
	const float HKZ14 = -2 * HKZ1 + 2 * HKZ2 + 2 * HKZ3;
	const float HKZ15 = 2 * HKZ4;
	const float HKZ16 = HKZ10 * P(0, 16) - HKZ11 * P(0, 17) + HKZ12 * P(0, 3) + HKZ13 * P(0, 0) - HKZ14 * P(0,
			    1) + HKZ15 * P(0, 2) + HKZ9 * P(0, 18) + P(0, 21);
	const float HKZ17 = HKZ10 * P(16, 18) - HKZ11 * P(17, 18) + HKZ12 * P(3, 18) + HKZ13 * P(0, 18) - HKZ14 * P(1,
			    18) + HKZ15 * P(2, 18) + HKZ9 * P(18, 18) + P(18, 21);
	const float HKZ18 = HKZ10 * P(16, 17) - HKZ11 * P(17, 17) + HKZ12 * P(3, 17) + HKZ13 * P(0, 17) - HKZ14 * P(1,
			    17) + HKZ15 * P(2, 17) + HKZ9 * P(17, 18) + P(17, 21);
	const float HKZ19 = HKZ10 * P(1, 16) - HKZ11 * P(1, 17) + HKZ12 * P(1, 3) + HKZ13 * P(0, 1) - HKZ14 * P(1,
			    1) + HKZ15 * P(1, 2) + HKZ9 * P(1, 18) + P(1, 21);
	const float HKZ20 = HKZ10 * P(16, 16) - HKZ11 * P(16, 17) + HKZ12 * P(3, 16) + HKZ13 * P(0, 16) - HKZ14 * P(1,
			    16) + HKZ15 * P(2, 16) + HKZ9 * P(16, 18) + P(16, 21);
	const float HKZ21 = HKZ10 * P(3, 16) - HKZ11 * P(3, 17) + HKZ12 * P(3, 3) + HKZ13 * P(0, 3) - HKZ14 * P(1,
			    3) + HKZ15 * P(2, 3) + HKZ9 * P(3, 18) + P(3, 21);
	const float HKZ22 = HKZ10 * P(2, 16) - HKZ11 * P(2, 17) + HKZ12 * P(2, 3) + HKZ13 * P(0, 2) - HKZ14 * P(1,
			    2) + HKZ15 * P(2, 2) + HKZ9 * P(2, 18) + P(2, 21);
	const float HKZ23 = HKZ10 * P(16, 21) - HKZ11 * P(17, 21) + HKZ12 * P(3, 21) + HKZ13 * P(0, 21) - HKZ14 * P(1,
			    21) + HKZ15 * P(2, 21) + HKZ9 * P(18, 21) + P(21, 21);
	innov_var = (HKZ10 * HKZ20 - HKZ11 * HKZ18 + HKZ12 * HKZ21 + HKZ13 * HKZ16 - HKZ14 * HKZ19 + HKZ15 * HKZ22 + HKZ17 *
		     HKZ9 + HKZ23 + R_MAG);
	const float HKZ24 = 1.0F / innov_var;

	// calculate Z axis observation jacobians
	float Hfusion[24] = {};
	Hfusion[0] = 2 * HKZ0;
	Hfusion[1] = 2 * HKZ1 - 2 * HKZ2 - 2 * HKZ3;
	Hfusion[2] = 2 * HKZ4;
	Hfusion[3] = 2 * HKZ5;
	Hfusion[16] = 2 * HKZ6;
	Hfusion[17] = 2 * HKZ7 - 2 * HKZ8;
	Hfusion[18] = HKZ9;
	Hfusion[21] = 1;

	// Calculate Z axis Kalman gains
	if (true) {
		Kfusion(0) = HKZ16 * HKZ24;
		Kfusion(1) = HKZ19 * HKZ24;
		Kfusion(2) = HKZ22 * HKZ24;
		Kfusion(3) = HKZ21 * HKZ24;

		for (unsigned row = 4; row <= 15; row++) {
			Kfusion(row) = HKZ24 * (HKZ10 * P(row, 16) - HKZ11 * P(row, 17) + HKZ12 * P(3, row) + HKZ13 * P(0, row) - HKZ14 * P(1,
						row) + HKZ15 * P(2, row) + HKZ9 * P(row, 18) + P(row, 21));
		}

		for (unsigned row = 22; row <= 23; row++) {
			Kfusion(row) = HKZ24 * (HKZ10 * P(16, row) - HKZ11 * P(17, row) + HKZ12 * P(3, row) + HKZ13 * P(0, row) - HKZ14 * P(1,
						row) + HKZ15 * P(2, row) + HKZ9 * P(18, row) + P(21, row));
		}
	}

	Kfusion(16) = HKZ20 * HKZ24;
	Kfusion(17) = HKZ18 * HKZ24;
	Kfusion(18) = HKZ17 * HKZ24;

	for (unsigned row = 19; row <= 20; row++) {
		Kfusion(row) = HKZ24 * (HKZ10 * P(16, row) - HKZ11 * P(17, row) + HKZ12 * P(3, row) + HKZ13 * P(0, row) - HKZ14 * P(1,
					row) + HKZ15 * P(2, row) + HKZ9 * P(18, row) + P(row, 21));
	}

	Kfusion(21) = HKZ23 * HKZ24;

	// save output and repeat calculation using legacy matlab generated code
	for (int row = 0; row < 24; row++) {
		H(row) = Hfusion[row];
	}
}

TEST(Mag3DFusionGenerated, SympyVsSymforce)
{
	// Compare calculation of observation Jacobians and Kalman gains for sympy and symforce generated equations
	const Quatf q(Eulerf(-M_PI_F / 2.f, M_PI_F / 3.f, M_PI_F * 4.f / 5.f));
	const float q0 = q(0);
	const float q1 = q(1);
	const float q2 = q(2);
	const float q3 = q(3);

	const float magN = 2.0f * ((float)randf() - 0.5f);
	const float magE = 2.0f * ((float)randf() - 0.5f);
	const float magD = 2.0f * ((float)randf() - 0.5f);

	Vector24f state_vector{};
	state_vector(0) = q0;
	state_vector(1) = q1;
	state_vector(2) = q2;
	state_vector(3) = q3;
	state_vector(16) = magN;
	state_vector(17) = magE;
	state_vector(18) = magD;

	const float R_MAG = sq(0.05f);

	SquareMatrix24f P = createRandomCovarianceMatrix24f();

	Vector24f Hfusion_sympy;
	Vector24f Kfusion_sympy;
	float mag_innov_var_sympy;

	Vector24f Hfusion_symforce;
	Vector24f Kfusion_symforce;
	Vector3f mag_innov_var_symforce;

	for (int i = 0; i < 3; i++) {
		if (i == 0) {
			sympyMagXInnovVarHxAndKx(q0, q1, q2, q3, magN, magE, magD, P, R_MAG, mag_innov_var_sympy, Kfusion_sympy, Hfusion_sympy);

			Vector3f innov;
			sym::ComputeMagInnovInnovVarAndHx(state_vector, P, Vector3f(), R_MAG, FLT_EPSILON, &innov, &mag_innov_var_symforce,
							  &Hfusion_symforce);

		} else if (i == 1) {
			sympyMagYInnovVarHyAndKy(q0, q1, q2, q3, magN, magE, magD, P, R_MAG, mag_innov_var_sympy, Kfusion_sympy, Hfusion_sympy);

			sym::ComputeMagYInnovVarAndH(state_vector, P, R_MAG, FLT_EPSILON, &(mag_innov_var_symforce(i)), &Hfusion_symforce);

		} else {
			sympyMagZInnovVarHzAndKz(q0, q1, q2, q3, magN, magE, magD, P, R_MAG, mag_innov_var_sympy, Kfusion_sympy, Hfusion_sympy);

			sym::ComputeMagZInnovVarAndH(state_vector, P, R_MAG, FLT_EPSILON, &(mag_innov_var_symforce(i)), &Hfusion_symforce);
		}

		// K isn't generated from symbolic anymore to save flash space
		Kfusion_symforce = P * Hfusion_symforce / mag_innov_var_symforce(i);

		DiffRatioReport report = computeDiffRatioVector24f(Hfusion_sympy, Hfusion_symforce);
		EXPECT_LT(report.max_diff_fraction, 1e-5f) << "i = " << i << "Airspeed Hfusion max diff fraction = " <<
				report.max_diff_fraction <<
				" location index = " << report.max_row << " sympy = " << report.max_v1 << " symforce = " << report.max_v2;

		report = computeDiffRatioVector24f(Kfusion_sympy, Kfusion_symforce);
		EXPECT_LT(report.max_diff_fraction, 1e-5f) << "i = " << i << "Airspeed Kfusion max diff fraction = " <<
				report.max_diff_fraction <<
				" location index = " << report.max_row << " sympy = " << report.max_v1 << " symforce = " << report.max_v2;
		EXPECT_NEAR(mag_innov_var_sympy, mag_innov_var_symforce(i), 1e-5f) << "i = " << i;
	}
}
