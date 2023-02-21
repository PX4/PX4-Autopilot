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

#include "../EKF/python/ekf_derivation/generated/yaw_est_compute_measurement_update.h"
#include "../EKF/python/ekf_derivation/generated/yaw_est_predict_covariance.h"

using namespace matrix;
typedef SquareMatrix<float, 3> SquareMatrix3f;

SquareMatrix3f createRandomCovarianceMatrix3f()
{
	// Create a symmetric square matrix
	SquareMatrix3f P;

	for (int col = 0; col <= 2; col++) {
		for (int row = 0; row <= col; row++) {
			if (row == col) {
				P(row, col) = randf();

			} else {
				P(col, row) = P(row, col) = 2.0f * (randf() - 0.5f);
			}
		}
	}

	// Make it positive definite
	P = P.transpose() * P;

	return P;
}

void sympyYawEstUpdate(const SquareMatrix3f &P, float velObsVar, SquareMatrix<float, 2> &S_inverse,
		       float &S_det_inverse, Matrix<float, 3, 2> &K, SquareMatrix3f &P_new)
{
	const float P00 = P(0, 0);
	const float P01 = P(0, 1);
	const float P02 = P(0, 2);
	const float P11 = P(1, 1);
	const float P12 = P(1, 2);
	const float P22 = P(2, 2);

	// optimized auto generated code from SymPy script src/lib/ecl/EKF/python/ekf_derivation/main.py
	const float t0 = ecl::powf(P01, 2);
	const float t1 = -t0;
	const float t2 = P00 * P11 + P00 * velObsVar + P11 * velObsVar + t1 + ecl::powf(velObsVar, 2);

	if (fabsf(t2) < 1e-6f) {
		return;
	}

	const float t3 = 1.0F / t2;
	const float t4 = P11 + velObsVar;
	const float t5 = P01 * t3;
	const float t6 = -t5;
	const float t7 = P00 + velObsVar;
	const float t8 = P00 * t4 + t1;
	const float t9 = t5 * velObsVar;
	const float t10 = P11 * t7;
	const float t11 = t1 + t10;
	const float t12 = P01 * P12;
	const float t13 = P02 * t4;
	const float t14 = P01 * P02;
	const float t15 = P12 * t7;
	const float t16 = t0 * velObsVar;
	const float t17 = powf(t2, -2);
	const float t18 = t4 * velObsVar + t8;
	const float t19 = t17 * t18;
	const float t20 = t17 * (t16 + t7 * t8);
	const float t21 = t0 - t10;
	const float t22 = t17 * t21;
	const float t23 = t14 - t15;
	const float t24 = P01 * t23;
	const float t25 = t12 - t13;
	const float t26 = t16 - t21 * t4;
	const float t27 = t17 * t26;
	const float t28 = t11 + t7 * velObsVar;
	const float t30 = t17 * t28;
	const float t31 = P01 * t25;
	const float t32 = t23 * t4 + t31;
	const float t33 = t17 * t32;
	const float t35 = t24 + t25 * t7;
	const float t36 = t17 * t35;

	S_det_inverse = t3;

	S_inverse(0, 0) = t3 * t4;
	S_inverse(0, 1) = t6;
	S_inverse(1, 1) = t3 * t7;
	S_inverse(1, 0) = S_inverse(0, 1);

	K(0, 0) = t3 * t8;
	K(1, 0) = t9;
	K(2, 0) = t3 * (-t12 + t13);
	K(0, 1) = t9;
	K(1, 1) = t11 * t3;
	K(2, 1) = t3 * (-t14 + t15);

	P_new(0, 0) = P00 - t16 * t19 - t20 * t8;
	P_new(0, 1) = P01 * (t18 * t22 - t20 * velObsVar + 1);
	P_new(1, 1) = P11 - t16 * t30 + t22 * t26;
	P_new(0, 2) = P02 + t19 * t24 + t20 * t25;
	P_new(1, 2) = P12 + t23 * t27 + t30 * t31;
	P_new(2, 2) = P22 - t23 * t33 - t25 * t36;
	P_new(1, 0) = P_new(0, 1);
	P_new(2, 0) = P_new(0, 2);
	P_new(2, 1) = P_new(1, 2);
}

void sympyYawEstPrediction(const Vector3f &state, const SquareMatrix3f &P, const Vector2f &d_vel, float d_vel_var,
			   float d_ang_var, SquareMatrix3f &P_new)
{
	const float P00 = P(0, 0);
	const float P01 = P(0, 1);
	const float P02 = P(0, 2);
	const float P11 = P(1, 1);
	const float P12 = P(1, 2);
	const float P22 = P(2, 2);
	const float psi = state(2);
	const float dvx = d_vel(0);
	const float dvy = d_vel(1);
	const float dvxVar = d_vel_var;
	const float dvyVar = d_vel_var;
	const float dazVar = d_ang_var;

	const float S0 = cosf(psi);
	const float S1 = ecl::powf(S0, 2);
	const float S2 = sinf(psi);
	const float S3 = ecl::powf(S2, 2);
	const float S4 = S0 * dvy + S2 * dvx;
	const float S5 = P02 - P22 * S4;
	const float S6 = S0 * dvx - S2 * dvy;
	const float S7 = S0 * S2;
	const float S8 = P01 + S7 * dvxVar - S7 * dvyVar;
	const float S9 = P12 + P22 * S6;

	P_new(0, 0) = P00 - P02 * S4 + S1 * dvxVar + S3 * dvyVar - S4 * S5;
	P_new(0, 1) = -P12 * S4 + S5 * S6 + S8;
	P_new(1, 1) = P11 + P12 * S6 + S1 * dvyVar + S3 * dvxVar + S6 * S9;
	P_new(0, 2) = S5;
	P_new(1, 2) = S9;
	P_new(2, 2) = P22 + dazVar;
}

TEST(YawEstimatorGenerated, SympyVsSymforceUpdate)
{
	const float R = sq(0.1f);

	SquareMatrix<float, 3> P = createRandomCovarianceMatrix3f();

	SquareMatrix<float, 2> innov_var_inv_sympy;
	float innov_var_det_inv_sympy;
	SquareMatrix3f P_new_sympy;
	Matrix<float, 3, 2> K_sympy;
	sympyYawEstUpdate(P, R, innov_var_inv_sympy, innov_var_det_inv_sympy, K_sympy, P_new_sympy);

	SquareMatrix<float, 2> innov_var_inv_symforce;
	float innov_var_det_inv_symforce;
	SquareMatrix3f P_new_symforce;
	Matrix<float, 3, 2> K_symforce;
	sym::YawEstComputeMeasurementUpdate(P, R, FLT_EPSILON,
					    &innov_var_inv_symforce,
					    &innov_var_det_inv_symforce, &K_symforce, &P_new_symforce);
	// copy upper to lower diagonal
	P_new_symforce(1, 0) = P_new_symforce(0, 1);
	P_new_symforce(2, 0) = P_new_symforce(0, 2);
	P_new_symforce(2, 1) = P_new_symforce(1, 2);

	EXPECT_FLOAT_EQ(innov_var_det_inv_sympy, innov_var_det_inv_symforce);
	EXPECT_TRUE(isEqual(P_new_sympy, P_new_symforce));
	EXPECT_TRUE(isEqual(K_sympy, K_symforce));
	EXPECT_TRUE(isEqual(innov_var_inv_sympy, innov_var_inv_symforce));
}

TEST(YawEstimatorGenerated, SympyVsSymforcePrediction)
{
	const float dt = 0.01f;
	const Vector2f d_vel = Vector2f(0.1f, -0.5f) * dt;
	const float d_vel_var = sq(2.f * dt);
	const float d_ang_var = sq(0.1f * dt);

	const Vector3f state(3.f, 10.f, M_PI_F / 2.f);

	SquareMatrix<float, 3> P = createRandomCovarianceMatrix3f();
	SquareMatrix<float, 3> P_new_symforce;

	sym::YawEstPredictCovariance(state, P, d_vel, d_vel_var, d_ang_var, &P_new_symforce);

	EXPECT_GT(P_new_symforce(0, 0), P(0, 0));
	EXPECT_GT(P_new_symforce(1, 1), P(1, 1));
	EXPECT_GT(P_new_symforce(2, 2), P(2, 2));

	SquareMatrix<float, 3> P_new_sympy;
	sympyYawEstPrediction(state, P, d_vel, d_vel_var, d_ang_var, P_new_sympy);

	EXPECT_TRUE(isEqual(P_new_sympy, P_new_symforce));
}
