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

#include "../EKF/python/ekf_derivation/generated/compute_gnss_yaw_pred_innov_var_and_h.h"

using namespace matrix;

void sympyGnssYawInnovVarHAndK(float q0, float q1, float q2, float q3, const SquareMatrix24f &P, float yaw_offset,
			       float R_YAW, float &innov_var, Vector24f &Hfusion, Vector24f &Kfusion)
{
	// calculate intermediate variables
	const float HK0 = sinf(yaw_offset);
	const float HK1 = q0 * q3;
	const float HK2 = q1 * q2;
	const float HK3 = 2 * HK0 * (HK1 - HK2);
	const float HK4 = cosf(yaw_offset);
	const float HK5 = powf(q1, 2);
	const float HK6 = powf(q2, 2);
	const float HK7 = powf(q0, 2) - powf(q3, 2);
	const float HK8 = HK4 * (HK5 - HK6 + HK7);
	const float HK9 = HK3 - HK8;

	if (fabsf(HK9) < 1e-3f) {
		return;
	}

	const float HK10 = 1.0F / HK9;
	const float HK11 = HK4 * q0;
	const float HK12 = HK0 * q3;
	const float HK13 = HK0 * (-HK5 + HK6 + HK7) + 2 * HK4 * (HK1 + HK2);
	const float HK14 = HK10 * HK13;
	const float HK15 = HK0 * q0 + HK4 * q3;
	const float HK16 = HK10 * (HK14 * (HK11 - HK12) + HK15);
	const float HK17 = powf(HK13, 2) / powf(HK9, 2) + 1;

	if (fabsf(HK17) < 1e-3f) {
		return;
	}

	const float HK18 = 2 / HK17;
	// const float HK19 = 1.0F/(-HK3 + HK8);
	const float HK19_inverse = -HK3 + HK8;

	if (fabsf(HK19_inverse) < 1e-6f) {
		return;
	}

	const float HK19 = 1.0F / HK19_inverse;
	const float HK20 = HK4 * q1;
	const float HK21 = HK0 * q2;
	const float HK22 = HK13 * HK19;
	const float HK23 = HK0 * q1 - HK4 * q2;
	const float HK24 = HK19 * (HK22 * (HK20 + HK21) + HK23);
	const float HK25 = HK19 * (-HK20 - HK21 + HK22 * HK23);
	const float HK26 = HK10 * (-HK11 + HK12 + HK14 * HK15);
	const float HK27 = -HK16 * P(0, 0) - HK24 * P(0, 1) - HK25 * P(0, 2) + HK26 * P(0, 3);
	const float HK28 = -HK16 * P(0, 1) - HK24 * P(1, 1) - HK25 * P(1, 2) + HK26 * P(1, 3);
	const float HK29 = 4 / powf(HK17, 2);
	const float HK30 = -HK16 * P(0, 2) - HK24 * P(1, 2) - HK25 * P(2, 2) + HK26 * P(2, 3);
	const float HK31 = -HK16 * P(0, 3) - HK24 * P(1, 3) - HK25 * P(2, 3) + HK26 * P(3, 3);
	const float HK32 = HK18 / (-HK16 * HK27 * HK29 - HK24 * HK28 * HK29 - HK25 * HK29 * HK30 + HK26 * HK29 * HK31 + R_YAW);
	innov_var = -HK16 * HK27 * HK29 - HK24 * HK28 * HK29 - HK25 * HK29 * HK30 + HK26 * HK29 * HK31 + R_YAW;

	// calculate observation jacobian
	// Observation jacobian and Kalman gain vectors
	SparseVector24f<0, 1, 2, 3> Hfusion_sparse;
	Hfusion_sparse.at<0>() = -HK16 * HK18;
	Hfusion_sparse.at<1>() = -HK18 * HK24;
	Hfusion_sparse.at<2>() = -HK18 * HK25;
	Hfusion_sparse.at<3>() = HK18 * HK26;

	for (unsigned i = 0; i < Hfusion_sparse.non_zeros(); i++) {
		Hfusion(i) = Hfusion_sparse.atCompressedIndex(i);
	}

	// calculate the Kalman gains
	// only calculate gains for states we are using
	Kfusion(0) = HK27 * HK32;
	Kfusion(1) = HK28 * HK32;
	Kfusion(2) = HK30 * HK32;
	Kfusion(3) = HK31 * HK32;

	for (unsigned row = 4; row <= 23; row++) {
		Kfusion(row) = HK32 * (-HK16 * P(0, row) - HK24 * P(1, row) - HK25 * P(2, row) + HK26 * P(3, row));
	}
}

TEST(GnssYawFusionGenerated, SympyVsSymforce)
{
	const float R_YAW = sq(0.3f);

	const float yaw_offset = 1.5f;

	const Quatf q(Eulerf(M_PI_F / 2.f, M_PI_F / 3.f, M_PI_F));

	Vector24f state_vector{};
	state_vector(0) = q(0);
	state_vector(1) = q(1);
	state_vector(2) = q(2);
	state_vector(3) = q(3);

	SquareMatrix24f P = createRandomCovarianceMatrix24f();

	float innov_var_sympy;
	Vector24f H_sympy;
	Vector24f K_sympy;
	sympyGnssYawInnovVarHAndK(q(0), q(1), q(2), q(3), P, yaw_offset, R_YAW, innov_var_sympy, H_sympy, K_sympy);

	float meas_pred_symforce;
	float innov_var_symforce;
	Vector24f H_symforce;
	sym::ComputeGnssYawPredInnovVarAndH(state_vector, P, yaw_offset, R_YAW, FLT_EPSILON, &meas_pred_symforce,
					    &innov_var_symforce, &H_symforce);

	// K isn't generated from symbolic anymore to save flash space
	Vector24f K_symforce = P * H_symforce / innov_var_symforce;

	DiffRatioReport report = computeDiffRatioVector24f(H_sympy, H_symforce);
	EXPECT_LT(report.max_diff_fraction, 1e-5f) << "H max diff fraction = " <<
			report.max_diff_fraction <<
			" location index = " << report.max_row << " sympy = " << report.max_v1 << " symforce = " << report.max_v2;

	report = computeDiffRatioVector24f(K_sympy, K_symforce);
	EXPECT_LT(report.max_diff_fraction, 1e-5f) << "K max diff fraction = " <<
			report.max_diff_fraction <<
			" location index = " << report.max_row << " sympy = " << report.max_v1 << " symforce = " << report.max_v2;

	EXPECT_NEAR(innov_var_sympy, innov_var_symforce, 1e-5f);
}

TEST(GnssYawFusionGenerated, SingularityPitch90)
{
	// GIVEN: a vertically oriented antenna (antenna vector aligned with the Forward axis)
	const Quatf q(Eulerf(0.f, -M_PI_F / 2.f, 0.f));
	const float yaw_offset = M_PI_F;

	Vector24f state_vector{};
	state_vector(0) = q(0);
	state_vector(1) = q(1);
	state_vector(2) = q(2);
	state_vector(3) = q(3);

	SquareMatrix24f P = createRandomCovarianceMatrix24f();
	const float R_YAW = sq(0.3f);

	float meas_pred;
	float innov_var;
	Vector24f H;
	sym::ComputeGnssYawPredInnovVarAndH(state_vector, P, yaw_offset, R_YAW, FLT_EPSILON, &meas_pred,
					    &innov_var, &H);
	Vector24f K = P * H / innov_var;

	// THEN: the arctan is singular, the attitude isn't observable, so the innovation variance
	// is almost infinite and the Kalman gain goes to 0
	EXPECT_GT(innov_var, 1e6f);
	EXPECT_NEAR(K.abs().max(), 0.f, 1e-6f);
}

TEST(GnssYawFusionGenerated, SingularityRoll90)
{
	// GIVEN: a vertically oriented antenna (antenna vector aligned with the Right axis)
	const Quatf q(Eulerf(-M_PI_F / 2.f, 0.f, 0.f));
	const float yaw_offset = M_PI_F / 2.f;

	Vector24f state_vector{};
	state_vector(0) = q(0);
	state_vector(1) = q(1);
	state_vector(2) = q(2);
	state_vector(3) = q(3);

	SquareMatrix24f P = createRandomCovarianceMatrix24f();
	const float R_YAW = sq(0.3f);

	float meas_pred;
	float innov_var;
	Vector24f H;
	sym::ComputeGnssYawPredInnovVarAndH(state_vector, P, yaw_offset, R_YAW, FLT_EPSILON, &meas_pred,
					    &innov_var, &H);
	Vector24f K = P * H / innov_var;

	// THEN: the arctan is singular, the attitude isn't observable, so the innovation variance
	// is almost infinite and the Kalman gain goes to 0
	EXPECT_GT(innov_var, 1e6f);
	EXPECT_NEAR(K.abs().max(), 0.f, 1e-6f);
}
