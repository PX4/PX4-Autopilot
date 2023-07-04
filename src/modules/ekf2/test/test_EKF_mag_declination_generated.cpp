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

#include "../EKF/python/ekf_derivation/generated/compute_mag_declination_pred_innov_var_and_h.h"

using namespace matrix;

TEST(MagDeclinationGenerated, declination90deg)
{
	// GIVEN: an estimated mag declination of 90 degrees
	Vector24f state_vector{};
	state_vector(16) = 0.f; // North mag field
	state_vector(17) = 0.2f; // East mag field

	const float R = sq(radians(sq(0.5f)));
	SquareMatrix24f P = createRandomCovarianceMatrix24f();

	Vector24f H;
	float decl_pred;
	float innov_var;

	const float decl = radians(90.f);
	sym::ComputeMagDeclinationPredInnovVarAndH(state_vector, P, R, FLT_EPSILON, &decl_pred, &innov_var, &H);

	// THEN: Even at the singularity point, atan2 is still defined
	EXPECT_TRUE(innov_var < 5000.f && innov_var > R) << "innov_var = " << innov_var;
	EXPECT_LT(fabsf(wrap_pi(decl_pred - decl)), 1e-6f);
}

TEST(MagDeclinationGenerated, declinationUndefined)
{
	// GIVEN: an undefined declination
	Vector24f state_vector{};
	state_vector(16) = 0.f; // North mag field
	state_vector(17) = 0.f; // East mag field

	const float R = sq(radians(sq(0.5f)));
	SquareMatrix24f P = createRandomCovarianceMatrix24f();

	Vector24f H;
	float decl_pred;
	float innov_var;

	const float decl = radians(0.f);
	sym::ComputeMagDeclinationPredInnovVarAndH(state_vector, P, R, FLT_EPSILON, &decl_pred, &innov_var, &H);

	// THEN: the innovation variance is gigantic but finite
	EXPECT_TRUE(PX4_ISFINITE(innov_var) && innov_var > R && innov_var > 1e9f) << "innov_var = " << innov_var;
	EXPECT_LT(fabsf(wrap_pi(decl_pred - decl)), 1e-6f);
}

void sympyMagDeclInnovVarHAndK(float magN, float magE, const SquareMatrix24f &P, float R_DECL,
			       float &innovation_variance, Vector24f &H, Vector24f &Kfusion)
{
	const float h_field_min = 1e-3f;
	const float magN_sq = sq(magN);

	if (magN_sq < sq(h_field_min)) {
		printf("bad numerical conditioning\n");
		return;
	}

	const float HK0 = 1.0F / magN_sq;
	const float HK1 = HK0 * sq(magE) + 1.0F;
	const float HK2 = 1.0F / HK1;
	const float HK3 = 1.0F / magN;
	const float HK4 = HK2 * HK3;
	const float HK5 = HK3 * magE;
	const float HK6 = HK5 * P(16, 17) - P(17, 17);
	const float HK7 = 1.0F / sq(HK1);
	const float HK8 = HK5 * P(16, 16) - P(16, 17);
	innovation_variance = -HK0 * HK6 * HK7 + HK7 * HK8 * magE / (magN * magN_sq) + R_DECL;
	float HK9;

	if (innovation_variance > R_DECL) {
		HK9 = HK4 / innovation_variance;

	} else {
		printf("bad numerical conditioning\n");
		return;
	}

	// Calculate the observation Jacobian
	// Note only 2 terms are non-zero which can be used in matrix operations for calculation of Kalman gains and covariance update to significantly reduce cost
	float Hfusion[24] = {};
	Hfusion[16] = -HK0 * HK2 * magE;
	Hfusion[17] = HK4;

	// Calculate the Kalman gains
	for (unsigned row = 0; row <= 15; row++) {
		Kfusion(row) = -HK9 * (HK5 * P(row, 16) - P(row, 17));
	}

	Kfusion(16) = -HK8 * HK9;
	Kfusion(17) = -HK6 * HK9;

	for (unsigned row = 18; row <= 23; row++) {
		Kfusion(row) = -HK9 * (HK5 * P(16, row) - P(17, row));
	}

	for (int row = 0; row < 24; row++) {
		H(row) = Hfusion[row];
	}
}

TEST(MagDeclinationGenerated, SympyVsSymforce)
{
	const float R_DECL = sq(0.3f);
	const float mag_n = 0.08f;
	const float mag_e = -0.06f;

	Vector24f state_vector{};
	state_vector(16) = mag_n;
	state_vector(17) = mag_e;

	const float decl = M_PI_F;

	SquareMatrix24f P = createRandomCovarianceMatrix24f();

	Vector24f H_sympy;
	Vector24f H_symforce;
	Vector24f K_sympy;
	Vector24f K_symforce;
	float innov_sympy;
	float pred_symforce;
	float innov_var_sympy;
	float innov_var_symforce;

	sympyMagDeclInnovVarHAndK(mag_n, mag_e, P, R_DECL, innov_var_sympy, H_sympy, K_sympy);
	innov_sympy = wrap_pi(std::atan2(mag_e, mag_n) - decl);
	sym::ComputeMagDeclinationPredInnovVarAndH(state_vector, P, R_DECL, FLT_EPSILON, &pred_symforce,
			&innov_var_symforce, &H_symforce);
	const float innov_symforce = wrap_pi(pred_symforce - decl);
	K_symforce = P * H_symforce / innov_var_symforce;

	EXPECT_NEAR(innov_sympy, innov_symforce, 1e-5f);
	EXPECT_NEAR(innov_var_sympy, innov_var_symforce, 1e-2f); // Slightly different because of epsilon

	DiffRatioReport report = computeDiffRatioVector24f(H_sympy, H_symforce);
	EXPECT_LT(report.max_diff_fraction, 2e-4f)
			<< "Max diff fraction = " << report.max_diff_fraction
			<< " location index = " << report.max_row
			<< " sympy = " << report.max_v1
			<< " symforce = " << report.max_v2;

	report = computeDiffRatioVector24f(K_sympy, K_symforce);
	EXPECT_LT(report.max_diff_fraction, 2e-4f)
			<< "Max diff fraction = " << report.max_diff_fraction
			<< " location index = " << report.max_row
			<< " sympy = " << report.max_v1
			<< " symforce = " << report.max_v2;
}
