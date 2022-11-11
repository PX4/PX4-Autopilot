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

#include "../EKF/python/ekf_derivation/generated/compute_flow_xy_innov_var_and_hx.h"
#include "../EKF/python/ekf_derivation/generated/compute_flow_y_innov_var_and_h.h"

using namespace matrix;

void sympyFlowInnovVarHAndK(float q0, float q1, float q2, float q3, float vn, float ve, float vd, float range,
			    const SquareMatrix24f &P, float R_LOS, Vector2f &innov_var,
			    Vector24f &Kfusion_x, Vector24f &Kfusion_y, Vector24f &H_x, Vector24f &H_y)
{
	const Dcmf Tbs;

	const float HK0 = -Tbs(1, 0) * q2 + Tbs(1, 1) * q1 + Tbs(1, 2) * q0;
	const float HK1 = Tbs(1, 0) * q3 + Tbs(1, 1) * q0 - Tbs(1, 2) * q1;
	const float HK2 = Tbs(1, 0) * q0 - Tbs(1, 1) * q3 + Tbs(1, 2) * q2;
	const float HK3 = HK0 * vd + HK1 * ve + HK2 * vn;
	const float HK4 = 1.0F / range;
	const float HK5 = 2 * HK4;
	const float HK6 = Tbs(1, 0) * q1 + Tbs(1, 1) * q2 + Tbs(1, 2) * q3;
	const float HK7 = -HK0 * ve + HK1 * vd + HK6 * vn;
	const float HK8 = HK0 * vn - HK2 * vd + HK6 * ve;
	const float HK9 = -HK1 * vn + HK2 * ve + HK6 * vd;
	const float HK10 = q0 * q2;
	const float HK11 = q1 * q3;
	const float HK12 = HK10 + HK11;
	const float HK13 = 2 * Tbs(1, 2);
	const float HK14 = q0 * q3;
	const float HK15 = q1 * q2;
	const float HK16 = HK14 - HK15;
	const float HK17 = 2 * Tbs(1, 1);
	const float HK18 = ecl::powf(q1, 2);
	const float HK19 = ecl::powf(q2, 2);
	const float HK20 = -HK19;
	const float HK21 = ecl::powf(q0, 2);
	const float HK22 = ecl::powf(q3, 2);
	const float HK23 = HK21 - HK22;
	const float HK24 = HK18 + HK20 + HK23;
	const float HK25 = HK12 * HK13 - HK16 * HK17 + HK24 * Tbs(1, 0);
	const float HK26 = HK14 + HK15;
	const float HK27 = 2 * Tbs(1, 0);
	const float HK28 = q0 * q1;
	const float HK29 = q2 * q3;
	const float HK30 = HK28 - HK29;
	const float HK31 = -HK18;
	const float HK32 = HK19 + HK23 + HK31;
	const float HK33 = -HK13 * HK30 + HK26 * HK27 + HK32 * Tbs(1, 1);
	const float HK34 = HK28 + HK29;
	const float HK35 = HK10 - HK11;
	const float HK36 = HK20 + HK21 + HK22 + HK31;
	const float HK37 = HK17 * HK34 - HK27 * HK35 + HK36 * Tbs(1, 2);
	const float HK38 = 2 * HK3;
	const float HK39 = 2 * HK7;
	const float HK40 = 2 * HK8;
	const float HK41 = 2 * HK9;
	const float HK42 = HK25 * P(0, 4) + HK33 * P(0, 5) + HK37 * P(0, 6) + HK38 * P(0, 0) + HK39 * P(0, 1) + HK40 * P(0,
			   2) + HK41 * P(0, 3);
	const float HK43 = ecl::powf(range, -2);
	const float HK44 = HK25 * P(4, 6) + HK33 * P(5, 6) + HK37 * P(6, 6) + HK38 * P(0, 6) + HK39 * P(1, 6) + HK40 * P(2,
			   6) + HK41 * P(3, 6);
	const float HK45 = HK25 * P(4, 5) + HK33 * P(5, 5) + HK37 * P(5, 6) + HK38 * P(0, 5) + HK39 * P(1, 5) + HK40 * P(2,
			   5) + HK41 * P(3, 5);
	const float HK46 = HK25 * P(4, 4) + HK33 * P(4, 5) + HK37 * P(4, 6) + HK38 * P(0, 4) + HK39 * P(1, 4) + HK40 * P(2,
			   4) + HK41 * P(3, 4);
	const float HK47 = HK25 * P(2, 4) + HK33 * P(2, 5) + HK37 * P(2, 6) + HK38 * P(0, 2) + HK39 * P(1, 2) + HK40 * P(2,
			   2) + HK41 * P(2, 3);
	const float HK48 = HK25 * P(3, 4) + HK33 * P(3, 5) + HK37 * P(3, 6) + HK38 * P(0, 3) + HK39 * P(1, 3) + HK40 * P(2,
			   3) + HK41 * P(3, 3);
	const float HK49 = HK25 * P(1, 4) + HK33 * P(1, 5) + HK37 * P(1, 6) + HK38 * P(0, 1) + HK39 * P(1, 1) + HK40 * P(1,
			   2) + HK41 * P(1, 3);

	const float HK51 = Tbs(0, 1) * q1;
	const float HK52 = Tbs(0, 2) * q0;
	const float HK53 = Tbs(0, 0) * q2;
	const float HK54 = HK51 + HK52 - HK53;
	const float HK55 = Tbs(0, 0) * q3;
	const float HK56 = Tbs(0, 1) * q0;
	const float HK57 = Tbs(0, 2) * q1;
	const float HK58 = HK55 + HK56 - HK57;
	const float HK59 = Tbs(0, 0) * q0;
	const float HK60 = Tbs(0, 2) * q2;
	const float HK61 = Tbs(0, 1) * q3;
	const float HK62 = HK59 + HK60 - HK61;
	const float HK63 = HK54 * vd + HK58 * ve + HK62 * vn;
	const float HK64 = Tbs(0, 0) * q1 + Tbs(0, 1) * q2 + Tbs(0, 2) * q3;
	const float HK65 = HK58 * vd + HK64 * vn;
	const float HK66 = -HK54 * ve + HK65;
	const float HK67 = HK54 * vn + HK64 * ve;
	const float HK68 = -HK62 * vd + HK67;
	const float HK69 = HK62 * ve + HK64 * vd;
	const float HK70 = -HK58 * vn + HK69;
	const float HK71 = 2 * Tbs(0, 1);
	const float HK72 = 2 * Tbs(0, 2);
	const float HK73 = HK12 * HK72 + HK24 * Tbs(0, 0);
	const float HK74 = -HK16 * HK71 + HK73;
	const float HK75 = 2 * Tbs(0, 0);
	const float HK76 = HK26 * HK75 + HK32 * Tbs(0, 1);
	const float HK77 = -HK30 * HK72 + HK76;
	const float HK78 = HK34 * HK71 + HK36 * Tbs(0, 2);
	const float HK79 = -HK35 * HK75 + HK78;
	const float HK80 = 2 * HK63;
	const float HK81 = 2 * HK65 + 2 * ve * (-HK51 - HK52 + HK53);
	const float HK82 = 2 * HK67 + 2 * vd * (-HK59 - HK60 + HK61);
	const float HK83 = 2 * HK69 + 2 * vn * (-HK55 - HK56 + HK57);
	const float HK84 = HK71 * (-HK14 + HK15) + HK73;
	const float HK85 = HK72 * (-HK28 + HK29) + HK76;
	const float HK86 = HK75 * (-HK10 + HK11) + HK78;
	const float HK87 = HK80 * P(0, 0) + HK81 * P(0, 1) + HK82 * P(0, 2) + HK83 * P(0, 3) + HK84 * P(0, 4) + HK85 * P(0,
			   5) + HK86 * P(0, 6);
	const float HK88 = HK80 * P(0, 6) + HK81 * P(1, 6) + HK82 * P(2, 6) + HK83 * P(3, 6) + HK84 * P(4, 6) + HK85 * P(5,
			   6) + HK86 * P(6, 6);
	const float HK89 = HK80 * P(0, 5) + HK81 * P(1, 5) + HK82 * P(2, 5) + HK83 * P(3, 5) + HK84 * P(4, 5) + HK85 * P(5,
			   5) + HK86 * P(5, 6);
	const float HK90 = HK80 * P(0, 4) + HK81 * P(1, 4) + HK82 * P(2, 4) + HK83 * P(3, 4) + HK84 * P(4, 4) + HK85 * P(4,
			   5) + HK86 * P(4, 6);
	const float HK91 = HK80 * P(0, 2) + HK81 * P(1, 2) + HK82 * P(2, 2) + HK83 * P(2, 3) + HK84 * P(2, 4) + HK85 * P(2,
			   5) + HK86 * P(2, 6);
	const float HK92 = 2 * HK43;
	const float HK93 = HK80 * P(0, 3) + HK81 * P(1, 3) + HK82 * P(2, 3) + HK83 * P(3, 3) + HK84 * P(3, 4) + HK85 * P(3,
			   5) + HK86 * P(3, 6);
	const float HK94 = HK80 * P(0, 1) + HK81 * P(1, 1) + HK82 * P(1, 2) + HK83 * P(1, 3) + HK84 * P(1, 4) + HK85 * P(1,
			   5) + HK86 * P(1, 6);

	// X-axis
	innov_var(0) = (HK25 * HK43 * HK46 + HK33 * HK43 * HK45 + HK37 * HK43 * HK44 + HK38 * HK42 * HK43 + HK39 * HK43 * HK49 +
			HK40 * HK43 * HK47 + HK41 * HK43 * HK48 + R_LOS);

	{
		const float HK50 = HK4 / innov_var(0);

		// Observation Jacobians - axis 0
		SparseVector24f<0, 1, 2, 3, 4, 5, 6> Hfusion;
		Hfusion.at<0>() = HK3 * HK5;
		Hfusion.at<1>() = HK5 * HK7;
		Hfusion.at<2>() = HK5 * HK8;
		Hfusion.at<3>() = HK5 * HK9;
		Hfusion.at<4>() = HK25 * HK4;
		Hfusion.at<5>() = HK33 * HK4;
		Hfusion.at<6>() = HK37 * HK4;

		// Kalman gains - axis 0
		Vector24f Kfusion;
		Kfusion(0) = HK42 * HK50;
		Kfusion(1) = HK49 * HK50;
		Kfusion(2) = HK47 * HK50;
		Kfusion(3) = HK48 * HK50;
		Kfusion(4) = HK46 * HK50;
		Kfusion(5) = HK45 * HK50;
		Kfusion(6) = HK44 * HK50;

		for (unsigned row = 7; row <= 23; row++) {
			Kfusion(row) = HK50 * (HK25 * P(4, row) + HK33 * P(5, row) + HK37 * P(6, row) + HK38 * P(0, row) + HK39 * P(1,
					       row) + HK40 * P(2, row) + HK41 * P(3, row));
		}

		// copy to arrays used for comparison
		for (int row = 0; row < 7; row++) {
			H_x(row) = Hfusion.atCompressedIndex(row);
		}

		for (int row = 0; row < 24; row++) {
			Kfusion_x(row) = Kfusion(row);
		}
	}

	// Y-axis
	innov_var(1) = (HK43 * HK74 * HK90 + HK43 * HK77 * HK89 + HK43 * HK79 * HK88 + HK43 * HK80 * HK87 + HK66 * HK92 * HK94 +
			HK68 * HK91 * HK92 + HK70 * HK92 * HK93 + R_LOS);

	{
		const float HK95 = HK4 / innov_var(1);

		// Observation Jacobians - axis 1
		SparseVector24f<0, 1, 2, 3, 4, 5, 6> Hfusion;
		Hfusion.at<0>() = -HK5 * HK63;
		Hfusion.at<1>() = -HK5 * HK66;
		Hfusion.at<2>() = -HK5 * HK68;
		Hfusion.at<3>() = -HK5 * HK70;
		Hfusion.at<4>() = -HK4 * HK74;
		Hfusion.at<5>() = -HK4 * HK77;
		Hfusion.at<6>() = -HK4 * HK79;

		// Kalman gains - axis 1
		Vector24f Kfusion;
		Kfusion(0) = -HK87 * HK95;
		Kfusion(1) = -HK94 * HK95;
		Kfusion(2) = -HK91 * HK95;
		Kfusion(3) = -HK93 * HK95;
		Kfusion(4) = -HK90 * HK95;
		Kfusion(5) = -HK89 * HK95;
		Kfusion(6) = -HK88 * HK95;

		for (unsigned row = 7; row <= 23; row++) {
			Kfusion(row) = -HK95 * (HK80 * P(0, row) + HK81 * P(1, row) + HK82 * P(2, row) + HK83 * P(3, row) + HK84 * P(4,
						row) + HK85 * P(5, row) + HK86 * P(6, row));
		}

		// copy to arrays used for comparison
		for (int row = 0; row < 7; row++) {
			H_y(row) = Hfusion.atCompressedIndex(row);
		}

		for (int row = 0; row < 24; row++) {
			Kfusion_y(row) = Kfusion(row);
		}
	}
}

TEST(OptFlowFusionGenerated, SympyVsSymforce)
{
	// Compare calculation of observation Jacobians and Kalman gains for sympy and symforce generated equations
	const Quatf q(Eulerf(-M_PI_F / 2.f, M_PI_F / 3.f, M_PI_F * 4.f / 5.f));
	const float q0 = q(0);
	const float q1 = q(1);
	const float q2 = q(2);
	const float q3 = q(3);

	const float vn = 10.0f * 2.0f * ((float)randf() - 0.5f);
	const float ve = 10.0f * 2.0f * ((float)randf() - 0.5f);
	const float vd = 2.0f * ((float)randf() - 0.5f);

	const float range = 5.0f;

	Vector24f state_vector{};
	state_vector(0) = q0;
	state_vector(1) = q1;
	state_vector(2) = q2;
	state_vector(3) = q3;
	state_vector(4) = vn;
	state_vector(5) = ve;
	state_vector(6) = vd;

	const float R_LOS = sq(0.15f);

	SquareMatrix24f P = createRandomCovarianceMatrix24f();

	Vector24f Hfusion_sympy_x;
	Vector24f Hfusion_sympy_y;
	Vector24f Kfusion_sympy_x;
	Vector24f Kfusion_sympy_y;
	Vector2f innov_var_sympy;

	Vector24f Hfusion_symforce;
	Vector24f Kfusion_symforce;
	Vector2f innov_var_symforce;

	sympyFlowInnovVarHAndK(q0, q1, q2, q3, vn, ve, vd, range, P, R_LOS, innov_var_sympy,
			       Kfusion_sympy_x, Kfusion_sympy_y, Hfusion_sympy_x, Hfusion_sympy_y);

	for (int i = 0; i < 2; i++) {
		Vector24f &Hfusion_sympy = Hfusion_sympy_x;
		Vector24f &Kfusion_sympy = Kfusion_sympy_x;

		if (i == 0) {
			sym::ComputeFlowXyInnovVarAndHx(state_vector, P, range, R_LOS, FLT_EPSILON, &innov_var_symforce, &Hfusion_symforce);

		} else {
			Hfusion_sympy = Hfusion_sympy_y;
			Kfusion_sympy = Kfusion_sympy_y;
			sym::ComputeFlowYInnovVarAndH(state_vector, P, range, R_LOS, FLT_EPSILON, &innov_var_symforce(1), &Hfusion_symforce);
		}

		// K isn't generated from symbolic anymore to save flash space
		Kfusion_symforce = P * Hfusion_symforce / innov_var_symforce(i);

		DiffRatioReport report = computeDiffRatioVector24f(Hfusion_sympy, Hfusion_symforce);
		EXPECT_LT(report.max_diff_fraction, 1e-5f) << "i = " << i << "Hfusion max diff fraction = " <<
				report.max_diff_fraction <<
				" location index = " << report.max_row << " sympy = " << report.max_v1 << " symforce = " << report.max_v2;

		report = computeDiffRatioVector24f(Kfusion_sympy, Kfusion_symforce);
		EXPECT_LT(report.max_diff_fraction, 1e-5f) << "i = " << i << "Kfusion max diff fraction = " <<
				report.max_diff_fraction <<
				" location index = " << report.max_row << " sympy = " << report.max_v1 << " symforce = " << report.max_v2;
		EXPECT_NEAR(innov_var_sympy(i), innov_var_symforce(i), 1e-5f) << "i = " << i;
	}
}
