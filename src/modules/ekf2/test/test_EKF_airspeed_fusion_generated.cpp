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

#include "../EKF/python/ekf_derivation/generated/compute_airspeed_innov_and_innov_var.h"
#include "../EKF/python/ekf_derivation/generated/compute_airspeed_h_and_k.h"

using namespace matrix;

TEST(AirspeedFusionGenerated, SympyVsSymforce)
{
	// Compare calculation of observation Jacobians and Kalman gains for sympy and symforce generated equations
	const float R_TAS = sq(1.5f);

	const float vn = 9.0f;
	const float ve = 12.0f;
	const float vd = -1.5f;

	const float vwn = -4.0f;
	const float vwe = 3.0f;

	SquareMatrix24f P = createRandomCovarianceMatrix24f();

	// First calculate observationjacobians and Kalman gains using sympy generated equations
	Vector24f Hfusion_sympy;
	Vector24f Kfusion_sympy;

	{
		// Intermediate variables
		const float HK0 = vn - vwn;
		const float HK1 = ve - vwe;
		const float HK2 = ecl::powf(HK0, 2) + ecl::powf(HK1, 2) + ecl::powf(vd, 2);
		const float v_tas_pred = sqrtf(HK2); // predicted airspeed
		//const float HK3 = powf(HK2, -1.0F/2.0F);
		// calculation can be badly conditioned for very low airspeed values so don't fuse this time
		EXPECT_GT(v_tas_pred, 1.f);
		const float HK3 = 1.0f / v_tas_pred;
		const float HK4 = HK0 * HK3;
		const float HK5 = HK1 * HK3;
		const float HK6 = 1.0F / HK2;
		const float HK7 = HK0 * P(4, 6) - HK0 * P(6, 22) + HK1 * P(5, 6) - HK1 * P(6, 23) + P(6, 6) * vd;
		const float HK8 = HK1 * P(5, 23);
		const float HK9 = HK0 * P(4, 5) - HK0 * P(5, 22) + HK1 * P(5, 5) - HK8 + P(5, 6) * vd;
		const float HK10 = HK1 * HK6;
		const float HK11 = HK0 * P(4, 22);
		const float HK12 = HK0 * P(4, 4) - HK1 * P(4, 23) + HK1 * P(4, 5) - HK11 + P(4, 6) * vd;
		const float HK13 = HK0 * HK6;
		const float HK14 = -HK0 * P(22, 23) + HK0 * P(4, 23) - HK1 * P(23, 23) + HK8 + P(6, 23) * vd;
		const float HK15 = -HK0 * P(22, 22) - HK1 * P(22, 23) + HK1 * P(5, 22) + HK11 + P(6, 22) * vd;
		const float inn_var = (-HK10 * HK14 + HK10 * HK9 + HK12 * HK13 - HK13 * HK15 + HK6 * HK7 * vd + R_TAS);
		const float HK16 = HK3 / inn_var;

		// Observation Jacobians
		SparseVector24f<4, 5, 6, 22, 23> Hfusion;
		Hfusion.at<4>() = HK4;
		Hfusion.at<5>() = HK5;
		Hfusion.at<6>() = HK3 * vd;
		Hfusion.at<22>() = -HK4;
		Hfusion.at<23>() = -HK5;

		Vector24f Kfusion;

		for (unsigned row = 0; row <= 3; row++) {
			Kfusion(row) = HK16 * (HK0 * P(4, row) - HK0 * P(row, 22) + HK1 * P(5, row) - HK1 * P(row, 23) + P(6, row) * vd);
		}

		Kfusion(4) = HK12 * HK16;
		Kfusion(5) = HK16 * HK9;
		Kfusion(6) = HK16 * HK7;

		for (unsigned row = 7; row <= 21; row++) {
			Kfusion(row) = HK16 * (HK0 * P(4, row) - HK0 * P(row, 22) + HK1 * P(5, row) - HK1 * P(row, 23) + P(6, row) * vd);
		}

		Kfusion(22) = HK15 * HK16;
		Kfusion(23) = HK14 * HK16;

		// save output
		Hfusion_sympy(4) = Hfusion.at<4>();
		Hfusion_sympy(5) = Hfusion.at<5>();
		Hfusion_sympy(6) = Hfusion.at<6>();
		Hfusion_sympy(22) = Hfusion.at<22>();
		Hfusion_sympy(23) = Hfusion.at<23>();
		Kfusion_sympy = Kfusion;
	}

	// Then calculate observationjacobians and Kalman gains using symforce generated equations
	Vector24f Hfusion_symforce;
	Vector24f Kfusion_symforce;

	{
		Vector24f state_vector{};
		state_vector(4) = vn;
		state_vector(5) = ve;
		state_vector(6) = vd;
		state_vector(22) = vwn;
		state_vector(23) = vwe;

		float innov;
		float innov_var;

		sym::ComputeAirspeedInnovAndInnovVar(state_vector, P, 0.f, R_TAS, FLT_EPSILON, &innov, &innov_var);
		sym::ComputeAirspeedHAndK(state_vector, P, innov_var, FLT_EPSILON, &Hfusion_symforce, &Kfusion_symforce);
	}

	DiffRatioReport report = computeDiffRatioVector24f(Hfusion_sympy, Hfusion_symforce);
	EXPECT_LT(report.max_diff_fraction, 1e-5f) << "Airspeed Hfusion max diff fraction = " << report.max_diff_fraction <<
			" location index = " << report.max_row << " sympy = " << report.max_v1 << " symforce = " << report.max_v2;

	report = computeDiffRatioVector24f(Kfusion_sympy, Kfusion_symforce);
	EXPECT_LT(report.max_diff_fraction, 1e-5f) << "Airspeed Kfusion max diff fraction = " << report.max_diff_fraction <<
			" location index = " << report.max_row << " sympy = " << report.max_v1 << " symforce = " << report.max_v2;
}
