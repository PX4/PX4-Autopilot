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

#include "../EKF/python/ekf_derivation/generated/compute_sideslip_innov_and_innov_var.h"
#include "../EKF/python/ekf_derivation/generated/compute_sideslip_h_and_k.h"

using namespace matrix;
using D = matrix::Dual<float, 24>;

void computeHDual(const Vector24f &state_vector, Vector24f &H)
{
	matrix::Quaternion<D> q(D(state_vector(0), 0),
				D(state_vector(1), 1),
				D(state_vector(2), 2),
				D(state_vector(3), 3));

	Vector3<D> vel_earth(D(state_vector(4), 4), D(state_vector(5), 5), D(state_vector(6), 6));
	Vector3<D> wind_earth(D(state_vector(22), 22), D(state_vector(23), 23), D());
	Vector3<D> vel_rel_body = Dcm<D>(q).transpose() * (vel_earth - wind_earth);
	D sideslip_pred = vel_rel_body(1) / vel_rel_body(0);

	H.setZero();

	for (int i = 0; i <= 23; i++) {
		H(i) = sideslip_pred.derivative(i);
	}
}

TEST(SideslipFusionGenerated, symforceVsDual)
{
	// Compare calculation of observation Jacobians and Kalman gains for sympy and symforce generated equations
	const float R_BETA = sq(2.5f);

	const Quatf q(Eulerf(-M_PI_F / 2.f, M_PI_F / 3.f, M_PI_F * 4.f / 5.f));
	const float q0 = q(0);
	const float q1 = q(1);
	const float q2 = q(2);
	const float q3 = q(3);

	const float vn = 9.0f;
	const float ve = 12.0f;
	const float vd = -1.5f;

	const float vwn = -4.0f;
	const float vwe = 3.0f;

	Vector24f state_vector{};
	state_vector(0) = q0;
	state_vector(1) = q1;
	state_vector(2) = q2;
	state_vector(3) = q3;
	state_vector(4) = vn;
	state_vector(5) = ve;
	state_vector(6) = vd;
	state_vector(22) = vwn;
	state_vector(23) = vwe;

	SquareMatrix24f P = createRandomCovarianceMatrix24f();

	float innov;
	float innov_var;
	Vector24f H_symforce;
	Vector24f K_symforce;
	Vector24f H_dual;

	sym::ComputeSideslipInnovAndInnovVar(state_vector, P, R_BETA, FLT_EPSILON, &innov, &innov_var);
	sym::ComputeSideslipHAndK(state_vector, P, innov_var, FLT_EPSILON, &H_symforce, &K_symforce);
	computeHDual(state_vector, H_dual);

	EXPECT_TRUE((H_symforce - H_dual).max() < 1e-3f) << H_symforce << H_dual;
}
