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
using D = matrix::Dual<float, 7>;

void computeHDual(const Vector24f &state_vector, float range, int axis, Vector24f &H)
{
	matrix::Quaternion<D> q(D(state_vector(0), 0),
				D(state_vector(1), 1),
				D(state_vector(2), 2),
				D(state_vector(3), 3));

	// calculate the velocity of the sensor in the earth frame
	Vector3<D> vel_earth(D(state_vector(4), 4), D(state_vector(5), 5), D(state_vector(6), 6));

	// rotate into body frame
	Vector3<D> vel_body = Dcm<D>(q).transpose() * vel_earth;
	matrix::Vector2<D> predicted_flow;
	predicted_flow(0) =  vel_body(1) / range;
	predicted_flow(1) = -vel_body(0) / range;

	H.setZero();

	for (int i = 0; i <= 6; i++) {
		H(i) = predicted_flow(axis).derivative(i);
	}
}

TEST(OptFlowFusionGenerated, symforceVsDual)
{
	// Compare calculation of observation Jacobians
	const Quatf q(Eulerf(M_PI_F / 4.f, -M_PI_F / 6.f, M_PI_F));
	const float q0 = q(0);
	const float q1 = q(1);
	const float q2 = q(2);
	const float q3 = q(3);

	const float vn = 5.f;
	const float ve = 0.f;
	const float vd = -1.5f;

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

	SquareMatrix24f P;

	Vector24f H;
	Vector24f H_dual;
	Vector2f innov_var;

	for (int i = 0; i < 2; i++) {
		computeHDual(state_vector, range, i, H_dual);

		if (i == 0) {
			sym::ComputeFlowXyInnovVarAndHx(state_vector, P, range, R_LOS, FLT_EPSILON, &innov_var, &H);

		} else {
			sym::ComputeFlowYInnovVarAndH(state_vector, P, range, R_LOS, FLT_EPSILON, &innov_var(1), &H);
		}

		// Both derivations are equivalent
		EXPECT_EQ(H, H_dual);

		// Since the state variance is 0, the observation variance is the innovation variance
		EXPECT_FLOAT_EQ(innov_var(i), R_LOS);
	}
}
