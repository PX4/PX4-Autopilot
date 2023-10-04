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
using D = matrix::Dual<float, 24>;

void computeHDual(const Vector24f &state_vector, int axis, Vector24f &H)
{
	matrix::Quaternion<D> q(D(state_vector(0), 0),
				D(state_vector(1), 1),
				D(state_vector(2), 2),
				D(state_vector(3), 3));

	Vector3<D> mag_field_earth(D(state_vector(16), 16), D(state_vector(17), 17), D(state_vector(18), 18));
	Vector3<D> mag_bias_body(D(state_vector(19), 19), D(state_vector(20), 20), D(state_vector(21), 21));

	Vector3<D> mag_pred = Dcm<D>(q).transpose() * mag_field_earth + mag_bias_body;

	H.setZero();

	for (int i = 0; i <= 23; i++) {
		H(i) = mag_pred(axis).derivative(i);
	}
}

TEST(Mag3DFusionGenerated, symforceVsDual)
{
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

	Vector24f H_dual;
	Vector24f Hfusion_symforce;
	Vector3f mag_innov_var_symforce;

	for (int i = 0; i < 3; i++) {
		computeHDual(state_vector, i, H_dual);

		if (i == 0) {
			Vector3f innov;
			sym::ComputeMagInnovInnovVarAndHx(state_vector, P, Vector3f(), R_MAG, FLT_EPSILON, &innov, &mag_innov_var_symforce,
							  &Hfusion_symforce);

		} else if (i == 1) {
			sym::ComputeMagYInnovVarAndH(state_vector, P, R_MAG, FLT_EPSILON, &(mag_innov_var_symforce(i)), &Hfusion_symforce);

		} else {
			sym::ComputeMagZInnovVarAndH(state_vector, P, R_MAG, FLT_EPSILON, &(mag_innov_var_symforce(i)), &Hfusion_symforce);
		}

		EXPECT_EQ(Hfusion_symforce, H_dual);
	}
}
