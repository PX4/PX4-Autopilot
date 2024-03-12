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

#include "../EKF/python/ekf_derivation/generated/compute_yaw_321_innov_var_and_h.h"
#include "../EKF/python/ekf_derivation/generated/compute_yaw_321_innov_var_and_h_alternate.h"
#include "../EKF/python/ekf_derivation/generated/compute_yaw_312_innov_var_and_h.h"
#include "../EKF/python/ekf_derivation/generated/compute_yaw_312_innov_var_and_h_alternate.h"
#include "../EKF/python/ekf_derivation/generated/compute_yaw_innov_var_and_h.h"

using namespace matrix;

TEST(YawFusionGenerated, yawSingularity)
{
	// GIVEN: an attitude that should give a singularity when transforming the
	// rotation matrix to Euler yaw
	StateSample state{};
	state.quat_nominal = Eulerf(M_PI_F, 0.f, M_PI_F);

	const float R = sq(radians(10.f));
	SquareMatrixState P = createRandomCovarianceMatrix();

	VectorState H_a;
	VectorState H_b;
	float innov_var_a;
	float innov_var_b;

	// WHEN: computing the innovation variance and H using two different methods
	sym::ComputeYaw321InnovVarAndH(state.vector(), P, R, FLT_EPSILON, &innov_var_a, &H_a);
	sym::ComputeYawInnovVarAndH(state.vector(), P, R, &innov_var_b, &H_b);

	// THEN: Even at the singularity point, the result is still correct
	EXPECT_TRUE(isEqual(H_a, H_b));

	EXPECT_NEAR(innov_var_a, innov_var_b, 1e-5f);
	EXPECT_TRUE(innov_var_a < 50.f && innov_var_a > R) << "innov_var = " << innov_var_a;
}

TEST(YawFusionGenerated, gimbalLock321vs312vsTangent)
{
	// GIVEN: an attitude at gimbal lock position
	StateSample state{};
	state.quat_nominal = Eulerf(0.f, -M_PI_F / 2.f, M_PI_F);

	const float R = sq(radians(10.f));
	SquareMatrixState P = createRandomCovarianceMatrix();

	VectorState H_321;
	VectorState H_312;
	VectorState H_tangent;
	float innov_var_321;
	float innov_var_312;
	float innov_var_tangent;
	sym::ComputeYaw321InnovVarAndH(state.vector(), P, R, FLT_EPSILON, &innov_var_321, &H_321);

	sym::ComputeYaw312InnovVarAndH(state.vector(), P, R, FLT_EPSILON, &innov_var_312, &H_312);
	sym::ComputeYawInnovVarAndH(state.vector(), P, R, &innov_var_tangent, &H_tangent);

	// THEN: both computation are not equivalent, 321 is undefined but 312 and "tangent" are valid
	EXPECT_FALSE(isEqual(H_321, H_312));
	EXPECT_TRUE(isEqual(H_312, H_tangent));
	EXPECT_GT(fabsf(innov_var_321 - innov_var_312), 1e6f);
	EXPECT_NEAR(innov_var_312, innov_var_tangent, 1e-6f);
	EXPECT_TRUE(innov_var_312 < 50.f && innov_var_312 > R) << "innov_var = " << innov_var_312;
}

Vector3f getRotVarNed(const Quatf &q, const SquareMatrixState &P)
{
	constexpr auto S = State::quat_nominal;
	matrix::SquareMatrix3f rot_cov_body = P.slice<S.dof, S.dof>(S.idx, S.idx);
	auto R_to_earth = Dcmf(q);
	return matrix::SquareMatrix<float, State::quat_nominal.dof>(R_to_earth * rot_cov_body * R_to_earth.T()).diag();
}

TEST(YawFusionGenerated, positiveVarianceAllOrientations)
{
	const float R = sq(radians(10.f));
	SquareMatrixState P = createRandomCovarianceMatrix();

	VectorState H;
	float innov_var;

	// GIVEN: all orientations
	for (float yaw = 0.f; yaw < 2.f * M_PI_F; yaw += M_PI_F / 4.f) {
		for (float pitch = 0.f; pitch < 2.f * M_PI_F; pitch += M_PI_F / 4.f) {
			for (float roll = 0.f; roll < 2.f * M_PI_F; roll += M_PI_F / 4.f) {
				StateSample state{};
				state.quat_nominal = Eulerf(roll, pitch, yaw);

				sym::ComputeYawInnovVarAndH(state.vector(), P, R, &innov_var, &H);

				// THEN: the innovation variance must be positive and finite
				EXPECT_TRUE(innov_var < 100.f && innov_var > R)
						<< "yaw = " << degrees(yaw)
						<< " pitch = " << degrees(pitch)
						<< " roll = " << degrees(roll)
						<< " innov_var = " << innov_var;

				// AND: it should be the same as the "true" innovation variance obtained by summing
				// the Z rotation variance in NED and the measurement variance
				const float innov_var_true = getRotVarNed(state.quat_nominal, P)(2) + R;
				EXPECT_NEAR(innov_var, innov_var_true, 1e-5f)
						<< "yaw = " << degrees(yaw)
						<< " pitch = " << degrees(pitch)
						<< " roll = " << degrees(roll)
						<< " innov_var = " << innov_var
						<< " innov_var_true = " << innov_var_true;
			}
		}
	}
}
