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

using namespace matrix;

TEST(YawFusionGenerated, singularityYawEquivalence)
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

	// WHEN: computing the innovation variance and H using two different
	// alternate forms (one is singular at pi/2 and the other one at 0)
	sym::ComputeYaw321InnovVarAndH(state.vector(), P, R, FLT_EPSILON, &innov_var_a, &H_a);
	sym::ComputeYaw321InnovVarAndHAlternate(state.vector(), P, R, FLT_EPSILON, &innov_var_b, &H_b);

	// THEN: Even at the singularity point, the result is still correct, thanks to epsilon
	EXPECT_TRUE(isEqual(H_a, H_b));
	EXPECT_NEAR(innov_var_a, innov_var_b, 1e-5f);
	EXPECT_TRUE(innov_var_a < 50.f && innov_var_a > R) << "innov_var = " << innov_var_a;
}

TEST(YawFusionGenerated, gimbalLock321vs312)
{
	// GIVEN: an attitude at gimbal lock position
	StateSample state{};
	state.quat_nominal = Eulerf(0.f, -M_PI_F / 2.f, M_PI_F);

	const float R = sq(radians(10.f));
	SquareMatrixState P = createRandomCovarianceMatrix();

	VectorState H_321;
	VectorState H_312;
	float innov_var_321;
	float innov_var_312;
	sym::ComputeYaw321InnovVarAndH(state.vector(), P, R, FLT_EPSILON, &innov_var_321, &H_321);

	sym::ComputeYaw312InnovVarAndH(state.vector(), P, R, FLT_EPSILON, &innov_var_312, &H_312);

	// THEN: both computation are not equivalent, 321 is undefined but 312 is valid
	EXPECT_FALSE(isEqual(H_321, H_312));
	EXPECT_GT(fabsf(innov_var_321 - innov_var_312), 1e6f);
	EXPECT_TRUE(innov_var_312 < 50.f && innov_var_312 > R) << "innov_var = " << innov_var_312;
}

TEST(YawFusionGenerated, positiveVarianceAllOrientations)
{
	const float R = sq(radians(10.f));
	SquareMatrixState P = createRandomCovarianceMatrix();

	VectorState H;
	float innov_var;

	// GIVEN: all orientations (90 deg steps)
	for (float yaw = 0.f; yaw < 2.f * M_PI_F; yaw += M_PI_F / 2.f) {
		for (float pitch = 0.f; pitch < 2.f * M_PI_F; pitch += M_PI_F / 2.f) {
			for (float roll = 0.f; roll < 2.f * M_PI_F; roll += M_PI_F / 2.f) {
				StateSample state{};
				state.quat_nominal = Eulerf(roll, pitch, yaw);

				if (shouldUse321RotationSequence(Dcmf(state.quat_nominal))) {
					sym::ComputeYaw321InnovVarAndH(state.vector(), P, R, FLT_EPSILON, &innov_var, &H);

				} else {
					sym::ComputeYaw312InnovVarAndH(state.vector(), P, R, FLT_EPSILON, &innov_var, &H);
				}

				// THEN: the innovation variance must be positive and finite
				EXPECT_TRUE(innov_var < 100.f && innov_var > R)
						<< "yaw = " << degrees(yaw)
						<< " pitch = " << degrees(pitch)
						<< " roll = " << degrees(roll)
						<< " innov_var = " << innov_var;
			}
		}
	}
}
