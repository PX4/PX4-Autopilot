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

TEST(GnssYawFusionGenerated, SingularityPitch90)
{
	// GIVEN: a vertically oriented antenna (antenna vector aligned with the Forward axis)
	StateSample state{};
	state.quat_nominal = Eulerf(0.f, -M_PI_F / 2.f, 0.f);
	const float yaw_offset = M_PI_F;

	SquareMatrixState P = createRandomCovarianceMatrix();
	const float R_YAW = sq(0.3f);

	float meas_pred;
	float innov_var;
	VectorState H;
	sym::ComputeGnssYawPredInnovVarAndH(state.vector(), P, yaw_offset, R_YAW, FLT_EPSILON, &meas_pred,
					    &innov_var, &H);
	VectorState K = P * H / innov_var;

	// THEN: the arctan is singular, the attitude isn't observable, so the innovation variance
	// is almost infinite and the Kalman gain goes to 0
	EXPECT_GT(innov_var, 1e6f);
	EXPECT_NEAR(K.abs().max(), 0.f, 1e-6f);
}

TEST(GnssYawFusionGenerated, SingularityRoll90)
{
	// GIVEN: a vertically oriented antenna (antenna vector aligned with the Right axis)
	StateSample state{};
	state.quat_nominal = Eulerf(-M_PI_F / 2.f, 0.f, 0.f);
	const float yaw_offset = M_PI_F / 2.f;

	SquareMatrixState P = createRandomCovarianceMatrix();
	const float R_YAW = sq(0.3f);

	float meas_pred;
	float innov_var;
	VectorState H;
	sym::ComputeGnssYawPredInnovVarAndH(state.vector(), P, yaw_offset, R_YAW, FLT_EPSILON, &meas_pred,
					    &innov_var, &H);
	VectorState K = P * H / innov_var;

	// THEN: the arctan is singular, the attitude isn't observable, so the innovation variance
	// is almost infinite and the Kalman gain goes to 0
	EXPECT_GT(innov_var, 1e6f);
	EXPECT_NEAR(K.abs().max(), 0.f, 1e-6f);
}
