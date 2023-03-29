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
using D = matrix::Dual<float, 4>;

void computeHDual(const Vector24f &state_vector, float yaw_offset, Vector24f &H)
{
	matrix::Quaternion<D> q(D(state_vector(0), 0),
				D(state_vector(1), 1),
				D(state_vector(2), 2),
				D(state_vector(3), 3));

	Dcm<D> R_to_earth(q);
	Vector3<D> ant_vec_bf(cos(D(yaw_offset)), sin(D(yaw_offset)), D());
	Vector3<D> ant_vec_ef = R_to_earth * ant_vec_bf;
	D meas_pred = atan2(ant_vec_ef(1), ant_vec_ef(0));

	H.setZero();

	for (int i = 0; i <= 3; i++) {
		H(i) = meas_pred.derivative(i);
	}
}

TEST(GnssYawFusionGenerated, SympyVsSymforce)
{
	const float R_YAW = sq(0.3f);

	const float yaw_offset = M_PI_F / 8.f;
	const float yaw = M_PI_F;

	const Quatf q(Eulerf(M_PI_F / 4.f, M_PI_F / 3.f, M_PI_F));

	Vector24f state_vector{};
	state_vector(0) = q(0);
	state_vector(1) = q(1);
	state_vector(2) = q(2);
	state_vector(3) = q(3);

	SquareMatrix24f P = createRandomCovarianceMatrix24f();

	Vector24f H_dual;
	computeHDual(state_vector, yaw_offset, H_dual);

	float meas_pred_symforce;
	float innov_var_symforce;
	Vector24f H_symforce;
	sym::ComputeGnssYawPredInnovVarAndH(state_vector, P, yaw_offset, R_YAW, FLT_EPSILON, &meas_pred_symforce,
					    &innov_var_symforce, &H_symforce);

	EXPECT_GT(innov_var_symforce, 50.f);
	EXPECT_LT(innov_var_symforce, 60.f);

	EXPECT_EQ(H_symforce, H_dual);

	// The predicted yaw is not exactly yaw + offset because roll and pitch are non-zero, but it's close to that
	EXPECT_NEAR(meas_pred_symforce, wrap_pi(yaw + yaw_offset), 0.05f);
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
