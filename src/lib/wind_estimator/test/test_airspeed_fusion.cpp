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

/**
 * Run this test only using make tests TESTFILTER=unit-test_airspeed_fusion
 */

#include <gtest/gtest.h>
#include <matrix/math.hpp>
#include <mathlib/math/Functions.hpp>
#include "../python/generated/fuse_airspeed.h"

using namespace matrix;

TEST(TestAirspeedFusion, SingularityHandling)
{
	// GIVEN: all inputs set to zero (singularity point)
	const float true_airspeed = 0.f;
	const Vector3f vel_i{0.f, 0.f, 0.f};
	const Vector3f state{0.f, 0.f, 0.f};
	const Matrix<float, 3, 3> P;
	const float tas_var = 0.f;

	matrix::Matrix<float, 1, 3> H;
	matrix::Matrix<float, 3, 1> K;
	float innov_var;
	float innov;

	// WHEN: no singularity handling is done
	float epsilon = 0.f;
	sym::FuseAirspeed(vel_i, state, P, true_airspeed, tas_var, epsilon,
			  &H, &K, &innov_var, &innov);

	// THEN: some of the returned values are not finite
	EXPECT_TRUE(std::isnan(H(0, 0))) << H(0, 0);
	EXPECT_TRUE(std::isnan(H(0, 1))) << H(0, 1);
	EXPECT_FLOAT_EQ(H(0, 2), 0.f) << H(0, 2);
	EXPECT_TRUE(std::isnan(K(0, 0))) << K(0, 0);
	EXPECT_TRUE(std::isnan(K(1, 0))) << K(1, 0);
	EXPECT_TRUE(std::isnan(K(2, 0))) << K(2, 0);
	EXPECT_TRUE(std::isnan(innov_var));
	EXPECT_FLOAT_EQ(innov, 0.f);

	// BUT WHEN: singularity is avoided by addind a small constant
	epsilon = FLT_EPSILON;
	sym::FuseAirspeed(vel_i, state, P, true_airspeed, tas_var, epsilon,
			  &H, &K, &innov_var, &innov);

	// THEN: the returned values are finite and the Kalman gain is null
	EXPECT_FLOAT_EQ(H(0, 0), 0.f) << H(0, 0);
	EXPECT_FLOAT_EQ(H(0, 1), 0.f) << H(0, 1);
	EXPECT_FLOAT_EQ(H(0, 2), std::sqrt(FLT_EPSILON)) << H(0, 2);
	EXPECT_TRUE(isEqual(K, Vector3f()));
	EXPECT_FLOAT_EQ(innov_var, tas_var);
	EXPECT_FLOAT_EQ(innov, 0.f);
}
