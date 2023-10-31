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

#include "../EKF/python/ekf_derivation/generated/compute_mag_declination_pred_innov_var_and_h.h"
#include "../EKF/python/ekf_derivation/generated/state.h"

using namespace matrix;

TEST(MagDeclinationGenerated, declination90deg)
{
	// GIVEN: an estimated mag declination of 90 degrees
	StateSample state{};
	state.mag_I(0) = 0.f; // North mag field
	state.mag_I(1) = 0.2f; // East mag field

	const float R = sq(radians(sq(0.5f)));
	SquareMatrixState P = createRandomCovarianceMatrix();

	VectorState H;
	float decl_pred;
	float innov_var;

	const float decl = radians(90.f);
	sym::ComputeMagDeclinationPredInnovVarAndH(state.vector(), P, R, FLT_EPSILON, &decl_pred, &innov_var, &H);

	// THEN: Even at the singularity point, atan2 is still defined
	EXPECT_TRUE(innov_var < 5000.f && innov_var > R) << "innov_var = " << innov_var;
	EXPECT_LT(fabsf(wrap_pi(decl_pred - decl)), 1e-6f);
}

TEST(MagDeclinationGenerated, declinationUndefined)
{
	// GIVEN: an undefined declination
	StateSample state{};
	state.mag_I(0) = 0.f; // North mag field
	state.mag_I(1) = 0.f; // East mag field

	const float R = sq(radians(sq(0.5f)));
	SquareMatrixState P = createRandomCovarianceMatrix();

	VectorState H;
	float decl_pred;
	float innov_var;

	const float decl = radians(0.f);
	sym::ComputeMagDeclinationPredInnovVarAndH(state.vector(), P, R, FLT_EPSILON, &decl_pred, &innov_var, &H);

	// THEN: the innovation variance is gigantic but finite
	EXPECT_TRUE(PX4_ISFINITE(innov_var) && innov_var > R && innov_var > 1e9f) << "innov_var = " << innov_var;
	EXPECT_LT(fabsf(wrap_pi(decl_pred - decl)), 1e-6f);
}
