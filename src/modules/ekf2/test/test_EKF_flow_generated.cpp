/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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

#include <cfloat>
#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "test_helper/comparison_helper.h"

#include "../EKF/python/ekf_derivation/generated/compute_flow_xy_innov_var_and_hx.h"
#include "../EKF/python/ekf_derivation/generated/compute_flow_y_innov_var_and_h.h"

using namespace matrix;

TEST(FlowGenerated, distBottom0xy)
{
	// GIVEN: 0 distance to the ground (singularity)
	StateSample state{};
	state.quat_nominal = Quatf();

	const float R = sq(radians(sq(0.5f)));
	SquareMatrixState P = createRandomCovarianceMatrix();

	VectorState H;
	Vector2f innov_var;
	sym::ComputeFlowXyInnovVarAndHx(state.vector(), P, R, FLT_EPSILON, &innov_var, &H);
	EXPECT_GT(innov_var(0), 1e12);
	EXPECT_GT(innov_var(1), 1e12);
}

TEST(FlowGenerated, distBottom0y)
{
	// GIVEN: 0 distance to the ground (singularity)
	StateSample state{};
	state.quat_nominal = Quatf();

	const float R = sq(radians(sq(0.5f)));
	SquareMatrixState P = createRandomCovarianceMatrix();

	VectorState H;
	float innov_var;
	sym::ComputeFlowYInnovVarAndH(state.vector(), P, R, FLT_EPSILON, &innov_var, &H);
	EXPECT_GT(innov_var, 1e12);
}

TEST(FlowGenerated, distBottomNeg)
{
	// GIVEN: a small negative distance to the ground (singularity)
	StateSample state{};
	state.quat_nominal = Quatf();
	state.pos(2) = 1e-3f;

	const float R = sq(radians(sq(0.5f)));
	SquareMatrixState P = createRandomCovarianceMatrix();

	VectorState H;
	Vector2f innov_var;
	sym::ComputeFlowXyInnovVarAndHx(state.vector(), P, R, FLT_EPSILON, &innov_var, &H);
	EXPECT_GT(innov_var(0), 1e6);
	EXPECT_GT(innov_var(1), 1e6);
	sym::ComputeFlowYInnovVarAndH(state.vector(), P, R, FLT_EPSILON, &innov_var(1), &H);
	EXPECT_GT(innov_var(1), 1e6);
}
