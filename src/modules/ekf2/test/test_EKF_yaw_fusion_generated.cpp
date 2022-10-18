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
	const Quatf q(Eulerf(M_PI_F, 0.f, M_PI_F));

	Vector24f state_vector{};
	state_vector(0) = q(0);
	state_vector(1) = q(1);
	state_vector(2) = q(2);
	state_vector(3) = q(3);

	const float R = sq(radians(10.f));
	SquareMatrix24f P = createRandomCovarianceMatrix24f();

	Vector24f H_a;
	Vector24f H_b;
	float innov_var_a;
	float innov_var_b;

	// WHEN: computing the innovation variance and H using two different
	// alternate forms (one is singular at pi/2 and the other one at 0)
	sym::ComputeYaw321InnovVarAndH(state_vector, P, R, FLT_EPSILON, &innov_var_a, &H_a);
	sym::ComputeYaw321InnovVarAndHAlternate(state_vector, P, R, FLT_EPSILON, &innov_var_b, &H_b);

	// THEN: Even at the singularity point, the result is still correct, thanks to epsilon
	EXPECT_TRUE(isEqual(H_a, H_b));
	EXPECT_NEAR(innov_var_a, innov_var_b, 1e-5f);
	EXPECT_TRUE(innov_var_a < 50.f && innov_var_a > R) << "innov_var = " << innov_var_a;
}

TEST(YawFusionGenerated, gimbalLock321vs312)
{
	// GIVEN: an attitude at gimbal lock position
	const Quatf q(Eulerf(0.f, -M_PI_F / 2.f, M_PI_F));

	Vector24f state_vector{};
	state_vector(0) = q(0);
	state_vector(1) = q(1);
	state_vector(2) = q(2);
	state_vector(3) = q(3);

	const float R = sq(radians(10.f));
	SquareMatrix24f P = createRandomCovarianceMatrix24f();

	Vector24f H_321;
	Vector24f H_312;
	float innov_var_321;
	float innov_var_312;
	sym::ComputeYaw321InnovVarAndH(state_vector, P, R, FLT_EPSILON, &innov_var_321, &H_321);

	sym::ComputeYaw312InnovVarAndH(state_vector, P, R, FLT_EPSILON, &innov_var_312, &H_312);

	// THEN: both computation are not equivalent, 321 is undefined but 312 is valid
	EXPECT_FALSE(isEqual(H_321, H_312));
	EXPECT_GT(fabsf(innov_var_321 - innov_var_312), 1e6f);
	EXPECT_TRUE(innov_var_312 < 50.f && innov_var_312 > R) << "innov_var = " << innov_var_312;
}

TEST(YawFusionGenerated, positiveVarianceAllOrientations)
{
	const float R = sq(radians(10.f));
	SquareMatrix24f P = createRandomCovarianceMatrix24f();

	Vector24f H;
	float innov_var;

	// GIVEN: all orientations (90 deg steps)
	for (float yaw = 0.f; yaw < 2.f * M_PI_F; yaw += M_PI_F / 2.f) {
		for (float pitch = 0.f; pitch < 2.f * M_PI_F; pitch += M_PI_F / 2.f) {
			for (float roll = 0.f; roll < 2.f * M_PI_F; roll += M_PI_F / 2.f) {
				const Quatf q(Eulerf(roll, pitch, yaw));
				Vector24f state_vector{};
				state_vector(0) = q(0);
				state_vector(1) = q(1);
				state_vector(2) = q(2);
				state_vector(3) = q(3);

				if (shouldUse321RotationSequence(Dcmf(q))) {
					sym::ComputeYaw321InnovVarAndH(state_vector, P, R, FLT_EPSILON, &innov_var, &H);

				} else {
					sym::ComputeYaw312InnovVarAndH(state_vector, P, R, FLT_EPSILON, &innov_var, &H);
				}

				// THEN: the innovation variance must be positive and finite
				EXPECT_TRUE(innov_var < 50.f && innov_var > R)
						<< "yaw = " << degrees(yaw)
						<< " pitch = " << degrees(pitch)
						<< " roll = " << degrees(roll)
						<< " innov_var = " << innov_var;
			}
		}
	}
}

void sympyYaw321A(float q0, float q1, float q2, float q3, const SquareMatrix24f &P, float R_YAW, Vector24f &H)
{
	// This first comparison is for the 321 sequence option A equations that have a singularity when
	// yaw is at +- 90 deg
	const float SA0 = 2 * q3;
	const float SA1 = 2 * q2;
	const float SA2 = SA0 * q0 + SA1 * q1;
	const float SA3 = powf(q0, 2) + powf(q1, 2) - powf(q2, 2) - powf(q3, 2);
	const float SA4 = powf(SA3, -2);
	const float SA5 = 1.0F / (powf(SA2, 2) * SA4 + 1);
	const float SA6 = 1.0F / SA3;
	const float SA7 = SA2 * SA4;
	const float SA8 = 2 * SA7;
	const float SA9 = 2 * SA6;

	float H_YAW[4];
	H_YAW[0] = SA5 * (SA0 * SA6 - SA8 * q0);
	H_YAW[1] = SA5 * (SA1 * SA6 - SA8 * q1);
	H_YAW[2] = SA5 * (SA1 * SA7 + SA9 * q1);
	H_YAW[3] = SA5 * (SA0 * SA7 + SA9 * q0);

	for (int row = 0; row < 4; row++) {
		H(row) = H_YAW[row];
	}
}

void sympyYaw321B(float q0, float q1, float q2, float q3, const SquareMatrix24f &P, float R_YAW, Vector24f &H)
{
	// This second comparison for the 321 sequence option B equations that have a singularity when
	// yaw is at 0 and +-180 deg
	const float SB0 = 2 * q0;
	const float SB1 = 2 * q1;
	const float SB2 = SB0 * q3 + SB1 * q2;
	const float SB3 = powf(SB2, -2);
	const float SB4 = powf(q0, 2) + powf(q1, 2) - powf(q2, 2) - powf(q3, 2);
	const float SB5 = 1.0F / (SB3 * powf(SB4, 2) + 1);
	const float SB6 = 1.0F / SB2;
	const float SB7 = SB3 * SB4;
	const float SB8 = 2 * SB7;
	const float SB9 = 2 * SB6;

	float H_YAW[4];
	H_YAW[0] = -SB5 * (SB0 * SB6 - SB8 * q3);
	H_YAW[1] = -SB5 * (SB1 * SB6 - SB8 * q2);
	H_YAW[2] = -SB5 * (-SB1 * SB7 - SB9 * q2);
	H_YAW[3] = -SB5 * (-SB0 * SB7 - SB9 * q3);

	for (int row = 0; row < 4; row++) {
		H(row) = H_YAW[row];
	}
}

void sympyYaw312A(float q0, float q1, float q2, float q3, const SquareMatrix24f &P, float R_YAW, Vector24f &H)
{
	// This first comparison is for the 312 sequence option A equations that have a singularity when
	// yaw is at +- 90 deg
	const float SA0 = 2 * q3;
	const float SA1 = 2 * q2;
	const float SA2 = SA0 * q0 - SA1 * q1;
	const float SA3 = powf(q0, 2) - powf(q1, 2) + powf(q2, 2) - powf(q3, 2);
	const float SA4 = powf(SA3, -2);
	const float SA5 = 1.0F / (powf(SA2, 2) * SA4 + 1);
	const float SA6 = 1.0F / SA3;
	const float SA7 = SA2 * SA4;
	const float SA8 = 2 * SA7;
	const float SA9 = 2 * SA6;

	float H_YAW[4];
	H_YAW[0] = SA5 * (SA0 * SA6 - SA8 * q0);
	H_YAW[1] = SA5 * (-SA1 * SA6 + SA8 * q1);
	H_YAW[2] = SA5 * (-SA1 * SA7 - SA9 * q1);
	H_YAW[3] = SA5 * (SA0 * SA7 + SA9 * q0);

	for (int row = 0; row < 4; row++) {
		H(row) = H_YAW[row];
	}
}

void sympyYaw312B(float q0, float q1, float q2, float q3, const SquareMatrix24f &P, float R_YAW, Vector24f &H)
{
	// This second comparison for the 312 sequence option B equations that have a singularity when
	// yaw is at 0 and +-180 deg
	const float SB0 = 2 * q0;
	const float SB1 = 2 * q1;
	const float SB2 = -SB0 * q3 + SB1 * q2;
	const float SB3 = powf(SB2, -2);
	const float SB4 = -powf(q0, 2) + powf(q1, 2) - powf(q2, 2) + powf(q3, 2);
	const float SB5 = 1.0F / (SB3 * powf(SB4, 2) + 1);
	const float SB6 = 1.0F / SB2;
	const float SB7 = SB3 * SB4;
	const float SB8 = 2 * SB7;
	const float SB9 = 2 * SB6;

	float H_YAW[4];
	H_YAW[0] = -SB5 * (-SB0 * SB6 + SB8 * q3);
	H_YAW[1] = -SB5 * (SB1 * SB6 - SB8 * q2);
	H_YAW[2] = -SB5 * (-SB1 * SB7 - SB9 * q2);
	H_YAW[3] = -SB5 * (SB0 * SB7 + SB9 * q3);

	for (int row = 0; row < 4; row++) {
		H(row) = H_YAW[row];
	}
}

TEST(YawFusionGenerated, SympyVsSymforce)
{
	const float R = sq(radians(10.f));
	SquareMatrix24f P = createRandomCovarianceMatrix24f();

	Vector24f Hfusion_sympy;
	Vector24f Hfusion_symforce;
	float innov_var;

	// GIVEN: all orientations (90 deg steps)
	for (float yaw = 0.f; yaw < 2.f * M_PI_F; yaw += M_PI_F / 2.f) {
		for (float pitch = 0.f; pitch < 2.f * M_PI_F; pitch += M_PI_F / 2.f) {
			for (float roll = 0.f; roll < 2.f * M_PI_F; roll += M_PI_F / 2.f) {
				const Quatf q(Eulerf(roll, pitch, yaw));
				Vector24f state_vector{};
				state_vector(0) = q(0);
				state_vector(1) = q(1);
				state_vector(2) = q(2);
				state_vector(3) = q(3);

				if (shouldUse321RotationSequence(Dcmf(q))) {
					sym::ComputeYaw321InnovVarAndH(state_vector, P, R, FLT_EPSILON, &innov_var, &Hfusion_symforce);

					if (fabsf(wrap_pi(yaw)) - (M_PI_F / 2.f) > (M_PI_F / 4.f)) {
						sympyYaw321A(q(0), q(1), q(2), q(3), P, R, Hfusion_sympy);

					} else {
						sympyYaw321B(q(0), q(1), q(2), q(3), P, R, Hfusion_sympy);
					}

				} else {
					sym::ComputeYaw312InnovVarAndH(state_vector, P, R, FLT_EPSILON, &innov_var, &Hfusion_symforce);

					if (fabsf(wrap_pi(yaw)) - (M_PI_F / 2.f) > (M_PI_F / 4.f)) {
						sympyYaw312A(q(0), q(1), q(2), q(3), P, R, Hfusion_sympy);

					} else {
						sympyYaw312B(q(0), q(1), q(2), q(3), P, R, Hfusion_sympy);
					}
				}

				const DiffRatioReport report = computeDiffRatioVector24f(Hfusion_sympy, Hfusion_symforce);
				EXPECT_LT(report.max_diff_fraction, 1e-5f)
						<< "Max diff fraction = " << report.max_diff_fraction
						<< " location index = " << report.max_row
						<< " sympy = " << report.max_v1
						<< " symforce = " << report.max_v2
						<< " yaw = " << degrees(yaw)
						<< " pitch = " << degrees(pitch)
						<< " roll = " << degrees(roll);
			}
		}
	}
}
