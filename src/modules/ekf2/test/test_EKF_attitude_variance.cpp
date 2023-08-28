/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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

#include "../EKF/python/ekf_derivation/generated/quat_var_to_rot_var.h"
#include "../EKF/python/ekf_derivation/generated/rot_var_ned_to_lower_triangular_quat_cov.h"
#include "../EKF/python/ekf_derivation/generated/compute_yaw_321_innov_var_and_h.h"
#include "../EKF/python/ekf_derivation/generated/compute_yaw_312_innov_var_and_h.h"

using namespace matrix;

Vector3f calcRotVarMatlab(const Quatf &q, const SquareMatrix24f &P)
{
	Vector3f rot_var_vec;
	float q0, q1, q2, q3;

	if (q(0) >= 0.0f) {
		q0 = q(0);
		q1 = q(1);
		q2 = q(2);
		q3 = q(3);

	} else {
		q0 = -q(0);
		q1 = -q(1);
		q2 = -q(2);
		q3 = -q(3);
	}

	float t2 = q0 * q0;
	float t3 = acosf(q0);
	float t4 = -t2 + 1.0f;
	float t5 = t2 - 1.0f;

	if ((t4 > 1e-9f) && (t5 < -1e-9f)) {
		float t6 = 1.0f / t5;
		float t7 = q1 * t6 * 2.0f;
		float t8 = 1.0f / powf(t4, 1.5f);
		float t9 = q0 * q1 * t3 * t8 * 2.0f;
		float t10 = t7 + t9;
		float t11 = 1.0f / sqrtf(t4);
		float t12 = q2 * t6 * 2.0f;
		float t13 = q0 * q2 * t3 * t8 * 2.0f;
		float t14 = t12 + t13;
		float t15 = q3 * t6 * 2.0f;
		float t16 = q0 * q3 * t3 * t8 * 2.0f;
		float t17 = t15 + t16;
		rot_var_vec(0) = t10 * (P(0, 0) * t10 + P(1, 0) * t3 * t11 * 2.0f) + t3 * t11 * (P(0, 1) * t10 + P(1,
				 1) * t3 * t11 * 2.0f) * 2.0f;
		rot_var_vec(1) = t14 * (P(0, 0) * t14 + P(2, 0) * t3 * t11 * 2.0f) + t3 * t11 * (P(0, 2) * t14 + P(2,
				 2) * t3 * t11 * 2.0f) * 2.0f;
		rot_var_vec(2) = t17 * (P(0, 0) * t17 + P(3, 0) * t3 * t11 * 2.0f) + t3 * t11 * (P(0, 3) * t17 + P(3,
				 3) * t3 * t11 * 2.0f) * 2.0f;

	} else {
		rot_var_vec = 4.0f * P.slice<3, 3>(1, 1).diag();
	}

	return rot_var_vec;
}

TEST(AttitudeVariance, matlabVsSymforce)
{
	Quatf q(Eulerf(M_PI_F / 4.f, -M_PI_F / 6.f, M_PI_F));
	q = -q; // use non-canonical quaternion to cover special case

	const SquareMatrix24f P = createRandomCovarianceMatrix24f();
	Vector3f rot_var_matlab = calcRotVarMatlab(q, P);

	Vector24f state_vector{};
	state_vector(0) = q(0);
	state_vector(1) = q(1);
	state_vector(2) = q(2);
	state_vector(3) = q(3);

	Vector3f rot_var_symforce;
	sym::QuatVarToRotVar(state_vector, P, FLT_EPSILON, &rot_var_symforce);

	EXPECT_EQ(rot_var_matlab, rot_var_symforce);
}

TEST(AttitudeVariance, matlabVsSymforceZeroTilt)
{
	Quatf q;

	const SquareMatrix24f P = createRandomCovarianceMatrix24f();
	Vector3f rot_var_matlab = calcRotVarMatlab(q, P);

	Vector24f state_vector{};
	state_vector(0) = q(0);
	state_vector(1) = q(1);
	state_vector(2) = q(2);
	state_vector(3) = q(3);

	Vector3f rot_var_symforce;
	sym::QuatVarToRotVar(state_vector, P, FLT_EPSILON, &rot_var_symforce);

	EXPECT_EQ(rot_var_matlab, rot_var_symforce);

	const Vector3f rot_var_true = 4.0f * P.slice<3, 3>(1, 1).diag(); // special case
	EXPECT_EQ(rot_var_symforce, rot_var_true);
}

void increaseYawVar(const Vector24f &state_vector, SquareMatrix24f &P, const float yaw_var)
{
	SquareMatrix<float, 4> q_cov;
	sym::RotVarNedToLowerTriangularQuatCov(state_vector, Vector3f(0.f, 0.f, yaw_var), &q_cov);
	q_cov.copyLowerToUpperTriangle();
	P.slice<4, 4>(0, 0) += q_cov;
}

void setTiltVar(const Vector24f &state_vector, SquareMatrix24f &P, const float tilt_var)
{
	SquareMatrix<float, 4> q_cov;
	sym::RotVarNedToLowerTriangularQuatCov(state_vector, Vector3f(tilt_var, tilt_var, 0.f), &q_cov);
	q_cov.copyLowerToUpperTriangle();
	P.slice<4, 4>(0, 0) = q_cov;
}

float getYawVar(const Vector24f &state_vector, const SquareMatrix24f &P)
{
	Vector24f H_YAW;
	float yaw_var = 0.f;
	sym::ComputeYaw312InnovVarAndH(state_vector, P, 0.f, FLT_EPSILON, &yaw_var, &H_YAW);
	return yaw_var;
}

TEST(AttitudeVariance, increaseYawVarNoTilt)
{
	Quatf q;
	SquareMatrix24f P;
	Vector24f state_vector{};
	state_vector(0) = q(0);
	state_vector(1) = q(1);
	state_vector(2) = q(2);
	state_vector(3) = q(3);

	const float yaw_var = radians(25.f);
	increaseYawVar(state_vector, P, yaw_var);

	const float var_new = getYawVar(state_vector, P);

	EXPECT_NEAR(var_new, yaw_var, 1e-6f);
}

TEST(AttitudeVariance, increaseYawVarPitch90)
{
	Quatf q(Eulerf(M_PI_F / 2.f, M_PI_F / 2.f, M_PI_F / 3.f));
	SquareMatrix24f P;
	Vector24f state_vector{};
	state_vector(0) = q(0);
	state_vector(1) = q(1);
	state_vector(2) = q(2);
	state_vector(3) = q(3);

	const float yaw_var = radians(25.f);
	increaseYawVar(state_vector, P, yaw_var);

	const float var_new = getYawVar(state_vector, P);

	EXPECT_NEAR(var_new, yaw_var, 1e-6f);
}

TEST(AttitudeVariance, increaseYawWithTilt)
{
	Quatf q(Eulerf(-M_PI_F, M_PI_F / 3.f, -M_PI_F / 5.f));
	SquareMatrix24f P;
	Vector24f state_vector{};
	state_vector(0) = q(0);
	state_vector(1) = q(1);
	state_vector(2) = q(2);
	state_vector(3) = q(3);

	// Set the yaw variance (from 0)
	const float yaw_var_1 = radians(25.f);
	increaseYawVar(state_vector, P, yaw_var_1);

	const float var_1 = getYawVar(state_vector, P);
	EXPECT_NEAR(var_1, yaw_var_1, 1e-6f);

	// Increase it even more
	const float yaw_var_2 = radians(12.f);
	increaseYawVar(state_vector, P, yaw_var_2);

	const float var_2 = getYawVar(state_vector, P);
	EXPECT_NEAR(var_2, yaw_var_1 + yaw_var_2, 1e-6f);
}

TEST(AttitudeVariance, setRotVarNoTilt)
{
	Quatf q;
	SquareMatrix24f P;
	Vector24f state_vector{};
	state_vector(0) = q(0);
	state_vector(1) = q(1);
	state_vector(2) = q(2);
	state_vector(3) = q(3);

	const float tilt_var = radians(1.2f);
	setTiltVar(state_vector, P, tilt_var);

	Vector3f rot_var;
	sym::QuatVarToRotVar(state_vector, P, FLT_EPSILON, &rot_var);

	EXPECT_NEAR(rot_var(0), tilt_var, 1e-6f);
	EXPECT_NEAR(rot_var(1), tilt_var, 1e-6f);
	EXPECT_EQ(rot_var(2), 0.f);

	// Compare against known values (special case)
	EXPECT_EQ(P(0, 0), 0.f);
	EXPECT_EQ(P(1, 1), 0.25f * tilt_var);
	EXPECT_EQ(P(2, 2), 0.25f * tilt_var);
	EXPECT_EQ(P(3, 3), 0.25f * 0.f); // no yaw var
}

TEST(AttitudeVariance, setRotVarPitch90)
{
	Quatf q(Eulerf(0.f, M_PI_F, 0.f));
	SquareMatrix24f P;
	Vector24f state_vector{};
	state_vector(0) = q(0);
	state_vector(1) = q(1);
	state_vector(2) = q(2);
	state_vector(3) = q(3);

	const float tilt_var = radians(1.2f);
	setTiltVar(state_vector, P, tilt_var);

	Vector3f rot_var;
	sym::QuatVarToRotVar(state_vector, P, FLT_EPSILON, &rot_var);

	// TODO: FIXME, due to the nonlinearity of the quaternion parameters,
	// setting the variance and getting it back is approximate.
	// The correct way would be to keep the uncertainty as a 3D vector in the tangent plane
	// instead of converting it to the parameter space
	// EXPECT_NEAR(rot_var(0), tilt_var, 1e-6f);
	// EXPECT_NEAR(rot_var(1), tilt_var, 1e-6f);
	// EXPECT_EQ(rot_var(2), 0.f);
}
