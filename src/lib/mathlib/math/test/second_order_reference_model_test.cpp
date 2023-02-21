/****************************************************************************
 *
 *   Copyright (c) 2022 ECL Development Team. All rights reserved.
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
 * @file second_order_reference_model_test.cpp
 *
 * @brief Unit tests for the second order reference model implementation
 */

#include <gtest/gtest.h>
#include <matrix/math.hpp>
#include <lib/mathlib/math/filter/second_order_reference_model.hpp>

using math::SecondOrderReferenceModel;
using matrix::Vector3f;
using matrix::AxisAnglef;
using matrix::Eulerf;

TEST(SecondOrderReferenceModel, FloatDefaultConstructorInitializesStatesToZero)
{
	SecondOrderReferenceModel<float> sys_float;

	// default constructor leaves states initialized to zero
	EXPECT_FLOAT_EQ(sys_float.getState(), 0.0f);
	EXPECT_FLOAT_EQ(sys_float.getRate(), 0.0f);
	EXPECT_FLOAT_EQ(sys_float.getAccel(), 0.0f);
}

TEST(SecondOrderReferenceModel, FloatReset)
{
	SecondOrderReferenceModel<float> sys_float;

	// reset the system states
	float init_state = 3.0f;
	float init_rate = 1.1f;
	sys_float.reset(init_state, init_rate);
	EXPECT_FLOAT_EQ(sys_float.getState(), init_state);
	EXPECT_FLOAT_EQ(sys_float.getRate(), init_rate);
	EXPECT_FLOAT_EQ(sys_float.getAccel(), 0.0f);

	// reset the system without providing rate
	init_state = 5.5f;
	sys_float.reset(init_state);
	EXPECT_FLOAT_EQ(sys_float.getState(), init_state);
	EXPECT_FLOAT_EQ(sys_float.getRate(), 0.0f);
	EXPECT_FLOAT_EQ(sys_float.getAccel(), 0.0f);
}

TEST(SecondOrderReferenceModel, Vector3DefaultConstructorInitializesStatesToZero)
{
	SecondOrderReferenceModel<Vector3f> sys_vector3f;

	// default constructor leaves states initialized to zero
	Vector3f state = sys_vector3f.getState();
	EXPECT_FLOAT_EQ(state(0), 0.0f);
	EXPECT_FLOAT_EQ(state(1), 0.0f);
	EXPECT_FLOAT_EQ(state(2), 0.0f);
	Vector3f rate = sys_vector3f.getRate();
	EXPECT_FLOAT_EQ(rate(0), 0.0f);
	EXPECT_FLOAT_EQ(rate(1), 0.0f);
	EXPECT_FLOAT_EQ(rate(2), 0.0f);
	Vector3f accel = sys_vector3f.getAccel();
	EXPECT_FLOAT_EQ(accel(0), 0.0f);
	EXPECT_FLOAT_EQ(accel(1), 0.0f);
	EXPECT_FLOAT_EQ(accel(2), 0.0f);
}

TEST(SecondOrderReferenceModel, Vector3Reset)
{
	SecondOrderReferenceModel<Vector3f> sys_vector3f;

	// reset the system states
	Vector3f init_state(1.0f, 2.0f, 3.0f);
	Vector3f init_rate(0.1f, 0.2f, 0.3f);
	sys_vector3f.reset(init_state, init_rate);
	Vector3f state = sys_vector3f.getState();
	EXPECT_FLOAT_EQ(state(0), init_state(0));
	EXPECT_FLOAT_EQ(state(1), init_state(1));
	EXPECT_FLOAT_EQ(state(2), init_state(2));
	Vector3f rate = sys_vector3f.getRate();
	EXPECT_FLOAT_EQ(rate(0), init_rate(0));
	EXPECT_FLOAT_EQ(rate(1), init_rate(1));
	EXPECT_FLOAT_EQ(rate(2), init_rate(2));
	Vector3f accel = sys_vector3f.getAccel();
	EXPECT_FLOAT_EQ(accel(0), 0.0f);
	EXPECT_FLOAT_EQ(accel(1), 0.0f);
	EXPECT_FLOAT_EQ(accel(2), 0.0f);

	// reset the system without providing rate
	init_state = Vector3f(4.0f, 5.0f, 6.0f);
	sys_vector3f.reset(init_state);
	state = sys_vector3f.getState();
	EXPECT_FLOAT_EQ(state(0), init_state(0));
	EXPECT_FLOAT_EQ(state(1), init_state(1));
	EXPECT_FLOAT_EQ(state(2), init_state(2));
	rate = sys_vector3f.getRate();
	EXPECT_FLOAT_EQ(rate(0), 0.0f);
	EXPECT_FLOAT_EQ(rate(1), 0.0f);
	EXPECT_FLOAT_EQ(rate(2), 0.0f);
	accel = sys_vector3f.getAccel();
	EXPECT_FLOAT_EQ(accel(0), 0.0f);
	EXPECT_FLOAT_EQ(accel(1), 0.0f);
	EXPECT_FLOAT_EQ(accel(2), 0.0f);
}

TEST(SecondOrderReferenceModel, Attitude)
{
	SecondOrderReferenceModel<AxisAnglef, Vector3f> sys;

	// reset the system states
	AxisAnglef init_state(Eulerf(0.f, 0.f, 0.f));
	Vector3f init_rate(0.f, 0.f, 0.f);
	sys.reset(init_state, init_rate);

	const float natural_freq = 5.f;
	const float damping_ratio = 0.707f;
	sys.setParameters(natural_freq, damping_ratio);

	AxisAnglef desired_state(Eulerf(M_PI_F / 4.f, -M_PI_F / 4.f, 0.f));

	const float dt = 0.01f;
	const float t_end = 1.f;

	for (float t = 0.f; t <= t_end; t += dt) {
		sys.update(dt, desired_state);
	}

	Vector3f rate = sys.getRate();
	// Synchronous roll and pitch body motion
	EXPECT_NEAR(rate(0), -rate(1), 1e-5f);
	// Induced body rate yaw
	EXPECT_LT(rate(2), -FLT_EPSILON);
}

TEST(SecondOrderReferenceModel, AngleWrapping)
{
	SecondOrderReferenceModel<AxisAnglef, Vector3f> sys;

	// reset the system states
	AxisAnglef init_state(0.f, 0.f, -M_PI_F + 0.01f);
	sys.reset(init_state);

	const float natural_freq = 5.f;
	const float damping_ratio = 0.707f;
	sys.setParameters(natural_freq, damping_ratio);
	sys.setDiscretizationMethod(SecondOrderReferenceModel<AxisAnglef, Vector3f>::DiscretizationMethod::kBilinear);

	AxisAnglef desired_state(0.f, 0.f, M_PI_F - 0.01f);

	const float dt = 0.01f;
	const float t_end = 0.2f;

	for (float t = 0.f; t <= t_end; t += dt) {
		AxisAnglef state = sys.getState();
		Vector3f accel = sys.getAccel();
		Vector3f rate = sys.getRate();
		AxisAnglef error = desired_state - state;
		printf("sp: %f, state: %f, error: %f, accel: %f, rate: %f\n", (double)desired_state(2), (double)state(2),
		       (double)error(2), (double)accel(2), (double)rate(2));
		sys.update(dt, desired_state);
		ASSERT_LT(fabsf(accel(2)), 1.f);
	}
}
