/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
#include <AttitudeControl.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

TEST(AttitudeControlTest, AllZeroCase)
{
	AttitudeControl attitude_control;
	Vector3f rate_setpoint = attitude_control.update(Quatf(), Quatf(), 0.f);
	EXPECT_EQ(rate_setpoint, Vector3f());
}

TEST(AttitudeControlTest, Convergence)
{
	AttitudeControl attitude_control;
	attitude_control.setProportionalGain(Vector3f(.1f,.1f,.1f));
	attitude_control.setRateLimit(Vector3f(10000,10000,10000));

	Quatf quat_goal;
	//Quatf quat_state(0.996f,0.087f,0.f,0.f);
	Quatf quat_state(1,0,0,0);
	quat_state.normalize();

	float error = 10.0f;

	int i;
	for (i = 100; i > 0; i--) {
		// run attitude control to get rate setpoints
		Vector3f rate_setpoint = attitude_control.update(quat_state, quat_goal, 0.f);
		// rotate the simulated state quaternion a bit according to the rate setpoint
		quat_state = quat_state * Quatf(AxisAnglef(rate_setpoint));
		// calculate a very simple error metric
		const float new_error = Vector<float, 4>(quat_state * math::signNoZero(quat_state(0)) - quat_goal).norm();
		// expect the error to get smaller with each iteration
		EXPECT_LT(new_error, error);
		error = new_error;
		// stop if the error is below a numerical threshold
		if (fabsf(error) < 1e-4f) {
			break;
		}
	}

	// it shouldn't have taken longer than an iteration timeout to converge
	EXPECT_GT(i, 0);
	// we need to have really reached the goal attitude
	EXPECT_EQ(quat_state * math::signNoZero(quat_state(0)), quat_goal);
}
