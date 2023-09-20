/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
#include <ControlMath.hpp>
#include <AttitudeControl.hpp>
#include <px4_defines.h>

using namespace matrix;

TEST(PositionControlTest, NavigationYawInfluence)
{
	const Vector3f thrust_setpoint(1.f, 1.f, -.5f);
	const float yaw_setpoint = 0.f;

	// Generate attitude
	vehicle_attitude_setpoint_s attitude_setpoint{};
	ControlMath::thrustToAttitude(thrust_setpoint, yaw_setpoint, attitude_setpoint);

	// Set up attitude control
	AttitudeControl attitude_control;
	attitude_control.setProportionalGain(Vector3f(1.f, 1.f, 1.f), 1.f);
	attitude_control.setRateLimit(Vector3f(100.f, 100.f, 100.f));

	// Execute on attitude
	Quatf qd(attitude_setpoint.q_d);
	attitude_control.setAttitudeSetpoint(qd, 0.f);
	const Vector3f rate_setpoint = attitude_control.update(Quatf());
	rate_setpoint.print();

	// expect symmetric angular rate command towards thrust direction without any body yaw correction
	EXPECT_GT(rate_setpoint(0), 0.f);
	EXPECT_LT(rate_setpoint(1), 0.f);
	EXPECT_FLOAT_EQ(fabsf(rate_setpoint(0)), fabsf(rate_setpoint(1)));
	EXPECT_FLOAT_EQ(rate_setpoint(2), 0.f);

	qd.print();
	Eulerf(qd).print();
	EXPECT_FLOAT_EQ(2.f * atan2f(qd(3), qd(0)), yaw_setpoint);
}
