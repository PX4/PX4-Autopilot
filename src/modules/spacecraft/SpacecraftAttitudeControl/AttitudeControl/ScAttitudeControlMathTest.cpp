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

#include "AttitudeControlMath.hpp"

using namespace matrix;
using namespace AttitudeControlMath;

static const Vector3f z_unit(0.f, 0.f, 1.f);

TEST(AttitudeControlMath, tiltCorrectionNoError)
{
	// GIVEN: some desired (non yaw-rotated) tilt setpoint
	Quatf q_tilt_sp_ne(z_unit, Vector3f(-0.3, 0.1, 0.7));

	// AND: a desired yaw setpoint
	const Quatf q_sp_yaw = AxisAnglef(z_unit, -1.23f);

	// WHEN: the current yaw error is zero (regardless of the tilt)
	const Quatf q = q_sp_yaw * Quatf(z_unit, Vector3f(0.1f, -0.2f, 1.f));
	const Quatf q_tilt_sp_ne_before = q_tilt_sp_ne;
	correctTiltSetpointForYawError(q_tilt_sp_ne, q, q_sp_yaw);

	// THEN: the tilt setpoint is unchanged
	EXPECT_TRUE(isEqual(q_tilt_sp_ne_before, q_tilt_sp_ne));
}

TEST(AttitudeControlMath, tiltCorrectionYaw180)
{
	// GIVEN: some desired (non yaw-rotated) tilt setpoint and a desired yaw setpoint
	Quatf q_tilt_sp_ne(z_unit, Vector3f(-0.3, 0.1, 0.7));
	const Quatf q_sp_yaw = AxisAnglef(z_unit, -M_PI_F / 2.f);

	// WHEN: there is a yaw error of 180 degrees
	const Quatf q_yaw = Quatf(AxisAnglef(z_unit, M_PI_F / 2.f));
	const Quatf q = q_yaw * Quatf(z_unit, Vector3f(0.1f, -0.2f, 1.f));
	const Quatf q_tilt_sp_ne_before = q_tilt_sp_ne;
	correctTiltSetpointForYawError(q_tilt_sp_ne, q, q_sp_yaw);

	// THEN: the tilt is reversed (the corrected tilt angle is the same but the axis of rotation is opposite)
	EXPECT_FLOAT_EQ(AxisAnglef(q_tilt_sp_ne_before).angle(), AxisAnglef(q_tilt_sp_ne).angle());
	EXPECT_TRUE(isEqual(AxisAnglef(q_tilt_sp_ne_before).axis(), -AxisAnglef(q_tilt_sp_ne).axis()));
}

TEST(AttitudeControlMath, tiltCorrection)
{
	// GIVEN: some desired (non yaw-rotated) tilt setpoint and a desired yaw setpoint
	Quatf q_tilt_sp_ne(z_unit, Vector3f(0.5, -0.1, 0.7));
	const Quatf q_sp_yaw = AxisAnglef(z_unit, -1.23f);

	// WHEN: there is a some yaw error
	const Quatf q_yaw = Quatf(AxisAnglef(z_unit, 3.1f));
	const Quatf q = q_yaw * Quatf(z_unit, Vector3f(0.1f, -0.2f, 1.f));
	const Quatf q_tilt_sp_ne_before = q_tilt_sp_ne;
	correctTiltSetpointForYawError(q_tilt_sp_ne, q, q_sp_yaw);

	// THEN: the tilt vector obtained by rotating the corrected tilt by the yaw setpoint is the same as
	// the one obtained by rotating the initial tilt by the current yaw of the vehicle
	EXPECT_TRUE(isEqual((q_sp_yaw * q_tilt_sp_ne).dcm_z(), (q_yaw * q_tilt_sp_ne_before).dcm_z()));
}
