/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
#include <cmath>
#include <mathlib/mathlib.h>

using namespace math::Utilities;
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;

TEST(UtilitiesYawUpdateTest, UpdateYawPreservesTiltRollHeavy)
{
	// more roll than pitch → 321 path
	const Dcmf R0(Eulerf(math::radians(40.f), math::radians(10.f), math::radians(15.f)));
	EXPECT_TRUE(shouldUse321RotationSequence(R0));

	const float new_yaw = math::radians(-42.f);
	const Dcmf R1 = updateYawInRotMat(new_yaw, R0);

	EXPECT_NEAR(getEuler321Yaw(R1), new_yaw, 1e-4f);
	// tilt magnitudes roughly preserved via z-axis components
	EXPECT_NEAR(R1(2, 1), R0(2, 1), 1e-4f); // sin(roll) term for 321
	EXPECT_NEAR(R1(2, 2), R0(2, 2), 5e-3f);
}

TEST(UtilitiesYawUpdateTest, UpdateYawPreservesTiltPitchHeavy)
{
	// more pitch than roll → 312 path
	const Dcmf R0(Eulerf(math::radians(10.f), math::radians(40.f), math::radians(-20.f)));
	EXPECT_FALSE(shouldUse321RotationSequence(R0));

	const float new_yaw = math::radians(1.1f);
	const Dcmf R1 = updateYawInRotMat(new_yaw, R0);

	EXPECT_NEAR(getEuler312Yaw(R1), new_yaw, 1e-3f);
	EXPECT_NEAR(R1(2, 1), R0(2, 1), 1e-3f); // roll from asin
}

TEST(UtilitiesYawUpdateTest, GetEulerYawMatchesSequenceChoice)
{
	const Quatf q_roll_heavy(Eulerf(math::radians(50.f), math::radians(5.f), math::radians(0.3f)));
	const Dcmf R_roll(q_roll_heavy);
	EXPECT_NEAR(getEulerYaw(R_roll), getEuler321Yaw(R_roll), 1e-5f);
	EXPECT_NEAR(getEulerYaw(q_roll_heavy), getEulerYaw(R_roll), 1e-5f);

	const Quatf q_pitch_heavy(Eulerf(math::radians(5.f), math::radians(50.f), math::radians(-0.7f)));
	const Dcmf R_pitch(q_pitch_heavy);
	EXPECT_NEAR(getEulerYaw(R_pitch), getEuler312Yaw(R_pitch), 1e-5f);
}
