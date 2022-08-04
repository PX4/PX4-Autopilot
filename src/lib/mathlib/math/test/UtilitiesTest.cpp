/****************************************************************************
 *
 *   Copyright (c) 2019 ECL Development Team. All rights reserved.
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
 * @file test_EKF_utils.cpp
 *
 * @brief Unit tests for the miscellaneous EKF utilities
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <mathlib/mathlib.h>

using namespace math::Utilities;

TEST(euler312YawTest, fromQuaternion)
{
	matrix::Quatf q1(3.5f, 2.4f, -0.5f, -3.f);
	q1.normalize();
	const matrix::Eulerf euler1(q1);
	EXPECT_FLOAT_EQ(euler1(2), getEuler321Yaw(q1));

	matrix::Quatf q2(0.f, 0, -1.f, 0.f);
	q2.normalize();
	const matrix::Eulerf euler2(q2);
	EXPECT_FLOAT_EQ(euler2(2), getEuler321Yaw(q2));
}

TEST(shouldUse321RotationSequenceTest, pitch90)
{
	matrix::Eulerf euler(0.f, math::radians(90), 0.f);
	matrix::Dcmf R(euler);
	EXPECT_FALSE(shouldUse321RotationSequence(R));
}

TEST(shouldUse321RotationSequenceTest, roll90)
{
	matrix::Eulerf euler(math::radians(90.f), 0.f, 0.f);
	matrix::Dcmf R(euler);
	EXPECT_TRUE(shouldUse321RotationSequence(R));
}

TEST(shouldUse321RotationSequenceTest, moreRollThanPitch)
{
	matrix::Eulerf euler(math::radians(45.f), math::radians(30.f), 0.f);
	matrix::Dcmf R(euler);
	EXPECT_TRUE(shouldUse321RotationSequence(R));
}

TEST(shouldUse321RotationSequenceTest, morePitchThanRoll)
{
	matrix::Eulerf euler(math::radians(30.f), math::radians(45.f), 0.f);
	matrix::Dcmf R(euler);
	EXPECT_FALSE(shouldUse321RotationSequence(R));
}
