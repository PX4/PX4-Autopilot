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

/**
 * @file SticksHelpersTest.cpp
 * Unit tests for pure stick helper methods (no uORB / ModuleParams required).
 */

#include <gtest/gtest.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>

#include "Sticks.hpp"

using matrix::Vector2f;

TEST(SticksHelpersTest, LimitStickUnitLengthXY)
{
	Vector2f inside(0.3f, -0.4f);
	Sticks::limitStickUnitLengthXY(inside);
	EXPECT_FLOAT_EQ(inside(0), 0.3f);
	EXPECT_FLOAT_EQ(inside(1), -0.4f);

	Vector2f corner(1.f, 1.f);
	Sticks::limitStickUnitLengthXY(corner);
	EXPECT_NEAR(corner.length(), 1.f, 1e-5f);
	EXPECT_NEAR(corner(0), corner(1), 1e-5f);

	Vector2f zero(0.f, 0.f);
	Sticks::limitStickUnitLengthXY(zero);
	EXPECT_FLOAT_EQ(zero(0), 0.f);
	EXPECT_FLOAT_EQ(zero(1), 0.f);
}

TEST(SticksHelpersTest, RotateIntoHeadingFrameXY)
{
	Vector2f v(1.f, 0.f);
	// 90 deg yaw (NED): body forward maps toward +East if yaw_sp NAN uses yaw
	Sticks::rotateIntoHeadingFrameXY(v, M_PI_F / 2.f, NAN);
	EXPECT_NEAR(v(0), 0.f, 1e-5f);
	EXPECT_NEAR(v(1), 1.f, 1e-5f);

	// yaw_setpoint overrides vehicle yaw
	Vector2f v2(1.f, 0.f);
	Sticks::rotateIntoHeadingFrameXY(v2, 0.f, M_PI_F);
	EXPECT_NEAR(v2(0), -1.f, 1e-5f);
	EXPECT_NEAR(v2(1), 0.f, 1e-5f);
}

TEST(SticksHelpersTest, ExpoDeadzoneMatchesMath)
{
	const float value = 0.5f;
	const float expo = 0.6f;
	const float dz = 0.1f;
	EXPECT_FLOAT_EQ(Sticks::expoDeadzone(value, expo, dz), math::expo(math::deadzone(value, dz), expo));
	EXPECT_FLOAT_EQ(Sticks::expoDeadzone(0.f, expo, dz), 0.f);
	EXPECT_FLOAT_EQ(Sticks::expoDeadzone(0.05f, expo, dz), 0.f);
}
