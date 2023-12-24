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
#include "DifferentialDriveKinematics.hpp"
#include <mathlib/math/Functions.hpp>

using namespace matrix;

TEST(DifferentialDriveKinematicsTest, AllZeroInputCase)
{
	DifferentialDriveKinematics kinematics;
	kinematics.setWheelBase(1.f);
	kinematics.setMaxSpeed(10.f);
	kinematics.setMaxAngularVelocity(10.f);

	// Test with zero linear velocity and zero yaw rate (stationary vehicle)
	EXPECT_EQ(kinematics.computeInverseKinematics(0.f, 0.f), Vector2f());
}


TEST(DifferentialDriveKinematicsTest, InvalidParameterCase)
{
	DifferentialDriveKinematics kinematics;
	kinematics.setWheelBase(0.f);
	kinematics.setMaxSpeed(10.f);
	kinematics.setMaxAngularVelocity(10.f);

	// Test with invalid parameters (zero wheel base and wheel radius)
	EXPECT_EQ(kinematics.computeInverseKinematics(0.f, .1f), Vector2f());
}


TEST(DifferentialDriveKinematicsTest, UnitCase)
{
	DifferentialDriveKinematics kinematics;
	kinematics.setWheelBase(1.f);
	kinematics.setMaxSpeed(10.f);
	kinematics.setMaxAngularVelocity(10.f);

	// Test with unit values for linear velocity and yaw rate
	EXPECT_EQ(kinematics.computeInverseKinematics(1.f, 1.f), Vector2f(0.05f, 0.15f));
}


TEST(DifferentialDriveKinematicsTest, UnitSaturationCase)
{
	DifferentialDriveKinematics kinematics;
	kinematics.setWheelBase(1.f);
	kinematics.setMaxSpeed(1.f);
	kinematics.setMaxAngularVelocity(1.f);

	// Test with unit values for linear velocity and yaw rate, but with max speed that requires saturation
	EXPECT_EQ(kinematics.computeInverseKinematics(1.f, 1.f), Vector2f(0, 1));
}


TEST(DifferentialDriveKinematicsTest, OppositeUnitSaturationCase)
{
	DifferentialDriveKinematics kinematics;
	kinematics.setWheelBase(1.f);
	kinematics.setMaxSpeed(1.f);
	kinematics.setMaxAngularVelocity(1.f);

	// Negative linear velocity for backward motion and positive yaw rate for turning right
	EXPECT_EQ(kinematics.computeInverseKinematics(-1.f, 1.f), Vector2f(-1, 0));
}

TEST(DifferentialDriveKinematicsTest, RandomCase)
{
	DifferentialDriveKinematics kinematics;
	kinematics.setWheelBase(2.f);
	kinematics.setMaxSpeed(1.f);
	kinematics.setMaxAngularVelocity(1.f);

	// Negative linear velocity for backward motion and positive yaw rate for turning right
	EXPECT_EQ(kinematics.computeInverseKinematics(0.5f, 0.7f), Vector2f(-0.4f, 1.0f));
}

TEST(DifferentialDriveKinematicsTest, RotateInPlaceCase)
{
	DifferentialDriveKinematics kinematics;
	kinematics.setWheelBase(1.f);
	kinematics.setMaxSpeed(1.f);
	kinematics.setMaxAngularVelocity(1.f);

	// Test rotating in place (zero linear velocity, non-zero yaw rate)
	EXPECT_EQ(kinematics.computeInverseKinematics(0.f, 1.f), Vector2f(-0.5f, 0.5f));
}

TEST(DifferentialDriveKinematicsTest, StraightMovementCase)
{
	DifferentialDriveKinematics kinematics;
	kinematics.setWheelBase(1.f);
	kinematics.setMaxSpeed(1.f);
	kinematics.setMaxAngularVelocity(1.f);

	// Test moving straight (non-zero linear velocity, zero yaw rate)
	EXPECT_EQ(kinematics.computeInverseKinematics(1.f, 0.f), Vector2f(1.f, 1.f));
}

TEST(DifferentialDriveKinematicsTest, MinInputValuesCase)
{
	DifferentialDriveKinematics kinematics;
	kinematics.setWheelBase(FLT_MIN);
	kinematics.setMaxSpeed(FLT_MIN);
	kinematics.setMaxAngularVelocity(FLT_MIN);

	// Test with minimum possible input values
	EXPECT_EQ(kinematics.computeInverseKinematics(FLT_MIN, FLT_MIN), Vector2f(0.f, 0.f));
}

TEST(DifferentialDriveKinematicsTest, MaxSpeedLimitCase)
{
	DifferentialDriveKinematics kinematics;
	kinematics.setWheelBase(1.f);
	kinematics.setMaxSpeed(1.f);
	kinematics.setMaxAngularVelocity(1.f);

	// Test with high linear velocity and yaw rate, expecting speeds to be scaled down to fit the max speed
	EXPECT_EQ(kinematics.computeInverseKinematics(10.f, 10.f), Vector2f(0.f, 1.f));
}

TEST(DifferentialDriveKinematicsTest, MaxSpeedForwardsCase)
{
	DifferentialDriveKinematics kinematics;
	kinematics.setWheelBase(1.f);
	kinematics.setMaxSpeed(1.f);
	kinematics.setMaxAngularVelocity(1.f);

	// Test with high linear velocity and yaw rate, expecting speeds to be scaled down to fit the max speed
	EXPECT_EQ(kinematics.computeInverseKinematics(10.f, 0.f), Vector2f(1.f, 1.f));
}

TEST(DifferentialDriveKinematicsTest, MaxAngularCase)
{
	DifferentialDriveKinematics kinematics;
	kinematics.setWheelBase(2.f);
	kinematics.setMaxSpeed(1.f);
	kinematics.setMaxAngularVelocity(1.f);

	// Test with high linear velocity and yaw rate, expecting speeds to be scaled down to fit the max speed
	EXPECT_EQ(kinematics.computeInverseKinematics(0.f, 10.f), Vector2f(-1.f, 1.f));
}
