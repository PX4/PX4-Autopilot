/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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
#include "StickTiltXY.hpp"

#include <geo/geo.h>

using namespace matrix;

TEST(StickTiltXYTest, AllZeroCase)
{
	StickTiltXY stick_tilt_xy{nullptr};
	Vector2f acc_xy = stick_tilt_xy.generateAccelerationSetpoints(Vector2f(), 0.f, 0.f, 0.f);
	EXPECT_EQ(acc_xy, Vector2f());
}

TEST(StickTiltXYTest, NormalRollPitchCases)
{
	// Disable autosaving parameters to avoid busy loop in param_set()
	param_control_autosave(false);

	float value = 45.f;
	param_set(param_find("MPC_MAN_TILT_MAX"), &value);

	StickTiltXY stick_tilt_xy{nullptr};
	// Pitch
	Vector2f acc_xy = stick_tilt_xy.generateAccelerationSetpoints(Vector2f(1.f, 0.f), 1.f, 0.f, 0.f);
	EXPECT_EQ(acc_xy, Vector2f(CONSTANTS_ONE_G, 0.f));
	acc_xy = stick_tilt_xy.generateAccelerationSetpoints(Vector2f(.5f, 0.f), 1.f, 0.f, 0.f);
	EXPECT_EQ(acc_xy, Vector2f(CONSTANTS_ONE_G / 2.f, 0.f));
	acc_xy = stick_tilt_xy.generateAccelerationSetpoints(Vector2f(-.5f, 0.f), 1.f, 0.f, 0.f);
	EXPECT_EQ(acc_xy, Vector2f(-CONSTANTS_ONE_G / 2.f, 0.f));
	acc_xy = stick_tilt_xy.generateAccelerationSetpoints(Vector2f(-1.f, 0.f), 1.f, 0.f, 0.f);
	EXPECT_EQ(acc_xy, Vector2f(-CONSTANTS_ONE_G, 0.f));
	// Roll
	acc_xy = stick_tilt_xy.generateAccelerationSetpoints(Vector2f(0.f, 1.f), 1.f, 0.f, 0.f);
	EXPECT_EQ(acc_xy, Vector2f(0.f, CONSTANTS_ONE_G));
	acc_xy = stick_tilt_xy.generateAccelerationSetpoints(Vector2f(0.f, .5f), 1.f, 0.f, 0.f);
	EXPECT_EQ(acc_xy, Vector2f(0.f, CONSTANTS_ONE_G / 2.f));
	acc_xy = stick_tilt_xy.generateAccelerationSetpoints(Vector2f(0.f, -.5f), 1.f, 0.f, 0.f);
	EXPECT_EQ(acc_xy, Vector2f(0.f, -CONSTANTS_ONE_G / 2.f));
	acc_xy = stick_tilt_xy.generateAccelerationSetpoints(Vector2f(0.f, -1.f), 1.f, 0.f, 0.f);
	EXPECT_EQ(acc_xy, Vector2f(0.f, -CONSTANTS_ONE_G));
	// Roll & Pitch
	acc_xy = stick_tilt_xy.generateAccelerationSetpoints(Vector2f(1.f, 1.f), 1.f, 0.f, 0.f);
	EXPECT_EQ(acc_xy, Vector2f(CONSTANTS_ONE_G / M_SQRT2_F, CONSTANTS_ONE_G / M_SQRT2_F));
	acc_xy = stick_tilt_xy.generateAccelerationSetpoints(Vector2f(1.f, -1.f), 1.f, 0.f, 0.f);
	EXPECT_EQ(acc_xy, Vector2f(CONSTANTS_ONE_G / M_SQRT2_F, -CONSTANTS_ONE_G / M_SQRT2_F));
	acc_xy = stick_tilt_xy.generateAccelerationSetpoints(Vector2f(-1.f, 1.f), 1.f, 0.f, 0.f);
	EXPECT_EQ(acc_xy, Vector2f(-CONSTANTS_ONE_G / M_SQRT2_F, CONSTANTS_ONE_G / M_SQRT2_F));
	acc_xy = stick_tilt_xy.generateAccelerationSetpoints(Vector2f(-1.f, -1.f), 1.f, 0.f, 0.f);
	EXPECT_EQ(acc_xy, Vector2f(-CONSTANTS_ONE_G / M_SQRT2_F, -CONSTANTS_ONE_G / M_SQRT2_F));
}

TEST(StickTiltXYTest, 90degreeCase)
{
	// Disable autosaving parameters to avoid busy loop in param_set()
	param_control_autosave(false);

	float value = 90.f;
	param_set(param_find("MPC_MAN_TILT_MAX"), &value);

	StickTiltXY stick_tilt_xy{nullptr};
	// Pitch
	// Zero input leads to zero output
	Vector2f acc_xy = stick_tilt_xy.generateAccelerationSetpoints(Vector2f(), 1.f, 0.f, 0.f);
	EXPECT_EQ(acc_xy, Vector2f());
	// Maximum input leads to the maximum of 3g sideways acceleration
	acc_xy = stick_tilt_xy.generateAccelerationSetpoints(Vector2f(1.f, 0.f), 1.f, 0.f, 0.f);
	EXPECT_EQ(acc_xy, Vector2f(3.f * CONSTANTS_ONE_G, 0.f));
}
