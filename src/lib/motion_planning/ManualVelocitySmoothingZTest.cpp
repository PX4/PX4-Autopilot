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
 * Test code for Manual Velocity Smoothing Z
 * Run: make tests TESTFILTER=ManualVelocitySmoothingZ
 */

#include <gtest/gtest.h>
#include <cmath>

#include <motion_planning/ManualVelocitySmoothingZ.hpp>

class ManualVelocitySmoothingZTest : public ::testing::Test
{
public:
	ManualVelocitySmoothingZ _smoothing;
};

TEST_F(ManualVelocitySmoothingZTest, resetClearsToInitialState)
{
	_smoothing.setMaxJerk(10.f);
	_smoothing.setMaxAccelUp(5.f);
	_smoothing.setMaxAccelDown(4.f);
	_smoothing.setMaxVelUp(3.f);
	_smoothing.setMaxVelDown(2.f);

	_smoothing.reset(0.f, -1.f, 12.f);

	// reset() seeds the underlying VelocitySmoothing trajectory and clears
	// position lock; ManualVelocitySmoothingZ getters read _state, which is
	// refreshed on update() / setCurrent*. After bare reset, lock is open.
	EXPECT_TRUE(std::isnan(_smoothing.getCurrentPosition()));
	EXPECT_FLOAT_EQ(_smoothing.getCurrentAcceleration(), 0.f);
	EXPECT_FLOAT_EQ(_smoothing.getCurrentJerk(), 0.f);

	// Velocity gets into getters via the public setter / update path
	_smoothing.setCurrentVelocity(-1.f);
	EXPECT_FLOAT_EQ(_smoothing.getCurrentVelocity(), -1.f);
}

TEST_F(ManualVelocitySmoothingZTest, setCurrentStateReflected)
{
	_smoothing.reset(0.f, 0.f, 0.f);
	_smoothing.setCurrentVelocity(-0.5f);
	_smoothing.setCurrentPosition(7.f);

	EXPECT_FLOAT_EQ(_smoothing.getCurrentVelocity(), -0.5f);
	// Without lock, getCurrentPosition returns the locked setpoint (NAN after reset)
	EXPECT_TRUE(std::isnan(_smoothing.getCurrentPosition()));
}

TEST_F(ManualVelocitySmoothingZTest, updateWithZeroTargetStaysBounded)
{
	_smoothing.setMaxJerk(20.f);
	_smoothing.setMaxAccelUp(5.f);
	_smoothing.setMaxAccelDown(5.f);
	_smoothing.setMaxVelUp(3.f);
	_smoothing.setMaxVelDown(3.f);
	_smoothing.reset(0.f, 0.f, 5.f);

	for (int i = 0; i < 50; i++) {
		_smoothing.setCurrentPositionEstimate(5.f);
		_smoothing.update(0.01f, 0.f);
	}

	EXPECT_LE(fabsf(_smoothing.getCurrentVelocity()), 0.2f);
	EXPECT_TRUE(std::isfinite(_smoothing.getCurrentAcceleration()));
}
