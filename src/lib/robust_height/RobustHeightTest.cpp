/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
#include "RobustHeight.hpp"

static constexpr hrt_abstime DT_US = 100000; // 0.1 s between samples

TEST(RobustHeightTest, FirstSampleIsTaken)
{
	RobustHeight robust_height;
	EXPECT_FLOAT_EQ(robust_height.update(1.2f, 0.f, 1000000), 1.2f);
}

TEST(RobustHeightTest, InvalidDistanceResets)
{
	RobustHeight robust_height;
	robust_height.update(1.f, 0.f, 1000000);
	EXPECT_TRUE(std::isnan(robust_height.update(NAN, 0.f, 1000000 + DT_US)));
	EXPECT_TRUE(std::isnan(robust_height.get()));
	// After an invalid sample the next valid distance is taken directly, even far away
	EXPECT_FLOAT_EQ(robust_height.update(4.f, 0.f, 1000000 + 2 * DT_US), 4.f);
}

TEST(RobustHeightTest, JumpIsRateLimited)
{
	RobustHeight robust_height;
	hrt_abstime t = 1000000;
	robust_height.update(1.f, 0.f, t);

	// Hovering (vz 0) while the distance jumps by 2 m: the height moves by at most 0.3 m/s
	for (int i = 1; i <= 10; i++) {
		t += DT_US;
		EXPECT_NEAR(robust_height.update(3.f, 0.f, t), 1.f + .03f * i, 1e-4f);
	}

	// A jump down is limited the same way
	t += DT_US;
	EXPECT_NEAR(robust_height.update(0.f, 0.f, t), 1.3f - .03f, 1e-4f);
}

TEST(RobustHeightTest, ClimbIsFollowed)
{
	RobustHeight robust_height;
	hrt_abstime t = 1000000;
	robust_height.update(1.f, 0.f, t);

	// Climbing at 1 m/s (vz is down positive): the distance grows with the velocity and is followed exactly
	for (int i = 1; i <= 10; i++) {
		t += DT_US;
		EXPECT_NEAR(robust_height.update(1.f + .1f * i, -1.f, t), 1.f + .1f * i, 1e-4f);
	}
}

TEST(RobustHeightTest, InvalidVelocityTakesDistance)
{
	RobustHeight robust_height;
	robust_height.update(1.f, 0.f, 1000000);
	EXPECT_FLOAT_EQ(robust_height.update(3.f, NAN, 1000000 + DT_US), 3.f);
}

TEST(ArmedHeightTest, FollowsWhileDisarmed)
{
	ArmedHeight armed_height;
	EXPECT_FLOAT_EQ(armed_height.update(-5.f, true, 0, 0.f, false), 0.f);
	EXPECT_FLOAT_EQ(armed_height.update(-7.f, true, 0, 0.f, false), 0.f);
}

TEST(ArmedHeightTest, HeightAboveArmingPoint)
{
	ArmedHeight armed_height;
	armed_height.update(-7.f, true, 0, 0.f, false);
	// Armed at z = -7 m (z is down positive): 1.5 m higher reads 1.5 m
	EXPECT_FLOAT_EQ(armed_height.update(-8.5f, true, 0, 0.f, true), 1.5f);
	EXPECT_FLOAT_EQ(armed_height.update(-6.5f, true, 0, 0.f, true), -.5f);
}

TEST(ArmedHeightTest, EstimatorResetIsCompensated)
{
	ArmedHeight armed_height;
	armed_height.update(-7.f, true, 0, 0.f, false);
	armed_height.update(-8.f, true, 0, 0.f, true);
	// The estimator resets its height by +2 m: z jumps from -8 to -6 but the height stays 1 m
	EXPECT_FLOAT_EQ(armed_height.update(-6.f, true, 1, 2.f, true), 1.f);
}

TEST(ArmedHeightTest, InvalidHeightIsUnknown)
{
	ArmedHeight armed_height;
	armed_height.update(-7.f, true, 0, 0.f, false);
	EXPECT_TRUE(std::isnan(armed_height.update(-8.f, false, 0, 0.f, true)));
	// Armed without ever having a valid height at the arming point
	ArmedHeight never_valid;
	EXPECT_TRUE(std::isnan(never_valid.update(-8.f, true, 0, 0.f, true)));
}
