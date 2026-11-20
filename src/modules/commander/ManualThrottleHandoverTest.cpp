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

#include "ManualThrottleHandover.hpp"

#include <gtest/gtest.h>

TEST(ManualThrottleHandover, RejectsLowThrottleRtlToStabilized)
{
	EXPECT_TRUE(commander::rejectManualThrottleHandover(0.f, -0.5f, true, true, false, true,
			vehicle_status_s::NAVIGATION_STATE_AUTO_RTL,
			vehicle_status_s::NAVIGATION_STATE_STAB));
}

TEST(ManualThrottleHandover, AllowsThrottleAtThreshold)
{
	EXPECT_FALSE(commander::rejectManualThrottleHandover(0.f, 0.f, true, true, false, true,
			vehicle_status_s::NAVIGATION_STATE_AUTO_RTL,
			vehicle_status_s::NAVIGATION_STATE_STAB));
}

TEST(ManualThrottleHandover, RejectsLowThrottleFromNonDirectManualThrustModes)
{
	EXPECT_TRUE(commander::rejectManualThrottleHandover(-0.5f, -0.6f, true, true, false, true,
			vehicle_status_s::NAVIGATION_STATE_POSCTL,
			vehicle_status_s::NAVIGATION_STATE_STAB));
	EXPECT_TRUE(commander::rejectManualThrottleHandover(0.f, -0.5f, true, true, false, true,
			vehicle_status_s::NAVIGATION_STATE_ALTCTL,
			vehicle_status_s::NAVIGATION_STATE_ACRO));
	EXPECT_TRUE(commander::rejectManualThrottleHandover(0.f, -0.5f, true, true, false, true,
			vehicle_status_s::NAVIGATION_STATE_OFFBOARD,
			vehicle_status_s::NAVIGATION_STATE_ACRO));
}

TEST(ManualThrottleHandover, DoesNotAffectOtherModesOrStates)
{
	EXPECT_FALSE(commander::rejectManualThrottleHandover(0.f, -1.f, true, true, false, true,
			vehicle_status_s::NAVIGATION_STATE_AUTO_RTL,
			vehicle_status_s::NAVIGATION_STATE_AUTO_RTL));
	EXPECT_FALSE(commander::rejectManualThrottleHandover(0.f, -1.f, true, true, true, true,
			vehicle_status_s::NAVIGATION_STATE_AUTO_RTL,
			vehicle_status_s::NAVIGATION_STATE_STAB));
	EXPECT_FALSE(commander::rejectManualThrottleHandover(-1.f, -1.f, true, true, false, true,
			vehicle_status_s::NAVIGATION_STATE_AUTO_RTL,
			vehicle_status_s::NAVIGATION_STATE_STAB));
	EXPECT_FALSE(commander::rejectManualThrottleHandover(0.f, -1.f, true, true, false, false,
			vehicle_status_s::NAVIGATION_STATE_AUTO_RTL,
			vehicle_status_s::NAVIGATION_STATE_STAB));
	EXPECT_FALSE(commander::rejectManualThrottleHandover(0.f, -1.f, true, true, false, true,
			vehicle_status_s::NAVIGATION_STATE_STAB,
			vehicle_status_s::NAVIGATION_STATE_STAB));
}

TEST(ManualThrottleHandover, RejectsUnavailableManualControl)
{
	EXPECT_TRUE(commander::rejectManualThrottleHandover(0.f, 1.f, false, true, false, true,
			vehicle_status_s::NAVIGATION_STATE_AUTO_RTL,
			vehicle_status_s::NAVIGATION_STATE_STAB));
}
