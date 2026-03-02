/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
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
#include "StickYaw.hpp"

#include <px4_platform_common/defines.h>

TEST(StickYawTest, UnaidedYawNanTransitionNoYawJump)
{
	// When unaided_yaw transitions from finite to NAN mid-flight,
	// the yaw setpoint must not jump discontinuously.
	// See: https://github.com/PX4/PX4-Autopilot/pull/25710

	param_control_autosave(false);

	StickYaw stick_yaw{nullptr};
	float yawspeed_sp = 0.f;
	float yaw_sp = NAN;
	const float dt = 0.01f;

	// Phase 1: Establish yaw lock at yaw=0 with unaided_yaw=0
	stick_yaw.reset(0.f, 0.f);

	for (int i = 0; i < 10; i++) {
		stick_yaw.generateYawSetpoint(yawspeed_sp, yaw_sp, 0.f, 0.f, dt, 0.f);
	}

	EXPECT_NEAR(yaw_sp, 0.f, 0.01f);

	// Phase 2: Simulate EKF yaw correction — yaw jumps to 0.3 rad while
	// unaided_yaw stays at 0. This creates a yaw_error of 0.3 and triggers
	// the convergence detector, building up _yaw_correction ≈ 0.3.
	for (int i = 0; i < 5; i++) {
		stick_yaw.generateYawSetpoint(yawspeed_sp, yaw_sp, 0.f, 0.3f, dt, 0.f);
	}

	const float yaw_sp_before = yaw_sp;
	ASSERT_TRUE(PX4_ISFINITE(yaw_sp_before));

	// Phase 3: unaided_yaw becomes NAN — yaw setpoint must not jump.
	stick_yaw.generateYawSetpoint(yawspeed_sp, yaw_sp, 0.f, 0.3f, dt, NAN);

	EXPECT_NEAR(yaw_sp, yaw_sp_before, 0.01f)
			<< "Yaw setpoint jumped from " << yaw_sp_before << " to " << yaw_sp
			<< " when unaided_yaw became NAN";
}
