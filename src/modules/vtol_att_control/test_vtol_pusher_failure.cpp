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

/**
 * @file test_vtol_pusher_failure.cpp
 *
 * A pusher failure is detected in multicopter mode when pusher assist pushes but the vehicle does not move
 * forward. It triggers the quad-chute failsafe, which front transitions then trigger again until reboot.
 */

#include <gtest/gtest.h>

#include <lib/parameters/param.h>
#include <px4_platform_common/time.h>

#include "standard.h"
#include "vtol_att_control_main.h"

using namespace matrix;
using namespace time_literals;

class VtolPusherFailureTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		param_control_autosave(false);

		const int32_t fwd_thrust_en = 4; // pusher assist enabled constantly
		param_set(param_find("VT_FWD_THRUST_EN"), &fwd_thrust_en);

		const float ef_time = 0.1f;
		param_set(param_find("VT_EF_TIME"), &ef_time);
	}

	void TearDown() override
	{
		param_control_autosave(true);
	}

	// Level hover with a pitch setpoint of -45 deg, which pusher assist replaces with thrust above VT_EF_THROT
	void setHover(VtolAttitudeControl &attc, float forward_velocity)
	{
		attc.get_control_mode()->flag_control_climb_rate_enabled = true;
		Quatf().copyTo(attc.get_att()->q);
		Quatf(Eulerf(0.f, math::radians(-45.f), 0.f)).copyTo(attc.get_mc_virtual_att_sp()->q_d);
		attc.get_local_pos()->v_xy_valid = true;
		attc.get_local_pos()->vx = forward_velocity;
		attc.get_local_pos_sp()->vx = 5.f;
	}

	// Fly multicopter mode until the quad-chute or the timeout, returns the flight time
	hrt_abstime fly(VtolAttitudeControl &attc, Standard &standard, hrt_abstime timeout)
	{
		const hrt_abstime start = hrt_absolute_time();

		while (!attc.get_vtol_vehicle_status()->fixed_wing_system_failure && hrt_elapsed_time(&start) < timeout) {
			standard.update_vtol_state();
			standard.update_mc_state();
			px4_usleep(10_ms);
		}

		return hrt_elapsed_time(&start);
	}
};

TEST_F(VtolPusherFailureTest, NoForwardMotionIsPusherFailure)
{
	VtolAttitudeControl attc;
	Standard standard(&attc);
	setHover(attc, 0.f);

	const hrt_abstime flight_time = fly(attc, standard, 2_s);
	EXPECT_TRUE(attc.get_vtol_vehicle_status()->fixed_wing_system_failure);
	EXPECT_GE(flight_time, 100_ms); // not before VT_EF_TIME

	// a front transition command clears the flag, the next update sets it again
	attc.get_vtol_vehicle_status()->fixed_wing_system_failure = false;
	standard.update_vtol_state();
	EXPECT_TRUE(attc.get_vtol_vehicle_status()->fixed_wing_system_failure);
}

TEST_F(VtolPusherFailureTest, ForwardMotionIsNoPusherFailure)
{
	VtolAttitudeControl attc;
	Standard standard(&attc);
	setHover(attc, 1.f);

	fly(attc, standard, 300_ms);
	EXPECT_FALSE(attc.get_vtol_vehicle_status()->fixed_wing_system_failure);
}
