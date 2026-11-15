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
 * @file test_vtol_quadchute.cpp
 *
 * A quad-chute commanded by the user only switches to multicopter and keeps pusher assist. A detected one is a
 * fixed-wing failure, which triggers the quad-chute failsafe action, blocks arming and stops pusher assist.
 */

#include <gtest/gtest.h>

#include <lib/parameters/param.h>

#include "standard.h"
#include "vtol_att_control_main.h"

using namespace matrix;

TEST(VtolQuadchuteTest, CommandedQuadchuteIsNoFixedWingFailure)
{
	VtolAttitudeControl vtol_att_control;
	vtol_att_control.quadchute(QuadchuteReason::ExternalCommand);
	EXPECT_TRUE(vtol_att_control.is_quadchute_active()); // forces multicopter flight
	EXPECT_FALSE(vtol_att_control.is_fixed_wing_system_failure());
}

TEST(VtolQuadchuteTest, DetectedQuadchuteIsFixedWingFailure)
{
	VtolAttitudeControl vtol_att_control;
	vtol_att_control.quadchute(QuadchuteReason::TransitionTimeout);
	EXPECT_TRUE(vtol_att_control.is_quadchute_active());
	EXPECT_TRUE(vtol_att_control.is_fixed_wing_system_failure());
}

TEST(VtolQuadchuteTest, DetectedQuadchuteReplacesCommandedOne)
{
	VtolAttitudeControl vtol_att_control;
	vtol_att_control.quadchute(QuadchuteReason::ExternalCommand);
	vtol_att_control.quadchute(QuadchuteReason::TransitionTimeout);
	EXPECT_TRUE(vtol_att_control.is_fixed_wing_system_failure());
}

class VtolQuadchutePusherTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		param_control_autosave(false);

		const int32_t fwd_thrust_en = 4; // pusher assist enabled constantly
		param_set(param_find("VT_FWD_THRUST_EN"), &fwd_thrust_en);
	}

	void TearDown() override
	{
		param_reset(param_find("VT_FWD_THRUST_EN"));
		param_control_autosave(true);
	}

	// Hover with a nose-down pitch setpoint, which pusher assist replaces with pusher thrust
	void setHover(VtolAttitudeControl &attc)
	{
		attc.get_control_mode()->flag_control_climb_rate_enabled = true;
		Quatf().copyTo(attc.get_att()->q);
		Quatf(Eulerf(0.f, math::radians(-30.f), 0.f)).copyTo(attc.get_mc_virtual_att_sp()->q_d);
	}

	// Pusher command of one Run() cycle, update_mc_state() only runs on a new attitude setpoint
	float runCycle(VtolAttitudeControl &attc, Standard &standard, bool new_attitude_setpoint)
	{
		standard.update_vtol_state();

		if (new_attitude_setpoint) {
			standard.update_mc_state();
		}

		standard.fill_actuator_outputs();
		return attc.get_thrust_setpoint_0()->xyz[0];
	}
};

TEST_F(VtolQuadchutePusherTest, CommandedQuadchuteKeepsPusherAssist)
{
	VtolAttitudeControl attc;
	Standard standard(&attc);
	setHover(attc);
	attc.quadchute(QuadchuteReason::ExternalCommand);

	const float pusher = runCycle(attc, standard, true);
	EXPECT_GT(pusher, 0.1f);

	// no stop and start between attitude setpoints
	EXPECT_FLOAT_EQ(runCycle(attc, standard, false), pusher);
}

TEST_F(VtolQuadchutePusherTest, DetectedQuadchuteStopsPusherAssist)
{
	VtolAttitudeControl attc;
	Standard standard(&attc);
	setHover(attc);
	attc.quadchute(QuadchuteReason::ExternalCommand);
	EXPECT_GT(runCycle(attc, standard, true), 0.1f);

	// a detected quad-chute stops the pusher at once and keeps pusher assist off (NAN stops the pusher)
	attc.quadchute(QuadchuteReason::TransitionTimeout);
	EXPECT_FALSE(PX4_ISFINITE(runCycle(attc, standard, false)));
	EXPECT_FALSE(PX4_ISFINITE(runCycle(attc, standard, true)));
}
