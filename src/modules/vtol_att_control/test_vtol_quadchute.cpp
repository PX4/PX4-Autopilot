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
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>

#include "standard.h"
#include "vtol_att_control_main.h"

using namespace matrix;
using namespace time_literals;

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
	void setHover(VtolAttitudeControl &attc, float pitch_degrees = -30.f)
	{
		attc.get_control_mode()->flag_control_climb_rate_enabled = true;
		Quatf().copyTo(attc.get_att()->q);
		Quatf(Eulerf(0.f, math::radians(pitch_degrees), 0.f)).copyTo(attc.get_mc_virtual_att_sp()->q_d);
	}

	// Pusher command of one Run() cycle, update_mc_state() only runs on a new attitude setpoint
	float runCycle(VtolAttitudeControl &attc, VtolType &vtol, bool new_attitude_setpoint)
	{
		vtol.update_vtol_state();

		if (new_attitude_setpoint) {
			vtol.update_mc_state();
		}

		vtol.fill_actuator_outputs();
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

// Forward propulsion failures detected in multicopter mode block front transitions until VT_EF_TIME is set to 0 or reboot.
class VtolForwardPropulsionFailureTest : public VtolQuadchutePusherTest
{
protected:
	void SetUp() override
	{
		VtolQuadchutePusherTest::SetUp();

		const float ef_time = 0.1f;
		param_set(param_find("VT_EF_TIME"), &ef_time);
	}

	void TearDown() override
	{
		param_reset(param_find("VT_EF_TIME"));
		param_reset(param_find("VT_TILT_MC"));
		VtolQuadchutePusherTest::TearDown();
	}

	// Level hover, by default with a pitch setpoint of -45 deg, which pusher assist replaces with thrust above
	// VT_EF_THROT
	void setHover(VtolAttitudeControl &attc, float forward_velocity, float forward_velocity_setpoint = 5.f,
		      float pitch_degrees = -45.f)
	{
		VtolQuadchutePusherTest::setHover(attc, pitch_degrees);
		attc.get_local_pos()->v_xy_valid = true;
		attc.get_local_pos()->vx = forward_velocity;
		attc.get_local_pos_sp()->vx = forward_velocity_setpoint;
	}

	void commandFrontTransition(VtolAttitudeControl &attc)
	{
		vehicle_command_s command{};
		command.timestamp = hrt_absolute_time();
		command.command = vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
		command.param1 = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;
		uORB::Publication<vehicle_command_s> command_pub{ORB_ID(vehicle_command)};
		ASSERT_TRUE(command_pub.publish(command));
		attc.vehicle_cmd_poll();
	}

	QuadchuteReason quadchuteReason(const VtolAttitudeControl &attc) const { return attc._quadchute_reason; }

	// Fly multicopter mode until the quad-chute or the timeout, returns the flight time
	hrt_abstime fly(VtolAttitudeControl &attc, VtolType &vtol, hrt_abstime timeout)
	{
		const hrt_abstime start = hrt_absolute_time();

		while (!attc.is_fixed_wing_system_failure() && hrt_elapsed_time(&start) < timeout) {
			runCycle(attc, vtol, true);
			px4_usleep(10_ms);
		}

		return hrt_elapsed_time(&start);
	}
};

TEST_F(VtolForwardPropulsionFailureTest, NoForwardMotionIsForwardPropulsionFailure)
{
	VtolAttitudeControl attc;
	Standard standard(&attc);
	setHover(attc, 0.f);

	const hrt_abstime flight_time = fly(attc, standard, 2_s);
	EXPECT_TRUE(attc.is_fixed_wing_system_failure());
	EXPECT_GE(flight_time, 100_ms); // not before VT_EF_TIME

	// a front transition command clears the reason, the next update detects the latched forward propulsion failure again
	commandFrontTransition(attc);
	ASSERT_TRUE(attc.is_fixed_wing_requested());
	ASSERT_FALSE(attc.is_quadchute_active());
	EXPECT_FALSE(PX4_ISFINITE(runCycle(attc, standard, false)));
	EXPECT_TRUE(attc.is_fixed_wing_system_failure());
	EXPECT_EQ(standard.get_mode(), mode::ROTARY_WING);
}

TEST_F(VtolForwardPropulsionFailureTest, DisablingDetectionAllowsFrontTransitions)
{
	VtolAttitudeControl attc;
	Standard standard(&attc);
	setHover(attc, 0.f);

	fly(attc, standard, 2_s);
	ASSERT_TRUE(attc.is_fixed_wing_system_failure());

	// VT_EF_TIME set to 0 in flight, then a front transition command clears the quad-chute for good
	const float ef_time = 0.f;
	param_set(param_find("VT_EF_TIME"), &ef_time);
	static_cast<VtolType &>(standard).parameters_update();
	commandFrontTransition(attc);
	ASSERT_TRUE(attc.is_fixed_wing_requested());
	ASSERT_FALSE(attc.is_quadchute_active());
	standard.update_vtol_state();
	EXPECT_FALSE(attc.is_fixed_wing_system_failure());
	EXPECT_EQ(standard.get_mode(), mode::TRANSITION_TO_FW);
}

TEST_F(VtolForwardPropulsionFailureTest, ForwardPropulsionFailureReplacesCommandedQuadchute)
{
	VtolAttitudeControl attc;
	Standard standard(&attc);
	setHover(attc, 0.f);
	attc.quadchute(QuadchuteReason::ExternalCommand);
	ASSERT_FALSE(attc.is_fixed_wing_system_failure());
	EXPECT_GT(runCycle(attc, standard, true), 0.1f);

	fly(attc, standard, 2_s);
	EXPECT_TRUE(attc.is_fixed_wing_system_failure());
	EXPECT_FALSE(PX4_ISFINITE(runCycle(attc, standard, false)));
	EXPECT_FALSE(PX4_ISFINITE(runCycle(attc, standard, true)));
}

TEST_F(VtolForwardPropulsionFailureTest, ForwardVelocityCloseToSetpointIsNoForwardPropulsionFailure)
{
	VtolAttitudeControl attc;
	Standard standard(&attc);
	setHover(attc, 2.f); // 3 m/s below the setpoint, VT_EF_VEL_ERR is 3.5 m/s

	fly(attc, standard, 300_ms);
	EXPECT_FALSE(attc.is_fixed_wing_system_failure());
}

TEST_F(VtolForwardPropulsionFailureTest, MovingBackwardWhenHoldingPositionIsForwardPropulsionFailure)
{
	VtolAttitudeControl attc;
	Standard standard(&attc);
	setHover(attc, -4.f, 0.f);

	fly(attc, standard, 2_s);
	EXPECT_TRUE(attc.is_fixed_wing_system_failure());
}

TEST_F(VtolForwardPropulsionFailureTest, PusherAssistBelowThresholdIsNoForwardPropulsionFailure)
{
	VtolAttitudeControl attc;
	Standard standard(&attc);
	setHover(attc, 0.f, 5.f, -30.f); // pusher assist 0.35, VT_EF_THROT is 0.4

	fly(attc, standard, 300_ms);
	EXPECT_FALSE(attc.is_fixed_wing_system_failure());
}

// A tiltrotor assists with forward tilt instead of a pusher, which the same detection checks
TEST_F(VtolForwardPropulsionFailureTest, TiltrotorWithoutForwardMotionIsForwardPropulsionFailure)
{
	VtolAttitudeControl attc;
	Tiltrotor tiltrotor(&attc);
	setHover(attc, 0.f);

	fly(attc, tiltrotor, 2_s);
	EXPECT_EQ(quadchuteReason(attc), QuadchuteReason::ForwardPropulsionFailure);
	EXPECT_FLOAT_EQ(tiltrotor.pusher_assist(), 0.f); // the tilt goes back to VT_TILT_MC

	// latched: a front transition command does not start a transition
	commandFrontTransition(attc);
	ASSERT_FALSE(attc.is_quadchute_active());
	tiltrotor.update_vtol_state();
	EXPECT_TRUE(attc.is_fixed_wing_system_failure());
	EXPECT_EQ(tiltrotor.get_mode(), mode::ROTARY_WING);
}

TEST_F(VtolForwardPropulsionFailureTest, TiltrotorTiltAssistBelowThresholdIsNoForwardPropulsionFailure)
{
	// only the tilt above VT_TILT_MC counts: 0.1 + 0.35 of tilt assist is still below VT_EF_THROT (0.4)
	const float tilt_mc = 0.1f;
	param_set(param_find("VT_TILT_MC"), &tilt_mc);
	VtolAttitudeControl attc;
	Tiltrotor tiltrotor(&attc);
	setHover(attc, 0.f, 5.f, -30.f);

	fly(attc, tiltrotor, 300_ms);
	EXPECT_FALSE(attc.is_fixed_wing_system_failure());
}

TEST_F(VtolForwardPropulsionFailureTest, TiltrotorDisablingDetectionAllowsFrontTransitions)
{
	VtolAttitudeControl attc;
	Tiltrotor tiltrotor(&attc);
	setHover(attc, 0.f);
	fly(attc, tiltrotor, 2_s);
	ASSERT_TRUE(attc.is_fixed_wing_system_failure());

	const float ef_time = 0.f;
	param_set(param_find("VT_EF_TIME"), &ef_time);
	static_cast<VtolType &>(tiltrotor).parameters_update();
	commandFrontTransition(attc);
	tiltrotor.update_vtol_state();
	EXPECT_FALSE(attc.is_fixed_wing_system_failure());
	EXPECT_EQ(tiltrotor.get_mode(), mode::TRANSITION_TO_FW);
}
