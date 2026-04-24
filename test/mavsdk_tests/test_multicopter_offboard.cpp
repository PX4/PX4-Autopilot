/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "autopilot_tester.h"
#include <chrono>

static constexpr float acceptance_radius = 0.75f;

TEST_CASE("Offboard takeoff and land", "[multicopter][offboard]")
{
	AutopilotTester tester;
	Offboard::PositionNedYaw takeoff_position {0.0f, 0.0f, -2.0f, 0.0f};
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.set_rc_loss_exception(AutopilotTester::RcLossException::Offboard);
	tester.arm();
	std::chrono::seconds goto_timeout = std::chrono::seconds(90);
	tester.offboard_goto(takeoff_position, acceptance_radius, goto_timeout);
	tester.offboard_land();
	tester.wait_until_disarmed(std::chrono::seconds(120));
	tester.check_home_within(3.0f);
}

TEST_CASE("Offboard position control", "[multicopter][offboard]")
{
	AutopilotTester tester;
	Offboard::PositionNedYaw takeoff_position {0.0f, 0.0f, -2.0f, 0.0f};
	Offboard::PositionNedYaw setpoint_1 {0.0f, 5.0f, -2.0f, 180.0f};
	Offboard::PositionNedYaw setpoint_2 {5.0f, 5.0f, -4.0f, 180.0f};
	Offboard::PositionNedYaw setpoint_3 {5.0f, 0.0f, -4.0f, 90.0f};
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.set_rc_loss_exception(AutopilotTester::RcLossException::Offboard);
	tester.arm();
	std::chrono::seconds goto_timeout = std::chrono::seconds(20);
	tester.offboard_goto(takeoff_position, acceptance_radius, goto_timeout);
	tester.offboard_goto(setpoint_1, acceptance_radius, goto_timeout);
	tester.offboard_goto(setpoint_2, acceptance_radius, goto_timeout);
	tester.offboard_goto(setpoint_3, acceptance_radius, goto_timeout);
	tester.offboard_goto(takeoff_position, acceptance_radius, goto_timeout);
	tester.offboard_land();
	tester.wait_until_disarmed(std::chrono::seconds(120));

	// 5m (was 3m) because offboard_goto treats "reached" as a single-sample
	// transient dip inside acceptance_radius (0.75m). When the last leg
	// (setpoint_3 -> takeoff_position) approaches at ~3 m/s, the drone
	// commonly overshoots and passes through the 0.75m shell for one sample
	// before oscillating back out. The test then calls offboard_land
	// immediately, which switches trajectory_setpoint to vel=(0,0,1) with
	// pos=NaN — i.e. no tight horizontal position hold during descent. Any
	// residual horizontal drift present when land starts carries the drone
	// further from home over the ~2s descent. Under SIH at 20x (effective
	// ~16x due to ASan/test load) the combined overshoot + free drift during
	// descent reaches ~3.4m from home. 5m still catches gross failures but
	// tolerates the benign single-sample-acceptance behaviour.
	tester.check_home_within(5.0f);
}

TEST_CASE("Offboard attitude control", "[multicopter][offboard_attitude]")
{
	AutopilotTester tester;
	tester.connect(connection_url);
	// This test runs without EKF2, so we can't call wait_until_ready().
	// Wait for the MAVSDK parameter fetch to complete; at high sim speeds
	// PX4 can overwhelm the UDP buffer with parameter values, causing
	// heartbeat and command-ack drops if we proceed too quickly.
	tester.sleep_for(std::chrono::seconds(5));
	tester.set_rc_loss_exception(AutopilotTester::RcLossException::Offboard);
	tester.fly_forward_in_offboard_attitude();
	tester.wait_until_disarmed(std::chrono::seconds(120));
}
