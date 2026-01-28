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

static constexpr float acceptance_radius = 0.3f;

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
	tester.check_home_within(2.0f);
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
	std::chrono::seconds goto_timeout = std::chrono::seconds(10);
	tester.offboard_goto(takeoff_position, acceptance_radius, goto_timeout);
	tester.offboard_goto(setpoint_1, acceptance_radius, goto_timeout);
	tester.offboard_goto(setpoint_2, acceptance_radius, goto_timeout);
	tester.offboard_goto(setpoint_3, acceptance_radius, goto_timeout);
	tester.offboard_goto(takeoff_position, acceptance_radius, goto_timeout);
	tester.offboard_land();
	tester.wait_until_disarmed(std::chrono::seconds(120));
	tester.check_home_within(2.0f);
}

TEST_CASE("Offboard attitude control", "[multicopter][offboard_attitude]")
{
	AutopilotTester tester;
	tester.connect(connection_url);
	tester.set_rc_loss_exception(AutopilotTester::RcLossException::Offboard);
	tester.fly_forward_in_offboard_attitude();
	tester.wait_until_disarmed(std::chrono::seconds(120));
}
