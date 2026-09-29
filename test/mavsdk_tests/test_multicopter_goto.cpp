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

// Integration tests for the multicopter-only Goto mode (NAVIGATION_STATE_GOTO), driven
// through goto_setpoint/GotoControl rather than position_setpoint_triplet.

#include "autopilot_tester_goto.h"

#include <chrono>

using namespace std::chrono_literals;

namespace
{

// src/modules/commander/px4_custom_mode.h (not includable here).
constexpr uint8_t PX4_CUSTOM_MAIN_MODE_AUTO = 4;
constexpr uint8_t PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3;
constexpr uint8_t PX4_CUSTOM_SUB_MODE_GOTO = 21;

constexpr float kTakeoffAltitude = 10.f;

void arm_takeoff_hold(AutopilotTesterGoto &tester)
{
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.set_takeoff_altitude(kTakeoffAltitude);
	tester.sleep_for(1s);
	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.wait_until_altitude(kTakeoffAltitude, 15s, 0.5f);
	tester.sleep_for(2s); // let the takeoff overshoot settle
}

} // namespace

TEST_CASE("Goto: reposition on a multicopter flies to and holds at the target", "[multicopter]")
{
	AutopilotTesterGoto tester;
	arm_takeoff_hold(tester);

	const AutopilotTesterGoto::LocalCoordinate target{50.0, 0.0};
	tester.command_reposition(target, kTakeoffAltitude);

	tester.wait_until_nav_state(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_GOTO, 5s);
	tester.wait_until_reaches(target, 2.f, 60s);
	tester.check_stays_within(target, 2.f, 10s);
}

TEST_CASE("Goto: pausing mid-reposition switches to Hold near the pause point", "[multicopter]")
{
	AutopilotTesterGoto tester;
	arm_takeoff_hold(tester);

	const auto hold_position = tester.current_local_position();

	// Far enough that the vehicle is still well short of it when we pause.
	const AutopilotTesterGoto::LocalCoordinate far_target{200.0, 0.0};
	tester.command_reposition(far_target, kTakeoffAltitude);
	tester.wait_until_nav_state(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_GOTO, 5s);

	tester.sleep_for(3s); // fly toward it, but nowhere near reaching it

	const auto pause_position = tester.current_local_position();
	tester.command_hold_here();
	tester.wait_until_nav_state(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER, 5s);

	// Brakes and holds near the pause point, never flies back toward the Hold point it left.
	const float dist_at_pause = AutopilotTesterGoto::horizontal_distance(pause_position, hold_position);
	tester.check_never_reaches(hold_position, dist_at_pause - 2.f, 10s);
	tester.check_stays_within(pause_position, 15.f, 5s);
}

TEST_CASE("Goto: in-place reposition while already in Hold stays in Hold and flies there", "[multicopter]")
{
	AutopilotTesterGoto tester;
	arm_takeoff_hold(tester);

	tester.wait_until_nav_state(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER, 5s);

	const AutopilotTesterGoto::LocalCoordinate target{30.0, 0.0};
	tester.command_reposition(target, kTakeoffAltitude, false);

	tester.wait_until_reaches(target, 2.f, 60s);
	tester.wait_until_nav_state(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER, 1s);
}

TEST_CASE("Goto: speed change while flying is applied", "[multicopter]")
{
	AutopilotTesterGoto tester;
	arm_takeoff_hold(tester);

	const AutopilotTesterGoto::LocalCoordinate far_target{200.0, 0.0};
	tester.command_reposition(far_target, kTakeoffAltitude);
	tester.wait_until_nav_state(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_GOTO, 5s);
	tester.sleep_for(5s); // accelerate to the default cruise speed

	// DO_CHANGE_SPEED: stays in Goto and slows down to the new limit
	constexpr float kSlowSpeed = 2.f;
	tester.change_speed(kSlowSpeed);
	tester.sleep_for(4s); // decelerate
	tester.check_max_horizontal_speed(kSlowSpeed + 0.5f, 5s);
	tester.wait_until_nav_state(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_GOTO, 1s);
}
