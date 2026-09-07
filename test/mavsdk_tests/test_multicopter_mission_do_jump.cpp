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

#include "autopilot_tester.h"
#include <chrono>

// Regression test for the DO_JUMP "idle setpoint" hang.
//
// Background: an external MISSION_SET_CURRENT sets the navigator's current sequence verbatim, with
// no jump resolution. If it points directly at a DO_JUMP item, the navigator used to load that jump
// as the current item, publish an invalid IDLE position setpoint, and hang there indefinitely
// (observed in a real flight). The fix resolves the jump when loading the current item.
//
// Setup: a multicopter flies a mission that contains a DO_JUMP (item 3, looping back to waypoint 1).
// Mid-mission, we send MISSION_SET_CURRENT pointing at the DO_JUMP index.
//
// Pass: the navigator resolves the jump instead of hanging, and the mission runs to completion,
// landing and disarming within the timeout.
TEST_CASE("Fly Multicopter Mission with external set-current onto a DO_JUMP item", "[multicopter]")
{
	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	AutopilotTester::MissionOptions mission_options;
	mission_options.leg_length_m = 100.0;
	mission_options.relative_altitude_m = 15.0;
	const int jump_index = tester.prepare_multicopter_mission_with_do_jump(mission_options, /*jump_repeats*/ 1);
	tester.arm();

	// Let the mission get underway, then - in the middle of the mission - command the current item to
	// be the DO_JUMP index. This is the exact scenario that used to strand the vehicle on an IDLE
	// setpoint.
	tester.start_mission_raw_and_wait_for_sequence(2);
	tester.send_set_current_mission_item(jump_index);

	// With the bug the vehicle would hang and never land; with the fix the jump resolves and the
	// mission completes normally.
	tester.wait_until_disarmed(std::chrono::seconds(240));
}
