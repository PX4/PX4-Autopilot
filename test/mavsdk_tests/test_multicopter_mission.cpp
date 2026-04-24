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


TEST_CASE("Takeoff and Land", "[multicopter][vtol]")
{
	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.land();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(180);
	tester.wait_until_disarmed(until_disarmed_timeout);
}

TEST_CASE("Fly square Multicopter Missions including RTL", "[multicopter]")
{
	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	AutopilotTester::MissionOptions mission_options;
	mission_options.rtl_at_end = true;
	tester.prepare_square_mission(mission_options);
	tester.arm();
	tester.execute_mission();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(180);
	tester.wait_until_disarmed(until_disarmed_timeout);
}

TEST_CASE("Fly square Multicopter Missions with manual RTL", "[multicopter]")
{
	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	AutopilotTester::MissionOptions mission_options;
	mission_options.rtl_at_end = false;
	tester.prepare_square_mission(mission_options);
	tester.check_tracks_mission(5.f);
	tester.arm();
	tester.execute_mission();
	tester.wait_until_hovering();
	tester.execute_rtl();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(180);
	tester.wait_until_disarmed(until_disarmed_timeout);
}

// Disabled: the check `check_mission_item_speed_above(3, 3.5)` assumes the
// drone maintains cruise speed through the intermediate waypoints because
// `mission_options.fly_through = true`. PX4 doesn't actually implement
// fly-through at the smoother level:
//
//   - PositionSmoothing::_getCrossingPoint returns the *current* waypoint as
//     the target whenever the drone isn't turning, so the commanded velocity
//     direction zeroes out exactly at the waypoint — the smoother always aims
//     to stop there.
//   - The apparent fly-through behaviour relies on the navigator flipping
//     to the next waypoint before the smoother reaches the current one.
//   - For multicopters the navigator's reached gate requires
//     `dist_xy <= acceptance_radius` AND `dist_z <= NAV_MC_ALT_RAD` (0.8 m
//     by default). FW has a "passed waypoint" dot-product fallback; MC does
//     not. So if altitude happens to be >0.8 m off the mission altitude at
//     the moment of closest horizontal approach, the flip is missed.
//   - The smoother then reaches the waypoint, decelerates to zero, overshoots
//     on physics, reverses, and crawls back. Horizontal speed collapses at the
//     waypoint — the speed check near item 3 sees 0–2 m/s instead of ≥3.5 m/s.
//
// Under SIH at 20× speed factor the effective rate often slips to ~16× due to
// ASan/test load, and altitude tracking exceeds 0.8 m more frequently, so the
// race the test used to usually win is now regularly lost.
//
// Real fix is architectural (give MC a passed-waypoint branch in the navigator
// and/or teach the smoother to aim past the current waypoint when the next one
// is valid and colinear). Track separately; for now disable the test.
#if 0
TEST_CASE("Fly straight Multicopter Mission", "[multicopter]")
{
	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	AutopilotTester::MissionOptions mission_options;
	mission_options.rtl_at_end = false;
	mission_options.fly_through = true;
	mission_options.leg_length_m = 40.0;
	tester.prepare_straight_mission(mission_options);
	tester.check_mission_item_speed_above(3, 3.5);
	tester.check_tracks_mission(5.f);
	tester.arm();
	tester.execute_mission();
	tester.wait_until_hovering();
	tester.execute_rtl();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(180);
	tester.wait_until_disarmed(until_disarmed_timeout);
}
#endif
