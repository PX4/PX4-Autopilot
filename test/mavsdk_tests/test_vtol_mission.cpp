/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "autopilot_tester_rtl.h"


TEST_CASE("Fly VTOL mission", "[vtol]")
{
	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.load_qgc_mission_raw_and_move_here("test/mavsdk_tests/vtol_mission.plan");
	tester.arm();
	tester.execute_mission_raw();
	tester.wait_until_disarmed();
}

TEST_CASE("RTL direct Home", "[vtol]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.load_qgc_mission_raw_and_move_here("test/mavsdk_tests/vtol_mission.plan");
	// fly directly to home position
	tester.set_rtl_type(0);
	tester.arm();
	tester.execute_rtl_when_reaching_mission_sequence(2);
	tester.wait_until_disarmed(std::chrono::seconds(90));
	tester.check_home_within(5.0f);
}

TEST_CASE("RTL with Mission Landing", "[vtol]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.load_qgc_mission_raw_and_move_here("test/mavsdk_tests/vtol_mission.plan");
	// Vehicle should follow the mission and use the mission landing
	tester.set_rtl_type(2);
	tester.arm();
	tester.execute_rtl_when_reaching_mission_sequence(2);
	tester.check_tracks_mission_raw(30.0f);
	tester.wait_until_disarmed(std::chrono::seconds(90));
}

TEST_CASE("RTL with Reverse Mission", "[vtol]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.set_takeoff_land_requirements(0);
	tester.load_qgc_mission_raw_and_move_here("test/mavsdk_tests/vtol_mission_without_landing.plan");
	// vehicle should follow the mission in reverse and land at the home position
	tester.set_rtl_type(2);
	tester.arm();
	tester.execute_rtl_when_reaching_mission_sequence(6);
	tester.check_tracks_mission_raw(30.0f);
	tester.wait_until_disarmed(std::chrono::seconds(90));
}
