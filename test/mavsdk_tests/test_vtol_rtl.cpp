/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

TEST_CASE("RTL direct Home", "[vtol]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.load_qgc_mission_raw_and_move_here("test/mavsdk_tests/vtol_mission_with_land_start.plan");
	// fly directly to home position
	tester.set_rtl_type(0);
	tester.set_rtl_appr_force(0);
	tester.arm();
	tester.execute_rtl_when_reaching_mission_sequence(2);
	tester.wait_until_disarmed(std::chrono::seconds(120));
	tester.check_home_within(5.0f);
}

TEST_CASE("RTL direct Mission Land", "[vtol]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.load_qgc_mission_raw_and_move_here("test/mavsdk_tests/vtol_mission_with_land_start.plan");
	// Do not allow home
	tester.set_rtl_type(1);
	tester.arm();
	tester.execute_rtl_when_reaching_mission_sequence(2);
	tester.wait_until_disarmed(std::chrono::seconds(120));
	tester.check_mission_land_within(5.0f);
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
	tester.check_tracks_mission_raw(40.0f);
	tester.wait_until_disarmed(std::chrono::seconds(120));
}

TEST_CASE("RTL with Reverse Mission", "[vtol]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.set_takeoff_land_requirements(0);
	tester.load_qgc_mission_raw_and_move_here("test/mavsdk_tests/vtol_mission_without_landing.plan");
	// vehicle should follow the mission in reverse and land at the home position // TODO enable checks again if MAVSDK can handle mission in reverse order
	tester.set_rtl_type(2);
	tester.arm();
	tester.execute_rtl_when_reaching_mission_sequence(6);
	//tester.check_tracks_mission_raw(35.0f);
	tester.wait_until_disarmed(std::chrono::seconds(120));
}

TEST_CASE("RTL direct home without approaches", "[vtol]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.load_qgc_mission_raw_and_move_here("test/mavsdk_tests/vtol_mission_with_land_start.plan");
	// fly directly to home position
	tester.set_rtl_type(0);
	tester.set_rtl_appr_force(0);
	// reupload rally points with approaches
	tester.add_home_to_rally_point();
	tester.upload_custom_mission(std::chrono::seconds(10));
	tester.arm();
	tester.execute_rtl_when_reaching_mission_sequence(4);
	tester.wait_until_disarmed(std::chrono::seconds(150));
	tester.check_home_within(5.0f);
}

TEST_CASE("RTL direct home without approaches forced", "[vtol]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.load_qgc_mission_raw_and_move_here("test/mavsdk_tests/vtol_mission_with_land_start.plan");
	// fly directly to home position
	tester.set_rtl_type(0);
	tester.set_rtl_appr_force(1);
	// reupload rally points with approaches
	tester.add_home_to_rally_point();
	tester.upload_custom_mission(std::chrono::seconds(10));
	tester.arm();
	tester.execute_rtl_when_reaching_mission_sequence(4);
	tester.wait_until_disarmed(std::chrono::seconds(150));
	tester.check_mission_land_within(5.f);
}

TEST_CASE("RTL direct home with approaches", "[vtol]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.load_qgc_mission_raw_and_move_here("test/mavsdk_tests/vtol_mission_with_land_start.plan");
	// fly directly to home position
	tester.set_rtl_type(0);
	// reupload rally points with approaches
	tester.add_home_with_approaches_to_rally_point();
	tester.upload_custom_mission(std::chrono::seconds(10));
	tester.arm();
	tester.execute_rtl_when_reaching_mission_sequence(4);
	tester.check_rtl_approaches(5., std::chrono::seconds(60));
	tester.wait_until_disarmed(std::chrono::seconds(150));
	tester.check_home_within(5.0f);
}

TEST_CASE("RTL direct home not as rally point", "[vtol]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.load_qgc_mission_raw_and_move_here("test/mavsdk_tests/vtol_mission_with_land_start.plan");
	// fly directly to home position
	tester.set_rtl_type(1);
	// reupload rally points with approaches
	tester.add_home_with_approaches_to_rally_point();
	tester.upload_custom_mission(std::chrono::seconds(10));
	tester.arm();
	tester.execute_rtl_when_reaching_mission_sequence(2);
	tester.wait_until_disarmed(std::chrono::seconds(150));
	tester.check_mission_land_within(5.0f);
}

TEST_CASE("RTL direct rally without approaches", "[vtol]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.load_qgc_mission_raw_and_move_here("test/mavsdk_tests/vtol_mission_with_land_start.plan");
	// Do not allow home position
	tester.set_rtl_type(1);
	tester.set_rtl_appr_force(0);
	// reupload rally points with approaches
	tester.add_local_rally_point({100., -200.});
	tester.upload_custom_mission(std::chrono::seconds(10));
	tester.arm();
	tester.execute_rtl_when_reaching_mission_sequence(3);
	tester.wait_until_disarmed(std::chrono::seconds(150));
	tester.check_rally_point_within(5.0f);
	tester.check_home_not_within(20.);
}

TEST_CASE("RTL direct rally without approaches forced", "[vtol]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.load_qgc_mission_raw_and_move_here("test/mavsdk_tests/vtol_mission_with_land_start.plan");
	// Do not allow home position
	tester.set_rtl_type(1);
	tester.set_rtl_appr_force(1);
	// reupload rally points with approaches
	tester.add_local_rally_point({100., -2000.});
	tester.upload_custom_mission(std::chrono::seconds(10));
	tester.arm();
	tester.execute_rtl_when_reaching_mission_sequence(3);
	tester.wait_until_disarmed(std::chrono::seconds(150));
	tester.check_mission_land_within(5.f);
}

TEST_CASE("RTL direct rally with approaches", "[vtol]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.load_qgc_mission_raw_and_move_here("test/mavsdk_tests/vtol_mission_with_land_start.plan");
	// Do not allow home position
	tester.set_rtl_type(1);
	tester.set_rtl_appr_force(0);
	// reupload rally points with approaches
	tester.add_local_rally_with_approaches_point({100., -200.});
	tester.upload_custom_mission(std::chrono::seconds(10));
	tester.arm();
	tester.execute_rtl_when_reaching_mission_sequence(3);
	tester.check_rtl_approaches(5., std::chrono::seconds(60));
	tester.wait_until_disarmed(std::chrono::seconds(150));
	tester.check_rally_point_within(5.0f);
	tester.check_home_not_within(20.);
}
