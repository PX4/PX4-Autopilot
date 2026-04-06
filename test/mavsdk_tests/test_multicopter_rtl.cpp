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

#include "autopilot_tester_rtl.h"


// TODO: Add test cases for these scenarios:
// - "RTL type 6 home within reach with no rally points", "[multicopter]"
// - "RTL type 6 home within reach with rally points and home is the closest", "[multicopter]"
// - "RTL type 6 home within reach with rally points and home is not the closest", "[multicopter]"
// - "RTL type 6 home out of reach with rally points and at least one rally point within reach", "[multicopter]"

TEST_CASE("RTL_TYPE=6 time_remaining_s NaN with no rally points", "[multicopter]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.set_rtl_type(6);
	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.execute_rtl();
	tester.wait_until_disarmed(std::chrono::seconds(90));
	tester.check_home_within(5.0f);
}

TEST_CASE("RTL_TYPE=6 time_remaining_s NaN with rally points and home is closest", "[multicopter]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.set_rtl_type(6);
	// Rally is farther than home; vehicle is above home after takeoff so home wins on distance
	tester.add_local_rally_point({100., -100.});
	tester.upload_rally_points();
	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.execute_rtl();
	tester.wait_until_disarmed(std::chrono::seconds(90));
	tester.check_home_within(5.0f);
}

TEST_CASE("RTL_TYPE=6 time_remaining_s NaN with rally points and home is not closest", "[multicopter]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.set_rtl_type(6);
	// Rally point is between the vehicle and home, so rally point wins on distance
	tester.add_local_rally_point({300., 0.});
	tester.upload_rally_points();
	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	// Distance to home is 400m, distance to rally is 100m
	tester.offboard_goto({400.f, 0.f, -20.f}, 5.f, std::chrono::seconds(60));
	tester.execute_rtl();
	tester.wait_until_disarmed(std::chrono::seconds(150));
	tester.check_rally_point_within(5.0f);
	tester.check_home_not_within(20.0f);
}
