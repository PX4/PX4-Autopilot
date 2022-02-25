/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include <chrono>

#include "autopilot_tester_follow_me.h"



TEST_CASE("Follow target - Position streaming", "[multicopter]")
{
	const bool stream_velocity = false;
	AutopilotTesterFollowMe tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.arm();

	const float takeoff_altitude = 10.0;
	tester.set_takeoff_altitude(takeoff_altitude);
	tester.takeoff();
	tester.wait_until_hovering();

	tester.straight_line_test(stream_velocity);

	tester.land();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(300);
	tester.wait_until_disarmed(until_disarmed_timeout);
}

TEST_CASE("Follow target - Position and velocity streaming", "[multicopter]")
{
	const bool stream_velocity = true;
	AutopilotTesterFollowMe tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.arm();

	const float takeoff_altitude = 10.0;
	tester.set_takeoff_altitude(takeoff_altitude);
	tester.takeoff();
	tester.wait_until_hovering();

	tester.straight_line_test(stream_velocity);

	tester.land();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(300);
	tester.wait_until_disarmed(until_disarmed_timeout);
}

TEST_CASE("Follow target - Velocity streaming only", "[multicopter]")
{
	// Streaming only velocity should not work, the drone should not move.
	// There needs to be at least one position message for follow-me
	// to be able to integrate velocity.
	AutopilotTesterFollowMe tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.arm();

	const float takeoff_altitude = 10.0;
	tester.set_takeoff_altitude(takeoff_altitude);
	tester.takeoff();
	tester.wait_until_hovering();

	tester.stream_velocity_only();

	tester.land();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(300);
	tester.wait_until_disarmed(until_disarmed_timeout);
	tester.check_home_within(1.0f);
}

TEST_CASE("Follow target - RC Adjustment", "[multicopter]")
{
	AutopilotTesterFollowMe tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.arm();

	const float takeoff_altitude = 10.0;
	tester.set_takeoff_altitude(takeoff_altitude);
	tester.takeoff();
	tester.wait_until_hovering();

	tester.rc_adjustment_test();

	tester.land();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(300);
	tester.wait_until_disarmed(until_disarmed_timeout);
}
