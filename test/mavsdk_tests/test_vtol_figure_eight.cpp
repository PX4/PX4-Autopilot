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

#include "autopilot_tester_figure_eight.h"

#include <chrono>
#include <thread>

TEST_CASE("Figure eight execution clockwise", "[vtol]")
{
	AutopilotTesterFigureEight tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	const float takeoff_altitude = 20.f;
	tester.set_takeoff_altitude(takeoff_altitude);
	tester.sleep_for(std::chrono::seconds(3));
	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.wait_until_altitude(takeoff_altitude, std::chrono::seconds(30));
	tester.transition_to_fixedwing();
	tester.wait_until_fixedwing(std::chrono::seconds(5));
	tester.sleep_for(std::chrono::seconds(1));
	tester.set_figure_eight(150., 50., 0., 200., 0., 20.);
	tester.execute_figure_eight();
	tester.check_tracks_figure_eight(std::chrono::seconds(120), 10.);
	// tester.check_receive_execution_status(std::chrono::seconds(
	// 		5)); //TODO With mavsdk we can't subscribe to custom messages. Need to wait until messages are recognised by mavsdk
}

TEST_CASE("Figure eight execution counterclockwise", "[vtol]")
{
	AutopilotTesterFigureEight tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	const float takeoff_altitude = 20.f;
	tester.set_takeoff_altitude(takeoff_altitude);
	tester.sleep_for(std::chrono::seconds(3));
	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.wait_until_altitude(takeoff_altitude, std::chrono::seconds(30));
	tester.transition_to_fixedwing();
	tester.wait_until_fixedwing(std::chrono::seconds(5));
	tester.sleep_for(std::chrono::seconds(1));
	tester.set_figure_eight(-150., 50., 30.*M_PI / 180., 200., 0., 20.);
	tester.execute_figure_eight();
	tester.check_tracks_figure_eight(std::chrono::seconds(120), 10.);
	// tester.check_receive_execution_status(std::chrono::seconds(
	// 		5)); //TODO With mavsdk we can't subscribe to custom messages. Need to wait until messages are recognised by mavsdk.
}
