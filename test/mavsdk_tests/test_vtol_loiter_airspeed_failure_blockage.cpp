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

#include "autopilot_tester.h"
#include "autopilot_tester_failure.h"


TEST_CASE("Fly VTOL Loiter with airspeed failure", "[vtol_airspeed_fail]")
{
	AutopilotTesterFailure tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	tester.enable_fixedwing_mectrics();

	// configuration
	const float takeoff_altitude = 10.f;
	tester.set_takeoff_altitude(takeoff_altitude);

	tester.load_qgc_mission_raw_and_move_here("test/mavsdk_tests/vtol_mission.plan");
	tester.arm();

	tester.takeoff();
	tester.wait_until_altitude(takeoff_altitude, std::chrono::seconds(30));
	tester.transition_to_fixedwing();


	// tester.wait_until_altitude(50.f, std::chrono::seconds(30));
	std::this_thread::sleep_for(std::chrono::seconds(10));
	tester.inject_failure(mavsdk::Failure::FailureUnit::SensorAirspeed, mavsdk::Failure::FailureType::Wrong, 0,
			      mavsdk::Failure::Result::Success);


	std::this_thread::sleep_for(std::chrono::seconds(10));

	tester.check_airspeed_is_invalid(); // it's enough to check once after landing, as invalidation is permanent
}
