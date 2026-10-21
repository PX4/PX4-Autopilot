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

#include "autopilot_tester_vision_target.h"

using namespace std::chrono_literals;
using Scenario = AutopilotTesterVisionTarget::Scenario;

static void land_on_static_target(Scenario scenario)
{
	AutopilotTesterVisionTarget tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	// The pad stays fixed on the ground, well outside the acceptance radius of a normal
	// landing at takeoff. Only the simulated target sensor tells PX4 where it is.
	const CoordinateTransformation::LocalCoordinate target_offset_m{5.0, 3.0};
	constexpr float takeoff_altitude_m = 12.0f;
	constexpr float landing_acceptance_radius_m = 0.5f;
	tester.set_landing_target(target_offset_m);
	tester.set_takeoff_altitude(takeoff_altitude_m);
	tester.arm();
	tester.takeoff();
	tester.wait_until_altitude(takeoff_altitude_m, 30s);

	tester.precision_land(scenario, 90s);
	tester.check_landed_at_target(landing_acceptance_radius_m);
}

TEST_CASE("Precision land with TARGET_RELATIVE", "[vision_target]")
{
	land_on_static_target(Scenario::Vision);
}

TEST_CASE("Precision land with TARGET_ABSOLUTE and GNSS velocity", "[vision_target]")
{
	land_on_static_target(Scenario::Gnss);
}

TEST_CASE("Precision land with both target messages", "[vision_target]")
{
	land_on_static_target(Scenario::VisionAndGnss);
}

TEST_CASE("Precision land from a mission corrected by vision", "[vision_target]")
{
	land_on_static_target(Scenario::Mission);
}

TEST_CASE("Precision land rejects a target GNSS jump", "[vision_target]")
{
	land_on_static_target(Scenario::GnssJump);
}

TEST_CASE("Precision land retains GNSS bias after vision loss", "[vision_target]")
{
	land_on_static_target(Scenario::VisionLoss);
}
