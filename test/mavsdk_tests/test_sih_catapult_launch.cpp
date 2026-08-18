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
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ****************************************************************************/

#include <chrono>
#include <cmath>

#include "autopilot_tester.h"

namespace
{

float wrap_180(float angle_deg)
{
	while (angle_deg > 180.f) { angle_deg -= 360.f; }

	while (angle_deg < -180.f) { angle_deg += 360.f; }

	return angle_deg;
}

} // namespace

// Catapult launch of a fixed-wing whose heading is only aligned at startup (EKF2_MAG_TYPE = init).
//
// Sitting on the catapult, the heading is unobservable and its uncertainty grows. The catapult then
// accelerates the vehicle harder than the simulated GNSS receiver's dynamic model can follow
// (SIM_GPS_ACC_MAX), so the reported velocity lags the true one and produces a large innovation along
// the launch direction.
//
// That innovation is a pure magnitude error: fusing it must not rotate the heading. It does rotate the
// heading when the sequential axis-by-axis fusion reuses the innovation and innovation variance computed
// before the previous axis was fused.
TEST_CASE("SIH catapult launch does not pull the heading", "[sih_catapult_launch]")
{
	using namespace std::chrono_literals;

	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	// The course over ground is the reference the estimated heading is checked against below.
	tester.set_rate_raw_gps(10.);

	// Wait on the catapult for the heading uncertainty to grow.
	tester.sleep_for(300s);

	const float yaw_on_catapult_deg = tester.get_attitude_euler().yaw_deg;

	// Takeoff mode first, so that launch detection is already running when the catapult fires on arming.
	tester.takeoff();
	tester.arm();

	// The vehicle travels straight along its nose, so the estimated heading has to agree with the
	// course over ground reported by the GNSS receiver. A heading pulled away by the velocity
	// innovation shows up directly as a disagreement between the two.
	float max_heading_error_deg = 0.f;

	for (int i = 0; i < 60; ++i) {
		tester.sleep_for(50ms);

		const Telemetry::RawGps raw_gps = tester.get_raw_gps();

		if (raw_gps.velocity_m_s < 8.f) {
			continue; // course over ground is meaningless at low speed
		}

		const float heading_error_deg = wrap_180(tester.get_attitude_euler().yaw_deg - raw_gps.cog_deg);
		max_heading_error_deg = std::max(max_heading_error_deg, std::fabs(heading_error_deg));
	}

	CAPTURE(yaw_on_catapult_deg);
	CAPTURE(max_heading_error_deg);
	CHECK(max_heading_error_deg > 0.f); // the vehicle was actually launched and picked up speed
	CHECK(max_heading_error_deg < 5.f);
}
