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

#include <chrono>
#include <cmath>

#include "autopilot_tester.h"

// Reproduces the stale home-yaw bug: when home is first set from a raw-GPS fix while the local position
// estimate is invalid, home.yaw must still end up at the true ground heading.
TEST_CASE("SIH: home yaw captured when local position invalid", "[sih_home_yaw]")
{
	using namespace std::chrono_literals;

	AutopilotTester tester;
	tester.connect(connection_url);

	// No GPS fix yet: wait for the magnetometer heading to align and settle, so that no later heading
	// reset can refresh home.yaw and mask the bug.
	tester.sleep_for(10s);
	const float ground_yaw_deg = tester.get_attitude_euler().yaw_deg;

	// Degraded GPS: good enough for the home position, too few sats for EKF fusion -> local position
	// stays invalid. This is the exact condition under which the buggy code leaves home.yaw at 0.
	tester.set_param_int("SIM_GPS_USED", 5);
	tester.sleep_for(6s);

	const mavlink_home_position_t home = tester.get_home_position();
	// Copy out of the packed message struct before forming a float* for the conversion helper.
	const float q[4] = {home.q[0], home.q[1], home.q[2], home.q[3]};
	float roll_rad, pitch_rad, home_yaw_rad;
	mavlink_quaternion_to_euler(q, &roll_rad, &pitch_rad, &home_yaw_rad);
	const float home_yaw_deg = home_yaw_rad * 180.f / static_cast<float>(M_PI);

	float diff_deg = std::fabs(ground_yaw_deg - home_yaw_deg);

	if (diff_deg > 180.f) {
		diff_deg = 360.f - diff_deg;
	}

	CAPTURE(ground_yaw_deg);
	CAPTURE(home_yaw_deg);
	// With the fix home.yaw tracks the true ground heading; with the bug it stays at 0 (North).
	CHECK(diff_deg < 20.f);
}
