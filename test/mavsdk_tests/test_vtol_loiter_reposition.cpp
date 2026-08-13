/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

// Integration tests for fixed-wing loiter / reposition / Hold interaction.
//
// These exercise how a Hold (MAV_CMD_DO_REPOSITION pause) and RTL interact with an already
// established loiter:
//   1. RTL while established on a non-default-radius loiter climbs on that same loiter.
//   2. A Hold (all-NaN reposition) while established keeps the loiter but holds the current altitude.
//   3. A Hold while still transiting toward a loiter creates a fresh loiter at the current position.
//   4. A Hold while flying a figure-eight reverts to a plain circular loiter (patterns are not mixed).
//   5. An altitude-only reposition keeps the loiter center and radius and only changes altitude.

#include "autopilot_tester_loiter.h"

#include <chrono>

using namespace std::chrono_literals;

namespace
{

// Home-relative center used for the "work" loiter. Placed well away from home so that RTL climbs to
// the full return altitude (close to home the RTL cone would cap the climb).
const AutopilotTesterLoiter::LocalCoordinate kWorkCenter{250.0, 0.0};

constexpr float kTakeoffAltitude = 40.f;
constexpr float kLoiterAltitude = 40.f;

// Clearly different from the default NAV_LOITER_RAD (80 m) so a default-radius circle would fall
// outside the tolerance band.
constexpr float kNonDefaultRadius = 120.f;
constexpr float kRadiusTolerance = 30.f;
constexpr float kAltitudeTolerance = 8.f;

void arm_takeoff_transition_hold(AutopilotTesterLoiter &tester)
{
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.set_takeoff_altitude(kTakeoffAltitude);
	tester.sleep_for(3s);
	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.wait_until_altitude(kTakeoffAltitude, 30s);
	tester.transition_to_fixedwing();
	tester.wait_until_fixedwing(5s);
	tester.sleep_for(1s);
	tester.enter_hold_and_wait();
}

void establish_work_orbit(AutopilotTesterLoiter &tester)
{
	tester.command_orbit(kWorkCenter, kNonDefaultRadius, kLoiterAltitude);
	tester.wait_until_on_loiter(kWorkCenter, kNonDefaultRadius, kRadiusTolerance, 12s, 180s);
}

} // namespace

TEST_CASE("Loiter: RTL climbs on an established non-default-radius loiter", "[loiter]")
{
	AutopilotTesterLoiter tester;
	arm_takeoff_transition_hold(tester);
	establish_work_orbit(tester);

	// Climb clearly above the loiter altitude when RTL is engaged.
	const float rtl_altitude = 80.f;
	tester.set_rtl_altitude(rtl_altitude);
	tester.execute_rtl();

	// RTL must climb to the return altitude while continuing to circle the very same loiter
	// (same center and non-default radius), not re-center a new circle.
	tester.check_climbs_on_loiter(kWorkCenter, kNonDefaultRadius, kRadiusTolerance, rtl_altitude,
				      kAltitudeTolerance, 180s);
}

TEST_CASE("Loiter: Hold during an altitude change keeps the loiter and locks the current altitude", "[loiter]")
{
	AutopilotTesterLoiter tester;
	arm_takeoff_transition_hold(tester);
	establish_work_orbit(tester); // established at kLoiterAltitude

	// Command the very same loiter (same center and radius) but at a higher altitude, so the vehicle
	// starts climbing while staying on the loiter.
	const float climb_target = 200.f;
	tester.command_orbit(kWorkCenter, kNonDefaultRadius, climb_target);

	// Catch the vehicle part way through the climb and issue an all-NaN Hold reposition.
	const float intercept_altitude = 60.f; // between kLoiterAltitude and climb_target
	tester.wait_until_relative_altitude_above(intercept_altitude, 120s);
	tester.command_hold_here();
	tester.sleep_for(2s); // let the Hold take effect and the climb arrest

	const float locked_altitude = tester.current_relative_altitude();

	// The Hold must arrest the climb well before reaching the commanded target altitude ...
	CHECK(locked_altitude < climb_target - kAltitudeTolerance);
	// ... lock the altitude the vehicle had reached (no further climb) ...
	tester.check_holds_altitude(locked_altitude, kAltitudeTolerance, 40s);
	// ... and keep circling the same loiter (same center and radius).
	tester.check_stays_on_loiter(kWorkCenter, kNonDefaultRadius, kRadiusTolerance, 40s);
}

TEST_CASE("Loiter: Hold while transiting creates a new loiter at the current position", "[loiter]")
{
	AutopilotTesterLoiter tester;
	arm_takeoff_transition_hold(tester);

	// Command a loiter far away and Hold before getting anywhere near it.
	const AutopilotTesterLoiter::LocalCoordinate far_center{600.0, 0.0};
	tester.command_reposition(far_center, kLoiterAltitude);
	tester.sleep_for(6s); // fly toward it, but nowhere near established

	const auto hold_position = tester.current_local_position();
	tester.command_hold_here();
	tester.sleep_for(10s); // let the fresh loiter circle be captured before asserting on it

	const float default_radius = tester.default_loiter_radius();

	// A fresh loiter must be created around where Hold was pressed: once captured the vehicle stays
	// within one loiter radius (plus the usual tracking band) of that point ...
	tester.check_stays_within(hold_position, default_radius + kRadiusTolerance, 40s);
	// ... and the vehicle must not keep flying toward the previously commanded far loiter.
	tester.check_never_reaches(far_center, 150.f, 40s);
}

TEST_CASE("Loiter: Hold on a figure-eight reverts to a plain circular loiter", "[loiter]")
{
	AutopilotTesterLoiter tester;
	arm_takeoff_transition_hold(tester);

	// Get onto a figure-eight (major 150 m, minor 50 m) around the work center.
	tester.command_figure_eight(kWorkCenter, 150.f, 50.f, kLoiterAltitude);
	tester.sleep_for(35s); // give it time to actually track the pattern

	const auto hold_position = tester.current_local_position();
	tester.command_hold_here();
	tester.sleep_for(2s);

	const float default_radius = tester.default_loiter_radius();

	// After Hold the vehicle must settle into a single circular loiter around the Hold position.
	// If it kept flying the figure-eight it would repeatedly swing out to the ~150 m lobes and
	// leave this bound; a plain circle stays close to its single center.
	tester.check_stays_within(hold_position, default_radius + 70.f, 40s);
}

TEST_CASE("Loiter: altitude-only reposition keeps the loiter and only changes altitude", "[loiter]")
{
	AutopilotTesterLoiter tester;
	arm_takeoff_transition_hold(tester);
	establish_work_orbit(tester);

	// Request only an altitude change (lat/lon NaN). The loiter center and non-default radius must
	// be preserved while the vehicle climbs to the new altitude.
	const float new_altitude = 70.f;
	tester.command_reposition_altitude(new_altitude);
	tester.check_climbs_on_loiter(kWorkCenter, kNonDefaultRadius, kRadiusTolerance, new_altitude,
				      kAltitudeTolerance, 120s);
}
