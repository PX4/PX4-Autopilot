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

// Exercises the RTL_DIRECT geofence avoidance planner (visibility graph + Dijkstra).
//
// Setup: mission flies the vehicle to a point from which the straight-line return-to-home would
// cut through an exclusion polygon. The planner must detour around it.
//
// Pass: the vehicle returns to home and disarms within the timeout AND its ground-truth position
// never violates any of the loaded geofences during the entire armed flight.
TEST_CASE("RTL direct with geofence obstruction", "[multicopter]")
{
	AutopilotTesterRtl tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();

	// Mission outbound leg ends at a point where straight-line RTL crosses the exclusion polygon
	// in the .plan; the planner must detour around it.
	tester.load_qgc_mission_and_geofence_here("test/mavsdk_tests/multicopter_mission_geofence_avoid.plan");

	// Warning only: the planner still picks up the fence (it reads polygon geometry independent of
	// GF_ACTION), but a hypothetical breach won't trigger Hold/RTL failsafe and mask test failure.
	tester.set_param_int("GF_ACTION", 1);

	tester.set_rtl_type(0); // RTL_DIRECT
	tester.set_rtl_appr_force(0);

	tester.sleep_for(std::chrono::seconds(3));

	tester.arm();

	// Latch a breach detector covering everything from here until disarm.
	tester.start_monitoring_geofence_breach();
	tester.execute_rtl_when_reaching_mission_sequence(3);

	tester.wait_until_disarmed(std::chrono::seconds(400));

	tester.check_no_geofence_breach();
	tester.check_home_within(5.0f);
}
