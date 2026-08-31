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

#include "autopilot_tester.h"

// Clipping the accelerometer behind the primary estimator instance mid mission, with a
// second IMU and a second EKF instance available to fall back on. The mission has to run
// to the end and the vehicle has to hold its altitude, measured against the simulator.
//
// The estimate itself is not used as an oracle here. A handover steps it on purpose, so a
// band on the estimate would have to allow that step and would no longer catch anything.
// It asserts that the fault is really present on the clipped IMU, read back from SCALED_IMU,
// that the selector leaves the instance behind it while the mission is still running, read
// back from estimator_selector_status through the MAVLink shell, that the vehicle stayed on
// the mission legs measured against the simulator, and that it held its altitude. The
// selector's own decision rules are covered in src/modules/ekf2/EKF2SelectorTest.cpp, which
// needs no simulator.
// Measured worst deviation through the fault is about 0.7 m, so this bounds the real path with
// margin while still catching a vehicle that wanders off the leg.
static constexpr float kMissionCorridorM = 3.f;

TEST_CASE("EKF instance failover during a mission", "[multicopter_ekf_failover]")
{
	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();

	AutopilotTester::MissionOptions mission_options;
	mission_options.leg_length_m = 30.;
	mission_options.relative_altitude_m = 20.;
	tester.prepare_square_mission(mission_options);

	// Answers the second half of the review request on this PR: that a local estimate jump
	// does not move the vehicle off the mission. Measured against the simulator, because the
	// estimate is the thing the fault disturbs.
	tester.check_tracks_mission_ground_truth(kMissionCorridorM);

	tester.arm();
	tester.execute_mission_and_degrade_primary_imu();
	tester.stop_tracking_mission_ground_truth();
	tester.execute_rtl();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(180);
	tester.wait_until_disarmed(until_disarmed_timeout);
}
