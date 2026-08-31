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

/**
 * @file test_ekf2_selector.cpp
 *
 * A hovering vehicle with two IMUs and two EKF instances, where accelerometer clipping
 * moves from one IMU to the other and is then cleared.
 *
 * What this asserts: the clipping really lands on the IMU that SIH_FAULT_IMU names and on
 * no other, read back from SCALED_IMU and SCALED_IMU2, the selector leaves the instance
 * behind the clipped IMU each time, read back from estimator_selector_status through the
 * MAVLink shell, and the vehicle neither drops out of the sky nor stays displaced: it
 * may climb while the clipped instance is still primary, it must not descend more than
 * the floor below the hover, and it must be back at the hover altitude once the selector
 * has handed over, all measured against the simulator.
 *
 * The selector's own decision rules are covered directly in
 * src/modules/ekf2/EKF2SelectorTest.cpp, which needs no simulator. This test covers the
 * part that unit test cannot, which is that a real vehicle with a real estimator and a real
 * controller stays in the air while the faults move.
 *
 * Requires SIH_IMU_COUNT 2 for the second IMU, EKF2_MULTI_IMU 2 for the second estimator
 * instance, and per-IMU fault injection through SIH_FAULT_IMU and SIH_FAULT_VIBE. The test
 * config sets all three.
 */

#include "autopilot_tester.h"

TEST_CASE("EKF2 selector - vehicle survives alternating IMU faults", "[ekf2_selector]")
{
	const float takeoff_altitude = 20.f;

	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	tester.set_takeoff_altitude(takeoff_altitude);
	tester.store_home();
	tester.sleep_for(std::chrono::seconds(1));

	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.wait_until_altitude(takeoff_altitude, std::chrono::seconds(30));
	tester.sleep_for(std::chrono::seconds(5));

	// Each phase clips one IMU and requires the noise to appear on that IMU and not the
	// other, then the faults are cleared. The vehicle has to stay within a band of where it
	// was hovering throughout.
	tester.execute_alternating_imu_faults();

	tester.land();
	tester.wait_until_disarmed(std::chrono::seconds(90));
}
