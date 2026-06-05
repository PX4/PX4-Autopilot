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
 * Regression test for EKF2 multi-instance selector bug (#27013).
 *
 * Reproduces the failure chain from the real flight:
 * 1. Inject accel clipping on IMU 0 so EKF0 declares cs_baro_fault
 *    and its Z state diverges (GPS-only height).
 * 2. Move the clipping to IMU 1. Now EKF0 has no filter_fault_flags
 *    (clipping stopped) but has a diverged Z. EKF1 gets bad_acc_clipping.
 * 3. The selector should NOT whipsaw to the diverged EKF0 just because
 *    EKF1 has a transient clipping fault.
 *
 * Requires SIH with dual-IMU support (EKF2_MULTI_IMU=2) and per-IMU
 * fault injection (SIH_FAULT_IMU / SIH_FAULT_VIBE).
 */

#include "autopilot_tester.h"

#include <iostream>

TEST_CASE("EKF2 selector - no whipsaw after baro fault and IMU swap", "[ekf2_selector]")
{
	const float takeoff_altitude = 20.f;
	const float altitude_tolerance = 5.f;

	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	tester.set_takeoff_altitude(takeoff_altitude);
	tester.store_home();
	tester.sleep_for(std::chrono::seconds(1));

	// Takeoff and stabilize
	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.wait_until_altitude(takeoff_altitude, std::chrono::seconds(30));
	tester.sleep_for(std::chrono::seconds(5));

	// Phase 1: Clip IMU 0 to force EKF0 into cs_baro_fault.
	// The corrupted accel integration causes EKF0's height prediction
	// to diverge from baro, triggering baro rejection. Once EKF0
	// switches to GPS-only height, its Z state drifts.
	std::cout << time_str() << "Phase 1: clipping IMU 0 to diverge EKF0\n";
	tester.set_param_int("SIH_FAULT_IMU", 1);  // 1-indexed: IMU 0
	tester.set_param_float("SIH_FAULT_VIBE", 200.f);

	// Hold long enough for EKF0 to declare baro fault and diverge
	tester.sleep_for(std::chrono::seconds(20));

	// Phase 2: Move clipping from IMU 0 to IMU 1.
	// EKF0 stops clipping (filter_fault_flags clears) but retains
	// cs_baro_fault with diverged Z. The selector sees EKF0 as
	// "healthy" because cs_baro_fault is not in filter_fault_flags.
	// EKF1 now has bad_acc_clipping, so the selector marks it unhealthy
	// and wants to switch to the "healthy" but diverged EKF0.
	std::cout << time_str() << "Phase 2: swapping clipping to IMU 1\n";
	tester.set_param_int("SIH_FAULT_IMU", 2);  // 1-indexed: IMU 1

	// Monitor altitude. If the selector whipsaws to the diverged EKF0,
	// altitude will spike well beyond tolerance.
	tester.start_checking_altitude(altitude_tolerance);
	tester.sleep_for(std::chrono::seconds(30));
	tester.stop_checking_altitude();

	// Cleanup
	std::cout << time_str() << "Removing faults\n";
	tester.set_param_int("SIH_FAULT_IMU", 0);
	tester.set_param_float("SIH_FAULT_VIBE", 0.f);

	tester.sleep_for(std::chrono::seconds(5));

	tester.land();
	tester.wait_until_disarmed(std::chrono::seconds(60));
}
