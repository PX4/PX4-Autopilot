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
 * @file test_multicopter_sih_gravity.cpp
 *
 * SIH regression for https://github.com/PX4/PX4-Autopilot/issues/24299
 *
 * When flying Stabilized near the ground without GPS/optical flow (and without
 * mag), mid-stick pitch/roll must bring true attitude back to near level.
 * Tilt is then constrained only by gravity fusion; if that drops out, the EKF
 * tilt drifts and mid-stick no longer means level flight.
 *
 * Note: clean SIH IMU noise may not fully reproduce the outdoor near-ground
 * dropout seen in logs, but this locks the intended contract before/after a
 * gravity-fusion fix and catches total loss of tilt aiding.
 */

#include "autopilot_tester.h"


TEST_CASE("Stabilize mid-stick returns to level without GPS (SIH)", "[sih][gravity]")
{
	AutopilotTester tester;
	tester.connect(connection_url);

	// Gravity fusion enabled (bit2), gyro+accel bias (bits 0-1).
	tester.set_param_int("EKF2_IMU_CTRL", 7);
	// Allow arming once we drop GPS aiding mid-test.
	tester.set_param_int("COM_ARM_WO_GPS", 1);

	// Low altitude: issue #24299 is most visible near the ground without OF/GPS.
	const float tilt_err_deg = tester.fly_stabilize_without_gps_and_measure_level_error(2.0f);

	// Mid-stick must keep true attitude near the EKF estimate (level setpoint).
	CHECK(tilt_err_deg < 6.f);
}
