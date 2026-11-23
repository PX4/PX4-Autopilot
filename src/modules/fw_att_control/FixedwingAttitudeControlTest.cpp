/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include "FixedwingAttitudeControl.hpp"

using namespace matrix;

/**
 * Computes the body rate setpoint yaw component (before turn coordination)
 * using the shared computeAttitudeError function from FixedwingAttitudeControl.hpp.
 *
 * @param q_current Current attitude quaternion
 * @param q_sp Attitude setpoint quaternion
 * @return Yaw rate setpoint before turn coordination feedforward
 */
static float computeYawRateSetpointBeforeTurnCoord(const Quatf &q_current, const Quatf &q_sp)
{
	const Vector3f att_err = computeAttitudeError(q_current, q_sp);
	return att_err(2);
}

TEST(FixedwingAttitudeControlTest, YawRateSetpointZeroBeforeTurnCoord_Identity)
{
	// When current and setpoint are both identity, yaw rate should be zero
	const Quatf q_current;
	const Quatf q_sp;

	const float yaw_rate = computeYawRateSetpointBeforeTurnCoord(q_current, q_sp);
	EXPECT_NEAR(yaw_rate, 0.f, 1e-5f);
}

TEST(FixedwingAttitudeControlTest, YawRateSetpointZeroBeforeTurnCoord_PureYawError)
{
	// Pure yaw error should result in zero yaw rate setpoint (yaw is compensated out)
	const Quatf q_current;
	const Quatf q_sp(Eulerf(0.f, 0.f, 0.5f)); // 0.5 rad (~29 deg) yaw offset

	const float yaw_rate = computeYawRateSetpointBeforeTurnCoord(q_current, q_sp);

	EXPECT_NEAR(yaw_rate, 0.f, 1e-5f);
}

TEST(FixedwingAttitudeControlTest, YawRateSetpointZeroBeforeTurnCoord_RollError)
{
	// Roll error with no pitch/yaw error
	const Quatf q_current;
	const Quatf q_sp(Eulerf(0.3f, 0.f, 0.f)); // 0.3 rad roll

	const float yaw_rate = computeYawRateSetpointBeforeTurnCoord(q_current, q_sp);

	EXPECT_NEAR(yaw_rate, 0.f, 1e-5f);
}

TEST(FixedwingAttitudeControlTest, YawRateSetpointZeroBeforeTurnCoord_PitchError)
{
	// Pitch error with no roll/yaw error
	const Quatf q_current;
	const Quatf q_sp(Eulerf(0.f, 0.2f, 0.f)); // 0.2 rad pitch

	const float yaw_rate = computeYawRateSetpointBeforeTurnCoord(q_current, q_sp);

	EXPECT_NEAR(yaw_rate, 0.f, 1e-5f);
}

TEST(FixedwingAttitudeControlTest, YawRateSetpointZeroBeforeTurnCoord_CombinedError)
{
	// Combined roll, pitch, and yaw error
	const Quatf q_current(Eulerf(0.1f, 0.05f, 0.2f));
	const Quatf q_sp(Eulerf(0.4f, 0.15f, 0.8f));

	const float yaw_rate = computeYawRateSetpointBeforeTurnCoord(q_current, q_sp);

	EXPECT_NEAR(yaw_rate, 0.f, 1e-4f);
}

TEST(FixedwingAttitudeControlTest, YawRateSetpointZeroBeforeTurnCoord_BankedTurn)
{
	// Simulating a banked turn scenario - current has roll, setpoint has roll + yaw
	const Quatf q_current(Eulerf(0.5f, 0.f, 0.f)); // 30 deg bank
	const Quatf q_sp(Eulerf(0.5f, 0.f, 0.3f));     // same bank, different heading

	const float yaw_rate = computeYawRateSetpointBeforeTurnCoord(q_current, q_sp);

	EXPECT_NEAR(yaw_rate, 0.f, 1e-4f);
}

TEST(FixedwingAttitudeControlTest, TailsitterFrameRotation)
{
	// NOTE: this test cannot guard against breaking the conversion in Run()
	// (while keeping the quaternion the same). This only sanity checks the
	// underlying math. We rely on integration tests to test the actual
	// implementation in Run().

	const Quatf &q_mc_to_fw = FixedwingAttitudeControl::_q_mc_to_fw;

	// The rotation maps the MC body axes onto the FW body axes: MC x -> FW z, MC z -> FW -x.
	const Vector3f x_fw = q_mc_to_fw.rotateVector(Vector3f(1.f, 0.f, 0.f));
	const Vector3f z_fw = q_mc_to_fw.rotateVector(Vector3f(0.f, 0.f, 1.f));
	EXPECT_NEAR(x_fw(0), 0.f, 1e-6f);
	EXPECT_NEAR(x_fw(2), 1.f, 1e-6f);
	EXPECT_NEAR(z_fw(0), -1.f, 1e-6f);
	EXPECT_NEAR(z_fw(2), 0.f, 1e-6f);

	// Attitude of level FW flight in MC frame
	const Quatf q_ned_mc(Eulerf(0.f, -M_PI_2_F, 0.f));
	// level fixed-wing setpoint = unit quaternion
	const Quatf q_sp;

	// The raw hover-frame attitude appears as a large error against the level FW setpoint.
	EXPECT_GT(computeAttitudeError(q_ned_mc, q_sp).norm(), 1.f);

	// Adapting it exactly as Run() does (att.q * _q_mc_to_fw.inversed()) removes the error.
	const Quatf q_adapted = q_ned_mc * q_mc_to_fw.inversed();
	EXPECT_NEAR(computeAttitudeError(q_adapted, q_sp).norm(), 0.f, 1e-5f);
}
