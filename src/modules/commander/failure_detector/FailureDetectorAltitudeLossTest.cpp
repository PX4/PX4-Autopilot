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

#include <gtest/gtest.h>

#include "FailureDetector.hpp"

#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

// to run: make tests TESTFILTER=FailureDetectorAltitudeLossTest

using namespace time_literals;

class FailureDetectorAltitudeLossTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);

		// FD_ALT_LOSS = 5m, FD_ALT_LOSS_T = 0s
		param_t param = param_handle(px4::params::FD_ALT_LOSS);
		float threshold = 5.f;
		param_set(param, &threshold);

		param = param_handle(px4::params::FD_ALT_LOSS_T);
		float ttri = 0.f;
		param_set(param, &ttri);
	}

	// Publish position and setpoint, then run the detector. Returns the alt flag state.
	bool update(float lpos_z, float lpos_sp_z, float delta_z = 0.f, uint8_t z_reset_counter = 0, float lpos_sp_vz = 0.f,
		    bool publish_setpoint = true)
	{
		// Each update represents 1s of flight for integrating the commanded vertical velocity
		_timestamp += 1_s;

		vehicle_local_position_s lpos{};
		lpos.timestamp = _timestamp;
		lpos.z = lpos_z;
		lpos.z_valid = true;
		lpos.delta_z = delta_z;
		lpos.z_reset_counter = z_reset_counter;
		_lpos_pub.publish(lpos);

		if (publish_setpoint) {
			vehicle_local_position_setpoint_s lpos_sp{};
			lpos_sp.timestamp = _timestamp;
			lpos_sp.z = lpos_sp_z;
			lpos_sp.vz = lpos_sp_vz;
			_lpos_sp_pub.publish(lpos_sp);
		}

		vehicle_status_s status{};
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

		vehicle_control_mode_s control_mode{};
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;

		_fd.update(status, control_mode);

		return _fd.getStatus().flags.alt;
	}

	// Update without altitude setpoint, only a commanded vertical velocity (NED, positive down)
	bool updateVelocity(float lpos_z, float lpos_sp_vz)
	{
		return update(lpos_z, NAN, 0.f, 0, lpos_sp_vz);
	}

	// Update the position only, the last published setpoint becomes stale
	bool updatePositionOnly(float lpos_z)
	{
		return update(lpos_z, NAN, 0.f, 0, NAN, false);
	}

private:
	FailureDetector _fd{nullptr};
	hrt_abstime _timestamp{1_s};

	uORB::Publication<vehicle_local_position_s> _lpos_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_local_position_setpoint_s> _lpos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
};

TEST_F(FailureDetectorAltitudeLossTest, NoTriggerWhenDisabled)
{
	param_t param = param_handle(px4::params::FD_ALT_LOSS);
	float threshold = 0.f;
	param_set(param, &threshold);

	EXPECT_FALSE(update(-90.f, -100.f));
}

TEST_F(FailureDetectorAltitudeLossTest, NoTriggerAboveSetpoint)
{
	EXPECT_FALSE(update(-105.f, -100.f));
}

TEST_F(FailureDetectorAltitudeLossTest, NoTriggerWithinThreshold)
{
	// 3m below setpoint, threshold is 5m
	EXPECT_FALSE(update(-97.f, -100.f));
}

TEST_F(FailureDetectorAltitudeLossTest, TriggerAfterSustainedDrop)
{
	// Ratchet initialises at -97; vehicle sinks to -91 (6m drop exceeds 5m threshold)
	EXPECT_FALSE(update(-97.f, -100.f)); // 0m drop
	EXPECT_FALSE(update(-95.f, -100.f)); // 2m drop
	EXPECT_FALSE(update(-93.f, -100.f)); // 4m drop
	EXPECT_TRUE(update(-91.f, -100.f));  // 6m drop, triggers
}

TEST_F(FailureDetectorAltitudeLossTest, RatchetHoldsOnPartialRecovery)
{
	// Ratchet tracks the best recovery point and is not reset by a partial climb
	EXPECT_FALSE(update(-96.f, -100.f)); // ratchet = -96, drop = 0m
	EXPECT_FALSE(update(-94.f, -100.f)); // ratchet = -96, drop = 2m
	EXPECT_FALSE(update(-96.f, -100.f)); // partial recovery, ratchet stays at -96, drop = 0m
	EXPECT_FALSE(update(-95.f, -100.f)); // ratchet = -96, drop = 1m
	EXPECT_TRUE(update(-90.f, -100.f));  // ratchet = -96, drop = 6m, triggers
}

TEST_F(FailureDetectorAltitudeLossTest, ResetWhenBackAboveSetpoint)
{
	// After triggering, climbing above setpoint clears the flag and resets the ratchet
	EXPECT_FALSE(update(-97.f, -100.f));
	EXPECT_TRUE(update(-91.f, -100.f));

	EXPECT_FALSE(update(-101.f, -100.f)); // above setpoint, resets

	EXPECT_FALSE(update(-97.f, -100.f)); // ratchet reinitialises at -97, drop = 0m
	EXPECT_FALSE(update(-95.f, -100.f)); // drop = 2m, no trigger
}

TEST_F(FailureDetectorAltitudeLossTest, NoTriggerOnEkfZReset)
{
	// EKF resets z by 6m downward; reference shifts with it so the drop stays 0m.
	EXPECT_FALSE(update(-97.f, -100.f));          // ratchet = -97, drop = 0m
	EXPECT_FALSE(update(-91.f, -100.f, 6.f, 1)); // EKF reset: ref shifts to -91, drop = 0m
}

TEST_F(FailureDetectorAltitudeLossTest, NoTriggerOnSetpointJump)
{
	// Setpoint jumps 10m higher; vehicle position unchanged so ratchet stays at -97, drop = 0m.
	EXPECT_FALSE(update(-97.f, -100.f)); // ratchet = -97, drop = 0m
	EXPECT_FALSE(update(-97.f, -110.f)); // setpoint jumps, vehicle 13m below sp, but drop = 0m
	EXPECT_FALSE(update(-97.f, -110.f)); // vehicle holds position, no trigger
}

TEST_F(FailureDetectorAltitudeLossTest, TriggerOnDropWhileClimbCommanded)
{
	// No height target, pilot commands climb but the vehicle falls
	EXPECT_FALSE(updateVelocity(-100.f, -2.f)); // ref = -100
	EXPECT_FALSE(updateVelocity(-102.f, -2.f)); // climbing, ref = -102
	EXPECT_FALSE(updateVelocity(-99.f, -2.f));  // drop = 3m
	EXPECT_TRUE(updateVelocity(-96.f, -2.f));   // drop = 6m, triggers
}

TEST_F(FailureDetectorAltitudeLossTest, TriggerOnDropWhileHoldCommanded)
{
	// No height target and zero commanded vertical velocity
	EXPECT_FALSE(updateVelocity(-100.f, 0.f));
	EXPECT_FALSE(updateVelocity(-97.f, 0.f)); // drop = 3m
	EXPECT_TRUE(updateVelocity(-94.f, 0.f));  // drop = 6m, triggers
}

TEST_F(FailureDetectorAltitudeLossTest, NoTriggerOnCommandedDescent)
{
	// Manual descent at 3m/s, vehicle follows the command
	EXPECT_FALSE(updateVelocity(-100.f, 3.f)); // ref = -100
	EXPECT_FALSE(updateVelocity(-97.f, 3.f));  // ref = -97
	EXPECT_FALSE(updateVelocity(-94.f, 3.f));
	EXPECT_FALSE(updateVelocity(-91.f, 3.f));
	EXPECT_FALSE(updateVelocity(-88.f, 3.f));
	EXPECT_FALSE(updateVelocity(-85.f, 3.f));
}

TEST_F(FailureDetectorAltitudeLossTest, NoTriggerOnSlowerThanCommandedDescent)
{
	// Vehicle descends slower than commanded: reference does not get below the vehicle
	EXPECT_FALSE(updateVelocity(-100.f, 3.f));
	EXPECT_FALSE(updateVelocity(-99.f, 3.f));
	EXPECT_FALSE(updateVelocity(-98.f, 3.f));
	EXPECT_FALSE(updateVelocity(-97.f, 0.f));
	EXPECT_FALSE(updateVelocity(-93.f, 0.f)); // drop = 4m from -97
}

TEST_F(FailureDetectorAltitudeLossTest, TriggerOnDropFasterThanCommandedDescent)
{
	// Manual descent commanded at 1m/s, vehicle falls at 4m/s
	EXPECT_FALSE(updateVelocity(-100.f, 1.f)); // ref = -100
	EXPECT_FALSE(updateVelocity(-96.f, 1.f));  // ref = -99, drop = 3m
	EXPECT_TRUE(updateVelocity(-92.f, 1.f));   // ref = -98, drop = 6m, triggers
}

TEST_F(FailureDetectorAltitudeLossTest, NoTriggerWithoutSetpoints)
{
	EXPECT_FALSE(update(-100.f, NAN, 0.f, 0, NAN));
	EXPECT_FALSE(update(-80.f, NAN, 0.f, 0, NAN));
}

TEST_F(FailureDetectorAltitudeLossTest, NoTriggerOnEkfZResetWithoutHeightTarget)
{
	EXPECT_FALSE(updateVelocity(-100.f, 0.f));
	EXPECT_FALSE(update(-94.f, NAN, 6.f, 1, 0.f)); // EKF reset: ref shifts to -94, drop = 0m
}

TEST_F(FailureDetectorAltitudeLossTest, TriggerOnDropAfterClimbHoldDescendClimb)
{
	// Climb, hold, descend, climb again below the highest point, then fall
	EXPECT_FALSE(updateVelocity(-100.f, -5.f));  // climb, ref = -100
	EXPECT_FALSE(updateVelocity(-105.f, -5.f));  // ref = -105
	EXPECT_FALSE(updateVelocity(-110.f, -5.f));  // ref = -110
	EXPECT_FALSE(update(-110.f, -110.f));        // hold at altitude setpoint, ref reset
	EXPECT_FALSE(updateVelocity(-110.f, 5.f));   // descend, ref = -110
	EXPECT_FALSE(updateVelocity(-105.f, 5.f));   // ref = -105
	EXPECT_FALSE(updateVelocity(-100.f, 5.f));   // ref = -100
	EXPECT_FALSE(updateVelocity(-102.f, -2.f));  // climb again, ref = -102
	EXPECT_FALSE(updateVelocity(-104.f, -2.f));  // ref = -104, still below the highest point (-110)
	EXPECT_FALSE(updateVelocity(-101.f, -2.f));  // fall, drop = 3m
	EXPECT_TRUE(updateVelocity(-98.f, -2.f));    // drop = 6m, triggers
}

TEST_F(FailureDetectorAltitudeLossTest, TriggerOnDropDuringVelocityHoldAfterDescent)
{
	// Same as above but the hold phase has no altitude setpoint yet (braking, vz = 0)
	EXPECT_FALSE(updateVelocity(-110.f, -5.f));  // ref = -110
	EXPECT_FALSE(updateVelocity(-110.f, 0.f));   // hold, ref = -110
	EXPECT_FALSE(updateVelocity(-105.f, 5.f));   // descend, ref = -105
	EXPECT_FALSE(updateVelocity(-100.f, 5.f));   // ref = -100
	EXPECT_FALSE(updateVelocity(-104.f, -2.f));  // climb again, ref = -104
	EXPECT_FALSE(updateVelocity(-104.f, 0.f));   // hold, ref = -104
	EXPECT_TRUE(updateVelocity(-98.f, 0.f));     // fall, drop = 6m, triggers
}

TEST_F(FailureDetectorAltitudeLossTest, TriggerOnDropWithStaleDescentSetpoint)
{
	// Position control stops publishing while a 3m/s descent is commanded: the stale
	// descent must not keep lowering the reference and mask the altitude loss.
	EXPECT_FALSE(updateVelocity(-100.f, 3.f));  // ref = -100
	EXPECT_FALSE(updateVelocity(-97.f, 3.f));   // ref = -97
	EXPECT_FALSE(updatePositionOnly(-94.f));    // setpoint stale, ref stays at -97, drop = 3m
	EXPECT_TRUE(updatePositionOnly(-91.f));     // drop = 6m, triggers
}
