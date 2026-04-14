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
	bool update(float lpos_z, float lpos_sp_z, float delta_z = 0.f, uint8_t z_reset_counter = 0)
	{
		vehicle_local_position_s lpos{};
		lpos.timestamp = hrt_absolute_time();
		lpos.z = lpos_z;
		lpos.z_valid = true;
		lpos.delta_z = delta_z;
		lpos.z_reset_counter = z_reset_counter;
		_lpos_pub.publish(lpos);

		vehicle_local_position_setpoint_s lpos_sp{};
		lpos_sp.timestamp = hrt_absolute_time();
		lpos_sp.z = lpos_sp_z;
		_lpos_sp_pub.publish(lpos_sp);

		vehicle_status_s status{};
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

		vehicle_control_mode_s control_mode{};
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;

		_fd.update(status, control_mode);

		return _fd.getStatus().flags.alt;
	}

private:
	FailureDetector _fd{nullptr};

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
