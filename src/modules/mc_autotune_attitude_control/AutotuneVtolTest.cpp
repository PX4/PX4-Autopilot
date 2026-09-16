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
#include <hrt_work.h>
#include "mc_autotune_attitude_control.hpp"
#include "../fw_autotune_attitude_control/fw_autotune_attitude_control.hpp"

class AutotuneVtolTest : public ::testing::Test
{
public:
	static void SetUpTestSuite() { hrt_work_queue_init(); }

protected:
	void SetUp() override { param_reset_all(); }

	void command(uint8_t vehicle_type, bool transition = false)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.vehicle_type = vehicle_type;
		status.is_vtol = true;
		status.in_transition_mode = transition;
		_status_pub.publish(status);

		vehicle_command_s request{};
		request.timestamp = hrt_absolute_time();
		request.command = vehicle_command_s::VEHICLE_CMD_DO_AUTOTUNE_ENABLE;
		request.param1 = 1.f;
		_command_pub.publish(request);
		_mc.Run();
		_fw.Run();
	}

	bool mcStarted() const { return _mc._vehicle_cmd_start_autotune; }
	bool fwStarted() const { return _fw._vehicle_cmd_start_autotune; }

	McAutotuneAttitudeControl _mc;
	FwAutotuneAttitudeControl _fw{true};
	uORB::Publication<vehicle_status_s> _status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_command_s> _command_pub{ORB_ID(vehicle_command)};
};

TEST_F(AutotuneVtolTest, HoverStartsOnlyMulticopterAutotune)
{
	command(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	EXPECT_TRUE(mcStarted());
	EXPECT_FALSE(fwStarted());
}

TEST_F(AutotuneVtolTest, FixedWingStartsOnlyFixedWingAutotune)
{
	command(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	EXPECT_FALSE(mcStarted());
	EXPECT_TRUE(fwStarted());
}

TEST_F(AutotuneVtolTest, ForwardTransitionStartsNeitherModule)
{
	command(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true);
	EXPECT_FALSE(mcStarted());
	EXPECT_FALSE(fwStarted());
}

TEST_F(AutotuneVtolTest, BackTransitionStartsNeitherModule)
{
	command(vehicle_status_s::VEHICLE_TYPE_FIXED_WING, true);
	EXPECT_FALSE(mcStarted());
	EXPECT_FALSE(fwStarted());
}

TEST_F(AutotuneVtolTest, UnknownVehicleTypeStartsNeitherModule)
{
	command(vehicle_status_s::VEHICLE_TYPE_UNSPECIFIED);
	EXPECT_FALSE(mcStarted());
	EXPECT_FALSE(fwStarted());
}
