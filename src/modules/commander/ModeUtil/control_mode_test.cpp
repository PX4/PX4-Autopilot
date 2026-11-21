/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
#include "control_mode.hpp"
#include <uORB/topics/vehicle_status.h>

TEST(ControlModeTest, AcroOutsideTransitionUsesRatesOnly)
{
	const offboard_control_mode_s offboard_control_mode{};

	for (uint8_t vehicle_type : {
		     vehicle_status_s::VEHICLE_TYPE_ROTARY_WING,
		     vehicle_status_s::VEHICLE_TYPE_FIXED_WING
	     }) {
		vehicle_control_mode_s control_mode{};
		mode_util::getVehicleControlMode(vehicle_status_s::NAVIGATION_STATE_ACRO,
						 vehicle_type, offboard_control_mode, control_mode,
						 mode_util::SetpointType::Invalid, false);

		EXPECT_TRUE(control_mode.flag_control_manual_enabled);
		EXPECT_FALSE(control_mode.flag_control_attitude_enabled);
		EXPECT_TRUE(control_mode.flag_control_rates_enabled);
		EXPECT_TRUE(control_mode.flag_control_allocation_enabled);
	}
}

TEST(ControlModeTest, AcroDuringVtolTransitionEnablesAttitude)
{
	const offboard_control_mode_s offboard_control_mode{};

	for (uint8_t vehicle_type : {
		     vehicle_status_s::VEHICLE_TYPE_ROTARY_WING,
		     vehicle_status_s::VEHICLE_TYPE_FIXED_WING
	     }) {
		vehicle_control_mode_s control_mode{};
		mode_util::getVehicleControlMode(vehicle_status_s::NAVIGATION_STATE_ACRO,
						 vehicle_type, offboard_control_mode, control_mode,
						 mode_util::SetpointType::Invalid, true);

		EXPECT_TRUE(control_mode.flag_control_manual_enabled);
		EXPECT_TRUE(control_mode.flag_control_attitude_enabled);
		EXPECT_TRUE(control_mode.flag_control_rates_enabled);
		EXPECT_TRUE(control_mode.flag_control_allocation_enabled);
	}
}

TEST(ControlModeTest, ManualUnchangedForRotaryAndFixedWing)
{
	const offboard_control_mode_s offboard_control_mode{};

	vehicle_control_mode_s mc_mode{};
	mode_util::getVehicleControlMode(vehicle_status_s::NAVIGATION_STATE_MANUAL,
					 vehicle_status_s::VEHICLE_TYPE_ROTARY_WING,
					 offboard_control_mode, mc_mode,
					 mode_util::SetpointType::Invalid, false);
	EXPECT_TRUE(mc_mode.flag_control_manual_enabled);
	EXPECT_TRUE(mc_mode.flag_control_attitude_enabled);
	EXPECT_TRUE(mc_mode.flag_control_rates_enabled);
	EXPECT_TRUE(mc_mode.flag_control_allocation_enabled);

	vehicle_control_mode_s fw_mode{};
	mode_util::getVehicleControlMode(vehicle_status_s::NAVIGATION_STATE_MANUAL,
					 vehicle_status_s::VEHICLE_TYPE_FIXED_WING,
					 offboard_control_mode, fw_mode,
					 mode_util::SetpointType::Invalid, false);
	EXPECT_TRUE(fw_mode.flag_control_manual_enabled);
	EXPECT_FALSE(fw_mode.flag_control_attitude_enabled);
	EXPECT_FALSE(fw_mode.flag_control_rates_enabled);
	EXPECT_TRUE(fw_mode.flag_control_allocation_enabled);
}
