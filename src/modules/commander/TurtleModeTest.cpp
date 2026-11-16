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
#include "UserModeIntention.hpp"
#include "HealthAndArmingChecks/checks/modeCheck.hpp"
#include "ModeUtil/control_mode.hpp"
#include "ModeUtil/mode_requirements.hpp"
#include <parameters/param.h>
#include <uORB/Publication.hpp>

class TurtleModeTest : public ::testing::Test
{
protected:
	void SetUp() override { param_control_autosave(false); }
};

TEST_F(TurtleModeTest, RejectsEntryWhileArmedIncludingForcedRequests)
{
	vehicle_status_s status{};
	status.nav_state = vehicle_status_s::NAVIGATION_STATE_STAB;
	status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	HealthAndArmingChecks checks{nullptr, status};
	UserModeIntention intention{status, checks, nullptr};

	EXPECT_FALSE(intention.change(vehicle_status_s::NAVIGATION_STATE_TURTLE));
	EXPECT_FALSE(intention.change(vehicle_status_s::NAVIGATION_STATE_TURTLE, ModeChangeSource::User, false, true));
	EXPECT_NE(intention.get(), vehicle_status_s::NAVIGATION_STATE_TURTLE);
}

TEST_F(TurtleModeTest, AllowsDisarmedSelectionAndExitWithoutFlightRequirements)
{
	vehicle_status_s status{};
	status.nav_state = vehicle_status_s::NAVIGATION_STATE_STAB;
	status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	HealthAndArmingChecks checks{nullptr, status};
	UserModeIntention intention{status, checks, nullptr};
	ASSERT_TRUE(intention.change(vehicle_status_s::NAVIGATION_STATE_TURTLE));

	status.nav_state = vehicle_status_s::NAVIGATION_STATE_TURTLE;
	status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	ASSERT_FALSE(checks.canRun(vehicle_status_s::NAVIGATION_STATE_POSCTL));
	EXPECT_TRUE(intention.change(vehicle_status_s::NAVIGATION_STATE_POSCTL));
	EXPECT_EQ(intention.get(), vehicle_status_s::NAVIGATION_STATE_POSCTL);
}

TEST_F(TurtleModeTest, EnablesOnlyManualInputAndAllocation)
{
	vehicle_control_mode_s control{};
	offboard_control_mode_s offboard{};
	mode_util::getVehicleControlMode(vehicle_status_s::NAVIGATION_STATE_TURTLE,
					 vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, offboard, control,
					 mode_util::SetpointType::DirectActuators);

	EXPECT_TRUE(control.flag_control_manual_enabled);
	EXPECT_TRUE(control.flag_control_allocation_enabled);
	EXPECT_FALSE(control.flag_control_rates_enabled);
	EXPECT_FALSE(control.flag_control_attitude_enabled);
	EXPECT_FALSE(control.flag_control_position_enabled);
	EXPECT_FALSE(control.flag_control_altitude_enabled);
	EXPECT_FALSE(control.flag_control_auto_enabled);
	EXPECT_FALSE(control.flag_control_offboard_enabled);
}

TEST_F(TurtleModeTest, RequiresAttitudeAndManualInputWithoutNavigationEstimate)
{
	failsafe_flags_s requirements{};
	mode_util::getModeRequirements(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, requirements);
	const uint32_t turtle = 1u << vehicle_status_s::NAVIGATION_STATE_TURTLE;
	EXPECT_NE(requirements.mode_req_attitude & turtle, 0u);
	EXPECT_NE(requirements.mode_req_manual_control & turtle, 0u);
	EXPECT_EQ(requirements.mode_req_local_position & turtle, 0u);
	EXPECT_EQ(requirements.mode_req_local_alt & turtle, 0u);
	EXPECT_EQ(requirements.mode_req_global_position & turtle, 0u);

	mode_util::getModeRequirements(vehicle_status_s::VEHICLE_TYPE_FIXED_WING, requirements);
	EXPECT_NE(requirements.mode_not_supported & turtle, 0u);
	mode_util::getModeRequirements(vehicle_status_s::VEHICLE_TYPE_ROVER, requirements);
	EXPECT_NE(requirements.mode_not_supported & turtle, 0u);
}

TEST_F(TurtleModeTest, SelectionDoesNotRequireAnAlreadyRunningTurtleController)
{
	const param_t enabled_handle = param_find("TRTL_EN");
	ASSERT_NE(enabled_handle, PARAM_INVALID);
	int32_t enabled = 1;
	ASSERT_EQ(param_set_no_notification(enabled_handle, &enabled), 0);
	vehicle_status_s status{};
	status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	failsafe_flags_s flags{};
	Context context(status);
	Report report(flags);
	ModeChecks check;
	check.checkAndReport(context, report);
	const uint32_t turtle = 1u << vehicle_status_s::NAVIGATION_STATE_TURTLE;
	EXPECT_EQ(flags.mode_not_supported & turtle, 0u);

	enabled = 0;
	ASSERT_EQ(param_set_no_notification(enabled_handle, &enabled), 0);
	check.checkAndReport(context, report);
	EXPECT_NE(flags.mode_not_supported & turtle, 0u);
}

TEST_F(TurtleModeTest, ArmingRequiresCurrentControlAndValidAllocatorGeometry)
{
	uORB::Publication<turtle_control_s> turtle_pub{ORB_ID(turtle_control)};
	uORB::Publication<control_allocator_status_s> allocation_pub{ORB_ID(control_allocator_status)};
	uORB::Publication<motor_direction_status_s> direction_pub{ORB_ID(motor_direction_status)};
	vehicle_status_s status{};
	status.nav_state = vehicle_status_s::NAVIGATION_STATE_TURTLE;
	status.nav_state_timestamp = hrt_absolute_time();
	ModeChecks check;

	motor_direction_status_s direction{};
	direction.timestamp = status.nav_state_timestamp;
	direction_pub.publish(direction);

	control_allocator_status_s allocation{};
	allocation.timestamp = status.nav_state_timestamp;
	allocation.turtle_mode_active = true;
	allocation_pub.publish(allocation);

	turtle_control_s turtle{};
	turtle.timestamp = status.nav_state_timestamp;
	turtle.enabled = true;
	turtle.ready_to_arm = true;
	turtle_pub.publish(turtle);
	EXPECT_FALSE(check.turtleArmingStatus(status).ready_to_arm);

	allocation.turtle_geometry_valid = true;
	allocation_pub.publish(allocation);
	EXPECT_TRUE(check.turtleArmingStatus(status).ready_to_arm);

	turtle.timestamp = status.nav_state_timestamp - 1;
	turtle_pub.publish(turtle);
	EXPECT_FALSE(check.turtleArmingStatus(status).ready_to_arm);

	turtle.timestamp = hrt_absolute_time();
	turtle_pub.publish(turtle);
	direction.state = motor_direction_status_s::STATE_PENDING;
	direction_pub.publish(direction);
	const auto restoring = check.turtleArmingStatus(status);
	EXPECT_TRUE(restoring.direction_active);
	EXPECT_FALSE(restoring.ready_to_arm);
}

TEST_F(TurtleModeTest, ThrowLaunchConfigurationExcludesTurtle)
{
	const param_t enabled_handle = param_find("TRTL_EN");
	const param_t throw_handle = param_find("COM_THROW_EN");
	ASSERT_NE(enabled_handle, PARAM_INVALID);
	ASSERT_NE(throw_handle, PARAM_INVALID);
	int32_t enabled = 1;
	ASSERT_EQ(param_set_no_notification(enabled_handle, &enabled), 0);
	ASSERT_EQ(param_set_no_notification(throw_handle, &enabled), 0);
	vehicle_status_s status{};
	status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	failsafe_flags_s flags{};
	Context context(status);
	Report report(flags);
	ModeChecks check;
	check.checkAndReport(context, report);
	const uint32_t turtle = 1u << vehicle_status_s::NAVIGATION_STATE_TURTLE;
	EXPECT_NE(flags.mode_not_supported & turtle, 0u);

	enabled = 0;
	ASSERT_EQ(param_set_no_notification(throw_handle, &enabled), 0);
	flags = {};
	check.checkAndReport(context, report);
	EXPECT_EQ(flags.mode_not_supported & turtle, 0u);
}
