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

#include "vtol_att_control_main.h"
#include "vtol_type.h"
#include <parameters/param.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <uORB/topics/vehicle_command_ack.h>

class VtolAttitudeControlTestPeer : public VtolAttitudeControl
{
public:
	~VtolAttitudeControlTestPeer() override { delete _vtol_type; _vtol_type = nullptr; }

	void configure(int32_t rtl_type, uint8_t nav_state, bool failed)
	{
		vehicle_command_s command{};

		while (_vehicle_cmd_sub.update(&command)) {}

		_param_rtl_type.set(rtl_type);
		_vehicle_status.nav_state = nav_state;
		_nav_state_prev = nav_state;
		_vtol_vehicle_status.fixed_wing_system_failure = failed;
	}

	void requestFrontTransition(bool external)
	{
		vehicle_command_s command{};
		command.timestamp = hrt_absolute_time();
		command.command = vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
		command.param1 = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;
		command.from_external = external;
		command.source_system = 1;
		command.source_component = 1;
		uORB::Publication<vehicle_command_s> publisher{ORB_ID(vehicle_command)};
		publisher.publish(command);
		vehicle_cmd_poll();
	}

	bool failed() const { return _vtol_vehicle_status.fixed_wing_system_failure; }

	void startFrontTransition()
	{
		_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;
		_vtol_type->update_vtol_state();
		ASSERT_EQ(_vtol_type->get_mode(), mode::TRANSITION_TO_FW);
	}

	void enterRtl(bool failed)
	{
		_vtol_vehicle_status.fixed_wing_system_failure = failed;
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		uORB::Publication<vehicle_status_s> publisher{ORB_ID(vehicle_status)};
		publisher.publish(status);
		vehicle_status_poll();
	}
};

class VtolRtlTransitionTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite()
	{
		param_control_autosave(false);
		px4::WorkQueueManagerStart();
		const int32_t standard_vtol = static_cast<int32_t>(vtol_type::STANDARD);
		ASSERT_EQ(param_set_no_notification(param_find("VT_TYPE"), &standard_vtol), 0);
	}

	static void TearDownTestSuite() { px4::WorkQueueManagerStop(); }
};

struct FrontTransitionCase {
	const char *name;
	int32_t rtl_type;
	bool external;
	bool failed;
	uint8_t nav_state;
	bool accepted;
};

class VtolRtlFrontTransitionTest : public VtolRtlTransitionTest,
	public ::testing::WithParamInterface<FrontTransitionCase> {};

TEST_P(VtolRtlFrontTransitionTest, OnlyHealthyInternalSrpCommandsCanFrontTransitionInRtl)
{
	const auto &test_case = GetParam();
	VtolAttitudeControlTestPeer controller;
	controller.configure(test_case.rtl_type, test_case.nav_state, test_case.failed);
	uORB::Subscription ack_sub{ORB_ID(vehicle_command_ack)};
	controller.requestFrontTransition(test_case.external);
	EXPECT_EQ(controller.is_fixed_wing_requested(), test_case.accepted);
	EXPECT_EQ(controller.failed(), test_case.failed);

	if (test_case.external) {
		vehicle_command_ack_s ack{};
		ASSERT_TRUE(ack_sub.update(&ack));
		EXPECT_EQ(ack.result, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);
	}
}

INSTANTIATE_TEST_SUITE_P(CommandPolicy, VtolRtlFrontTransitionTest, ::testing::Values(
				 FrontTransitionCase{"HealthySrp", 7, false, false, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL, true},
				 FrontTransitionCase{"ExternalSrp", 7, true, false, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL, false},
				 FrontTransitionCase{"FailedSrp", 7, false, true, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL, false},
				 FrontTransitionCase{"DirectRtl", 0, false, false, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL, false},
				 FrontTransitionCase{"MissionRtl", 2, false, false, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL, false},
				 FrontTransitionCase{"Takeoff", 7, false, false, vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF, false},
				 FrontTransitionCase{"Land", 7, false, false, vehicle_status_s::NAVIGATION_STATE_AUTO_LAND, false},
				 FrontTransitionCase{"Orbit", 7, false, false, vehicle_status_s::NAVIGATION_STATE_ORBIT, false},
				 FrontTransitionCase{"Mission", 0, false, false, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, true}),
[](const ::testing::TestParamInfo<FrontTransitionCase> &test_info) { return test_info.param.name; });

TEST_F(VtolRtlTransitionTest, RtlEntryPreservesFrontTransitionOnlyForHealthySrp)
{
	// The command can arrive before the controller observes AUTO_RTL.
	for (int32_t rtl_type : {0, 7}) {
		for (bool failed : {false, true}) {
			VtolAttitudeControlTestPeer controller;
			controller.configure(rtl_type, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, false);
			controller.startFrontTransition();
			controller.enterRtl(failed);
			EXPECT_EQ(controller.is_fixed_wing_requested(), rtl_type == 7 && !failed);
		}
	}
}
