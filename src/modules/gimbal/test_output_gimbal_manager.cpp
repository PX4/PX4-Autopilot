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
 * @file test_output_gimbal_manager.cpp
 *
 * PX4 as a client of an external gimbal manager: control is requested when
 * onboard intent starts and released when it ends. COMMAND_ACK can't tell
 * which of two in-flight DO_GIMBAL_MANAGER_CONFIGURE commands it answers, so a
 * retried acquire can reach the manager after the release. The manager's
 * status is what decides whether the release has to be sent again.
 */

#include <gtest/gtest.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/external_gimbal_manager_status.h>
#include <uORB/topics/vehicle_command.h>

#include "output_gimbal_manager.h"

using namespace gimbal;

namespace
{
constexpr uint8_t OWN_SYSID = 1;
constexpr uint8_t OWN_COMPID = 1;
constexpr uint8_t MANAGER_SYSID = 1;
constexpr uint8_t MANAGER_COMPID = 154;
constexpr uint8_t OTHER_SYSID = 255;
constexpr uint8_t OTHER_COMPID = 190;

// Longer than OutputToGimbalManager::CONFIGURE_RETRY_INTERVAL (2 s).
constexpr uint32_t PAST_RETRY_INTERVAL_US = 2200000;
}

class OutputToGimbalManagerTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		_parameters.mav_sysid = OWN_SYSID;
		_parameters.mav_compid = OWN_COMPID;

		// Drop any command left over from a previous test.
		vehicle_command_s cmd;

		while (_command_sub.update(&cmd)) {}
	}

	void publishStatus(uint8_t primary_sysid, uint8_t primary_compid)
	{
		external_gimbal_manager_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.manager_sysid = MANAGER_SYSID;
		status.manager_compid = MANAGER_COMPID;
		status.primary_control_sysid = primary_sysid;
		status.primary_control_compid = primary_compid;
		_status_pub.publish(status);
	}

	static ControlData onboardIntent()
	{
		ControlData control_data{};
		control_data.type = ControlData::Type::Angle;
		control_data.type_data.angle.q[0] = 1.f;
		control_data.sysid_primary_control = OWN_SYSID;
		control_data.compid_primary_control = OWN_COMPID;
		return control_data;
	}

	static ControlData noIntent()
	{
		return ControlData{};
	}

	void update(OutputToGimbalManager &output, const ControlData &control_data)
	{
		uint8_t gimbal_device_id = 0;
		output.update(control_data, true, gimbal_device_id);
	}

	// Acquire and release configure commands sent since the last call.
	struct Configures {
		int acquires{0};
		int releases{0};
	};

	Configures sentConfigures()
	{
		Configures configures;
		vehicle_command_s cmd;

		while (_command_sub.update(&cmd)) {
			if (cmd.command != vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE) {
				continue;
			}

			EXPECT_EQ(cmd.target_system, MANAGER_SYSID);
			EXPECT_EQ(cmd.target_component, MANAGER_COMPID);

			if (cmd.param1 < 0.f) {
				EXPECT_FLOAT_EQ(cmd.param1, -3.f);
				EXPECT_FLOAT_EQ(cmd.param2, -3.f);
				++configures.releases;

			} else {
				EXPECT_FLOAT_EQ(cmd.param1, OWN_SYSID);
				EXPECT_FLOAT_EQ(cmd.param2, OWN_COMPID);
				++configures.acquires;
			}
		}

		return configures;
	}

	Parameters _parameters{};
	uORB::Publication<external_gimbal_manager_status_s> _status_pub{ORB_ID(external_gimbal_manager_status)};
	uORB::Subscription _command_sub{ORB_ID(vehicle_command)};
};

TEST_F(OutputToGimbalManagerTest, ReleasesAgainWhileManagerStillReportsUsInControl)
{
	OutputToGimbalManager output(_parameters);
	publishStatus(0, 0);

	// GIVEN: onboard intent started, we asked for control and got it
	update(output, onboardIntent());
	EXPECT_EQ(sentConfigures().acquires, 1);
	publishStatus(OWN_SYSID, OWN_COMPID);
	update(output, onboardIntent());

	// WHEN: intent ends, we release, but a retried acquire reaches the manager
	// after the release, so it keeps reporting us in control
	update(output, noIntent());
	Configures configures = sentConfigures();
	EXPECT_EQ(configures.releases, 1);
	EXPECT_EQ(configures.acquires, 0);
	publishStatus(OWN_SYSID, OWN_COMPID);

	// THEN: no repeated release before the retry interval, so the manager has
	// time to process the first one
	update(output, noIntent());
	EXPECT_EQ(sentConfigures().releases, 0);

	// THEN: after the retry interval we release again
	px4_usleep(PAST_RETRY_INTERVAL_US);
	update(output, noIntent());
	configures = sentConfigures();
	EXPECT_EQ(configures.releases, 1);
	EXPECT_EQ(configures.acquires, 0);

	// THEN: once the manager reports that nobody is in control, we stop
	publishStatus(0, 0);
	px4_usleep(PAST_RETRY_INTERVAL_US);
	update(output, noIntent());
	configures = sentConfigures();
	EXPECT_EQ(configures.releases, 0);
	EXPECT_EQ(configures.acquires, 0);
}

TEST_F(OutputToGimbalManagerTest, DoesNotReleaseAnotherClientsControl)
{
	OutputToGimbalManager output(_parameters);

	// GIVEN: no onboard intent and another client (e.g. a ground station) in control
	publishStatus(OTHER_SYSID, OTHER_COMPID);
	update(output, noIntent());

	// WHEN: the retry interval passes
	px4_usleep(PAST_RETRY_INTERVAL_US);
	update(output, noIntent());

	// THEN: we never send a configure, it's not our control to release
	const Configures configures = sentConfigures();
	EXPECT_EQ(configures.releases, 0);
	EXPECT_EQ(configures.acquires, 0);
}

TEST_F(OutputToGimbalManagerTest, KeepsControlWhileIntentLasts)
{
	OutputToGimbalManager output(_parameters);
	publishStatus(0, 0);

	// GIVEN: onboard intent and the manager reports us in control
	update(output, onboardIntent());
	EXPECT_EQ(sentConfigures().acquires, 1);
	publishStatus(OWN_SYSID, OWN_COMPID);

	// WHEN: the retry interval passes with the intent still active
	px4_usleep(PAST_RETRY_INTERVAL_US);
	update(output, onboardIntent());

	// THEN: we neither release nor ask again
	const Configures configures = sentConfigures();
	EXPECT_EQ(configures.releases, 0);
	EXPECT_EQ(configures.acquires, 0);
}
