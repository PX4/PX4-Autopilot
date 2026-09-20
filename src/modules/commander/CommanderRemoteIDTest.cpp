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

#include "Commander.hpp"
#include "HealthAndArmingChecks/checks/openDroneIDCheck.hpp"

#include <memory>
#include <uORB/Publication.hpp>

// To run: make tests TESTFILTER=CommanderRemoteID

class CommanderRemoteIDTest : public ::testing::Test
{
public:

	void SetUp() override
	{
		param_control_autosave(false);
		ASSERT_EQ(param_get(param_find("COM_ARM_ODID"), &_saved_com_arm_odid), PX4_OK);
		setArmingCheck(2);

		// uORB retains the last sample after unadvertising; clear it between tests.
		ASSERT_TRUE(_can_pub.publish(open_drone_id_arm_status_s{}));
		ASSERT_TRUE(_telemetry_pub.publish(telemetry_status_s{}));
		_commander.reset(new Commander());
		_commander->_config_overrides.disable_auto_set_home = true;
	}

	void TearDown() override
	{
		_commander.reset();
		EXPECT_EQ(param_set(param_find("COM_ARM_ODID"), &_saved_com_arm_odid), PX4_OK);
	}

	void setArmingCheck(int32_t value)
	{
		ASSERT_EQ(param_set(param_find("COM_ARM_ODID"), &value), PX4_OK);
	}

	void publishCAN(hrt_abstime timestamp, uint8_t status)
	{
		open_drone_id_arm_status_s arm_status{};
		arm_status.timestamp = timestamp;
		arm_status.status = status;
		ASSERT_TRUE(_can_pub.publish(arm_status));
	}

	void publishMAVLink(bool healthy)
	{
		telemetry_status_s telemetry{};
		telemetry.timestamp = hrt_absolute_time();
		telemetry.heartbeat_type_open_drone_id = true;
		telemetry.open_drone_id_system_healthy = healthy;
		ASSERT_TRUE(_telemetry_pub.publish(telemetry));
	}

	void update(hrt_abstime now) { _commander->updateOpenDroneIDStatus(now); }
	void updateDataLinks() { _commander->dataLinkCheck(); }
	bool usingCAN() const { return _commander->_open_drone_id_can_seen; }
	const vehicle_status_s &status() const { return _commander->_vehicle_status; }
	bool armingCheckUnhealthy() const { return _commander->_failsafe_flags.remote_id_unhealthy; }

	void checkRID(bool can_arm, bool error, bool warning = false)
	{
		OpenDroneIDChecks check;
		check.updateParams();
		Report reporter{_failsafe_flags, 0};
		Context context{status()};
		check.checkAndReport(context, reporter);
		EXPECT_EQ(reporter.armingCheckResults().can_arm == NavModes::All, can_arm);
		EXPECT_EQ(bool(reporter.healthResults().error & health_component_t::open_drone_id), error);
		EXPECT_EQ(bool(reporter.healthResults().warning & health_component_t::open_drone_id), warning);
		EXPECT_EQ(_failsafe_flags.remote_id_unhealthy,
			  !status().open_drone_id_system_present || !status().open_drone_id_system_healthy);
	}

	transition_result_t arm(arm_disarm_reason_t reason = arm_disarm_reason_t::command_external, bool checks = true)
	{
		return _commander->arm(reason, checks);
	}

	void recentlyDisarmed() { _commander->_last_disarmed_timestamp = hrt_absolute_time(); }

	failsafe_flags_s _failsafe_flags{};

private:

	int32_t _saved_com_arm_odid{};
	uORB::Publication<open_drone_id_arm_status_s> _can_pub{ORB_ID(open_drone_id_arm_status)};
	uORB::Publication<telemetry_status_s> _telemetry_pub{ORB_ID(telemetry_status)};
	std::unique_ptr<Commander> _commander;
};

TEST_F(CommanderRemoteIDTest, MissingStatusBlocksArming)
{
	update(10_s);
	EXPECT_FALSE(usingCAN());
	EXPECT_FALSE(status().open_drone_id_system_present);
	EXPECT_FALSE(status().open_drone_id_system_healthy);
	checkRID(false, true);
}

TEST_F(CommanderRemoteIDTest, InvalidTimestampsDoNotSelectCAN)
{
	publishCAN(0, 0);
	update(10_s);
	EXPECT_FALSE(usingCAN());

	publishCAN(11_s, 0);
	update(10_s);
	EXPECT_FALSE(usingCAN());
	checkRID(false, true);

	publishMAVLink(true);
	updateDataLinks();
	checkRID(true, false);
	EXPECT_FALSE(usingCAN());
}

TEST_F(CommanderRemoteIDTest, CANFaultUnknownStatusAndRecovery)
{
	hrt_abstime now = 10_s;
	publishCAN(now, 0);
	update(now);
	EXPECT_TRUE(usingCAN());
	EXPECT_TRUE(status().open_drone_id_system_present);
	EXPECT_TRUE(status().open_drone_id_system_healthy);
	checkRID(true, false);

	const uint8_t failed_statuses[] = {1, UINT8_MAX};

	for (uint8_t failed_status : failed_statuses) {
		SCOPED_TRACE(failed_status);
		now += 1_s;
		publishCAN(now, failed_status);
		update(now);
		EXPECT_TRUE(status().open_drone_id_system_present);
		EXPECT_FALSE(status().open_drone_id_system_healthy);
		checkRID(false, true);
	}

	now += 1_s;
	publishCAN(now, 0);
	update(now);
	checkRID(true, false);
}

TEST_F(CommanderRemoteIDTest, TimeoutBoundaryAndRecovery)
{
	publishCAN(10_s, 0);
	update(10_s);
	update(13_s);
	checkRID(true, false);

	update(13_s + 1_us);
	EXPECT_TRUE(usingCAN());
	EXPECT_FALSE(status().open_drone_id_system_present);
	EXPECT_FALSE(status().open_drone_id_system_healthy);
	checkRID(false, true);

	publishCAN(14_s, 1);
	update(14_s);
	EXPECT_TRUE(status().open_drone_id_system_present);
	checkRID(false, true);

	publishCAN(15_s, 0);
	update(15_s);
	checkRID(true, false);
}

TEST_F(CommanderRemoteIDTest, FirstCANFaultTakesPrecedenceOverMAVLink)
{
	publishMAVLink(true);
	publishCAN(hrt_absolute_time(), 1);
	updateDataLinks();
	EXPECT_TRUE(usingCAN());
	EXPECT_TRUE(status().open_drone_id_system_present);
	checkRID(false, true);

	publishMAVLink(true);
	updateDataLinks();
	checkRID(false, true);
}

TEST_F(CommanderRemoteIDTest, FirstStaleCANMessagePreventsMAVLinkFallback)
{
	const hrt_abstime now = hrt_absolute_time();
	ASSERT_GT(now, 4_s);
	publishMAVLink(true);
	publishCAN(now - 4_s, 0);
	updateDataLinks();
	EXPECT_TRUE(usingCAN());
	EXPECT_FALSE(status().open_drone_id_system_present);
	checkRID(false, true);

	publishMAVLink(true);
	updateDataLinks();
	checkRID(false, true);
}

TEST_F(CommanderRemoteIDTest, HealthyMAVLinkCannotMaskCANTimeout)
{
	const hrt_abstime now = hrt_absolute_time();
	ASSERT_GT(now, 4_s);
	const hrt_abstime timestamp = now - 4_s;
	publishCAN(timestamp, 0);
	update(timestamp);
	checkRID(true, false);

	publishMAVLink(true);
	updateDataLinks();
	EXPECT_FALSE(status().open_drone_id_system_present);
	checkRID(false, true);
}

TEST_F(CommanderRemoteIDTest, MAVLinkHealthTimeoutAndRecoveryRemainSupported)
{
	publishMAVLink(true);
	updateDataLinks();
	checkRID(true, false);

	publishMAVLink(false);
	updateDataLinks();
	EXPECT_TRUE(status().open_drone_id_system_present);
	checkRID(false, true);

	update(hrt_absolute_time() + 3_s + 1_us);
	EXPECT_FALSE(status().open_drone_id_system_present);
	checkRID(false, true);

	publishMAVLink(true);
	updateDataLinks();
	checkRID(true, false);
	EXPECT_FALSE(usingCAN());
}

TEST_F(CommanderRemoteIDTest, DisabledWarningAndErrorPoliciesRemainSupported)
{
	for (int32_t policy = 0; policy <= 2; ++policy) {
		SCOPED_TRACE(policy);
		setArmingCheck(policy);
		checkRID(policy < 2, policy == 2, policy == 1);
	}

	publishCAN(10_s, 0);
	update(10_s);

	for (int32_t policy = 0; policy <= 2; ++policy) {
		setArmingCheck(policy);
		checkRID(true, false);
	}

	publishCAN(11_s, 1);
	update(11_s);

	for (int32_t policy = 0; policy <= 2; ++policy) {
		SCOPED_TRACE(policy);
		setArmingCheck(policy);
		checkRID(policy < 2, policy == 2, policy == 1);
	}
}

TEST_F(CommanderRemoteIDTest, ArmingRequestConsumesNewCANFault)
{
	publishCAN(hrt_absolute_time(), 0);
	updateDataLinks();
	checkRID(true, false);
	publishCAN(hrt_absolute_time(), 1);

	EXPECT_EQ(arm(), TRANSITION_DENIED);
	EXPECT_FALSE(status().open_drone_id_system_healthy);
	EXPECT_TRUE(armingCheckUnhealthy());
}

TEST_F(CommanderRemoteIDTest, ArmingRequestRejectsExpiredCANHealth)
{
	const hrt_abstime now = hrt_absolute_time();
	ASSERT_GT(now, 4_s);
	const hrt_abstime timestamp = now - 4_s;
	publishCAN(timestamp, 0);
	update(timestamp);
	checkRID(true, false);

	EXPECT_EQ(arm(), TRANSITION_DENIED);
	EXPECT_FALSE(status().open_drone_id_system_present);
	EXPECT_TRUE(armingCheckUnhealthy());
}

TEST_F(CommanderRemoteIDTest, ArmingRequestConsumesCANRecovery)
{
	publishCAN(hrt_absolute_time(), 1);
	updateDataLinks();
	checkRID(false, true);
	publishCAN(hrt_absolute_time(), 0);

	// Other preflight checks still apply; only the RID failure must clear.
	arm();
	EXPECT_TRUE(status().open_drone_id_system_healthy);
	EXPECT_FALSE(armingCheckUnhealthy());
	checkRID(true, false);
}

TEST_F(CommanderRemoteIDTest, ExistingRearmingGracePeriodIsPreserved)
{
	publishCAN(hrt_absolute_time(), 1);
	updateDataLinks();
	checkRID(false, true);
	recentlyDisarmed();

	EXPECT_EQ(arm(arm_disarm_reason_t::rc_switch), TRANSITION_CHANGED);
	EXPECT_EQ(status().arming_state, vehicle_status_s::ARMING_STATE_ARMED);
}

TEST_F(CommanderRemoteIDTest, ExplicitPreflightBypassIsPreserved)
{
	checkRID(false, true);
	EXPECT_EQ(arm(arm_disarm_reason_t::command_internal, false), TRANSITION_CHANGED);
	EXPECT_EQ(status().arming_state, vehicle_status_s::ARMING_STATE_ARMED);
}

TEST_F(CommanderRemoteIDTest, ErrorOnlyPolicyDoesNotChangeFlightAction)
{
	EXPECT_EQ(arm(arm_disarm_reason_t::command_internal, false), TRANSITION_CHANGED);
	publishCAN(10_s, 0);
	update(10_s);
	checkRID(true, false);

	Failsafe failsafe{nullptr};
	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	bool stick_override_request = false;
	failsafe.update(10_s, state, false, stick_override_request, _failsafe_flags);
	EXPECT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);

	publishCAN(11_s, 1);
	update(11_s);
	checkRID(false, true);
	EXPECT_EQ(status().arming_state, vehicle_status_s::ARMING_STATE_ARMED);
	EXPECT_EQ(failsafe.update(11_s, state, false, stick_override_request, _failsafe_flags), state.user_intended_mode);
	EXPECT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);
}
