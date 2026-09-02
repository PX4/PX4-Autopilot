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
#include "ModeManagement.hpp"
#include "HealthAndArmingChecks/checks/externalChecks.hpp"
#include <uORB/uORBManager.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/unregister_ext_component.h>

static bool modeValid(uint8_t mode)
{
	return mode >= Modes::FIRST_EXTERNAL_NAV_STATE && mode <= Modes::LAST_EXTERNAL_NAV_STATE;
}

static int32_t readHash(int idx)
{
	char buffer[20];
	snprintf(buffer, sizeof(buffer), "COM_MODE%u_HASH", idx);
	param_t param = param_find(buffer);
	int32_t value{};
	param_get(param, &value);
	return value;
}

TEST(ModeManagementTest, Hashes)
{
	param_control_autosave(false);

	// Reset parameters
	for (int i = 0; i < Modes::MAX_NUM; ++i) {
		char buffer[20];
		snprintf(buffer, sizeof(buffer), "COM_MODE%u_HASH", i);
		param_t param = param_find(buffer);
		param_reset(param);
	}

	// Add full set of modes, which stores the hashes
	Modes modes;
	Modes::Mode mode;

	for (int i = 0; i < Modes::MAX_NUM; ++i) {
		snprintf(mode.name, sizeof(mode.name), "mode %i", i);
		EXPECT_EQ(modes.addExternalMode(mode), Modes::FIRST_EXTERNAL_NAV_STATE + i);
		EXPECT_EQ(readHash(i), events::util::hash_32_fnv1a_const(mode.name));
	}

	EXPECT_FALSE(modes.hasFreeExternalModes());

	// Remove all modes, except last
	for (int i = 0; i < Modes::MAX_NUM - 1; ++i) {
		snprintf(mode.name, sizeof(mode.name), "mode %i", i);
		EXPECT_TRUE(modes.removeExternalMode(Modes::FIRST_EXTERNAL_NAV_STATE + i, mode.name));
	}

	// Add some mode, ensure it gets the same index
	const int mode_to_add_idx = 3;
	snprintf(mode.name, sizeof(mode.name), "mode %i", mode_to_add_idx);
	EXPECT_EQ(modes.addExternalMode(mode), Modes::FIRST_EXTERNAL_NAV_STATE + mode_to_add_idx);

	// Duplicate name registration must fail, stale entries are evicted in checkNewRegistrations()
	// before addExternalMode() is called, so a valid slot with the same name is a logic error.
	uint8_t added_mode_nav_state = modes.addExternalMode(mode);
	EXPECT_FALSE(modeValid(added_mode_nav_state));

	// 2 Modes are used now. Add N-2 new ones which must overwrite previous hashes
	for (int i = 0; i < Modes::MAX_NUM - 2; ++i) {
		snprintf(mode.name, sizeof(mode.name), "new mode %i", i);
		added_mode_nav_state = modes.addExternalMode(mode);
		EXPECT_TRUE(modeValid(added_mode_nav_state));
		EXPECT_EQ(readHash(added_mode_nav_state - Modes::FIRST_EXTERNAL_NAV_STATE),
			  events::util::hash_32_fnv1a_const(mode.name));
	}

	EXPECT_FALSE(modes.hasFreeExternalModes());
}

// ---------------------------------------------------------------------------
// Registration fixture, tests for ModeManagement::checkNewRegistrations()
// ---------------------------------------------------------------------------

static constexpr uint8_t kVehicleType = 0; // multirotor
static constexpr uint8_t kLoiter = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

class ModeManagementRegistrationTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		param_control_autosave(false);

		for (int i = 0; i < Modes::MAX_NUM; ++i) {
			char buf[20];
			snprintf(buf, sizeof(buf), "COM_MODE%u_HASH", i);
			param_reset(param_find(buf));
		}

		orb_advertise(ORB_ID(register_ext_component_request), nullptr);
		orb_advertise(ORB_ID(unregister_ext_component), nullptr);
		orb_advertise(ORB_ID(arming_check_reply), nullptr);

		external_checks = new ExternalChecks();
		mode_management = new ModeManagement(*external_checks);
		request_pub = new uORB::Publication<register_ext_component_request_s>(ORB_ID(register_ext_component_request));
		unreg_pub = new uORB::Publication<unregister_ext_component_s>(ORB_ID(unregister_ext_component));
		reply_sub = new uORB::Subscription(ORB_ID(register_ext_component_reply));
		drainStaleMessages();
	}

	// Flush stale request (from previous test) that the new ModeManagement subscription would
	// otherwise re-process, and drain the resulting replies so every test starts clean.
	void drainStaleMessages()
	{
		ModeManagement::UpdateRequest dummy{};
		mode_management->update(kVehicleType, false, kLoiter, dummy);
		register_ext_component_reply_s stale{};

		while (reply_sub->update(&stale)) {}
	}

	void TearDown() override
	{
		delete reply_sub;
		delete unreg_pub;
		delete request_pub;
		delete mode_management;
		delete external_checks;
	}

	register_ext_component_reply_s doRegister(const char *name,
			uint8_t nav_state = kLoiter,
			bool with_executor = true)
	{
		register_ext_component_request_s req{};
		strncpy(req.name, name, sizeof(req.name));
		req.register_mode = true;
		req.register_arming_check = true;
		req.register_mode_executor = with_executor;
		req.request_id = next_request_id++;
		req.timestamp = hrt_absolute_time();
		request_pub->publish(req);

		last_update = {};
		mode_management->update(kVehicleType, false, nav_state, last_update);

		register_ext_component_reply_s reply{};
		reply_sub->update(&reply);
		return reply;
	}

	void doUnregister(const register_ext_component_reply_s &reply, const char *name, uint8_t nav_state)
	{
		unregister_ext_component_s req{};
		strncpy(req.name, name, sizeof(req.name));
		req.mode_id = reply.mode_id;
		req.mode_executor_id = reply.mode_executor_id;
		req.arming_check_id = reply.arming_check_id;
		req.timestamp = hrt_absolute_time();
		unreg_pub->publish(req);
		last_update = {};
		mode_management->update(kVehicleType, false, nav_state, last_update);
	}

	ExternalChecks *external_checks{};
	ModeManagement *mode_management{};
	uORB::Publication<register_ext_component_request_s> *request_pub{};
	uORB::Publication<unregister_ext_component_s> *unreg_pub{};
	uORB::Subscription *reply_sub{};

	uint64_t next_request_id{1};
	ModeManagement::UpdateRequest last_update{};
};

// T1: baseline, first registration gets deterministic slot 0
TEST_F(ModeManagementRegistrationTest, FirstRegistration)
{
	auto reply = doRegister("mode A");
	EXPECT_TRUE(reply.success);
	EXPECT_EQ(reply.mode_id, Modes::FIRST_EXTERNAL_NAV_STATE);
	EXPECT_EQ(reply.mode_executor_id, ModeExecutors::FIRST_EXECUTOR_ID);
	EXPECT_EQ(reply.arming_check_id, 0);
	EXPECT_FALSE(last_update.change_user_intended_nav_state);
}

// T2: two different names coexist, no eviction
TEST_F(ModeManagementRegistrationTest, TwoDifferentModes)
{
	auto reply_A = doRegister("mode A");
	auto reply_B = doRegister("mode B");
	EXPECT_TRUE(reply_A.success);
	EXPECT_TRUE(reply_B.success);
	EXPECT_EQ(reply_A.mode_id, Modes::FIRST_EXTERNAL_NAV_STATE);
	EXPECT_EQ(reply_B.mode_id, Modes::FIRST_EXTERNAL_NAV_STATE + 1);
	EXPECT_NE(reply_A.mode_executor_id, reply_B.mode_executor_id);
	EXPECT_NE(reply_A.arming_check_id, reply_B.arming_check_id);
	EXPECT_FALSE(last_update.change_user_intended_nav_state);
}

// T3: fast restart, mode not active, same ids returned, executor in charge unchanged
TEST_F(ModeManagementRegistrationTest, FastRestartModeNotActive)
{
	auto reply1 = doRegister("mode A", kLoiter);
	auto reply2 = doRegister("mode A", kLoiter);

	EXPECT_TRUE(reply2.success);
	EXPECT_EQ(reply2.mode_id, reply1.mode_id);
	EXPECT_EQ(reply2.mode_executor_id, reply1.mode_executor_id);
	EXPECT_EQ(reply2.arming_check_id, reply1.arming_check_id);
	EXPECT_FALSE(last_update.change_user_intended_nav_state);
	EXPECT_EQ(mode_management->modeExecutorInCharge(), ModeExecutors::AUTOPILOT_EXECUTOR_ID);
}

// T4: fast restart while mode is active, executor handed over within the same update() call
TEST_F(ModeManagementRegistrationTest, FastRestartModeActive)
{
	auto reply1 = doRegister("mode A", kLoiter);
	mode_management->onUserIntendedNavStateChange(ModeChangeSource::User, reply1.mode_id);
	EXPECT_EQ(mode_management->modeExecutorInCharge(), reply1.mode_executor_id);

	auto reply2 = doRegister("mode A", reply1.mode_id);

	EXPECT_TRUE(reply2.success);
	EXPECT_EQ(reply2.mode_id, reply1.mode_id);
	EXPECT_EQ(reply2.mode_executor_id, reply1.mode_executor_id);
	EXPECT_EQ(reply2.arming_check_id, reply1.arming_check_id);
	EXPECT_EQ(mode_management->modeExecutorInCharge(), reply2.mode_executor_id); // restored immediately
	EXPECT_FALSE(last_update.change_user_intended_nav_state); // no spurious mode switch
}

// T5: eviction runs before free-slot checks, restart succeeds even when executor pool appears full
TEST_F(ModeManagementRegistrationTest, EvictionBeforeFreeSlotCheck)
{
	auto reply_A = doRegister("mode A", kLoiter); // executor 1
	doRegister("mode 1", kLoiter);                // executor 2
	doRegister("mode 2", kLoiter);                // executor 3
	doRegister("mode 3", kLoiter);                // executor 4
	doRegister("mode 4", kLoiter);                // executor 5, pool full

	auto reply_A2 = doRegister("mode A", kLoiter);

	EXPECT_TRUE(reply_A2.success);
	EXPECT_EQ(reply_A2.mode_id, reply_A.mode_id);
	EXPECT_EQ(reply_A2.mode_executor_id, reply_A.mode_executor_id);
	EXPECT_EQ(reply_A2.arming_check_id, reply_A.arming_check_id);
}

// T6: active mode evicted, restart escalates to executor with full pool, AUTO_LOITER fallback fires
TEST_F(ModeManagementRegistrationTest, ActiveModeEvictedRestartFails)
{
	auto reply_A = doRegister("mode A", kLoiter, false);
	doRegister("mode 1", kLoiter); // executor 1
	doRegister("mode 2", kLoiter); // executor 2
	doRegister("mode 3", kLoiter); // executor 3
	doRegister("mode 4", kLoiter); // executor 4
	doRegister("mode 5", kLoiter); // executor 5, pool full

	mode_management->onUserIntendedNavStateChange(ModeChangeSource::User, reply_A.mode_id);

	auto reply_A2 = doRegister("mode A", reply_A.mode_id, true);

	EXPECT_FALSE(reply_A2.success);
	EXPECT_TRUE(last_update.change_user_intended_nav_state);
	EXPECT_EQ(last_update.user_intended_nav_state, (uint8_t)vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
}

// T7: clean unregister of active mode still triggers AUTO_LOITER fallback (regression)
TEST_F(ModeManagementRegistrationTest, CleanUnregisterActiveModeTriggersLoiterFallback)
{
	auto reply = doRegister("mode A", kLoiter);
	doUnregister(reply, "mode A", reply.mode_id);

	EXPECT_TRUE(last_update.change_user_intended_nav_state);
	EXPECT_EQ(last_update.user_intended_nav_state, (uint8_t)vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
}

// T8: arming-check-only request, eviction loop not entered, two registrations coexist
TEST_F(ModeManagementRegistrationTest, ArmingCheckOnlyNoEviction)
{
	auto doArmingCheckRegister = [&]() -> register_ext_component_reply_s {
		register_ext_component_request_s req{};
		strncpy(req.name, "check A", sizeof(req.name));
		req.register_arming_check = true;
		req.register_mode = false;
		req.register_mode_executor = false;
		req.request_id = next_request_id++;
		req.timestamp = hrt_absolute_time();
		request_pub->publish(req);
		last_update = {};
		mode_management->update(kVehicleType, false, kLoiter, last_update);
		register_ext_component_reply_s reply{};
		reply_sub->update(&reply);
		return reply;
	};

	auto reply1 = doArmingCheckRegister();
	auto reply2 = doArmingCheckRegister();

	EXPECT_TRUE(reply1.success);
	EXPECT_TRUE(reply2.success);
	EXPECT_EQ(reply1.arming_check_id, 0);
	EXPECT_EQ(reply2.arming_check_id, 1); // separate slot, no eviction
}
