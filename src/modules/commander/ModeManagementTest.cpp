/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/vehicle_rates_setpoint.h>

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

	// Try to add another one with the same name: should succeed, with the hash of the added index reset
	uint8_t added_mode_nav_state = modes.addExternalMode(mode);
	EXPECT_EQ(readHash(added_mode_nav_state - Modes::FIRST_EXTERNAL_NAV_STATE), 0);

	// 3 Modes are used now. Add N-3 new ones which must overwrite previous hashes
	for (int i = 0; i < Modes::MAX_NUM - 3; ++i) {
		snprintf(mode.name, sizeof(mode.name), "new mode %i", i);
		added_mode_nav_state = modes.addExternalMode(mode);
		EXPECT_TRUE(modeValid(added_mode_nav_state));
		EXPECT_EQ(readHash(added_mode_nav_state - Modes::FIRST_EXTERNAL_NAV_STATE),
			  events::util::hash_32_fnv1a_const(mode.name));
	}

	EXPECT_FALSE(modes.hasFreeExternalModes());
}

TEST(ModeManagementTest, VtolFixedwingSetpointRegistration)
{
	using mode_util::SetpointType;
	using mode_util::SetpointTypeResult;

	// A plain (non-VTOL) rotary wing cannot register the fixed-wing setpoint type.
	EXPECT_EQ(mode_util::isSetpointTypeValid(SetpointType::FixedwingLateralLongitudinal,
			vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false),
		  SetpointTypeResult::Unsupported);

	// A VTOL currently in multicopter form must still be able to register it, so it
	// can command its own first transition to fixed-wing from an external mode.
	EXPECT_EQ(mode_util::isSetpointTypeValid(SetpointType::FixedwingLateralLongitudinal,
			vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true),
		  SetpointTypeResult::Success);

	// Already fixed-wing: valid regardless of is_vtol.
	EXPECT_EQ(mode_util::isSetpointTypeValid(SetpointType::FixedwingLateralLongitudinal,
			vehicle_status_s::VEHICLE_TYPE_FIXED_WING, false),
		  SetpointTypeResult::Success);
}

TEST(ModeManagementTest, SetpointTimeout)
{
	// A mode that keeps answering the arming checks but stops sending setpoints (a stuck control thread)
	// is flagged once the timeout it configured expires, so the failsafe switches away from it
	param_control_autosave(false); // registering a mode sets a parameter

	ExternalChecks external_checks;
	ModeManagement mode_management{external_checks};

	uORB::Publication<register_ext_component_request_s> register_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Subscription register_reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Publication<unregister_ext_component_s> unregister_pub{ORB_ID(unregister_ext_component)};
	uORB::Subscription arming_check_request_sub{ORB_ID(arming_check_request)};
	uORB::Publication<arming_check_reply_s> arming_check_reply_pub{ORB_ID(arming_check_reply)};
	uORB::Publication<setpoint_config_s> setpoint_config_pub{ORB_ID(setpoint_config)};
	uORB::Publication<vehicle_rates_setpoint_s> rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<actuator_servos_s> servos_pub{ORB_ID(actuator_servos)};

	// Commander time is injected, the test does not depend on the wall clock
	hrt_abstime now = 10_s;

	// Commander iterations every 10 ms. The modes keep answering the arming checks, so they stay responsive.
	// The requests follow the wall clock (every 300 ms), so answer the last request for every registration in each
	// iteration, which also covers modes registered after the last request.
	uint8_t arming_check_request_id = 0;

	// From the land detector, as passed by Commander
	bool landed = false;

	auto run = [&](hrt_abstime duration, bool armed, uint8_t intention, uint8_t nav_state, bool send_setpoints) {
		const hrt_abstime end = now + duration;

		while (now < end) {
			now += 10_ms;

			if (send_setpoints) {
				vehicle_rates_setpoint_s setpoint{};
				setpoint.timestamp = hrt_absolute_time();
				rates_setpoint_pub.publish(setpoint);
			}

			ModeManagement::UpdateRequest update_request{};
			mode_management.update(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false, armed, landed, intention, nav_state,
					       now, update_request);
			mode_management.getSetpointType(nav_state); // as Commander::updateControlMode()

			arming_check_request_s request;

			if (arming_check_request_sub.update(&request)) {
				arming_check_request_id = request.request_id;
			}

			for (int i = 0; i < ExternalChecks::MAX_NUM_REGISTRATIONS; ++i) {
				arming_check_reply_s reply{};
				reply.timestamp = hrt_absolute_time();
				reply.request_id = arming_check_request_id;
				reply.registration_id = i;
				reply.can_arm_and_run = true;
				arming_check_reply_pub.publish(reply);
			}
		}
	};

	auto registerMode = [&](uint64_t request_id, const char *name, uint8_t replaced_nav_state) -> uint8_t {
		register_ext_component_request_s request{};
		request.timestamp = hrt_absolute_time();
		request.request_id = request_id;
		snprintf(request.name, sizeof(request.name), "%s", name);
		request.register_arming_check = true;
		request.register_mode = true;
		request.enable_replace_internal_mode = replaced_nav_state != Modes::Mode::REPLACES_NAV_STATE_NONE;
		request.replace_internal_mode = replaced_nav_state;
		register_request_pub.publish(request);
		run(10_ms, false, vehicle_status_s::NAVIGATION_STATE_POSCTL, vehicle_status_s::NAVIGATION_STATE_POSCTL, false);

		// uORB data outlives a test, skip replies to other requests
		register_ext_component_reply_s reply{};

		while (register_reply_sub.update(&reply) && reply.request_id != request_id) {}

		EXPECT_EQ(reply.request_id, request_id);
		EXPECT_TRUE(reply.success);
		return reply.mode_id;
	};

	auto configureSetpoints = [&](uint8_t nav_state, uint16_t type, uint16_t timeout_ms) {
		setpoint_config_s config{};
		config.timestamp = hrt_absolute_time();
		config.type = type;
		config.source_id = nav_state;
		config.should_apply = true;
		config.timeout_ms = timeout_ms;
		setpoint_config_pub.publish(config);
	};

	// The bit that makes the failsafe switch away from a mode
	auto modeReqOther = [&](uint8_t nav_state) {
		failsafe_flags_s failsafe_flags{};
		Report reporter{failsafe_flags, 0_s};
		vehicle_status_s status{};
		external_checks.checkAndReport(Context{status}, reporter);
		return (failsafe_flags.mode_req_other & (1u << nav_state)) != 0;
	};

	// A failing arming check for the mode, which gives the failsafe and a refused selection of the mode their reason
	auto armingCheckFails = [&](uint8_t nav_state) {
		failsafe_flags_s failsafe_flags{};
		Report reporter{failsafe_flags, 0_s};
		vehicle_status_s status{};
		external_checks.checkAndReport(Context{status}, reporter);
		return ((uint32_t)reporter.armingCheckResults().can_arm & (1u << nav_state)) == 0;
	};

	const uint8_t posctl = vehicle_status_s::NAVIGATION_STATE_POSCTL;
	const uint8_t rtl = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

	const uint8_t mode = registerMode(1, "setpoint timeout", Modes::Mode::REPLACES_NAV_STATE_NONE);
	ASSERT_TRUE(modeValid(mode));
	run(20_ms, false, mode, mode, false); // answer the first arming check request
	EXPECT_FALSE(modeReqOther(mode));

	// Timeout 0, what modes send today: never flagged
	configureSetpoints(mode, setpoint_config_s::TYPE_RATES, 0);
	run(10_s, true, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));

	// 100 ms timeout: not flagged while setpoints arrive
	configureSetpoints(mode, setpoint_config_s::TYPE_RATES, 100);
	run(500_ms, true, mode, mode, true);
	EXPECT_FALSE(modeReqOther(mode));

	// Setpoints stop, arming checks are still answered: flagged once more than 100 ms passed
	run(100_ms, true, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));
	EXPECT_FALSE(armingCheckFails(mode));
	run(10_ms, true, mode, mode, false);
	EXPECT_TRUE(modeReqOther(mode));
	EXPECT_TRUE(armingCheckFails(mode));

	// Stays flagged when setpoints arrive again, before and after the failsafe switched to Return
	run(10_ms, true, mode, mode, true);
	EXPECT_TRUE(modeReqOther(mode));
	run(1_s, true, mode, rtl, true);
	EXPECT_TRUE(modeReqOther(mode));

	// Cleared when the pilot selects another mode
	run(10_ms, true, posctl, posctl, false);
	EXPECT_FALSE(modeReqOther(mode));
	EXPECT_FALSE(armingCheckFails(mode));

	// Selecting the mode again starts a new timeout (a mode sends its configuration when activated)
	configureSetpoints(mode, setpoint_config_s::TYPE_RATES, 100);
	run(100_ms, true, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));

	// A new configuration restarts it. With the attitude type, rates setpoints do not count.
	configureSetpoints(mode, setpoint_config_s::TYPE_ATTITUDE, 100);
	run(110_ms, true, mode, mode, true);
	EXPECT_FALSE(modeReqOther(mode));
	run(10_ms, true, mode, mode, true);
	EXPECT_TRUE(modeReqOther(mode));

	// Cleared on disarm
	run(10_ms, false, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));

	// A replacement mode falls back to the mode it replaces until the pilot selects another mode or disarms
	const uint8_t hold = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
	const uint8_t replacement = registerMode(2, "setpoint timeout replacement", hold);
	ASSERT_TRUE(modeValid(replacement));
	EXPECT_EQ(mode_management.getNavStateReplacementIfValid(hold), replacement);

	// Hold selected: like Commander, resolve the replacement for the user intention and the nav_state
	auto runHold = [&](int iterations, bool armed, bool send_setpoints) {
		for (int i = 0; i < iterations; ++i) {
			run(10_ms, armed, mode_management.getNavStateReplacementIfValid(hold, false),
			    mode_management.getNavStateReplacementIfValid(hold), send_setpoints);
		}
	};

	configureSetpoints(replacement, setpoint_config_s::TYPE_RATES, 100);
	runHold(10, true, true);
	EXPECT_EQ(mode_management.getNavStateReplacementIfValid(hold), replacement);
	EXPECT_FALSE(modeReqOther(replacement));
	runHold(20, true, false);
	EXPECT_EQ(mode_management.getNavStateReplacementIfValid(hold), hold);
	EXPECT_TRUE(modeReqOther(replacement));
	EXPECT_FALSE(modeReqOther(hold)); // the internal mode can run
	runHold(20, true, true);
	EXPECT_EQ(mode_management.getNavStateReplacementIfValid(hold), hold);
	runHold(1, false, false);
	EXPECT_EQ(mode_management.getNavStateReplacementIfValid(hold), replacement);

	// Armed again in the mode with the configuration of the previous flight: the mode first has to notice that it
	// is active again, so the timeout starts with its first setpoint
	configureSetpoints(mode, setpoint_config_s::TYPE_RATES, 100);
	run(100_ms, true, mode, mode, true);
	run(100_ms, false, mode, mode, false); // landed and disarmed in the mode
	run(500_ms, true, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));
	run(10_ms, true, mode, mode, true);
	run(100_ms, true, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));
	run(10_ms, true, mode, mode, false);
	EXPECT_TRUE(modeReqOther(mode));
	run(10_ms, false, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));

	// A replacement selected by a failsafe stays flagged until the user selects another mode. Here the failsafe
	// comes from the first mode timing out, so two modes are flagged at the same time.
	const uint8_t rtl_replacement = registerMode(3, "setpoint timeout return", rtl);
	ASSERT_TRUE(modeValid(rtl_replacement));
	configureSetpoints(mode, setpoint_config_s::TYPE_RATES, 100);
	run(100_ms, true, mode, mode, true);
	run(110_ms, true, mode, mode, false);
	EXPECT_TRUE(modeReqOther(mode));
	EXPECT_EQ(mode_management.getNavStateReplacementIfValid(rtl), rtl_replacement);

	configureSetpoints(rtl_replacement, setpoint_config_s::TYPE_RATES, 100);
	run(100_ms, true, mode, rtl_replacement, true);
	EXPECT_FALSE(modeReqOther(rtl_replacement));
	run(110_ms, true, mode, rtl_replacement, false);
	EXPECT_TRUE(modeReqOther(rtl_replacement));
	EXPECT_EQ(mode_management.getNavStateReplacementIfValid(rtl), rtl);

	run(1_s, true, mode, rtl, false);
	EXPECT_TRUE(modeReqOther(mode));
	EXPECT_TRUE(modeReqOther(rtl_replacement));
	EXPECT_EQ(mode_management.getNavStateReplacementIfValid(rtl), rtl);

	run(10_ms, true, posctl, posctl, false);
	EXPECT_FALSE(modeReqOther(mode));
	EXPECT_FALSE(modeReqOther(rtl_replacement));
	EXPECT_EQ(mode_management.getNavStateReplacementIfValid(rtl), rtl_replacement);
	run(10_ms, false, posctl, posctl, false);

	// The timeout only runs while the vehicle is not landed. A mode that landed and stopped sending setpoints is
	// left to the auto-disarm. If the vehicle is no longer landed, the timeout runs from that moment.
	configureSetpoints(mode, setpoint_config_s::TYPE_RATES, 100);
	landed = true;
	run(1_s, true, mode, mode, true);
	landed = false;
	run(10_s, true, mode, mode, true);
	landed = true;
	run(100_ms, true, mode, mode, true);
	run(10_s, true, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));
	landed = false;
	run(100_ms, true, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));
	run(10_ms, true, mode, mode, false);
	EXPECT_TRUE(modeReqOther(mode));
	run(10_ms, false, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));

	// Sending on the ground and stuck at lift-off: flagged once the timeout passed after lift-off
	configureSetpoints(mode, setpoint_config_s::TYPE_RATES, 100);
	landed = true;
	run(1_s, true, mode, mode, true);
	landed = false;
	run(100_ms, true, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));
	run(10_ms, true, mode, mode, false);
	EXPECT_TRUE(modeReqOther(mode));
	run(10_ms, false, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));

	// A rover counts as landed for 500 ms after arming. Armed with the configuration from before and no setpoint after
	// the arming, the timeout still starts with the first setpoint, also if the vehicle is no longer landed before the
	// mode reacted.
	configureSetpoints(mode, setpoint_config_s::TYPE_RATES, 100);
	landed = true;
	run(100_ms, false, mode, mode, true);
	run(500_ms, true, mode, mode, false);
	landed = false;
	run(1_s, true, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));
	run(10_ms, true, mode, mode, true);
	run(100_ms, true, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));
	run(10_ms, true, mode, mode, false);
	EXPECT_TRUE(modeReqOther(mode));
	run(10_ms, false, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));

	// Selected while disarmed with the configuration kept, the mode sends one setpoint right after the arming, before
	// the first armed update, and then hangs at lift-off. That setpoint is its first one after the arming and starts the
	// timeout, which runs once the vehicle is no longer landed.
	configureSetpoints(mode, setpoint_config_s::TYPE_RATES, 100);
	landed = true;
	run(100_ms, false, mode, mode, true);
	run(10_ms, true, mode, mode, true);
	run(1_s, true, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));
	landed = false;
	run(100_ms, true, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));
	run(10_ms, true, mode, mode, false);
	EXPECT_TRUE(modeReqOther(mode));
	run(10_ms, false, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));

	// A configuration with timeout 0 from the active mode stops the watch right away, so a mode can stop sending
	// setpoints while it stays selected (after it completed, until the next mode takes over)
	configureSetpoints(mode, setpoint_config_s::TYPE_RATES, 100);
	run(100_ms, true, mode, mode, true);
	configureSetpoints(mode, setpoint_config_s::TYPE_RATES, 0);
	run(10_s, true, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));
	configureSetpoints(mode, setpoint_config_s::TYPE_RATES, 100);
	run(110_ms, true, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));
	run(10_ms, true, mode, mode, false);
	EXPECT_TRUE(modeReqOther(mode));
	run(10_ms, false, mode, mode, false);
	EXPECT_FALSE(modeReqOther(mode));

	// Flagged in the air and landed by the failsafe: still flagged until the user selects another mode or the
	// vehicle disarms
	const uint8_t land = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
	configureSetpoints(mode, setpoint_config_s::TYPE_RATES, 100);
	run(100_ms, true, mode, mode, true);
	run(110_ms, true, mode, mode, false);
	EXPECT_TRUE(modeReqOther(mode));
	run(1_s, true, mode, land, false);
	landed = true;
	run(1_s, true, mode, land, false);
	EXPECT_TRUE(modeReqOther(mode));
	run(10_ms, true, posctl, posctl, false);
	EXPECT_FALSE(modeReqOther(mode));
	run(10_ms, false, posctl, posctl, false);

	landed = false;
	configureSetpoints(mode, setpoint_config_s::TYPE_RATES, 100);
	run(100_ms, true, mode, mode, true);
	run(110_ms, true, mode, mode, false);
	EXPECT_TRUE(modeReqOther(mode));
	landed = true;
	run(1_s, true, mode, land, false);
	EXPECT_TRUE(modeReqOther(mode));
	run(10_ms, false, mode, land, false);
	EXPECT_FALSE(modeReqOther(mode));

	// With two setpoint topics a sample on either one counts: a mode that only sends servos is not flagged
	landed = false;
	configureSetpoints(mode, setpoint_config_s::TYPE_DIRECT_ACTUATORS, 100);

	for (int i = 0; i < 50; ++i) {
		actuator_servos_s servos{};
		servos.timestamp = hrt_absolute_time();
		servos_pub.publish(servos);
		run(10_ms, true, mode, mode, false);
	}

	EXPECT_FALSE(modeReqOther(mode));
	run(110_ms, true, mode, mode, false);
	EXPECT_TRUE(modeReqOther(mode));
	run(10_ms, false, mode, mode, false);

	// A flagged mode that unregisters in the first update after the disarm leaves the flag in its freed arming check
	// registration. The next mode that gets this registration must not be flagged.
	const uint8_t removed = registerMode(5, "setpoint timeout removed", Modes::Mode::REPLACES_NAV_STATE_NONE);
	ASSERT_TRUE(modeValid(removed));
	int8_t nav_mode_id = -1;
	const int registration_id = external_checks.findByRequestId(5, nav_mode_id);
	configureSetpoints(removed, setpoint_config_s::TYPE_RATES, 100);
	run(100_ms, true, removed, removed, true);
	run(110_ms, true, removed, removed, false);
	EXPECT_TRUE(modeReqOther(removed));

	unregister_ext_component_s unregister{};
	unregister.timestamp = hrt_absolute_time();
	snprintf(unregister.name, sizeof(unregister.name), "%s", "setpoint timeout removed");
	unregister.arming_check_id = registration_id;
	unregister.mode_id = removed;
	unregister.mode_executor_id = -1;
	unregister_pub.publish(unregister);
	run(10_ms, false, removed, removed, false);

	const uint8_t next = registerMode(6, "setpoint timeout next", Modes::Mode::REPLACES_NAV_STATE_NONE);
	ASSERT_TRUE(modeValid(next));
	EXPECT_EQ(external_checks.findByRequestId(6, nav_mode_id), registration_id);
	run(20_ms, false, next, next, false); // answer the first arming check request
	EXPECT_FALSE(modeReqOther(next));

	// A setpoint published right before a mode change does not start the next mode's timeout, also on the same topic:
	// the next mode first has to notice its activation. A mode normally gets its timeout from a configuration it sends
	// once active, which starts the timeout anyway; here the second mode is configured while the first one is active
	// (PX4 applies a configuration for any registered mode), so it has a timeout at its activation without a new
	// configuration in that update.
	const uint8_t second = registerMode(4, "setpoint timeout second", Modes::Mode::REPLACES_NAV_STATE_NONE);
	ASSERT_TRUE(modeValid(second));
	landed = false;
	configureSetpoints(mode, setpoint_config_s::TYPE_RATES, 100);
	run(100_ms, true, mode, mode, true);
	configureSetpoints(second, setpoint_config_s::TYPE_RATES, 100);
	run(100_ms, true, mode, mode, true);
	run(10_ms, true, second, second, true);
	run(1_s, true, second, second, false);
	EXPECT_FALSE(modeReqOther(second));
	EXPECT_FALSE(modeReqOther(mode));
	run(10_ms, false, posctl, posctl, false);
}
