/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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

#define MODULE_NAME "rc_update"

#include <gtest/gtest.h>
#include "rc_update.h"

using namespace rc_update;

TEST(RCUpdateTest, ModeSlotUnassigned)
{
	RCUpdate rc_update;
	// GIVEN: Default configuration with no assigned mode switch
	EXPECT_EQ(rc_update._param_rc_map_fltmode.get(), 0);

	// WHEN: we update the switches two times to pass the simple outlier protection
	rc_update.UpdateManualSwitches(0);
	rc_update.UpdateManualSwitches(0);

	// THEN: we receive no mode slot
	uORB::SubscriptionData<manual_control_switches_s> manual_control_switches_sub{ORB_ID(manual_control_switches)};
	manual_control_switches_sub.update();

	EXPECT_EQ(manual_control_switches_sub.get().mode_slot, 0); // manual_control_switches_s::MODE_SLOT_NONE
}

void checkModeSlotSwitch(float channel_value, uint8_t expected_slot)
{
	RCUpdate rc_update;

	// GIVEN: First channel is configured as mode switch
	rc_update._param_rc_map_fltmode.set(1);
	EXPECT_EQ(rc_update._param_rc_map_fltmode.get(), 1);
	// GIVEN: First channel has some value
	rc_update._rc.channels[0] = channel_value;

	// WHEN: we update the switches two times to pass the simple outlier protection
	rc_update.UpdateManualSwitches(0);
	rc_update.UpdateManualSwitches(0);

	// THEN: we receive the expected mode slot
	uORB::SubscriptionData<manual_control_switches_s> manual_control_switches_sub{ORB_ID(manual_control_switches)};
	manual_control_switches_sub.update();

	EXPECT_EQ(manual_control_switches_sub.get().mode_slot, expected_slot);
}

TEST(RCUpdateTest, ModeSlotSwitchAllValues)
{
	checkModeSlotSwitch(-1.f, 1); // manual_control_switches_s::MODE_SLOT_1
	checkModeSlotSwitch(-.5f, 2); // manual_control_switches_s::MODE_SLOT_2
	checkModeSlotSwitch(-.1f, 3); // manual_control_switches_s::MODE_SLOT_3
	checkModeSlotSwitch(0.f, 4); // manual_control_switches_s::MODE_SLOT_4
	checkModeSlotSwitch(.5f, 5); // manual_control_switches_s::MODE_SLOT_5
	checkModeSlotSwitch(1.f, 6); // manual_control_switches_s::MODE_SLOT_6
}

void checkTriggerAction(uint8_t button_mask, uint8_t channel, float channel_value, uint8_t action, bool expected_state)
{
	RCUpdate rc_update;
	hrt_abstime time_elapsed {0};

	// GIVEN: Buttons are configured
	rc_update._param_rc_trig_btn_mask.set(button_mask);
	EXPECT_EQ(rc_update._param_rc_trig_btn_mask.get(), button_mask);
	// GIVEN: Trigger Channel is configured (We use trigger slot 1, which is index 0)
	rc_update._parameters.generic_trigger_chan[0] = channel;
	// GIVEN: Trigger action is configured
	rc_update._parameters.generic_trigger_action[0] = action;
	// GIVEN: Trigger Action to Channel is mapped
	rc_update._trigger_action_to_channel_mapping[action] = channel;
	// GIVEN: Give channel some value
	rc_update._rc.channels[channel - 1] = channel_value;

	// WHEN
	// - Initiate the manual switch state
	rc_update.UpdateManualSwitches(time_elapsed);
	time_elapsed += 51_ms;
	// - simulate a 51 ms elapsing since the last input
	rc_update.UpdateManualSwitches(time_elapsed);
	time_elapsed += 20_ms;
	// - Hold the RC signal for extra 20 ms
	rc_update.UpdateManualSwitches(51_ms);

	// THEN: The internal action state equals the expected state
	EXPECT_EQ(rc_update._trigger_action_states[action], expected_state);
}

TEST(RCUpdateTest, TriggerAction)
{
	const uint8_t no_buttons_mask = 0;
	const uint8_t action = RC_TRIGGER_ACTIONS::RC_TRIGGER_ACTION_GEAR;
	// Giving 1.0 (max, pressed) value for the channel should trigger the action
	checkTriggerAction(no_buttons_mask, 6, 1.0f, action, true);
	// Giving 0.0 (neutral) value for the channel should not trigger the action
	checkTriggerAction(no_buttons_mask, 6, 0.0f, action, false);
	// Giving -1.0 (default) value for the channel should not trigger the action
	checkTriggerAction(no_buttons_mask, 6, -1.0f, action, false);
}
