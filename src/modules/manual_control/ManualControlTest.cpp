/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#define MODULE_NAME "manual_control"

#include <gtest/gtest.h>
#include "ManualControl.hpp"

static constexpr uint64_t SOME_TIME = 12345678;

static constexpr uint8_t ACTION_KILL = action_request_s::ACTION_KILL;
static constexpr uint8_t ACTION_UNKILL = action_request_s::ACTION_UNKILL;
static constexpr uint8_t ACTION_VTOL_TRANSITION_TO_FIXEDWING = action_request_s::ACTION_VTOL_TRANSITION_TO_FIXEDWING;
static constexpr uint8_t ACTION_VTOL_TRANSITION_TO_MULTICOPTER =
	action_request_s::ACTION_VTOL_TRANSITION_TO_MULTICOPTER;
static constexpr uint8_t ACTION_SWITCH_MODE = action_request_s::ACTION_SWITCH_MODE;

static constexpr uint8_t NAVIGATION_STATE_MANUAL = vehicle_status_s::NAVIGATION_STATE_MANUAL;
static constexpr uint8_t NAVIGATION_STATE_ALTCTL = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
static constexpr uint8_t NAVIGATION_STATE_POSCTL = vehicle_status_s::NAVIGATION_STATE_POSCTL;
static constexpr uint8_t NAVIGATION_STATE_AUTO_MISSION = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
static constexpr uint8_t NAVIGATION_STATE_AUTO_LOITER = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
static constexpr uint8_t NAVIGATION_STATE_ACRO = vehicle_status_s::NAVIGATION_STATE_ACRO;

class TestManualControl : public ManualControl
{
public:
	void processInput(hrt_abstime now) { ManualControl::processInput(now); }
	static int8_t navStateFromParam(int32_t param_value) { return ManualControl::navStateFromParam(param_value); }
};

class SwitchTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		// Disable autosaving parameters to avoid busy loop in param_set()
		param_control_autosave(false);

		// Set stick input timeout to half a second
		const float com_rc_loss_t = .5f;
		param_set(param_find("COM_RC_LOSS_T"), &com_rc_loss_t);

		int32_t mode = NAVIGATION_STATE_ACRO;
		param_set(param_find("COM_FLTMODE1"), &mode);
		mode = NAVIGATION_STATE_MANUAL;
		param_set(param_find("COM_FLTMODE2"), &mode);
		mode = NAVIGATION_STATE_ALTCTL;
		param_set(param_find("COM_FLTMODE3"), &mode);
		mode = NAVIGATION_STATE_POSCTL;
		param_set(param_find("COM_FLTMODE4"), &mode);
		mode = NAVIGATION_STATE_AUTO_LOITER;
		param_set(param_find("COM_FLTMODE5"), &mode);
		mode = NAVIGATION_STATE_AUTO_MISSION;
		param_set(param_find("COM_FLTMODE6"), &mode);
	}

	uORB::Publication<manual_control_switches_s> _manual_control_switches_pub{ORB_ID(manual_control_switches)};
	uORB::Publication<manual_control_setpoint_s> _manual_control_input_pub{ORB_ID(manual_control_input)};
	uORB::SubscriptionData<manual_control_setpoint_s> _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::SubscriptionData<action_request_s> _action_request_sub{ORB_ID(action_request)};

	TestManualControl _manual_control;
	hrt_abstime _timestamp{SOME_TIME};
};


TEST_F(SwitchTest, KillSwitch)
{
	// GIVEN: valid stick input from RC
	_manual_control_input_pub.publish({.timestamp_sample = _timestamp, .valid = true, .data_source = manual_control_setpoint_s::SOURCE_RC});
	// and kill switch in off position
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .kill_switch = manual_control_switches_s::SWITCH_POS_ON});
	_manual_control.processInput(_timestamp += 100_ms);

	// THEN: the stick input is published for use
	EXPECT_TRUE(_manual_control_setpoint_sub.update());
	EXPECT_TRUE(_manual_control_setpoint_sub.get().valid);
	// but there is no action requested
	EXPECT_FALSE(_action_request_sub.update());

	// WHEN: the kill switch is switched on
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .kill_switch = manual_control_switches_s::SWITCH_POS_OFF});
	_manual_control.processInput(_timestamp += 100_ms);

	// THEN: a kill action request is published
	EXPECT_TRUE(_action_request_sub.update());
	EXPECT_EQ(_action_request_sub.get().action, ACTION_UNKILL);

	// WHEN: the kill switch is switched off again
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .kill_switch = manual_control_switches_s::SWITCH_POS_ON});
	_manual_control.processInput(_timestamp += 100_ms);

	// THEN: an unkill action request is published
	EXPECT_TRUE(_action_request_sub.update());
	EXPECT_EQ(_action_request_sub.get().action, ACTION_KILL);
}

TEST_F(SwitchTest, TransitionSwitch)
{
	// GIVEN: valid stick input from RC
	_manual_control_input_pub.publish({.timestamp_sample = _timestamp, .valid = true, .data_source = manual_control_setpoint_s::SOURCE_RC});
	// and transition switch in off position
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .transition_switch = manual_control_switches_s::SWITCH_POS_OFF});
	_manual_control.processInput(_timestamp += 100_ms);

	// THEN: the stick input is published for use
	EXPECT_TRUE(_manual_control_setpoint_sub.update());
	EXPECT_TRUE(_manual_control_setpoint_sub.get().valid);
	// but there is no action requested
	EXPECT_FALSE(_action_request_sub.update());

	// WHEN: the transition switch is switched on
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .transition_switch = manual_control_switches_s::SWITCH_POS_ON});
	_manual_control.processInput(_timestamp += 100_ms);

	// THEN: a forward transition action request is published
	EXPECT_TRUE(_action_request_sub.update());
	EXPECT_EQ(_action_request_sub.get().action, ACTION_VTOL_TRANSITION_TO_FIXEDWING);

	// WHEN: the kill switch is switched off again
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .transition_switch = manual_control_switches_s::SWITCH_POS_OFF});
	_manual_control.processInput(_timestamp += 100_ms);

	// THEN: an backward transition action request is published
	EXPECT_TRUE(_action_request_sub.update());
	EXPECT_EQ(_action_request_sub.get().action, ACTION_VTOL_TRANSITION_TO_MULTICOPTER);
}

TEST_F(SwitchTest, TransitionSwitchStaysRcLoss)
{
	// GIVEN: valid stick input from the RC
	_manual_control_input_pub.publish({.timestamp_sample = _timestamp, .valid = true, .data_source = manual_control_setpoint_s::SOURCE_RC});
	// and transition switch in off position
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .transition_switch = manual_control_switches_s::SWITCH_POS_OFF});
	_manual_control.processInput(_timestamp += 100_ms);

	// THEN: the stick input is published for use
	EXPECT_TRUE(_manual_control_setpoint_sub.update());
	EXPECT_TRUE(_manual_control_setpoint_sub.get().valid);
	// but there is no action requested
	EXPECT_FALSE(_action_request_sub.update());

	// WHEN: RC signal times out
	_manual_control.processInput(_timestamp += 1_s);

	// THEN: the stick input is invalidated
	EXPECT_TRUE(_manual_control_setpoint_sub.update());
	EXPECT_FALSE(_manual_control_setpoint_sub.get().valid);
	// and there is no action requested
	EXPECT_FALSE(_action_request_sub.update());

	// WHEN: RC signal comes back
	_manual_control_input_pub.publish({.timestamp_sample = _timestamp, .valid = true, .data_source = manual_control_setpoint_s::SOURCE_RC});
	_manual_control.processInput(_timestamp += 100_ms);

	// THEN: the stick input is avaialble again
	EXPECT_TRUE(_manual_control_setpoint_sub.update());
	EXPECT_TRUE(_manual_control_setpoint_sub.get().valid);
	// but there is still no action requested
	EXPECT_FALSE(_action_request_sub.update());
}

TEST_F(SwitchTest, TransitionSwitchChangesRcLoss)
{
	// GIVEN: valid stick input from the RC
	_manual_control_input_pub.publish({.timestamp_sample = _timestamp, .valid = true, .data_source = manual_control_setpoint_s::SOURCE_RC});
	// and transition switch in off position
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .transition_switch = manual_control_switches_s::SWITCH_POS_OFF});
	_manual_control.processInput(_timestamp += 100_ms);

	// THEN: the stick input is published for use
	EXPECT_TRUE(_manual_control_setpoint_sub.update());
	EXPECT_TRUE(_manual_control_setpoint_sub.get().valid);
	// but there is no action requested
	EXPECT_FALSE(_action_request_sub.update());

	// WHEN: RC signal times out
	_manual_control.processInput(_timestamp += 1_s);

	// THEN: the stick input is invalidated
	EXPECT_TRUE(_manual_control_setpoint_sub.update());
	EXPECT_FALSE(_manual_control_setpoint_sub.get().valid);
	// and there is no action requested
	EXPECT_FALSE(_action_request_sub.update());

	// WHEN: RC signal comes back with the switch on
	_manual_control_input_pub.publish({.timestamp_sample = _timestamp, .valid = true, .data_source = manual_control_setpoint_s::SOURCE_RC});
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .transition_switch = manual_control_switches_s::SWITCH_POS_ON});
	_manual_control.processInput(_timestamp += 100_ms);

	// THEN: the stick input is avaialble again
	EXPECT_TRUE(_manual_control_setpoint_sub.update());
	EXPECT_TRUE(_manual_control_setpoint_sub.get().valid);
	// but there is still no action requested
	EXPECT_FALSE(_action_request_sub.update());
}

TEST_F(SwitchTest, ModeSwitch)
{
	// GIVEN: valid stick input from RC
	_manual_control_input_pub.publish({.timestamp_sample = _timestamp, .valid = true, .data_source = manual_control_setpoint_s::SOURCE_RC});
	// and mode switch in position 1
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .mode_slot = manual_control_switches_s::MODE_SLOT_1});
	_manual_control.processInput(_timestamp += 10_ms);

	// THEN: the stick input is published for use
	EXPECT_TRUE(_manual_control_setpoint_sub.update());
	EXPECT_TRUE(_manual_control_setpoint_sub.get().valid);
	// and action requested to switch to mode 1
	EXPECT_TRUE(_action_request_sub.update());
	EXPECT_EQ(_action_request_sub.get().action, ACTION_SWITCH_MODE);
	EXPECT_EQ(_action_request_sub.get().mode, TestManualControl::navStateFromParam(NAVIGATION_STATE_ACRO));

	// WHEN: the mode switch is switched to 2
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .mode_slot = manual_control_switches_s::MODE_SLOT_2});
	_manual_control.processInput(_timestamp += 10_ms);
	// THEN: action requested to switch to mode 2
	EXPECT_TRUE(_action_request_sub.update());
	EXPECT_EQ(_action_request_sub.get().action, ACTION_SWITCH_MODE);
	EXPECT_EQ(_action_request_sub.get().mode, TestManualControl::navStateFromParam(NAVIGATION_STATE_MANUAL));

	// WHEN: the mode switch is switched to 3
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .mode_slot = manual_control_switches_s::MODE_SLOT_3});
	_manual_control.processInput(_timestamp += 10_ms);
	// THEN: action requested to switch to mode 3
	EXPECT_TRUE(_action_request_sub.update());
	EXPECT_EQ(_action_request_sub.get().action, ACTION_SWITCH_MODE);
	EXPECT_EQ(_action_request_sub.get().mode, TestManualControl::navStateFromParam(NAVIGATION_STATE_ALTCTL));

	// WHEN: the mode switch is switched to 4
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .mode_slot = manual_control_switches_s::MODE_SLOT_4});
	_manual_control.processInput(_timestamp += 10_ms);
	// THEN: action requested to switch to mode 4
	EXPECT_TRUE(_action_request_sub.update());
	EXPECT_EQ(_action_request_sub.get().action, ACTION_SWITCH_MODE);
	EXPECT_EQ(_action_request_sub.get().mode, TestManualControl::navStateFromParam(NAVIGATION_STATE_POSCTL));

	// WHEN: the mode switch is switched to 5
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .mode_slot = manual_control_switches_s::MODE_SLOT_5});
	_manual_control.processInput(_timestamp += 10_ms);
	// THEN: action requested to switch to mode 5
	EXPECT_TRUE(_action_request_sub.update());
	EXPECT_EQ(_action_request_sub.get().action, ACTION_SWITCH_MODE);
	EXPECT_EQ(_action_request_sub.get().mode, TestManualControl::navStateFromParam(NAVIGATION_STATE_AUTO_LOITER));

	// WHEN: the mode switch is switched to 6
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .mode_slot = manual_control_switches_s::MODE_SLOT_6});
	_manual_control.processInput(_timestamp += 10_ms);
	// THEN: action requested to switch to mode 6
	EXPECT_TRUE(_action_request_sub.update());
	EXPECT_EQ(_action_request_sub.get().action, ACTION_SWITCH_MODE);
	EXPECT_EQ(_action_request_sub.get().mode, TestManualControl::navStateFromParam(NAVIGATION_STATE_AUTO_MISSION));
}

TEST_F(SwitchTest, ModeSwitchInitialization)
{
	// GIVEN: the mode switch is already in position 1 but there's no valid RC stick input yet
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .mode_slot = manual_control_switches_s::MODE_SLOT_1});
	_manual_control.processInput(_timestamp += 10_ms);
	// THEN: there is no action requested
	EXPECT_FALSE(_action_request_sub.update());

	// GIVEN: new valid stick input from RC
	_manual_control_input_pub.publish({.timestamp_sample = _timestamp, .valid = true, .data_source = manual_control_setpoint_s::SOURCE_RC});
	_manual_control.processInput(_timestamp += 10_ms);
	// THEN: there is still no action requested because the switches were not updated yet
	EXPECT_FALSE(_action_request_sub.update());

	// GIVEN: switch update with the same positions
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .mode_slot = manual_control_switches_s::MODE_SLOT_1});
	_manual_control.processInput(_timestamp += 10_ms);
	// THEN: the mode switch is requested
	EXPECT_TRUE(_action_request_sub.update());
	EXPECT_EQ(_action_request_sub.get().action, ACTION_SWITCH_MODE);
	EXPECT_EQ(_action_request_sub.get().mode, TestManualControl::navStateFromParam(NAVIGATION_STATE_ACRO));
}

TEST_F(SwitchTest, ModeSwitchInitializationArmed)
{
	// GIVEN: vehicle is armed
	uORB::Publication<vehicle_status_s> vehicle_status_pub{ORB_ID(vehicle_status)};
	vehicle_status_pub.publish({.arming_state = vehicle_status_s::ARMING_STATE_ARMED});

	// GIVEN: valid stick input from RC
	_manual_control_input_pub.publish({.timestamp_sample = _timestamp, .valid = true, .data_source = manual_control_setpoint_s::SOURCE_RC});
	// and mode switch in position 1
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .mode_slot = manual_control_switches_s::MODE_SLOT_1});
	_manual_control.processInput(_timestamp += 10_ms);

	// THEN: no action requested because the vehicle is flying
	EXPECT_FALSE(_action_request_sub.update());

	// GIVEN: the switch changes position
	_manual_control_switches_pub.publish({.timestamp_sample = _timestamp, .mode_slot = manual_control_switches_s::MODE_SLOT_2});
	_manual_control.processInput(_timestamp += 10_ms);
	// THEN: the mode switch is requested
	EXPECT_TRUE(_action_request_sub.update());
	EXPECT_EQ(_action_request_sub.get().action, ACTION_SWITCH_MODE);
	EXPECT_EQ(_action_request_sub.get().mode, TestManualControl::navStateFromParam(NAVIGATION_STATE_MANUAL));
}
