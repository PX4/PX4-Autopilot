/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Simon Wilks <sjwilks@gmail.com>
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
 * @file state_machine_helper_test.cpp
 * System state machine unit test.
 *
 */

#include "state_machine_helper_test.h"

#include "../state_machine_helper.h"
#include <unit_test.h>
#include "../Arming/ArmStateMachine/ArmStateMachine.hpp"
#include "../Arming/PreFlightCheck/PreFlightCheck.hpp"

class StateMachineHelperTest : public UnitTest
{
public:
	StateMachineHelperTest() = default;
	~StateMachineHelperTest() override = default;

	bool run_tests() override;

private:
	bool mainStateTransitionTest();
};

bool StateMachineHelperTest::mainStateTransitionTest()
{
	// This structure represent a single test case for testing Main State transitions.
	typedef struct {
		const char *assertMsg;				// Text to show when test case fails
		uint8_t	 condition_bits;			// Bits for various condition_* values
		uint8_t	from_state;				// State prior to transition request
		main_state_t to_state;				// State to transition to
		transition_result_t	expected_transition_result;	// Expected result from main_state_transition call
	} MainTransitionTest_t;

	// Bits for condition_bits
#define MTT_ALL_NOT_VALID		0
#define MTT_ROTARY_WING			1 << 0
#define MTT_LOC_ALT_VALID		1 << 1
#define MTT_LOC_POS_VALID		1 << 2
#define MTT_HOME_POS_VALID		1 << 3
#define MTT_GLOBAL_POS_VALID		1 << 4

	static const MainTransitionTest_t rgMainTransitionTests[] = {

		// TRANSITION_NOT_CHANGED tests

		{
			"no transition: identical states",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_MANUAL, TRANSITION_NOT_CHANGED
		},

		// TRANSITION_CHANGED tests

		{
			"transition: MANUAL to ACRO - rotary",
			MTT_ROTARY_WING,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_ACRO, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to ACRO - not rotary",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_ACRO, TRANSITION_CHANGED
		},

		{
			"transition: ACRO to MANUAL",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_ACRO, commander_state_s::MAIN_STATE_MANUAL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to AUTO_MISSION - global position valid, home position valid",
			MTT_GLOBAL_POS_VALID | MTT_HOME_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_MISSION, TRANSITION_CHANGED
		},

		{
			"transition: AUTO_MISSION to MANUAL - global position valid, home position valid",
			MTT_GLOBAL_POS_VALID | MTT_HOME_POS_VALID,
			commander_state_s::MAIN_STATE_AUTO_MISSION, commander_state_s::MAIN_STATE_MANUAL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to AUTO_LOITER - global position valid",
			MTT_GLOBAL_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_LOITER, TRANSITION_CHANGED
		},

		{
			"transition: AUTO_LOITER to MANUAL - global position valid",
			MTT_GLOBAL_POS_VALID,
			commander_state_s::MAIN_STATE_AUTO_LOITER, commander_state_s::MAIN_STATE_MANUAL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to AUTO_RTL - global position valid, home position valid",
			MTT_GLOBAL_POS_VALID | MTT_HOME_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_RTL, TRANSITION_CHANGED
		},

		{
			"transition: AUTO_RTL to MANUAL - global position valid, home position valid",
			MTT_GLOBAL_POS_VALID | MTT_HOME_POS_VALID,
			commander_state_s::MAIN_STATE_AUTO_RTL, commander_state_s::MAIN_STATE_MANUAL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to ALTCTL - not rotary",
			MTT_LOC_ALT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_ALTCTL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to ALTCTL - rotary, global position not valid, local altitude valid",
			MTT_ROTARY_WING | MTT_LOC_ALT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_ALTCTL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to ALTCTL - rotary, global position valid, local altitude not valid",
			MTT_ROTARY_WING | MTT_GLOBAL_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_ALTCTL, TRANSITION_CHANGED
		},

		{
			"transition: ALTCTL to MANUAL - local altitude valid",
			MTT_LOC_ALT_VALID,
			commander_state_s::MAIN_STATE_ALTCTL, commander_state_s::MAIN_STATE_MANUAL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to POSCTL - local position not valid, global position valid",
			MTT_GLOBAL_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_POSCTL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to POSCTL - local position valid, global position not valid",
			MTT_LOC_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_POSCTL, TRANSITION_CHANGED
		},

		{
			"transition: POSCTL to MANUAL - local position valid, global position valid",
			MTT_LOC_POS_VALID,
			commander_state_s::MAIN_STATE_POSCTL, commander_state_s::MAIN_STATE_MANUAL, TRANSITION_CHANGED
		},

		// TRANSITION_DENIED tests

		{
			"no transition: MANUAL to AUTO_MISSION - global position not valid",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_MISSION, TRANSITION_DENIED
		},

		{
			"no transition: MANUAL to AUTO_LOITER - global position not valid",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_LOITER, TRANSITION_DENIED
		},

		{
			"no transition: MANUAL to AUTO_RTL - global position not valid, home position not valid",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_RTL, TRANSITION_DENIED
		},

		{
			"no transition: MANUAL to AUTO_RTL - global position not valid, home position valid",
			MTT_HOME_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_RTL, TRANSITION_DENIED
		},

		{
			"no transition: MANUAL to AUTO_RTL - global position valid, home position not valid",
			MTT_GLOBAL_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_RTL, TRANSITION_DENIED
		},

		{
			"transition: MANUAL to ALTCTL - not rotary",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_ALTCTL, TRANSITION_DENIED
		},

		{
			"no transition: MANUAL to ALTCTL - rotary, global position not valid, local altitude not valid",
			MTT_ROTARY_WING,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_ALTCTL, TRANSITION_DENIED
		},

		{
			"no transition: MANUAL to POSCTL - local position not valid, global position not valid",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_POSCTL, TRANSITION_DENIED
		},
	};

	size_t cMainTransitionTests = sizeof(rgMainTransitionTests) / sizeof(rgMainTransitionTests[0]);

	for (size_t i = 0; i < cMainTransitionTests; i++) {
		const MainTransitionTest_t *test = &rgMainTransitionTests[i];

		// Setup initial machine state
		struct vehicle_status_s current_vehicle_status = {};
		struct commander_state_s current_commander_state = {};
		struct vehicle_status_flags_s current_status_flags = {};

		current_commander_state.main_state = test->from_state;
		current_vehicle_status.vehicle_type = (test->condition_bits & MTT_ROTARY_WING) ?
						      vehicle_status_s::VEHICLE_TYPE_ROTARY_WING : vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
		current_status_flags.local_altitude_valid = test->condition_bits & MTT_LOC_ALT_VALID;
		current_status_flags.local_position_valid = test->condition_bits & MTT_LOC_POS_VALID;
		current_status_flags.home_position_valid = test->condition_bits & MTT_HOME_POS_VALID;
		current_status_flags.global_position_valid = test->condition_bits & MTT_GLOBAL_POS_VALID;
		current_status_flags.auto_mission_available = true;

		// Attempt transition
		transition_result_t result = main_state_transition(current_vehicle_status, test->to_state, current_status_flags,
					     current_commander_state);

		// Validate result of transition
		ut_compare(test->assertMsg, test->expected_transition_result, result);

		if (test->expected_transition_result == result) {
			if (test->expected_transition_result == TRANSITION_CHANGED) {
				ut_compare(test->assertMsg, test->to_state, current_commander_state.main_state);

			} else {
				ut_compare(test->assertMsg, test->from_state, current_commander_state.main_state);
			}
		}
	}

	return true;
}

bool StateMachineHelperTest::run_tests()
{
	ut_run_test(mainStateTransitionTest);

	return (_tests_failed == 0);
}

ut_declare_test(stateMachineHelperTest, StateMachineHelperTest)
