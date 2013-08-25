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
#include <unit_test/unit_test.h>

class StateMachineHelperTest : public UnitTest
{
public:
	StateMachineHelperTest();
	virtual ~StateMachineHelperTest();

	virtual const char*	run_tests();

private:
	const char*	arming_state_transition_test();
	const char*	arming_state_transition_arm_disarm_test();
	const char*	main_state_transition_test();
	const char*	is_safe_test();
};

StateMachineHelperTest::StateMachineHelperTest() {
}

StateMachineHelperTest::~StateMachineHelperTest() {
}

const char*
StateMachineHelperTest::arming_state_transition_test()
{
	struct vehicle_status_s status;
	struct safety_s safety;
	arming_state_t new_arming_state;
	struct actuator_armed_s armed;

	// Identical states.
	status.arming_state = ARMING_STATE_INIT;
	new_arming_state = ARMING_STATE_INIT;
	mu_assert("no transition: identical states",
		  TRANSITION_NOT_CHANGED == arming_state_transition(&status, &safety, new_arming_state, &armed));

	// INIT to STANDBY.
	armed.armed = false;
	armed.ready_to_arm = false;
	status.arming_state = ARMING_STATE_INIT;
	status.condition_system_sensors_initialized = true;
	new_arming_state = ARMING_STATE_STANDBY;
	mu_assert("transition: init to standby",
		  TRANSITION_CHANGED == arming_state_transition(&status, &safety, new_arming_state, &armed));
	mu_assert("current state: standby", ARMING_STATE_STANDBY == status.arming_state);
	mu_assert("not armed", !armed.armed);
	mu_assert("ready to arm", armed.ready_to_arm);

	// INIT to STANDBY, sensors not initialized.
	armed.armed = false;
	armed.ready_to_arm = false;
	status.arming_state = ARMING_STATE_INIT;
	status.condition_system_sensors_initialized = false;
	new_arming_state = ARMING_STATE_STANDBY;
	mu_assert("no transition: sensors not initialized",
		  TRANSITION_DENIED == arming_state_transition(&status, &safety, new_arming_state, &armed));
	mu_assert("current state: init", ARMING_STATE_INIT == status.arming_state);
	mu_assert("not armed", !armed.armed);
	mu_assert("not ready to arm", !armed.ready_to_arm);

	return 0;
}

const char*
StateMachineHelperTest::arming_state_transition_arm_disarm_test()
{
	struct vehicle_status_s status;
	struct safety_s safety;
	arming_state_t new_arming_state;
	struct actuator_armed_s armed;

	// TODO(sjwilks): ARM then DISARM.
	return 0;
}

const char*
StateMachineHelperTest::main_state_transition_test()
{
	struct vehicle_status_s current_state;
	main_state_t new_main_state;
	
	// Identical states.
	current_state.main_state = MAIN_STATE_MANUAL;
	new_main_state = MAIN_STATE_MANUAL;
	mu_assert("no transition: identical states",
		  TRANSITION_NOT_CHANGED == main_state_transition(&current_state, new_main_state));
	mu_assert("current state: manual", MAIN_STATE_MANUAL == current_state.main_state);	

	// AUTO to MANUAL.
	current_state.main_state = MAIN_STATE_AUTO;
	new_main_state = MAIN_STATE_MANUAL;
	mu_assert("transition changed: auto to manual",
		  TRANSITION_CHANGED == main_state_transition(&current_state, new_main_state));
	mu_assert("new state: manual", MAIN_STATE_MANUAL == current_state.main_state);

	// MANUAL to SEATBELT.
	current_state.main_state = MAIN_STATE_MANUAL;
	current_state.condition_local_altitude_valid = true;
	new_main_state = MAIN_STATE_SEATBELT;
	mu_assert("tranisition: manual to seatbelt", 
		  TRANSITION_CHANGED == main_state_transition(&current_state, new_main_state));
	mu_assert("new state: seatbelt", MAIN_STATE_SEATBELT == current_state.main_state);

	// MANUAL to SEATBELT, invalid local altitude.
	current_state.main_state = MAIN_STATE_MANUAL;
	current_state.condition_local_altitude_valid = false;
	new_main_state = MAIN_STATE_SEATBELT;
	mu_assert("no transition: invalid local altitude",
		  TRANSITION_DENIED == main_state_transition(&current_state, new_main_state));
	mu_assert("current state: manual", MAIN_STATE_MANUAL == current_state.main_state);

	// MANUAL to EASY.
	current_state.main_state = MAIN_STATE_MANUAL;
	current_state.condition_local_position_valid = true;
	new_main_state = MAIN_STATE_EASY;
	mu_assert("transition: manual to easy",
		  TRANSITION_CHANGED == main_state_transition(&current_state, new_main_state));
	mu_assert("current state: easy", MAIN_STATE_EASY == current_state.main_state);

	// MANUAL to EASY, invalid local position.
	current_state.main_state = MAIN_STATE_MANUAL;
	current_state.condition_local_position_valid = false;
	new_main_state = MAIN_STATE_EASY;
	mu_assert("no transition: invalid position",
		  TRANSITION_DENIED == main_state_transition(&current_state, new_main_state));
	mu_assert("current state: manual", MAIN_STATE_MANUAL == current_state.main_state);

	// MANUAL to AUTO.
	current_state.main_state = MAIN_STATE_MANUAL;
	current_state.condition_global_position_valid = true;
	new_main_state = MAIN_STATE_AUTO;
	mu_assert("transition: manual to auto",
		  TRANSITION_CHANGED == main_state_transition(&current_state, new_main_state));
	mu_assert("current state: auto", MAIN_STATE_AUTO == current_state.main_state);

	// MANUAL to AUTO, invalid global position.
	current_state.main_state = MAIN_STATE_MANUAL;
	current_state.condition_global_position_valid = false;
	new_main_state = MAIN_STATE_AUTO;
	mu_assert("no transition: invalid global position",
		  TRANSITION_DENIED == main_state_transition(&current_state, new_main_state));
	mu_assert("current state: manual", MAIN_STATE_MANUAL == current_state.main_state);

	return 0;
}

const char*
StateMachineHelperTest::is_safe_test()
{
	struct vehicle_status_s current_state;
	struct safety_s safety;
	struct actuator_armed_s armed;

	armed.armed = false;
	armed.lockdown = false;
	safety.safety_switch_available = true;
	safety.safety_off = false;
	mu_assert("is safe: not armed", is_safe(&current_state, &safety, &armed));

	armed.armed = false;
	armed.lockdown = true;
	safety.safety_switch_available = true;
	safety.safety_off = true;
	mu_assert("is safe: software lockdown", is_safe(&current_state, &safety, &armed));

	armed.armed = true;
	armed.lockdown = false;
	safety.safety_switch_available = true;
	safety.safety_off = true;
	mu_assert("not safe: safety off", !is_safe(&current_state, &safety, &armed));

	armed.armed = true;
	armed.lockdown = false;
	safety.safety_switch_available = true;
	safety.safety_off = false;
	mu_assert("is safe: safety off", is_safe(&current_state, &safety, &armed));

	armed.armed = true;
	armed.lockdown = false;
	safety.safety_switch_available = false;
	safety.safety_off = false;
	mu_assert("not safe: no safety switch", !is_safe(&current_state, &safety, &armed));

	return 0;
}

const char*
StateMachineHelperTest::run_tests()
{
	mu_run_test(arming_state_transition_test);
	mu_run_test(arming_state_transition_arm_disarm_test);
	mu_run_test(main_state_transition_test);
	mu_run_test(is_safe_test);

	return 0;
}

void
state_machine_helper_test()
{
	StateMachineHelperTest* test = new StateMachineHelperTest();
	test->UnitTest::print_results(test->run_tests());
}
