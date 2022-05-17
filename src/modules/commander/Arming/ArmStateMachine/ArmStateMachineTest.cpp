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

#include <gtest/gtest.h>
#include <ArmStateMachine.hpp>

TEST(ArmStateMachineTest, ArmingStateTransitionTest)
{
	ArmStateMachine arm_state_machine;

	// These are the critical values from vehicle_status_s and actuator_armed_s which must be primed
	// to simulate machine state prior to testing an arming state transition. This structure is also
	// use to represent the expected machine state after the transition has been requested.
	typedef struct {
		arming_state_t  arming_state;   // vehicle_status_s.arming_state
		bool            armed;          // actuator_armed_s.armed
		bool            ready_to_arm;   // actuator_armed_s.ready_to_arm
	} ArmingTransitionVolatileState_t;

	// This structure represents a test case for arming_state_transition. It contains the machine
	// state prior to transition, the requested state to transition to and finally the expected
	// machine state after transition.
	typedef struct {
		const char                     *assertMsg;                              // Text to show when test case fails
		ArmingTransitionVolatileState_t current_state;                          // Machine state prior to transition
		hil_state_t                     hil_state;                              // Current vehicle_status_s.hil_state
		bool
		system_sensors_initialized;   // Current vehicle_status_s.system_sensors_initialized
		bool                            safety_switch_available;                // Current safety_s.safety_switch_available
		bool                            safety_off;                             // Current safety_s.safety_off
		arming_state_t                  requested_state;                        // Requested arming state to transition to
		ArmingTransitionVolatileState_t expected_state;                         // Expected machine state after transition
		transition_result_t             expected_transition_result;             // Expected result from arming_state_transition
	} ArmingTransitionTest_t;

	// We use these defines so that our test cases are more readable
#define ATT_ARMED true
#define ATT_DISARMED false
#define ATT_READY_TO_ARM true
#define ATT_NOT_READY_TO_ARM false
#define ATT_SENSORS_INITIALIZED true
#define ATT_SENSORS_NOT_INITIALIZED false
#define ATT_SAFETY_AVAILABLE true
#define ATT_SAFETY_NOT_AVAILABLE true
#define ATT_SAFETY_OFF true
#define ATT_SAFETY_ON false

	// These are test cases for arming_state_transition
	static const ArmingTransitionTest_t rgArmingTransitionTests[] = {
		// TRANSITION_NOT_CHANGED tests

		{
			"no transition: identical states",
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_INIT,
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_NOT_CHANGED
		},

		// TRANSITION_CHANGED tests

		// Check all basic valid transitions, these don't require special state in vehicle_status_t or safety_s

		{
			"transition: init to standby",
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY,
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: init to standby error",
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY_ERROR,
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: init to reboot",
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: standby to init",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_INIT,
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: standby to standby error",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY_ERROR,
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: standby to reboot",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: armed to standby",
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY,
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: standby error to reboot",
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: in air restore to armed",
			{ vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: in air restore to reboot",
			{ vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		// hil on tests, standby error to standby not normally allowed

		{
			"transition: standby error to standby, hil on",
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_ON, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY,
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		// Safety switch arming tests

		{
			"transition: standby to armed, no safety switch",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_ON, ATT_SENSORS_INITIALIZED, ATT_SAFETY_NOT_AVAILABLE, ATT_SAFETY_OFF,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: standby to armed, safety switch off",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_ON, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_OFF,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		// TRANSITION_DENIED tests

		// Check some important basic invalid transitions, these don't require special state in vehicle_status_t or safety_s

		{
			"no transition: init to armed",
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_DENIED
		},

		{
			"no transition: armed to init",
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_INIT,
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, TRANSITION_DENIED
		},

		{
			"no transition: armed to reboot",
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, TRANSITION_DENIED
		},

		{
			"no transition: standby error to armed",
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_DENIED
		},

		{
			"no transition: standby error to standby",
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY,
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_DENIED
		},

		{
			"no transition: reboot to armed",
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_DENIED
		},

		{
			"no transition: in air restore to standby",
			{ vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY,
			{ vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_DENIED
		},

		// Sensor tests

		//{ "transition to standby error: init to standby - sensors not initialized",
		//    { vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_NOT_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
		//    vehicle_status_s::ARMING_STATE_STANDBY,
		//    { vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_DENIED },

		// Safety switch arming tests

		{
			"no transition: init to armed, safety switch on",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, TRANSITION_DENIED
		},
	};

	struct vehicle_status_s status {};
	struct vehicle_status_flags_s status_flags {};
	struct safety_s safety {};
	struct actuator_armed_s armed {};

	size_t cArmingTransitionTests = sizeof(rgArmingTransitionTests) / sizeof(rgArmingTransitionTests[0]);

	for (size_t i = 0; i < cArmingTransitionTests; i++) {
		const ArmingTransitionTest_t *test = &rgArmingTransitionTests[i];

		PreFlightCheck::arm_requirements_t arm_req{};

		// Setup initial machine state
		arm_state_machine.forceArmState(test->current_state.arming_state);
		status_flags.system_sensors_initialized = test->system_sensors_initialized;
		status.hil_state = test->hil_state;
		// The power status of the test unit is not relevant for the unit test
		status_flags.circuit_breaker_engaged_power_check = true;
		safety.safety_switch_available = test->safety_switch_available;
		safety.safety_off = test->safety_off;

		armed.armed = test->current_state.armed;
		armed.ready_to_arm = test->current_state.ready_to_arm;

		vehicle_control_mode_s control_mode{};

		// Attempt transition
		transition_result_t result = arm_state_machine.arming_state_transition(
						     status,
						     control_mode,
						     safety,
						     test->requested_state,
						     armed,
						     true /* enable pre-arm checks */,
						     nullptr /* no mavlink_log_pub */,
						     status_flags,
						     arm_req,
						     2e6, /* 2 seconds after boot, everything should be checked */
						     arm_disarm_reason_t::unit_test);

		// Validate result of transition
		EXPECT_EQ(result, test->expected_transition_result) << test->assertMsg;
		EXPECT_EQ(arm_state_machine.getArmState(), test->expected_state.arming_state) << test->assertMsg;
		EXPECT_EQ(armed.armed, test->expected_state.armed) << test->assertMsg;
		EXPECT_EQ(armed.ready_to_arm, test->expected_state.ready_to_arm) << test->assertMsg;
	}
}
