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
 * @file DaaActionPolicyTest.cpp
 * @brief Unit tests for the DAA action policy.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include <gtest/gtest.h>

#include <lib/adsb/DaaActionPolicy.h>
#include <uORB/topics/detect_and_avoid.h>
#include <uORB/topics/vehicle_status.h>

namespace
{

// Same action parameter value for every conflict level, so the decide() tests
// behave identically in F3442 and Crosstrack builds.
daa_action_params_s params_for_all_levels(const int32_t action_param)
{
	daa_action_params_s params{};
#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
	params.lvl_low_act = action_param;
	params.lvl_med_act = action_param;
	params.lvl_high_act = action_param;
	params.lvl_crit_act = action_param;
#else
	params.traff_avoid = action_param;
#endif // CONFIG_NAVIGATOR_ADSB_F3442
	return params;
}

// Navigation state groups by the weakest DAA action allowed to escalate above them.
const uint8_t kStatesEnableHold[] {
	vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION,
	vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF,
	vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET,
	vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF,
};

const uint8_t kStatesEnableRtl[] {
	vehicle_status_s::NAVIGATION_STATE_ORBIT,
	vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER,
};

const uint8_t kStatesEnableLand[] {
	vehicle_status_s::NAVIGATION_STATE_AUTO_RTL,
};

const uint8_t kStatesEnableTermination[] {
	vehicle_status_s::NAVIGATION_STATE_AUTO_LAND,
	vehicle_status_s::NAVIGATION_STATE_DESCEND,
	vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND,
	vehicle_status_s::NAVIGATION_STATE_MANUAL,
	vehicle_status_s::NAVIGATION_STATE_ALTCTL,
	vehicle_status_s::NAVIGATION_STATE_POSCTL,
	vehicle_status_s::NAVIGATION_STATE_ACRO,
	vehicle_status_s::NAVIGATION_STATE_STAB,
};

const uint8_t kStatesNeverEnable[] {
	vehicle_status_s::NAVIGATION_STATE_TERMINATION,
	vehicle_status_s::NAVIGATION_STATE_OFFBOARD,
};

struct escalation_case_s {
	DaaAction action{DaaAction::kDisabled};
	bool hold_allowed{false};
	bool rtl_allowed{false};
	bool land_allowed{false};
	bool terminate_allowed{false};
};

void check_escalation_matrix(const escalation_case_s &test_case)
{
	for (const uint8_t nav_state : kStatesEnableHold) {
		EXPECT_EQ(DaaActionPolicy::action_escalates_above_nav_state(test_case.action, nav_state),
			  test_case.hold_allowed) << "action " << (int)test_case.action << " nav_state " << (int)nav_state;
	}

	for (const uint8_t nav_state : kStatesEnableRtl) {
		EXPECT_EQ(DaaActionPolicy::action_escalates_above_nav_state(test_case.action, nav_state),
			  test_case.rtl_allowed) << "action " << (int)test_case.action << " nav_state " << (int)nav_state;
	}

	for (const uint8_t nav_state : kStatesEnableLand) {
		EXPECT_EQ(DaaActionPolicy::action_escalates_above_nav_state(test_case.action, nav_state),
			  test_case.land_allowed) << "action " << (int)test_case.action << " nav_state " << (int)nav_state;
	}

	for (const uint8_t nav_state : kStatesEnableTermination) {
		EXPECT_EQ(DaaActionPolicy::action_escalates_above_nav_state(test_case.action, nav_state),
			  test_case.terminate_allowed) << "action " << (int)test_case.action << " nav_state " << (int)nav_state;
	}

	for (const uint8_t nav_state : kStatesNeverEnable) {
		EXPECT_FALSE(DaaActionPolicy::action_escalates_above_nav_state(test_case.action, nav_state))
				<< "action " << (int)test_case.action << " nav_state " << (int)nav_state;
	}
}

} // namespace

// WHY: Operator messages and parameters share the NAV_TRAFF_AVOID numbering while escalation uses
// the internal severity ladder; the two mappings must stay exact inverses.
// WHAT: Round-trip every parameter value and reject invalid input.
TEST(DaaActionPolicyTest, ActionParamMappingRoundTrip)
{
	for (int32_t param = 0; param <= 5; ++param) {
		const DaaAction action = DaaActionPolicy::action_param_to_daa_action(param);
		EXPECT_EQ(DaaActionPolicy::daa_action_to_action_param(action), param);
	}

	EXPECT_EQ(DaaActionPolicy::action_param_to_daa_action(-1), DaaAction::kDisabled);
	EXPECT_EQ(DaaActionPolicy::action_param_to_daa_action(6), DaaAction::kDisabled);
	EXPECT_EQ(DaaActionPolicy::daa_action_to_action_param(DaaAction::kMaxActionValue), 0);
}

// WHY: Automatic DAA actions must only escalate when they are stronger than the current navigator
// state; this gate protects manual and offboard flight from unintended mode changes.
// WHAT: Evaluate each requested action across all representative navigation states.
TEST(DaaActionPolicyTest, ActionEscalatesOnlyAboveNavState)
{
	check_escalation_matrix({DaaAction::kDisabled, false, false, false, false});
	check_escalation_matrix({DaaAction::kWarnOnly, false, false, false, false});
	check_escalation_matrix({DaaAction::kPositionHoldMode, true, false, false, false});
	check_escalation_matrix({DaaAction::kReturnMode, true, true, false, false});
	check_escalation_matrix({DaaAction::kLandMode, true, true, true, false});
	check_escalation_matrix({DaaAction::kTerminate, true, true, true, true});
}

// WHY: Invalid conflict levels must never request an action.
// WHAT: Map NONE and an invalid level with actions configured everywhere.
TEST(DaaActionPolicyTest, NoActionForInvalidLevels)
{
	const daa_action_params_s params = params_for_all_levels(5); // Terminate everywhere

	EXPECT_EQ(DaaActionPolicy::action_from_conflict_level(detect_and_avoid_s::DAA_CONFLICT_LVL_NONE, params),
		  DaaAction::kDisabled);
	EXPECT_EQ(DaaActionPolicy::action_from_conflict_level(detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL + 1, params),
		  DaaAction::kDisabled);
}

#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
// WHY: F3442 zones are nested, so a breached inner zone with a disabled action must fall back to
// the next larger breached zone.
// WHAT: Configure per-level actions with gaps and verify the fallback chain.
TEST(DaaActionPolicyTest, F3442NestedZoneFallback)
{
	daa_action_params_s params{};
	params.lvl_low_act = 1;		// Warn only
	params.lvl_med_act = 4;		// Hold
	params.lvl_high_act = 0;	// Disabled -> falls back to MEDIUM's action
	params.lvl_crit_act = 5;	// Terminate

	EXPECT_EQ(DaaActionPolicy::action_from_conflict_level(detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, params),
		  DaaAction::kWarnOnly);
	EXPECT_EQ(DaaActionPolicy::action_from_conflict_level(detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM, params),
		  DaaAction::kPositionHoldMode);
	EXPECT_EQ(DaaActionPolicy::action_from_conflict_level(detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH, params),
		  DaaAction::kPositionHoldMode);
	EXPECT_EQ(DaaActionPolicy::action_from_conflict_level(detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, params),
		  DaaAction::kTerminate);

	// All levels disabled: the chain ends at LOW and stays disabled.
	EXPECT_EQ(DaaActionPolicy::action_from_conflict_level(detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL,
			params_for_all_levels(0)), DaaAction::kDisabled);
}
#else
// WHY: Crosstrack has a single threshold, so every breached level maps through NAV_TRAFF_AVOID.
// WHAT: Map all in-range levels with one configured action.
TEST(DaaActionPolicyTest, CrosstrackSingleActionMapping)
{
	const daa_action_params_s params = params_for_all_levels(3); // Land

	for (uint8_t level = detect_and_avoid_s::DAA_CONFLICT_LVL_LOW;
	     level <= detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL; ++level) {
		EXPECT_EQ(DaaActionPolicy::action_from_conflict_level(level, params), DaaAction::kLandMode);
	}
}
#endif // CONFIG_NAVIGATOR_ADSB_F3442

// WHY: An unchanged most urgent level must never produce a command or warning.
// WHAT: Call decide() with equal levels.
TEST(DaaActionPolicyTest, DecideNoResponseWithoutLevelChange)
{
	const daa_action_decision_s decision = DaaActionPolicy::decide(
			detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL,
			vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, false, true,
			DaaAction::kDisabled, params_for_all_levels(5));

	EXPECT_EQ(decision.action_command, DaaAction::kDisabled);
	EXPECT_FALSE(decision.announce_action);
	EXPECT_FALSE(decision.warn_on_ground);
}

// WHY: On the ground, an action-requiring conflict must warn the operator (blocked arm or takeoff)
// WHAT: Run a landed escalation armed, disarmed and with a warn only configuration.
TEST(DaaActionPolicyTest, DecideWarnsOnGroundInsteadOfActing)
{
	const uint8_t critical = detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL;
	const uint8_t none = detect_and_avoid_s::DAA_CONFLICT_LVL_NONE;
	const uint8_t nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;

	daa_action_decision_s decision = DaaActionPolicy::decide(critical, none, nav_state, true, true,
					 DaaAction::kDisabled, params_for_all_levels(5));
	EXPECT_TRUE(decision.warn_on_ground);
	EXPECT_EQ(decision.ground_warning_cause, NotifyLandedActCause::kConflictAndArmed);
	EXPECT_EQ(decision.action_command, DaaAction::kDisabled);

	decision = DaaActionPolicy::decide(critical, none, nav_state, true, false,
					   DaaAction::kDisabled, params_for_all_levels(5));
	EXPECT_TRUE(decision.warn_on_ground);
	EXPECT_EQ(decision.ground_warning_cause, NotifyLandedActCause::kConflictAndDisarmed);

	// Warn-only configuration: no ground warning through the action path.
	decision = DaaActionPolicy::decide(critical, none, nav_state, true, true,
					   DaaAction::kDisabled, params_for_all_levels(1));
	EXPECT_FALSE(decision.warn_on_ground);
	EXPECT_EQ(decision.action_command, DaaAction::kDisabled);
}

// WHY: In the air, commands are sent once per escalation, requested again without announcing again when
// the same action is needed again, and never issued on de-escalation or against a stronger mode.
// WHAT: Test escalation, repeated escalation, blocked escalation and de-escalation.
TEST(DaaActionPolicyTest, DecideRequestsCommandOnAirborneEscalationOnly)
{
	const uint8_t critical = detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL;
	const uint8_t low = detect_and_avoid_s::DAA_CONFLICT_LVL_LOW;
	const daa_action_params_s land_params = params_for_all_levels(3); // Land

	// Escalation in a mission: command requested and announced.
	daa_action_decision_s decision = DaaActionPolicy::decide(critical, low,
					 vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, false, true,
					 DaaAction::kDisabled, land_params);
	EXPECT_EQ(decision.action_command, DaaAction::kLandMode);
	EXPECT_TRUE(decision.announce_action);
	EXPECT_FALSE(decision.warn_on_ground);

	// Same action requested again, do not announce
	decision = DaaActionPolicy::decide(critical, low, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION,
					   false, true, DaaAction::kLandMode, land_params);
	EXPECT_EQ(decision.action_command, DaaAction::kLandMode);
	EXPECT_FALSE(decision.announce_action);

	// Current mode already stronger than the requested action: no command.
	decision = DaaActionPolicy::decide(critical, low, vehicle_status_s::NAVIGATION_STATE_TERMINATION,
					   false, true, DaaAction::kDisabled, land_params);
	EXPECT_EQ(decision.action_command, DaaAction::kDisabled);

	// De-escalation: never a command.
	decision = DaaActionPolicy::decide(low, critical, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION,
					   false, true, DaaAction::kLandMode, land_params);
	EXPECT_EQ(decision.action_command, DaaAction::kDisabled);
	EXPECT_FALSE(decision.announce_action);
}
