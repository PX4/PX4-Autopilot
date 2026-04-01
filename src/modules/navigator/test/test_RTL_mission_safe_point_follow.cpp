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
 * @file test_RTL_mission_safe_point_follow.cpp
 *
 * Unit tests for the lightweight RtlMissionSafePointFollow stage machine.
 * Focuses on setNextMissionItem() stage transitions without constructing a
 * full Navigator stack.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include <gtest/gtest.h>

#include "navigator.h"
#include "rtl_mission_safe_point_follow.h"
#include "test_RTL_helpers.h"

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

#include <vector>

using rtl_test_reference::kAlt;
using rtl_test_reference::kBaseLat;
using rtl_test_reference::kBaseLon;

/**
 * Lightweight peer for the RTL stage machine.
 *
 * Overrides mission-item loading with vector-backed storage and exposes only
 * the minimal internal state needed to exercise setNextMissionItem().
 */
class RtlMissionSafePointFollowTestPeer : public RtlMissionSafePointFollow
{
public:
	using Stage = RtlMissionSafePointFollow::Stage;
	using VtolTransitionAction = MissionBase::VtolTransitionAction;

	explicit RtlMissionSafePointFollowTestPeer(Navigator *navigator = nullptr)
		: RtlMissionSafePointFollow(navigator, mission_s{})
	{
	}

	~RtlMissionSafePointFollowTestPeer() override = default;

	bool loadMissionItemFromCache(int32_t index, mission_item_s &mission_item) override
	{
		if (index < 0 || index >= static_cast<int32_t>(_items.size())) {
			return false;
		}

		mission_item = _items[static_cast<size_t>(index)];
		return true;
	}

	void loadTestMission(const std::vector<mission_item_s> &items)
	{
		_items = items;
		_mission = {};
		_mission.count = static_cast<int32_t>(items.size());
		_mission.current_seq = 0;
		_state = {};
		_plan = {};
	}

	void setStageForTest(Stage stage)
	{
		_state.stage = stage;
	}

	Stage stageForTest() const
	{
		return _state.stage;
	}

	void setCurrentSequenceForTest(int32_t index)
	{
		_mission.current_seq = index;
	}

	int32_t currentSequenceForTest() const
	{
		return _mission.current_seq;
	}

	void setSafePointSelectionForTest(bool direction_reversed, int32_t branch_off_index)
	{
		_plan = {};
		_plan.selection.found = true;
		_plan.selection.safe_point_found = true;
		_plan.selection.goal_type = MissionRoutePlanner::GoalType::SafePoint;
		_plan.selection.path.direction_reversed = direction_reversed;
		_plan.selection.path.first_item_index = branch_off_index;
		_plan.selection.branch_off_segment.start.idx = direction_reversed ? branch_off_index : branch_off_index - 1;
		_plan.selection.branch_off_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
		_plan.selection.branch_off_segment.end.idx = direction_reversed ? branch_off_index + 1 : branch_off_index;
		_plan.selection.branch_off_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	}

	void setTransitionTargetIndexForTest(int32_t index)
	{
		_state.transition_target_index = index;
	}

	void setTransitionStateForTest(VtolTransitionAction action, bool command_sent, bool advance_route_after_transition)
	{
		_state.transition_action = action;
		_state.transition_command_sent = command_sent;
		_state.advance_route_after_transition = advance_route_after_transition;
	}

	void setVehicleStatusForTest(bool is_vtol, bool is_fixed_wing, bool in_transition_to_fw)
	{
		vehicle_status_s status{};
		status.is_vtol = is_vtol;
		status.vehicle_type = is_fixed_wing
				      ? vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				      : vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status.in_transition_to_fw = in_transition_to_fw;
		status.in_transition_mode = in_transition_to_fw;
		status.timestamp = hrt_absolute_time();
		_vehicle_status_pub.publish(status);
		_vehicle_status_sub.update();
	}

	void setLandedForTest(bool landed)
	{
		vehicle_land_detected_s land_detected{};
		land_detected.landed = landed;
		land_detected.timestamp = hrt_absolute_time();
		_land_detected_pub.publish(land_detected);
		_land_detected_sub.update();
	}

	void setGoalLandApproachForTest(const loiter_point_s &land_approach)
	{
		_goal_land_approach = land_approach;
	}

	int32_t transitionTargetIndexForTest() const
	{
		return _state.transition_target_index;
	}

	VtolTransitionAction transitionActionForTest() const
	{
		return _state.transition_action;
	}

	bool transitionCommandSentForTest() const
	{
		return _state.transition_command_sent;
	}

	bool advanceRouteAfterTransitionForTest() const
	{
		return _state.advance_route_after_transition;
	}

	bool advanceStageForTest()
	{
		return setNextMissionItem();
	}

	void normalizeRouteMissionItemForTest(mission_item_s &mission_item) const
	{
		normalizeRouteMissionItem(mission_item);
	}

	void publishActiveMissionItemsForTest()
	{
		setActiveMissionItems();
	}

	void resetExecutorProgressForTest()
	{
		resetExecutorProgress();
	}

	void setCurrentMissionItemForTest(const mission_item_s &mission_item)
	{
		_mission_item = mission_item;
	}

private:
	std::vector<mission_item_s> _items;
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};
};

/**
 * @brief Fixture for lightweight RTL mission-safe-point-follow stage transitions.
 */
class RtlMissionSafePointFollowStageTest : public NavigatorDatamanTestBase
{
protected:
	RtlMissionSafePointFollowTestPeer executor{};

	void SetUp() override
	{
		executor.loadTestMission({});
		executor.setVehicleStatusForTest(false, false, false);
		executor.setLandedForTest(false);
	}
};

// WHY: TransitionDuringRoute is a one-shot state used only to issue a VTOL transition.
// WHAT: setNextMissionItem returns to FollowRoute and clears the remembered transition target.
TEST_F(RtlMissionSafePointFollowStageTest, TransitionDuringRouteResumesFollowRoute)
{
	// GIVEN: An executor paused in the transition stage with a remembered target index.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::TransitionDuringRoute);
	executor.setTransitionTargetIndexForTest(1);
	executor.setTransitionStateForTest(RtlMissionSafePointFollowTestPeer::VtolTransitionAction::FrontTransition, true,
					   false);

	// WHEN: setNextMissionItem advances the stage machine.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The transition stage completes and route following resumes.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	EXPECT_EQ(executor.transitionTargetIndexForTest(), -1);
}

// WHY: A transition may now protect the final route segment before branch-off.
// WHAT: Completing that transition hands control to BranchOff instead of back to FollowRoute.
TEST_F(RtlMissionSafePointFollowStageTest, TransitionDuringRouteResumesBranchOffWhenTargetIsBranchOff)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::TransitionDuringRoute);
	executor.setCurrentSequenceForTest(2);
	executor.setSafePointSelectionForTest(false, 2);
	executor.setTransitionTargetIndexForTest(2);
	executor.setTransitionStateForTest(RtlMissionSafePointFollowTestPeer::VtolTransitionAction::FrontTransition, true,
					   false);

	const bool advanced = executor.advanceStageForTest();

	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::BranchOff);
	EXPECT_EQ(executor.transitionTargetIndexForTest(), -1);
}

// WHY: In reverse route following, reaching a waypoint with an attached transition must hold
//      the current route target and enter TransitionDuringRoute instead of advancing immediately.
// WHAT: setNextMissionItem arms the transition on the reached reverse target and leaves current_seq unchanged.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseReachedWaypointArmsTransitionBeforeAdvancing)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),                                                   // idx 0: WP1
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),                 // idx 1: attached to WP1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt + 20.f),                                   // idx 2: WP2
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),                 // idx 3: attached to WP2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt + 10.f),                                   // idx 4: WP3
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(2);
	executor.setSafePointSelectionForTest(true, -1);
	executor.setVehicleStatusForTest(true, false, false);
	executor.setLandedForTest(false);

	const bool advanced = executor.advanceStageForTest();

	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::TransitionDuringRoute);
	EXPECT_EQ(executor.currentSequenceForTest(), 2);
	EXPECT_EQ(executor.transitionTargetIndexForTest(), 2);
	EXPECT_EQ(executor.transitionActionForTest(), RtlMissionSafePointFollowTestPeer::VtolTransitionAction::FrontTransition);
	EXPECT_FALSE(executor.transitionCommandSentForTest());
	EXPECT_TRUE(executor.advanceRouteAfterTransitionForTest());
}

// WHY: Once the reverse transition is armed, the controller must track the next reverse target
//      during the transition instead of flying back toward the waypoint that was already reached.
// WHAT: TransitionDuringRoute publishes the previous route target as current setpoint for reverse post-reach transitions.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseRouteTransitionPublishesPreviousTargetDuringTransition)
{
	Navigator navigator;
	RtlMissionSafePointFollowTestPeer executor_with_nav(&navigator);

	std::vector<mission_item_s> items = {
		makePositionItem(kBaseLat, kBaseLon, kAlt),                                                   // idx 0: WP1
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),                 // idx 1: attached to WP1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt + 20.f),                                   // idx 2: WP2
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),                 // idx 3: attached to WP2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt + 10.f),                                   // idx 4: WP3
	};

	executor_with_nav.loadTestMission(items);
	executor_with_nav.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::TransitionDuringRoute);
	executor_with_nav.setCurrentSequenceForTest(2);
	executor_with_nav.setCurrentMissionItemForTest(items[2]);
	executor_with_nav.setSafePointSelectionForTest(true, -1);
	executor_with_nav.setTransitionTargetIndexForTest(2);
	executor_with_nav.setTransitionStateForTest(RtlMissionSafePointFollowTestPeer::VtolTransitionAction::FrontTransition,
			false, true);
	executor_with_nav.setVehicleStatusForTest(true, false, false);
	executor_with_nav.setLandedForTest(false);

	vehicle_global_position_s global_position{};
	global_position.lat = items[2].lat;
	global_position.lon = items[2].lon;
	global_position.alt = items[2].altitude;
	*navigator.get_global_position() = global_position;

	executor_with_nav.publishActiveMissionItemsForTest();

	const position_setpoint_triplet_s *triplet = navigator.get_position_setpoint_triplet();
	ASSERT_TRUE(triplet->current.valid);
	EXPECT_NEAR(triplet->current.lat, items[0].lat, 1e-9);
	EXPECT_NEAR(triplet->current.lon, items[0].lon, 1e-9);
	EXPECT_NEAR(triplet->current.alt, items[0].altitude, 1e-3f);
	EXPECT_EQ(triplet->current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
}

// WHY: Once a reverse reached-waypoint transition completes, the executor should advance the route
//      immediately instead of waiting to "re-reach" the same waypoint on the next cycle.
// WHAT: Completing a reverse route transition advances current_seq to the previous route target.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseRouteTransitionCompletionAdvancesToPreviousTarget)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::TransitionDuringRoute);
	executor.setCurrentSequenceForTest(2);
	executor.setSafePointSelectionForTest(true, -1);
	executor.setTransitionTargetIndexForTest(2);
	executor.setTransitionStateForTest(RtlMissionSafePointFollowTestPeer::VtolTransitionAction::FrontTransition, true,
					   true);

	const bool advanced = executor.advanceStageForTest();

	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	EXPECT_EQ(executor.currentSequenceForTest(), 1);
	EXPECT_EQ(executor.transitionTargetIndexForTest(), -1);
}

// WHY: After the virtual branch-off waypoint is reached, the executor must commit to the final landing stage.
// WHAT: setNextMissionItem moves BranchOff to LandAtGoal.
TEST_F(RtlMissionSafePointFollowStageTest, BranchOffTransitionsToLandAtGoal)
{
	// GIVEN: An executor that has already reached the branch-off waypoint.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::BranchOff);

	// WHEN: setNextMissionItem advances the stage machine.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor commits to the landing stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
}

// WHY: Route-safe-point RTL should use the same wind-selected VTOL approach behavior as direct RTL
//      once the safe point has already been chosen, instead of going straight from branch-off to land.
// WHAT: With a valid goal approach configured, setNextMissionItem moves BranchOff to ApproachAtGoal.
TEST_F(RtlMissionSafePointFollowStageTest, BranchOffTransitionsToApproachAtGoalWhenGoalApproachValid)
{
	// GIVEN: An executor that has reached the branch-off waypoint for a safe point with a chosen approach.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});

	loiter_point_s goal_land_approach{};
	goal_land_approach.lat = kBaseLat + 0.0005;
	goal_land_approach.lon = kBaseLon + 0.0002;
	goal_land_approach.height_m = kAlt + 20.f;
	goal_land_approach.loiter_radius_m = 60.f;
	executor.setGoalLandApproachForTest(goal_land_approach);
	executor.setSafePointSelectionForTest(false, 1);
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::BranchOff);

	// WHEN: setNextMissionItem advances the stage machine.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor commits to the goal-approach stage before landing.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::ApproachAtGoal);
}

// WHY: Once the goal approach loiter has been completed, the executor must hand over to the
//      shared MissionBase landing pipeline instead of staying in the loiter stage.
// WHAT: setNextMissionItem moves ApproachAtGoal to LandAtGoal.
TEST_F(RtlMissionSafePointFollowStageTest, ApproachAtGoalTransitionsToLandAtGoal)
{
	// GIVEN: An executor already flying the selected safe-point landing approach.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::ApproachAtGoal);

	// WHEN: setNextMissionItem advances the stage machine.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor leaves the approach stage and enters the landing stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
}

// WHY: The final branch-off segment now stays in FollowRoute so handleFollowRouteStage can still
//      evaluate a pending VTOL transition before the virtual branch-off waypoint is published.
// WHAT: Advancing forward onto the branch-off index updates current_seq but leaves the stage in FollowRoute.
TEST_F(RtlMissionSafePointFollowStageTest, ForwardRouteAdvanceKeepsFollowRouteUntilBranchOffPublication)
{
	// GIVEN: A forward route with the next position item equal to the cached branch-off index.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(1);
	executor.setSafePointSelectionForTest(false, 2);

	// WHEN: setNextMissionItem advances along the nominal route.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor advances to the branch-off anchor but defers the BranchOff stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	EXPECT_EQ(executor.currentSequenceForTest(), 2);
}

// WHY: Reverse route following also keeps FollowRoute active on the branch-off anchor so the
//      final segment can still stage a VTOL transition before BranchOff takes over.
// WHAT: Advancing backward onto the branch-off index updates current_seq but leaves the stage unchanged.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseRouteAdvanceKeepsFollowRouteUntilBranchOffPublication)
{
	// GIVEN: A reverse route whose previous position item is the cached branch-off index.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(2);
	executor.setSafePointSelectionForTest(true, 1);

	// WHEN: setNextMissionItem advances along the reverse route.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor reaches the branch-off anchor but defers the BranchOff stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	EXPECT_EQ(executor.currentSequenceForTest(), 1);
}

// WHY: Route-follow exhaustion should continue RTL toward the selected goal.
// WHAT: When forward traversal is already at the last route item, advancing from FollowRoute moves to LandAtGoal.
TEST_F(RtlMissionSafePointFollowStageTest, ForwardRouteExhaustionTransitionsToLandAtGoal)
{
	// GIVEN: A forward route whose current sequence is already the final position item.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(1);

	// WHEN: setNextMissionItem tries to advance beyond the route end.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor keeps RTL alive by handing over to the landing stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
}

// WHY: When route traversal exhausts before reaching a chosen safe point, the executor should
//      still fly the selected VTOL approach before landing instead of dropping straight into land.
// WHAT: With a valid goal approach configured, forward route exhaustion moves FollowRoute to ApproachAtGoal.
TEST_F(RtlMissionSafePointFollowStageTest, ForwardRouteExhaustionTransitionsToApproachAtGoalWhenGoalApproachValid)
{
	// GIVEN: A forward route whose current sequence is already the last position item, plus a chosen goal approach.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});

	loiter_point_s goal_land_approach{};
	goal_land_approach.lat = kBaseLat + 0.0005;
	goal_land_approach.lon = kBaseLon + 0.0002;
	goal_land_approach.height_m = kAlt + 20.f;
	goal_land_approach.loiter_radius_m = 60.f;
	executor.setGoalLandApproachForTest(goal_land_approach);
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(1);
	executor.setSafePointSelectionForTest(false, -1);

	// WHEN: setNextMissionItem tries to advance beyond the route end.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor continues with the approach stage instead of going straight to land.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::ApproachAtGoal);
}

// WHY: Reverse route exhaustion should also continue RTL toward the selected goal.
// WHAT: When reverse traversal is already at the first route item, advancing from FollowRoute moves to LandAtGoal.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseRouteExhaustionTransitionsToLandAtGoal)
{
	// GIVEN: A reverse route whose current sequence is already the first position item.
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(0);
	executor.setSafePointSelectionForTest(true, 0);

	// WHEN: setNextMissionItem tries to advance past the reverse route start.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor keeps RTL alive by handing over to the landing stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
}

// WHY: Inactive executors should not keep reporting a landing stage from a previous run.
// WHAT: Resetting transient executor progress clears the stage and remembered transition target.
TEST_F(RtlMissionSafePointFollowStageTest, ResetExecutorProgressClearsStageAndTransitionTarget)
{
	executor.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::ApproachAtGoal);
	executor.setTransitionTargetIndexForTest(1);

	executor.resetExecutorProgressForTest();

	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::Idle);
	EXPECT_EQ(executor.transitionTargetIndexForTest(), -1);
}

// WHY: Route-safe-point RTL follows mission geometry, but takeoff commands carry altitude semantics
//      that differ from a plain waypoint and must not be silently rewritten.
// WHAT: normalizeRouteMissionItem leaves NAV_CMD_TAKEOFF unchanged.
TEST_F(RtlMissionSafePointFollowStageTest, NormalizeRouteMissionItemPreservesTakeoffCommand)
{
	mission_item_s takeoff_item = makeTakeoffItem(kBaseLat, kBaseLon, kAlt + 30.f);
	takeoff_item.time_inside = 12.f;

	executor.normalizeRouteMissionItemForTest(takeoff_item);

	EXPECT_EQ(takeoff_item.nav_cmd, NAV_CMD_TAKEOFF);
	EXPECT_FLOAT_EQ(takeoff_item.time_inside, 12.f);
	EXPECT_FALSE(takeoff_item.autocontinue);
}

// WHY: Mission-endpoint fallback keys off the actual endpoint command encountered on the route,
//      so landing commands must remain intact instead of being flattened into route waypoints.
// WHAT: normalizeRouteMissionItem leaves NAV_CMD_LAND unchanged.
TEST_F(RtlMissionSafePointFollowStageTest, NormalizeRouteMissionItemPreservesLandingCommand)
{
	mission_item_s landing_item = makeLandItem(kBaseLat, kBaseLon, kAlt - 5.f);
	landing_item.time_inside = 9.f;

	executor.normalizeRouteMissionItemForTest(landing_item);

	EXPECT_EQ(landing_item.nav_cmd, NAV_CMD_LAND);
	EXPECT_FLOAT_EQ(landing_item.time_inside, 9.f);
	EXPECT_FALSE(landing_item.autocontinue);
}

// WHY: Loiter-style position items should still be flattened into geometry-only route waypoints
//      so RTL does not stop and wait at intermediate loiter commands while following the route.
// WHAT: normalizeRouteMissionItem converts NAV_CMD_LOITER_TO_ALT into NAV_CMD_WAYPOINT.
TEST_F(RtlMissionSafePointFollowStageTest, NormalizeRouteMissionItemFlattensLoiterCommand)
{
	mission_item_s loiter_item = makePositionItem(kBaseLat, kBaseLon, kAlt + 20.f, NAV_CMD_LOITER_TO_ALT);
	loiter_item.autocontinue = false;
	loiter_item.time_inside = 8.f;

	executor.normalizeRouteMissionItemForTest(loiter_item);

	EXPECT_EQ(loiter_item.nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_TRUE(loiter_item.autocontinue);
	EXPECT_FLOAT_EQ(loiter_item.time_inside, 0.f);
}
