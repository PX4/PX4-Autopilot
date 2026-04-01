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

#include "rtl_mission_safe_point_follow.h"
#include "test_RTL_helpers.h"

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

	RtlMissionSafePointFollowTestPeer()
		: RtlMissionSafePointFollow(nullptr, mission_s{})
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

	void setSafePointSelectionForTest(bool direction_reversed, int32_t branch_off_index)
	{
		_plan = {};
		_plan.selection.found = true;
		_plan.selection.safe_point_found = true;
		_plan.selection.goal_type = MissionRoutePlanner::GoalType::SafePoint;
		_plan.selection.path.direction_reversed = direction_reversed;
		_state.branch_off_index = branch_off_index;
	}

	void setTransitionTargetIndexForTest(int32_t index)
	{
		_state.transition_target_index = index;
	}

	void setGoalLandApproachForTest(const loiter_point_s &land_approach)
	{
		_goal_land_approach = land_approach;
	}

	int32_t transitionTargetIndexForTest() const
	{
		return _state.transition_target_index;
	}

	bool advanceStageForTest()
	{
		return setNextMissionItem();
	}

	void normalizeRouteMissionItemForTest(mission_item_s &mission_item) const
	{
		normalizeRouteMissionItem(mission_item);
	}

	void resetExecutorProgressForTest()
	{
		resetExecutorProgress();
	}

private:
	std::vector<mission_item_s> _items;
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

	// WHEN: setNextMissionItem advances the stage machine.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The transition stage completes and route following resumes.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
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

// WHY: Nominal route following must switch to BranchOff exactly when the next route target is the branch-off anchor.
// WHAT: Advancing forward onto the branch-off index moves FollowRoute to BranchOff.
TEST_F(RtlMissionSafePointFollowStageTest, ForwardRouteAdvanceTransitionsToBranchOff)
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

	// THEN: The executor advances to the branch-off anchor and switches stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::BranchOff);
}

// WHY: Reverse route following must use the previous position item as the next route target.
// WHAT: Advancing backward onto the branch-off index moves FollowRoute to BranchOff.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseRouteAdvanceTransitionsToBranchOff)
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

	// THEN: The executor reaches the branch-off anchor and switches stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::BranchOff);
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
