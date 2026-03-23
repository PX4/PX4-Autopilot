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
		_stage = Stage::Idle;
		_branch_off_index = -1;
		_should_go_straight_to_goal = false;
		_transition_target_index = -1;
		_plan = {};
	}

	void setStageForTest(Stage stage)
	{
		_stage = stage;
	}

	Stage stageForTest() const
	{
		return _stage;
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
		_branch_off_index = branch_off_index;
	}

	void setTransitionTargetIndexForTest(int32_t index)
	{
		_transition_target_index = index;
	}

	int32_t transitionTargetIndexForTest() const
	{
		return _transition_target_index;
	}

	bool shouldGoStraightToGoalForTest() const
	{
		return _should_go_straight_to_goal;
	}

	bool advanceStageForTest()
	{
		return setNextMissionItem();
	}

private:
	std::vector<mission_item_s> _items;
};

class RtlMissionSafePointFollowStageTest : public ::testing::Test
{
protected:
	RtlMissionSafePointFollowTestPeer executor{};

	void SetUp() override
	{
		executor.loadTestMission({});
	}

	static constexpr double kLat = 47.397742;
	static constexpr double kLon = 8.545594;
	static constexpr float kAlt = 500.f;
};

// WHY: TransitionDuringRoute is a one-shot state used only to issue a VTOL transition.
// WHAT: setNextMissionItem returns to FollowRoute and clears the remembered transition target.
TEST_F(RtlMissionSafePointFollowStageTest, TransitionDuringRouteResumesFollowRoute)
{
	// GIVEN: An executor paused in the transition stage with a remembered target index.
	executor.loadTestMission({
		makePositionItem(kLat, kLon, kAlt),
		makePositionItem(kLat + 0.001, kLon, kAlt),
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
// WHAT: setNextMissionItem moves BranchOff to LandAtGoal and latches straight-to-goal mode.
TEST_F(RtlMissionSafePointFollowStageTest, BranchOffTransitionsToLandAtGoal)
{
	// GIVEN: An executor that has already reached the branch-off waypoint.
	executor.loadTestMission({
		makePositionItem(kLat, kLon, kAlt),
		makePositionItem(kLat + 0.001, kLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::BranchOff);

	// WHEN: setNextMissionItem advances the stage machine.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor commits to the landing stage and stays on the direct branch.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_TRUE(executor.shouldGoStraightToGoalForTest());
}

// WHY: Nominal route following must switch to BranchOff exactly when the next route target is the branch-off anchor.
// WHAT: Advancing forward onto the branch-off index moves FollowRoute to BranchOff.
TEST_F(RtlMissionSafePointFollowStageTest, ForwardRouteAdvanceTransitionsToBranchOff)
{
	// GIVEN: A forward route with the next position item equal to the cached branch-off index.
	executor.loadTestMission({
		makePositionItem(kLat, kLon, kAlt),
		makePositionItem(kLat + 0.001, kLon, kAlt),
		makePositionItem(kLat + 0.002, kLon, kAlt),
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
		makePositionItem(kLat, kLon, kAlt),
		makePositionItem(kLat + 0.001, kLon, kAlt),
		makePositionItem(kLat + 0.002, kLon, kAlt),
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
		makePositionItem(kLat, kLon, kAlt),
		makePositionItem(kLat + 0.001, kLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(1);

	// WHEN: setNextMissionItem tries to advance beyond the route end.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor keeps RTL alive by handing over to the landing stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_TRUE(executor.shouldGoStraightToGoalForTest());
}

// WHY: Reverse route exhaustion should also continue RTL toward the selected goal.
// WHAT: When reverse traversal is already at the first route item, advancing from FollowRoute moves to LandAtGoal.
TEST_F(RtlMissionSafePointFollowStageTest, ReverseRouteExhaustionTransitionsToLandAtGoal)
{
	// GIVEN: A reverse route whose current sequence is already the first position item.
	executor.loadTestMission({
		makePositionItem(kLat, kLon, kAlt),
		makePositionItem(kLat + 0.001, kLon, kAlt),
	});
	executor.setStageForTest(RtlMissionSafePointFollowTestPeer::Stage::FollowRoute);
	executor.setCurrentSequenceForTest(0);
	executor.setSafePointSelectionForTest(true, 0);

	// WHEN: setNextMissionItem tries to advance past the reverse route start.
	const bool advanced = executor.advanceStageForTest();

	// THEN: The executor keeps RTL alive by handing over to the landing stage.
	EXPECT_TRUE(advanced);
	EXPECT_EQ(executor.stageForTest(), RtlMissionSafePointFollowTestPeer::Stage::LandAtGoal);
	EXPECT_TRUE(executor.shouldGoStraightToGoalForTest());
}
