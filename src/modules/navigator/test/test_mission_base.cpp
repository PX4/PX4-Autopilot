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
 * @file test_mission_base.cpp
 *
 * Unit tests for MissionBase position traversal helpers:
 *   - findNextPositionIndex()
 *   - findPreviousPositionIndex()
 *   - getNextPositionItems()
 *   - getPreviousPositionItems()
 *   - goToNextPositionItem()
 *   - goToPreviousPositionItem()
 *
 * These tests cover both traversal modes:
 *   - Follow mission control flow, which preserves the legacy DO_JUMP behavior
 *   - Ignore DO_JUMP, which treats jump items as geometry-free control items
 */

#include <gtest/gtest.h>

#include "mission_base.h"

#include <lib/parameters/param.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>

#include <initializer_list>
#include <vector>

extern "C" int dataman_main(int argc, char *argv[]);

class NavigatorDatamanRuntime
{
public:
	NavigatorDatamanRuntime()
	{
		param_control_autosave(false);
		px4::WorkQueueManagerStart();

		char name[] = "dataman";
		char start[] = "start";
		char ram[] = "-r";
		char *argv[] = {name, start, ram};
		dataman_main(3, argv);
	}

	~NavigatorDatamanRuntime()
	{
		param_control_autosave(true);

		char name[] = "dataman";
		char stop[] = "stop";
		char *argv[] = {name, stop};
		dataman_main(2, argv);

		px4::WorkQueueManagerStop();
	}
};

static NavigatorDatamanRuntime &navigatorDatamanRuntime()
{
	static NavigatorDatamanRuntime runtime{};
	return runtime;
}

class MissionBaseTestPeer : public MissionBase
{
public:
	MissionBaseTestPeer() : MissionBase(nullptr, 8, 0) {}

	void setActiveMissionItems() override {}
	bool setNextMissionItem() override { return false; }

	bool loadMissionItemFromCache(int32_t index, mission_item_s &mission_item) override
	{
		for (int32_t failed_index : _load_failure_indices) {
			if (failed_index == index) {
				return false;
			}
		}

		if (index < 0 || index >= static_cast<int32_t>(_items.size())) {
			return false;
		}

		mission_item = _items[static_cast<size_t>(index)];
		return true;
	}

	void loadTestMission(const std::vector<mission_item_s> &items)
	{
		_items = items;
		_mission.count = static_cast<int32_t>(items.size());
		_mission.current_seq = 0;
		clearLoadFailures();
	}

	void setLoadFailureIndices(std::initializer_list<int32_t> indices)
	{
		_load_failure_indices.assign(indices.begin(), indices.end());
	}

	void clearLoadFailures()
	{
		_load_failure_indices.clear();
	}

	void setCurrentSequence(int32_t current_seq)
	{
		_mission.current_seq = current_seq;
	}

	int32_t currentSequence() const
	{
		return _mission.current_seq;
	}

	using MissionBase::findNextPositionIndex;
	using MissionBase::findPreviousPositionIndex;
	using MissionBase::getNextPositionItems;
	using MissionBase::getPreviousPositionItems;
	using MissionBase::goToNextPositionItem;
	using MissionBase::goToPreviousPositionItem;
	using MissionBase::PositionTraversalType;

private:
	std::vector<mission_item_s> _items;
	std::vector<int32_t> _load_failure_indices;
};

static constexpr double kBaseLat = 47.0;
static constexpr double kBaseLon = 8.0;
static constexpr float kAlt = 100.f;

static mission_item_s makePositionItem(double lat, double lon, float altitude)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_WAYPOINT;
	item.lat = lat;
	item.lon = lon;
	item.altitude = altitude;
	return item;
}

static mission_item_s makeDoJump(int32_t target_index, uint16_t repeat_count, uint16_t current_count = 0)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_DO_JUMP;
	item.do_jump_mission_index = target_index;
	item.do_jump_repeat_count = repeat_count;
	item.do_jump_current_count = current_count;
	return item;
}

static mission_item_s makeVtolTransitionItem(int transition_mode)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
	item.params[0] = static_cast<float>(transition_mode);
	return item;
}

class MissionBaseTraversalTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite()
	{
		(void)navigatorDatamanRuntime();
	}

	static void TearDownTestSuite() {}

	MissionBaseTestPeer mission_base{};
};

// WHY: Geometry-only position traversal must skip non-position mission items while searching for the next waypoint.
// WHAT: Starting from a VTOL transition item, the helper skips it and returns the next waypoint.
TEST_F(MissionBaseTraversalTest, FindNextSkipsNonPositionItems)
{
	// GIVEN: A position item, a non-position VTOL transition, and then another position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});

	int32_t next_index{-1};

	// WHEN: Geometry-only traversal searches forward from the non-position item.
	const bool found = mission_base.findNextPositionIndex(1, next_index,
			   MissionBaseTestPeer::PositionTraversalType::IgnoreDoJump);

	// THEN: The next position item is returned.
	EXPECT_TRUE(found);
	EXPECT_EQ(next_index, 2);
}

// WHY: Geometry-only traversal must not execute DO_JUMP commands. It must treat
//      jump items as non-position items and keep scanning for the next physical waypoint.
// WHAT: [WP, DO_JUMP, WP, WP] starting from idx 1 returns idx 2.
TEST_F(MissionBaseTraversalTest, FindNextSkipsDoJumpItems)
{
	// GIVEN: A mission where a DO_JUMP sits between two position items.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 3), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});

	int32_t next_index{-1};

	// WHEN: Geometry-only traversal starts at the DO_JUMP item.
	const bool found = mission_base.findNextPositionIndex(1, next_index,
			   MissionBaseTestPeer::PositionTraversalType::IgnoreDoJump);

	// THEN: The first position item after the jump is returned.
	EXPECT_TRUE(found);
	EXPECT_EQ(next_index, 2);
}

// WHY: Consecutive non-position must be skipped.
// WHAT: [WP, DO_JUMP, VTOL_FW, WP] starting from idx 1 returns idx 3.
TEST_F(MissionBaseTraversalTest, FindNextSkipsConsecutiveNonPositionItems)
{
	// GIVEN: Consecutive non-position items before the next waypoint.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 5), // idx 1
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW), // idx 2
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 3
	});

	int32_t next_index{-1};

	// WHEN: Geometry-only traversal walks forward through the control items.
	const bool found = mission_base.findNextPositionIndex(1, next_index,
			   MissionBaseTestPeer::PositionTraversalType::IgnoreDoJump);

	// THEN: It returns the first following position item.
	EXPECT_TRUE(found);
	EXPECT_EQ(next_index, 3);
}

// WHY: Callers need a failure when no later position item exists.
// WHAT: [WP, DO_JUMP] starting from idx 1 returns false.
TEST_F(MissionBaseTraversalTest, FindNextReturnsFalseAtEnd)
{
	// GIVEN: A mission with no position item after the starting index.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 3), // idx 1
	});

	int32_t next_index{-1};

	// WHEN: Geometry-only traversal searches past the last item.
	const bool found = mission_base.findNextPositionIndex(1, next_index,
			   MissionBaseTestPeer::PositionTraversalType::IgnoreDoJump);

	// THEN: The helper reports that no next position item exists.
	EXPECT_FALSE(found);
	EXPECT_EQ(next_index, -1);
}

// WHY: Traversal should fail cleanly when a mission item cannot be loaded.
// WHAT: A cache failure on the next position item makes findNextPositionIndex() return false.
TEST_F(MissionBaseTraversalTest, FindNextReturnsFalseOnCacheReadFailure)
{
	// GIVEN: A mission where the next position item cannot be loaded.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 3), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});
	mission_base.setLoadFailureIndices({2});

	int32_t next_index{-1};

	// WHEN: Geometry-only traversal advances past the DO_JUMP item.
	const bool found = mission_base.findNextPositionIndex(1, next_index,
			   MissionBaseTestPeer::PositionTraversalType::IgnoreDoJump);

	// THEN: The unreadable waypoint produces a clean failure.
	EXPECT_FALSE(found);
	EXPECT_EQ(next_index, -1);
}

// WHY: Geometry-only backward traversal must skip DO_JUMP items.
// WHAT: [WP, WP, DO_JUMP, WP] starting from idx 3 returns idx 1.
TEST_F(MissionBaseTraversalTest, FindPreviousSkipsDoJumpItems)
{
	// GIVEN: A DO_JUMP directly before the current physical waypoint.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 3), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});

	int32_t previous_index{-1};

	// WHEN: Geometry-only traversal searches backward from the last waypoint.
	const bool found = mission_base.findPreviousPositionIndex(3, previous_index,
			   MissionBaseTestPeer::PositionTraversalType::IgnoreDoJump);

	// THEN: It returns the previous physical waypoint, not the jump item.
	EXPECT_TRUE(found);
	EXPECT_EQ(previous_index, 1);
}

// WHY: Backward traversal must continue to preserve legacy skipping behavior across multiple jumps.
// WHAT: [WP, DO_JUMP, DO_JUMP, WP] starting from idx 3 returns idx 0.
TEST_F(MissionBaseTraversalTest, FindPreviousSkipsConsecutiveDoJumps)
{
	// GIVEN: Two consecutive DO_JUMP items before the current waypoint.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 3), // idx 1
		makeDoJump(0, 2), // idx 2
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 3
	});

	int32_t previous_index{-1};

	// WHEN: Geometry-only traversal walks backward across both jumps.
	const bool found = mission_base.findPreviousPositionIndex(3, previous_index,
			   MissionBaseTestPeer::PositionTraversalType::IgnoreDoJump);

	// THEN: It returns the nearest earlier physical waypoint.
	EXPECT_TRUE(found);
	EXPECT_EQ(previous_index, 0);
}

// WHY: Backward traversal should report failure when only jumps exist before the start index.
// WHAT: [DO_JUMP, WP] starting from idx 1 returns false.
TEST_F(MissionBaseTraversalTest, FindPreviousReturnsFalseWhenOnlyJumpsBefore)
{
	// GIVEN: No earlier position item exists before the current waypoint.
	mission_base.loadTestMission({
		makeDoJump(0, 3), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
	});

	int32_t previous_index{-1};

	// WHEN: Geometry-only traversal searches backward.
	const bool found = mission_base.findPreviousPositionIndex(1, previous_index,
			   MissionBaseTestPeer::PositionTraversalType::IgnoreDoJump);

	// THEN: The helper reports that no earlier position item exists.
	EXPECT_FALSE(found);
	EXPECT_EQ(previous_index, -1);
}

// WHY: The shared traversal API must expose both behaviors on the same mission.
// WHAT: [DO_JUMP->2, WP1, WP2] starting from idx 0 resolves to idx 2 in mission-control
//       mode and idx 1 in geometry-only mode.
TEST_F(MissionBaseTraversalTest, FindNextSupportsBothTraversalSemantics)
{
	// GIVEN: A jump whose target is a later physical waypoint.
	mission_base.loadTestMission({
		makeDoJump(2, 1, 0), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});

	int32_t next_follow{-1};
	int32_t next_geometry{-1};

	// WHEN: The same lookup is performed in both traversal modes.
	const bool found_follow = mission_base.findNextPositionIndex(0, next_follow,
				  MissionBaseTestPeer::PositionTraversalType::FollowMissionControlFlow);
	const bool found_geometry = mission_base.findNextPositionIndex(0, next_geometry,
				    MissionBaseTestPeer::PositionTraversalType::IgnoreDoJump);

	// THEN: Mission-control mode follows the jump, while geometry-only mode skips it.
	EXPECT_TRUE(found_follow);
	EXPECT_TRUE(found_geometry);
	EXPECT_EQ(next_follow, 2);
	EXPECT_EQ(next_geometry, 1);
}

// WHY: Backward single-item lookup also needs both semantics exposed on the same mission.
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] starting from idx 3 resolves to idx 0 in mission-control
//       mode and idx 1 in geometry-only mode.
TEST_F(MissionBaseTraversalTest, FindPreviousSupportsBothTraversalSemantics)
{
	// GIVEN: A jump loop between the current waypoint and the adjacent physical waypoint.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 2, 0), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});

	int32_t previous_follow{-1};
	int32_t previous_geometry{-1};

	// WHEN: The same backward lookup is performed in both traversal modes.
	const bool found_follow = mission_base.findPreviousPositionIndex(3, previous_follow,
				  MissionBaseTestPeer::PositionTraversalType::FollowMissionControlFlow);
	const bool found_geometry = mission_base.findPreviousPositionIndex(3, previous_geometry,
				    MissionBaseTestPeer::PositionTraversalType::IgnoreDoJump);

	// THEN: Mission-control mode follows the active jump, while geometry-only mode skips it.
	EXPECT_TRUE(found_follow);
	EXPECT_TRUE(found_geometry);
	EXPECT_EQ(previous_follow, 0);
	EXPECT_EQ(previous_geometry, 1);
}

// WHY: The original bug was that goToNextPositionItem(IgnoreDoJump) still followed DO_JUMP
//      control flow.
// WHAT: [DO_JUMP->2, WP1, WP2] from current_seq=-1 should land on idx 1 in geometry-only mode.
TEST_F(MissionBaseTraversalTest, GoToNextPositionItemHonorsExecuteJumpFlag)
{
	// GIVEN: A mission whose first item is a DO_JUMP.
	mission_base.loadTestMission({
		makeDoJump(2, 1, 0), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});
	mission_base.setCurrentSequence(-1);

	// WHEN: The caller explicitly requests geometry-only traversal.
	const int ret = mission_base.goToNextPositionItem(MissionBaseTestPeer::PositionTraversalType::IgnoreDoJump);

	// THEN: Traversal lands on the first physical waypoint instead of the jump target.
	EXPECT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_base.currentSequence(), 1);
}

// WHY: The refactor must not change the legacy mission-control behavior.
// WHAT: [DO_JUMP->2, WP1, WP2] from current_seq=-1 should still land on idx 2.
TEST_F(MissionBaseTraversalTest, GoToNextPositionItemFollowsMissionControlFlowWhenEnabled)
{
	// GIVEN: A mission whose first item is an active DO_JUMP.
	mission_base.loadTestMission({
		makeDoJump(2, 1, 0), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});
	mission_base.setCurrentSequence(-1);

	// WHEN: The caller requests mission-control traversal.
	const int ret = mission_base.goToNextPositionItem(MissionBaseTestPeer::PositionTraversalType::FollowMissionControlFlow);

	// THEN: Traversal follows the jump target exactly as before.
	EXPECT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_base.currentSequence(), 2);
}

// WHY: The same traversal-mode bug existed in the backward position API and should be covered too.
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] from current_seq=3 should land on idx 1 in geometry-only mode.
TEST_F(MissionBaseTraversalTest, GoToPreviousPositionItemHonorsExecuteJumpFlag)
{
	// GIVEN: A DO_JUMP between the current waypoint and the previous physical waypoint.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 2, 0), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});
	mission_base.setCurrentSequence(3);

	// WHEN: The caller explicitly requests geometry-only traversal.
	const int ret = mission_base.goToPreviousPositionItem(MissionBaseTestPeer::PositionTraversalType::IgnoreDoJump);

	// THEN: Traversal lands on the adjacent physical waypoint instead of following the jump.
	EXPECT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_base.currentSequence(), 1);
}

// WHY: The backward wrapper must also preserve the legacy mission-control behavior.
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] from current_seq=3 should still land on idx 0.
TEST_F(MissionBaseTraversalTest, GoToPreviousPositionItemFollowsMissionControlFlowWhenEnabled)
{
	// GIVEN: A mission with an active jump loop before the current waypoint.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 2, 0), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});
	mission_base.setCurrentSequence(3);

	// WHEN: The caller requests mission-control traversal.
	const int ret = mission_base.goToPreviousPositionItem(MissionBaseTestPeer::PositionTraversalType::FollowMissionControlFlow);

	// THEN: Traversal follows the active jump exactly as before.
	EXPECT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_base.currentSequence(), 0);
}

// WHY: Existing mission execution relies on getNextPositionItems() following active DO_JUMP
//      control flow by default.
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] starting from idx 2 returns idx 0 then idx 1.
TEST_F(MissionBaseTraversalTest, GetNextPositionItemsFollowsActiveDoJump)
{
	// GIVEN: A mission with an active jump loop.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 2, 0), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});

	int32_t next_items[2] = {-1, -1};
	size_t num_found_items = 0;

	// WHEN: The legacy multi-item helper walks forward with default traversal semantics.
	mission_base.getNextPositionItems(2, next_items, num_found_items, 2u);

	// THEN: The active DO_JUMP is followed.
	ASSERT_EQ(num_found_items, 2u);
	EXPECT_EQ(next_items[0], 0);
	EXPECT_EQ(next_items[1], 1);
}

// WHY: Existing reverse mission flows rely on getPreviousPositionItems() preserving the
//      legacy behavior of following active DO_JUMP control flow by default.
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] starting from idx 3 returns idx 0.
TEST_F(MissionBaseTraversalTest, GetPreviousPositionItemsFollowsActiveDoJump)
{
	// GIVEN: A mission with an active jump loop before the current waypoint.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 2, 0), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});

	int32_t previous_items[1] = {-1};
	size_t num_found_items = 0;

	// WHEN: The legacy multi-item helper walks backward with default traversal semantics.
	mission_base.getPreviousPositionItems(3, previous_items, num_found_items, 1u);

	// THEN: The active DO_JUMP is followed.
	ASSERT_EQ(num_found_items, 1u);
	EXPECT_EQ(previous_items[0], 0);
}
