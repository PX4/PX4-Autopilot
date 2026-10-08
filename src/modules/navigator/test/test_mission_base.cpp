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
 * MissionBase position traversal tests.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include "mission.h"
#include "mission_base.h"
#include "navigator.h"
#include "support/mission_route_cache_test_peer.h"
#include "support/navigator_dataman_test.h"
#include "support/vector_mission_item_store.h"

#include <initializer_list>
#include <vector>

#include <cstring>
#include <uORB/Subscription.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

class MissionBaseTestPeer : public MissionBase
{
public:
	explicit MissionBaseTestPeer(Navigator *navigator = nullptr) : MissionBase(navigator, 8, 0) {}

	void setActiveMissionItems() override {}
	bool setNextMissionItem() override { return false; }

	bool loadMissionItemFromCache(int32_t index, mission_item_s &mission_item) override
	{
		return _mission_store.loadItem(index, mission_item);
	}

	void loadTestMission(const std::vector<mission_item_s> &items)
	{
		_mission_store.setItems(items);
		_mission.count = static_cast<int32_t>(_mission_store.itemCount());
		_mission.current_seq = 0;
	}

	void loadTestMission(const std::vector<mission_item_s> &items, const mission_s &mission)
	{
		loadTestMission(items);
		_mission = mission;
		_mission.count = static_cast<int32_t>(_mission_store.itemCount());
		_mission.current_seq = 0;
	}

	void setLoadFailureIndices(std::initializer_list<int32_t> indices)
	{
		_mission_store.setLoadFailureIndices(indices);
	}

	void clearLoadFailures()
	{
		_mission_store.clearLoadFailures();
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
	using MissionBase::getNonJumpItem;
	using MissionBase::getNextPositionItems;
	using MissionBase::getPreviousPositionItems;
	using MissionBase::goToNextPositionItem;
	using MissionBase::goToPreviousPositionItem;
	using MissionBase::MissionTraversalType;
	using MissionBase::resetMissionJumpCounter;

private:
	navigator_test::VectorMissionItemStore _mission_store{};
};

class IgnoreDoJumpMissionBaseTestPeer : public MissionBaseTestPeer
{
protected:
	MissionTraversalType traversalType() const override
	{
		return MissionTraversalType::IgnoreDoJump;
	}
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

class MissionBaseTraversalTest : public NavigatorDatamanTestBase
{
protected:
	MissionBaseTestPeer mission_base{};
};

class IgnoreDoJumpMissionBaseTraversalTest : public NavigatorDatamanTestBase
{
protected:
	IgnoreDoJumpMissionBaseTestPeer mission_base{};
};

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
class MissionBaseRouteCacheSyncTest : public NavigatorDatamanTestBase
{
protected:
	void SetUp() override
	{
		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_WAYPOINTS_OFFBOARD_0));
		_navigator.get_mission_route_cache().invalidate();
	}

	void TearDown() override
	{
		_navigator.get_mission_route_cache().invalidate();
	}

	DatamanClient _dataman_client{};
	Navigator _navigator{};
	MissionBaseTestPeer _mission_base{&_navigator};
};

TEST_F(MissionBaseRouteCacheSyncTest, DoJumpWritesKeepRouteCacheCurrent)
{
	const std::vector<mission_item_s> items{
		makeDoJump(1, 2),
		makePositionItem(kBaseLat, kBaseLon, kAlt),
	};

	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.mission_id = 1;
	mission.count = static_cast<uint16_t>(items.size());
	mission.current_seq = 0;
	mission.land_start_index = -1;
	mission.land_index = -1;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	_mission_base.loadTestMission(items, mission);

	for (size_t i = 0; i < items.size(); ++i) {
		mission_item_s item = items[i];
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, static_cast<uint32_t>(i),
						      reinterpret_cast<uint8_t *>(&item), sizeof(item)));
	}

	MissionRouteCache &route_cache = _navigator.get_mission_route_cache();
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(route_cache, mission,
			[&] { return route_cache.missionItemsReady(mission); }));

	int32_t mission_index = 0;
	mission_item_s mission_item{};
	ASSERT_EQ(_mission_base.getNonJumpItem(mission_index, mission_item,
					       MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow, true), PX4_OK);

	mission_item_s cached_item{};
	ASSERT_TRUE(route_cache.loadMissionItem(mission, 0, cached_item));
	EXPECT_EQ(cached_item.do_jump_current_count, 1);

	mission_item_s stored_item{};
	ASSERT_TRUE(_dataman_client.readSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0,
					     reinterpret_cast<uint8_t *>(&stored_item), sizeof(stored_item)));
	EXPECT_EQ(stored_item.do_jump_current_count, 1);

	_mission_base.resetMissionJumpCounter();

	ASSERT_TRUE(route_cache.loadMissionItem(mission, 0, cached_item));
	EXPECT_EQ(cached_item.do_jump_current_count, 0);
	ASSERT_TRUE(_dataman_client.readSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0,
					     reinterpret_cast<uint8_t *>(&stored_item), sizeof(stored_item)));
	EXPECT_EQ(stored_item.do_jump_current_count, 0);
}
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE

// WHY: getNonJumpItem is used to find the next mission item.
// WHAT: A non-DO_JUMP item is returned unchanged.
TEST_F(MissionBaseTraversalTest, GetNonJumpItemReturnsCurrentNonJumpItem)
{
	// GIVEN: A mission that starts with a normal position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 1, 0), // idx 1
	});

	int32_t mission_index = 0;
	mission_item_s mission_item{};

	// WHEN: The helper loads the current item directly.
	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			false, false);

	// THEN: It returns the same item and leaves the index unchanged.
	ASSERT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_index, 0);
	EXPECT_EQ(mission_item.nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_DOUBLE_EQ(mission_item.lat, kBaseLat);
	EXPECT_DOUBLE_EQ(mission_item.lon, kBaseLon);
	EXPECT_FLOAT_EQ(mission_item.altitude, kAlt);
}

// WHY: getNonJumpItem() must follow active DO_JUMP targets.
// WHAT: [DO_JUMP->2, WP1, WP2] starting from idx 0 returns idx 2.
TEST_F(MissionBaseTraversalTest, GetNonJumpItemFollowsActiveForwardDoJump)
{
	// GIVEN: A forward DO_JUMP that points to a later position item.
	mission_base.loadTestMission({
		makeDoJump(2, 1, 0), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});

	int32_t mission_index = 0;
	mission_item_s mission_item{};

	// WHEN: Mission-control traversal resolves the jump without writing counters.
	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			false, false);

	// THEN: The helper follows the jump and returns the target item.
	ASSERT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_index, 2);
	EXPECT_EQ(mission_item.nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_DOUBLE_EQ(mission_item.lat, kBaseLat + 0.001);
	EXPECT_DOUBLE_EQ(mission_item.lon, kBaseLon);
	EXPECT_FLOAT_EQ(mission_item.altitude, kAlt);
}

// WHY: Once a DO_JUMP has already used all repeats, callers should move on to the next item.
// WHAT: [WP0, DO_JUMP->0 done, WP2] starting from idx 1 returns idx 2.
TEST_F(MissionBaseTraversalTest, GetNonJumpItemSkipsDoJumpAfterLastRepeat)
{
	// GIVEN: A DO_JUMP whose repeat count is already reached.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 1, 1), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});

	int32_t mission_index = 1;
	mission_item_s mission_item{};

	// WHEN: The helper resolves the jump while traversing forward.
	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			false, false);

	// THEN: The jump is skipped and the next non-jump item is returned.
	ASSERT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_index, 2);
	EXPECT_EQ(mission_item.nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_DOUBLE_EQ(mission_item.lat, kBaseLat + 0.001);
}

// WHY: Reverse traversal that ignores DO_JUMP must step backward instead of following control flow.
// WHAT: [WP0, DO_JUMP->2, WP2] starting from idx 1 returns idx 0.
TEST_F(MissionBaseTraversalTest, GetNonJumpItemSkipsDoJumpBackwardWhenIgnoringJumps)
{
	// GIVEN: An active forward DO_JUMP with a valid non-jump item before it.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(2, 1, 0), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});

	int32_t mission_index = 1;
	mission_item_s mission_item{};

	// WHEN: The helper resolves the jump while moving backward in geometry-only mode.
	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump,
			false, true);

	// THEN: The DO_JUMP is skipped and the previous non-jump item is returned.
	ASSERT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_index, 0);
	EXPECT_EQ(mission_item.nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_DOUBLE_EQ(mission_item.lat, kBaseLat);
}

// WHY: Bad jump targets must return an error.
// WHAT: A DO_JUMP that points beyond the mission bounds returns PX4_ERROR.
TEST_F(MissionBaseTraversalTest, GetNonJumpItemReturnsErrorForOutOfBoundsDoJumpTarget)
{
	// GIVEN: A mission with a DO_JUMP that points outside the mission.
	mission_base.loadTestMission({
		makeDoJump(3, 1, 0), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
	});

	int32_t mission_index = 0;
	mission_item_s mission_item{};

	// WHEN: The helper tries to resolve that jump.
	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			false, false);

	// THEN: The helper returns an error.
	EXPECT_EQ(ret, PX4_ERROR);
	EXPECT_EQ(mission_index, 0);
}

// Fixture with a real Navigator so the storage failure path is observable, it
// publishes to mavlink_log through the navigator instead of a null pointer.
class MissionBasePastBoundsTraversalTest : public NavigatorDatamanTestBase
{
protected:
	bool storageErrorPublished()
	{
		mavlink_log_s report;

		while (_mavlink_log_sub.update(&report)) {
			if (strstr(reinterpret_cast<const char *>(report.text), "could not be read") != nullptr) {
				return true;
			}
		}

		return false;
	}

	Navigator _navigator{};
	MissionBaseTestPeer mission_base{&_navigator};
	uORB::Subscription _mavlink_log_sub{ORB_ID(mavlink_log)};
};

// WHY: Walking off the end of the mission while skipping an exhausted DO_JUMP is the
// normal end of a mission, not a storage failure.
// WHAT: [WP0, DO_JUMP->0 done] entered at the DO_JUMP returns PX4_ERROR without
// publishing a storage error.
TEST_F(MissionBasePastBoundsTraversalTest, GetNonJumpItemReturnsErrorPastMissionEnd)
{
	// GIVEN: A mission whose last item is a DO_JUMP with no repeats left.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 1, 1), // idx 1
	});

	int32_t mission_index = 1;
	mission_item_s mission_item{};
	(void)storageErrorPublished(); // drain anything already queued

	// WHEN: The helper skips the exhausted jump while traversing forward.
	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			false, false);

	// THEN: It reports no further item, exactly like an out of range entry index,
	// and no "could not be read" error is published.
	EXPECT_EQ(ret, PX4_ERROR);
	EXPECT_EQ(mission_index, 1);
	EXPECT_FALSE(storageErrorPublished());
}

// WHY: The same walk moving backward can step in front of the first item.
// WHAT: [DO_JUMP->1 done, WP1] entered at the DO_JUMP backward returns PX4_ERROR
// without publishing a storage error.
TEST_F(MissionBasePastBoundsTraversalTest, GetNonJumpItemReturnsErrorPastMissionStart)
{
	// GIVEN: A mission that starts with a DO_JUMP with no repeats left.
	mission_base.loadTestMission({
		makeDoJump(1, 1, 1), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
	});

	int32_t mission_index = 0;
	mission_item_s mission_item{};
	(void)storageErrorPublished(); // drain anything already queued

	// WHEN: The helper skips the exhausted jump while traversing backward.
	const int ret = mission_base.getNonJumpItem(mission_index, mission_item,
			MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow,
			false, true);

	// THEN: It reports no further item, exactly like an out of range entry index,
	// and no "could not be read" error is published.
	EXPECT_EQ(ret, PX4_ERROR);
	EXPECT_EQ(mission_index, 0);
	EXPECT_FALSE(storageErrorPublished());
}

// WHY: Geometry-only position traversal must skip non-position mission items.
// WHAT: Starting from a VTOL transition item, the helper skips it and returns the next position item.
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
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The next position item is returned.
	EXPECT_TRUE(found);
	EXPECT_EQ(next_index, 2);
}

// WHY: Geometry-only position traversal must skip non-position mission items.
// WHAT: Starting from a position item after a VTOL transition, the helper skips it and returns the previous position item.
TEST_F(MissionBaseTraversalTest, FindPreviousSkipsNonPositionItems)
{
	// GIVEN: A position item, a non-position VTOL transition, and then another position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});

	int32_t previous_index{-1};

	// WHEN: Geometry-only traversal searches backward from the non-position item.
	const bool found = mission_base.findPreviousPositionIndex(2, previous_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The previous position item is returned.
	EXPECT_TRUE(found);
	EXPECT_EQ(previous_index, 0);
}

// WHY: Geometry-only traversal must skip DO_JUMP items.
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
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The first position item after the jump is returned.
	EXPECT_TRUE(found);
	EXPECT_EQ(next_index, 2);
}

// WHY: Geometry-only traversal must skip DO_JUMP items.
// WHAT: [WP, WP, DO_JUMP, WP] starting from idx 3 returns idx 1.
TEST_F(MissionBaseTraversalTest, FindPreviousSkipsDoJumpItems)
{
	// GIVEN: A mission where a DO_JUMP sits between two position items.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 3), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});

	int32_t previous_index{-1};

	// WHEN: Geometry-only traversal starts at the DO_JUMP item.
	const bool found = mission_base.findPreviousPositionIndex(3, previous_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The first position item before the jump is returned.
	EXPECT_TRUE(found);
	EXPECT_EQ(previous_index, 1);
}

// WHY: Consecutive non-position items must be skipped.
// WHAT: [WP, DO_JUMP, VTOL_FW, WP] starting from idx 1 returns idx 3.
TEST_F(MissionBaseTraversalTest, FindNextSkipsConsecutiveNonPositionItems)
{
	// GIVEN: Consecutive non-position items before the next position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 5), // idx 1
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW), // idx 2
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 3
	});

	int32_t next_index{-1};

	// WHEN: Geometry-only traversal walks forward through the control items.
	const bool found = mission_base.findNextPositionIndex(1, next_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: It returns the first following position item.
	EXPECT_TRUE(found);
	EXPECT_EQ(next_index, 3);
}

// WHY: Consecutive non-position items must be skipped in reverse.
// WHAT: [WP, DO_JUMP, VTOL_FW, WP] starting from idx 3 returns idx 0.
TEST_F(MissionBaseTraversalTest, FindPreviousSkipsConsecutiveNonPositionItems)
{
	// GIVEN: Consecutive non-position items before the previous position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 3), // idx 1
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW), // idx 2
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 3
	});

	int32_t previous_index{-1};

	// WHEN: Geometry-only traversal walks backward through the control items.
	const bool found = mission_base.findPreviousPositionIndex(3, previous_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: It returns the first previous position item.
	EXPECT_TRUE(found);
	EXPECT_EQ(previous_index, 0);
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
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The helper reports that no next position item exists.
	EXPECT_FALSE(found);
	EXPECT_EQ(next_index, -1);
}

// WHY: Callers need a failure when no earlier position item exists.
// WHAT: [DO_JUMP, WP] starting from idx 1 returns false.
TEST_F(MissionBaseTraversalTest, FindPreviousReturnsFalseAtStart)
{
	// GIVEN: A mission with no position item before the starting index.
	mission_base.loadTestMission({
		makeDoJump(0, 3), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
	});

	int32_t previous_index{-1};

	// WHEN: Geometry-only traversal searches before the first position item.
	const bool found = mission_base.findPreviousPositionIndex(1, previous_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The helper reports that no previous position item exists.
	EXPECT_FALSE(found);
	EXPECT_EQ(previous_index, -1);
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
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The unreadable position item produces a clean failure.
	EXPECT_FALSE(found);
	EXPECT_EQ(next_index, -1);
}

// WHY: Traversal should fail cleanly when a mission item cannot be loaded.
// WHAT: A cache failure on the previous position item makes findPreviousPositionIndex() return false.
TEST_F(MissionBaseTraversalTest, FindPreviousReturnsFalseOnCacheReadFailure)
{
	// GIVEN: A mission where the previous position item cannot be loaded.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makeDoJump(0, 3), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});
	mission_base.setLoadFailureIndices({0});

	int32_t previous_index{-1};

	// WHEN: Geometry-only traversal moves backward past the DO_JUMP item.
	const bool found = mission_base.findPreviousPositionIndex(2, previous_index,
			   MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: The unreadable position item produces a clean failure.
	EXPECT_FALSE(found);
	EXPECT_EQ(previous_index, -1);
}

// WHY: findNextPositionIndex must use MissionTraversalType
// WHAT: [DO_JUMP->2, WP1, WP2] starting from idx 0 resolves to idx 2 in mission-control
//       mode and idx 1 in geometry-only mode.
TEST_F(MissionBaseTraversalTest, FindNextSupportsBothTraversalSemantics)
{
	// GIVEN: A jump whose target is a later position item.
	mission_base.loadTestMission({
		makeDoJump(2, 1, 0), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});

	int32_t next_follow{-1};
	int32_t next_geometry{-1};

	// WHEN: The same lookup is performed in both traversal modes.
	const bool found_follow = mission_base.findNextPositionIndex(0, next_follow,
				  MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow);
	const bool found_geometry = mission_base.findNextPositionIndex(0, next_geometry,
				    MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: Mission-control mode follows the jump, while geometry-only mode skips it.
	EXPECT_TRUE(found_follow);
	EXPECT_TRUE(found_geometry);
	EXPECT_EQ(next_follow, 2);
	EXPECT_EQ(next_geometry, 1);
}

// WHY: findPreviousPositionIndex must use MissionTraversalType
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] starting from idx 3 resolves to idx 0 in mission-control
//       mode and idx 1 in geometry-only mode.
TEST_F(MissionBaseTraversalTest, FindPreviousSupportsBothTraversalSemantics)
{
	// GIVEN: A jump whose target is an earlier position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 2, 0), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});

	int32_t previous_follow{-1};
	int32_t previous_geometry{-1};

	// WHEN: The same lookup is performed in both traversal modes.
	const bool found_follow = mission_base.findPreviousPositionIndex(3, previous_follow,
				  MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow);
	const bool found_geometry = mission_base.findPreviousPositionIndex(3, previous_geometry,
				    MissionBaseTestPeer::MissionTraversalType::IgnoreDoJump);

	// THEN: Mission-control mode follows the jump, while geometry-only mode skips it.
	EXPECT_TRUE(found_follow);
	EXPECT_TRUE(found_geometry);
	EXPECT_EQ(previous_follow, 0);
	EXPECT_EQ(previous_geometry, 1);
}

// WHY: The refactor must not change the legacy mission-control behavior.
// WHAT: [DO_JUMP->2, WP1, WP2] from current_seq=-1 should still land on idx 2.
TEST_F(MissionBaseTraversalTest, GoToNextPositionItemFollowsMissionControlFlow)
{
	// GIVEN: A mission whose first item is an active DO_JUMP.
	mission_base.loadTestMission({
		makeDoJump(2, 1, 0), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});
	mission_base.setCurrentSequence(-1);

	// WHEN: The caller requests mission-control traversal.
	const int ret = mission_base.goToNextPositionItem(MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow);

	// THEN: Traversal follows the jump target exactly as before.
	EXPECT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_base.currentSequence(), 2);
}

// WHY: The backward wrapper must also preserve the legacy mission-control behavior.
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] from current_seq=3 should still land on idx 0.
TEST_F(MissionBaseTraversalTest, GoToPreviousPositionItemFollowsMissionControlFlow)
{
	// GIVEN: A mission with an active jump loop before the current position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 2, 0), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});
	mission_base.setCurrentSequence(3);

	// WHEN: The caller requests mission-control traversal.
	const int ret = mission_base.goToPreviousPositionItem(MissionBaseTestPeer::MissionTraversalType::FollowMissionControlFlow);

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

	// WHEN: The multi-item helper walks forward with default traversal semantics.
	mission_base.getNextPositionItems(2, next_items, num_found_items, 2u);

	// THEN: The active DO_JUMP is followed.
	ASSERT_EQ(num_found_items, 2u);
	EXPECT_EQ(next_items[0], 0);
	EXPECT_EQ(next_items[1], 1);
}

// WHY: Reverse mission flows rely on getPreviousPositionItems() following active DO_JUMP.
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] starting from idx 3 returns idx 0.
TEST_F(MissionBaseTraversalTest, GetPreviousPositionItemsFollowsActiveDoJump)
{
	// GIVEN: A mission with an active jump loop before the current position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 2, 0), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});

	int32_t previous_items[1] = {-1};
	size_t num_found_items = 0;

	// WHEN: The multi-item helper walks backward with default traversal semantics.
	mission_base.getPreviousPositionItems(3, previous_items, num_found_items, 1u);

	// THEN: The active DO_JUMP is followed.
	ASSERT_EQ(num_found_items, 1u);
	EXPECT_EQ(previous_items[0], 0);
}

// WHY: Mission-based RTL configures position traversal to skip DO_JUMP loops consistently.
// WHAT: [DO_JUMP->2, WP1, WP2] from current_seq=-1 lands on idx 1 with the configured traversal.
TEST_F(IgnoreDoJumpMissionBaseTraversalTest, ConfiguredTraversalSkipsDoJumpForGoToNextPositionItem)
{
	// GIVEN: A mission whose first item is an active DO_JUMP.
	mission_base.loadTestMission({
		makeDoJump(2, 1, 0), // idx 0
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 1
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 2
	});
	mission_base.setCurrentSequence(-1);

	// WHEN: The mode advances using its configured traversal policy.
	const int ret = mission_base.goToNextPositionItem();

	// THEN: The DO_JUMP loop is skipped and the geometric next waypoint is selected.
	EXPECT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_base.currentSequence(), 1);
}

// WHY: Reverse mission-path RTL must skip DO_JUMP loops for backward progression too.
// WHAT: [WP0, WP1, DO_JUMP->0, WP3] from current_seq=3 lands on idx 1 with the configured traversal.
TEST_F(IgnoreDoJumpMissionBaseTraversalTest, ConfiguredTraversalSkipsDoJumpForGoToPreviousPositionItem)
{
	// GIVEN: A mission with an active jump loop before the current position item.
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt), // idx 0
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt), // idx 1
		makeDoJump(0, 2, 0), // idx 2
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt), // idx 3
	});
	mission_base.setCurrentSequence(3);

	// WHEN: The mode advances backward using its configured traversal policy.
	const int ret = mission_base.goToPreviousPositionItem();

	// THEN: The DO_JUMP loop is skipped and the geometric previous waypoint is selected.
	EXPECT_EQ(ret, PX4_OK);
	EXPECT_EQ(mission_base.currentSequence(), 1);
}

class MissionBaseHandleLandingTestPeer : public MissionBaseTestPeer
{
public:
	explicit MissionBaseHandleLandingTestPeer(Navigator *navigator) : MissionBaseTestPeer(navigator) {}

	using MissionBase::handleLanding;
	using MissionBase::WorkItemType;
	using MissionBase::_mission_item;
	using MissionBase::_work_item_type;

	void updateVehicleState()
	{
		_vehicle_status_sub.update();
		_land_detected_sub.update();
	}
};

class MissionBaseHandleLandingTest : public NavigatorDatamanTestBase
{
protected:
	void SetUp() override
	{
		publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
		publishLandDetected(false);
	}

	void TearDown() override
	{
		if (_vehicle_status_pub != nullptr) {
			orb_unadvertise(_vehicle_status_pub);
			_vehicle_status_pub = nullptr;
		}

		if (_land_detected_pub != nullptr) {
			orb_unadvertise(_land_detected_pub);
			_land_detected_pub = nullptr;
		}
	}

	void publishVehicleStatus(bool is_vtol, uint8_t vehicle_type)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.is_vtol = is_vtol;
		status.vehicle_type = vehicle_type;

		if (_vehicle_status_pub == nullptr) {
			_vehicle_status_pub = orb_advertise(ORB_ID(vehicle_status), &status);

		} else {
			orb_publish(ORB_ID(vehicle_status), _vehicle_status_pub, &status);
		}
	}

	void publishLandDetected(bool landed)
	{
		vehicle_land_detected_s land_detected{};
		land_detected.timestamp = hrt_absolute_time();
		land_detected.landed = landed;

		if (_land_detected_pub == nullptr) {
			_land_detected_pub = orb_advertise(ORB_ID(vehicle_land_detected), &land_detected);

		} else {
			orb_publish(ORB_ID(vehicle_land_detected), _land_detected_pub, &land_detected);
		}
	}

	Navigator _navigator{};
	MissionBaseHandleLandingTestPeer mission_base{&_navigator};

	orb_advert_t _vehicle_status_pub{nullptr};
	orb_advert_t _land_detected_pub{nullptr};
};

// WHY: When RTL needs to climb back up to RTL_RETURN_ALT before flying to the landing point,
// the mission item preceding the land item is a WORK_ITEM_TYPE_CLIMB item rather than the
// default one. Before this fix, handleLanding() only started the VTOL landing sequence when
// coming from WORK_ITEM_TYPE_DEFAULT, so a landing reached after that climb never got a back
// transition and was flown fixed wing.
// WHAT: handleLanding() also starts the VTOL landing sequence when the previous work item was
// WORK_ITEM_TYPE_CLIMB.
TEST_F(MissionBaseHandleLandingTest, LandPointAfterReturnAltitudeClimbTriggersMoveToLand)
{
	// GIVEN: a fixed-wing VTOL reaching the VTOL land item right after climbing back to
	// RTL_RETURN_ALT.
	mission_base._mission_item.nav_cmd = NAV_CMD_VTOL_LAND;
	mission_base._work_item_type = MissionBaseHandleLandingTestPeer::WorkItemType::WORK_ITEM_TYPE_CLIMB;
	mission_base.updateVehicleState();

	auto new_work_item_type = MissionBaseHandleLandingTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT;
	mission_item_s next_mission_items[1] {};
	size_t num_found_items = 0;

	// WHEN: handleLanding() processes the item.
	mission_base.handleLanding(new_work_item_type, next_mission_items, num_found_items);

	// THEN: the vehicle still starts the move-to-land sequence instead of skipping the back
	// transition because the previous work item was a climb rather than the default one.
	EXPECT_EQ(new_work_item_type, MissionBaseHandleLandingTestPeer::WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND);
}

class MissionTestPeer : public Mission
{
public:
	explicit MissionTestPeer(Navigator *navigator) : Mission(navigator) {}

	using MissionBase::_mission_item;
	using MissionBase::_work_item_type;
	using MissionBase::WorkItemType;

	void loadTestMission(const std::vector<mission_item_s> &items)
	{
		_mission_store.setItems(items);
	}

	// Bypasses Mission::on_activation()'s forced MissionFeasibilityChecker run -- that check
	// needs a much larger harness (home position, geofence, takeoff/landing requirements) to
	// satisfy and is unrelated to what this test is exercising. MissionBase::on_activation() is
	// the real, shared activation path (checkClimbRequired() -> set_mission_items() ->
	// setActiveMissionItems()) that both Mission and the RTL mission-landing modes go through.
	void activateForTest()
	{
		MissionBase::on_activation();
	}

	uint16_t activeNavCommand() const { return _mission_item.nav_cmd; }
	bool activeVtolBackTransition() const { return _mission_item.vtol_back_transition; }

protected:
	bool loadMissionItemFromCache(int32_t index, mission_item_s &mission_item) override
	{
		return _mission_store.loadItem(index, mission_item);
	}

private:
	navigator_test::VectorMissionItemStore _mission_store{};
};

class MissionHandleTakeoffClimbTest : public NavigatorDatamanTestBase
{
protected:
	void TearDown() override
	{
		if (_mission_pub != nullptr) {
			orb_unadvertise(_mission_pub);
			_mission_pub = nullptr;
		}

		if (_vehicle_status_pub != nullptr) {
			orb_unadvertise(_vehicle_status_pub);
			_vehicle_status_pub = nullptr;
		}

		if (_global_position_pub != nullptr) {
			orb_unadvertise(_global_position_pub);
			_global_position_pub = nullptr;
		}

		if (_land_detected_pub != nullptr) {
			orb_unadvertise(_land_detected_pub);
			_land_detected_pub = nullptr;
		}
	}

	void publishMission(const mission_s &mission)
	{
		if (_mission_pub == nullptr) {
			_mission_pub = orb_advertise(ORB_ID(mission), &mission);

		} else {
			orb_publish(ORB_ID(mission), _mission_pub, &mission);
		}
	}

	void publishVehicleStatus(bool is_vtol, uint8_t vehicle_type)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.is_vtol = is_vtol;
		status.vehicle_type = vehicle_type;

		if (_vehicle_status_pub == nullptr) {
			_vehicle_status_pub = orb_advertise(ORB_ID(vehicle_status), &status);

		} else {
			orb_publish(ORB_ID(vehicle_status), _vehicle_status_pub, &status);
		}
	}

	void publishGlobalPosition(double lat, double lon, float altitude)
	{
		vehicle_global_position_s global_position{};
		global_position.timestamp = hrt_absolute_time();
		global_position.lat = lat;
		global_position.lon = lon;
		global_position.alt = altitude;

		if (_global_position_pub == nullptr) {
			_global_position_pub = orb_advertise(ORB_ID(vehicle_global_position), &global_position);

		} else {
			orb_publish(ORB_ID(vehicle_global_position), _global_position_pub, &global_position);
		}
	}

	void publishLandDetected(bool landed)
	{
		vehicle_land_detected_s land_detected{};
		land_detected.timestamp = hrt_absolute_time();
		land_detected.landed = landed;

		if (_land_detected_pub == nullptr) {
			_land_detected_pub = orb_advertise(ORB_ID(vehicle_land_detected), &land_detected);

		} else {
			orb_publish(ORB_ID(vehicle_land_detected), _land_detected_pub, &land_detected);
		}
	}

	static constexpr double kLat = 47.397742;
	static constexpr double kLon = 8.545594;
	static constexpr float kLandAlt = 500.f;

	Navigator _navigator{};
	MissionTestPeer mission_peer{&_navigator};

	orb_advert_t _mission_pub{nullptr};
	orb_advert_t _vehicle_status_pub{nullptr};
	orb_advert_t _global_position_pub{nullptr};
	orb_advert_t _land_detected_pub{nullptr};
};

// WHY: Resuming a mission in the air (e.g. a stale mission whose remaining item is the landing
// point) can leave the vehicle below the altitude that item was planned at. Mission::handleTakeoff()
// inserts a WORK_ITEM_TYPE_CLIMB waypoint to make up the difference before mission.cpp:324
// re-evaluates the actual mission item. On main, a fixed-wing VTOL that reaches a VTOL_LAND item
// straight out of that climb never transitions back to MC and flies the
// fixed-wing-incompatible VTOL_LAND descent as a fixed wing -- this is the path the SDT-325 crash
// investigation traced the real incident to.
// WHAT: Driving Mission through a real activation (climb required, climb reached, mission item
// re-evaluated) with a fixed-wing VTOL and a single VTOL_LAND mission item routes through
// WORK_ITEM_TYPE_MOVE_TO_LAND (NAV_CMD_WAYPOINT with vtol_back_transition set) instead of flying
// the land item unmodified.
TEST_F(MissionHandleTakeoffClimbTest, LandItemAboveVehicleBackTransitionsAfterClimb)
{
	mission_item_s land{};
	land.nav_cmd = NAV_CMD_VTOL_LAND;
	land.lat = kLat;
	land.lon = kLon;
	land.altitude = kLandAlt;
	land.altitude_is_relative = false;
	land.autocontinue = true;

	mission_s test_mission{};
	test_mission.timestamp = hrt_absolute_time();
	test_mission.mission_id = 1;
	test_mission.count = 1;
	test_mission.current_seq = 0;
	test_mission.land_start_index = -1;
	test_mission.land_index = -1;
	test_mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;

	// Well below the land item's altitude: enough to require a climb (more than the altitude
	// acceptance radius below target), unlike the real flight the vehicle hasn't moved yet.
	// Navigator keeps its own copies of vehicle_status/global_position/land_detected (separate
	// from MissionBase's own subscriptions), so both need to be kept in sync: MissionBase's
	// handleTakeoff()/handleLanding() read the uORB-published copies, while the generic
	// is_mission_item_reached_or_completed() distance/altitude checks in mission_block.cpp read
	// straight off the Navigator object.
	const float kInitialAlt = kLandAlt - 50.f;
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishGlobalPosition(kLat, kLon, kInitialAlt);
	publishLandDetected(false);
	publishMission(test_mission);
	_navigator.get_mission_result()->valid = true;

	_navigator.get_vstatus()->is_vtol = true;
	_navigator.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	_navigator.get_land_detected()->landed = false;
	_navigator.get_global_position()->lat = kLat;
	_navigator.get_global_position()->lon = kLon;
	_navigator.get_global_position()->alt = kInitialAlt;

	mission_peer.loadTestMission({land});

	// GIVEN: the mission is resumed in the air, directly on the land item.
	mission_peer.on_inactive();
	mission_peer.activateForTest();

	ASSERT_EQ(mission_peer.activeNavCommand(), NAV_CMD_LOITER_TO_ALT);
	ASSERT_EQ(mission_peer._work_item_type, MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_CLIMB);

	// First on_active(): the climb's position setpoint altitude (established at the vehicle's
	// current altitude per the NAV_CMD_LOITER_TO_ALT convention) matches the vehicle's actual
	// altitude, so it snaps the setpoint to the real target altitude without yet reporting the
	// item as reached.
	mission_peer.on_active();
	ASSERT_EQ(mission_peer.activeNavCommand(), NAV_CMD_LOITER_TO_ALT);

	// WHEN: the climb completes (the vehicle reaches the land item's altitude) and the mission
	// item is re-evaluated.
	publishGlobalPosition(kLat, kLon, kLandAlt);
	_navigator.get_global_position()->alt = kLandAlt;
	mission_peer.on_active();

	// THEN: the vehicle is routed to move to the land point as fixed wing ahead of the back
	// transition, instead of flying the raw VTOL_LAND item as fixed wing.
	EXPECT_EQ(mission_peer.activeNavCommand(), NAV_CMD_WAYPOINT);
	EXPECT_TRUE(mission_peer.activeVtolBackTransition());
}
