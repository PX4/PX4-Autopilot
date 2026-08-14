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

#include "mission_base.h"
#include "navigator.h"
#include "support/mission_route_cache_test_peer.h"
#include "support/mission_route_test_helpers.h"
#include "support/navigator_dataman_test.h"
#include "support/vector_mission_item_store.h"

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include "mission.h"
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE

#include <initializer_list>
#include <vector>

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

	void setMissionRestartState(bool activated, bool disarmed_while_inactive, int32_t inactivation_index)
	{
		_mission_has_been_activated = activated;
		_system_disarmed_while_inactive = disarmed_while_inactive;
		_inactivation_index = inactivation_index;
	}

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	void setVehicleStatus(bool is_vtol, bool fixed_wing, bool in_transition_to_fw = false)
	{
		vehicle_status_s status{};
		status.is_vtol = is_vtol;
		status.vehicle_type = fixed_wing ? vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				      : vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status.in_transition_mode = in_transition_to_fw;
		status.in_transition_to_fw = in_transition_to_fw;
		status.timestamp = hrt_absolute_time();
		_vehicle_status_pub.publish(status);
		_vehicle_status_sub.update();
	}

	uint8_t vtolStateAt(int32_t anchor_index)
	{
		return getVtolStateAtMissionIndex(anchor_index);
	}

	void captureCurrentVtolState()
	{
		captureMissionStartVtolState();
	}

	VtolTransitionAction transitionForTarget(int32_t target_index, bool direction_reversed)
	{
		return vtolTransitionActionForTarget(target_index, direction_reversed);
	}

	VtolTransitionAction transitionAfterReverseTarget(int32_t target_index)
	{
		return vtolTransitionActionAfterReachingReverseTarget(target_index);
	}

	mission_route::Segment loopSegmentForNextNominalAdvance()
	{
		mission_route::Segment segment{};
		updateLastFlownLoopSegmentForNominalAdvance(segment);
		return segment;
	}

	void updateLoopSegmentForNextNominalAdvance(mission_route::Segment &segment)
	{
		updateLastFlownLoopSegmentForNominalAdvance(segment);
	}

	void setupJoinRouteForTest(mission_route::JoinContext &join_context,
				   const mission_route::RoutePath &path)
	{
		setupJoinRoute(join_context, path);
	}

	bool runJoinWorkItem()
	{
		position_setpoint_triplet_s *triplet = _navigator->get_position_setpoint_triplet();
		const position_setpoint_s current_setpoint_copy = triplet->current;
		return handleJoinRouteWorkItems(triplet, current_setpoint_copy);
	}

	void setJoinWaypointReached(bool reached)
	{
		_waypoint_position_reached = reached;
		_waypoint_yaw_reached = reached;
	}

	void processMissionSourceChange()
	{
		onMissionUpdate(true);
	}

	bool joinWorkItemActive() const
	{
		return _work_item_type == WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE;
	}

	bool transitionAfterJoinActive() const
	{
		return _work_item_type == WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN;
	}

	const mission_route::JoinContext &joinContext() const
	{
		return _route_join_context;
	}

	const mission_item_s &currentMissionItem() const
	{
		return _mission_item;
	}
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE

	using MissionBase::findNextPositionIndex;
	using MissionBase::findPreviousPositionIndex;
	using MissionBase::getIncomingMissionCurrentSeq;
	using MissionBase::getNonJumpItem;
	using MissionBase::getNextPositionItems;
	using MissionBase::getPreviousPositionItems;
	using MissionBase::goToNextPositionItem;
	using MissionBase::goToPreviousPositionItem;
	using MissionBase::MissionTraversalType;
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	using MissionBase::VtolTransitionAction;
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
	using MissionBase::resetMissionJumpCounter;

private:
	navigator_test::VectorMissionItemStore _mission_store{};
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	uORB::Publication<vehicle_status_s> _vehicle_status_pub {ORB_ID(vehicle_status)};
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
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


class MissionBaseTraversalTest : public NavigatorDatamanTestBase
{
protected:
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	void SetUp() override
	{
		mission_base.setVehicleStatus(false, false);
		mission_base.captureCurrentVtolState();
	}
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE

	MissionBaseTestPeer mission_base{};
};

class IgnoreDoJumpMissionBaseTraversalTest : public NavigatorDatamanTestBase
{
protected:
	IgnoreDoJumpMissionBaseTestPeer mission_base{};
};

TEST(MissionBaseMissionSourceTest, NewMissionWithoutCurrentSequenceStartsAtBeginning)
{
	mission_s current{};
	current.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	current.mission_id = 10;
	current.count = 8;
	current.current_seq = 6;

	mission_s replacement = current;
	replacement.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;
	replacement.mission_id = 11;
	replacement.count = 3;
	replacement.current_seq = -1;

	EXPECT_EQ(MissionBaseTestPeer::getIncomingMissionCurrentSeq(replacement, current), 0);
}

TEST(MissionBaseMissionSourceTest, SameMissionWithoutCurrentSequenceKeepsProgress)
{
	mission_s current{};
	current.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	current.mission_id = 10;
	current.count = 8;
	current.current_seq = 6;

	mission_s update = current;
	update.current_seq = -1;

	EXPECT_EQ(MissionBaseTestPeer::getIncomingMissionCurrentSeq(update, current), 6);
}

TEST_F(MissionBaseTraversalTest, FinishedMissionRestartsEvenWhenSequenceIsNotAtEnd)
{
	Navigator navigator{};
	MissionBaseTestPeer mission_base_with_nav{&navigator};
	mission_base_with_nav.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	mission_base_with_nav.setCurrentSequence(1);
	mission_base_with_nav.setMissionRestartState(true, true, 1);
	navigator.get_mission_result()->valid = true;
	navigator.get_mission_result()->finished = true;

	mission_base_with_nav.on_activation();

	EXPECT_EQ(mission_base_with_nav.currentSequence(), 0);
	EXPECT_FALSE(navigator.get_mission_result()->finished);
}

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

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
TEST_F(MissionBaseTraversalTest, ConstructorCapturesCurrentVtolStateForPreloadedMission)
{
	uORB::Publication<vehicle_status_s> vehicle_status_pub{ORB_ID(vehicle_status)};
	vehicle_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.is_vtol = true;
	status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	vehicle_status_pub.publish(status);

	MissionBaseTestPeer mission_base_after_status{};
	mission_base_after_status.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)});

	EXPECT_EQ(mission_base_after_status.vtolStateAt(0), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);

	status.timestamp = hrt_absolute_time();
	status.is_vtol = false;
	status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	vehicle_status_pub.publish(status);
}

TEST_F(MissionBaseTraversalTest, VtolTakeoffDefinesFollowingSegmentsAsFixedWing)
{
	mission_item_s vtol_takeoff = makePositionItem(kBaseLat, kBaseLon, kAlt);
	vtol_takeoff.nav_cmd = NAV_CMD_VTOL_TAKEOFF;
	mission_base.loadTestMission({
		vtol_takeoff,
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt + 20.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});

	EXPECT_EQ(mission_base.vtolStateAt(0), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission_base.vtolStateAt(1), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission_base.vtolStateAt(3), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);

	mission_base.setVehicleStatus(true, false);
	EXPECT_EQ(mission_base.transitionForTarget(1, false),
		  MissionBaseTestPeer::VtolTransitionAction::kFrontTransition);
	EXPECT_EQ(mission_base.transitionForTarget(0, true),
		  MissionBaseTestPeer::VtolTransitionAction::kFrontTransition);
}

TEST_F(MissionBaseTraversalTest, VtolStateAndActionsFollowMissionSegments)
{
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});

	EXPECT_EQ(mission_base.vtolStateAt(0), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	EXPECT_EQ(mission_base.vtolStateAt(2), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission_base.vtolStateAt(4), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);

	mission_base.setVehicleStatus(true, false);
	EXPECT_EQ(mission_base.transitionForTarget(2, false),
		  MissionBaseTestPeer::VtolTransitionAction::kFrontTransition);

	EXPECT_EQ(mission_base.transitionForTarget(0, true),
		  MissionBaseTestPeer::VtolTransitionAction::kFrontTransition);

	mission_base.setVehicleStatus(true, true);
	EXPECT_EQ(mission_base.transitionForTarget(4, false),
		  MissionBaseTestPeer::VtolTransitionAction::kBackTransition);
	EXPECT_EQ(mission_base.transitionForTarget(2, true),
		  MissionBaseTestPeer::VtolTransitionAction::kBackTransition);

	mission_base.setVehicleStatus(true, false);
	EXPECT_EQ(mission_base.transitionAfterReverseTarget(2),
		  MissionBaseTestPeer::VtolTransitionAction::kFrontTransition);

	mission_base.setVehicleStatus(false, false);
	EXPECT_EQ(mission_base.transitionForTarget(2, false),
		  MissionBaseTestPeer::VtolTransitionAction::kNone);
}

TEST_F(MissionBaseTraversalTest, JoinRouteRunsWaypointTransitionAndResumeFlow)
{
	Navigator navigator{};
	MissionBaseTestPeer mission_base_with_nav{&navigator};
	const std::vector<mission_item_s> items{
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt + 20.f),
	};
	mission_base_with_nav.loadTestMission(items);
	mission_base_with_nav.setCurrentSequence(2);
	mission_base_with_nav.setVehicleStatus(true, false);

	vehicle_global_position_s global_position{};
	global_position.lat = kBaseLat + 0.0004;
	global_position.lon = kBaseLon;
	global_position.alt = kAlt + 5.f;
	*navigator.get_global_position() = global_position;

	mission_route::JoinContext join_context{};
	join_context.projection = {kBaseLat + 0.0005, kBaseLon, kAlt + 10.f};
	join_context.skip_altitude_requirement = true;
	mission_route::RoutePath path{};
	path.first_item_index = 2;
	path.direction_reversed = false;

	mission_base_with_nav.setupJoinRouteForTest(join_context, path);
	ASSERT_TRUE(mission_base_with_nav.joinWorkItemActive());
	EXPECT_EQ(join_context.transition_action, MissionBaseTestPeer::VtolTransitionAction::kFrontTransition);

	ASSERT_TRUE(mission_base_with_nav.runJoinWorkItem());
	const position_setpoint_s &join_setpoint = navigator.get_position_setpoint_triplet()->current;
	ASSERT_TRUE(join_setpoint.valid);
	EXPECT_DOUBLE_EQ(join_setpoint.lat, join_context.projection.lat);
	EXPECT_DOUBLE_EQ(join_setpoint.lon, join_context.projection.lon);
	EXPECT_FLOAT_EQ(join_setpoint.alt, global_position.alt);

	mission_base_with_nav.setJoinWaypointReached(true);
	ASSERT_TRUE(mission_base_with_nav.runJoinWorkItem());
	ASSERT_TRUE(mission_base_with_nav.transitionAfterJoinActive());
	EXPECT_EQ(mission_base_with_nav.currentMissionItem().nav_cmd, NAV_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(mission_base_with_nav.currentMissionItem().params[0],
			static_cast<float>(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW));
	const float expected_yaw = get_bearing_to_next_waypoint(global_position.lat, global_position.lon,
				   items[2].lat, items[2].lon);
	EXPECT_NEAR(mission_base_with_nav.currentMissionItem().yaw, expected_yaw, 1e-4f);

	mission_base_with_nav.setVehicleStatus(true, true);
	EXPECT_FALSE(mission_base_with_nav.runJoinWorkItem());
	EXPECT_FALSE(mission_base_with_nav.joinWorkItemActive());
	EXPECT_FALSE(mission_base_with_nav.transitionAfterJoinActive());
	EXPECT_FALSE(mission_base_with_nav.joinContext().valid());
}

TEST_F(MissionBaseTraversalTest, JoinRouteResumesDirectlyWhenNoTransitionIsRequired)
{
	Navigator navigator{};
	MissionBaseTestPeer mission_base_with_nav{&navigator};
	mission_base_with_nav.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)});
	mission_base_with_nav.setVehicleStatus(false, false);

	mission_route::JoinContext join_context{};
	join_context.projection = {kBaseLat, kBaseLon, kAlt};
	mission_route::RoutePath path{};
	path.first_item_index = 0;
	mission_base_with_nav.setupJoinRouteForTest(join_context, path);

	ASSERT_TRUE(mission_base_with_nav.runJoinWorkItem());
	mission_base_with_nav.setJoinWaypointReached(true);
	EXPECT_FALSE(mission_base_with_nav.runJoinWorkItem());
	EXPECT_FALSE(mission_base_with_nav.joinWorkItemActive());
	EXPECT_FALSE(mission_base_with_nav.joinContext().valid());
}

TEST_F(MissionBaseTraversalTest, MissionSourceChangeClearsJoinAndCapturesVtolState)
{
	Navigator navigator{};
	MissionBaseTestPeer mission_base_with_nav{&navigator};
	mission_base_with_nav.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)});
	mission_base_with_nav.setVehicleStatus(true, false);

	mission_route::JoinContext join_context{};
	join_context.projection = {kBaseLat, kBaseLon, kAlt};
	mission_route::RoutePath path{};
	path.first_item_index = 0;
	mission_base_with_nav.setupJoinRouteForTest(join_context, path);
	ASSERT_TRUE(mission_base_with_nav.joinWorkItemActive());

	mission_base_with_nav.setVehicleStatus(true, true);
	mission_base_with_nav.processMissionSourceChange();

	EXPECT_FALSE(mission_base_with_nav.joinWorkItemActive());
	EXPECT_FALSE(mission_base_with_nav.joinContext().valid());
	EXPECT_EQ(mission_base_with_nav.vtolStateAt(0), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
}

TEST_F(MissionBaseTraversalTest, NextNominalAdvanceCapturesActiveLoopEdge)
{
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makeDoJump(0, 3, 1),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	mission_base.setCurrentSequence(1);

	const mission_route::Segment segment = mission_base.loopSegmentForNextNominalAdvance();
	ASSERT_TRUE(segment.validLoop());
	EXPECT_EQ(segment.start.idx, 1);
	EXPECT_EQ(segment.end.idx, 0);
	EXPECT_EQ(segment.loops_remaining, 2);
}

TEST_F(MissionBaseTraversalTest, InvalidLoopTargetClearsPreviousLoopSegment)
{
	mission_base.loadTestMission({
		makePositionItem(kBaseLat, kBaseLon, kAlt),
		makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
		makeDoJump(10, 3, 1),
		makePositionItem(kBaseLat + 0.002, kBaseLon, kAlt),
	});
	mission_base.setCurrentSequence(1);

	mission_route::Segment segment{};
	segment.start.idx = 1;
	segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	segment.end.idx = 0;
	segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	segment.is_loop = true;
	segment.loops_remaining = 2;
	ASSERT_TRUE(segment.validLoop());

	mission_base.updateLoopSegmentForNextNominalAdvance(segment);

	EXPECT_FALSE(segment.valid());
	EXPECT_FALSE(segment.is_loop);
	EXPECT_EQ(segment.loops_remaining, 0);
}

/**
 * @brief Mission peer that exposes smart-rejoin state for navigator-level tests.
 */
class MissionTestPeer : public Mission
{
public:
	explicit MissionTestPeer(Navigator *navigator) : Mission(navigator) {}

	using Mission::trySetRouteJoinOnActivation;
	using MissionBase::VtolTransitionAction;
	using MissionBase::WorkItemType;

	WorkItemType workItemTypeForTest() const { return _work_item_type; }
	int32_t currentSequenceForTest() const { return _mission.current_seq; }
	const mission_route::JoinContext &joinContextForTest() const { return _route_join_context; }
	VtolTransitionAction joinTransitionActionForTest() const { return _route_join_context.transition_action; }
};

/**
 * @brief End-to-end fixture driving the real Mission class with dataman-backed mission data.
 */
class MissionRouteJoinTest : public NavigatorDatamanTestBase
{
protected:
	void SetUp() override
	{
		const hrt_abstime now = hrt_absolute_time();

		mission_s mission{};
		mission.timestamp = now;
		_mission_pub.publish(mission);

		vehicle_status_s vehicle_status{};
		vehicle_status.timestamp = now;
		vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
		vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_UNSPECIFIED;
		_vehicle_status_pub.publish(vehicle_status);
		_vehicle_status = vehicle_status;

		vehicle_land_detected_s land_detected{};
		land_detected.timestamp = now;
		land_detected.landed = true;
		_land_detected_pub.publish(land_detected);
		_land_detected = land_detected;

		vehicle_global_position_s global_position{};
		global_position.timestamp = now;
		global_position.lat = static_cast<double>(NAN);
		global_position.lon = static_cast<double>(NAN);
		global_position.alt = NAN;
		_vehicle_global_position_pub.publish(global_position);
		_global_position = global_position;

		vehicle_local_position_s local_position{};
		local_position.timestamp = now;
		local_position.heading = NAN;
		local_position.vx = NAN;
		local_position.vy = NAN;
		_vehicle_local_position_pub.publish(local_position);
		_local_position = local_position;

		home_position_s home_position{};
		home_position.timestamp = now;
		_home_position_pub.publish(home_position);
		_home_position = home_position;
	}

	static constexpr float kBaseAlt = kAlt;

	void setIntParam(const char *name, int32_t value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set(handle, &value), PX4_OK) << name;
	}

	void writeMissionItems(const std::vector<mission_item_s> &items, dm_item_t dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0)
	{
		for (size_t i = 0; i < items.size(); ++i) {
			mission_item_s item = items[i];
			ASSERT_TRUE(_dataman_client.writeSync(dataman_id, i,
							      reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		}
	}

	void writeSafePointState(uint16_t num_items, uint32_t opaque_id = 1, dm_item_t dataman_id = DM_KEY_SAFE_POINTS_0)
	{
		mission_stats_entry_s stats{};
		stats.num_items = num_items;
		stats.opaque_id = opaque_id;
		stats.dataman_id = static_cast<uint8_t>(dataman_id);
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_STATE, 0,
						      reinterpret_cast<uint8_t *>(&stats), sizeof(stats)));
	}

	void publishMission(const mission_s &mission)
	{
		_mission_pub.publish(mission);
	}

	void publishVehicleStatus(bool is_vtol, uint8_t vehicle_type, bool in_transition_to_fw = false)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.is_vtol = is_vtol;
		status.vehicle_type = vehicle_type;
		status.in_transition_mode = in_transition_to_fw;
		status.in_transition_to_fw = in_transition_to_fw;
		status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		_vehicle_status_pub.publish(status);
		_vehicle_status = status;
	}

	void publishLandDetected(bool landed)
	{
		vehicle_land_detected_s land_detected{};
		land_detected.timestamp = hrt_absolute_time();
		land_detected.landed = landed;
		_land_detected_pub.publish(land_detected);
		_land_detected = land_detected;
	}

	void publishGlobalPosition(const mission_route::Position &position)
	{
		vehicle_global_position_s global_position{};
		global_position.timestamp = hrt_absolute_time();
		global_position.lat = position.lat;
		global_position.lon = position.lon;
		global_position.alt = position.alt;
		_vehicle_global_position_pub.publish(global_position);
		_global_position = global_position;
	}

	void publishLocalPosition(float heading_rad = 0.f, float vx = 0.f, float vy = 0.f)
	{
		vehicle_local_position_s local_position{};
		local_position.timestamp = hrt_absolute_time();
		local_position.xy_valid = true;
		local_position.z_valid = true;
		local_position.heading = heading_rad;
		local_position.vx = vx;
		local_position.vy = vy;
		_vehicle_local_position_pub.publish(local_position);
		_local_position = local_position;
	}

	void publishHomePosition(const mission_route::Position &position)
	{
		home_position_s home_position{};
		home_position.timestamp = hrt_absolute_time();
		home_position.valid_hpos = true;
		home_position.valid_alt = true;
		home_position.lat = position.lat;
		home_position.lon = position.lon;
		home_position.alt = position.alt;
		_home_position_pub.publish(home_position);
		_home_position = home_position;
	}

	void primeNavigatorState()
	{
		*_navigator.get_vstatus() = _vehicle_status;
		*_navigator.get_land_detected() = _land_detected;
		*_navigator.get_global_position() = _global_position;
		*_navigator.get_local_position() = _local_position;
		*_navigator.get_home_position() = _home_position;
	}

	void markMissionResultValid()
	{
		*_navigator.get_mission_result() = mission_result_s{};
		_navigator.get_mission_result()->valid = true;
	}

	void updateRouteCacheUntilReady(const mission_s &mission)
	{
		MissionRouteCache &route_cache = _navigator.get_mission_route_cache();
		ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(route_cache, mission,
				[&] { return route_cache.missionItemsReady(mission) && route_cache.safePointsReady(); }))
				<< "MissionRouteCache did not become ready within the deterministic cache driver timeout";
	}

	Navigator _navigator{};
	DatamanClient _dataman_client{};
	uORB::Publication<mission_s> _mission_pub{ORB_ID(mission)};
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_global_position_s> _vehicle_global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_local_position_s> _vehicle_local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<home_position_s> _home_position_pub{ORB_ID(home_position)};

	vehicle_status_s _vehicle_status{};
	vehicle_land_detected_s _land_detected{};
	vehicle_global_position_s _global_position{};
	vehicle_local_position_s _local_position{};
	home_position_s _home_position{};
};

// Rejoin picks the closest loop exit of a DO_JUMP mission, not the first iteration.
TEST_F(MissionRouteJoinTest, MissionSmartRejoinUsesShortestLoopExit)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);

	// A 3-waypoint DO_JUMP loop plus a landing item, vehicle near WP2.
	std::vector<mission_item_s> mission_items = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kBaseAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kBaseAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kBaseAlt),
		makeDoJump(0, 2, 2),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f,   0.f, kBaseAlt - 10.f),
	};

	writeMissionItems(mission_items);
	writeSafePointState(0, 11);

	mission_s mission_state{};
	mission_state.timestamp = hrt_absolute_time();
	mission_state.current_seq = 0;
	mission_state.land_start_index = 4;
	mission_state.land_index = 4;
	mission_state.mission_id = 21;
	mission_state.safe_points_id = 11;
	mission_state.count = static_cast<uint16_t>(mission_items.size());
	mission_state.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	mission_state.fence_dataman_id = DM_KEY_FENCE_POINTS_0;
	mission_state.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;
	publishMission(mission_state);

	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishLandDetected(false);
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 95.f, kBaseAlt));
	publishLocalPosition(0.f, 12.f, 0.f);
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -200.f, 0.f, kBaseAlt));
	primeNavigatorState();

	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();

	ASSERT_TRUE(mission.trySetRouteJoinOnActivation(false));

	// Rejoin targets WP2 with a valid JOIN_ROUTE work item and no VTOL transition.
	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_TRUE(mission.joinContextForTest().valid());
	EXPECT_EQ(mission.joinTransitionActionForTest(), MissionTestPeer::VtolTransitionAction::kNone);
}

// Rejoining near the landing segment keeps the vehicle altitude instead of forcing a climb.
TEST_F(MissionRouteJoinTest, MissionSmartRejoinNearLandingSkipsAltitudeRequirement)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);

	// Takeoff, waypoint, land; vehicle close to the land item but 6 m below mission altitude.
	std::vector<mission_item_s> mission_items = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kBaseAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kBaseAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 120.f, 0.f, kBaseAlt - 10.f),
	};

	writeMissionItems(mission_items);
	writeSafePointState(0, 13);

	mission_s mission_state{};
	mission_state.timestamp = hrt_absolute_time();
	mission_state.current_seq = 2;
	mission_state.land_start_index = 2;
	mission_state.land_index = 2;
	mission_state.mission_id = 23;
	mission_state.safe_points_id = 13;
	mission_state.count = static_cast<uint16_t>(mission_items.size());
	mission_state.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	mission_state.fence_dataman_id = DM_KEY_FENCE_POINTS_0;
	mission_state.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;
	publishMission(mission_state);

	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 118.f, 0.f, kBaseAlt - 6.f);

	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishLandDetected(false);
	publishGlobalPosition(vehicle_position);
	publishLocalPosition(0.f, 8.f, 0.f);
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -100.f, 0.f, kBaseAlt));
	primeNavigatorState();

	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();

	ASSERT_TRUE(mission.trySetRouteJoinOnActivation(false));

	// Rejoin targets the land item and keeps the vehicle altitude (skip_altitude_requirement).
	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_TRUE(mission.joinContextForTest().valid());
	EXPECT_TRUE(mission.joinContextForTest().skip_altitude_requirement);
	EXPECT_NEAR(mission.joinContextForTest().projection.alt, vehicle_position.alt, 0.01f);
}

// Rejoining into a fixed-wing segment promotes JOIN_ROUTE to TRANSITION_AFTER_JOIN at the branch-in waypoint.
TEST_F(MissionRouteJoinTest, MissionSmartRejoinUsesTransitionAfterJoinWorkItemForFrontTransition)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);

	std::vector<mission_item_s> mission_items = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kBaseAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 150.f, 0.f, kBaseAlt + 20.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kBaseAlt + 20.f),
	};

	writeMissionItems(mission_items);
	writeSafePointState(0, 31);

	mission_s mission_state{};
	mission_state.timestamp = hrt_absolute_time();
	mission_state.current_seq = 2;
	mission_state.mission_id = 31;
	mission_state.safe_points_id = 31;
	mission_state.count = static_cast<uint16_t>(mission_items.size());
	mission_state.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	mission_state.fence_dataman_id = DM_KEY_FENCE_POINTS_0;
	mission_state.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;
	publishMission(mission_state);

	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishLandDetected(false);
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 60.f, 15.f, kBaseAlt + 5.f));
	publishLocalPosition(0.f, 5.f, 0.f);
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kBaseAlt - 20.f));
	primeNavigatorState();

	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();
	markMissionResultValid();

	mission.on_activation();
	EXPECT_EQ(mission.joinTransitionActionForTest(), MissionTestPeer::VtolTransitionAction::kFrontTransition);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);

	const mission_route::Position join_projection = mission.joinContextForTest().projection;
	publishGlobalPosition(join_projection);
	publishLocalPosition(0.f, 0.f, 0.f);
	primeNavigatorState();

	mission.on_active();

	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN);
}
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
