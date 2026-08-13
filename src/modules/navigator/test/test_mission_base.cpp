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
#include "support/navigator_dataman_test.h"
#include "support/vector_mission_item_store.h"

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_status.h>
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
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
