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
#include "support/navigator_dataman_test.h"
#include "support/vector_mission_item_store.h"

#include <vector>

#include <lib/geo/geo.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/home_position.h>

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

	// Mirror only successful dataman writes.
	bool writeMissionItemToCache(int32_t index, mission_item_s &mission_item) override
	{
		if (!_mission_store.canWriteItem(index)
		    || !MissionBase::writeMissionItemToCache(index, mission_item)) {
			return false;
		}

		return _mission_store.writeItem(index, mission_item);
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

	void setCurrentSequence(int32_t current_seq)
	{
		_mission.current_seq = current_seq;
	}

	int32_t currentSequence() const
	{
		return _mission.current_seq;
	}

	void checkMission(bool forced = false)
	{
		// Normal MissionBase callers mark the check before calling the validator.
		_mission_checked = true;
		check_mission_valid(forced);
	}

	bool missionChecked() const { return _mission_checked; }

	using MissionBase::findNextPositionIndex;
	using MissionBase::findPreviousPositionIndex;
	using MissionBase::getNonJumpItem;
	using MissionBase::getNextPositionItems;
	using MissionBase::goToNextPositionItem;
	using MissionBase::goToPreviousPositionItem;
	using MissionBase::MissionTraversalType;

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

class MissionBaseTestNavigator : public Navigator
{
public:
	using Navigator::updateParams;
};

class MissionBaseFeasibilityTest : public NavigatorDatamanTestBase
{
protected:
	void SetUp() override
	{
		param_reset_all();
		_navigator.updateParams();
		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_WAYPOINTS_OFFBOARD_0));
		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_FENCE_POINTS_0));

		home_position_s *home = _navigator.get_home_position();
		home->timestamp = hrt_absolute_time();
		home->lat = kBaseLat;
		home->lon = kBaseLon;
		home->alt = 0.f;
		home->valid_hpos = true;
		home->valid_alt = true;
		_home_pub.publish(*home);
		_navigator.get_land_detected()->landed = true;

		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		*_navigator.get_vstatus() = status;
		_status_pub.publish(status);

		mission_fence_point_s circle{};
		circle.nav_cmd = NAV_CMD_FENCE_CIRCLE_INCLUSION;
		circle.frame = NAV_FRAME_GLOBAL;
		circle.lat = kBaseLat;
		circle.lon = kBaseLon;
		circle.circle_radius = 100.f;
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_FENCE_POINTS_0, 0,
						      reinterpret_cast<uint8_t *>(&circle), sizeof(circle)));
		mission_stats_entry_s stats{};
		stats.dataman_id = DM_KEY_FENCE_POINTS_0;
		stats.opaque_id = 1;
		stats.num_items = 1;
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_FENCE_POINTS_STATE, 0,
						      reinterpret_cast<uint8_t *>(&stats), sizeof(stats)));
		_navigator.get_geofence().updateFence();
		ASSERT_TRUE(finishFenceUpdate());

		mission_s mission{};
		mission.timestamp = hrt_absolute_time();
		mission.mission_id = 1;
		mission.geofence_id = stats.opaque_id;
		mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
		_mission_base.loadTestMission({makePositionItem(kBaseLat, kBaseLon, kAlt)}, mission);
		ASSERT_TRUE(writeWaypoint(0.f));
	}

	void TearDown() override { param_reset_all(); }

	bool finishFenceUpdate()
	{
		Geofence &fence = _navigator.get_geofence();
		const hrt_abstime start = hrt_absolute_time();

		while (!fence.isReadyForPathChecks() && hrt_elapsed_time(&start) < 1_s) {
			fence.run();
			px4_usleep(1000);
		}

		return fence.isReadyForPathChecks();
	}

	bool writeWaypoint(float north_m)
	{
		mission_item_s item = makePositionItem(kBaseLat, kBaseLon, kAlt);
		add_vector_to_global_position(kBaseLat, kBaseLon, north_m, 0.f, &item.lat, &item.lon);
		return _dataman_client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0,
						 reinterpret_cast<uint8_t *>(&item), sizeof(item));
	}

	DatamanClient _dataman_client{};
	MissionBaseTestNavigator _navigator{};
	MissionBaseTestPeer _mission_base{&_navigator};
	uORB::Publication<home_position_s> _home_pub{ORB_ID(home_position)};
	uORB::Publication<vehicle_status_s> _status_pub{ORB_ID(vehicle_status)};
};

TEST_F(MissionBaseFeasibilityTest, DeferredCheckPreservesVerdictUntilRetry)
{
	_mission_base.checkMission(true);
	const mission_result_s checked_result = *_navigator.get_mission_result();
	ASSERT_TRUE(checked_result.valid);

	// Change storage without changing IDs to verify that the pending check really runs.
	ASSERT_TRUE(writeWaypoint(200.f));
	_navigator.get_geofence().updateFence();
	_mission_base.checkMission(true);
	EXPECT_TRUE(_navigator.get_mission_result()->valid);
	EXPECT_FALSE(_mission_base.missionChecked());
	EXPECT_EQ(_navigator.get_mission_result()->mission_id, checked_result.mission_id);
	EXPECT_EQ(_navigator.get_mission_result()->geofence_id, checked_result.geofence_id);
	EXPECT_EQ(_navigator.get_mission_result()->home_position_counter, checked_result.home_position_counter);

	ASSERT_TRUE(finishFenceUpdate());
	_mission_base.checkMission();
	ASSERT_FALSE(_navigator.get_mission_result()->valid);
	EXPECT_TRUE(_mission_base.missionChecked());

	// A completed failure stays cached until another check is requested.
	ASSERT_TRUE(writeWaypoint(0.f));
	_mission_base.checkMission();
	EXPECT_FALSE(_navigator.get_mission_result()->valid);

	_navigator.get_geofence().updateFence();
	_mission_base.checkMission(true);
	EXPECT_FALSE(_navigator.get_mission_result()->valid);
	EXPECT_FALSE(_mission_base.missionChecked());
	ASSERT_TRUE(finishFenceUpdate());
	_mission_base.checkMission();
	EXPECT_TRUE(_navigator.get_mission_result()->valid);
	EXPECT_TRUE(_mission_base.missionChecked());
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

// WHY: Mission-based Return configures position traversal to skip DO_JUMP loops consistently.
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

// WHY: Reverse mission-path Return must skip DO_JUMP loops for backward progression too.
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
