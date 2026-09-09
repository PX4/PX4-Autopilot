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
 * @file test_mission_lookahead.cpp
 *
 * Tests for the gate that decides whether the speed planning lookahead may be published, i.e. whether
 * the vehicle really flies through the next waypoint instead of stopping at it.
 */

#include <gtest/gtest.h>

#include "mission.h"
#include "navigator.h"
#include "support/navigator_dataman_test.h"

#include <dataman_client/DatamanClient.hpp>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/vehicle_status.h>

#include <vector>

class MissionLookaheadTestPeer : public Mission
{
public:
	explicit MissionLookaheadTestPeer(Navigator *navigator) : Mission(navigator) {}

	void useMissionDataman(dm_item_t dataman_id)
	{
		_mission.mission_dataman_id = static_cast<uint8_t>(dataman_id);
	}

	/* The predicate only reads items in between from the cache, so prime it the way updateDatamanCache() does. */
	void cacheItems(dm_item_t dataman_id, int32_t count)
	{
		_dataman_cache.invalidate();

		for (int32_t index = 0; index < count; index++) {
			_dataman_cache.load(dataman_id, static_cast<uint32_t>(index));
		}

		const hrt_abstime start = hrt_absolute_time();

		while (_dataman_cache.isLoading() && (hrt_elapsed_time(&start) < 1_s)) {
			_dataman_cache.update();
			px4_usleep(1000);
		}
	}

	void dropCachedItems()
	{
		_dataman_cache.invalidate();
	}

	using Mission::isFlownThroughWithoutStopping;
};

static mission_item_s makeWaypoint()
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_WAYPOINT;
	item.autocontinue = true;
	item.lat = 47.0;
	item.lon = 8.0;
	item.altitude = 100.f;
	return item;
}

static mission_item_s makeCommand(uint16_t nav_cmd)
{
	mission_item_s item{};
	item.nav_cmd = nav_cmd;
	item.autocontinue = true;
	return item;
}

class MissionLookaheadTest : public NavigatorDatamanTestBase
{
protected:
	static constexpr dm_item_t kMissionDataman{DM_KEY_WAYPOINTS_OFFBOARD_0};

	void SetUp() override
	{
		ASSERT_TRUE(_dataman_client.clearSync(kMissionDataman));
		/* get_time_inside() only holds a plain waypoint for a rotary wing, same as brake_for_hold. */
		_navigator.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		_mission.useMissionDataman(kMissionDataman);
	}

	void writeMission(const std::vector<mission_item_s> &items)
	{
		for (size_t index = 0; index < items.size(); index++) {
			mission_item_s item = items[index];
			ASSERT_TRUE(_dataman_client.writeSync(kMissionDataman, static_cast<uint32_t>(index),
							      reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		}

		_mission.cacheItems(kMissionDataman, static_cast<int32_t>(items.size()));
	}

	DatamanClient _dataman_client{};
	Navigator _navigator{};
	MissionLookaheadTestPeer _mission{&_navigator};
};

TEST_F(MissionLookaheadTest, PlainWaypointIsFlownThrough)
{
	// GIVEN: two consecutive plain waypoints
	const std::vector<mission_item_s> items{makeWaypoint(), makeWaypoint()};
	writeMission(items);

	// THEN: the first one is passed at speed, so the lookahead may be published
	EXPECT_TRUE(_mission.isFlownThroughWithoutStopping(items[0], 0, 1));
}

TEST_F(MissionLookaheadTest, WaypointWithTimeInsideStopsTheVehicle)
{
	// GIVEN: a waypoint the vehicle has to hold at
	std::vector<mission_item_s> items{makeWaypoint(), makeWaypoint()};
	items[0].time_inside = 5.f;
	writeMission(items);

	// THEN: no lookahead, the vehicle must be able to stop there
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 1));
}

TEST_F(MissionLookaheadTest, WaypointWithoutAutocontinueStopsTheVehicle)
{
	// GIVEN: a waypoint the mission does not continue past by itself
	std::vector<mission_item_s> items{makeWaypoint(), makeWaypoint()};
	items[0].autocontinue = false;
	writeMission(items);

	// THEN: no lookahead
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 1));
}

TEST_F(MissionLookaheadTest, LoiterStopsTheVehicle)
{
	// GIVEN: a loiter item instead of a plain waypoint
	std::vector<mission_item_s> items{makeCommand(NAV_CMD_LOITER_TIME_LIMIT), makeWaypoint()};
	items[0].time_inside = 10.f;
	writeMission(items);

	// THEN: no lookahead
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 1));
}

TEST_F(MissionLookaheadTest, LandingStopsTheVehicle)
{
	// GIVEN: a landing instead of a plain waypoint
	const std::vector<mission_item_s> items{makeCommand(NAV_CMD_LAND), makeWaypoint()};
	writeMission(items);

	// THEN: no lookahead
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 1));
}

TEST_F(MissionLookaheadTest, DelayBetweenPositionItemsStopsTheVehicle)
{
	// GIVEN: a delay between the waypoint and the following position item
	const std::vector<mission_item_s> items{makeWaypoint(), makeCommand(NAV_CMD_DELAY), makeWaypoint()};
	writeMission(items);

	// THEN: no lookahead, the vehicle waits at the waypoint
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 2));
}

TEST_F(MissionLookaheadTest, VtolTransitionBetweenPositionItemsStopsTheVehicle)
{
	// GIVEN: a transition between the waypoint and the following position item
	const std::vector<mission_item_s> items{makeWaypoint(), makeCommand(NAV_CMD_DO_VTOL_TRANSITION), makeWaypoint()};
	writeMission(items);

	// THEN: no lookahead
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 2));
}

TEST_F(MissionLookaheadTest, DoJumpBetweenPositionItemsStopsTheVehicle)
{
	// GIVEN: a jump between the waypoint and the following position item, which may redirect the mission
	const std::vector<mission_item_s> items{makeWaypoint(), makeCommand(NAV_CMD_DO_JUMP), makeWaypoint()};
	writeMission(items);

	// THEN: no lookahead, the item after next is not the one the jump leads to
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 2));
}

TEST_F(MissionLookaheadTest, PayloadCommandWithTimeoutBetweenPositionItemsStopsTheVehicle)
{
	// GIVEN: a payload command the vehicle waits for between the waypoint and the following position item
	const std::vector<mission_item_s> items{makeWaypoint(), makeCommand(NAV_CMD_DO_GRIPPER), makeWaypoint()};
	writeMission(items);

	// THEN: no lookahead
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 2));
}

TEST_F(MissionLookaheadTest, HarmlessCommandBetweenPositionItemsIsFlownThrough)
{
	// GIVEN: a command that does not hold the vehicle between the waypoint and the following position item
	const std::vector<mission_item_s> items{makeWaypoint(), makeCommand(NAV_CMD_DO_CHANGE_SPEED), makeWaypoint()};
	writeMission(items);

	// THEN: the waypoint is still flown through
	EXPECT_TRUE(_mission.isFlownThroughWithoutStopping(items[0], 0, 2));
}

TEST_F(MissionLookaheadTest, FollowingItemBeforeTheWaypointStopsTheVehicle)
{
	// GIVEN: a following position item that lies before the waypoint, i.e. the mission goes backwards
	const std::vector<mission_item_s> items{makeWaypoint(), makeWaypoint(), makeWaypoint()};
	writeMission(items);

	// THEN: no lookahead, the geometry after the waypoint is not known
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[2], 2, 1));
}

TEST_F(MissionLookaheadTest, UncachedItemInBetweenStopsTheVehicle)
{
	// GIVEN: an item in between that is not in the dataman cache
	const std::vector<mission_item_s> items{makeWaypoint(), makeCommand(NAV_CMD_DO_CHANGE_SPEED), makeWaypoint()};
	writeMission(items);
	_mission.dropCachedItems();

	// THEN: the cache miss is treated like an item that stops the vehicle
	EXPECT_FALSE(_mission.isFlownThroughWithoutStopping(items[0], 0, 2));
}
