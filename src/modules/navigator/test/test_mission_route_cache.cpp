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
 * @file test_mission_route_cache.cpp
 *
 * MissionRouteCache regression tests.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include <gtest/gtest.h>

#include <drivers/drv_hrt.h>

#include "mission_route_cache.h"
#include "support/mission_route_cache_test_peer.h"
#include "support/mission_route_test_helpers.h"

#include <vector>

using navigator_test::makeLandItemFromOffset;
using navigator_test::makePositionItemFromOffset;
using navigator_test::makeSafePointFromOffset;
using navigator_test::makeTakeoffItemFromOffset;
using navigator_test::route_test_reference::kAlt;
using navigator_test::route_test_reference::kBaseLat;
using navigator_test::route_test_reference::kBaseLon;

class MissionRouteCacheTest : public NavigatorDatamanTestBase
{
protected:
	void SetUp() override
	{
		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_WAYPOINTS_OFFBOARD_0));
		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_WAYPOINTS_OFFBOARD_1));
		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_SAFE_POINTS_0));
		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_SAFE_POINTS_1));

		mission_stats_entry_s empty_stats{};
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_STATE, 0,
						      reinterpret_cast<uint8_t *>(&empty_stats), sizeof(empty_stats)));

		_cache.invalidate();
	}

	mission_s makeMission(uint32_t mission_id, uint16_t count, uint32_t safe_points_id = 0,
			      int32_t land_index = -1, dm_item_t mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0,
			      dm_item_t safepoint_dataman_id = DM_KEY_SAFE_POINTS_0) const
	{
		mission_s mission{};
		mission.timestamp = hrt_absolute_time();
		mission.mission_id = mission_id;
		mission.count = count;
		mission.land_index = land_index;
		mission.mission_dataman_id = static_cast<uint8_t>(mission_dataman_id);
		mission.safe_points_id = safe_points_id;
		mission.safepoint_dataman_id = static_cast<uint8_t>(safepoint_dataman_id);
		return mission;
	}

	void writeMissionItem(const mission_item_s &item, uint32_t index,
			      dm_item_t dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0)
	{
		mission_item_s copy = item;
		ASSERT_TRUE(_dataman_client.writeSync(dataman_id, index,
						      reinterpret_cast<uint8_t *>(&copy), sizeof(copy)));
	}

	void writeMissionItems(const std::vector<mission_item_s> &items,
			       dm_item_t dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0)
	{
		for (size_t i = 0; i < items.size(); ++i) {
			writeMissionItem(items[i], static_cast<uint32_t>(i), dataman_id);
		}
	}

	void writeSafePointItems(const std::vector<mission_item_s> &items, uint16_t num_items,
				 uint32_t opaque_id, dm_item_t dataman_id = DM_KEY_SAFE_POINTS_0)
	{
		for (size_t i = 0; i < items.size(); ++i) {
			mission_item_s copy = items[i];
			ASSERT_TRUE(_dataman_client.writeSync(dataman_id, static_cast<uint32_t>(i),
							      reinterpret_cast<uint8_t *>(&copy), sizeof(copy)));
		}

		writeSafePointState(num_items, opaque_id, dataman_id);
	}

	void writeSafePointState(uint16_t num_items, uint32_t opaque_id,
				 dm_item_t dataman_id = DM_KEY_SAFE_POINTS_0)
	{
		mission_stats_entry_s stats{};
		stats.num_items = num_items;
		stats.opaque_id = opaque_id;
		stats.dataman_id = static_cast<uint8_t>(dataman_id);
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_STATE, 0,
						      reinterpret_cast<uint8_t *>(&stats), sizeof(stats)));
	}

	static void expectMissionItemMatches(const mission_item_s &actual, const mission_item_s &expected)
	{
		EXPECT_EQ(actual.nav_cmd, expected.nav_cmd);
		EXPECT_DOUBLE_EQ(actual.lat, expected.lat);
		EXPECT_DOUBLE_EQ(actual.lon, expected.lon);
		EXPECT_FLOAT_EQ(actual.altitude, expected.altitude);
		EXPECT_EQ(actual.altitude_is_relative, expected.altitude_is_relative);
		EXPECT_EQ(actual.autocontinue, expected.autocontinue);
	}

	DatamanClient _dataman_client{};
	MissionRouteCache _cache{};
};

// The dedicated land-item cache is not exposed until the async load has been validated.
TEST_F(MissionRouteCacheTest, MissionLandItemIsHiddenUntilValidated)
{
	const std::vector<mission_item_s> mission_items{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt + 10.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
	};
	const int32_t land_index_expected = 1;
	const mission_s mission = makeMission(22, static_cast<uint16_t>(mission_items.size()), 0, land_index_expected);
	writeMissionItems(mission_items);

	_cache.update(mission);

	EXPECT_TRUE(_cache.missionLandItemUpdatePending());
	EXPECT_FALSE(_cache.missionLandItemReady());

	// Failed reads leave output parameters untouched.
	int32_t land_index = 123;
	mission_item_s land_item{};
	EXPECT_FALSE(_cache.getMissionLandItem(land_index, land_item));
	EXPECT_EQ(land_index, 123);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.missionLandItemReady(); }))
			<< "mission land item did not become ready";
	EXPECT_FALSE(_cache.missionLandItemUpdatePending());
}

// Published land_index loads the dedicated land-item cache.
TEST_F(MissionRouteCacheTest, MissionLandItemLoadsReferencedWaypoint)
{
	// land_index points to the final landing item.
	const std::vector<mission_item_s> mission_items{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f,  0.f, kAlt + 15.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 120.f, 0.f, kAlt + 30.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 240.f, 0.f, kAlt),
	};
	const int32_t land_index_expected = 2;
	const mission_s mission = makeMission(20, static_cast<uint16_t>(mission_items.size()), 0, land_index_expected);
	writeMissionItems(mission_items);

	// Wait for the land-item cache.
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] {
		int32_t ready_index = -1;
		mission_item_s ready_land_item{};
		return _cache.getMissionLandItem(ready_index, ready_land_item);
	}))
			<< "mission land item did not become ready";

	// The cached land item follows the published index.
	EXPECT_TRUE(_cache.missionLandItemReady());
	EXPECT_FALSE(_cache.missionLandItemUpdatePending());
	int32_t land_index = -1;
	mission_item_s land_item{};
	ASSERT_TRUE(_cache.getMissionLandItem(land_index, land_item));
	EXPECT_EQ(land_index, land_index_expected);
	expectMissionItemMatches(land_item, mission_items[land_index_expected]);
}

// The cache trusts land_index instead of rescanning for a landing item.
TEST_F(MissionRouteCacheTest, MissionLandItemRejectsOutOfBoundsPublishedIndex)
{
	// The mission has a landing item, but land_index is invalid.
	const std::vector<mission_item_s> mission_items{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f,  0.f, kAlt + 15.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 120.f, 0.f, kAlt + 30.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 240.f, 0.f, kAlt),
	};
	const mission_s mission = makeMission(21, static_cast<uint16_t>(mission_items.size()), 0,
					      static_cast<int32_t>(mission_items.size()));
	writeMissionItems(mission_items);

	// An invalid index queues nothing, so a single update settles the land state.
	_cache.update(mission);

	// No land item is exposed from the invalid index.
	EXPECT_FALSE(_cache.missionLandItemReady());
	EXPECT_FALSE(_cache.missionLandItemUpdatePending());
	int32_t land_index = -1;
	mission_item_s land_item{};
	EXPECT_FALSE(_cache.getMissionLandItem(land_index, land_item));
}

// A published land_index must contain a land command.
TEST_F(MissionRouteCacheTest, MissionLandItemRejectsNonLandPublishedIndex)
{
	// The mission contains a land item, but the published land_index points at a normal waypoint.
	const std::vector<mission_item_s> mission_items{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f,  0.f, kAlt + 15.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 120.f, 0.f, kAlt + 30.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 240.f, 0.f, kAlt),
	};
	const mission_s mission = makeMission(23, static_cast<uint16_t>(mission_items.size()), 0, 1);
	writeMissionItems(mission_items);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission,
			[&] { return MissionRouteCacheTestPeer::missionLandRetryScheduled(_cache); }))
			<< "mission land cache retry was not scheduled";

	EXPECT_FALSE(_cache.missionLandItemReady());
	EXPECT_TRUE(_cache.missionLandItemUpdatePending());
	EXPECT_GT(MissionRouteCacheTestPeer::missionLandRetryCount(_cache), 0U);
	EXPECT_TRUE(_cache.missionLandItemAttemptFailed());

	// Failed reads leave output parameters untouched.
	int32_t land_index = 123;
	mission_item_s land_item{};
	EXPECT_FALSE(_cache.getMissionLandItem(land_index, land_item));
	EXPECT_EQ(land_index, 123);
}

TEST_F(MissionRouteCacheTest, MissionLandItemRejectsInvalidDatamanId)
{
	const mission_s mission = makeMission(24, 1, 0, 0, static_cast<dm_item_t>(DM_KEY_NUM_KEYS));
	_cache.update(mission);

	EXPECT_FALSE(_cache.missionLandItemReady());
	EXPECT_FALSE(_cache.missionLandItemUpdatePending());
}

// Transient safe-point state errors retry without changing safe_points_id.
TEST_F(MissionRouteCacheTest, SafePointCacheRetriesAfterInvalidStateWithoutIdChange)
{
	// Start with an invalid state entry for the current safe_points_id.
	const mission_s mission = makeMission(0, 0, 41);
	writeSafePointState(DM_KEY_SAFE_POINTS_MAX + 1, 41);
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission,
			[&] { return MissionRouteCacheTestPeer::safePointRetryScheduled(_cache); }))
			<< "safe-point retry was not scheduled";
	EXPECT_FALSE(_cache.safePointsReady());

	const std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 20.f, 5.f, kAlt),
	};
	writeSafePointItems(safe_points, static_cast<uint16_t>(safe_points.size()), 41);

	// Skip the retry backoff wait and keep driving the cache on the now-valid state.
	MissionRouteCacheTestPeer::expireSafePointRetryBackoff(_cache);
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.safePointsReady(); }))
			<< "safe-point cache did not become ready";

	// The safe point becomes available after retry.
	EXPECT_EQ(_cache.safePointCount(), 1);
	mission_item_s safe_point{};
	ASSERT_TRUE(_cache.loadSafePointItem(0, safe_point));
	expectMissionItemMatches(safe_point, safe_points[0]);
}

TEST_F(MissionRouteCacheTest, SafePointCacheRejectsMismatchedSourceId)
{
	const mission_s mission = makeMission(0, 0, 90);
	writeSafePointState(1, 91);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission,
			[&] { return MissionRouteCacheTestPeer::safePointRetryScheduled(_cache); }));
	EXPECT_FALSE(_cache.safePointsReady());
}

// safe_points_id participates in cache identity.
TEST_F(MissionRouteCacheTest, SafePointIdChangeReloadsReplacementSet)
{
	const std::vector<mission_item_s> safe_points_a{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt),
	};

	const std::vector<mission_item_s> safe_points_b{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 120.f, 0.f, kAlt),
	};

	mission_s mission = makeMission(0, 0, 100);
	writeSafePointItems(safe_points_a, static_cast<uint16_t>(safe_points_a.size()), 100);

	// Load the first set before changing safe_points_id.
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.safePointsReady(); }))
			<< "safe-point cache did not become ready";

	mission_item_s safe_point{};
	ASSERT_TRUE(_cache.loadSafePointItem(0, safe_point));
	expectMissionItemMatches(safe_point, safe_points_a[0]);

	writeSafePointItems(safe_points_b, static_cast<uint16_t>(safe_points_b.size()), 101);
	mission.safe_points_id = 101;
	mission.timestamp = hrt_absolute_time();

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.safePointsReady(); }))
			<< "safe-point cache did not become ready";

	// The stale set is replaced.
	ASSERT_TRUE(_cache.loadSafePointItem(0, safe_point));
	expectMissionItemMatches(safe_point, safe_points_b[0]);
}

// Source changes during safe-point loads discard stale work.
TEST_F(MissionRouteCacheTest, SafePointSourceChangeDuringLoadDoesNotExposeStaleData)
{
	// A large set is interrupted by a replacement set.
	const std::vector<mission_item_s> safe_points_a{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 20.f, 0.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 30.f, 0.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 40.f, 0.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 60.f, 0.f, kAlt),
	};

	const std::vector<mission_item_s> safe_points_b{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	const mission_s mission_a = makeMission(0, 0, 200);
	writeSafePointItems(safe_points_a, static_cast<uint16_t>(safe_points_a.size()), 200);

	// Stop once the first set is in flight.
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission_a,
			[&] { return MissionRouteCacheTestPeer::safePointLoadInProgress(_cache); }))
			<< "safe-point load did not start";
	ASSERT_FALSE(_cache.safePointsReady());

	// Change the source while the first set is still loading.
	mission_s mission_b = makeMission(0, 0, 201);
	writeSafePointItems(safe_points_b, static_cast<uint16_t>(safe_points_b.size()), 201);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission_b, [&] { return _cache.safePointsReady(); }))
			<< "safe-point cache did not become ready";

	// Only the replacement set is visible.
	mission_item_s safe_point{};
	ASSERT_TRUE(_cache.loadSafePointItem(0, safe_point));
	expectMissionItemMatches(safe_point, safe_points_b[0]);
	EXPECT_EQ(_cache.safePointCount(), 1);
}

// A zero-count safe-point set is a valid ready-and-empty state.
TEST_F(MissionRouteCacheTest, SafePointZeroCountIsReadyAndEmpty)
{
	const mission_s mission = makeMission(0, 0, 50);
	writeSafePointState(0, 50);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.safePointsReady(); }))
			<< "zero-count safe-point set did not become ready";

	EXPECT_EQ(_cache.safePointCount(), 0);

	mission_item_s safe_point{};
	EXPECT_FALSE(_cache.loadSafePointItem(0, safe_point));
}

TEST_F(MissionRouteCacheTest, SafePointZeroCountIgnoresUnusedDatamanId)
{
	const mission_s mission = makeMission(0, 0, 51);
	writeSafePointState(0, 51, DM_KEY_WAYPOINTS_OFFBOARD_0);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.safePointsReady(); }));
	EXPECT_EQ(_cache.safePointCount(), 0);
}

// Changed safe-point stats with a reused opaque_id and unchanged safe_points_id still reload.
TEST_F(MissionRouteCacheTest, SafePointStatsChangeWithSameOpaqueIdReloads)
{
	const std::vector<mission_item_s> safe_points_a{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt),
	};
	const std::vector<mission_item_s> safe_points_b{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 30.f, 0.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 60.f, 0.f, kAlt),
	};

	const mission_s mission = makeMission(0, 0, 60);
	writeSafePointItems(safe_points_a, static_cast<uint16_t>(safe_points_a.size()), 60);
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.safePointsReady(); }))
			<< "safe-point cache did not become ready";
	ASSERT_EQ(_cache.safePointCount(), 1);

	// Grow the stored set while reusing both the opaque id and safe_points_id.
	writeSafePointItems(safe_points_b, static_cast<uint16_t>(safe_points_b.size()), 60);
	MissionRouteCacheTestPeer::requestSafePointRecheck(_cache);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.safePointCount() == 2; }))
			<< "safe-point cache did not reload after stats change";

	mission_item_s safe_point{};
	ASSERT_TRUE(_cache.loadSafePointItem(1, safe_point));
	expectMissionItemMatches(safe_point, safe_points_b[1]);
}

// A safe-point source-bank change reloads from the new storage.
TEST_F(MissionRouteCacheTest, SafePointDatamanIdChangeReloadsFromNewStorage)
{
	const std::vector<mission_item_s> safe_points_0{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 15.f, 0.f, kAlt),
	};
	const std::vector<mission_item_s> safe_points_1{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 45.f, 0.f, kAlt),
	};

	mission_s mission = makeMission(0, 0, 70);
	writeSafePointItems(safe_points_0, static_cast<uint16_t>(safe_points_0.size()), 70, DM_KEY_SAFE_POINTS_0);
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.safePointsReady(); }))
			<< "safe-point cache did not become ready";

	mission_item_s safe_point{};
	ASSERT_TRUE(_cache.loadSafePointItem(0, safe_point));
	expectMissionItemMatches(safe_point, safe_points_0[0]);

	writeSafePointItems(safe_points_1, static_cast<uint16_t>(safe_points_1.size()), 70, DM_KEY_SAFE_POINTS_1);
	mission.safepoint_dataman_id = DM_KEY_SAFE_POINTS_1;
	mission.timestamp = hrt_absolute_time();

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.safePointsReady(); }))
			<< "safe-point cache did not reload after dataman id change";

	ASSERT_TRUE(_cache.loadSafePointItem(0, safe_point));
	expectMissionItemMatches(safe_point, safe_points_1[0]);
}

// Safe-point states pointing at a non safe-point dataman key are rejected and retried.
TEST_F(MissionRouteCacheTest, SafePointCacheRejectsInvalidDatamanId)
{
	const mission_s mission = makeMission(0, 0, 80, -1, DM_KEY_WAYPOINTS_OFFBOARD_0,
					      DM_KEY_WAYPOINTS_OFFBOARD_0);

	// A plausible count but an unsupported storage key must be rejected before any load.
	writeSafePointState(1, 80, DM_KEY_WAYPOINTS_OFFBOARD_0);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission,
			[&] { return MissionRouteCacheTestPeer::safePointRetryScheduled(_cache); }))
			<< "safe-point retry was not scheduled for invalid dataman id";

	EXPECT_FALSE(_cache.safePointsReady());
	EXPECT_EQ(_cache.safePointCount(), 0);
}
