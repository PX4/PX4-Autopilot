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
#include <px4_platform_common/posix.h>

#include "full_mission_cache.h"
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

	// Load item 0, fail item 1, and wait for retry backoff.
	bool enterRetryBackoffAfterFirstItem(const mission_s &mission)
	{
		if (!MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] {
		return MissionRouteCacheTestPeer::missionNextIndex(_cache) == 1
			&& MissionRouteCacheTestPeer::missionLoadInProgress(_cache);
		})) {
			return false;
		}

		if (!MissionRouteCacheTestPeer::failPendingMissionLoad(_cache)) {
			return false;
		}

		return MissionRouteCacheTestPeer::runCacheUntil(_cache, mission,
				[&] { return MissionRouteCacheTestPeer::missionRetryScheduled(_cache); });
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

// Fully loaded mission cache with readable items.
TEST_F(MissionRouteCacheTest, MissionCacheLoadsAllMissionItems)
{
	// Ten mission items in dataman.
	const std::vector<mission_item_s> mission_items{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt + 10.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt + 20.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f,  20.f, kAlt + 25.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f,  40.f, kAlt + 30.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f,  60.f, kAlt + 35.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f,  80.f, kAlt + 40.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 600.f, 100.f, kAlt + 45.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 700.f, 120.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 800.f, 140.f, kAlt + 55.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 900.f, 160.f, kAlt),
	};
	const mission_s mission = makeMission(17, static_cast<uint16_t>(mission_items.size()), 0,
					      static_cast<int32_t>(mission_items.size() - 1));
	writeMissionItems(mission_items);

	// Drive the async cache until ready.
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.missionItemsReady(mission); }))
			<< "mission cache did not become ready";

	MissionRouteCache::MissionView view{};
	ASSERT_TRUE(_cache.getMissionView(mission, view));
	ASSERT_NE(view.items, nullptr);
	EXPECT_EQ(view.count, static_cast<int32_t>(mission_items.size()));
	EXPECT_EQ(view.mission_id, mission.mission_id);
	EXPECT_EQ(view.dataman_id, mission.mission_dataman_id);
	EXPECT_NE(view.generation, 0U);
	EXPECT_EQ(reinterpret_cast<uintptr_t>(view.items) % alignof(mission_item_s), 0U);

	// Every cached item matches the source.
	ASSERT_EQ(_cache.missionCount(), static_cast<int>(mission_items.size()));

	for (size_t i = 0; i < mission_items.size(); ++i) {
		expectMissionItemMatches(view.items[i], mission_items[i]);

		mission_item_s cached_item{};
		ASSERT_TRUE(_cache.loadMissionItem(mission, static_cast<int32_t>(i), cached_item));
		expectMissionItemMatches(cached_item, mission_items[i]);
	}
}

TEST_F(MissionRouteCacheTest, InvalidateImmediatelyHidesMissionView)
{
	const std::vector<mission_item_s> mission_items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
	};
	const mission_s mission = makeMission(16, static_cast<uint16_t>(mission_items.size()));
	writeMissionItems(mission_items);
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.missionItemsReady(mission); }));

	MissionRouteCache::MissionView view{};
	ASSERT_TRUE(_cache.getMissionView(mission, view));

	_cache.invalidate();

	EXPECT_FALSE(_cache.getMissionView(mission, view));
	EXPECT_EQ(_cache.missionCount(), 0);
}

// Failed mission reads stay unavailable and schedule a retry.
TEST_F(MissionRouteCacheTest, MissionCacheSchedulesRetryWhenReadFails)
{
	const std::vector<mission_item_s> mission_items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	const mission_s mission = makeMission(18, static_cast<uint16_t>(mission_items.size()));
	writeMissionItems(mission_items);

	ASSERT_TRUE(enterRetryBackoffAfterFirstItem(mission))
			<< "mission cache retry was not scheduled after a read failure";

	// The load front stopped at the failed second item and nothing is in flight.
	EXPECT_EQ(MissionRouteCacheTestPeer::missionNextIndex(_cache), 1);
	EXPECT_FALSE(MissionRouteCacheTestPeer::missionLoadInProgress(_cache));
	EXPECT_EQ(_cache.fullMissionResponseSubscription(), ORB_SUB_INVALID);

	// The failed generation stays unavailable.
	EXPECT_FALSE(_cache.missionItemsReady(mission));
	EXPECT_EQ(_cache.missionCount(), 0);
	EXPECT_GT(MissionRouteCacheTestPeer::missionRetryCount(_cache), 0U);
	EXPECT_EQ(MissionRouteCacheTestPeer::missionLoadPerfEventCount(_cache), 0U);
	MissionRouteCache::MissionView view{};
	EXPECT_FALSE(_cache.getMissionView(mission, view));

	mission_item_s cached_item{};
	EXPECT_FALSE(_cache.loadMissionItem(mission, 0, cached_item));

	// The resumed load completes the same generation from the failed index.
	MissionRouteCacheTestPeer::expireMissionRetryBackoff(_cache);
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.missionItemsReady(mission); }));
	ASSERT_TRUE(_cache.getMissionView(mission, view));
	EXPECT_EQ(MissionRouteCacheTestPeer::missionLoadPerfEventCount(_cache), 1U);
	expectMissionItemMatches(view.items[0], mission_items[0]);
	expectMissionItemMatches(view.items[1], mission_items[1]);
}

// Oversized missions are rejected before cache loads are queued.
TEST_F(MissionRouteCacheTest, MissionCacheRejectsTooLargeMission)
{
	// One item beyond the configured cache size.
	const mission_s mission = makeMission(19, static_cast<uint16_t>(MissionRouteCache::kMaxFullMissionCacheSize + 1));

	// Driving the cache never makes an oversized mission ready and exposes no items.
	EXPECT_FALSE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.missionItemsReady(mission); }));

	EXPECT_FALSE(_cache.missionItemsReady(mission));
	EXPECT_EQ(_cache.missionCount(), 0);

	mission_item_s cached_item{};
	EXPECT_FALSE(_cache.loadMissionItem(mission, 0, cached_item));
}

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
	EXPECT_FALSE(_cache.missionLandItemAttemptFailed());

	// Failed reads leave output parameters untouched.
	int32_t land_index = 123;
	mission_item_s land_item{};
	EXPECT_FALSE(_cache.getMissionLandItem(land_index, land_item));
	EXPECT_EQ(land_index, 123);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.missionLandItemReady(); }))
			<< "mission land item did not become ready";
	EXPECT_FALSE(_cache.missionLandItemUpdatePending());
	EXPECT_FALSE(_cache.missionLandItemAttemptFailed());
}

// A new mission-land source immediately hides the previous item while its replacement loads asynchronously.
TEST_F(MissionRouteCacheTest, MissionLandSourceChangeHidesPreviousItemUntilValidated)
{
	const mission_item_s land_item_a = makeLandItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);
	const mission_item_s land_item_b = makeLandItemFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt);
	const mission_s mission_a = makeMission(30, 1, 0, 0, DM_KEY_WAYPOINTS_OFFBOARD_0);
	const mission_s mission_b = makeMission(31, 1, 0, 0, DM_KEY_WAYPOINTS_OFFBOARD_1);

	writeMissionItem(land_item_a, 0, DM_KEY_WAYPOINTS_OFFBOARD_0);
	writeMissionItem(land_item_b, 0, DM_KEY_WAYPOINTS_OFFBOARD_1);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission_a, [&] { return _cache.missionLandItemReady(); }))
			<< "first mission land item did not become ready";

	int32_t land_index = -1;
	mission_item_s land_item{};
	ASSERT_TRUE(_cache.getMissionLandItem(land_index, land_item));
	expectMissionItemMatches(land_item, land_item_a);

	_cache.update(mission_b);

	EXPECT_FALSE(_cache.missionLandItemReady());
	EXPECT_TRUE(_cache.missionLandItemUpdatePending());
	EXPECT_FALSE(_cache.missionLandItemAttemptFailed());
	land_index = 123;
	EXPECT_FALSE(_cache.getMissionLandItem(land_index, land_item));
	EXPECT_EQ(land_index, 123);
	expectMissionItemMatches(land_item, land_item_a);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission_b, [&] { return _cache.missionLandItemReady(); }))
			<< "replacement mission land item did not become ready";
	EXPECT_FALSE(_cache.missionLandItemUpdatePending());
	EXPECT_FALSE(_cache.missionLandItemAttemptFailed());
	ASSERT_TRUE(_cache.getMissionLandItem(land_index, land_item));
	EXPECT_EQ(land_index, 0);
	expectMissionItemMatches(land_item, land_item_b);
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

	// Wait for mission and land-item caches.
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] {
		int32_t ready_index = -1;
		mission_item_s ready_land_item{};
		return _cache.missionItemsReady(mission) && _cache.getMissionLandItem(ready_index, ready_land_item);
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

	// The mission cache itself still becomes ready.
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.missionItemsReady(mission); }))
			<< "mission cache did not become ready";

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

	// A source change immediately hides the old generation instead of blocking or exposing stale data.
	_cache.update(mission);
	EXPECT_EQ(_cache.safePointsId(), mission.safe_points_id);
	EXPECT_FALSE(_cache.safePointsReady());
	EXPECT_TRUE(_cache.safePointUpdatePending());
	EXPECT_EQ(_cache.safePointCount(), 0);
	EXPECT_FALSE(_cache.loadSafePointItem(0, safe_point));

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.safePointsReady(); }))
			<< "safe-point cache did not become ready";
	EXPECT_FALSE(_cache.safePointUpdatePending());

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

// An empty mission is immediately ready and exposes no items.
TEST_F(MissionRouteCacheTest, EmptyMissionIsReadyAndHasNoItems)
{
	mission_s mission = makeMission(0, 0);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.missionItemsReady(mission); }))
			<< "empty mission did not become ready";

	EXPECT_TRUE(_cache.missionItemsReady(mission));
	EXPECT_EQ(_cache.missionCount(), 0);

	MissionRouteCache::MissionView view{};
	ASSERT_TRUE(_cache.getMissionView(mission, view));
	EXPECT_EQ(view.items, nullptr);
	EXPECT_EQ(view.count, 0);
	EXPECT_EQ(view.mission_id, mission.mission_id);
	EXPECT_NE(view.generation, 0U);

	mission_item_s cached_item{};
	EXPECT_FALSE(_cache.loadMissionItem(mission, 0, cached_item));

	// An empty mission remains valid when its otherwise unused storage key changes.
	mission.mission_dataman_id = DM_KEY_SAFE_POINTS_0;
	_cache.update(mission);
	EXPECT_TRUE(_cache.missionItemsReady(mission));
}

// A mission_dataman_id change reloads the cache from the new storage.
TEST_F(MissionRouteCacheTest, MissionDatamanIdChangeReloadsCache)
{
	const std::vector<mission_item_s> mission_items_0{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt + 10.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
	};
	const std::vector<mission_item_s> mission_items_1{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt + 20.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt),
	};

	// Load the mission from storage 0.
	const mission_s mission_0 = makeMission(31, static_cast<uint16_t>(mission_items_0.size()), 0, -1,
						DM_KEY_WAYPOINTS_OFFBOARD_0);
	writeMissionItems(mission_items_0, DM_KEY_WAYPOINTS_OFFBOARD_0);
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission_0, [&] { return _cache.missionItemsReady(mission_0); }))
			<< "mission cache did not become ready";

	mission_item_s cached_item{};
	ASSERT_TRUE(_cache.loadMissionItem(mission_0, 1, cached_item));
	expectMissionItemMatches(cached_item, mission_items_0[1]);
	MissionRouteCache::MissionView view_0{};
	ASSERT_TRUE(_cache.getMissionView(mission_0, view_0));

	// Same mission_id and count, but now stored in dataman id 1.
	const mission_s mission_1 = makeMission(31, static_cast<uint16_t>(mission_items_1.size()), 0, -1,
						DM_KEY_WAYPOINTS_OFFBOARD_1);
	writeMissionItems(mission_items_1, DM_KEY_WAYPOINTS_OFFBOARD_1);
	_cache.update(mission_1);

	// A replacement source immediately hides the old generation while the new one loads.
	EXPECT_FALSE(_cache.missionItemsReady(mission_0));
	EXPECT_EQ(_cache.missionCount(), 0);
	MissionRouteCache::MissionView unavailable_view{};
	EXPECT_FALSE(_cache.getMissionView(mission_0, unavailable_view));
	EXPECT_FALSE(_cache.getMissionView(mission_1, unavailable_view));

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission_1, [&] { return _cache.missionItemsReady(mission_1); }))
			<< "mission cache did not reload after dataman id change";

	MissionRouteCache::MissionView view_1{};
	ASSERT_TRUE(_cache.getMissionView(mission_1, view_1));
	EXPECT_NE(view_1.generation, view_0.generation);
	EXPECT_EQ(view_1.dataman_id, mission_1.mission_dataman_id);

	ASSERT_TRUE(_cache.loadMissionItem(mission_1, 1, cached_item));
	expectMissionItemMatches(cached_item, mission_items_1[1]);
}

// A source replacement drains an older read before writing the shared typed buffer.
TEST_F(MissionRouteCacheTest, MissionSourceChangeDuringReadDoesNotPublishOldGeneration)
{
	const std::vector<mission_item_s> mission_items_a{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 10.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
	};
	const std::vector<mission_item_s> mission_items_b{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt + 30.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt + 40.f),
	};
	const mission_s mission_a = makeMission(32, static_cast<uint16_t>(mission_items_a.size()));
	const mission_s mission_b = makeMission(33, static_cast<uint16_t>(mission_items_b.size()));
	writeMissionItems(mission_items_a);

	_cache.update(mission_a);
	ASSERT_TRUE(MissionRouteCacheTestPeer::missionLoadInProgress(_cache));
	ASSERT_NE(_cache.fullMissionResponseSubscription(), ORB_SUB_INVALID);
	EXPECT_EQ(MissionRouteCacheTestPeer::missionLoadPerfEventCount(_cache), 0U);

	// Replace the contents without changing the Dataman bank or item indices.
	writeMissionItems(mission_items_b);
	_cache.update(mission_b);
	// The old request is either still pending or was consumed and replaced in this
	// call. In both cases the replacement load remains response-driven.
	EXPECT_NE(_cache.fullMissionResponseSubscription(), ORB_SUB_INVALID);

	MissionRouteCache::MissionView view{};
	EXPECT_FALSE(_cache.getMissionView(mission_a, view));
	EXPECT_FALSE(_cache.getMissionView(mission_b, view));

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission_b, [&] { return _cache.missionItemsReady(mission_b); }));
	EXPECT_EQ(_cache.fullMissionResponseSubscription(), ORB_SUB_INVALID);
	ASSERT_TRUE(_cache.getMissionView(mission_b, view));
	ASSERT_EQ(view.count, static_cast<int32_t>(mission_items_b.size()));
	EXPECT_EQ(MissionRouteCacheTestPeer::missionLoadPerfEventCount(_cache), 1U);

	for (size_t i = 0; i < mission_items_b.size(); ++i) {
		expectMissionItemMatches(view.items[i], mission_items_b[i]);
	}
}

// A nonempty mission never casts an invalid published storage key to dm_item_t.
TEST_F(MissionRouteCacheTest, MissionCacheRejectsInvalidDatamanId)
{
	mission_s mission = makeMission(32, 1);
	mission.mission_dataman_id = DM_KEY_SAFE_POINTS_0;

	_cache.update(mission);

	EXPECT_FALSE(_cache.missionItemsReady(mission));
	EXPECT_EQ(_cache.missionCount(), 0);

	mission_item_s cached_item{};
	cached_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
	EXPECT_FALSE(_cache.loadMissionItem(mission, 0, cached_item));
	EXPECT_EQ(cached_item.nav_cmd, NAV_CMD_LOITER_UNLIMITED);
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

// A successful write patches a ready cache without starting another load.
TEST_F(MissionRouteCacheTest, SyncMissionItemPatchesReadyMissionCache)
{
	const std::vector<mission_item_s> mission_items{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt + 10.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 20.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	const mission_s mission = makeMission(40, static_cast<uint16_t>(mission_items.size()));
	writeMissionItems(mission_items);
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.missionItemsReady(mission); }))
			<< "mission cache did not become ready";
	MissionRouteCache::MissionView before_sync{};
	ASSERT_TRUE(_cache.getMissionView(mission, before_sync));
	ASSERT_EQ(_cache.fullMissionResponseSubscription(), ORB_SUB_INVALID);

	const mission_item_s updated = makePositionItemFromOffset(kBaseLat, kBaseLon, 150.f, 25.f, kAlt + 33.f);
	writeMissionItem(updated, 1);
	ASSERT_EQ(_cache.syncMissionItem(mission, 1, updated), MissionRouteCache::SyncResult::kPatched);
	EXPECT_TRUE(_cache.missionItemsReady(mission));
	EXPECT_EQ(_cache.fullMissionResponseSubscription(), ORB_SUB_INVALID);
	EXPECT_FALSE(_cache.missionViewStillValid(before_sync));

	MissionRouteCache::MissionView after_sync{};
	ASSERT_TRUE(_cache.getMissionView(mission, after_sync));
	EXPECT_NE(after_sync.generation, before_sync.generation);
	expectMissionItemMatches(after_sync.items[1], updated);

	mission_item_s cached_item{};
	ASSERT_TRUE(_cache.loadMissionItem(mission, 1, cached_item));
	expectMissionItemMatches(cached_item, updated);
}

// An authoritative write during an in-flight read rebuilds the generation instead of publishing stale data.
TEST_F(MissionRouteCacheTest, SyncMissionItemRereadsInFlightMissionItem)
{
	const std::vector<mission_item_s> mission_items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 20.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	const mission_s mission = makeMission(45, static_cast<uint16_t>(mission_items.size()));
	writeMissionItems(mission_items);
	_cache.update(mission);
	ASSERT_TRUE(MissionRouteCacheTestPeer::missionLoadInProgress(_cache));

	// Complete the authoritative write before the cache consumes its old read response.
	const mission_item_s updated = makePositionItemFromOffset(kBaseLat, kBaseLon, 150.f, 25.f, kAlt + 33.f);
	writeMissionItem(updated, 0);

	EXPECT_EQ(_cache.syncMissionItem(mission, 0, updated), MissionRouteCache::SyncResult::kDeferred);
	EXPECT_FALSE(_cache.missionItemsReady(mission));
	EXPECT_EQ(_cache.missionCount(), 0);

	// The in-flight response is discarded, so the load front stays on the synced index.
	EXPECT_EQ(MissionRouteCacheTestPeer::missionNextIndex(_cache), 0);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.missionItemsReady(mission); }))
			<< "mission cache did not reload after an in-flight item update";

	mission_item_s cached_item{};
	ASSERT_TRUE(_cache.loadMissionItem(mission, 0, cached_item));
	expectMissionItemMatches(cached_item, updated);
}

// A write past the load front keeps the already loaded items and still lands in the cache.
TEST_F(MissionRouteCacheTest, SyncMissionItemAheadOfLoadFrontKeepsProgress)
{
	std::vector<mission_item_s> mission_items;

	for (int i = 0; i < 6; ++i) {
		mission_items.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f * i, 0.f, kAlt + 10.f));
	}

	const int32_t sync_index = 5;
	const mission_s mission = makeMission(48, static_cast<uint16_t>(mission_items.size()));
	writeMissionItems(mission_items);

	// Load a prefix, leaving a read in flight below the index we sync.
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] {
		return MissionRouteCacheTestPeer::missionNextIndex(_cache) >= 2
		&& MissionRouteCacheTestPeer::missionLoadInProgress(_cache);
	}));
	const int32_t load_front = MissionRouteCacheTestPeer::missionNextIndex(_cache);
	ASSERT_LT(load_front, sync_index) << "the synced index must still be unread";

	const mission_item_s updated = makePositionItemFromOffset(kBaseLat, kBaseLon, 999.f, 42.f, kAlt + 77.f);
	writeMissionItem(updated, sync_index);
	EXPECT_EQ(_cache.syncMissionItem(mission, sync_index, updated), MissionRouteCache::SyncResult::kDeferred);

	// No progress was dropped: the loaded prefix is not re-read.
	EXPECT_EQ(MissionRouteCacheTestPeer::missionNextIndex(_cache), load_front);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.missionItemsReady(mission); }));

	// The sequential load picked the new item up from dataman on its own.
	mission_item_s cached_item{};
	ASSERT_TRUE(_cache.loadMissionItem(mission, sync_index, cached_item));
	expectMissionItemMatches(cached_item, updated);
	ASSERT_TRUE(_cache.loadMissionItem(mission, 0, cached_item));
	expectMissionItemMatches(cached_item, mission_items[0]);
}

// syncMissionItem() is not a substitute for the dataman write: an index the load has not
// reached is always filled from storage, so syncing one that was never written is ignored.
TEST_F(MissionRouteCacheTest, SyncMissionItemWithoutDatamanWriteKeepsStoredItem)
{
	std::vector<mission_item_s> mission_items;

	for (int i = 0; i < 6; ++i) {
		mission_items.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f * i, 0.f, kAlt + 10.f));
	}

	const int32_t sync_index = 5;
	const mission_s mission = makeMission(49, static_cast<uint16_t>(mission_items.size()));
	writeMissionItems(mission_items);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] {
		return MissionRouteCacheTestPeer::missionNextIndex(_cache) >= 2
		&& MissionRouteCacheTestPeer::missionLoadInProgress(_cache);
	}));
	ASSERT_LT(MissionRouteCacheTestPeer::missionNextIndex(_cache), sync_index);

	// Sync an unread index whose new value never reached dataman.
	const mission_item_s never_written = makePositionItemFromOffset(kBaseLat, kBaseLon, 999.f, 42.f, kAlt + 77.f);
	EXPECT_EQ(_cache.syncMissionItem(mission, sync_index, never_written), MissionRouteCache::SyncResult::kDeferred);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.missionItemsReady(mission); }));

	// Storage wins for unread indices, unlike the loaded ones patched in place above.
	mission_item_s cached_item{};
	ASSERT_TRUE(_cache.loadMissionItem(mission, sync_index, cached_item));
	expectMissionItemMatches(cached_item, mission_items[sync_index]);
}

// A sync during the retry backoff patches the loaded prefix in place; the retry resumes at the failed index.
TEST_F(MissionRouteCacheTest, SyncMissionItemDuringRetryBackoffPatchesLoadedPrefix)
{
	const std::vector<mission_item_s> mission_items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 10.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
	};
	const mission_s mission = makeMission(50, static_cast<uint16_t>(mission_items.size()));
	writeMissionItems(mission_items);

	ASSERT_TRUE(enterRetryBackoffAfterFirstItem(mission));

	const mission_item_s updated = makePositionItemFromOffset(kBaseLat, kBaseLon, 150.f, 25.f, kAlt + 33.f);
	writeMissionItem(updated, 0);
	EXPECT_EQ(_cache.syncMissionItem(mission, 0, updated), MissionRouteCache::SyncResult::kPatched);

	// The retry resumes at the failed index.
	MissionRouteCacheTestPeer::expireMissionRetryBackoff(_cache);
	_cache.update(mission);
	EXPECT_EQ(MissionRouteCacheTestPeer::missionNextIndex(_cache), 1);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.missionItemsReady(mission); }));

	mission_item_s cached_item{};
	ASSERT_TRUE(_cache.loadMissionItem(mission, 0, cached_item));
	expectMissionItemMatches(cached_item, updated);
	ASSERT_TRUE(_cache.loadMissionItem(mission, 1, cached_item));
	expectMissionItemMatches(cached_item, mission_items[1]);
}

// Patching the land item keeps both the mission cache and the dedicated land cache coherent.
TEST_F(MissionRouteCacheTest, SyncMissionItemUpdatesMissionLandCache)
{
	const std::vector<mission_item_s> mission_items{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt + 10.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 20.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	const int32_t land_index = 2;
	const mission_s mission = makeMission(41, static_cast<uint16_t>(mission_items.size()), 0, land_index);
	writeMissionItems(mission_items);
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] {
		int32_t ready_index = -1;
		mission_item_s ready_land_item{};
		return _cache.missionItemsReady(mission) && _cache.getMissionLandItem(ready_index, ready_land_item);
	}))
			<< "mission land item did not become ready";

	const mission_item_s updated_land = makeLandItemFromOffset(kBaseLat, kBaseLon, 222.f, 11.f, kAlt + 1.f);
	writeMissionItem(updated_land, land_index);
	ASSERT_EQ(_cache.syncMissionItem(mission, land_index, updated_land), MissionRouteCache::SyncResult::kPatched);

	int32_t out_index = -1;
	mission_item_s land_item{};
	ASSERT_TRUE(_cache.getMissionLandItem(out_index, land_item));
	EXPECT_EQ(out_index, land_index);
	expectMissionItemMatches(land_item, updated_land);

	// The mission cache reflects the same patch.
	mission_item_s cached_item{};
	ASSERT_TRUE(_cache.loadMissionItem(mission, land_index, cached_item));
	expectMissionItemMatches(cached_item, updated_land);
}

// Changing the published land item to a non-land command makes the dedicated land cache unavailable.
TEST_F(MissionRouteCacheTest, SyncMissionItemInvalidatesMissionLandCacheForNonLandCommand)
{
	const std::vector<mission_item_s> mission_items{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt + 10.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 20.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	const int32_t land_index = 2;
	const mission_s mission = makeMission(43, static_cast<uint16_t>(mission_items.size()), 0, land_index);
	writeMissionItems(mission_items);
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] {
		int32_t ready_index = -1;
		mission_item_s ready_land_item{};
		return _cache.missionItemsReady(mission) && _cache.getMissionLandItem(ready_index, ready_land_item);
	}))
			<< "mission land item did not become ready";

	const mission_item_s updated_item = makePositionItemFromOffset(kBaseLat, kBaseLon, 222.f, 11.f, kAlt + 1.f);
	writeMissionItem(updated_item, land_index);
	ASSERT_EQ(_cache.syncMissionItem(mission, land_index, updated_item), MissionRouteCache::SyncResult::kPatched);

	EXPECT_FALSE(_cache.missionLandItemReady());
	EXPECT_TRUE(_cache.missionLandItemUpdatePending());

	// Failed reads leave output parameters untouched.
	int32_t out_index = 123;
	mission_item_s land_item{};
	EXPECT_FALSE(_cache.getMissionLandItem(out_index, land_item));
	EXPECT_EQ(out_index, 123);

	mission_item_s cached_item{};
	ASSERT_TRUE(_cache.loadMissionItem(mission, land_index, cached_item));
	expectMissionItemMatches(cached_item, updated_item);
}

// Syncing the land index while its dedicated cache is pending must discard the stale pending read.
TEST_F(MissionRouteCacheTest, SyncMissionItemInvalidatesPendingMissionLandCacheForNonLandCommand)
{
	const std::vector<mission_item_s> mission_items{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt + 10.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 20.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	const int32_t land_index = 2;
	const mission_s mission_without_land = makeMission(44, static_cast<uint16_t>(mission_items.size()));
	writeMissionItems(mission_items);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission_without_land,
			[&] { return _cache.missionItemsReady(mission_without_land); }))
			<< "mission cache did not become ready";

	mission_s mission = mission_without_land;
	mission.land_index = land_index;
	_cache.update(mission);
	ASSERT_TRUE(_cache.missionItemsReady(mission));
	ASSERT_TRUE(_cache.missionLandItemUpdatePending());
	ASSERT_FALSE(_cache.missionLandItemReady());

	const mission_item_s updated_item = makePositionItemFromOffset(kBaseLat, kBaseLon, 222.f, 11.f, kAlt + 1.f);
	writeMissionItem(updated_item, land_index);
	ASSERT_EQ(_cache.syncMissionItem(mission, land_index, updated_item), MissionRouteCache::SyncResult::kPatched);

	const uint8_t retry_count = MissionRouteCacheTestPeer::missionLandRetryCount(_cache);
	ASSERT_GT(retry_count, 0U);
	MissionRouteCacheTestPeer::expireMissionLandRetryBackoff(_cache);
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission,
			[&] { return MissionRouteCacheTestPeer::missionLandRetryCount(_cache) > retry_count; }))
			<< "mission land cache did not reject the replacement non-land item";

	EXPECT_FALSE(_cache.missionLandItemReady());
	EXPECT_TRUE(_cache.missionLandItemUpdatePending());

	// Failed reads leave output parameters untouched.
	int32_t out_index = 123;
	mission_item_s land_item{};
	EXPECT_FALSE(_cache.getMissionLandItem(out_index, land_item));
	EXPECT_EQ(out_index, 123);

	mission_item_s cached_item{};
	ASSERT_TRUE(_cache.loadMissionItem(mission, land_index, cached_item));
	expectMissionItemMatches(cached_item, updated_item);
}

// syncMissionItem only patches the active mission it was loaded for.
TEST_F(MissionRouteCacheTest, SyncMissionItemRejectsInactiveMission)
{
	const std::vector<mission_item_s> mission_items{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt + 10.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
	};
	const mission_s mission = makeMission(42, static_cast<uint16_t>(mission_items.size()));
	writeMissionItems(mission_items);
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.missionItemsReady(mission); }))
			<< "mission cache did not become ready";

	// A mission that does not match the cached one must be rejected.
	mission_s other_mission = mission;
	other_mission.mission_id = 999;
	const mission_item_s updated = makePositionItemFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 5.f);
	EXPECT_EQ(_cache.syncMissionItem(other_mission, 0, updated), MissionRouteCache::SyncResult::kRejected);

	// The original cached item is untouched.
	mission_item_s cached_item{};
	ASSERT_TRUE(_cache.loadMissionItem(mission, 0, cached_item));
	expectMissionItemMatches(cached_item, mission_items[0]);
}

// The land item cache is kept coherent by syncMissionItem() even when the mission
// cache cannot hold the mission. E.g. boards where CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE = 0.
TEST_F(MissionRouteCacheTest, SyncMissionItemMaintainsLandCacheWithoutMissionCache)
{
	const int32_t land_index = 2;
	const uint16_t count = static_cast<uint16_t>(MissionRouteCache::kMaxFullMissionCacheSize + 1);
	const mission_s mission = makeMission(46, count, 0, land_index);
	writeMissionItem(makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt), land_index);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] {
		int32_t ready_index = -1;
		mission_item_s ready_land_item{};
		return _cache.getMissionLandItem(ready_index, ready_land_item);
	})) << "land item did not become ready for an oversized mission";
	EXPECT_FALSE(_cache.missionItemsReady(mission));

	// The mission cache rejects the write, the land cache still applies it.
	const mission_item_s updated_land = makeLandItemFromOffset(kBaseLat, kBaseLon, 222.f, 11.f, kAlt + 1.f);
	writeMissionItem(updated_land, land_index);
	EXPECT_EQ(_cache.syncMissionItem(mission, land_index, updated_land), MissionRouteCache::SyncResult::kRejected);

	int32_t out_index = -1;
	mission_item_s land_item{};
	ASSERT_TRUE(_cache.getMissionLandItem(out_index, land_item));
	EXPECT_EQ(out_index, land_index);
	expectMissionItemMatches(land_item, updated_land);

	// A non-land replacement invalidates the land cache through the same path.
	const mission_item_s non_land = makePositionItemFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 5.f);
	writeMissionItem(non_land, land_index);
	EXPECT_EQ(_cache.syncMissionItem(mission, land_index, non_land), MissionRouteCache::SyncResult::kRejected);
	EXPECT_FALSE(_cache.missionLandItemReady());
	EXPECT_TRUE(_cache.missionLandItemUpdatePending());
}

// missionViewStillValid() pins a borrowed view to its generation.
TEST_F(MissionRouteCacheTest, MissionViewStillValidTracksGeneration)
{
	const std::vector<mission_item_s> mission_items{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt + 10.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
	};
	const mission_s mission = makeMission(46, static_cast<uint16_t>(mission_items.size()));
	writeMissionItems(mission_items);
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(_cache, mission, [&] { return _cache.missionItemsReady(mission); }))
			<< "mission cache did not become ready";

	// A default view has never been published.
	EXPECT_FALSE(_cache.missionViewStillValid(MissionRouteCache::MissionView{}));

	MissionRouteCache::MissionView view{};
	ASSERT_TRUE(_cache.getMissionView(mission, view));
	EXPECT_TRUE(_cache.missionViewStillValid(view));

	// An authoritative in-place patch advances the generation and stales older views.
	const mission_item_s updated = makePositionItemFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 5.f);
	writeMissionItem(updated, 0);
	ASSERT_EQ(_cache.syncMissionItem(mission, 0, updated), MissionRouteCache::SyncResult::kPatched);
	EXPECT_FALSE(_cache.missionViewStillValid(view));

	MissionRouteCache::MissionView refreshed{};
	ASSERT_TRUE(_cache.getMissionView(mission, refreshed));
	EXPECT_TRUE(_cache.missionViewStillValid(refreshed));

	_cache.invalidate();
	EXPECT_FALSE(_cache.missionViewStillValid(refreshed));
}

// Each Dataman completion wakes the cache and immediately chains the next async read.
TEST_F(MissionRouteCacheTest, FullMissionCachePollsPendingResponses)
{
	static constexpr int kDatamanResponsePollTimeoutMs{5000};

	const std::vector<mission_item_s> mission_items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt + 5.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 20.f, 0.f, kAlt + 6.f),
	};

	const mission_s mission = makeMission(47, static_cast<uint16_t>(mission_items.size()));
	writeMissionItems(mission_items);
	FullMissionCache full_cache;

	EXPECT_EQ(full_cache.responseSubscription(), ORB_SUB_INVALID);
	full_cache.update(mission);
	ASSERT_NE(full_cache.responseSubscription(), ORB_SUB_INVALID);
	EXPECT_FALSE(full_cache.missionItemsReady(mission));

	px4_pollfd_struct_t response_fd{};
	response_fd.fd = full_cache.responseSubscription();
	response_fd.events = POLLIN;
	ASSERT_GT(px4_poll(&response_fd, 1, kDatamanResponsePollTimeoutMs), 0);
	ASSERT_NE(response_fd.revents & POLLIN, 0);

	// Consuming item zero queues item one in this same call, keeping the fd armed.
	full_cache.update(mission);
	ASSERT_NE(full_cache.responseSubscription(), ORB_SUB_INVALID);
	EXPECT_FALSE(full_cache.missionItemsReady(mission));

	response_fd = {};
	response_fd.fd = full_cache.responseSubscription();
	response_fd.events = POLLIN;
	ASSERT_GT(px4_poll(&response_fd, 1, kDatamanResponsePollTimeoutMs), 0);
	ASSERT_NE(response_fd.revents & POLLIN, 0);
	full_cache.update(mission);

	ASSERT_TRUE(full_cache.missionItemsReady(mission));
	EXPECT_EQ(full_cache.responseSubscription(), ORB_SUB_INVALID);
	EXPECT_EQ(full_cache.missionCount(), static_cast<int>(mission_items.size()));

	mission_item_s cached_item{};
	ASSERT_TRUE(full_cache.loadMissionItem(0, cached_item));
	expectMissionItemMatches(cached_item, mission_items.front());
	ASSERT_TRUE(full_cache.loadMissionItem(static_cast<int32_t>(mission_items.size()) - 1, cached_item));
	expectMissionItemMatches(cached_item, mission_items.back());
}
