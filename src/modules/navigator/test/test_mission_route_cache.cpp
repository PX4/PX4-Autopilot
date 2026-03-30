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
 * Targeted MissionRouteCache regression tests for mission and safe-point
 * cache state handling.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include <gtest/gtest.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/time.h>

#include "mission_route_cache.h"
#include "test_RTL_helpers.h"

#include <vector>

using rtl_test_reference::kAlt;
using rtl_test_reference::kBaseLat;
using rtl_test_reference::kBaseLon;

static constexpr int kMissionRouteCacheMaxPolls = 4096;
static constexpr useconds_t kMissionRouteCachePollSleepUs = 1000;

/**
 * @brief Test class for error handling and helper behavior that is
 *        not observable through public methods.
 */
class MissionRouteCacheTestPeer
{
public:
	static bool missionRetryScheduled(const MissionRouteCache &cache)
	{
		return cache._mission.retry_at != 0;
	}

	static uint8_t missionRetryCount(const MissionRouteCache &cache)
	{
		return cache._mission.retry_count;
	}

	static bool queueMissionCacheLoads(MissionRouteCache &cache, const mission_s &mission)
	{
		return cache.queueMissionCacheLoads(mission);
	}

	static bool preparePartialMissionCache(MissionRouteCache &cache, const mission_s &mission, uint32_t cached_index)
	{
		cache._mission = {};
		cache._mission.id = mission.mission_id;
		cache._mission.count = mission.count;
		cache._mission.dataman_id = mission.mission_dataman_id;
		cache._mission.validation_pending = true;
		cache._dataman_cache_mission.invalidate();

		if (cache._dataman_cache_mission.size() != mission.count) {
			cache._dataman_cache_mission.resize(mission.count);
		}

		if (cache._dataman_cache_mission.size() != mission.count) {
			return false;
		}

		return cache._dataman_cache_mission.load(static_cast<dm_item_t>(mission.mission_dataman_id), cached_index);
	}
};

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
			      int32_t land_index = -1, dm_item_t mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0) const
	{
		mission_s mission{};
		mission.timestamp = hrt_absolute_time();
		mission.mission_id = mission_id;
		mission.count = count;
		mission.land_index = land_index;
		mission.mission_dataman_id = static_cast<uint8_t>(mission_dataman_id);
		mission.safe_points_id = safe_points_id;
		mission.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;
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

	template<typename Predicate>
	bool updateUntil(const mission_s &mission, Predicate &&predicate, int max_polls = kMissionRouteCacheMaxPolls)
	{
		for (int poll = 0; poll < max_polls; ++poll) {
			_cache.update(mission);

			if (predicate()) {
				return true;
			}

			px4_usleep(kMissionRouteCachePollSleepUs);
		}

		return false;
	}

	void updateFor(const mission_s &mission, int polls)
	{
		for (int poll = 0; poll < polls; ++poll) {
			_cache.update(mission);
			px4_usleep(kMissionRouteCachePollSleepUs);
		}
	}

	DatamanClient _dataman_client{};
	MissionRouteCache _cache{};
};

// WHY: Mission smart rejoin and route planning depend on a fully loaded mission cache.
// WHAT: A ten-item mission should become ready and every item should be readable.
TEST_F(MissionRouteCacheTest, MissionCacheLoadsAllMissionItems)
{
	// GIVEN: A ten-item mission written into dataman.
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

	// WHEN: The cache is updated until the mission becomes ready.
	ASSERT_TRUE(updateUntil(mission, [&] { return _cache.isReady(mission); }))
			<< "MissionRouteCache did not finish loading a valid mission";

	// THEN: The mission count is reported correctly and every cached item matches the source.
	ASSERT_EQ(_cache.missionCount(), static_cast<int>(mission_items.size()));

	for (size_t i = 0; i < mission_items.size(); ++i) {
		mission_item_s cached_item{};
		ASSERT_TRUE(_cache.loadMissionItem(mission, static_cast<int32_t>(i), cached_item));
		expectMissionItemMatches(cached_item, mission_items[i]);
	}
}

// WHY: Validation must reject a partially filled mission cache even after asynchronous loading stops.
// WHAT: If only part of the mission cache is populated, MissionRouteCache should stay not-ready and schedule a retry.
TEST_F(MissionRouteCacheTest, MissionCacheSchedulesRetryWhenCacheIsIncomplete)
{
	// GIVEN: A mission with two items but the cache state only queued the first item for loading.
	const mission_s mission = makeMission(18, 2);
	writeMissionItem(makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt), 0);
	ASSERT_TRUE(MissionRouteCacheTestPeer::preparePartialMissionCache(_cache, mission, 0));

	// WHEN: The cache is updated until validation runs on the incomplete mission cache.
	ASSERT_TRUE(updateUntil(mission, [&] { return MissionRouteCacheTestPeer::missionRetryScheduled(_cache); }))
			<< "MissionRouteCache did not schedule a retry after mission cache validation failed";

	// THEN: The mission stays unavailable and the retry counter advances.
	EXPECT_FALSE(_cache.isReady(mission));
	EXPECT_EQ(_cache.missionCount(), 0);
	EXPECT_GT(MissionRouteCacheTestPeer::missionRetryCount(_cache), 0U);

	mission_item_s cached_item{};
	EXPECT_FALSE(_cache.loadMissionItem(mission, 0, cached_item));
}

// WHY: Missions larger than the configured cache limit must be rejected deterministically.
// WHAT: An oversized mission should be flagged as too large and queueMissionCacheLoads should refuse to queue it.
TEST_F(MissionRouteCacheTest, MissionCacheRejectsTooLargeMission)
{
	// GIVEN: A mission that exceeds the configured mission cache size.
	const mission_s mission = makeMission(19, static_cast<uint16_t>(MissionRouteCache::MAX_ROUTE_MISSION_CACHE_SIZE + 1));

	// WHEN: The cache processes the new mission and the private queue helper is asked to queue it.
	updateFor(mission, 8);
	const bool queued = MissionRouteCacheTestPeer::queueMissionCacheLoads(_cache, mission);

	// THEN: The mission is rejected from caching and never becomes ready.
	EXPECT_TRUE(_cache.missionExceedsCacheLimit(mission));
	EXPECT_FALSE(queued);
	EXPECT_FALSE(_cache.isReady(mission));
	EXPECT_EQ(_cache.missionCount(), 0);
}

// WHY: Route fallback to the mission land item depends on the dedicated land-item cache being coherent.
// WHAT: A valid published land_index should make getMissionLandItem return the referenced landing waypoint.
TEST_F(MissionRouteCacheTest, MissionLandItemLoadsReferencedWaypoint)
{
	// GIVEN: A mission whose published land_index points to the final landing item.
	const std::vector<mission_item_s> mission_items{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f,  0.f, kAlt + 15.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 120.f, 0.f, kAlt + 30.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 240.f, 0.f, kAlt),
	};
	const int32_t land_index_expected = 2;
	const mission_s mission = makeMission(20, static_cast<uint16_t>(mission_items.size()), 0, land_index_expected);
	writeMissionItems(mission_items);

	// WHEN: The mission cache is updated until the mission is ready.
	ASSERT_TRUE(updateUntil(mission, [&] { return _cache.isReady(mission); }))
			<< "MissionRouteCache did not finish loading the mission before land item lookup";

	// THEN: The dedicated land-item cache returns the published landing waypoint and index.
	int32_t land_index = -1;
	mission_item_s land_item{};
	ASSERT_TRUE(_cache.getMissionLandItem(land_index, land_item));
	EXPECT_EQ(land_index, land_index_expected);
	expectMissionItemMatches(land_item, mission_items[land_index_expected]);
}

// WHY: MissionRouteCache intentionally trusts the published land_index instead of rescanning the mission.
// WHAT: If land_index is out of bounds, getMissionLandItem should report no land item even if one exists in the mission.
TEST_F(MissionRouteCacheTest, MissionLandItemRejectsOutOfBoundsPublishedIndex)
{
	// GIVEN: A mission that contains a landing item, but the published land_index is invalid.
	const std::vector<mission_item_s> mission_items{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f,  0.f, kAlt + 15.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 120.f, 0.f, kAlt + 30.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 240.f, 0.f, kAlt),
	};
	const mission_s mission = makeMission(21, static_cast<uint16_t>(mission_items.size()), 0,
					      static_cast<int32_t>(mission_items.size()));
	writeMissionItems(mission_items);

	// WHEN: The mission cache is updated until the mission itself is ready.
	ASSERT_TRUE(updateUntil(mission, [&] { return _cache.isReady(mission); }))
			<< "MissionRouteCache did not finish loading the mission with an invalid published land index";

	// THEN: The mission stays usable, but no mission land item is exposed.
	int32_t land_index = -1;
	mission_item_s land_item{};
	EXPECT_FALSE(_cache.getMissionLandItem(land_index, land_item));
}

// WHY: A transient safe-point state read error must not permanently disable safe-point loading.
// WHAT: After an invalid safe-point state read, the cache should retry and eventually load the same safe_points_id.
TEST_F(MissionRouteCacheTest, SafePointCacheRetriesAfterInvalidStateWithoutIdChange)
{
	// GIVEN: A safe-point state entry that is temporarily invalid for the current safe_points_id.
	const mission_s mission = makeMission(0, 0, 41);
	writeSafePointState(DM_KEY_SAFE_POINTS_MAX + 1, 0);
	updateFor(mission, 256);
	EXPECT_FALSE(_cache.safePointsReady());

	const std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 20.f, 5.f, kAlt),
	};
	writeSafePointItems(safe_points, static_cast<uint16_t>(safe_points.size()), 0);

	// WHEN: The valid safe-point state replaces the bad one and the cache keeps updating.
	ASSERT_TRUE(updateUntil(mission, [&] { return _cache.safePointsReady(); }))
			<< "MissionRouteCache did not retry the safe-point state read after a transient error";

	// THEN: The safe point becomes available.
	EXPECT_EQ(_cache.safePointCount(), 1);
	mission_item_s safe_point{};
	ASSERT_TRUE(_cache.loadSafePointItem(0, safe_point));
	expectMissionItemMatches(safe_point, safe_points[0]);
}

// WHY: A new safe_points_id must force a reload even when the source reuses the same opaque_id.
// WHAT: Changing safe_points_id with an unchanged opaque_id should replace the cached safe-point contents.
TEST_F(MissionRouteCacheTest, SafePointIdChangeReloadsWhenOpaqueIdStaysTheSame)
{
	// GIVEN: Two different safe-point sets that reuse the same opaque_id.
	const std::vector<mission_item_s> safe_points_a{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt),
	};

	const std::vector<mission_item_s> safe_points_b{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 120.f, 0.f, kAlt),
	};

	mission_s mission = makeMission(0, 0, 100);
	writeSafePointItems(safe_points_a, static_cast<uint16_t>(safe_points_a.size()), 77);

	// WHEN: The first set loads, then safe_points_id changes to a new set with the same opaque_id.
	ASSERT_TRUE(updateUntil(mission, [&] { return _cache.safePointsReady(); }))
			<< "Initial safe-point cache load did not complete";

	mission_item_s safe_point{};
	ASSERT_TRUE(_cache.loadSafePointItem(0, safe_point));
	expectMissionItemMatches(safe_point, safe_points_a[0]);

	writeSafePointItems(safe_points_b, static_cast<uint16_t>(safe_points_b.size()), 77);
	mission.safe_points_id = 101;
	mission.timestamp = hrt_absolute_time();

	ASSERT_TRUE(updateUntil(mission, [&] { return _cache.safePointsReady(); }))
			<< "Safe-point cache did not reload after safe_points_id changed";

	// THEN: The cache exposes the new safe-point set instead of the stale one.
	ASSERT_TRUE(_cache.loadSafePointItem(0, safe_point));
	expectMissionItemMatches(safe_point, safe_points_b[0]);
}

// WHY: A source change during an in-flight safe-point load must discard stale work and publish only the new source.
// WHAT: If safe_points_id changes mid-load, the final cache contents should come only from the new safe-point set.
TEST_F(MissionRouteCacheTest, SafePointSourceChangeDuringLoadDoesNotExposeStaleData)
{
	// GIVEN: One large safe-point set that is still loading, followed by a second replacement set.
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
	writeSafePointItems(safe_points_a, static_cast<uint16_t>(safe_points_a.size()), 11);

	// WHEN: The source changes before the first set is fully loaded.
	updateFor(mission_a, 8);
	ASSERT_FALSE(_cache.safePointsReady());

	mission_s mission_b = makeMission(0, 0, 201);
	writeSafePointItems(safe_points_b, static_cast<uint16_t>(safe_points_b.size()), 12);

	ASSERT_TRUE(updateUntil(mission_b, [&] { return _cache.safePointsReady(); }))
			<< "Safe-point cache did not finish reloading after the source changed mid-update";

	// THEN: Only the replacement safe-point set is visible.
	mission_item_s safe_point{};
	ASSERT_TRUE(_cache.loadSafePointItem(0, safe_point));
	expectMissionItemMatches(safe_point, safe_points_b[0]);
	EXPECT_EQ(_cache.safePointCount(), 1);
}
