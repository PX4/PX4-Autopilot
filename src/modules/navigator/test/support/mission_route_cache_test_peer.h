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
 * @file mission_route_cache_test_peer.h
 *
 * Helper for MissionRouteCache tests.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <drivers/drv_hrt.h>

#include "mission_route_cache.h"
#include "navigator_dataman_test.h"

class MissionRouteCacheTestPeer
{
public:
	static constexpr hrt_abstime kDefaultCacheTestTimeoutUs{5'000'000}; // 5 s

	static bool missionRetryScheduled(const MissionRouteCache &cache)
	{
		return cache._mission.retry.retry_at != 0;
	}

	static uint8_t missionRetryCount(const MissionRouteCache &cache)
	{
		return cache._mission.retry.retry_count;
	}

	static bool missionLandRetryScheduled(const MissionRouteCache &cache)
	{
		return cache._mission_land.retry.retry_at != 0;
	}

	static uint8_t missionLandRetryCount(const MissionRouteCache &cache)
	{
		return cache._mission_land.retry.retry_count;
	}

	static bool safePointRetryScheduled(const MissionRouteCache &cache)
	{
		return cache._safe_point.retry.retry_at != 0;
	}

	static bool safePointLoadInProgress(const MissionRouteCache &cache)
	{
		return cache._safe_point.dataman_state == MissionRouteCache::SafePointDatamanState::kLoad
		       && cache._dataman_cache_safepoint.isLoading();
	}

	static bool preparePartialMissionCache(MissionRouteCache &cache, const mission_s &mission, uint32_t cached_index)
	{
		cache._mission = {};
		cache._mission.id = mission.mission_id;
		cache._mission.count = mission.count;
		cache._mission.dataman_id = mission.mission_dataman_id;
		cache._mission.validation_pending = true;
		cache._dataman_cache_mission.invalidate();

		if (static_cast<int32_t>(cache._dataman_cache_mission.size()) < mission.count) {
			return false;
		}

		return cache._dataman_cache_mission.load(static_cast<dm_item_t>(mission.mission_dataman_id), cached_index);
	}

	// Force safe-point read without changing safe_points_id, simulating a re-validation cycle.
	static void requestSafePointRecheck(MissionRouteCache &cache)
	{
		cache._safe_point.update_requested = true;
		cache._safe_point.dataman_state = MissionRouteCache::SafePointDatamanState::kUpdateRequestWait;
		cache._safe_point.retry.retry_at = 0;
	}

	static void expireSafePointRetryBackoff(MissionRouteCache &cache)
	{
		cache._safe_point.retry.retry_at = 0;
	}

	// Step the cache forward, draining async dataman work, until the predicate holds or it times out.
	template<typename Predicate>
	static bool runCacheUntil(MissionRouteCache &cache, const mission_s &mission, Predicate predicate,
				  hrt_abstime timeout = kDefaultCacheTestTimeoutUs)
	{
		const hrt_abstime start_time = hrt_absolute_time();

		while (true) {
			cache.update(mission);

			if (predicate()) {
				return true;
			}

			const hrt_abstime elapsed = hrt_elapsed_time(&start_time);

			if (elapsed >= timeout) {
				break;
			}

			if (!progressOneEvent(cache, mission, timeout - elapsed)) {
				break;
			}
		}

		cache.update(mission);
		return predicate();
	}

private:
	// Advance the next pending async step.
	// Returns false when nothing can progress without waiting on a scheduled retry backoff.
	static bool progressOneEvent(MissionRouteCache &cache, const mission_s &mission, hrt_abstime timeout)
	{
		if (cache._mission.validation_pending && cache._dataman_cache_mission.isLoading()) {
			return DatamanCacheTestPeer::processNextBlocking(cache._dataman_cache_mission, timeout);
		}

		if (cache._mission_land.index >= 0 && cache._dataman_cache_land_item.isLoading()) {
			return DatamanCacheTestPeer::processNextBlocking(cache._dataman_cache_land_item, timeout);
		}

		if (cache._safe_point.dataman_state == MissionRouteCache::SafePointDatamanState::kUpdateRequestWait
		    && cache._safe_point.update_requested
		    && cache._safe_point.retry.retry_at == 0) {
			cache.update(mission);
			return true;
		}

		if (cache._safe_point.dataman_state == MissionRouteCache::SafePointDatamanState::kRead) {
			cache.update(mission);
			return true;
		}

		if (cache._safe_point.dataman_state == MissionRouteCache::SafePointDatamanState::kReadWait) {
			if (!DatamanClientTestPeer::waitForOperation(cache._dataman_client_safepoint, timeout)) {
				return false;
			}

			cache.update(mission);
			return true;
		}

		if (cache._safe_point.dataman_state == MissionRouteCache::SafePointDatamanState::kLoad
		    && cache._dataman_cache_safepoint.isLoading()) {
			return DatamanCacheTestPeer::processNextBlocking(cache._dataman_cache_safepoint, timeout);
		}

		return false;
	}
};
