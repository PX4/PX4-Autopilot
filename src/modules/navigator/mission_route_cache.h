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
 * @file mission_route_cache.h
 *
 * Navigator-owned cache of the dataman-backed data RTL destination selection
 * needs: the rally (safe) points and the published mission land item.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <dataman_client/DatamanClient.hpp>
#include <uORB/topics/mission.h>

#include "navigation.h"
#include "mission_route_provider.h"

class MissionRouteCache : public mission_route::Provider
{
public:
	// Consumer reads are cache-only and must not block on dataman.
	static constexpr hrt_abstime kCacheOnlyLoadWait{0};
	static constexpr hrt_abstime kCacheRetryBackoff{500000}; // 500 ms
	static constexpr uint8_t kMaxRetryBackoffShift{3}; // Retry 3+: 500ms << 3 = 4000 ms.
	static constexpr uint32_t kInitialSafePointCacheSize{0};
	static constexpr uint32_t kInitialLandItemCacheSize{1};

	MissionRouteCache() = default;
	~MissionRouteCache() = default;
	MissionRouteCache(const MissionRouteCache &) = delete;
	MissionRouteCache &operator=(const MissionRouteCache &) = delete;

	void update(const mission_s &mission);
	void invalidate();

	bool safePointsReady() const
	{
		return _safe_point.ready
		       && !_safe_point.update_requested
		       && _safe_point.dataman_state == SafePointDatamanState::kUpdateRequestWait;
	}
	bool safePointUpdatePending() const
	{
		return _safe_point.update_requested
		       || _safe_point.dataman_state != SafePointDatamanState::kUpdateRequestWait;
	}
	uint32_t safePointsId() const { return _safe_point.source_id; }
	bool missionLandItemReady() const
	{
		return _mission_land.ready
		       && _mission_land.index >= 0
		       && _mission_land.index < _mission_land.count;
	}
	bool missionLandItemUpdatePending() const
	{
		return _mission_land.index >= 0
		       && _mission_land.index < _mission_land.count
		       && !_mission_land.ready
		       && (_mission_land.validation_pending || _mission_land.retry.retry_at != 0);
	}

	int safePointCount() const override;
	bool loadSafePointItem(int index, mission_item_s &safe_point_item) const override;

	/** @brief Load the mission item referenced by the active mission's published land_index. */
	bool getMissionLandItem(int32_t &index, mission_item_s &land_item) const;

private:
	friend class MissionRouteCacheTestPeer;

	enum class SafePointDatamanState {
		kUpdateRequestWait,
		kRead,
		kReadWait,
		kLoad,
		kError
	};

	// Shared retry/backoff bookkeeping for the land and safe-point caches.
	struct RetryBackoff {
		hrt_abstime retry_at{0};
		uint8_t retry_count{0};

		bool due(hrt_abstime now) const { return retry_at != 0 && now >= retry_at; }
		void clear()
		{
			retry_at = 0;
			retry_count = 0;
		}

		void scheduleRetry(hrt_abstime now)
		{
			const uint8_t backoff_shift = (retry_count < kMaxRetryBackoffShift) ? retry_count : kMaxRetryBackoffShift;
			retry_at = now + (kCacheRetryBackoff << backoff_shift);

			if (retry_count < UINT8_MAX) {
				++retry_count;
			}
		}
	};

	struct MissionLandState {
		uint32_t mission_id{0};
		uint8_t dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_0};
		int32_t index{-1};
		int32_t count{0}; ///< Published mission.count the index was validated against.
		bool ready{false};
		bool validation_pending{false};
		RetryBackoff retry{};
	};

	struct SafePointState {
		SafePointDatamanState dataman_state{SafePointDatamanState::kUpdateRequestWait};
		SafePointDatamanState error_state{SafePointDatamanState::kUpdateRequestWait};
		mission_stats_entry_s read_stats{};   ///< Async DM_KEY_SAFE_POINTS_STATE read buffer.
		mission_stats_entry_s cached_stats{}; ///< Identity of the safe-point set currently in the RAM cache.
		uint32_t source_id{0};                ///< Active mission.safe_points_id being tracked.
		uint32_t cached_source_id{0};         ///< mission.safe_points_id the RAM cache was built for.
		bool ready{false};
		bool update_requested{true};
		RetryBackoff retry{};
	};

	void updateMissionLandItemCache(const mission_s &mission);
	bool queueMissionLandItem();
	void updateSafePointCache(const mission_s &mission);
	bool missionLandItemCacheFullyLoaded() const;
	bool safePointCacheFullyLoaded() const;
	bool safePointCacheMatchesReadStats() const;
	void publishSafePointCache();
	void resetSafePointCacheState(bool clear_source_identity);

	// Navigator runs MissionRouteCache from one work queue. Mutable caches let
	// const reader methods serve preloaded RAM entries without blocking.
	mutable DatamanCache _dataman_cache_safepoint{"navigator_dm_cache_route_safepoint", kInitialSafePointCacheSize};
	mutable DatamanCache _dataman_cache_land_item{"navigator_dm_cache_route_land", kInitialLandItemCacheSize};
	DatamanClient &_dataman_client_safepoint = _dataman_cache_safepoint.client();

	MissionLandState _mission_land{};
	SafePointState _safe_point{};
};
