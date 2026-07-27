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
 * Navigator-owned mission-route cache.
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
	// Planner/provider reads are cache-only and must not block on dataman.
	static constexpr hrt_abstime kCacheOnlyLoadWait{0};
	static constexpr hrt_abstime kCacheRetryBackoff{500000}; // 500 ms
	static constexpr uint8_t kMaxRetryBackoffShift{3}; // Retry 3+: 500ms << 3 = 4000 ms.
	// Disarmed-only warm-up: bounded synchronous dataman reads per update() call.
	static constexpr hrt_abstime kBlockingLoadBudget{20000}; // 20 ms per update() call
	static constexpr hrt_abstime kBlockingLoadItemTimeout{50000}; // 50 ms per item read
	static constexpr int32_t kMaxRouteMissionCacheSize{CONFIG_RTL_MISSION_CACHE_SIZE};
	static constexpr uint32_t kInitialSafePointCacheSize{0};
	static constexpr uint32_t kInitialLandItemCacheSize{1};

	struct MissionView {
		const mission_item_s *items{nullptr};
		int32_t count{0};
		uint32_t mission_id{0};
		uint8_t dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_0};
		uint32_t generation{0};
	};

	/** @brief Outcome of syncMissionItem() for the full-mission cache. */
	enum class SyncResult : uint8_t {
		kRejected,  ///< The mission cache did not apply the write: source mismatch, index out of range, or cache compiled out.
		kPatched,   ///< The item was applied in place; any previously borrowed view is stale.
		kRestarted  ///< The item was not stably loaded; progress was dropped and a full reload (re)started.
	};

	// The optional mavlink log advert (Navigator's) reports a mission exceeding the cache.
	explicit MissionRouteCache(orb_advert_t *mavlink_log_pub = nullptr);
	~MissionRouteCache();
	MissionRouteCache(const MissionRouteCache &) = delete;
	MissionRouteCache &operator=(const MissionRouteCache &) = delete;

	/**
	 * @brief Advance the cache state machines for the published mission.
	 *
	 * allow_blocking_load permits short synchronous dataman reads (kBlockingLoadBudget per
	 * call) for the pending mission load. Only pass true while disarmed.
	 */
	void update(const mission_s &mission, bool allow_blocking_load = false);
	void invalidate();

	bool missionExceedsCacheLimit(const mission_s &mission) const;
	bool isReady(const mission_s &mission) const;
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
	/** True after queuing, reading, or validating the current mission land item failed. */
	bool missionLandItemAttemptFailed() const { return _mission_land.retry.retry_count > 0; }

	/**
	 * Returns the item count of the complete, validated mission generation.
	 * Returns zero while a replacement is pending and for a ready empty mission.
	 * Index-based reads carry no mission identity; use getMissionView() /
	 * missionViewStillValid() for consistency across reads or update() cycles.
	 */
	int missionCount() const;
	bool loadMissionItem(int index, mission_item_s &mission_item) const;

	/**
	 * @brief Borrow the complete typed mission view without copying its items.
	 *
	 * The returned view is immutable and valid only for the current synchronous
	 * planning invocation. It must not be retained across update(), invalidate(),
	 * source replacement, or syncMissionItem(); use missionViewStillValid() to verify.
	 *
	 * @return true for a complete view, including a ready empty mission.
	 *         On failure the view output is left unchanged.
	 */
	bool getMissionView(const mission_s &mission, MissionView &view) const;

	/**
	 * @brief True while a borrowed view still matches the cached mission.
	 *
	 * Source changes, invalidate(), reloads, and syncMissionItem() advance the generation.
	 */
	bool missionViewStillValid(const MissionView &view) const;

	/**
	 * Returns the item count of the complete, validated safe-point generation.
	 * Returns zero while a replacement is pending (check safePointsReady()) and for a ready empty set.
	 */
	int safePointCount() const override;
	bool loadSafePointItem(int index, mission_item_s &safe_point_item) const override;

	/**
	 * @brief Load the mission item referenced by the active mission's published land_index.
	 *
	 * This lookup is cache-only and never waits for Dataman. While the asynchronous load or a source replacement is
	 * pending, it returns false and leaves both output parameters unchanged.
	 */
	bool getMissionLandItem(int32_t &index, mission_item_s &land_item) const;

	bool loadMissionItem(const mission_s &mission, int32_t index, mission_item_s &mission_item) const;

	/**
	 * @brief Apply an authoritative mission item write to the cached copies.
	 *
	 * The result describes the full-mission cache; kRejected is always returned when
	 * that cache is compiled out. The land-item cache is refreshed alongside on every
	 * board when index is the published land index; its failures retry asynchronously.
	 */
	SyncResult syncMissionItem(const mission_s &mission, int32_t index, const mission_item_s &mission_item);

private:
	friend class MissionRouteCacheTestPeer;

	enum class SafePointDatamanState {
		kUpdateRequestWait,
		kRead,
		kReadWait,
		kLoad,
		kError
	};

	// Shared retry/backoff bookkeeping for the mission, land, and safe-point caches.
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

#if CONFIG_RTL_MISSION_CACHE_SIZE > 0
	struct MissionCacheState {
		uint32_t id{0};
		int32_t count{0};
		uint8_t dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_0};
		int32_t next_index{0}; ///< Sequential load front; items below it are loaded.
		bool initialized{false};
		bool source_valid{false};
		bool ready{false};
		bool too_large{false};
		bool validation_pending{false};
		RetryBackoff retry{};
	};
#endif

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
		mission_stats_entry_s read_stats{};   ///< Async DM_KEY_SAFE_POINTS_STATE read buffer.
		mission_stats_entry_s cached_stats{}; ///< Identity of the safe-point set currently in the RAM cache.
		uint32_t source_id{0};                ///< Active mission.safe_points_id being tracked.
		uint8_t source_dataman_id{DM_KEY_SAFE_POINTS_0}; ///< Active mission.safepoint_dataman_id being tracked.
		uint32_t cached_source_id{0};         ///< mission.safe_points_id the RAM cache was built for.
		bool ready{false};
		bool update_requested{true};
		RetryBackoff retry{};
	};

#if CONFIG_RTL_MISSION_CACHE_SIZE > 0
	void updateMissionCache(const mission_s &mission, bool allow_blocking_load);
	bool missionMatchesCache(const mission_s &mission) const;
	bool missionCacheAvailable() const;
	void startMissionLoadOrRetry(const mission_s &mission, hrt_abstime now);
	bool blockingLoadMissionItems(MissionCacheState &state);
	void advanceMissionGeneration();
#endif
	void updateMissionLandItemCache(const mission_s &mission);
	void syncMissionLandItem(const mission_s &mission, int32_t index, const mission_item_s &mission_item);
	bool missionLandMatchesCache(const mission_s &mission) const;
	bool queueMissionLandItem();
	void updateSafePointCache(const mission_s &mission);
	bool missionLandItemCacheFullyLoaded() const;
	bool safePointCacheFullyLoaded() const;
	bool safePointCacheMatchesReadStats() const;
	void publishSafePointCache();
	void resetSafePointCacheState(bool clear_source_identity);

	// Navigator accesses MissionRouteCache from one serialized task loop. The
	// typed mission view can therefore be borrowed for one synchronous planning call.
#if CONFIG_RTL_MISSION_CACHE_SIZE > 0
	mission_item_s *_mission_items {nullptr};
	DatamanClient _dataman_client_mission{};
	orb_advert_t *_mavlink_log_pub{nullptr};
	uint32_t _mission_generation{0};
	uint32_t _mission_request_generation{0};
	int32_t _mission_request_index{-1};
	bool _mission_request_pending{false};
#endif
	mutable DatamanCache _dataman_cache_safepoint {"navigator_dm_cache_route_safepoint", kInitialSafePointCacheSize};
	mutable DatamanCache _dataman_cache_land_item{"navigator_dm_cache_route_land", kInitialLandItemCacheSize};
	DatamanClient &_dataman_client_safepoint = _dataman_cache_safepoint.client();

#if CONFIG_RTL_MISSION_CACHE_SIZE > 0
	MissionCacheState _mission {};
#endif
	MissionLandState _mission_land {};
	SafePointState _safe_point{};
};
