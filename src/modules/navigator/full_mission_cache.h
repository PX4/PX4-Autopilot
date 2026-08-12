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
 * @file full_mission_cache.h
 *
 * Optional RAM cache for a complete mission route.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <dataman_client/DatamanClient.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/topics/mission.h>

#include "navigation.h"

class MissionRouteCacheTestPeer;

class FullMissionCache
{
public:
	static constexpr int32_t kMaxMissionCacheSize{CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE};

	struct MissionView {
		const mission_item_s *items{nullptr};
		int32_t count{0};
		uint32_t mission_id{0};
		uint8_t dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_0};
		uint32_t generation{0};
	};

	enum class SyncResult : uint8_t {
		kRejected,  ///< Source mismatch, index out of range, or the cache is compiled out.
		kPatched,   ///< A loaded item was updated in place; a published view becomes stale.
		kDeferred   ///< The item is not loaded yet; the pending load reads it from dataman.
	};

	explicit FullMissionCache(orb_advert_t *mavlink_log_pub = nullptr);
	~FullMissionCache();
	FullMissionCache(const FullMissionCache &) = delete;
	FullMissionCache &operator=(const FullMissionCache &) = delete;

	void update(const mission_s &mission);
	void invalidate();

	bool missionItemsReady(const mission_s &mission) const;
	int missionCount() const;
	bool loadMissionItem(int index, mission_item_s &mission_item) const;
	bool getMissionView(const mission_s &mission, MissionView &view) const;
	bool missionViewStillValid(const MissionView &view) const;

	/** Apply a successful Dataman item write from Navigator's serialized task. */
	SyncResult syncMissionItem(const mission_s &mission, int32_t index, const mission_item_s &mission_item);

	/**
	 * Response subscription while a read is pending. Polling it while idle can spin
	 * on another client's response.
	 */
	orb_sub_t responseSubscription() const;

private:
	friend class MissionRouteCacheTestPeer;

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	static constexpr hrt_abstime kCacheRetryBackoff {500000}; // 500 ms
	static constexpr uint8_t kMaxRetryBackoffShift{3}; // Retry 3+: 500ms << 3 = 4000 ms.

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

	struct MissionCacheState {
		uint32_t id{0};
		int32_t count{0};
		uint8_t dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_0};
		int32_t next_index{0}; ///< First unread item; retries resume here.
		bool initialized{false};
		bool source_valid{false};
		bool ready{false};
		bool too_large{false};
		bool validation_pending{false};
		RetryBackoff retry{};
	};

	struct MissionRequest {
		uint32_t generation{0}; ///< Generation the read was issued for; a mismatch discards the response.
		int32_t index{-1};      ///< Mission item index being read.
		bool pending{false};    ///< generation and index are only meaningful while true.
	};

	bool missionMatchesCache(const mission_s &mission) const;
	bool missionCacheAvailable() const;
	void startMissionLoad();
	void resumeMissionLoad();
	void advanceMissionGeneration();

	mission_item_s *_mission_items{nullptr};
	DatamanClient _dataman_client_mission{};
	orb_advert_t *_mavlink_log_pub{nullptr};
	// Wall time for successful non-empty loads.
	perf_counter_t _load_perf{perf_alloc(PC_ELAPSED, "navigator: full mission cache load")};
	// Keep outside of MissionCacheState: it must survive `_mission = {}` reset.
	uint32_t _mission_generation{0};
	MissionRequest _mission_request{};
	MissionCacheState _mission{};
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
};
