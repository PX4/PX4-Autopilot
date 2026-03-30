/**
 * @file mission_route_cache.h
 *
 * Shared full-mission and safe-point cache used by Mission smart rejoin and
 * Route Safe Point Return planning.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <dataman_client/DatamanClient.hpp>
#include <uORB/topics/mission.h>

#include "navigation.h"
#include "mission_route_planner.h"

class MissionRouteCache : public MissionRoutePlanner::Provider
{
public:
	static constexpr hrt_abstime MAX_DATAMAN_LOAD_WAIT{500000}; // 500 ms
	static constexpr hrt_abstime CACHE_RETRY_BACKOFF{500000}; // 500 ms
	static constexpr uint8_t MAX_RETRY_BACKOFF_SHIFT{3}; // Retry 3+: 500ms << 3 = 500ms * 2^3 = 4000 ms (4 sec) wait.
	static constexpr int32_t MAX_ROUTE_MISSION_CACHE_SIZE{CONFIG_RTL_MISSION_CACHE_SIZE};
	static constexpr uint32_t INITIAL_SAFEPOINT_CACHE_SIZE{0};
	static constexpr uint32_t INITIAL_LAND_ITEM_CACHE_SIZE{1};

	MissionRouteCache() = default;
	~MissionRouteCache() = default;
	MissionRouteCache(const MissionRouteCache &) = delete;
	MissionRouteCache &operator=(const MissionRouteCache &) = delete;

	/** @brief Refresh the cached mission geometry, mission land item, and safe points. */
	void update(const mission_s &mission);
	/** @brief Drop all cached mission and safe-point state. */
	void invalidate();

	/** @brief Return whether the mission is too large for the configured route cache. */
	bool missionExceedsCacheLimit(const mission_s &mission) const;
	/** @brief Return whether the cached mission geometry matches the active mission and is fully loaded. */
	bool isReady(const mission_s &mission) const;
	/** @brief Return whether the current safe-point set is fully loaded and ready for planning. */
	bool safePointsReady() const
	{
		return _safe_point.ready
		       && !_safe_point.update_requested
		       && _safe_point.dataman_state == SafePointDatamanState::UpdateRequestWait;
	}
	/** @brief Return whether an asynchronous safe-point refresh is still in progress. */
	bool safePointUpdatePending() const
	{
		return _safe_point.update_requested
		       || _safe_point.dataman_state != SafePointDatamanState::UpdateRequestWait;
	}
	/** @brief Return the safe-point set id currently tracked by the cache. */
	uint32_t safePointsId() const { return _safe_point.source_id; }

	/** @brief Return the number of cached mission items available to the planner. */
	int missionCount() const override;
	/** @brief Load one cached mission item by index. */
	bool loadMissionItem(int index, mission_item_s &mission_item) const override;
	/** @brief Return the number of cached safe points available to the planner. */
	int safePointCount() const override;
	/** @brief Load one cached safe point by index. */
	bool loadSafePointItem(int index, mission_item_s &safe_point_item) const override;
	/** @brief Load the cached mission landing item, if the mission-published land index points to one. */
	bool getMissionLandItem(int32_t &index, mission_item_s &land_item) const override;
	/** @brief Load one mission item only while the cache still matches the supplied mission. */
	bool loadMissionItem(const mission_s &mission, int32_t index, mission_item_s &mission_item) const;
	/** @brief Mirror an updated mission item into the planner-facing cache entries. */
	bool syncMissionItem(const mission_s &mission, int32_t index, const mission_item_s &mission_item);

private:
	friend class MissionRouteCacheTestPeer;

	enum class SafePointDatamanState {
		UpdateRequestWait,
		Read,
		ReadWait,
		Load,
		Error
	};

	struct MissionCacheState {
		uint32_t id{0};
		int32_t count{0};
		uint8_t dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_0};
		bool ready{false};
		bool too_large{false};
		bool validation_pending{false};
		hrt_abstime retry_at{0};
		uint8_t retry_count{0};
	};

	struct MissionLandState {
		uint32_t mission_id{0};
		uint8_t dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_0};
		int32_t index{-1};
	};

	struct SafePointState {
		SafePointDatamanState dataman_state{SafePointDatamanState::UpdateRequestWait};
		SafePointDatamanState error_state{SafePointDatamanState::UpdateRequestWait};
		mission_stats_entry_s stats{};
		uint32_t source_id{0};
		uint32_t opaque_id{0};
		bool opaque_id_valid{false};
		bool ready{false};
		bool update_requested{true};
		hrt_abstime retry_at{0};
		uint8_t retry_count{0};
	};

	/** @brief Refresh the mission-item cache when the active mission changes. */
	void updateMissionCache(const mission_s &mission);
	/** @brief Refresh the cached mission land item when the landing anchor changes. */
	void updateMissionLandItemCache(const mission_s &mission);
	/** @brief Advance the asynchronous safe-point cache state machine. */
	void updateSafePointCache(const mission_s &mission);
	/** @brief Return whether the cached mission identity still matches the supplied mission. */
	bool missionMatchesCache(const mission_s &mission) const;
	/** @brief Queue a full mission-cache refresh for the supplied mission. */
	bool queueMissionCacheLoads(const mission_s &mission);
	/** @brief Return the next retry backoff for a cache reload attempt. */
	static hrt_abstime nextRetryBackoff(uint8_t retry_count);
	/** @brief Check whether every mission item is already resident in the cache. */
	bool missionCacheFullyLoaded(const mission_s &mission) const;
	/** @brief Check whether every safe point is already resident in the cache. */
	bool safePointCacheFullyLoaded() const;
	/** @brief Reset safe-point async state and cached items, optionally dropping the tracked source id. */
	void resetSafePointCacheState(bool clear_source_identity);

	// MissionRouteCache is used from Navigator's single-threaded work-loop. The mutable caches
	// allow const planner/provider methods to service loadWait() misses under that assumption.
	mutable DatamanCache _dataman_cache_mission{"navigator_dm_cache_route_mission", 0};
	mutable DatamanCache _dataman_cache_safepoint{"navigator_dm_cache_route_safepoint", INITIAL_SAFEPOINT_CACHE_SIZE};
	mutable DatamanCache _dataman_cache_land_item{"navigator_dm_cache_route_land", INITIAL_LAND_ITEM_CACHE_SIZE};
	DatamanClient &_dataman_client_safepoint = _dataman_cache_safepoint.client();

	MissionCacheState _mission{};
	MissionLandState _mission_land{};
	SafePointState _safe_point{};
};
