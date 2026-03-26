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

using namespace time_literals;

class MissionRouteCache : public MissionRoutePlanner::Provider
{
public:
	static constexpr hrt_abstime MAX_DATAMAN_LOAD_WAIT{500_ms};
	static constexpr int32_t MAX_ROUTE_MISSION_CACHE_SIZE = CONFIG_RTL_MISSION_CACHE_SIZE;

	MissionRouteCache() = default;
	~MissionRouteCache() = default;

	/** @brief Refresh the cached mission geometry, mission land item, and safe points. */
	void update(const mission_s &mission);
	/** @brief Drop all cached mission and safe-point state. */
	void invalidate();

	/** @brief Return whether the mission is too large for the configured route cache. */
	bool missionExceedsCacheLimit(const mission_s &mission) const;
	/** @brief Return whether the cached mission geometry matches the active mission and is fully loaded. */
	bool isReady(const mission_s &mission) const;
	/** @brief Return whether the current safe-point set is fully loaded and ready for planning. */
	bool safePointsReady() const { return _safe_points_ready && _safe_point_dataman_state == SafePointDatamanState::UpdateRequestWait; }
	/** @brief Return whether an asynchronous safe-point refresh is still in progress. */
	bool safePointUpdatePending() const { return _safe_point_dataman_state != SafePointDatamanState::UpdateRequestWait; }
	/** @brief Return the safe-point set id currently tracked by the cache. */
	uint32_t safePointsId() const { return _safe_points_id; }

	/** @brief Return the number of cached mission items available to the planner. */
	int missionCount() const override;
	/** @brief Load one cached mission item by index. */
	bool loadMissionItem(int index, mission_item_s &mission_item) const override;
	/** @brief Return the number of cached safe points available to the planner. */
	int safePointCount() const override;
	/** @brief Load one cached safe point by index. */
	bool loadSafePointItem(int index, mission_item_s &safe_point_item) const override;
	/** @brief Load the cached mission landing item, if one exists. */
	bool getMissionLandItem(int32_t &index, mission_item_s &land_item) const override;
	/** @brief Load one mission item only while the cache still matches the supplied mission. */
	bool loadMissionItem(const mission_s &mission, int32_t index, mission_item_s &mission_item) const;
	/** @brief Mirror an updated mission item into the planner-facing cache entries. */
	bool syncMissionItem(const mission_s &mission, int32_t index, const mission_item_s &mission_item);

private:
	enum class SafePointDatamanState {
		UpdateRequestWait,
		Read,
		ReadWait,
		Load,
		Error
	};

	/** @brief Refresh the mission-item cache when the active mission changes. */
	void updateMissionCache(const mission_s &mission);
	/** @brief Refresh the cached mission land item when the landing anchor changes. */
	void updateMissionLandItemCache(const mission_s &mission);
	/** @brief Advance the asynchronous safe-point cache state machine. */
	void updateSafePointCache(const mission_s &mission);
	/** @brief Return whether the cached mission identity still matches the supplied mission. */
	bool missionMatchesCache(const mission_s &mission) const;

	mutable DatamanCache _dataman_cache_mission{"navigator_dm_cache_route_mission", 0};
	mutable DatamanCache _dataman_cache_safepoint{"rtl_dm_cache_miss_geo", 4};
	mutable DatamanCache _dataman_cache_land_item{"rtl_dm_cache_miss_land", 2};
	DatamanClient &_dataman_client_safepoint = _dataman_cache_safepoint.client();

	uint32_t _mission_id{0};
	int32_t _mission_count{0};
	uint8_t _mission_dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_0};
	bool _ready{false};
	bool _mission_too_large{false};

	uint32_t _mission_land_mission_id{0};
	uint8_t _mission_land_dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_0};
	int32_t _mission_land_index{-1};

	SafePointDatamanState _safe_point_dataman_state{SafePointDatamanState::UpdateRequestWait};
	SafePointDatamanState _safe_point_error_state{SafePointDatamanState::UpdateRequestWait};
	mission_stats_entry_s _safe_point_stats{};
	uint32_t _safe_points_id{0};
	uint32_t _opaque_id{0};
	bool _safe_points_ready{false};
	bool _initiate_safe_points_updated{true};
};
