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

	void update(const mission_s &mission);
	void invalidate();

	bool missionExceedsCacheLimit(const mission_s &mission) const;
	bool isReady(const mission_s &mission) const;
	bool safePointsReady() const { return _safe_points_ready && _safe_point_dataman_state == SafePointDatamanState::UpdateRequestWait; }
	bool safePointUpdatePending() const { return _safe_point_dataman_state != SafePointDatamanState::UpdateRequestWait; }
	uint32_t safePointsId() const { return _safe_points_id; }

	int missionCount() const override;
	bool loadMissionItem(int index, mission_item_s &mission_item) const override;
	int safePointCount() const override;
	bool loadSafePointItem(int index, mission_item_s &safe_point_item) const override;
	bool getMissionLandItem(int32_t &index, mission_item_s &land_item) const override;
	bool loadMissionItem(const mission_s &mission, int32_t index, mission_item_s &mission_item) const;
	bool syncMissionItem(const mission_s &mission, int32_t index, const mission_item_s &mission_item);

private:
	enum class SafePointDatamanState {
		UpdateRequestWait,
		Read,
		ReadWait,
		Load,
		Error
	};

	void updateMissionCache(const mission_s &mission);
	void updateMissionLandItemCache(const mission_s &mission);
	void updateSafePointCache(const mission_s &mission);
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
