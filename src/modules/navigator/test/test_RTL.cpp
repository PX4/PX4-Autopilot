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
 * @file test_RTL.cpp
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <parameters/param.h>
#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/wind.h>

#include "navigator.h"
#include "rtl.h"
#include "rtl_direct_mission_land.h"
#include "mission_route_types.h"
#include "support/mission_route_cache_test_peer.h"
#include "support/mission_route_test_helpers.h"

namespace
{

constexpr double kBaseLat = 47.397742;
constexpr double kBaseLon = 8.545594;
constexpr float kAlt = 500.f;
constexpr float kApproachRadius = 50.f;
constexpr double kNanDouble = static_cast<double>(NAN);

mission_item_s makeSafePointItem(double lat, double lon, float altitude, NAV_FRAME frame,
				 NAV_CMD nav_cmd = NAV_CMD_RALLY_POINT)
{
	mission_item_s item{};
	item.nav_cmd = nav_cmd;
	item.frame = frame;
	item.lat = lat;
	item.lon = lon;
	item.altitude = altitude;
	return item;
}

mission_item_s makeLandApproachItem(double lat, double lon, float altitude, float loiter_radius_m,
				    NAV_FRAME frame = NAV_FRAME_GLOBAL)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_LOITER_TO_ALT;
	item.frame = frame;
	item.lat = lat;
	item.lon = lon;
	item.altitude = altitude;
	item.altitude_is_relative = (frame == NAV_FRAME_GLOBAL_RELATIVE_ALT)
				    || (frame == NAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
	item.loiter_radius = loiter_radius_m;
	return item;
}

PositionYawSetpoint makePositionYawSetpointFromOffset(double base_lat, double base_lon, float north_m, float east_m,
		float alt)
{
	MapProjection ref{base_lat, base_lon};
	PositionYawSetpoint position{};
	ref.reproject(north_m, east_m, position.lat, position.lon);
	position.alt = alt;
	position.yaw = NAN;
	return position;
}

loiter_point_s makeLoiterPoint(const PositionYawSetpoint &position, float loiter_radius_m = kApproachRadius)
{
	loiter_point_s loiter_point{};
	loiter_point.lat = position.lat;
	loiter_point.lon = position.lon;
	loiter_point.height_m = position.alt;
	loiter_point.loiter_radius_m = loiter_radius_m;
	return loiter_point;
}

uint8_t countValidApproaches(const land_approaches_s &vtol_land_approaches)
{
	uint8_t count = 0;

	for (uint8_t i = 0; i < land_approaches_s::num_approaches_max; ++i) {
		if (vtol_land_approaches.approaches[i].isValid()) {
			++count;
		}
	}

	return count;
}

void expectLoiterPointNear(const loiter_point_s &actual, const PositionYawSetpoint &expected,
			   float loiter_radius_m = kApproachRadius)
{
	ASSERT_TRUE(actual.isValid());
	EXPECT_NEAR(actual.lat, expected.lat, 1e-9);
	EXPECT_NEAR(actual.lon, expected.lon, 1e-9);
	EXPECT_NEAR(actual.height_m, expected.alt, 0.01f);
	EXPECT_NEAR(actual.loiter_radius_m, loiter_radius_m, 0.01f);
}

struct ExtractValidSafePointPositionCase {
	const char *test_name;
	mission_item_s item;
	float home_altitude_amsl;
	bool expected_valid;
	double expected_lat;
	double expected_lon;
	float expected_alt;
};

struct ApproachGeometry {
	PositionYawSetpoint land;
	PositionYawSetpoint north;
	PositionYawSetpoint south;
};

struct VehicleStateCase {
	const char *test_name;
	bool is_vtol;
	uint8_t vehicle_type;
	bool expect_valid;
};

struct ReadFailureCase {
	const char *test_name;
	int32_t failure_index;
	bool expected_found;
	uint8_t expected_count;
};

} // namespace

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
class RtlLifecycleTestExecutor : public RtlBase
{
public:
	RtlLifecycleTestExecutor(Navigator *navigator, bool landing,
				 mission_route::Segment loop_segment = {}) :
		RtlBase(navigator, 0),
		_landing(landing),
		_loop_segment(loop_segment)
	{}

	void on_activation() override {}
	void on_active() override {}
	void on_inactivation() override { _deactivated = true; }
	bool isLanding() override { return _landing; }
	rtl_time_estimate_s calc_rtl_time_estimate() override { return {}; }
	mission_route::Segment lastFlownLoopSegment() const override { return _loop_segment; }
	bool deactivated() const { return _deactivated; }

private:
	bool setNextMissionItem() override { return false; }
	void setActiveMissionItems() override {}

	bool _landing{false};
	mission_route::Segment _loop_segment{};
	bool _deactivated{false};
};
#endif

class RTLTestPeer : public RTL
{
public:
	explicit RTLTestPeer(Navigator *navigator) : RTL(navigator) {}

	bool extractValidSafePointPositionForTest(const mission_item_s &safe_point_item, float home_altitude_amsl,
			mission_route::Position &position) const
	{
		return mission_route::extractSafePointPosition(safe_point_item, home_altitude_amsl, position);
	}

	loiter_point_s chooseBestLandingApproachForTest(const land_approaches_s &vtol_land_approaches)
	{
		_wind_sub.update();
		return chooseBestLandingApproach(vtol_land_approaches);
	}

	loiter_point_s selectLandingApproachForTest(const PositionYawSetpoint &destination)
	{
		_home_pos_sub.update();
		_vehicle_status_sub.update();
		_wind_sub.update();
		return selectLandingApproach(destination);
	}

	bool hasValidMissionForTest()
	{
		_mission_sub.update();
		_home_pos_sub.update();
		return hasValidMission();
	}

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	void activateRouteSafePointReturnForTest()
	{
		_param_rtl_type.set(6);
		run(true);
	}

	void forceRouteRetryForTest() { _destination_check_time = hrt_absolute_time() - 3'000'000; }
	void failNextRouteExecutorInitForTest() { _fail_next_route_executor_init = true; }

	void replaceMissionExecutorForTest(RtlBase *executor, RtlType rtl_type)
	{
		stopAndDeleteRtlMissionType(false);
		_rtl_type = rtl_type;
		_rtl_mission_type_handle = executor;
		_rtl_mission_type_handle->initialize();
		_rtl_mission_type_handle->run(true);
	}

	void setMissionExecutorLoopSegmentForTest(const mission_route::Segment &loop_segment)
	{
		ASSERT_NE(_rtl_mission_type_handle, nullptr);
		RtlBase::RouteSafePointConfig config{};
		config.plan.projection_context.route_projection.segment = loop_segment;
		_rtl_mission_type_handle->configureRouteSafePoint(config);
	}

	const RtlBase *missionExecutorForTest() const { return _rtl_mission_type_handle; }
	RtlType rtlTypeForTest() const { return _rtl_type; }
	bool routePlanSourceStillValidForTest() const { return routePlanSourceStillValid(); }
	uint32_t routePlanMissionGenerationForTest() const { return _route_safe_point.missionGeneration(); }
	mission_route::Segment lastRouteLoopSegmentForTest() const { return _route_safe_point.lastFlownLoopSegment(); }

protected:
	bool initRtlMissionType(RtlType new_rtl_type, float rtl_alt) override
	{
		if (_fail_next_route_executor_init
		    && new_rtl_type == RtlType::RTL_MISSION_SAFE_POINT_FOLLOW) {
			_fail_next_route_executor_init = false;
			return false;
		}

		return RTL::initRtlMissionType(new_rtl_type, rtl_alt);
	}

private:
	bool _fail_next_route_executor_init{false};
#endif
};

class RtlDirectMissionLandTestPeer : public RtlDirectMissionLand
{
public:
	RtlDirectMissionLandTestPeer(Navigator *navigator, const mission_s &mission) :
		RtlDirectMissionLand(navigator, mission) {}

	const mission_s &mission() const { return _mission; }
};

class RTLTest : public NavigatorDatamanTestBase
{
protected:
	Navigator _navigator{};
	RTLTestPeer _rtl{&_navigator};

	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();

		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_SAFE_POINTS_0));
		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_WAYPOINTS_OFFBOARD_0));

		mission_stats_entry_s empty_stats{};
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_STATE, 0,
						      reinterpret_cast<uint8_t *>(&empty_stats), sizeof(empty_stats)));

		_navigator.get_mission_route_cache().invalidate();

		publishHomePosition(makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt));
		publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
		publishWind(0.f, 0.f);
	}

	void TearDown() override
	{
		_navigator.get_mission_route_cache().invalidate();

		if (_home_pub != nullptr) {
			orb_unadvertise(_home_pub);
			_home_pub = nullptr;
		}

		if (_vehicle_status_pub != nullptr) {
			orb_unadvertise(_vehicle_status_pub);
			_vehicle_status_pub = nullptr;
		}

		if (_wind_pub != nullptr) {
			orb_unadvertise(_wind_pub);
			_wind_pub = nullptr;
		}

		if (_mission_pub != nullptr) {
			orb_unadvertise(_mission_pub);
			_mission_pub = nullptr;
		}

		param_control_autosave(true);
	}

	void loadSafePointsIntoRouteCache(const std::vector<mission_item_s> &items)
	{
		for (size_t i = 0; i < items.size(); ++i) {
			mission_item_s copy = items[i];
			ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_0, static_cast<uint32_t>(i),
							      reinterpret_cast<uint8_t *>(&copy), sizeof(copy)));
		}

		mission_stats_entry_s stats{};
		stats.num_items = static_cast<uint16_t>(items.size());
		stats.opaque_id = ++_safe_points_opaque_id;
		stats.dataman_id = DM_KEY_SAFE_POINTS_0;
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_STATE, 0,
						      reinterpret_cast<uint8_t *>(&stats), sizeof(stats)));

		mission_s mission{};
		mission.timestamp = hrt_absolute_time();
		mission.safe_points_id = ++_safe_points_id;
		mission.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;

		MissionRouteCache &mission_route_cache = _navigator.get_mission_route_cache();
		mission_route_cache.invalidate();
		ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(mission_route_cache, mission,
				[&] { return mission_route_cache.safePointsReady(); }))
				<< "test safe points did not load";
	}

	void publishHomePosition(const PositionYawSetpoint &position, uint32_t update_count = 0)
	{
		home_position_s home{};
		home.timestamp = hrt_absolute_time();
		home.lat = position.lat;
		home.lon = position.lon;
		home.alt = position.alt;
		home.valid_hpos = true;
		home.valid_alt = true;
		home.update_count = update_count;

		if (_home_pub == nullptr) {
			_home_pub = orb_advertise(ORB_ID(home_position), &home);

		} else {
			orb_publish(ORB_ID(home_position), _home_pub, &home);
		}
	}

	void publishMission(const mission_s &mission)
	{
		if (_mission_pub == nullptr) {
			_mission_pub = orb_advertise(ORB_ID(mission), &mission);

		} else {
			orb_publish(ORB_ID(mission), _mission_pub, &mission);
		}
	}

	void publishVehicleStatus(bool is_vtol, uint8_t vehicle_type)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.is_vtol = is_vtol;
		status.vehicle_type = vehicle_type;

		if (_vehicle_status_pub == nullptr) {
			_vehicle_status_pub = orb_advertise(ORB_ID(vehicle_status), &status);

		} else {
			orb_publish(ORB_ID(vehicle_status), _vehicle_status_pub, &status);
		}
	}

	void publishWind(float windspeed_north, float windspeed_east)
	{
		wind_s wind{};
		wind.timestamp = hrt_absolute_time();
		wind.windspeed_north = windspeed_north;
		wind.windspeed_east = windspeed_east;

		if (_wind_pub == nullptr) {
			_wind_pub = orb_advertise(ORB_ID(wind), &wind);

		} else {
			orb_publish(ORB_ID(wind), _wind_pub, &wind);
		}
	}

	void publishGlobalPosition(const PositionYawSetpoint &position)
	{
		vehicle_global_position_s global_position{};
		global_position.timestamp = hrt_absolute_time();
		global_position.lat = position.lat;
		global_position.lon = position.lon;
		global_position.alt = position.alt;
		_global_position_pub.publish(global_position);
	}

	void setMissionResultValid(const mission_s &mission)
	{
		mission_result_s *mission_result = _navigator.get_mission_result();
		mission_result->valid = true;
		mission_result->mission_id = mission.mission_id;
		mission_result->geofence_id = mission.geofence_id;
		mission_result->home_position_counter = 0;
	}

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	mission_s loadRoutePlanningCache(const std::vector<mission_item_s> &mission_items,
					 const std::vector<mission_item_s> &safe_points)
	{
		for (size_t i = 0; i < mission_items.size(); ++i) {
			mission_item_s item = mission_items[i];
			EXPECT_TRUE(_dataman_client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, static_cast<uint32_t>(i),
							      reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		}

		for (size_t i = 0; i < safe_points.size(); ++i) {
			mission_item_s item = safe_points[i];
			EXPECT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_0, static_cast<uint32_t>(i),
							      reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		}

		mission_stats_entry_s stats{};
		stats.num_items = static_cast<uint16_t>(safe_points.size());
		stats.opaque_id = ++_safe_points_opaque_id;
		stats.dataman_id = DM_KEY_SAFE_POINTS_0;
		EXPECT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_STATE, 0,
						      reinterpret_cast<uint8_t *>(&stats), sizeof(stats)));

		mission_s mission{};
		mission.timestamp = hrt_absolute_time();
		mission.mission_id = 42;
		mission.count = static_cast<uint16_t>(mission_items.size());
		mission.current_seq = mission_items.size() > 1 ? 1 : 0;
		mission.land_start_index = -1;
		mission.land_index = -1;
		mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
		mission.safe_points_id = ++_safe_points_id;
		mission.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;

		MissionRouteCache &cache = _navigator.get_mission_route_cache();
		cache.invalidate();
		EXPECT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(cache, mission, [&] {
			return cache.missionItemsReady(mission) && cache.safePointsReady();
		}));
		return mission;
	}
#endif

	ApproachGeometry makeApproachGeometry() const
	{
		return ApproachGeometry{
			makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
			makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 150.f, 100.f, kAlt + 20.f),
			makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 50.f, 100.f, kAlt + 20.f),
		};
	}

	orb_advert_t _home_pub{nullptr};
	orb_advert_t _vehicle_status_pub{nullptr};
	orb_advert_t _wind_pub{nullptr};
	orb_advert_t _mission_pub{nullptr};
	DatamanClient _dataman_client{};
	uORB::Publication<vehicle_global_position_s> _global_position_pub{ORB_ID(vehicle_global_position)};
	uint32_t _safe_points_id{0};
	uint32_t _safe_points_opaque_id{0};
};

TEST_F(RTLTest, MissionValidityMatchesMissionAndFeasibilityInputs)
{
	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.mission_id = 42;
	mission.geofence_id = 7;
	const uint32_t home_update_count = 3;
	publishHomePosition(makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt), home_update_count);
	publishMission(mission);

	mission_result_s *mission_result = _navigator.get_mission_result();
	mission_result->valid = true;
	mission_result->mission_id = mission.mission_id - 1;
	EXPECT_FALSE(_rtl.hasValidMissionForTest());

	mission_result->mission_id = mission.mission_id;
	mission_result->geofence_id = mission.geofence_id - 1;
	EXPECT_FALSE(_rtl.hasValidMissionForTest());

	mission_result->geofence_id = mission.geofence_id;
	mission_result->home_position_counter = home_update_count - 1;
	EXPECT_FALSE(_rtl.hasValidMissionForTest());

	mission_result->home_position_counter = home_update_count;
	EXPECT_TRUE(_rtl.hasValidMissionForTest());
}

TEST_F(RTLTest, DirectMissionLandStartsWithCurrentMission)
{
	mission_s mission{};
	mission.mission_id = 43;
	mission.count = 4;
	mission.land_start_index = 2;
	mission.land_index = 3;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;

	RtlDirectMissionLandTestPeer direct_mission_land{&_navigator, mission};
	EXPECT_EQ(direct_mission_land.mission().mission_id, mission.mission_id);
	EXPECT_EQ(direct_mission_land.mission().count, mission.count);
	EXPECT_EQ(direct_mission_land.mission().land_start_index, mission.land_start_index);
	EXPECT_EQ(direct_mission_land.mission().land_index, mission.land_index);
	EXPECT_EQ(direct_mission_land.mission().mission_dataman_id, mission.mission_dataman_id);
}

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
TEST_F(RTLTest, RouteSafePointReturnUsesPlannerAndTracksCacheGeneration)
{
	uORB::SubscriptionData<rtl_status_s> rtl_status_sub{ORB_ID(rtl_status)};
	const std::vector<mission_item_s> mission_items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	const mission_route::Position safe_position = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 20.f, kAlt);
	const std::vector<mission_item_s> safe_points{
		makeSafePointItem(safe_position.lat, safe_position.lon, safe_position.alt, NAV_FRAME_GLOBAL),
	};
	const mission_s mission = loadRoutePlanningCache(mission_items, safe_points);

	publishMission(mission);
	publishGlobalPosition(makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 20.f, 0.f, kAlt));
	setMissionResultValid(mission);

	_rtl.activateRouteSafePointReturnForTest();

	EXPECT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_MISSION_SAFE_POINT_FOLLOW);
	EXPECT_TRUE(_rtl.routePlanSourceStillValidForTest());
	const uint32_t original_generation = _rtl.routePlanMissionGenerationForTest();
	ASSERT_TRUE(rtl_status_sub.update());
	EXPECT_EQ(rtl_status_sub.get().rtl_type, rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_SAFE_POINT);
	EXPECT_EQ(rtl_status_sub.get().safe_point_index, 0);

	mission_item_s updated = mission_items[0];
	updated.altitude += 1.f;
	ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0,
					      reinterpret_cast<uint8_t *>(&updated), sizeof(updated)));
	ASSERT_EQ(_navigator.get_mission_route_cache().syncMissionItem(mission, 0, updated),
		  MissionRouteCache::SyncResult::kPatched);
	EXPECT_FALSE(_rtl.routePlanSourceStillValidForTest());

	MissionRouteCache::MissionView updated_view{};
	ASSERT_TRUE(_navigator.get_mission_route_cache().getMissionView(mission, updated_view));
	ASSERT_NE(updated_view.generation, original_generation);

	_rtl.run(true);

	EXPECT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_MISSION_SAFE_POINT_FOLLOW);
	EXPECT_TRUE(_rtl.routePlanSourceStillValidForTest());
	EXPECT_EQ(_rtl.routePlanMissionGenerationForTest(), updated_view.generation);
}

TEST_F(RTLTest, RouteSafePointReturnPromotesWhenCacheBecomesReady)
{
	const std::vector<mission_item_s> mission_items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	const mission_route::Position safe_position = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 20.f, kAlt);
	const std::vector<mission_item_s> safe_points{
		makeSafePointItem(safe_position.lat, safe_position.lon, safe_position.alt, NAV_FRAME_GLOBAL),
	};
	const mission_s mission = loadRoutePlanningCache(mission_items, safe_points);
	MissionRouteCache &cache = _navigator.get_mission_route_cache();
	cache.invalidate();

	publishMission(mission);
	publishGlobalPosition(makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 20.f, 0.f, kAlt));
	setMissionResultValid(mission);

	_rtl.activateRouteSafePointReturnForTest();

	EXPECT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_DIRECT);
	EXPECT_FALSE(_rtl.routePlanSourceStillValidForTest());

	_rtl.forceRouteRetryForTest();
	_rtl.run(true);
	EXPECT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_DIRECT);

	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(cache, mission, [&] {
		return cache.missionItemsReady(mission) && cache.safePointsReady();
	}));

	_rtl.forceRouteRetryForTest();
	_rtl.run(true);
	EXPECT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_MISSION_SAFE_POINT_FOLLOW);

	_rtl.run(true);
	EXPECT_TRUE(_rtl.routePlanSourceStillValidForTest());
}

TEST_F(RTLTest, RouteSafePointReturnDetectsSafePointCountChangeWithSameId)
{
	const std::vector<mission_item_s> mission_items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	const mission_route::Position safe_position = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 20.f, kAlt);
	const std::vector<mission_item_s> safe_points{
		makeSafePointItem(safe_position.lat, safe_position.lon, safe_position.alt, NAV_FRAME_GLOBAL),
	};
	const mission_s mission = loadRoutePlanningCache(mission_items, safe_points);

	publishMission(mission);
	publishGlobalPosition(makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 20.f, 0.f, kAlt));
	setMissionResultValid(mission);
	_rtl.activateRouteSafePointReturnForTest();
	ASSERT_TRUE(_rtl.routePlanSourceStillValidForTest());

	mission_item_s second_safe_point = makeSafePointFromOffset(kBaseLat, kBaseLon, 180.f, 20.f, kAlt);
	ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_0, 1,
					      reinterpret_cast<uint8_t *>(&second_safe_point), sizeof(second_safe_point)));
	mission_stats_entry_s updated_stats{};
	updated_stats.num_items = 2;
	updated_stats.opaque_id = mission.safe_points_id;
	updated_stats.dataman_id = DM_KEY_SAFE_POINTS_0;
	ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_STATE, 0,
					      reinterpret_cast<uint8_t *>(&updated_stats), sizeof(updated_stats)));

	MissionRouteCache &cache = _navigator.get_mission_route_cache();
	MissionRouteCacheTestPeer::requestSafePointRecheck(cache);
	ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(cache, mission, [&] {
		return cache.safePointsReady() && cache.safePointCount() == 2;
	}));

	EXPECT_FALSE(_rtl.routePlanSourceStillValidForTest());
}

TEST_F(RTLTest, RouteSafePointReturnUsesDirectFallbackForVtol)
{
	const std::vector<mission_item_s> mission_items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	const mission_route::Position safe_position = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 20.f, kAlt);
	const std::vector<mission_item_s> safe_points{
		makeSafePointItem(safe_position.lat, safe_position.lon, safe_position.alt, NAV_FRAME_GLOBAL),
	};
	const mission_s mission = loadRoutePlanningCache(mission_items, safe_points);

	publishMission(mission);
	publishGlobalPosition(makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 20.f, 0.f, kAlt));
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	setMissionResultValid(mission);

	_rtl.activateRouteSafePointReturnForTest();

	EXPECT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_DIRECT);
	EXPECT_FALSE(_rtl.routePlanSourceStillValidForTest());
}

TEST_F(RTLTest, RouteSafePointReturnPreservesLoopAnchorWhenSafePointReloadStarts)
{
	const std::vector<mission_item_s> mission_items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 200.f, kAlt),
		makeDoJump(0, 3),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 200.f, kAlt),
	};
	const mission_route::Position safe_position = makePositionFromOffset(kBaseLat, kBaseLon, 205.f, 100.f, kAlt);
	const std::vector<mission_item_s> safe_points{
		makeSafePointItem(safe_position.lat, safe_position.lon, safe_position.alt, NAV_FRAME_GLOBAL),
	};
	const mission_s mission = loadRoutePlanningCache(mission_items, safe_points);

	publishMission(mission);
	publishGlobalPosition(makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt));
	setMissionResultValid(mission);
	_rtl.activateRouteSafePointReturnForTest();
	ASSERT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_MISSION_SAFE_POINT_FOLLOW);

	mission_route::Segment loop_segment{};
	loop_segment.start.idx = 2;
	loop_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	loop_segment.end.idx = 0;
	loop_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	loop_segment.is_loop = true;
	loop_segment.loops_remaining = 3;
	ASSERT_TRUE(loop_segment.validLoop());

	MissionRouteCache &cache = _navigator.get_mission_route_cache();
	mission_s updated_mission = mission;
	updated_mission.timestamp = hrt_absolute_time();
	++updated_mission.safe_points_id;
	mission_stats_entry_s updated_stats{};
	updated_stats.num_items = static_cast<uint16_t>(safe_points.size());
	updated_stats.opaque_id = updated_mission.safe_points_id;
	updated_stats.dataman_id = DM_KEY_SAFE_POINTS_0;
	ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_STATE, 0,
					      reinterpret_cast<uint8_t *>(&updated_stats), sizeof(updated_stats)));

	_rtl.setMissionExecutorLoopSegmentForTest(loop_segment);

	publishMission(updated_mission);
	cache.update(updated_mission);
	ASSERT_FALSE(cache.safePointsReady());

	_rtl.run(true);

	ASSERT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_DIRECT);
	const mission_route::Segment preserved_loop = _rtl.lastRouteLoopSegmentForTest();
	EXPECT_EQ(preserved_loop.start.idx, loop_segment.start.idx);
	EXPECT_EQ(preserved_loop.end.idx, loop_segment.end.idx);
	EXPECT_EQ(preserved_loop.loops_remaining, loop_segment.loops_remaining);
	EXPECT_TRUE(preserved_loop.validLoop());

}

TEST_F(RTLTest, RouteSafePointReturnKeepsCommittedLandingHandlers)
{
	const std::vector<mission_item_s> mission_items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	const mission_route::Position safe_position = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 20.f, kAlt);
	const std::vector<mission_item_s> safe_points{
		makeSafePointItem(safe_position.lat, safe_position.lon, safe_position.alt, NAV_FRAME_GLOBAL),
	};
	const mission_s mission = loadRoutePlanningCache(mission_items, safe_points);

	publishMission(mission);
	publishGlobalPosition(makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 20.f, 0.f, kAlt));
	setMissionResultValid(mission);
	_rtl.activateRouteSafePointReturnForTest();
	ASSERT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_MISSION_SAFE_POINT_FOLLOW);

	mission_item_s updated = mission_items[0];
	updated.altitude += 1.f;
	ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0,
					      reinterpret_cast<uint8_t *>(&updated), sizeof(updated)));

	auto *landing_executor = new RtlLifecycleTestExecutor{&_navigator, true};
	_rtl.replaceMissionExecutorForTest(landing_executor, RTL::RtlType::RTL_MISSION_SAFE_POINT_FOLLOW);

	ASSERT_EQ(_navigator.get_mission_route_cache().syncMissionItem(mission, 0, updated),
		  MissionRouteCache::SyncResult::kPatched);
	ASSERT_FALSE(_rtl.routePlanSourceStillValidForTest());

	_rtl.run(true);

	EXPECT_EQ(_rtl.missionExecutorForTest(), landing_executor);
	EXPECT_FALSE(landing_executor->deactivated());
	EXPECT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_MISSION_SAFE_POINT_FOLLOW);

	auto *landing_fallback = new RtlLifecycleTestExecutor{&_navigator, true};
	_rtl.replaceMissionExecutorForTest(landing_fallback, RTL::RtlType::RTL_DIRECT_MISSION_LAND);
	_rtl.forceRouteRetryForTest();
	_rtl.run(true);

	EXPECT_EQ(_rtl.missionExecutorForTest(), landing_fallback);
	EXPECT_FALSE(landing_fallback->deactivated());
	EXPECT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_DIRECT_MISSION_LAND);
}

TEST_F(RTLTest, RouteSafePointExecutorInitFailureUsesDirectFallbackSelection)
{
	uORB::SubscriptionData<rtl_status_s> rtl_status_sub{ORB_ID(rtl_status)};
	const std::vector<mission_item_s> mission_items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};
	const mission_route::Position safe_position = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 20.f, kAlt);
	const std::vector<mission_item_s> safe_points{
		makeSafePointItem(safe_position.lat, safe_position.lon, safe_position.alt, NAV_FRAME_GLOBAL),
	};
	const mission_s mission = loadRoutePlanningCache(mission_items, safe_points);

	publishMission(mission);
	publishGlobalPosition(makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 20.f, 0.f, kAlt));
	setMissionResultValid(mission);
	_rtl.failNextRouteExecutorInitForTest();

	_rtl.activateRouteSafePointReturnForTest();

	EXPECT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_DIRECT);
	ASSERT_TRUE(rtl_status_sub.update());
	EXPECT_EQ(rtl_status_sub.get().rtl_type, rtl_status_s::RTL_STATUS_TYPE_DIRECT_SAFE_POINT);
	EXPECT_EQ(rtl_status_sub.get().safe_point_index, UINT8_MAX);

	// A completed planning attempt uses a sticky fallback; only cache-pending attempts are retried.
	_rtl.forceRouteRetryForTest();
	_rtl.run(true);
	EXPECT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_DIRECT);
}
#endif

// WHY: No land point means no usable approach bearing.
// WHAT: The chooser should return an invalid loiter.
TEST_F(RTLTest, ChooseBestLandingApproachRequiresLandLocation)
{
	// GIVEN: A valid loiter and no land point.
	publishWind(1.f, 0.f);

	const PositionYawSetpoint north_approach = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 20.f);
	land_approaches_s vtol_land_approaches{};
	vtol_land_approaches.approaches[0] = makeLoiterPoint(north_approach);

	// WHEN: The chooser runs.
	const loiter_point_s selected_approach = _rtl.chooseBestLandingApproachForTest(vtol_land_approaches);

	// THEN: It returns no approach.
	EXPECT_FALSE(selected_approach.isValid());
}

// WHY: Approach bearing is measured from the land point.
// WHAT: Home should not affect the choice.
TEST_F(RTLTest, ChooseBestLandingApproachUsesLandLocationAsBearingOrigin)
{
	// GIVEN: Two approaches on opposite sides of the land point and a 60 degree wind.
	publishWind(1.f, std::sqrt(3.0f));

	const ApproachGeometry geometry = makeApproachGeometry();
	land_approaches_s vtol_land_approaches{};
	vtol_land_approaches.land_location_lat_lon(0) = geometry.land.lat;
	vtol_land_approaches.land_location_lat_lon(1) = geometry.land.lon;
	vtol_land_approaches.approaches[0] = makeLoiterPoint(geometry.north);
	vtol_land_approaches.approaches[1] = makeLoiterPoint(geometry.south);

	// WHEN: The chooser evaluates the block.
	const loiter_point_s selected_approach = _rtl.chooseBestLandingApproachForTest(vtol_land_approaches);

	// THEN: The north approach is selected.
	expectLoiterPointNear(selected_approach, geometry.north);
}

class SelectLandingApproachVehicleStateTest :
	public RTLTest,
	public ::testing::WithParamInterface<VehicleStateCase>
{
};

// WHY: Only VTOL in FW mode should use an approach loiter.
// WHAT: All other vehicle states should reject it.
TEST_P(SelectLandingApproachVehicleStateTest, SelectLandingApproachHonorsVehicleState)
{
	const VehicleStateCase &test_case = GetParam();

	// GIVEN: One valid approach block, a 60 degree wind, and one vehicle state.
	const ApproachGeometry geometry = makeApproachGeometry();
	loadSafePointsIntoRouteCache({
		makeSafePointItem(geometry.land.lat, geometry.land.lon, geometry.land.alt, NAV_FRAME_GLOBAL),
		makeLandApproachItem(geometry.north.lat, geometry.north.lon, geometry.north.alt, kApproachRadius),
		makeLandApproachItem(geometry.south.lat, geometry.south.lon, geometry.south.alt, kApproachRadius),
	});
	publishWind(1.f, std::sqrt(3.0f));
	publishVehicleStatus(test_case.is_vtol, test_case.vehicle_type);

	// WHEN: selectLandingApproach evaluates the destination.
	const loiter_point_s selected_approach = _rtl.selectLandingApproachForTest(geometry.land);

	// THEN: Only VTOL FW gets the selected approach.
	if (test_case.expect_valid) {
		expectLoiterPointNear(selected_approach, geometry.north);

	} else {
		EXPECT_FALSE(selected_approach.isValid());
	}
}

INSTANTIATE_TEST_SUITE_P(
	RTL,
	SelectLandingApproachVehicleStateTest,
	::testing::Values(
		VehicleStateCase{"VtolRotaryWing", true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false},
		VehicleStateCase{"NonVtolFixedWing", false, vehicle_status_s::VEHICLE_TYPE_FIXED_WING, false},
		VehicleStateCase{"VtolFixedWing", true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING, true}),
	[](const ::testing::TestParamInfo<SelectLandingApproachVehicleStateTest::ParamType> &param_info)
{
	return std::string(param_info.param.test_name);
});

// WHY: Each rally point owns the loiters that follow it.
// WHAT: Scanning should stop at the next rally point.
TEST_F(RTLTest, ScanVtolLandApproachBlockStopsAtNextRallyPoint)
{
	// GIVEN: One block with two loiters, then a new rally point.
	const PositionYawSetpoint land_1 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const PositionYawSetpoint loiter_1 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 20.f);
	const PositionYawSetpoint loiter_2 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 50.f, kAlt + 30.f);
	const PositionYawSetpoint land_2 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt);
	const PositionYawSetpoint loiter_3 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 40.f);

	VectorProvider provider({
		makeSafePointItem(land_1.lat, land_1.lon, land_1.alt, NAV_FRAME_GLOBAL),
		makeLandApproachItem(loiter_1.lat, loiter_1.lon, loiter_1.alt, kApproachRadius),
		makeLandApproachItem(loiter_2.lat, loiter_2.lon, loiter_2.alt, kApproachRadius),
		makeSafePointItem(land_2.lat, land_2.lon, land_2.alt, NAV_FRAME_GLOBAL),
		makeLandApproachItem(loiter_3.lat, loiter_3.lon, loiter_3.alt, kApproachRadius),
	});

	// WHEN: The first block is scanned.
	land_approaches_s scanned_block{};
	const bool found_approach = provider.scanVtolLandApproachBlockForTest(0, kAlt, &scanned_block);

	// THEN: Only the first two loiters are returned.
	ASSERT_TRUE(found_approach);
	EXPECT_EQ(countValidApproaches(scanned_block), 2);
	expectLoiterPointNear(scanned_block.approaches[0], loiter_1);
	expectLoiterPointNear(scanned_block.approaches[1], loiter_2);
	EXPECT_FALSE(scanned_block.approaches[2].isValid());
}

// WHY: The result array has a fixed size.
// WHAT: Extra loiters should be ignored once it is full.
TEST_F(RTLTest, ScanVtolLandApproachBlockCapsApproachCount)
{
	// GIVEN: More valid loiters than the block can hold.
	const PositionYawSetpoint land = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	std::vector<mission_item_s> mission_items{
		makeSafePointItem(land.lat, land.lon, land.alt, NAV_FRAME_GLOBAL),
	};

	PositionYawSetpoint last_included{};

	for (uint8_t i = 0; i < land_approaches_s::num_approaches_max + 2; ++i) {
		const PositionYawSetpoint approach = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 25.f * (i + 1), 0.f,
						     kAlt + 10.f + i);

		if (i == land_approaches_s::num_approaches_max - 1) {
			last_included = approach;
		}

		mission_items.push_back(makeLandApproachItem(approach.lat, approach.lon, approach.alt, kApproachRadius));
	}

	VectorProvider provider(mission_items);

	// WHEN: The block is scanned.
	land_approaches_s scanned_block{};
	const bool found_approach = provider.scanVtolLandApproachBlockForTest(0, kAlt, &scanned_block);

	// THEN: The result stops at the hard limit.
	ASSERT_TRUE(found_approach);
	EXPECT_EQ(countValidApproaches(scanned_block), land_approaches_s::num_approaches_max);
	expectLoiterPointNear(scanned_block.approaches[0], makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 25.f, 0.f, kAlt + 10.f));
	expectLoiterPointNear(scanned_block.approaches[land_approaches_s::num_approaches_max - 1], last_included);
}

// WHY: A rally point can own an empty block.
// WHAT: Another rally point right after it should keep the block empty.
TEST_F(RTLTest, ScanVtolLandApproachBlockHandlesEmptyBlockBeforeNextRallyPoint)
{
	// GIVEN: A rally point followed immediately by another rally point.
	const PositionYawSetpoint land_1 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const PositionYawSetpoint land_2 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);
	const PositionYawSetpoint loiter = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 150.f, 0.f, kAlt + 20.f);

	VectorProvider provider({
		makeSafePointItem(land_1.lat, land_1.lon, land_1.alt, NAV_FRAME_GLOBAL),
		makeSafePointItem(land_2.lat, land_2.lon, land_2.alt, NAV_FRAME_GLOBAL),
		makeLandApproachItem(loiter.lat, loiter.lon, loiter.alt, kApproachRadius),
	});

	// WHEN: The first block is scanned.
	land_approaches_s scanned_block{};
	const bool found_approach = provider.scanVtolLandApproachBlockForTest(0, kAlt, &scanned_block);

	// THEN: It stays empty.
	EXPECT_FALSE(found_approach);
	EXPECT_EQ(countValidApproaches(scanned_block), 0);
}

// WHY: End-of-mission is the other empty-block case.
// WHAT: A final rally point should also return zero approaches.
TEST_F(RTLTest, ScanVtolLandApproachBlockHandlesEmptyBlockAtMissionEnd)
{
	// GIVEN: A mission that ends with a rally point.
	const PositionYawSetpoint land = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);

	VectorProvider provider({
		makeSafePointItem(land.lat, land.lon, land.alt, NAV_FRAME_GLOBAL),
	});

	// WHEN: Its block is scanned.
	land_approaches_s scanned_block{};
	const bool found_approach = provider.scanVtolLandApproachBlockForTest(0, kAlt, &scanned_block);

	// THEN: It is empty.
	EXPECT_FALSE(found_approach);
	EXPECT_EQ(countValidApproaches(scanned_block), 0);
}

/**
 * @brief Read-failure cases while scanning a safe-point approach block.
 */
class ScanVtolLandApproachBlockReadFailureTest :
	public RTLTest,
	public ::testing::WithParamInterface<ReadFailureCase>
{
};

// WHY: Read failures should not leave partial results behind.
// WHAT: Scanning should stop cleanly on a broken item.
TEST_P(ScanVtolLandApproachBlockReadFailureTest, ScanVtolLandApproachBlockHandlesReadFailures)
{
	const ReadFailureCase &test_case = GetParam();

	// GIVEN: One rally point with two loiters.
	const PositionYawSetpoint land = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const PositionYawSetpoint loiter_1 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 20.f);
	const PositionYawSetpoint loiter_2 = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 30.f);
	const std::vector<mission_item_s> mission_items{
		makeSafePointItem(land.lat, land.lon, land.alt, NAV_FRAME_GLOBAL),
		makeLandApproachItem(loiter_1.lat, loiter_1.lon, loiter_1.alt, kApproachRadius),
		makeLandApproachItem(loiter_2.lat, loiter_2.lon, loiter_2.alt, kApproachRadius),
	};

	VectorProvider provider(mission_items, std::vector<int32_t> {test_case.failure_index});

	// WHEN: The block scan hits a read failure.
	land_approaches_s scanned_block{};
	const bool found_approach = provider.scanVtolLandApproachBlockForTest(0, kAlt, &scanned_block);

	// THEN: The scan stops without inventing extra approaches.
	EXPECT_EQ(found_approach, test_case.expected_found);
	EXPECT_EQ(countValidApproaches(scanned_block), test_case.expected_count);

	if (test_case.expected_count > 0) {
		expectLoiterPointNear(scanned_block.approaches[0], loiter_1);
	}

	EXPECT_FALSE(scanned_block.approaches[1].isValid());
}

INSTANTIATE_TEST_SUITE_P(
	RTL,
	ScanVtolLandApproachBlockReadFailureTest,
	::testing::Values(
		ReadFailureCase{"FirstLoiter", 1, false, 0},
		ReadFailureCase{"SecondLoiter", 2, true, 1}),
	[](const ::testing::TestParamInfo<ScanVtolLandApproachBlockReadFailureTest::ParamType> &param_info)
{
	return std::string(param_info.param.test_name);
});

class FindAssociatedSafePointTest : public ::testing::Test {};

// WHY: Association is distance-limited.
// WHAT: A rally point outside the 10 m window should be skipped.
TEST_F(FindAssociatedSafePointTest, FindAssociatedSafePointIndexRejectsFarSafePoints)
{
	// GIVEN: One rally point outside the threshold and one inside.
	const PositionYawSetpoint rtl_destination = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const PositionYawSetpoint outside_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 10.25f, 0.f, kAlt);
	const PositionYawSetpoint inside_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 9.75f, kAlt);

	VectorProvider provider({
		makeSafePointItem(outside_safe_point.lat, outside_safe_point.lon, outside_safe_point.alt, NAV_FRAME_GLOBAL),
		makeSafePointItem(inside_safe_point.lat, inside_safe_point.lon, inside_safe_point.alt, NAV_FRAME_GLOBAL),
	});

	// WHEN: The association lookup runs.
	const land_approaches_s vtol_land_approaches = provider.getVtolLandApproachesNearLocation(rtl_destination, kAlt);

	// THEN: The nearby rally point is selected.
	ASSERT_TRUE(vtol_land_approaches.land_location_lat_lon.isAllFinite());
	EXPECT_NEAR(vtol_land_approaches.land_location_lat_lon(0), inside_safe_point.lat, 1e-9);
	EXPECT_NEAR(vtol_land_approaches.land_location_lat_lon(1), inside_safe_point.lon, 1e-9);
}

// WHY: The first valid nearby rally point owns the block.
// WHAT: A later match should not replace it.
TEST_F(FindAssociatedSafePointTest, FindAssociatedSafePointIndexReturnsFirstMatch)
{
	// GIVEN: Two nearby valid rally points.
	const PositionYawSetpoint rtl_destination = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const PositionYawSetpoint first_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 4.f, 0.f, kAlt);
	const PositionYawSetpoint second_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 5.f, kAlt);

	VectorProvider provider({
		makeSafePointItem(first_safe_point.lat, first_safe_point.lon, first_safe_point.alt, NAV_FRAME_GLOBAL),
		makeSafePointItem(second_safe_point.lat, second_safe_point.lon, second_safe_point.alt, NAV_FRAME_GLOBAL),
	});

	// WHEN: The association lookup runs.
	const land_approaches_s vtol_land_approaches = provider.getVtolLandApproachesNearLocation(rtl_destination, kAlt);

	// THEN: The first match is returned.
	ASSERT_TRUE(vtol_land_approaches.land_location_lat_lon.isAllFinite());
	EXPECT_NEAR(vtol_land_approaches.land_location_lat_lon(0), first_safe_point.lat, 1e-9);
	EXPECT_NEAR(vtol_land_approaches.land_location_lat_lon(1), first_safe_point.lon, 1e-9);
}

TEST_F(FindAssociatedSafePointTest, IndexedApproachLookupUsesSelectedSafePoint)
{
	const PositionYawSetpoint first_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const PositionYawSetpoint first_approach = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 20.f);
	const PositionYawSetpoint second_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 5.f, kAlt);
	const PositionYawSetpoint second_approach = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 55.f, kAlt + 30.f);

	VectorProvider provider({
		makeSafePointItem(first_safe_point.lat, first_safe_point.lon, first_safe_point.alt, NAV_FRAME_GLOBAL),
		makeLandApproachItem(first_approach.lat, first_approach.lon, first_approach.alt, kApproachRadius),
		makeSafePointItem(second_safe_point.lat, second_safe_point.lon, second_safe_point.alt, NAV_FRAME_GLOBAL),
		makeLandApproachItem(second_approach.lat, second_approach.lon, second_approach.alt, kApproachRadius),
	});

	const land_approaches_s approaches = provider.getVtolLandApproachesAtSafePointIndex(2, kAlt);

	ASSERT_TRUE(approaches.land_location_lat_lon.isAllFinite());
	EXPECT_NEAR(approaches.land_location_lat_lon(0), second_safe_point.lat, 1e-9);
	EXPECT_NEAR(approaches.land_location_lat_lon(1), second_safe_point.lon, 1e-9);
	EXPECT_EQ(countValidApproaches(approaches), 1);
	expectLoiterPointNear(approaches.approaches[0], second_approach);
}

// WHY: Association reads can fail too.
// WHAT: A failed load should stop the search and return no match.
TEST_F(FindAssociatedSafePointTest, FindAssociatedSafePointIndexHandlesReadFailure)
{
	// GIVEN: A matching rally point hidden behind a failed read.
	const PositionYawSetpoint rtl_destination = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const PositionYawSetpoint skipped_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 4.f, 0.f, kAlt);
	const PositionYawSetpoint later_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 5.f, kAlt);

	const std::vector<mission_item_s> safe_points{
		makeSafePointItem(skipped_safe_point.lat, skipped_safe_point.lon, skipped_safe_point.alt, NAV_FRAME_GLOBAL),
		makeSafePointItem(later_safe_point.lat, later_safe_point.lon, later_safe_point.alt, NAV_FRAME_GLOBAL),
	};
	VectorProvider provider(safe_points, std::vector<int32_t> {0});

	// WHEN: The association lookup hits the failed read.
	const land_approaches_s vtol_land_approaches = provider.getVtolLandApproachesNearLocation(rtl_destination, kAlt);

	// THEN: The search stops and reports no match.
	EXPECT_FALSE(vtol_land_approaches.land_location_lat_lon.isAllFinite());
}

// WHY: An invalid rally point should not block the next one.
// WHAT: Association should skip bad entries and keep scanning.
TEST_F(FindAssociatedSafePointTest, FindAssociatedSafePointIndexSkipsInvalidRallyPoints)
{
	// GIVEN: One invalid nearby rally point followed by a valid nearby rally point.
	const PositionYawSetpoint rtl_destination = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const PositionYawSetpoint valid_safe_point = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 5.f, kAlt);

	VectorProvider provider({
		makeSafePointItem(91.0, kBaseLon, kAlt, NAV_FRAME_GLOBAL),
		makeSafePointItem(valid_safe_point.lat, valid_safe_point.lon, valid_safe_point.alt, NAV_FRAME_GLOBAL),
	});

	// WHEN: The association lookup runs.
	const land_approaches_s vtol_land_approaches = provider.getVtolLandApproachesNearLocation(rtl_destination, kAlt);

	// THEN: The valid rally point is returned.
	ASSERT_TRUE(vtol_land_approaches.land_location_lat_lon.isAllFinite());
	EXPECT_NEAR(vtol_land_approaches.land_location_lat_lon(0), valid_safe_point.lat, 1e-9);
	EXPECT_NEAR(vtol_land_approaches.land_location_lat_lon(1), valid_safe_point.lon, 1e-9);
}

class ExtractValidSafePointPositionTest :
	public RTLTest,
	public ::testing::WithParamInterface<ExtractValidSafePointPositionCase>
{
};

// WHY: Safe-point parsing should fail fast on bad input.
// WHAT: Valid frames pass; bad commands, frames and coordinates do not.
TEST_P(ExtractValidSafePointPositionTest, ExtractValidSafePointPositionValidatesInput)
{
	const ExtractValidSafePointPositionCase &test_case = GetParam();
	mission_route::Position extracted_position{};

	// GIVEN: One safe-point item.
	// WHEN: The parser runs.
	const bool is_valid = _rtl.extractValidSafePointPositionForTest(test_case.item, test_case.home_altitude_amsl,
			      extracted_position);

	// THEN: Only valid items are accepted.
	EXPECT_EQ(is_valid, test_case.expected_valid);

	if (test_case.expected_valid) {
		EXPECT_NEAR(extracted_position.lat, test_case.expected_lat, 1e-9);
		EXPECT_NEAR(extracted_position.lon, test_case.expected_lon, 1e-9);
		EXPECT_NEAR(extracted_position.alt, test_case.expected_alt, 0.01f);
	}
}

INSTANTIATE_TEST_SUITE_P(
	RTL,
	ExtractValidSafePointPositionTest,
	::testing::Values(
ExtractValidSafePointPositionCase{
	"GlobalAbsoluteRallyPoint",
	makeSafePointItem(kBaseLat, kBaseLon, 510.f, NAV_FRAME_GLOBAL),
	kAlt,
	true,
	kBaseLat,
	kBaseLon,
	510.f,
},
ExtractValidSafePointPositionCase{
	"GlobalIntRallyPoint",
	makeSafePointItem(kBaseLat, kBaseLon, 510.f, NAV_FRAME_GLOBAL_INT),
	kAlt,
	true,
	kBaseLat,
	kBaseLon,
	510.f,
},
ExtractValidSafePointPositionCase{
	"GlobalRelativeRallyPoint",
	makeSafePointItem(kBaseLat, kBaseLon, 25.f, NAV_FRAME_GLOBAL_RELATIVE_ALT),
	kAlt,
	true,
	kBaseLat,
	kBaseLon,
	kAlt + 25.f,
},
ExtractValidSafePointPositionCase{
	"GlobalRelativeIntRallyPoint",
	makeSafePointItem(kBaseLat, kBaseLon, 25.f, NAV_FRAME_GLOBAL_RELATIVE_ALT_INT),
	kAlt,
	true,
	kBaseLat,
	kBaseLon,
	kAlt + 25.f,
},
ExtractValidSafePointPositionCase{
	"RelativeRallyPointWithoutHomeAltitude",
	makeSafePointItem(kBaseLat, kBaseLon, 25.f, NAV_FRAME_GLOBAL_RELATIVE_ALT),
	NAN,
	false,
	kNanDouble,
	kNanDouble,
	NAN,
},
ExtractValidSafePointPositionCase{
	"UnsupportedFrame",
	makeSafePointItem(kBaseLat, kBaseLon, 510.f, NAV_FRAME_MISSION),
	kAlt,
	false,
	kNanDouble,
	kNanDouble,
	NAN,
},
ExtractValidSafePointPositionCase{
	"NonRallyCommand",
	makeSafePointItem(kBaseLat, kBaseLon, 510.f, NAV_FRAME_GLOBAL, NAV_CMD_WAYPOINT),
	kAlt,
	false,
	kNanDouble,
	kNanDouble,
	NAN,
},
ExtractValidSafePointPositionCase{
	"NanLatitude",
	makeSafePointItem(kNanDouble, kBaseLon, 510.f, NAV_FRAME_GLOBAL),
	kAlt,
	false,
	kNanDouble,
	kNanDouble,
	NAN,
},
ExtractValidSafePointPositionCase{
	"NullIsland",
	makeSafePointItem(0.0, 0.0, 510.f, NAV_FRAME_GLOBAL),
	kAlt,
	false,
	kNanDouble,
	kNanDouble,
	NAN,
},
ExtractValidSafePointPositionCase{
	"LatitudeOutOfRange",
	makeSafePointItem(91.0, kBaseLon, 510.f, NAV_FRAME_GLOBAL),
	kAlt,
	false,
	kNanDouble,
	kNanDouble,
	NAN,
},
ExtractValidSafePointPositionCase{
	"LongitudeOutOfRange",
	makeSafePointItem(kBaseLat, 181.0, 510.f, NAV_FRAME_GLOBAL),
	kAlt,
	false,
	kNanDouble,
	kNanDouble,
	NAN,
}),
[](const ::testing::TestParamInfo<ExtractValidSafePointPositionTest::ParamType> &param_info)
{
	return std::string(param_info.param.test_name);
});

// WHY: Approach altitude can be absolute or relative.
// WHAT: Relative altitude should add home altitude; absolute altitude should not.
TEST_F(RTLTest, MakeVtolLandApproachPointConvertsRelativeAndAbsoluteAltitude)
{
	// GIVEN: Absolute and relative loiter items in both MAVLink frame variants.
	const PositionYawSetpoint absolute_position = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 40.f, 0.f, 530.f);
	const PositionYawSetpoint relative_position = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 40.f, NAN);

	const mission_item_s absolute_item = makeLandApproachItem(absolute_position.lat, absolute_position.lon,
					     absolute_position.alt, kApproachRadius);
	const mission_item_s absolute_int_item = makeLandApproachItem(absolute_position.lat, absolute_position.lon,
			absolute_position.alt, kApproachRadius, NAV_FRAME_GLOBAL_INT);
	const mission_item_s relative_item = makeLandApproachItem(relative_position.lat, relative_position.lon, 25.f,
					     kApproachRadius, NAV_FRAME_GLOBAL_RELATIVE_ALT);
	const mission_item_s relative_int_item = makeLandApproachItem(relative_position.lat, relative_position.lon, 25.f,
			kApproachRadius, NAV_FRAME_GLOBAL_RELATIVE_ALT_INT);

	VectorProvider provider({
		makeSafePointItem(kBaseLat, kBaseLon, kAlt, NAV_FRAME_GLOBAL),
		absolute_item,
		absolute_int_item,
		relative_item,
		relative_int_item,
	});

	// WHEN: The mission items are converted while reading the approach block.
	land_approaches_s vtol_land_approaches{};
	provider.scanVtolLandApproachBlockForTest(0, kAlt, &vtol_land_approaches);
	const loiter_point_s absolute_point = vtol_land_approaches.approaches[0];
	const loiter_point_s absolute_int_point = vtol_land_approaches.approaches[1];
	const loiter_point_s relative_point = vtol_land_approaches.approaches[2];
	const loiter_point_s relative_int_point = vtol_land_approaches.approaches[3];

	// THEN: The AMSL altitude is resolved correctly.
	ASSERT_TRUE(absolute_point.isValid());
	ASSERT_TRUE(absolute_int_point.isValid());
	ASSERT_TRUE(relative_point.isValid());
	ASSERT_TRUE(relative_int_point.isValid());
	EXPECT_NEAR(absolute_point.height_m, 530.f, 0.01f);
	EXPECT_NEAR(absolute_int_point.height_m, 530.f, 0.01f);
	EXPECT_NEAR(relative_point.height_m, kAlt + 25.f, 0.01f);
	EXPECT_NEAR(relative_int_point.height_m, kAlt + 25.f, 0.01f);
	EXPECT_NEAR(absolute_point.loiter_radius_m, kApproachRadius, 0.01f);
	EXPECT_NEAR(absolute_int_point.loiter_radius_m, kApproachRadius, 0.01f);
	EXPECT_NEAR(relative_point.loiter_radius_m, kApproachRadius, 0.01f);
	EXPECT_NEAR(relative_int_point.loiter_radius_m, kApproachRadius, 0.01f);
}

TEST_F(RTLTest, MakeVtolLandApproachPointRejectsInvalidInput)
{
	const mission_item_s invalid_latitude = makeLandApproachItem(91.0, kBaseLon, kAlt, kApproachRadius);
	const mission_item_s invalid_longitude = makeLandApproachItem(kBaseLat, 181.0, kAlt, kApproachRadius);
	const mission_item_s invalid_altitude = makeLandApproachItem(kBaseLat, kBaseLon, NAN, kApproachRadius);
	const mission_item_s invalid_radius = makeLandApproachItem(kBaseLat, kBaseLon, kAlt, NAN);

	EXPECT_FALSE(mission_route::makeVtolLandApproachPoint(invalid_latitude, kAlt).isValid());
	EXPECT_FALSE(mission_route::makeVtolLandApproachPoint(invalid_longitude, kAlt).isValid());
	EXPECT_FALSE(mission_route::makeVtolLandApproachPoint(invalid_altitude, kAlt).isValid());
	EXPECT_FALSE(mission_route::makeVtolLandApproachPoint(invalid_radius, kAlt).isValid());
}
