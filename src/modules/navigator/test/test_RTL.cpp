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
#include <tuple>
#include <vector>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <parameters/param.h>
#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/wind.h>

#include "navigator.h"
#include "rtl.h"
#include "rtl_direct_mission_land.h"
#include "rtl_mission_fast.h"
#include "rtl_mission_fast_reverse.h"
#include "rtl_mission_safe_point_follow.h"
#include "mission_route_land_approaches.h"
#include "mission_route_types.h"
#include "support/mission_route_cache_test_peer.h"
#include "support/mission_route_test_helpers.h"
#include "support/vector_mission_item_store.h"

namespace
{

constexpr double kBaseLat = 47.397742;
constexpr double kBaseLon = 8.545594;
constexpr float kAlt = 500.f;
constexpr double kNanDouble = static_cast<double>(NAN);
constexpr float kApproachRadius = 50.f;

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
				 mission_route::ActiveJumpAnchor loop_segment = {}) :
		RtlBase(navigator, 0),
		_landing(landing),
		_loop_segment(loop_segment)
	{}

	void on_activation() override {}
	void on_active() override {}
	void on_inactivation() override { _deactivated = true; }
	bool isLanding() override { return _landing; }
	rtl_time_estimate_s calc_rtl_time_estimate() override { return {}; }
	mission_route::ActiveJumpAnchor activeJumpAnchor() const override { return _loop_segment; }
	bool deactivated() const { return _deactivated; }

private:
	bool setNextMissionItem() override { return false; }
	void setActiveMissionItems() override {}

	bool _landing{false};
	mission_route::ActiveJumpAnchor _loop_segment{};
	bool _deactivated{false};
};
#endif

class NavigatorMissionStateTestPeer
{
public:
	static void observeMission(Navigator &navigator, const mission_s &mission)
	{
		navigator.updateMissionVtolStateOnUpload(mission);
	}
};

class RTLTestPeer : public RTL
{
public:
	explicit RTLTestPeer(Navigator *navigator) : RTL(navigator) {}

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

	// parameters are read on a parameter update notification, which the tests do not publish
	void updateParamsForTest() { updateParams(); }

	// the return type is re-decided every few seconds of inactive time, which a test does not have
	void decideRtlTypeForTest() { setRtlTypeAndDestination(); }

	RtlType rtlTypeForTest() const { return _rtl_type; }

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	void activateReturnTypeForTest(int32_t rtl_type)
	{
		_param_rtl_type.set(rtl_type);
		run(true);
	}

	void activateRouteSafePointReturnForTest() { activateReturnTypeForTest(7); }

	void evaluateInactiveRouteSafePointReturnForTest()
	{
		parameters_update();
		_param_rtl_type.set(7);
		forceRouteRetryForTest();
		run(false);
	}

	mission_route::RtlRoutePlan routePlanForTest() const
	{
		if (_rtl_type != RtlType::RTL_MISSION_SAFE_POINT_FOLLOW || _rtl_mission_type_handle == nullptr) {
			ADD_FAILURE() << "Route follower is unavailable";
			return {};
		}

		return static_cast<const RtlMissionSafePointFollow *>(_rtl_mission_type_handle)->_plan;
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

	void setMissionExecutorLoopSegmentForTest(const mission_route::ActiveJumpAnchor &loop_segment)
	{
		ASSERT_EQ(_rtl_type, RtlType::RTL_MISSION_SAFE_POINT_FOLLOW);
		ASSERT_NE(_rtl_mission_type_handle, nullptr);
		static_cast<RtlMissionSafePointFollow *>(_rtl_mission_type_handle)->_active_jump_anchor = loop_segment;
	}

	RtlBase *missionExecutorForTest() const { return _rtl_mission_type_handle; }
	int32_t missionSequenceForTest() const { return _mission_sub.get().current_seq; }
	bool routePlanSourceStillValidForTest() const { return routePlanSourceStillValid(); }
	uint32_t routePlanMissionGenerationForTest() const { return _route_safe_point.missionGeneration(); }
	mission_route::ActiveJumpAnchor lastRouteLoopSegmentForTest() const { return _route_safe_point.activeJumpAnchor(); }

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

template <typename RtlMissionType>
class RtlMissionTestPeer : public RtlMissionType
{
public:
	RtlMissionTestPeer(Navigator *navigator, const mission_s &mission) :
		RtlMissionType(navigator, mission) {}

	const mission_s &mission() const { return this->_mission; }

	void loadTestMission(const std::vector<mission_item_s> &items)
	{
		_mission_store.setItems(items);
		this->_mission.count = static_cast<int32_t>(_mission_store.itemCount());
	}

	// RTL::on_activation() refreshes the mode's mission copy right before it activates the mode
	void activateForTest()
	{
		this->_vehicle_status_sub.update();
		this->refreshMission();
		this->on_activation();
	}

	uint16_t activeNavCommand() const { return this->_mission_item.nav_cmd; }
	bool isClimbing() const { return this->_work_item_type == RtlMissionType::WorkItemType::WORK_ITEM_TYPE_CLIMB; }
	const mission_item_s &activeItem() const { return this->_mission_item; }
	bool activeItemValid() const { return this->_is_current_planned_mission_item_valid; }

protected:
	bool loadMissionItemFromCache(int32_t index, mission_item_s &mission_item) override
	{
		return _mission_store.loadItem(index, mission_item);
	}

private:
	navigator_test::VectorMissionItemStore _mission_store{};
};

using RtlDirectMissionLandTestPeer = RtlMissionTestPeer<RtlDirectMissionLand>;
using RtlMissionFastTestPeer = RtlMissionTestPeer<RtlMissionFast>;
using RtlMissionFastReverseTestPeer = RtlMissionTestPeer<RtlMissionFastReverse>;

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

		if (_global_position_pub != nullptr) {
			orb_unadvertise(_global_position_pub);
			_global_position_pub = nullptr;
		}

		if (_land_detected_pub != nullptr) {
			orb_unadvertise(_land_detected_pub);
			_land_detected_pub = nullptr;
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

	void publishVehicleStatus(bool is_vtol, uint8_t vehicle_type,
				  uint8_t nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.is_vtol = is_vtol;
		status.vehicle_type = vehicle_type;
		status.nav_state = nav_state;

		if (_vehicle_status_pub == nullptr) {
			_vehicle_status_pub = orb_advertise(ORB_ID(vehicle_status), &status);

		} else {
			orb_publish(ORB_ID(vehicle_status), _vehicle_status_pub, &status);
		}
	}

	void publishGlobalPosition(double lat, double lon, float altitude)
	{
		vehicle_global_position_s global_position{};
		global_position.timestamp = hrt_absolute_time();
		global_position.lat = lat;
		global_position.lon = lon;
		global_position.alt = altitude;

		if (_global_position_pub == nullptr) {
			_global_position_pub = orb_advertise(ORB_ID(vehicle_global_position), &global_position);

		} else {
			orb_publish(ORB_ID(vehicle_global_position), _global_position_pub, &global_position);
		}
	}

	void publishLandDetected(bool landed)
	{
		vehicle_land_detected_s land_detected{};
		land_detected.timestamp = hrt_absolute_time();
		land_detected.landed = landed;

		if (_land_detected_pub == nullptr) {
			_land_detected_pub = orb_advertise(ORB_ID(vehicle_land_detected), &land_detected);

		} else {
			orb_publish(ORB_ID(vehicle_land_detected), _land_detected_pub, &land_detected);
		}
	}

	// the items of one mission, in the dataman slot the mission header names
	void writeMissionToDataman(dm_item_t slot, const std::vector<mission_item_s> &items)
	{
		for (size_t index = 0; index < items.size(); index++) {
			mission_item_s item = items[index];
			ASSERT_TRUE(_dataman_client.writeSync(slot, index, reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		}
	}

	// Record the mission target through the normal inactive cycle before switching to RTL.
	template <typename Mode>
	void flyMissionThenTriggerReturn(Mode &mode, const mission_s &mission)
	{
		publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
		publishMission(mission);
		mode.on_inactive();
		publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
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
		publishGlobalPosition(position.lat, position.lon, position.alt);
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

	mission_s prepareFinalMissionLegScenario()
	{
		const std::vector<mission_item_s> mission_items{
			makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 50.f),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 50.f),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 50.f),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 20.f, kAlt + 50.f),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 20.f, kAlt + 50.f),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 20.f, kAlt + 50.f),
		};
		mission_s mission = loadRoutePlanningCache(mission_items, {});
		mission.current_seq = 5;
		publishMission(mission);
		publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
		publishGlobalPosition(makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 50.f, 5.f, kAlt + 50.f));
		publishLandDetected(false);
		setMissionResultValid(mission);
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
	orb_advert_t _global_position_pub{nullptr};
	orb_advert_t _land_detected_pub{nullptr};
	DatamanClient _dataman_client{};
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

TEST_F(RTLTest, DirectMissionLandKeepsVtolInMulticopterMode)
{
	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.current_seq = 0;
	mission.land_start_index = 0;
	mission.land_index = 2;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;

	mission_item_s land_start{};
	land_start.nav_cmd = NAV_CMD_DO_LAND_START;
	land_start.autocontinue = true;

	mission_item_s approach = makeLandApproachItem(kBaseLat, kBaseLon, kAlt, kApproachRadius);
	approach.autocontinue = true;

	mission_item_s land = makeSafePointItem(kBaseLat, kBaseLon, kAlt, NAV_FRAME_GLOBAL, NAV_CMD_VTOL_LAND);
	land.autocontinue = true;

	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(kBaseLat, kBaseLon, kAlt);
	publishLandDetected(false);
	_navigator.get_mission_result()->valid = true;

	RtlDirectMissionLandTestPeer direct_mission_land{&_navigator, mission};
	direct_mission_land.loadTestMission({land_start, approach, land});
	direct_mission_land.setRtlAlt(kAlt);

	// the mode refreshes the mission from the topic at activation, so publish the injected one
	mission.count = 3;
	publishMission(mission);

	direct_mission_land.activateForTest();

	EXPECT_EQ(direct_mission_land.activeNavCommand(), NAV_CMD_DO_LAND_START);
}

class RTLClimbUpdateTest : public RTLTest, public ::testing::WithParamInterface<std::tuple<bool, uint8_t>> {};

TEST_P(RTLClimbUpdateTest, KeepsPendingClimbAcrossCursorUpdates)
{
	const bool is_vtol = std::get<0>(GetParam());
	const uint8_t vehicle_type = std::get<1>(GetParam());
	const float return_alt = kAlt + 60.f;
	publishVehicleStatus(is_vtol, vehicle_type);
	publishLandDetected(false);
	_navigator.get_vstatus()->is_vtol = is_vtol;
	_navigator.get_vstatus()->vehicle_type = vehicle_type;
	_navigator.get_land_detected()->landed = false;
	_navigator.get_mission_result()->valid = true;

	auto setAltitude = [&](float alt) {
		publishGlobalPosition(kBaseLat, kBaseLon, alt);
		_navigator.get_global_position()->lat = kBaseLat;
		_navigator.get_global_position()->lon = kBaseLon;
		_navigator.get_global_position()->alt = alt;
	};
	setAltitude(kAlt);

	mission_item_s land_start{};
	land_start.nav_cmd = NAV_CMD_DO_LAND_START;
	land_start.autocontinue = true;
	const auto first_position = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 30.f);
	mission_item_s first = makeSafePointItem(first_position.lat, first_position.lon, first_position.alt,
			       NAV_FRAME_GLOBAL, NAV_CMD_WAYPOINT);
	first.autocontinue = true;
	mission_item_s second = first;
	second.lat += 0.001;
	mission_item_s land = second;
	land.nav_cmd = NAV_CMD_LAND;
	std::vector<mission_item_s> items{land_start, first, second, land};

	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.count = items.size();
	mission.land_start_index = 0;
	mission.land_index = 3;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;

	for (size_t i = 0; i < items.size(); ++i) {
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_1, i,
						      reinterpret_cast<uint8_t *>(&items[i]), sizeof(items[i])));
	}

	publishMission(mission);
	RtlDirectMissionLandTestPeer rtl{&_navigator, mission};
	rtl.loadTestMission(items);
	rtl.setRtlAlt(return_alt);
	rtl.run(false);
	rtl.run(true);
	ASSERT_TRUE(rtl.isClimbing());

	// Real mission-topic updates must preserve the climb and accept the new cursor.
	for (int32_t index : {1, 2, 1}) {
		mission.current_seq = index;
		mission.timestamp = hrt_absolute_time();
		publishMission(mission);
		rtl.run(true);
		EXPECT_EQ(rtl.mission().current_seq, index);
		EXPECT_TRUE(rtl.isClimbing());
		const auto &setpoint = _navigator.get_position_setpoint_triplet()->current;
		EXPECT_TRUE(setpoint.valid);
		EXPECT_NEAR(setpoint.lat, kBaseLat, 1e-9);
		EXPECT_NEAR(setpoint.lon, kBaseLon, 1e-9);
		EXPECT_FLOAT_EQ(setpoint.alt, return_alt);
	}

	// Reaching the climb altitude resumes the selected item without skipping it.
	setAltitude(return_alt);
	rtl.run(true);
	EXPECT_FALSE(rtl.isClimbing());
	EXPECT_EQ(rtl.mission().current_seq, 1);
	EXPECT_NEAR(_navigator.get_position_setpoint_triplet()->current.lat, first.lat, 1e-9);
	rtl.run(true);
	EXPECT_EQ(rtl.mission().current_seq, 1);

	// A later cursor update below the return altitude must not restart the initial climb.
	setAltitude(kAlt);
	mission.current_seq = 2;
	mission.timestamp = hrt_absolute_time();
	publishMission(mission);
	rtl.run(true);
	EXPECT_FALSE(rtl.isClimbing());
	EXPECT_NEAR(_navigator.get_position_setpoint_triplet()->current.lat, second.lat, 1e-9);

	// Re-entering RTL recomputes the requirement from the current altitude.
	rtl.run(false);
	rtl.run(true);
	EXPECT_TRUE(rtl.isClimbing());
	rtl.run(false);
	setAltitude(return_alt + 10.f);
	rtl.run(true);
	EXPECT_FALSE(rtl.isClimbing());
}

INSTANTIATE_TEST_SUITE_P(VehicleTypes, RTLClimbUpdateTest,
			 ::testing::Combine(::testing::Bool(),
					 ::testing::Values(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING,
							 vehicle_status_s::VEHICLE_TYPE_FIXED_WING)));

TEST_F(RTLTest, MissionUploadVtolStateSurvivesProgressAndSafePointUpdates)
{
	vehicle_status_s &status = *_navigator.get_vstatus();
	status.timestamp = hrt_absolute_time();
	status.is_vtol = true;
	status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	mission_s mission{};
	mission.mission_id = 42;
	mission.count = 5;
	NavigatorMissionStateTestPeer::observeMission(_navigator, mission);
	EXPECT_EQ(_navigator.getMissionVtolStateOnUpload(), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);

	status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	mission.current_seq = 3;
	++mission.safe_points_id;
	++mission.geofence_id;
	NavigatorMissionStateTestPeer::observeMission(_navigator, mission);
	EXPECT_EQ(_navigator.getMissionVtolStateOnUpload(), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);

	++mission.mission_id;
	NavigatorMissionStateTestPeer::observeMission(_navigator, mission);
	EXPECT_EQ(_navigator.getMissionVtolStateOnUpload(), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);

	status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	++mission.mission_dataman_id;
	NavigatorMissionStateTestPeer::observeMission(_navigator, mission);
	EXPECT_EQ(_navigator.getMissionVtolStateOnUpload(), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);

	mission.count = 0;
	NavigatorMissionStateTestPeer::observeMission(_navigator, mission);
	EXPECT_EQ(_navigator.getMissionVtolStateOnUpload(), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED);
}

TEST_F(RTLTest, MissionUploadedDuringVtolTransitionStartsInMc)
{
	vehicle_status_s &status = *_navigator.get_vstatus();
	status.timestamp = hrt_absolute_time();
	status.is_vtol = true;
	status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	status.in_transition_mode = true;
	mission_s mission{};
	mission.count = 5;
	NavigatorMissionStateTestPeer::observeMission(_navigator, mission);
	EXPECT_EQ(_navigator.getMissionVtolStateOnUpload(), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
}

TEST_F(RTLTest, MissionUploadWithoutVehicleStatusLeavesVtolStateUnknown)
{
	mission_s mission{};
	mission.count = 5;
	NavigatorMissionStateTestPeer::observeMission(_navigator, mission);
	EXPECT_EQ(_navigator.getMissionVtolStateOnUpload(), vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED);
}

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
TEST_F(RTLTest, BatteryAwareReturnTypePreservesDirectSelection)
{
	// A usable route must not change the existing RTL_TYPE=6 destination policy.
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

	_rtl.activateReturnTypeForTest(6);
	EXPECT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_DIRECT);
	EXPECT_FALSE(_rtl.routePlanSourceStillValidForTest());
}

// An inactive RTL estimate must not become the current flight direction. Near the
// parallel first and last legs, that would move the next estimate onto the first
// leg and lose continuity with the final waypoint that Mission is still flying.
TEST_F(RTLTest, InactiveRouteEstimatesPreserveNominalMissionSegmentThroughActivation)
{
	prepareFinalMissionLegScenario();
	const auto expected_join = makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 20.f, kAlt + 50.f);

	const auto expect_last_leg_plan = [&]() {
		const mission_route::RtlRoutePlan plan = _rtl.routePlanForTest();
		ASSERT_TRUE(plan.valid());
		EXPECT_EQ(plan.goal_type, mission_route::GoalType::kMissionTakeoff);
		EXPECT_TRUE(plan.direction_reversed);
		EXPECT_EQ(plan.first_mission_item_index, 4);
		EXPECT_LT(get_distance_to_next_waypoint(plan.join_position.lat, plan.join_position.lon,
							expected_join.lat, expected_join.lon), 0.1f);
	};

	for (int estimate = 0; estimate < 3; ++estimate) {
		SCOPED_TRACE(::testing::Message() << "Inactive estimate " << estimate);
		_rtl.evaluateInactiveRouteSafePointReturnForTest();
		expect_last_leg_plan();
	}

	_rtl.activateRouteSafePointReturnForTest();
	ASSERT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_MISSION_SAFE_POINT_FOLLOW);
	expect_last_leg_plan();
}

TEST_F(RTLTest, InactiveForecastReplansBranchInFromCurrentMissionIndexAndPosition)
{
	mission_s mission = prepareFinalMissionLegScenario();
	uORB::SubscriptionData<rtl_time_estimate_s> estimate_sub{ORB_ID(rtl_time_estimate)};
	uORB::SubscriptionData<vehicle_global_position_s> global_sub{ORB_ID(vehicle_global_position)};
	uORB::SubscriptionData<home_position_s> home_sub{ORB_ID(home_position)};
	home_sub.update();
	*_navigator.get_home_position() = home_sub.get();
	global_sub.update();
	*_navigator.get_global_position() = global_sub.get();

	_rtl.evaluateInactiveRouteSafePointReturnForTest();
	ASSERT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_MISSION_SAFE_POINT_FOLLOW);
	ASSERT_TRUE(estimate_sub.update());
	ASSERT_TRUE(estimate_sub.get().valid);
	const float last_leg_time = estimate_sub.get().time_estimate;
	RtlBase *executor = _rtl.missionExecutorForTest();
	const uint32_t generation = _rtl.routePlanMissionGenerationForTest();
	const auto last_leg_join = makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 20.f, kAlt + 50.f);
	const auto last_leg_plan = _rtl.routePlanForTest();
	ASSERT_TRUE(last_leg_plan.valid());
	EXPECT_LT(get_distance_to_next_waypoint(last_leg_plan.join_position.lat, last_leg_plan.join_position.lon,
						last_leg_join.lat, last_leg_join.lon), 0.1f);

	// Same mission/cache and vehicle position, but Mission now targets the parallel first leg.
	mission.current_seq = 1;
	mission.timestamp = hrt_absolute_time();
	publishMission(mission);
	_rtl.evaluateInactiveRouteSafePointReturnForTest();
	ASSERT_EQ(_rtl.missionSequenceForTest(), mission.current_seq);
	ASSERT_TRUE(estimate_sub.update());
	ASSERT_TRUE(estimate_sub.get().valid);
	const float first_leg_time = estimate_sub.get().time_estimate;
	EXPECT_LT(first_leg_time, last_leg_time);
	EXPECT_EQ(_rtl.missionExecutorForTest(), executor);
	EXPECT_EQ(_rtl.routePlanMissionGenerationForTest(), generation);
	const auto first_leg_join = makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 50.f);
	const auto first_leg_plan = _rtl.routePlanForTest();
	ASSERT_TRUE(first_leg_plan.valid());
	EXPECT_LT(get_distance_to_next_waypoint(first_leg_plan.join_position.lat, first_leg_plan.join_position.lon,
						first_leg_join.lat, first_leg_join.lon), 0.1f);

	// The next refresh must also move the branch-in when only the vehicle position changes.
	publishGlobalPosition(makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 75.f, 5.f, kAlt + 50.f));
	global_sub.update();
	*_navigator.get_global_position() = global_sub.get();
	_rtl.evaluateInactiveRouteSafePointReturnForTest();
	ASSERT_TRUE(estimate_sub.update());
	ASSERT_TRUE(estimate_sub.get().valid);
	EXPECT_GT(estimate_sub.get().time_estimate, first_leg_time);
	const auto moved_join = makePositionFromOffset(kBaseLat, kBaseLon, 75.f, 0.f, kAlt + 50.f);
	const auto moved_plan = _rtl.routePlanForTest();
	ASSERT_TRUE(moved_plan.valid());
	EXPECT_LT(get_distance_to_next_waypoint(moved_plan.join_position.lat, moved_plan.join_position.lon,
						moved_join.lat, moved_join.lon), 0.1f);
	EXPECT_EQ(_rtl.missionExecutorForTest(), executor);
	EXPECT_FALSE(_rtl.isActive());
	EXPECT_FALSE(executor->isActive());
}

// Pending validation can defer initial RTL planning until RTL is already active.
// That retry must still start from Mission's nominal direction, not the reverse
// direction of an estimate made before activation.
TEST_F(RTLTest, InitialRouteActivationValidationRetryDiscardsInactiveEstimateDirection)
{
	const mission_s mission = prepareFinalMissionLegScenario();
	_rtl.evaluateInactiveRouteSafePointReturnForTest();
	const mission_route::RtlRoutePlan initial_estimate = _rtl.routePlanForTest();
	ASSERT_TRUE(initial_estimate.valid());
	ASSERT_TRUE(initial_estimate.direction_reversed);
	ASSERT_EQ(initial_estimate.first_mission_item_index, 4);

	_navigator.get_mission_result()->valid = false;
	_rtl.activateRouteSafePointReturnForTest();
	ASSERT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_DIRECT);

	setMissionResultValid(mission);
	_rtl.forceRouteRetryForTest();
	_rtl.run(true);
	ASSERT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_MISSION_SAFE_POINT_FOLLOW);
	const mission_route::RtlRoutePlan retry_plan = _rtl.routePlanForTest();
	ASSERT_TRUE(retry_plan.valid());
	EXPECT_EQ(retry_plan.first_mission_item_index, initial_estimate.first_mission_item_index);
	EXPECT_LT(get_distance_to_next_waypoint(retry_plan.join_position.lat, retry_plan.join_position.lon,
						initial_estimate.join_position.lat, initial_estimate.join_position.lon), 0.1f);
}

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

TEST_F(RTLTest, RouteSafePointReturnWaitsForMissionValidation)
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
	_navigator.get_mission_result()->valid = false;
	_rtl.activateRouteSafePointReturnForTest();
	EXPECT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_DIRECT);

	_rtl.forceRouteRetryForTest();
	_rtl.run(true);
	EXPECT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_DIRECT);

	setMissionResultValid(mission);
	_rtl.forceRouteRetryForTest();
	_rtl.run(true);
	EXPECT_EQ(_rtl.rtlTypeForTest(), RTL::RtlType::RTL_MISSION_SAFE_POINT_FOLLOW);
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

	mission_route::ActiveJumpAnchor loop_segment{};
	loop_segment.jump_item_index = 3;
	ASSERT_TRUE(loop_segment.valid());

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
	const mission_route::ActiveJumpAnchor preserved_loop = _rtl.lastRouteLoopSegmentForTest();
	EXPECT_EQ(preserved_loop.jump_item_index, loop_segment.jump_item_index);
	EXPECT_TRUE(preserved_loop.valid());

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
	// Executor creation failure must publish the fallback destination, not the abandoned route selection.
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
TEST_F(RTLTest, MissionFastKeepsVtolInMulticopterMode)
{
	// A VTOL flying in multicopter mode continues the mission as a multicopter.
	// The attitude controller refuses a transition to fixed wing during RTL, so
	// asking for one here left the vehicle waiting on it forever.
	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.current_seq = 0;
	mission.land_start_index = -1;
	mission.land_index = -1;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;

	const PositionYawSetpoint second_position = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt);
	mission_item_s first = makeSafePointItem(kBaseLat, kBaseLon, kAlt, NAV_FRAME_GLOBAL, NAV_CMD_WAYPOINT);
	first.autocontinue = true;
	mission_item_s second = makeSafePointItem(second_position.lat, second_position.lon, kAlt, NAV_FRAME_GLOBAL,
				NAV_CMD_WAYPOINT);
	second.autocontinue = true;

	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(kBaseLat, kBaseLon, kAlt);
	publishLandDetected(false);
	_navigator.get_mission_result()->valid = true;

	RtlMissionFastTestPeer mission_fast{&_navigator, mission};
	mission_fast.loadTestMission({first, second});

	// the mode refreshes the mission from the topic at activation, so publish the injected one
	mission.count = 2;
	publishMission(mission);

	mission_fast.activateForTest();

	EXPECT_EQ(mission_fast.activeNavCommand(), NAV_CMD_WAYPOINT);
}

TEST_F(RTLTest, MissionFastReverseKeepsVtolInMulticopterMode)
{
	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.current_seq = 0;
	mission.land_start_index = -1;
	mission.land_index = -1;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;

	const PositionYawSetpoint second_position = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt);
	mission_item_s first = makeSafePointItem(kBaseLat, kBaseLon, kAlt, NAV_FRAME_GLOBAL, NAV_CMD_WAYPOINT);
	first.autocontinue = true;
	mission_item_s second = makeSafePointItem(second_position.lat, second_position.lon, kAlt, NAV_FRAME_GLOBAL,
				NAV_CMD_WAYPOINT);
	second.autocontinue = true;

	// At the second waypoint, so the reverse mission has a previous item to fly to
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(second_position.lat, second_position.lon, kAlt);
	publishLandDetected(false);
	_navigator.get_mission_result()->valid = true;

	RtlMissionFastReverseTestPeer mission_fast_reverse{&_navigator, mission};
	mission_fast_reverse.loadTestMission({first, second});

	// the mode refreshes the mission from the topic at activation, so publish the injected one
	mission.count = 2;
	publishMission(mission);

	mission_fast_reverse.activateForTest();

	EXPECT_EQ(mission_fast_reverse.activeNavCommand(), NAV_CMD_WAYPOINT);
}

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
TEST_F(RTLTest, GetVtolLandApproachesAtSafePointStopsAtNextRallyPoint)
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

	// WHEN: The first block is requested.
	const land_approaches_s scanned_block =
		mission_route::getVtolLandApproachesAtSafePointIndex(provider, 0, kAlt);

	// THEN: Only the first two loiters are returned.
	ASSERT_TRUE(scanned_block.isAnyApproachValid());
	EXPECT_EQ(countValidApproaches(scanned_block), 2);
	expectLoiterPointNear(scanned_block.approaches[0], loiter_1);
	expectLoiterPointNear(scanned_block.approaches[1], loiter_2);
	EXPECT_FALSE(scanned_block.approaches[2].isValid());
}

// WHY: The result array has a fixed size.
// WHAT: Extra loiters should be ignored once it is full.
TEST_F(RTLTest, GetVtolLandApproachesAtSafePointCapsApproachCount)
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

	// WHEN: The block is requested.
	const land_approaches_s scanned_block =
		mission_route::getVtolLandApproachesAtSafePointIndex(provider, 0, kAlt);

	// THEN: The result stops at the hard limit.
	ASSERT_TRUE(scanned_block.isAnyApproachValid());
	EXPECT_EQ(countValidApproaches(scanned_block), land_approaches_s::num_approaches_max);
	expectLoiterPointNear(scanned_block.approaches[0], makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 25.f, 0.f, kAlt + 10.f));
	expectLoiterPointNear(scanned_block.approaches[land_approaches_s::num_approaches_max - 1], last_included);
}

// WHY: A rally point can own an empty block.
// WHAT: Another rally point right after it should keep the block empty.
TEST_F(RTLTest, GetVtolLandApproachesAtSafePointHandlesEmptyBlockBeforeNextRallyPoint)
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

	// WHEN: The first block is requested.
	const land_approaches_s scanned_block =
		mission_route::getVtolLandApproachesAtSafePointIndex(provider, 0, kAlt);

	// THEN: It stays empty.
	EXPECT_FALSE(scanned_block.isAnyApproachValid());
	EXPECT_EQ(countValidApproaches(scanned_block), 0);
}

// WHY: End-of-mission is the other empty-block case.
// WHAT: A final rally point should also return zero approaches.
TEST_F(RTLTest, GetVtolLandApproachesAtSafePointHandlesEmptyBlockAtMissionEnd)
{
	// GIVEN: A mission that ends with a rally point.
	const PositionYawSetpoint land = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);

	VectorProvider provider({
		makeSafePointItem(land.lat, land.lon, land.alt, NAV_FRAME_GLOBAL),
	});

	// WHEN: Its block is requested.
	const land_approaches_s scanned_block =
		mission_route::getVtolLandApproachesAtSafePointIndex(provider, 0, kAlt);

	// THEN: It is empty.
	EXPECT_FALSE(scanned_block.isAnyApproachValid());
	EXPECT_EQ(countValidApproaches(scanned_block), 0);
}

TEST_F(RTLTest, IndexedLandApproachQueriesRequireValidRallyAnchor)
{
	const mission_item_s approach = makeLandApproachItem(kBaseLat, kBaseLon, kAlt + 30.f, kApproachRadius);
	VectorProvider provider({
		approach,                                                       // 0: loiter with no rally before it
		makeSafePointItem(kBaseLat, kBaseLon, kAlt, NAV_FRAME_GLOBAL),  // 1: valid rally
		approach,                                                       // 2: loiter in rally 1's block
		approach,                                                       // 3: loiter in rally 1's block
		makeSafePointItem(91.0, kBaseLon, kAlt, NAV_FRAME_GLOBAL),      // 4: rally with an invalid latitude
		approach,                                                       // 5
		makeSafePointItem(kBaseLat, kBaseLon, kAlt, NAV_FRAME_MISSION), // 6: rally with an unsupported frame
		approach,                                                       // 7
	});

	// No block for anything that is not a valid rally point, even when a loiter follows it:
	// -1 and safePointCount() are out of range, 0 and 2 are loiter items, 4 and 6 are unusable rally points.
	for (const int index : {-1, 0, 2, 4, 6, provider.safePointCount()}) {
		SCOPED_TRACE(index);
		EXPECT_FALSE(mission_route::hasVtolLandApproachesAtSafePointIndex(provider, index, kAlt));
		const land_approaches_s block = mission_route::getVtolLandApproachesAtSafePointIndex(provider, index, kAlt);
		EXPECT_FALSE(block.isAnyApproachValid());
		EXPECT_FALSE(block.land_location_lat_lon.isAllFinite());
	}

	EXPECT_TRUE(mission_route::hasVtolLandApproachesAtSafePointIndex(provider, 1, kAlt));
	EXPECT_TRUE(mission_route::getVtolLandApproachesAtSafePointIndex(provider, 1, kAlt).isAnyApproachValid());
}

TEST_F(RTLTest, IndexedLandApproachQueriesRequireReadableRallyAnchor)
{
	VectorProvider provider({
		makeSafePointItem(kBaseLat, kBaseLon, kAlt, NAV_FRAME_GLOBAL),
		makeLandApproachItem(kBaseLat, kBaseLon, kAlt + 30.f, kApproachRadius),
	}, {0});

	// A readable approach cannot be used when the rally point before it failed to load.
	EXPECT_FALSE(mission_route::hasVtolLandApproachesAtSafePointIndex(provider, 0, kAlt));
	const land_approaches_s block = mission_route::getVtolLandApproachesAtSafePointIndex(provider, 0, kAlt);
	EXPECT_FALSE(block.isAnyApproachValid());
	EXPECT_FALSE(block.land_location_lat_lon.isAllFinite());
}

/**
 * @brief Read-failure cases while scanning a safe-point approach block.
 */
class GetVtolLandApproachesAtSafePointReadFailureTest :
	public RTLTest,
	public ::testing::WithParamInterface<ReadFailureCase>
{
};

// WHY: A read failure ends the current approach block; entries loaded before it remain available.
// WHAT: Scanning should stop cleanly on a broken item.
TEST_P(GetVtolLandApproachesAtSafePointReadFailureTest, HandlesReadFailures)
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

	VectorProvider provider(mission_items, {test_case.failure_index});

	// WHEN: Reading the block hits a load failure.
	const land_approaches_s scanned_block =
		mission_route::getVtolLandApproachesAtSafePointIndex(provider, 0, kAlt);

	// THEN: The scan stops without inventing extra approaches.
	EXPECT_EQ(scanned_block.isAnyApproachValid(), test_case.expected_found);
	EXPECT_EQ(countValidApproaches(scanned_block), test_case.expected_count);

	if (test_case.expected_count > 0) {
		expectLoiterPointNear(scanned_block.approaches[0], loiter_1);
	}

	EXPECT_FALSE(scanned_block.approaches[1].isValid());
}

INSTANTIATE_TEST_SUITE_P(
	RTL,
	GetVtolLandApproachesAtSafePointReadFailureTest,
	::testing::Values(
		ReadFailureCase{"FirstLoiter", 1, false, 0},
		ReadFailureCase{"SecondLoiter", 2, true, 1}),
	[](const ::testing::TestParamInfo<GetVtolLandApproachesAtSafePointReadFailureTest::ParamType> &param_info)
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
	const land_approaches_s vtol_land_approaches =
		mission_route::getVtolLandApproachesNearLocation(provider, rtl_destination, kAlt);

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
	const land_approaches_s vtol_land_approaches =
		mission_route::getVtolLandApproachesNearLocation(provider, rtl_destination, kAlt);

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

	const land_approaches_s approaches = mission_route::getVtolLandApproachesAtSafePointIndex(provider, 2, kAlt);

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
	VectorProvider provider(safe_points, {0});

	// WHEN: The association lookup hits the failed read.
	const land_approaches_s vtol_land_approaches =
		mission_route::getVtolLandApproachesNearLocation(provider, rtl_destination, kAlt);

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
	const land_approaches_s vtol_land_approaches =
		mission_route::getVtolLandApproachesNearLocation(provider, rtl_destination, kAlt);

	// THEN: The valid rally point is returned.
	ASSERT_TRUE(vtol_land_approaches.land_location_lat_lon.isAllFinite());
	EXPECT_NEAR(vtol_land_approaches.land_location_lat_lon(0), valid_safe_point.lat, 1e-9);
	EXPECT_NEAR(vtol_land_approaches.land_location_lat_lon(1), valid_safe_point.lon, 1e-9);
}

class ExtractValidSafePointPositionTest :
	public ::testing::TestWithParam<ExtractValidSafePointPositionCase>
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
	const bool is_valid = mission_route::extractSafePointPosition(test_case.item, test_case.home_altitude_amsl,
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
	const land_approaches_s vtol_land_approaches =
		mission_route::getVtolLandApproachesAtSafePointIndex(provider, 0, kAlt);
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

// WHY: activation skips the inactive update, so a cached mission can miss a newly published land start.
// WHAT: direct mission landing uses the land start published since its last inactive cycle.
TEST_F(RTLTest, DirectMissionLandUsesMissionPublishedBeforeActivation)
{
	mission_s stale{};
	stale.timestamp = hrt_absolute_time();
	stale.mission_id = 7;
	stale.current_seq = 0;
	stale.land_start_index = -1;
	stale.land_index = -1;
	stale.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;

	mission_item_s land_start{};
	land_start.nav_cmd = NAV_CMD_DO_LAND_START;
	land_start.autocontinue = true;

	mission_item_s approach = makeLandApproachItem(kBaseLat, kBaseLon, kAlt, kApproachRadius);
	approach.autocontinue = true;

	mission_item_s land = makeSafePointItem(kBaseLat, kBaseLon, kAlt, NAV_FRAME_GLOBAL, NAV_CMD_VTOL_LAND);
	land.autocontinue = true;

	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(kBaseLat, kBaseLon, kAlt);
	publishLandDetected(false);
	_navigator.get_mission_result()->valid = true;

	RtlDirectMissionLandTestPeer direct_mission_land{&_navigator, stale};
	direct_mission_land.loadTestMission({land_start, approach, land});
	direct_mission_land.setRtlAlt(kAlt);

	// GIVEN: a newer mission with a land start is published after the mode last ran inactive
	mission_s fresh = stale;
	fresh.timestamp = hrt_absolute_time();
	fresh.mission_id = 8;
	fresh.land_start_index = 0;
	fresh.land_index = 2;
	fresh.count = 3;
	publishMission(fresh);

	// WHEN: the mode is activated on the next cycle
	direct_mission_land.activateForTest();

	// THEN: it flies the published mission from its land start, not the stale copy
	EXPECT_EQ(direct_mission_land.mission().mission_id, fresh.mission_id);
	EXPECT_EQ(direct_mission_land.mission().land_start_index, 0);
	EXPECT_EQ(direct_mission_land.activeNavCommand(), NAV_CMD_DO_LAND_START);
}

// Two waypoints at the given offsets north of the base position
static std::vector<mission_item_s> makeTwoWaypointMission(float first_north_m, float second_north_m)
{
	const PositionYawSetpoint first_position = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, first_north_m, 0.f,
			kAlt);
	const PositionYawSetpoint second_position = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, second_north_m, 0.f,
			kAlt);
	mission_item_s first = makeSafePointItem(first_position.lat, first_position.lon, kAlt, NAV_FRAME_GLOBAL,
			       NAV_CMD_WAYPOINT);
	first.autocontinue = true;
	mission_item_s second = makeSafePointItem(second_position.lat, second_position.lon, kAlt, NAV_FRAME_GLOBAL,
				NAV_CMD_WAYPOINT);
	second.autocontinue = true;
	return {first, second};
}

static mission_s makeMissionHeader(uint32_t mission_id, int32_t current_seq, uint16_t count)
{
	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.mission_id = mission_id;
	mission.current_seq = current_seq;
	mission.count = count;
	mission.land_start_index = -1;
	mission.land_index = -1;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;
	return mission;
}

// WHY: both fast RTL modes can hold a stale mission when activation begins.
// WHAT: both fly the mission published since the last inactive cycle. They share a fixture because
// dataman client IDs are not reused and this binary is close to the limit.
TEST_F(RTLTest, MissionFastUsesMissionPublishedBeforeActivation)
{
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(kBaseLat, kBaseLon, kAlt);
	publishLandDetected(false);
	_navigator.get_mission_result()->valid = true;

	auto fliesThePublishedMission = [&](auto & mission_fast) {
		// the copy the mode holds: two waypoints north of the vehicle
		mission_fast.loadTestMission(makeTwoWaypointMission(200.f, 400.f));

		// GIVEN: a newer mission with different waypoints is published after the mode last ran inactive
		const std::vector<mission_item_s> fresh_items = makeTwoWaypointMission(1000.f, 1200.f);
		mission_fast.loadTestMission(fresh_items);
		publishMission(makeMissionHeader(8, 0, 2));

		// WHEN: the mode is activated on the next cycle
		mission_fast.activateForTest();

		// THEN: it flies the closest item of the published mission
		EXPECT_EQ(mission_fast.mission().mission_id, 8u);
		EXPECT_EQ(mission_fast.activeNavCommand(), NAV_CMD_WAYPOINT);
		EXPECT_DOUBLE_EQ(mission_fast.activeItem().lat, fresh_items[0].lat);
		EXPECT_DOUBLE_EQ(mission_fast.activeItem().lon, fresh_items[0].lon);
	};

	RtlMissionFastTestPeer mission_fast{&_navigator, makeMissionHeader(7, 0, 2)};
	fliesThePublishedMission(mission_fast);

	RtlMissionFastReverseTestPeer mission_fast_reverse{&_navigator, makeMissionHeader(7, 0, 2)};
	fliesThePublishedMission(mission_fast_reverse);
}

// WHY: the recorded target is valid only for its original mission, even if that mission's cursor changes.
// WHAT: both RTL directions keep it for the same mission and select the closest waypoint after replacement.
// The cases share a fixture because dataman client IDs are not reused.
TEST_F(RTLTest, MissionFastModesKeepThePriorIndexOnlyForTheSameMission)
{
	publishLandDetected(false);
	_navigator.get_mission_result()->valid = true;

	{
		SCOPED_TRACE("same mission");
		publishGlobalPosition(kBaseLat, kBaseLon, kAlt);

		RtlMissionFastTestPeer mission_fast{&_navigator, makeMissionHeader(7, 1, 2)};
		const std::vector<mission_item_s> items = makeTwoWaypointMission(200.f, 400.f);
		mission_fast.loadTestMission(items);

		// GIVEN: the vehicle was flying towards item 1 when RTL was triggered, then the cursor moved
		flyMissionThenTriggerReturn(mission_fast, makeMissionHeader(7, 1, 2));
		publishMission(makeMissionHeader(7, 0, 2));

		mission_fast.activateForTest();

		// THEN: forward RTL keeps target 1, not the closest item 0
		EXPECT_TRUE(mission_fast.activeItemValid());
		EXPECT_DOUBLE_EQ(mission_fast.activeItem().lat, items[1].lat);
		EXPECT_DOUBLE_EQ(mission_fast.activeItem().lon, items[1].lon);

		// GIVEN: reverse RTL starts between the waypoints, closer to item 1
		const PositionYawSetpoint vehicle = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 350.f, 0.f, kAlt);
		publishGlobalPosition(vehicle.lat, vehicle.lon, kAlt);
		RtlMissionFastReverseTestPeer mission_fast_reverse{&_navigator, makeMissionHeader(7, 1, 2)};
		mission_fast_reverse.loadTestMission(items);
		flyMissionThenTriggerReturn(mission_fast_reverse, makeMissionHeader(7, 1, 2));
		publishMission(makeMissionHeader(7, 0, 2));

		mission_fast_reverse.activateForTest();

		// THEN: reverse RTL goes back to item 0, before the recorded target 1
		EXPECT_TRUE(mission_fast_reverse.activeItemValid());
		EXPECT_EQ(mission_fast_reverse.mission().current_seq, 0);
		EXPECT_DOUBLE_EQ(mission_fast_reverse.activeItem().lat, items[0].lat);
		EXPECT_DOUBLE_EQ(mission_fast_reverse.activeItem().lon, items[0].lon);
	}

	{
		SCOPED_TRACE("replacement mission");
		publishGlobalPosition(kBaseLat, kBaseLon, kAlt);

		const std::vector<mission_item_s> fresh_items = makeTwoWaypointMission(1000.f, 1200.f);
		auto dropsThePriorIndex = [&](auto & mode, const mission_s & prior_mission) {
			flyMissionThenTriggerReturn(mode, prior_mission);

			// GIVEN: a two-item mission replaced the recorded mission before RTL activated
			mode.loadTestMission(fresh_items);
			publishMission(makeMissionHeader(8, 0, 2));

			mode.activateForTest();

			// THEN: choose the closest item 0; reusing the saved index would select item 1
			EXPECT_TRUE(mode.activeItemValid());
			EXPECT_EQ(mode.mission().current_seq, 0);
			EXPECT_DOUBLE_EQ(mode.activeItem().lat, fresh_items[0].lat);
			EXPECT_DOUBLE_EQ(mode.activeItem().lon, fresh_items[0].lon);
		};

		RtlMissionFastTestPeer mission_fast{&_navigator, makeMissionHeader(7, 1, 3)};
		dropsThePriorIndex(mission_fast, makeMissionHeader(7, 1, 3));

		RtlMissionFastReverseTestPeer mission_fast_reverse{&_navigator, makeMissionHeader(7, 2, 3)};
		dropsThePriorIndex(mission_fast_reverse, makeMissionHeader(7, 2, 3));
	}
}

// WHY: the controller must refresh an existing mode's mission before destination selection and activation.
// WHAT: a mission replaced after the last inactive cycle supplies the waypoint flown on RTL activation.
TEST_F(RTLTest, ReturnActivationFliesTheMissionPublishedSinceTheLastInactiveCycle)
{
	int32_t rtl_type = 2; // RTL_TYPE_MISSION_FAST, follows the mission to its landing, so it needs a land start
	param_set(param_find("RTL_TYPE"), &rtl_type);
	RTLTestPeer &rtl = _rtl;
	rtl.updateParamsForTest();

	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(kBaseLat, kBaseLon, kAlt);
	publishLandDetected(false);

	mission_item_s land_start{};
	land_start.nav_cmd = NAV_CMD_DO_LAND_START;
	land_start.autocontinue = true;

	auto make_mission = [&](uint32_t mission_id, dm_item_t slot, float waypoint_north_m, float land_north_m) {
		const PositionYawSetpoint waypoint_position = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon,
				waypoint_north_m, 0.f, kAlt);
		const PositionYawSetpoint land_position = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, land_north_m, 0.f,
				kAlt);
		mission_item_s waypoint = makeSafePointItem(waypoint_position.lat, waypoint_position.lon, kAlt, NAV_FRAME_GLOBAL,
					  NAV_CMD_WAYPOINT);
		waypoint.autocontinue = true;
		mission_item_s land = makeSafePointItem(land_position.lat, land_position.lon, kAlt, NAV_FRAME_GLOBAL, NAV_CMD_LAND);
		land.autocontinue = true;
		writeMissionToDataman(slot, {waypoint, land_start, land});

		mission_s mission = makeMissionHeader(mission_id, 0, 3);
		mission.mission_dataman_id = slot;
		mission.land_start_index = 1;
		mission.land_index = 2;
		return mission;
	};

	// the controller ran inactive with mission 7, a waypoint 4 km north of the vehicle, and prepared
	// the mission fast mode from it
	const mission_s stale = make_mission(7, DM_KEY_WAYPOINTS_OFFBOARD_1, 4000.f, 5000.f);
	publishMission(stale);
	setMissionResultValid(stale);
	rtl.on_inactive();
	rtl.decideRtlTypeForTest();
	ASSERT_EQ(rtl.rtlTypeForTest(), RTL::RtlType::RTL_MISSION_FAST);

	// GIVEN: mission 8 replaced it before the return activated, with a waypoint 1 km north
	const mission_s fresh = make_mission(8, DM_KEY_WAYPOINTS_OFFBOARD_0, 1000.f, 6000.f);
	publishMission(fresh);
	setMissionResultValid(fresh);

	// WHEN: the return activates
	rtl.on_activation();

	// THEN: it flies to mission 8's waypoint, neither mission 7's nor a loiter where it is
	const PositionYawSetpoint fresh_waypoint = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt);
	const position_setpoint_s &current = _navigator.get_position_setpoint_triplet()->current;
	EXPECT_TRUE(current.valid);
	EXPECT_EQ(current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
	EXPECT_NEAR(current.lat, fresh_waypoint.lat, 1e-7);
	EXPECT_NEAR(current.lon, fresh_waypoint.lon, 1e-7);

	// the dataman is shared by every case in this binary, so leave the mission slots as they were found
	EXPECT_TRUE(_dataman_client.clearSync(DM_KEY_WAYPOINTS_OFFBOARD_0));
	EXPECT_TRUE(_dataman_client.clearSync(DM_KEY_WAYPOINTS_OFFBOARD_1));
}
