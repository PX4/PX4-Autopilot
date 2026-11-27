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
 * @file test_mission_route_join.cpp
 *
 * Mission route join integration tests.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include "mission.h"
#include "mission_item_utils.h"
#include "navigator.h"
#include "rtl_mission_fast_reverse.h"
#include "support/event_recorder.h"
#include "support/mission_route_cache_test_peer.h"
#include "support/mission_route_test_helpers.h"
#include "support/navigator_dataman_test.h"

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include <cstdio>
#include <ostream>
#include <string>
#include <vector>

/* EVENT
 * @skip-file
 */

static constexpr double kBaseLat = 47.0;
static constexpr double kBaseLon = 8.0;
static constexpr float kAlt = 100.f;

static mission_item_s makeCameraTriggerItem(float distance_m)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_DO_SET_CAM_TRIGG_DIST;
	item.params[0] = distance_m;
	item.autocontinue = true;
	return item;
}

/**
 * @brief Mission peer that exposes smart-rejoin state for navigator-level tests.
 */
class MissionTestPeer : public Mission
{
public:
	explicit MissionTestPeer(Navigator *navigator) : Mission(navigator) {}

	const mission_route::ActiveJumpAnchor &jumpAnchorForTest() const { return _active_jump_anchor; }
	bool advanceMissionForTest() { return setNextMissionItem(); }
	bool plannedItemValidForTest() const { return _is_current_planned_mission_item_valid; }
	void setWriteFailureIndexForTest(int32_t index) { _write_failure_index = index; }
	bool writeMissionItemToCache(int32_t index, mission_item_s &item) override
	{
		return index != _write_failure_index && MissionBase::writeMissionItemToCache(index, item);
	}

	void setDisarmedSinceActivationForTest()
	{
		_mission_has_been_activated = true;
		_system_disarmed_while_inactive = true;
	}
	using MissionBase::WorkItemType;

	void beginItemForTest(WorkItemType work)
	{
		_work_item_type = work;
		_mission_type = MissionType::MISSION_TYPE_MISSION;
		_is_current_planned_mission_item_valid = true;
		_mission_check_state = MissionCheckState::Complete;
		_mission_init_climb_altitude_amsl = NAN;
		_navigator->reset_triplets();
		set_mission_items();
	}

	// Read changed parameters now, instead of on the next parameter_update.
	void refreshParametersForTest() { updateParams(); }

	// A pending check, as after a geofence upload; on_active() completes it with the current verdict.
	void requestFeasibilityCheckForTest() { _mission_check_state = MissionCheckState::Unchecked; }
	const mission_item_s &currentMissionItemForTest() const { return _mission_item; }

	WorkItemType workItemTypeForTest() const { return _work_item_type; }
	int32_t currentSequenceForTest() const { return _mission.current_seq; }
	const mission_route::Position &joinPositionForTest() const { return _route_join_position; }

private:
	int32_t _write_failure_index{-1};
};

/**
 * @brief End-to-end fixture driving the real Mission class with dataman-backed mission data.
 */
class MissionRouteJoinTest : public NavigatorDatamanTestBase
{
protected:
	void SetUp() override
	{
		param_reset_all();
		const hrt_abstime now = hrt_absolute_time();

		mission_s mission{};
		mission.timestamp = now;
		_mission_pub.publish(mission);

		vehicle_status_s vehicle_status{};
		vehicle_status.timestamp = now;
		vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
		vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_UNSPECIFIED;
		_vehicle_status_pub.publish(vehicle_status);
		_vehicle_status = vehicle_status;

		vehicle_land_detected_s land_detected{};
		land_detected.timestamp = now;
		land_detected.landed = true;
		_land_detected_pub.publish(land_detected);
		_land_detected = land_detected;

		vehicle_global_position_s global_position{};
		global_position.timestamp = now;
		global_position.lat = static_cast<double>(NAN);
		global_position.lon = static_cast<double>(NAN);
		global_position.alt = NAN;
		_vehicle_global_position_pub.publish(global_position);
		_global_position = global_position;

		vehicle_local_position_s local_position{};
		local_position.timestamp = now;
		local_position.heading = NAN;
		local_position.vx = NAN;
		local_position.vy = NAN;
		_vehicle_local_position_pub.publish(local_position);
		_local_position = local_position;

		home_position_s home_position{};
		home_position.timestamp = now;
		_home_position_pub.publish(home_position);
		_home_position = home_position;

		// Mission activation forces a feasibility check, which requires a completed fence load.
		mission_stats_entry_s fence_stats{};
		fence_stats.dataman_id = DM_KEY_FENCE_POINTS_0;
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_FENCE_POINTS_STATE, 0,
						      reinterpret_cast<uint8_t *>(&fence_stats), sizeof(fence_stats)));
		Geofence &fence = _navigator.get_geofence();
		fence.updateFence();
		const hrt_abstime start = hrt_absolute_time();

		while (!fence.isReadyForPathChecks() && hrt_elapsed_time(&start) < 1_s) {
			fence.run();
			px4_usleep(1000);
		}

		ASSERT_TRUE(fence.isReadyForPathChecks());
	}

	void TearDown() override { param_reset_all(); }

	void setIntParam(const char *name, int32_t value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set(handle, &value), PX4_OK) << name;
	}

	void writeMissionItems(const std::vector<mission_item_s> &items, dm_item_t dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0)
	{
		for (size_t i = 0; i < items.size(); ++i) {
			mission_item_s item = items[i];
			ASSERT_TRUE(_dataman_client.writeSync(dataman_id, i, reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		}
	}

	// Stores the items with an empty rally set and publishes their mission state.
	mission_s publishMissionItems(const std::vector<mission_item_s> &items, uint32_t mission_id, int32_t current_seq,
				      int32_t land_index = -1, dm_item_t dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0)
	{
		writeMissionItems(items, dataman_id);
		mission_stats_entry_s rally_stats{};
		rally_stats.opaque_id = mission_id;
		rally_stats.dataman_id = DM_KEY_SAFE_POINTS_0;
		EXPECT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_STATE, 0,
						      reinterpret_cast<uint8_t *>(&rally_stats), sizeof(rally_stats)));
		mission_s state{};
		state.timestamp = hrt_absolute_time();
		state.current_seq = current_seq;
		state.land_start_index = land_index;
		state.land_index = land_index;
		state.mission_id = mission_id;
		state.safe_points_id = mission_id;
		state.count = items.size();
		state.mission_dataman_id = dataman_id;
		state.fence_dataman_id = DM_KEY_FENCE_POINTS_0;
		state.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;
		publishMission(state);
		return state;
	}

	void publishMission(const mission_s &mission)
	{
		_mission_pub.publish(mission);
		_mission_state = mission;
	}

	// As commander publishes it: a VTOL reports ROTARY_WING in both transitions.
	void publishVehicleStatus(bool is_vtol, uint8_t vehicle_type, bool in_transition_to_fw = false,
				  bool in_back_transition = false)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.is_vtol = is_vtol;
		status.vehicle_type = vehicle_type;
		status.in_transition_mode = in_transition_to_fw || in_back_transition;
		status.in_transition_to_fw = in_transition_to_fw;
		status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		_vehicle_status_pub.publish(status);
		_vehicle_status = status;
	}

	void publishNavState(uint8_t nav_state)
	{
		_vehicle_status.timestamp = hrt_absolute_time();

		if (_vehicle_status.nav_state != nav_state) {
			_vehicle_status.nav_state_timestamp = _vehicle_status.timestamp;
		}

		_vehicle_status.nav_state = nav_state;
		_vehicle_status_pub.publish(_vehicle_status);
		*_navigator.get_vstatus() = _vehicle_status;
	}

	void publishLandDetected(bool landed)
	{
		vehicle_land_detected_s land_detected{};
		land_detected.timestamp = hrt_absolute_time();
		land_detected.landed = landed;
		_land_detected_pub.publish(land_detected);
		_land_detected = land_detected;
	}

	void publishGlobalPosition(const mission_route::Position &position)
	{
		vehicle_global_position_s global_position{};
		global_position.timestamp = hrt_absolute_time();
		global_position.lat = position.lat;
		global_position.lon = position.lon;
		global_position.alt = position.alt;
		_vehicle_global_position_pub.publish(global_position);
		_global_position = global_position;
	}

	void publishLocalPosition(float heading_rad = 0.f, float vx = 0.f, float vy = 0.f)
	{
		vehicle_local_position_s local_position{};
		local_position.timestamp = hrt_absolute_time();
		local_position.xy_valid = true;
		local_position.v_xy_valid = true;
		local_position.z_valid = true;
		local_position.heading = heading_rad;
		local_position.heading_good_for_control = true;
		local_position.vx = vx;
		local_position.vy = vy;
		_vehicle_local_position_pub.publish(local_position);
		_local_position = local_position;
	}

	void publishHomePosition(const mission_route::Position &position)
	{
		home_position_s home_position{};
		home_position.timestamp = hrt_absolute_time();
		home_position.valid_hpos = true;
		home_position.valid_alt = true;
		home_position.lat = position.lat;
		home_position.lon = position.lon;
		home_position.alt = position.alt;
		_home_position_pub.publish(home_position);
		_home_position = home_position;
	}

	void primeNavigatorState()
	{
		*_navigator.get_vstatus() = _vehicle_status;
		*_navigator.get_land_detected() = _land_detected;
		*_navigator.get_global_position() = _global_position;
		*_navigator.get_local_position() = _local_position;
		*_navigator.get_home_position() = _home_position;
	}

	void markMissionResultValid()
	{
		*_navigator.get_mission_result() = mission_result_s{};
		_navigator.get_mission_result()->valid = true;
		_navigator.get_mission_result()->mission_id = _mission_state.mission_id;
		_navigator.get_mission_result()->geofence_id = _mission_state.geofence_id;
		_navigator.get_mission_result()->home_position_counter = _home_position.update_count;
	}

	void updateRouteCacheUntilReady(const mission_s &mission)
	{
		MissionRouteCache &route_cache = _navigator.get_mission_route_cache();
		ASSERT_TRUE(MissionRouteCacheTestPeer::runCacheUntil(route_cache, mission,
				[&] { return route_cache.missionItemsReady(mission) && route_cache.safePointsReady(); }))
				<< "MissionRouteCache did not become ready within the deterministic cache driver timeout";
	}

	// Activates the mission at item 1, flies it to pause_index and pauses with the vehicle at resume_position.
	void pauseMission(MissionTestPeer &mission, const std::vector<mission_item_s> &items, int32_t pause_index,
			  const mission_route::Position &resume_position)
	{
		const mission_s mission_state = publishMissionItems(items, 71, 1);
		publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
		publishLandDetected(false);
		publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt));
		publishLocalPosition();
		publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt));
		primeNavigatorState();
		updateRouteCacheUntilReady(mission_state);
		mission.on_inactive();
		markMissionResultValid();
		activateWithoutJoin(mission);

		for (int step = 0; (step < 20) && (mission.currentSequenceForTest() < pause_index); ++step) {
			reachCurrentItem(mission);
		}

		ASSERT_EQ(mission.currentSequenceForTest(), pause_index);
		mission.on_inactivation();
		publishGlobalPosition(resume_position);
		publishLocalPosition();
		primeNavigatorState();
		mission.on_inactive();
		markMissionResultValid();
	}

	// A loop around Home: the first leg starts 300 m away, the last leg ends at Home.
	static std::vector<mission_item_s> loopAroundHome()
	{
		return {
			makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 300.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 300.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		};
	}

	// An airborne multicopter at position, flying north.
	void publishAirborneMulticopter(const mission_route::Position &position)
	{
		publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
		publishLandDetected(false);
		publishGlobalPosition(position);
		publishLocalPosition(0.f, 15.f, 0.f);
		publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt));
		primeNavigatorState();
	}

	// Flies the vehicle onto the published setpoint until Mission reports the current item reached.
	void reachCurrentItem(MissionTestPeer &mission)
	{
		const int32_t index = mission.currentSequenceForTest();

		for (int cycle = 0; (cycle < 10) && (_navigator.get_mission_result()->seq_reached != index); ++cycle) {
			const position_setpoint_s &setpoint = _navigator.get_position_setpoint_triplet()->current;

			if (mission_item_contains_position(mission.currentMissionItemForTest()) && setpoint.valid) {
				publishGlobalPosition({setpoint.lat, setpoint.lon, setpoint.alt});
				primeNavigatorState();
			}

			mission.on_active();
		}

		ASSERT_EQ(_navigator.get_mission_result()->seq_reached, index);
	}

	// Flies the vehicle onto the published setpoint until Mission leaves the current work item.
	void reachWorkItem(MissionTestPeer &mission)
	{
		const MissionTestPeer::WorkItemType work_item = mission.workItemTypeForTest();

		for (int cycle = 0; (cycle < 10) && (mission.workItemTypeForTest() == work_item); ++cycle) {
			const position_setpoint_s &setpoint = _navigator.get_position_setpoint_triplet()->current;
			publishGlobalPosition({setpoint.lat, setpoint.lon, setpoint.alt});
			primeNavigatorState();
			mission.on_active();
		}

		ASSERT_NE(mission.workItemTypeForTest(), work_item);
	}

	// Activates Mission at its current item without a join, as a setup step. MIS_ROUTE_JOIN is restored after.
	void activateWithoutJoin(MissionTestPeer &mission)
	{
		int32_t route_join = 0;
		ASSERT_EQ(param_get(param_find("MIS_ROUTE_JOIN"), &route_join), PX4_OK);
		setIntParam("MIS_ROUTE_JOIN", 0);
		mission.refreshParametersForTest();
		mission.on_activation();
		setIntParam("MIS_ROUTE_JOIN", route_join);
		mission.refreshParametersForTest();
	}

	// Reaches the current item, then pauses on the way to the next one at resume_position.
	void startReachAndPause(MissionTestPeer &mission, const mission_route::Position &resume_position)
	{
		activateWithoutJoin(mission);
		reachCurrentItem(mission);
		mission.on_inactivation();
		publishGlobalPosition(resume_position);
		primeNavigatorState();
		mission.on_inactive();
	}

	mission_s _mission_state{};
	Navigator _navigator{};
	DatamanClient _dataman_client{};
	uORB::Publication<mission_s> _mission_pub{ORB_ID(mission)};
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_global_position_s> _vehicle_global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_local_position_s> _vehicle_local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<home_position_s> _home_position_pub{ORB_ID(home_position)};

	vehicle_status_s _vehicle_status{};
	vehicle_land_detected_s _land_detected{};
	vehicle_global_position_s _global_position{};
	vehicle_local_position_s _local_position{};
	home_position_s _home_position{};
};

// A rejoin skipped while the cache loads is reported; once the cache is ready the vehicle joins the
// leg it is next to.
TEST_F(MissionRouteJoinTest, FallsBackUntilTheRouteCacheIsReady)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);

	// A 3-waypoint DO_JUMP loop plus a landing item, vehicle near WP2.
	std::vector<mission_item_s> mission_items = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 300.f, kAlt),
		makeDoJump(0, 2, 2),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f,   0.f, kAlt - 10.f),
	};

	const mission_s mission_state = publishMissionItems(mission_items, 21, 0, 4);
	const mission_route::Position near_wp2 = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 290.f, kAlt);

	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishLandDetected(false);
	publishGlobalPosition(near_wp2);
	publishLocalPosition(0.f, 12.f, 0.f);
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -200.f, 0.f, kAlt));
	primeNavigatorState();

	mission.on_inactive();
	navigator_test::EventRecorder events;

	// Start at WP0 without a join, so nothing is planned while the cache loads. Reach it and pause near WP2.
	startReachAndPause(mission, near_wp2);
	EXPECT_FALSE(events.sent(events::ID("mission_route_rejoin_unavailable")));

	// Resume while the cache is loading.
	mission.on_activation();
	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	// The skipped join is reported, not only logged.
	EXPECT_TRUE(events.sent(events::ID("mission_route_rejoin_unavailable")));

	mission.on_inactivation();
	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();

	mission.on_activation();
	EXPECT_FALSE(events.sent(events::ID("mission_route_rejoin_unavailable")));

	// Rejoin targets WP2 with a valid JOIN_ROUTE work item.
	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_TRUE(mission.joinPositionForTest().valid());
}

// Mission and smart rejoin must agree on whether a DO_JUMP was actually followed.
TEST_F(MissionRouteJoinTest, NominalJumpAnchorMatchesStoredCounterAndRouteRejoin)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 200.f, kAlt),
		makeDoJump(0, 2, 0),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 200.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 500.f, 200.f, kAlt - 20.f),
	};
	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishLandDetected(false);
	// Both the loop edge and the post-loop segment are nearby. Continuity must select
	// the segment actually being flown, rather than the jump predicted before its write.
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 205.f, 190.f, kAlt));
	publishLocalPosition();
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt - 20.f));
	primeNavigatorState();

	for (bool fail_write : {false, true}) {
		SCOPED_TRACE(fail_write);
		const mission_s state = publishMissionItems(items, fail_write ? 52 : 51, 2, 5);
		updateRouteCacheUntilReady(state);
		mission.on_inactive();
		markMissionResultValid();
		mission.beginItemForTest(MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
		mission.setWriteFailureIndexForTest(fail_write ? 3 : -1);

		ASSERT_EQ(mission.currentSequenceForTest(), 2);
		ASSERT_TRUE(mission.advanceMissionForTest());
		EXPECT_EQ(mission.currentSequenceForTest(), fail_write ? 4 : 0);
		EXPECT_EQ(mission.jumpAnchorForTest().jump_item_index, fail_write ? -1 : 3);
		mission_item_s stored_jump{};
		ASSERT_TRUE(_dataman_client.readSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 3,
						     reinterpret_cast<uint8_t *>(&stored_jump), sizeof(stored_jump)));
		EXPECT_EQ(stored_jump.do_jump_current_count, fail_write ? 0 : 1);

		mission.on_activation();
		EXPECT_EQ(mission.currentSequenceForTest(), fail_write ? 4 : 0);
		EXPECT_EQ(mission.jumpAnchorForTest().jump_item_index, fail_write ? -1 : 3);
		EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
		EXPECT_TRUE(mission.joinPositionForTest().valid());
	}
}

// Leaving a loop ends its jump edge, so a later join does not go back to the loop.
TEST_F(MissionRouteJoinTest, LeavingTheLoopEndsItsJumpEdge)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	// A loop 1, 2, back to 1 once, then a return lane 20 m east: 4, 5, 6.
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makeDoJump(1, 1, 0),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 20.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 20.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 20.f, kAlt),
	};
	const mission_s mission_state = publishMissionItems(items, 951, 1);
	publishAirborneMulticopter(makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt));
	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();
	markMissionResultValid();
	activateWithoutJoin(mission);

	// Following the loop back to item 1 anchors its jump.
	reachCurrentItem(mission);
	reachCurrentItem(mission);
	ASSERT_EQ(mission.currentSequenceForTest(), 1);
	ASSERT_EQ(mission.jumpAnchorForTest().jump_item_index, 3);

	// The vehicle leaves the loop onto the return lane.
	for (int reached = 0; reached < 3; ++reached) {
		reachCurrentItem(mission);
	}

	ASSERT_EQ(mission.currentSequenceForTest(), 5);
	EXPECT_TRUE(mission.jumpAnchorForTest().empty());

	// A resume beside the return lane joins that lane, not the loop edge.
	mission.on_inactivation();
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 12.f, kAlt));
	primeNavigatorState();
	mission.on_inactive();
	markMissionResultValid();
	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 5);
	ASSERT_TRUE(mission.joinPositionForTest().valid());
	float north = NAN;
	float east = NAN;
	get_vector_to_next_waypoint(kBaseLat, kBaseLon, mission.joinPositionForTest().lat,
				    mission.joinPositionForTest().lon, &north, &east);
	EXPECT_NEAR(east, 20.f, 1.f);
}

// The jump edge lasts through the commands at the jump target until its position.
TEST_F(MissionRouteJoinTest, JumpEdgeLastsUntilTheTargetPosition)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	// The loop jumps back to the camera trigger at 1, before its position at 2.
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makeCameraTriggerItem(10.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makeDoJump(1, 2, 0),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 100.f, kAlt),
	};
	const mission_s state = publishMissionItems(items, 954, 0);
	publishAirborneMulticopter(makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt));
	updateRouteCacheUntilReady(state);
	mission.on_inactive();
	markMissionResultValid();
	activateWithoutJoin(mission);

	// Items 0 to 3, then the jump back to the camera trigger and on to its position.
	for (int reached = 0; reached < 5; ++reached) {
		reachCurrentItem(mission);
	}

	ASSERT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.jumpAnchorForTest().jump_item_index, 4);
}

// Smart rejoin is off by default: an airborne resume flies to the current item.
TEST_F(MissionRouteJoinTest, DisabledRouteJoinKeepsCurrentItem)
{
	MissionTestPeer mission(&_navigator);
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
	};
	// Beside the leg to item 2, which an enabled join would target.
	const mission_route::Position beside_leg = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 10.f, kAlt);
	const mission_s mission_state = publishMissionItems(items, 63, 0);
	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishLandDetected(false);
	publishGlobalPosition(beside_leg);
	publishLocalPosition();
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt));
	primeNavigatorState();
	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();
	markMissionResultValid();
	startReachAndPause(mission, beside_leg);

	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_NE(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_FALSE(mission.joinPositionForTest().valid());
}

TEST_F(MissionRouteJoinTest, FinishedMissionHoldsWithoutJoin)
{
	// A mission finished in the air holds when Mission is selected again, as without the join.
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
	};
	const mission_s mission_state = publishMissionItems(items, 53, 3);
	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishLandDetected(false);
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt));
	publishLocalPosition();
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt));
	primeNavigatorState();
	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();
	markMissionResultValid();
	activateWithoutJoin(mission);

	// The last item is reached: the mission is finished.
	reachCurrentItem(mission);
	ASSERT_FALSE(mission.plannedItemValidForTest());

	// Mission is selected again from beside the first leg.
	mission.on_inactivation();
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 10.f, kAlt));
	primeNavigatorState();
	mission.on_inactive();
	markMissionResultValid();
	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 3);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_FALSE(mission.joinPositionForTest().valid());
	EXPECT_FALSE(mission.plannedItemValidForTest());
}

// Below the branch-in point the join climbs in place first, as Mission does before a higher item. It climbs only to
// the branch-in altitude, on this ascending leg 10 m: the rest is climbed along the leg. From above, it flies straight
// to the branch-in point, without a descent in place.
TEST_F(MissionRouteJoinTest, JoinClimbsInPlaceToTheBranchInAltitude)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt + 40.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt + 40.f),
	};
	const position_setpoint_triplet_s &triplet = *_navigator.get_position_setpoint_triplet();
	uint32_t mission_id = 61;

	for (const bool below : {true, false}) {
		SCOPED_TRACE(below ? "below the branch-in point" : "above the branch-in point");
		MissionTestPeer mission(&_navigator);
		const mission_route::Position beside_leg = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 10.f,
				below ? kAlt - 20.f : kAlt + 30.f);
		const mission_s mission_state = publishMissionItems(items, ++mission_id, 1);
		publishAirborneMulticopter(beside_leg);
		updateRouteCacheUntilReady(mission_state);
		mission.on_inactive();
		markMissionResultValid();
		startReachAndPause(mission, beside_leg);

		mission.on_activation();

		ASSERT_EQ(mission.currentSequenceForTest(), 2);
		const mission_route::Position join = mission.joinPositionForTest();
		ASSERT_TRUE(join.valid());
		EXPECT_NEAR(join.alt, kAlt + 10.f, 1.f);

		if (below) {
			// 1. Climb at the current position, with the branch-in waypoint next.
			ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_CLIMB);
			const mission_item_s &climb = mission.currentMissionItemForTest();
			EXPECT_EQ(climb.nav_cmd, NAV_CMD_LOITER_TO_ALT);
			EXPECT_DOUBLE_EQ(climb.lat, beside_leg.lat);
			EXPECT_DOUBLE_EQ(climb.lon, beside_leg.lon);
			EXPECT_FLOAT_EQ(climb.altitude, join.alt);
			EXPECT_DOUBLE_EQ(triplet.current.lat, beside_leg.lat);
			ASSERT_TRUE(triplet.next.valid);
			EXPECT_DOUBLE_EQ(triplet.next.lat, join.lat);
			EXPECT_DOUBLE_EQ(triplet.next.lon, join.lon);
			EXPECT_FLOAT_EQ(triplet.next.alt, join.alt);
			reachWorkItem(mission);
		}

		// 2. The branch-in waypoint, with the joined item next.
		ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
		EXPECT_DOUBLE_EQ(triplet.current.lat, join.lat);
		EXPECT_DOUBLE_EQ(triplet.current.lon, join.lon);
		EXPECT_FLOAT_EQ(triplet.current.alt, join.alt);
		EXPECT_DOUBLE_EQ(triplet.next.lat, items[2].lat);
		reachWorkItem(mission);

		// 3. The joined item at its own altitude. Neither the climb nor the branch-in point reports an item reached.
		EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
		EXPECT_FALSE(mission.joinPositionForTest().valid());
		EXPECT_EQ(mission.currentSequenceForTest(), 2);
		EXPECT_DOUBLE_EQ(triplet.current.lat, items[2].lat);
		EXPECT_FLOAT_EQ(triplet.current.alt, items[2].altitude);
		EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
		EXPECT_EQ(_navigator.get_mission_result()->seq_reached, -1);
	}
}

// The joined item is the next setpoint of the branch-in waypoint, so a multicopter does not stop there.
TEST_F(MissionRouteJoinTest, JoinWaypointHasTheJoinedItemNext)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
	};
	const mission_route::Position beside_leg = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 40.f, kAlt);
	const mission_s mission_state = publishMissionItems(items, 62, 0);
	publishAirborneMulticopter(beside_leg);
	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();
	markMissionResultValid();
	startReachAndPause(mission, beside_leg);

	mission.on_activation();

	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	ASSERT_EQ(mission.currentSequenceForTest(), 2);
	const position_setpoint_triplet_s &triplet = *_navigator.get_position_setpoint_triplet();
	EXPECT_DOUBLE_EQ(triplet.current.lat, mission.joinPositionForTest().lat);
	EXPECT_DOUBLE_EQ(triplet.current.lon, mission.joinPositionForTest().lon);
	ASSERT_TRUE(triplet.next.valid);
	EXPECT_DOUBLE_EQ(triplet.next.lat, items[2].lat);
	EXPECT_DOUBLE_EQ(triplet.next.lon, items[2].lon);
	EXPECT_FLOAT_EQ(triplet.next.alt, items[2].altitude);
}

TEST_F(MissionRouteJoinTest, RestartedMissionCanJoinWhileAirborne)
{
	// Restart still resets the mission, but an airborne activation can join near its end.
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
	};
	const mission_s mission_state = publishMissionItems(items, 52, 3);
	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishLandDetected(false);
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 10.f, kAlt));
	publishLocalPosition();
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt));
	primeNavigatorState();
	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();
	markMissionResultValid();
	mission.setDisarmedSinceActivationForTest();

	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 3);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_TRUE(mission.joinPositionForTest().valid());
}

// The first airborne start can join a later leg, including after an upload or a reboot.
TEST_F(MissionRouteJoinTest, NeverActivatedMissionJoinsNearestLegWhileAirborne)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	const mission_s mission_state = publishMissionItems(loopAroundHome(), 54, 0);
	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishLandDetected(false);
	// Taken off in Position mode, 5 m beside the last leg.
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 5.f, 150.f, kAlt));
	publishLocalPosition();
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt));
	primeNavigatorState();
	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();
	markMissionResultValid();
	navigator_test::EventRecorder events;

	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 3);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_TRUE(mission.joinPositionForTest().valid());
	EXPECT_FALSE(events.sent(events::ID("mission_route_rejoin_unavailable")));

	// A pause before reaching a mission item does not prevent another join.
	mission.on_inactivation();
	mission.on_inactive();
	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 3);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_TRUE(mission.joinPositionForTest().valid());
}

TEST_F(MissionRouteJoinTest, GroundStartKeepsCurrentItemThenAirborneResumeCanJoin)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	const mission_s state = publishMissionItems(loopAroundHome(), 55, 0);
	publishAirborneMulticopter(makePositionFromOffset(kBaseLat, kBaseLon, 5.f, 150.f, kAlt));
	publishLandDetected(true);
	primeNavigatorState();
	updateRouteCacheUntilReady(state);
	mission.on_inactive();
	markMissionResultValid();
	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 0);
	EXPECT_FALSE(mission.joinPositionForTest().valid());

	// Pause before reaching the first mission position item, then resume after a manual takeoff.
	mission.on_inactivation();
	publishLandDetected(false);
	primeNavigatorState();
	mission.on_inactive();
	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 3);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_TRUE(mission.joinPositionForTest().valid());
}

// After landing and disarming, another airborne activation can join a later leg.
TEST_F(MissionRouteJoinTest, NewFlightAfterLandingJoinsNearestLeg)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	const mission_s mission_state = publishMissionItems(loopAroundHome(), 57, 0);
	const mission_route::Position beside_last_leg = makePositionFromOffset(kBaseLat, kBaseLon, 5.f, 150.f, kAlt);
	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishLandDetected(false);
	publishGlobalPosition(beside_last_leg);
	publishLocalPosition();
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt));
	primeNavigatorState();
	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();
	markMissionResultValid();
	startReachAndPause(mission, beside_last_leg);
	ASSERT_EQ(mission.currentSequenceForTest(), 1);

	// Landed and disarmed while Mission is inactive, e.g. after a Return.
	_vehicle_status.timestamp = hrt_absolute_time();
	_vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	_vehicle_status_pub.publish(_vehicle_status);
	publishLandDetected(true);
	primeNavigatorState();
	mission.on_inactive();

	// Re-armed and taken off in Position mode beside the last leg, then Mission.
	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishLandDetected(false);
	primeNavigatorState();
	mission.on_inactive();
	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 3);
	EXPECT_TRUE(mission.joinPositionForTest().valid());
}

// An item selected while Mission is paused is the current item of the join, like any resume.
TEST_F(MissionRouteJoinTest, ItemSelectedWhilePausedJoins)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	// Out on 0, 1, 2 and back 20 m east on 3, 4, 5.
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 20.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 20.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 20.f, kAlt),
	};
	// Between the lanes, closer to the way back.
	const mission_route::Position between_lanes = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 12.f, kAlt);
	publishAirborneMulticopter(between_lanes);
	uint32_t mission_id = 77;

	// Item 1 reached, then paused between the lanes.
	const auto start_and_pause = [&]() {
		const mission_s state = publishMissionItems(items, ++mission_id, 1);
		updateRouteCacheUntilReady(state);
		mission.on_inactive();
		markMissionResultValid();
		startReachAndPause(mission, between_lanes);
		return state;
	};

	// Without a selection, the resume joins the leg it was flying.
	start_and_pause();
	mission.on_activation();
	ASSERT_EQ(mission.currentSequenceForTest(), 2);
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	mission.on_inactivation();

	// Selecting 4 makes the way back the leg to join. DO_SET_MISSION_CURRENT and MISSION_START select
	// through set_current_mission_index(), MISSION_SET_CURRENT through the mission topic.
	for (const bool from_mission_topic : {false, true}) {
		SCOPED_TRACE(from_mission_topic);
		const mission_s state = start_and_pause();

		if (from_mission_topic) {
			mission_s selected = state;
			selected.timestamp = hrt_absolute_time();
			selected.current_seq = 4;
			publishMission(selected);
			mission.on_inactive();

		} else {
			ASSERT_TRUE(mission.set_current_mission_index(4));
		}

		mission.on_activation();

		EXPECT_EQ(mission.currentSequenceForTest(), 4);
		EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
		mission.on_inactivation();
	}
}

// A selection while Mission is active flies to the item directly: it cancels a pending join and ends the jump edge.
// So does a jump counter reset, which keeps the current item.
TEST_F(MissionRouteJoinTest, SelectionInFlightCancelsJoinAndJumpEdge)
{
	// A loop 1, 2, back to 1 once, then a return lane 20 m east: 4, 5, 6.
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makeDoJump(1, 1, 0),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 20.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 20.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 20.f, kAlt),
	};
	// The joined item, given as its index.
	static constexpr int32_t kJoinedItem{-2};
	uint32_t mission_id = 952;

	// Select item 5. Reset the jump counters at the joined item, given as -1 or as its index.
	for (const int32_t selected : {int32_t{5}, int32_t{-1}, kJoinedItem}) {
		SCOPED_TRACE(selected);
		setIntParam("MIS_ROUTE_JOIN", 0);
		MissionTestPeer mission(&_navigator);
		const mission_s mission_state = publishMissionItems(items, ++mission_id, 1);
		publishAirborneMulticopter(makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt));
		updateRouteCacheUntilReady(mission_state);

		// Run Mission as Navigator does, so that it is active while it handles the selection. Start without a join.
		mission.run(false);
		markMissionResultValid();
		mission.run(true);
		reachCurrentItem(mission);
		reachCurrentItem(mission);
		ASSERT_EQ(mission.jumpAnchorForTest().jump_item_index, 3);

		// Paused beside the jump edge, then resumed with the join on.
		setIntParam("MIS_ROUTE_JOIN", 1);
		mission.refreshParametersForTest();
		mission.run(false);
		publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 150.f, -15.f, kAlt));
		primeNavigatorState();
		mission.run(false);
		mission.run(true);
		ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
		ASSERT_EQ(mission.jumpAnchorForTest().jump_item_index, 3);
		const int32_t joined_index = mission.currentSequenceForTest();

		const bool reset_jump_counters = selected != 5;
		ASSERT_TRUE(mission.set_current_mission_index(selected == kJoinedItem ? joined_index : selected,
				reset_jump_counters));

		const int32_t flown_index = reset_jump_counters ? joined_index : selected;
		EXPECT_EQ(mission.currentSequenceForTest(), flown_index);
		EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
		EXPECT_FALSE(mission.joinPositionForTest().valid());
		EXPECT_TRUE(mission.jumpAnchorForTest().empty());
		EXPECT_DOUBLE_EQ(_navigator.get_position_setpoint_triplet()->current.lat, items[flown_index].lat);
		EXPECT_DOUBLE_EQ(_navigator.get_position_setpoint_triplet()->current.lon, items[flown_index].lon);
		mission_item_s stored_jump{};
		ASSERT_TRUE(_dataman_client.readSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 3,
						     reinterpret_cast<uint8_t *>(&stored_jump), sizeof(stored_jump)));
		EXPECT_EQ(stored_jump.do_jump_current_count, reset_jump_counters ? 0 : 1);
	}
}

// A Return along the mission moves the index, and continuing the mission rejoins the route.
TEST_F(MissionRouteJoinTest, ResumeAfterReturnAlongTheMissionRejoins)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	std::vector<mission_item_s> items;

	for (int index = 0; index <= 6; ++index) {
		items.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, index * 100.f, 0.f, kAlt));
	}

	const mission_route::Position beside_leg_4_5 = makePositionFromOffset(kBaseLat, kBaseLon, 450.f, 5.f, kAlt);
	publishAirborneMulticopter(beside_leg_4_5);
	const mission_s state = publishMissionItems(items, 81, 4);
	updateRouteCacheUntilReady(state);
	publishNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	mission.on_inactive();
	markMissionResultValid();
	activateWithoutJoin(mission);
	reachCurrentItem(mission);
	ASSERT_EQ(mission.currentSequenceForTest(), 5);

	// RTL_TYPE 2 without a mission landing returns along the reversed mission and moves the index back.
	RtlMissionFastReverse rtl(&_navigator, state);
	rtl.on_inactive();
	publishNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
	publishGlobalPosition(beside_leg_4_5);
	primeNavigatorState();
	mission.on_inactivation();
	rtl.on_activation();
	mission.on_inactive();
	ASSERT_EQ(mission.currentSequenceForTest(), 4);

	// The operator continues the mission.
	rtl.on_inactivation();
	publishNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 5);
	EXPECT_TRUE(mission.joinPositionForTest().valid());
}

// With the join, a survey paused while the camera triggers is joined like any mission, not flown back to the previous
// survey waypoint. The camera trigger restarts once the joined item is reached.
TEST_F(MissionRouteJoinTest, PausedCameraSurveyIsJoined)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makeCameraTriggerItem(10.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt),
	};

	// Paused on the survey leg 3-4 while the camera triggers, resumed beside that leg.
	pauseMission(mission, items, 4, makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 10.f, kAlt));
	uORB::Subscription commands{ORB_ID(vehicle_command)};
	vehicle_command_s command{};
	const auto trigger_sent = [&]() {
		bool sent = false;

		while (commands.update(&command)) {
			sent |= command.command == NAV_CMD_DO_SET_CAM_TRIGG_DIST;
		}

		return sent;
	};

	while (commands.update(&command)) {}

	mission.on_activation();

	// Not rewound to item 3: the join flies its branch-in point on leg 3-4.
	EXPECT_EQ(mission.currentSequenceForTest(), 4);
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);

	// No heading alignment of a rewind is pending: facing east, one cycle at the branch-in point flies on to item 4.
	publishGlobalPosition(mission.joinPositionForTest());
	publishLocalPosition(1.5f);
	primeNavigatorState();
	mission.on_active();
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);

	// The camera trigger restarts once item 4 is reached, not before.
	publishLocalPosition();
	reachCurrentItem(mission);
	EXPECT_FALSE(trigger_sent());
	mission.on_active();
	EXPECT_TRUE(trigger_sent());
}

// A resume within the acceptance radius of the paused landing lands there directly. Farther away,
// the vehicle climbs in place back to the approach altitude and rejoins the route.
TEST_F(MissionRouteJoinTest, PausedLandingLandsNearbyOrClimbsBackToTheApproach)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	uORB::Subscription commands{ORB_ID(vehicle_command)};
	vehicle_command_s command{};
	const float descent_altitude = kAlt + 2.f;
	const float approach_altitude = kAlt + 50.f;
	uint32_t mission_id = 431;

	// Resume at LAND, inside its radius, just outside it, or after a GoTo near an earlier leg.
	for (int resume_location = 0; resume_location < 4; ++resume_location) {
		SCOPED_TRACE(::testing::Message() << "resume_location=" << resume_location);
		auto takeoff = makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, approach_altitude);
		auto land = makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt);
		land.yaw = NAN;

		const std::vector<mission_item_s> items{
			takeoff, makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, approach_altitude), land
		};
		const mission_s state = publishMissionItems(items, ++mission_id, 1, 2);
		publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
		_vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		_vehicle_status_pub.publish(_vehicle_status);
		publishLandDetected(false);
		publishGlobalPosition({items[1].lat, items[1].lon, approach_altitude});
		publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt));
		publishLocalPosition();
		primeNavigatorState();
		updateRouteCacheUntilReady(state);
		mission.on_inactive();
		markMissionResultValid();
		// Fly the approach waypoint and the move to the landing point, then descend.
		activateWithoutJoin(mission);
		reachCurrentItem(mission);
		reachCurrentItem(mission);
		publishGlobalPosition({land.lat, land.lon, descent_altitude});
		primeNavigatorState();
		auto &sp = _navigator.get_position_setpoint_triplet()->current;
		ASSERT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_LAND);

		mission.on_inactivation();

		if (resume_location == 3) {
			publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 50.f, descent_altitude));

		} else {
			const float radius_fraction = resume_location == 0 ? 0.f : (resume_location == 1 ? 0.5f : 1.5f);
			publishGlobalPosition(makePositionFromOffset(land.lat, land.lon,
					      -radius_fraction * _navigator.get_acceptance_radius(), 0.f, descent_altitude));
		}

		primeNavigatorState();
		mission.on_inactive();

		while (commands.update(&command)) {}

		mission.on_activation();
		// After the GoTo the nearest leg is the earlier one, and Mission continues from there.
		const bool near_land = resume_location < 2;
		const int32_t target_index = resume_location == 3 ? 1 : 2;
		EXPECT_EQ(mission.currentSequenceForTest(), target_index);

		if (near_land) {
			EXPECT_FALSE(mission.joinPositionForTest().valid());
			EXPECT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_LAND);
			EXPECT_DOUBLE_EQ(sp.lat, land.lat);
			EXPECT_DOUBLE_EQ(sp.lon, land.lon);

		} else {
			// Climbs in place back to the approach altitude first.
			ASSERT_TRUE(mission.joinPositionForTest().valid());
			EXPECT_FLOAT_EQ(mission.joinPositionForTest().alt, approach_altitude);
			EXPECT_EQ(mission.currentMissionItemForTest().nav_cmd, NAV_CMD_LOITER_TO_ALT);
			EXPECT_FLOAT_EQ(mission.currentMissionItemForTest().altitude, approach_altitude);
		}

		// At the paused descent altitude the vehicle keeps landing, or keeps climbing.
		for (int i = 0; i < 3; ++i) {
			mission.on_active();
			EXPECT_EQ(mission.workItemTypeForTest(), near_land ? MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT
				  : MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_CLIMB);
			EXPECT_EQ(sp.type, near_land ? position_setpoint_s::SETPOINT_TYPE_LAND : position_setpoint_s::SETPOINT_TYPE_LOITER);
			EXPECT_EQ(mission.currentSequenceForTest(), target_index);
		}

		if (!near_land) {
			// Then it flies to the branch-in point at the approach altitude.
			reachWorkItem(mission);
			EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
			EXPECT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
			EXPECT_FLOAT_EQ(sp.alt, approach_altitude);
		}

		while (commands.update(&command)) {
			EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
		}
	}
}

// A resume beside a landing that is not the paused item lands there directly, without a join waypoint.
TEST_F(MissionRouteJoinTest, NearbyLandingIsSelectedWithoutJoin)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	std::vector<mission_item_s> items = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, -300.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt - 10.f),
	};
	items[3].yaw = NAN;
	const mission_s mission_state = publishMissionItems(items, 23, 1, 3);
	// 2 m from LAND and 6 m below the approach, while item 2 is the current item.
	const mission_route::Position near_land = makePositionFromOffset(kBaseLat, kBaseLon, 298.f, 0.f, kAlt - 6.f);
	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishLandDetected(false);
	publishGlobalPosition(near_land);
	publishLocalPosition(0.f, 8.f, 0.f);
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -100.f, 0.f, kAlt - 10.f));
	primeNavigatorState();
	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();
	markMissionResultValid();
	startReachAndPause(mission, near_land);
	ASSERT_EQ(mission.currentSequenceForTest(), 2);

	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 3);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_FALSE(mission.joinPositionForTest().valid());
	const position_setpoint_s &setpoint = _navigator.get_position_setpoint_triplet()->current;
	EXPECT_EQ(setpoint.type, position_setpoint_s::SETPOINT_TYPE_LAND);
	EXPECT_DOUBLE_EQ(setpoint.lat, items[3].lat);
	EXPECT_DOUBLE_EQ(setpoint.lon, items[3].lon);
}

// A feasibility check during the join, for example after a fence upload, keeps the join. A check during the climb
// before the branch-in point still climbs to the branch-in altitude, not to the altitude of the item.
TEST_F(MissionRouteJoinTest, FeasibilityRefreshKeepsJoin)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	// The leg to item 2 climbs 40 m, so its branch-in point beside the vehicle lies 10 m above item 1.
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt + 40.f),
	};
	uint32_t mission_id = 301;

	for (const bool climbing : {false, true}) {
		SCOPED_TRACE(climbing ? "during the climb" : "during the branch-in waypoint");
		MissionTestPeer mission(&_navigator);
		const mission_route::Position beside_leg = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 40.f,
				climbing ? kAlt - 20.f : kAlt + 10.f);
		const mission_s mission_state = publishMissionItems(items, ++mission_id, 0);
		publishAirborneMulticopter(beside_leg);
		updateRouteCacheUntilReady(mission_state);
		mission.on_inactive();
		markMissionResultValid();
		startReachAndPause(mission, beside_leg);

		// run() makes isActive() true, as for the feasibility refresh in flight.
		mission.run(true);
		const MissionTestPeer::WorkItemType work_item = climbing ? MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_CLIMB
				: MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE;
		ASSERT_EQ(mission.workItemTypeForTest(), work_item);
		ASSERT_EQ(mission.currentSequenceForTest(), 2);
		const mission_item_s flown = mission.currentMissionItemForTest();

		geofence_status_s fence_status{};
		fence_status.timestamp = hrt_absolute_time();
		fence_status.geofence_id = mission_state.geofence_id;
		fence_status.status = geofence_status_s::GF_STATUS_READY;
		uORB::Publication<geofence_status_s> fence_status_pub{ORB_ID(geofence_status)};
		fence_status_pub.publish(fence_status);
		mission.requestFeasibilityCheckForTest();

		mission.run(true);

		EXPECT_EQ(mission.workItemTypeForTest(), work_item);
		EXPECT_EQ(mission.currentSequenceForTest(), 2);
		EXPECT_DOUBLE_EQ(mission.currentMissionItemForTest().lat, flown.lat);
		EXPECT_DOUBLE_EQ(mission.currentMissionItemForTest().lon, flown.lon);
		EXPECT_FLOAT_EQ(mission.currentMissionItemForTest().altitude, flown.altitude);
		EXPECT_NEAR(flown.altitude, kAlt + 10.f, 1.f);
	}
}

TEST_F(MissionRouteJoinTest, ExhaustedJumpRejoinResumesNominalExecutionWithoutAnotherRepeat)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	const std::vector<mission_item_s> items{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 200.f, kAlt),
		makeDoJump(0, 3, 2),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 200.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 500.f, 200.f, kAlt - 20.f),
	};
	const mission_s mission_state = publishMissionItems(items, 41, 2, 5);
	const mission_route::Position on_loop_edge = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt);
	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishLandDetected(false);
	publishGlobalPosition(on_loop_edge);
	publishLocalPosition();
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt - 20.f));
	primeNavigatorState();
	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();
	// Reaching waypoint 2 follows the last repeat back to item 0; paused on that loop edge.
	startReachAndPause(mission, on_loop_edge);
	ASSERT_EQ(mission.currentSequenceForTest(), 0);
	ASSERT_EQ(mission.jumpAnchorForTest().jump_item_index, 3);

	mission.on_activation();

	ASSERT_EQ(mission.currentSequenceForTest(), 2);
	ASSERT_TRUE(mission.joinPositionForTest().valid());
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);

	publishGlobalPosition(mission.joinPositionForTest());
	primeNavigatorState();
	mission.on_active();

	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, -1);
	EXPECT_DOUBLE_EQ(_navigator.get_position_setpoint_triplet()->current.lat, items[2].lat);

	publishGlobalPosition({items[2].lat, items[2].lon, items[2].altitude});
	primeNavigatorState();
	mission.on_active();

	EXPECT_EQ(mission.currentSequenceForTest(), 4);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 2);
	EXPECT_DOUBLE_EQ(_navigator.get_position_setpoint_triplet()->current.lat, items[4].lat);
	mission_item_s exhausted_jump{};
	ASSERT_TRUE(_dataman_client.readSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 3,
					     reinterpret_cast<uint8_t *>(&exhausted_jump), sizeof(exhausted_jump)));
	EXPECT_EQ(exhausted_jump.do_jump_current_count, 3);
}

// A vehicle without the join, as commander publishes it: a VTOL reports ROTARY_WING in hover and in both
// transitions. The multicopter row has MIS_ROUTE_JOIN = 0; the other rows also fly with MIS_ROUTE_JOIN = 1.
struct UntouchedVehicle {
	const char *name;
	bool is_vtol;
	uint8_t vehicle_type;
	bool in_transition_to_fw;
	bool in_back_transition;
	bool also_with_join_param;
};

// Row names instead of raw bytes in failure messages.
static void PrintTo(const UntouchedVehicle &vehicle, std::ostream *os)
{
	*os << vehicle.name;
}

class MissionRouteJoinUntouchedTest : public MissionRouteJoinTest, public ::testing::WithParamInterface<UntouchedVehicle>
{
protected:
	// The mission index, the work item and the published triplet.
	std::string snapshot(const MissionTestPeer &mission)
	{
		const position_setpoint_triplet_s &triplet = *_navigator.get_position_setpoint_triplet();
		std::string line = "seq " + std::to_string(mission.currentSequenceForTest())
				   + " work " + std::to_string(static_cast<int>(mission.workItemTypeForTest()));

		for (const position_setpoint_s *setpoint : {&triplet.previous, &triplet.current, &triplet.next}) {
			char text[128];
			snprintf(text, sizeof(text), " | %d %d %.7f %.7f %.2f %.3f", setpoint->valid, setpoint->type, setpoint->lat,
				 setpoint->lon, static_cast<double>(setpoint->alt), static_cast<double>(setpoint->yaw));
			line += text;
		}

		return line;
	}

	// Flies items 1 to 4, follows the loop back to 3, pauses beside the first leg and resumes there.
	// With the camera on, the paused item is selected again before the resume.
	void flyPauseAndResume(bool join, bool camera_and_reselect, uint32_t mission_id, std::vector<std::string> &trace)
	{
		const UntouchedVehicle &vehicle = GetParam();
		setIntParam("MIS_ROUTE_JOIN", join ? 1 : 0);
		MissionTestPeer mission(&_navigator);
		const std::vector<mission_item_s> items{
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
			makeCameraTriggerItem(camera_and_reselect ? 10.f : 0.f),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
			makeDoJump(3, 1, 0),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt),
		};
		const mission_s state = publishMissionItems(items, mission_id, 1);
		publishVehicleStatus(vehicle.is_vtol, vehicle.vehicle_type, vehicle.in_transition_to_fw, vehicle.in_back_transition);
		publishLandDetected(false);
		publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt));
		publishLocalPosition();
		publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt));
		primeNavigatorState();
		updateRouteCacheUntilReady(state);
		mission.on_inactive();
		markMissionResultValid();
		navigator_test::EventRecorder rejoin_events;
		navigator_test::EventRecorder unavailable_events;

		const auto record = [&]() {
			EXPECT_NE(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
			trace.push_back(snapshot(mission));
		};

		ASSERT_TRUE(mission.set_current_mission_index(1));
		mission.on_activation();
		record();

		// Reaching item 4 follows the loop back to item 3.
		for (const int32_t index : {1, 2, 3, 4}) {
			ASSERT_EQ(mission.currentSequenceForTest(), index);
			reachCurrentItem(mission);
			record();
		}

		ASSERT_EQ(mission.currentSequenceForTest(), 3);

		// Pause on the loop edge, then resume beside the first leg, where a multicopter would join.
		mission.on_inactivation();
		publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 10.f, kAlt));
		primeNavigatorState();
		mission.on_inactive();

		if (camera_and_reselect) {
			ASSERT_TRUE(mission.set_current_mission_index(3));
			mission.on_inactive();
		}

		mission.on_activation();
		record();

		// Without the camera, Mission resumes at the paused item; with it, at the position item before it.
		EXPECT_EQ(mission.currentSequenceForTest(), camera_and_reselect ? 1 : 3);

		for (int cycle = 0; cycle < 3; ++cycle) {
			mission.on_active();
			record();
		}

		EXPECT_FALSE(rejoin_events.sent(events::ID("mission_route_rejoin")));
		EXPECT_FALSE(unavailable_events.sent(events::ID("mission_route_rejoin_unavailable")));
	}

	void publishArmed(bool armed)
	{
		_vehicle_status.timestamp = hrt_absolute_time();
		_vehicle_status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
		_vehicle_status_pub.publish(_vehicle_status);
		*_navigator.get_vstatus() = _vehicle_status;
	}

	// Selections, uploads and a restart, run through NavigatorMode::run() so that isActive() is real.
	// The expected indices are main's for the same steps.
	void flyMainPath(uint32_t mission_id)
	{
		const UntouchedVehicle &vehicle = GetParam();
		setIntParam("MIS_ROUTE_JOIN", vehicle.also_with_join_param ? 1 : 0);
		MissionTestPeer mission(&_navigator);
		const auto lane = [](float east_m) {
			std::vector<mission_item_s> items;

			for (int index = 0; index < 4; ++index) {
				items.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, index * 100.f, east_m, kAlt));
			}

			return items;
		};
		const auto select = [&](int32_t seq) {
			mission_s selected = _mission_state;
			selected.timestamp = hrt_absolute_time();
			selected.current_seq = seq;
			publishMission(selected);
		};

		// The first mission while the route cache holds no mission: no join and no warning.
		publishMissionItems(lane(0.f), mission_id, 0);
		publishVehicleStatus(vehicle.is_vtol, vehicle.vehicle_type, vehicle.in_transition_to_fw, vehicle.in_back_transition);
		publishNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
		publishLandDetected(false);
		publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 10.f, kAlt));
		publishLocalPosition();
		publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt));
		primeNavigatorState();
		mission.run(false);
		markMissionResultValid();
		navigator_test::EventRecorder unavailable_events;
		publishNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
		mission.run(true);
		EXPECT_EQ(mission.currentSequenceForTest(), 0);
		EXPECT_FALSE(unavailable_events.sent(events::ID("mission_route_rejoin_unavailable")));

		// Selecting the current item while active, by MAVLink and by command, does not replan.
		reachCurrentItem(mission);
		mission.run(true);
		const std::string flying = snapshot(mission);
		select(1);
		mission.run(true);
		EXPECT_EQ(snapshot(mission), flying);
		ASSERT_TRUE(mission.set_current_mission_index(1));
		mission.run(true);
		EXPECT_EQ(snapshot(mission), flying);

		// MISSION_SET_CURRENT(3) and DO_SET_MISSION_CURRENT(1) in the same cycle: Mission flies 3.
		select(3);
		ASSERT_TRUE(mission.set_current_mission_index(1));
		mission.run(true);
		EXPECT_EQ(mission.currentSequenceForTest(), 3);

		// An upload while active flies the new mission from its current item.
		publishMissionItems(lane(100.f), mission_id + 1, 2, -1, DM_KEY_WAYPOINTS_OFFBOARD_1);
		mission.run(true);
		EXPECT_EQ(mission.currentSequenceForTest(), 2);

		// Finished, landed and disarmed while paused, then airborne again: Mission restarts at item 0.
		reachCurrentItem(mission);
		reachCurrentItem(mission);
		publishNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
		mission.run(false);
		publishArmed(false);
		publishLandDetected(true);
		primeNavigatorState();
		mission.run(false);
		mission.run(false);
		publishArmed(true);
		publishLandDetected(false);
		primeNavigatorState();
		mission.run(false);
		publishNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
		mission.run(true);
		EXPECT_EQ(mission.currentSequenceForTest(), 0);

		// An upload in the cycle that switches to Mission is read one cycle later.
		reachCurrentItem(mission);
		publishNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
		mission.run(false);
		mission.run(false);
		publishMissionItems(lane(200.f), mission_id + 2, 3);
		publishNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
		mission.run(true);
		EXPECT_EQ(mission.currentSequenceForTest(), 1);
		mission.run(true);
		EXPECT_EQ(mission.currentSequenceForTest(), 3);
	}
};

// Vehicles without the join run Mission as on boards without the mission cache, whatever MIS_ROUTE_JOIN is.
TEST_P(MissionRouteJoinUntouchedTest, MissionRunsAsWithoutTheJoin)
{
	uint32_t mission_id = 300;

	for (const bool camera_and_reselect : {false, true}) {
		SCOPED_TRACE(camera_and_reselect ? "camera on, paused item selected again" : "plain resume");
		std::vector<std::string> without_join;
		flyPauseAndResume(false, camera_and_reselect, ++mission_id, without_join);

		if (GetParam().also_with_join_param) {
			std::vector<std::string> with_join;
			flyPauseAndResume(true, camera_and_reselect, ++mission_id, with_join);
			EXPECT_EQ(with_join, without_join);
		}
	}

	SCOPED_TRACE("selections, uploads and a restart");
	flyMainPath(400);
}

static std::string untouchedVehicleName(const ::testing::TestParamInfo<UntouchedVehicle> &row)
{
	return row.param.name;
}

INSTANTIATE_TEST_SUITE_P(Vehicles, MissionRouteJoinUntouchedTest, ::testing::Values(
				 UntouchedVehicle{"FixedWing", false, vehicle_status_s::VEHICLE_TYPE_FIXED_WING, false, false, true},
				 UntouchedVehicle{"VtolHover", true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false, false, true},
				 UntouchedVehicle{"VtolFrontTransition", true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true, false, true},
				 UntouchedVehicle{"VtolFixedWing", true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING, false, false, true},
				 UntouchedVehicle{"VtolBackTransition", true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false, true, true},
				 UntouchedVehicle{"Rover", false, vehicle_status_s::VEHICLE_TYPE_ROVER, false, false, true},
				 UntouchedVehicle{"Unspecified", false, vehicle_status_s::VEHICLE_TYPE_UNSPECIFIED, false, false, true},
				 UntouchedVehicle{"MulticopterJoinOff", false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false, false, false}),
			 untouchedVehicleName);
