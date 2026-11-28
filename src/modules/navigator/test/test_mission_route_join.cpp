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

	using MissionBase::WorkItemType;

	const mission_route::ActiveJumpAnchor &jumpAnchorForTest() const { return _active_jump_anchor; }
	bool plannedItemValidForTest() const { return _is_current_planned_mission_item_valid; }

	// Read changed parameters now, instead of on the next parameter_update.
	void refreshParametersForTest() { updateParams(); }

	// A pending check, as after a geofence upload; on_active() completes it with the current verdict.
	void requestFeasibilityCheckForTest() { _mission_check_state = MissionCheckState::Unchecked; }
	const mission_item_s &currentMissionItemForTest() const { return _mission_item; }

	WorkItemType workItemTypeForTest() const { return _work_item_type; }
	int32_t currentSequenceForTest() const { return _mission.current_seq; }
	const mission_route::Position &joinPositionForTest() const { return _route_join_position; }
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

	// Publishes the mission, fills the route cache with it and lets Mission read it while inactive.
	mission_s loadMission(MissionTestPeer &mission, const std::vector<mission_item_s> &items, uint32_t mission_id,
			      int32_t current_seq, int32_t land_index = -1)
	{
		const mission_s state = publishMissionItems(items, mission_id, current_seq, land_index);
		updateRouteCacheUntilReady(state);
		mission.on_inactive();
		markMissionResultValid();
		return state;
	}

	// Activates the mission at item 1, flies it to pause_index and pauses with the vehicle at resume_position.
	void pauseMission(MissionTestPeer &mission, const std::vector<mission_item_s> &items, int32_t pause_index,
			  const mission_route::Position &resume_position)
	{
		publishAirborneMulticopter(makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt));
		loadMission(mission, items, 71, 1);
		activateWithoutJoin(mission);

		for (int step = 0; (step < 20) && (mission.currentSequenceForTest() < pause_index); ++step) {
			reachCurrentItem(mission);
		}

		ASSERT_EQ(mission.currentSequenceForTest(), pause_index);
		pauseAt(mission, resume_position);
	}

	// Pauses Mission and moves the vehicle to position, ready for the next activation.
	void pauseAt(MissionTestPeer &mission, const mission_route::Position &position)
	{
		mission.on_inactivation();
		publishGlobalPosition(position);
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

	// Out on 0, 1, 2 and back 20 m east on the last three items. The loop adds a DO_JUMP at 3 that flies 1, 2 once more.
	static std::vector<mission_item_s> outAndBackLanes(bool with_loop = false)
	{
		std::vector<mission_item_s> items{
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 20.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 20.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 20.f, kAlt),
		};

		if (with_loop) {
			items.insert(items.begin() + 3, makeDoJump(1, 1, 0));
		}

		return items;
	}

	// An airborne multicopter at position.
	void publishAirborneMulticopter(const mission_route::Position &position,
					const mission_route::Position &home = makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt))
	{
		publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
		publishLandDetected(false);
		publishGlobalPosition(position);
		publishLocalPosition(0.f, 15.f, 0.f);
		publishHomePosition(home);
		primeNavigatorState();
	}

	void publishArmed(bool armed)
	{
		_vehicle_status.timestamp = hrt_absolute_time();
		_vehicle_status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
		_vehicle_status_pub.publish(_vehicle_status);
		*_navigator.get_vstatus() = _vehicle_status;
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
		pauseAt(mission, resume_position);
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

// A join that cannot be planned falls back with a warning: Mission flies to its current item. This happens while
// the route cache loads, while it still holds a previous mission, and while the geofence reloads.
TEST_F(MissionRouteJoinTest, FallsBackWhileTheJoinCannotBePlanned)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);

	// A 3-waypoint DO_JUMP loop plus a landing item, moved east_m to the east.
	const auto loop_mission = [](float east_m) {
		return std::vector<mission_item_s> {
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, east_m, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, east_m, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 300.f + east_m, kAlt),
			makeDoJump(0, 2, 2),
			makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, east_m, kAlt - 10.f),
		};
	};

	const mission_s mission_state = publishMissionItems(loop_mission(0.f), 21, 0, 4);
	const mission_route::Position near_wp2 = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 290.f, kAlt);
	publishAirborneMulticopter(near_wp2, makePositionFromOffset(kBaseLat, kBaseLon, -200.f, 0.f, kAlt));
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

	// A new mission, 50 m to the east, that Mission reads before the route cache does: no join on the old route.
	mission.on_inactivation();
	const mission_s moved_state = publishMissionItems(loop_mission(50.f), 22, 1, 4);
	mission.on_inactive();
	markMissionResultValid();

	// The geofence reloads after the cache holds the new mission: the join cannot check its legs.
	for (const bool fence_reloading : {false, true}) {
		SCOPED_TRACE(fence_reloading ? "geofence reloading" : "cache with the previous mission");

		if (fence_reloading) {
			mission.on_inactivation();
			updateRouteCacheUntilReady(moved_state);
			_navigator.get_geofence().updateFence();
		}

		mission.on_activation();
		EXPECT_EQ(mission.currentSequenceForTest(), 1);
		EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
		EXPECT_FALSE(mission.joinPositionForTest().valid());
		EXPECT_TRUE(events.sent(events::ID("mission_route_rejoin_unavailable")));
	}
}

// Leaving a loop ends its jump edge, so a later join does not go back to the loop.
TEST_F(MissionRouteJoinTest, LeavingTheLoopEndsItsJumpEdge)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	publishAirborneMulticopter(makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt));
	loadMission(mission, outAndBackLanes(true), 951, 1);
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
	pauseAt(mission, makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 12.f, kAlt));
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
	publishAirborneMulticopter(makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt));
	loadMission(mission, items, 954, 0);
	activateWithoutJoin(mission);

	// Items 0 to 3, then the jump back to the camera trigger and on to its position.
	for (int reached = 0; reached < 5; ++reached) {
		reachCurrentItem(mission);
	}

	ASSERT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.jumpAnchorForTest().jump_item_index, 4);
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
	publishAirborneMulticopter(makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt));
	loadMission(mission, items, 53, 3);
	activateWithoutJoin(mission);

	// The last item is reached: the mission is finished.
	reachCurrentItem(mission);
	ASSERT_FALSE(mission.plannedItemValidForTest());

	// Mission is selected again from beside the first leg.
	pauseAt(mission, makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 10.f, kAlt));
	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 3);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_FALSE(mission.joinPositionForTest().valid());
	EXPECT_FALSE(mission.plannedItemValidForTest());
}

// More than NAV_MC_ALT_RAD below the branch-in point, the join climbs in place first, as Mission does before a higher
// item. It climbs only to the branch-in altitude, on this ascending leg 10 m: the rest is climbed along the leg. Less
// far below or from above, it flies straight to the branch-in point, without a descent in place. The joined item is
// the next setpoint of the branch-in waypoint, so a multicopter does not stop there.
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

	struct Row {
		float above_branch_in_m;
		bool climbs;
	};

	// 5 m below, 0.5 m below (inside NAV_MC_ALT_RAD, 0.8 m) and 20 m above the branch-in point.
	for (const Row &row : {Row{-5.f, true}, Row{-0.5f, false}, Row{20.f, false}}) {
		SCOPED_TRACE(row.above_branch_in_m);
		MissionTestPeer mission(&_navigator);
		const mission_route::Position beside_leg = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 10.f,
				kAlt + 10.f + row.above_branch_in_m);
		publishAirborneMulticopter(beside_leg);
		loadMission(mission, items, ++mission_id, 1);
		startReachAndPause(mission, beside_leg);

		mission.on_activation();

		ASSERT_EQ(mission.currentSequenceForTest(), 2);
		const mission_route::Position join = mission.joinPositionForTest();
		ASSERT_TRUE(join.valid());
		EXPECT_NEAR(join.alt, kAlt + 10.f, 1.f);

		if (row.climbs) {
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
		ASSERT_TRUE(triplet.next.valid);
		EXPECT_DOUBLE_EQ(triplet.next.lat, items[2].lat);
		EXPECT_DOUBLE_EQ(triplet.next.lon, items[2].lon);
		EXPECT_FLOAT_EQ(triplet.next.alt, items[2].altitude);
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

// Every airborne start joins: the first one after an upload, also from a later leg, and one after a pause before
// any item was reached. A start on the ground keeps the current item. After a landing and a disarm, the airborne
// start of the next flight restarts the finished mission and joins it again, without a second restart after the join.
TEST_F(MissionRouteJoinTest, EveryAirborneStartJoinsButNotAGroundStart)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	// Taken off in Position mode, 5 m beside the last leg.
	const mission_route::Position beside_last_leg = makePositionFromOffset(kBaseLat, kBaseLon, 5.f, 150.f, kAlt);
	publishAirborneMulticopter(beside_last_leg, makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt));
	loadMission(mission, loopAroundHome(), 54, 0);
	navigator_test::EventRecorder events;

	mission.on_activation();
	EXPECT_EQ(mission.currentSequenceForTest(), 3);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_TRUE(mission.joinPositionForTest().valid());
	EXPECT_FALSE(events.sent(events::ID("mission_route_rejoin_unavailable")));

	// Landed, then Mission starts on the ground.
	mission.on_inactivation();
	publishLandDetected(true);
	primeNavigatorState();
	mission.on_inactive();
	mission.on_activation();
	EXPECT_EQ(mission.currentSequenceForTest(), 3);
	EXPECT_FALSE(mission.joinPositionForTest().valid());

	// Paused before reaching the item, then resumed after a manual takeoff.
	mission.on_inactivation();
	publishLandDetected(false);
	primeNavigatorState();
	mission.on_inactive();
	mission.on_activation();
	EXPECT_EQ(mission.currentSequenceForTest(), 3);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);

	// The last item is reached: the mission is finished.
	reachWorkItem(mission);
	reachCurrentItem(mission);
	ASSERT_FALSE(mission.plannedItemValidForTest());

	// Landed and disarmed while Mission is inactive, then armed and taken off beside the last leg again. Only the
	// restart to item 0 lets the finished mission join, here the last leg again.
	mission.on_inactivation();
	publishLandDetected(true);
	publishArmed(false);
	mission.on_inactive();
	publishArmed(true);
	publishLandDetected(false);
	publishGlobalPosition(beside_last_leg);
	primeNavigatorState();
	mission.on_inactive();
	mission.on_activation();
	EXPECT_EQ(mission.currentSequenceForTest(), 3);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
}

// An item selected while Mission is paused is the current item of the join, like any resume.
TEST_F(MissionRouteJoinTest, ItemSelectedWhilePausedJoins)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	// Between the lanes, closer to the way back.
	const mission_route::Position between_lanes = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 12.f, kAlt);
	publishAirborneMulticopter(between_lanes);
	uint32_t mission_id = 77;

	// Item 1 reached, then paused between the lanes.
	const auto start_and_pause = [&]() {
		const mission_s state = loadMission(mission, outAndBackLanes(), ++mission_id, 1);
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

// A selection that arrives in the cycle Mission starts is read before the join is planned. Otherwise the mission
// publish of the join would replace it: the mission topic keeps only its last message.
TEST_F(MissionRouteJoinTest, SelectionInTheActivationCycleIsJoined)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	const mission_route::Position between_lanes = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 12.f, kAlt);
	publishAirborneMulticopter(between_lanes);
	const mission_s state = loadMission(mission, outAndBackLanes(), 85, 4);

	// Item 4 reached on the way back, then paused between the lanes. A resume would join the way back at item 4.
	startReachAndPause(mission, between_lanes);
	ASSERT_EQ(mission.currentSequenceForTest(), 5);

	// MISSION_SET_CURRENT(2) arrives in the cycle Mission starts.
	mission_s selected = state;
	selected.timestamp = hrt_absolute_time();
	selected.current_seq = 2;
	publishMission(selected);

	mission.on_activation();
	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);

	// Still the selected item one cycle later.
	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
}

// A selection while Mission is active flies to the item directly: it cancels a pending join and ends the jump edge.
// So does a jump counter reset, which keeps the current item.
TEST_F(MissionRouteJoinTest, SelectionOrCounterResetInFlightCancelsJoin)
{
	const std::vector<mission_item_s> items = outAndBackLanes(true);
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

// A resume within the acceptance radius of the landing lands there directly, also when the landing is not the
// paused item. Farther away, the vehicle climbs in place back to the approach altitude and rejoins the route. From a
// branch-in point on the landing leg, it then moves to the landing at that altitude, as after any waypoint.
TEST_F(MissionRouteJoinTest, PausedLandingLandsNearbyOrClimbsBackToTheApproach)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	uORB::Subscription commands{ORB_ID(vehicle_command)};
	vehicle_command_s command{};
	const float descent_altitude = kAlt + 2.f;
	const float approach_altitude = kAlt + 50.f;
	uint32_t mission_id = 431;

	struct Row {
		const char *name;
		bool paused_descending;
		float before_land_m; // NAN: after a GoTo near the first leg
		bool lands_directly;
		int32_t target_index;
	};

	// NAV_ACC_RAD is 10 m and MIS_MC_SEG_DIST 30 m.
	const Row rows[] {
		{"0 m before LAND", true, 0.f, true, 2},
		{"5 m before LAND", true, 5.f, true, 2},
		{"15 m before LAND", true, 15.f, false, 2},
		{"GoTo near the first leg", true, NAN, false, 1},
		{"50 m before LAND", true, 50.f, false, 2},
		{"5 m before LAND, paused before the approach waypoint", false, 5.f, true, 2},
	};

	for (const Row &row : rows) {
		SCOPED_TRACE(row.name);
		auto takeoff = makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, approach_altitude);
		auto land = makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt);
		land.yaw = NAN;

		const std::vector<mission_item_s> items{
			takeoff, makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, approach_altitude), land
		};
		publishAirborneMulticopter({items[1].lat, items[1].lon, approach_altitude},
					   makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt));
		publishNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
		loadMission(mission, items, ++mission_id, 1, 2);
		activateWithoutJoin(mission);
		auto &sp = _navigator.get_position_setpoint_triplet()->current;

		if (row.paused_descending) {
			// Fly the approach waypoint and the move to the landing point, then descend.
			reachCurrentItem(mission);
			reachCurrentItem(mission);
			publishGlobalPosition({land.lat, land.lon, descent_altitude});
			primeNavigatorState();
			ASSERT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_LAND);
		}

		pauseAt(mission, PX4_ISFINITE(row.before_land_m)
			? makePositionFromOffset(land.lat, land.lon, -row.before_land_m, 0.f, descent_altitude)
			: makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 50.f, descent_altitude));

		while (commands.update(&command)) {}

		mission.on_activation();
		// After the GoTo the nearest leg is the earlier one, and Mission continues from there.
		EXPECT_EQ(mission.currentSequenceForTest(), row.target_index);

		if (row.lands_directly) {
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
			EXPECT_EQ(mission.workItemTypeForTest(), row.lands_directly ? MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT
				  : MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_CLIMB);
			EXPECT_EQ(sp.type, row.lands_directly ? position_setpoint_s::SETPOINT_TYPE_LAND
				  : position_setpoint_s::SETPOINT_TYPE_LOITER);
			EXPECT_EQ(mission.currentSequenceForTest(), row.target_index);
		}

		if (!row.lands_directly) {
			// Then it flies to the branch-in point at the approach altitude.
			reachWorkItem(mission);
			EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
			EXPECT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
			EXPECT_FLOAT_EQ(sp.alt, approach_altitude);

			if (row.target_index == 2) {
				reachWorkItem(mission);
				EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND);
				EXPECT_FLOAT_EQ(sp.alt, approach_altitude);
			}
		}

		while (commands.update(&command)) {
			EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
		}
	}
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
		publishAirborneMulticopter(beside_leg);
		const mission_s mission_state = loadMission(mission, items, ++mission_id, 0);
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
	const mission_route::Position on_loop_edge = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt);
	publishAirborneMulticopter(on_loop_edge, makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt - 20.f));
	loadMission(mission, items, 41, 2, 5);

	// Paused before reaching waypoint 2, the vehicle has not taken the jump. A resume on the loop edge joins it with
	// the jump edge from the plan, and flies the repeat that is left from the jump target.
	activateWithoutJoin(mission);
	pauseAt(mission, on_loop_edge);
	mission.on_activation();
	EXPECT_EQ(mission.currentSequenceForTest(), 0);
	EXPECT_EQ(mission.jumpAnchorForTest().jump_item_index, 3);
	mission.on_inactivation();
	ASSERT_TRUE(mission.set_current_mission_index(2));

	// Reaching waypoint 2 follows the last repeat back to item 0; paused on that loop edge.
	startReachAndPause(mission, on_loop_edge);
	ASSERT_EQ(mission.currentSequenceForTest(), 0);
	ASSERT_EQ(mission.jumpAnchorForTest().jump_item_index, 3);

	mission.on_activation();

	ASSERT_EQ(mission.currentSequenceForTest(), 2);
	ASSERT_TRUE(mission.joinPositionForTest().valid());
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);

	reachWorkItem(mission);

	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, -1);
	EXPECT_DOUBLE_EQ(_navigator.get_position_setpoint_triplet()->current.lat, items[2].lat);

	reachCurrentItem(mission);

	EXPECT_EQ(mission.currentSequenceForTest(), 4);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 2);
	EXPECT_DOUBLE_EQ(_navigator.get_position_setpoint_triplet()->current.lat, items[4].lat);
	mission_item_s exhausted_jump{};
	ASSERT_TRUE(_dataman_client.readSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 3,
					     reinterpret_cast<uint8_t *>(&exhausted_jump), sizeof(exhausted_jump)));
	EXPECT_EQ(exhausted_jump.do_jump_current_count, 3);
}

// A vehicle without the join, as commander publishes it: a VTOL reports ROTARY_WING in hover. The gate reads only
// the vehicle type and is_vtol. The multicopter row has MIS_ROUTE_JOIN off; the other rows also fly with it on.
struct UntouchedVehicle {
	const char *name;
	bool is_vtol;
	uint8_t vehicle_type;
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

		// Off is the default.
		if (join) {
			setIntParam("MIS_ROUTE_JOIN", 1);

		} else {
			param_reset(param_find("MIS_ROUTE_JOIN"));
		}

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
		publishVehicleStatus(vehicle.is_vtol, vehicle.vehicle_type);
		publishLandDetected(false);
		publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt));
		publishLocalPosition();
		publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt));
		primeNavigatorState();
		loadMission(mission, items, mission_id, 1);
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
		pauseAt(mission, makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 10.f, kAlt));

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

	// Selections, uploads and a restart, run through NavigatorMode::run() so that isActive() is real.
	// The expected indices are main's for the same steps.
	void flyMainPath(uint32_t mission_id)
	{
		const UntouchedVehicle &vehicle = GetParam();
		setIntParam("MIS_ROUTE_JOIN", vehicle.also_with_join_param ? 1 : 0);
		MissionTestPeer mission(&_navigator);
		const auto lane = [](float east_m) {
			std::vector<mission_item_s> items;
			items.reserve(4);

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
		publishVehicleStatus(vehicle.is_vtol, vehicle.vehicle_type);
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
				 UntouchedVehicle{"FixedWing", false, vehicle_status_s::VEHICLE_TYPE_FIXED_WING, true},
				 UntouchedVehicle{"VtolHover", true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true},
				 UntouchedVehicle{"Rover", false, vehicle_status_s::VEHICLE_TYPE_ROVER, true},
				 UntouchedVehicle{"MulticopterJoinOff", false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false}),
			 untouchedVehicleName);
