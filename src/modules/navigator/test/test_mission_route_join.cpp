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
 * Mission route join and VTOL transition integration tests.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include "mission.h"
#include "mission_item_utils.h"
#include "navigator.h"
#include "rtl_direct_mission_land.h"
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
#include <uORB/topics/position_controller_landing_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

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

	using Mission::trySetRouteJoinOnActivation;
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
	using MissionBase::VtolTransitionAction;
	using MissionBase::RouteJoinContext;
	using MissionBase::WorkItemType;

	void beginItemForTest(WorkItemType work)
	{
		resetVtolTransition();
		_work_item_type = work;
		_mission_type = MissionType::MISSION_TYPE_MISSION;
		_is_current_planned_mission_item_valid = true;
		_mission_check_state = MissionCheckState::Complete;
		_mission_init_climb_altitude_amsl = NAN;
		_navigator->reset_triplets();
		set_mission_items();
	}

	void publishItemsForTest() { set_mission_items(); }
	// A pending check, as after a geofence upload; on_active() completes it with the current verdict.
	void requestFeasibilityCheckForTest() { _mission_check_state = MissionCheckState::Unchecked; }
	bool transitionActiveForTest() const { return vtolTransitionActive(); }
	const mission_item_s &currentMissionItemForTest() const { return _mission_item; }

	WorkItemType workItemTypeForTest() const { return _work_item_type; }
	int32_t currentSequenceForTest() const { return _mission.current_seq; }
	const RouteJoinContext &joinContextForTest() const { return _route_join_context; }
	VtolTransitionAction joinTransitionActionForTest() const { return _route_join_context.transition_action; }

private:
	int32_t _write_failure_index{-1};
};

/**
 * @brief Return to the mission landing (RTL_TYPE 1) that exposes its mission cursor.
 */
class RtlDirectMissionLandTestPeer : public RtlDirectMissionLand
{
public:
	RtlDirectMissionLandTestPeer(Navigator *navigator, const mission_s &mission) : RtlDirectMissionLand(navigator, mission) {}

	int32_t currentSequenceForTest() const { return _mission.current_seq; }
	uint16_t currentNavCmdForTest() const { return _mission_item.nav_cmd; }
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

	void writeMissionItems(const std::vector<mission_item_s> &items)
	{
		for (size_t i = 0; i < items.size(); ++i) {
			mission_item_s item = items[i];
			ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, i,
							      reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		}
	}

	// Stores the items with an empty rally set and publishes their mission state.
	mission_s publishMissionItems(const std::vector<mission_item_s> &items, uint32_t mission_id, int32_t current_seq,
				      int32_t land_index = -1)
	{
		writeMissionItems(items);
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
		state.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
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

	void prepareExecution(MissionTestPeer &mission, const std::vector<mission_item_s> &items, uint32_t mission_id,
			      MissionTestPeer::WorkItemType work = MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT)
	{
		writeMissionItems(items);
		mission_s state{};
		state.timestamp = hrt_absolute_time();
		state.mission_id = mission_id;
		state.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
		state.count = items.size();
		publishMission(state);
		mission.on_inactive();
		markMissionResultValid();
		_navigator.get_mission_result()->seq_reached = -1;
		mission.beginItemForTest(work);
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
		mission.on_activation();

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

	// Survey legs 0-6 at 100 m spacing, then a far landing approach (7) and its LAND (8).
	static std::vector<mission_item_s> surveyThenLanding()
	{
		std::vector<mission_item_s> items;

		for (int index = 0; index <= 6; ++index) {
			items.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, index * 100.f, 0.f, kAlt + 50.f));
		}

		items.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, 2000.f, 1000.f, kAlt + 50.f));
		items.push_back(makeLandItemFromOffset(kBaseLat, kBaseLon, 2100.f, 1000.f, kAlt));
		return items;
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

	// Fixed-wing mission with DO_LAND_START (2), the approach (3) and LAND (4); the vehicle flies the first leg.
	void publishFixedWingLandingMission(uint32_t mission_id)
	{
		mission_item_s land_start{};
		land_start.nav_cmd = NAV_CMD_DO_LAND_START;
		land_start.autocontinue = true;
		const std::vector<mission_item_s> items{
			makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 100.f),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 100.f),
			land_start,
			makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 1500.f, kAlt + 60.f),
			makeLandItemFromOffset(kBaseLat, kBaseLon, 0.f, 1500.f, kAlt),
		};
		mission_s state = publishMissionItems(items, mission_id, 1, 4);
		state.land_start_index = 2;
		publishMission(state);
		publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
		publishLandDetected(false);
		publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 100.f));
		publishLocalPosition(0.f, 15.f, 0.f);
		publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt));
		primeNavigatorState();
		updateRouteCacheUntilReady(state);
	}

	// The position controller reports an operator abort of the landing, or clears it.
	void publishLandingAbort(bool aborted)
	{
		position_controller_landing_status_s landing_status{};
		landing_status.timestamp = hrt_absolute_time();
		landing_status.abort_status = aborted ? position_controller_landing_status_s::ABORTED_BY_OPERATOR : 0;
		_landing_status_pub.publish(landing_status);
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

	// Starts the mission, reaches its current item and pauses on the way to the next one with the vehicle at
	// resume_position, so the next activation resumes a mission flown along its route.
	void startReachAndPause(MissionTestPeer &mission, const mission_route::Position &resume_position)
	{
		mission.on_activation();
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
	uORB::Publication<position_controller_landing_status_s> _landing_status_pub{ORB_ID(position_controller_landing_status)};

	vehicle_status_s _vehicle_status{};
	vehicle_land_detected_s _land_detected{};
	vehicle_global_position_s _global_position{};
	vehicle_local_position_s _local_position{};
	home_position_s _home_position{};
};

TEST_F(MissionRouteJoinTest, NominalTransitionsShareAlignmentAndPreserveCommandParameters)
{
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	auto target = makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);
	target.yaw = M_PI_4_F;
	const float yaw = get_bearing_to_next_waypoint(position.lat, position.lon, target.lat, target.lon);
	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};
	uint32_t mission_id = 100;
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = true;

	for (bool back_transitioning : {false, true}) {
		SCOPED_TRACE(back_transitioning);
		const std::vector<mission_item_s> items{
			front_transition, target,
		};
		publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false, back_transitioning);
		publishGlobalPosition(position);
		publishLocalPosition(matrix::wrap_pi(yaw + M_PI_2_F));
		publishLandDetected(false);
		publishHomePosition(position);
		primeNavigatorState();
		prepareExecution(mission, items, ++mission_id);

		while (command_sub.update(&command)) {}

		if (back_transitioning) {
			EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_WAIT_FOR_BACK_TRANSITION);
			mission.on_active();
			EXPECT_FALSE(command_sub.updated());
			publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
			primeNavigatorState();
			mission.on_active();
		}

		const auto &triplet = *_navigator.get_position_setpoint_triplet();
		ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
		// Nominal Mission still tracks the next waypoint while checking a separate alignment item.
		EXPECT_DOUBLE_EQ(triplet.current.lat, target.lat);
		EXPECT_DOUBLE_EQ(triplet.current.lon, target.lon);
		EXPECT_FLOAT_EQ(triplet.current.yaw, target.yaw);
		EXPECT_NEAR(mission.currentMissionItemForTest().yaw, yaw, 1e-4f);
		mission.publishItemsForTest();
		mission.on_active();
		EXPECT_FALSE(command_sub.updated());

		if (!back_transitioning) {
			// If BT starts during alignment, wait and align again before commanding FW.
			publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false, true);
			primeNavigatorState();
			mission.on_active();
			EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_WAIT_FOR_BACK_TRANSITION);
			publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
			primeNavigatorState();
			mission.on_active();
			EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
			EXPECT_FALSE(command_sub.updated());
		}

		const auto &alignment = mission.currentMissionItemForTest();
		publishGlobalPosition({alignment.lat, alignment.lon, alignment.altitude});
		publishLocalPosition(yaw);
		primeNavigatorState();
		mission.on_active();
		ASSERT_TRUE(command_sub.update(&command));
		EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
		EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
		EXPECT_DOUBLE_EQ(triplet.current.lat, target.lat);
		EXPECT_FLOAT_EQ(triplet.current.yaw, target.yaw);
		EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);

		// Republish during FT without repeating the command or advancing the uploaded item.
		publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true);
		primeNavigatorState();
		mission.on_active();
		mission.publishItemsForTest();
		EXPECT_FALSE(command_sub.updated());
		EXPECT_EQ(mission.currentSequenceForTest(), 0);
		publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
		primeNavigatorState();
		// Refreshing setpoints cannot consume a completed uploaded command.
		mission.publishItemsForTest();
		EXPECT_TRUE(mission.transitionActiveForTest());
		EXPECT_EQ(mission.currentSequenceForTest(), 0);
		mission.on_active();
		EXPECT_EQ(mission.currentSequenceForTest(), 1);
		EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
	}

	// A nominal transition already in the requested mode is still commanded, as before the sequencer.
	prepareExecution(mission, {front_transition, target}, ++mission_id);
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);

	// A nominal immediate BT keeps its uploaded parameter and waits for stable MC.
	auto back_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	back_transition.params[1] = 1.f;
	back_transition.autocontinue = true;
	prepareExecution(mission, {back_transition, target}, ++mission_id);
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	EXPECT_FLOAT_EQ(command.param2, 1.f);
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false, true);
	primeNavigatorState();
	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 0);
	EXPECT_FALSE(command_sub.updated());
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	primeNavigatorState();
	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 1);

	// Report a paused command through the same active loop without issuing it again.
	front_transition.autocontinue = false;
	publishGlobalPosition(position);
	publishLocalPosition(yaw);
	primeNavigatorState();
	prepareExecution(mission, {front_transition, target}, ++mission_id);
	mission.on_active();
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	primeNavigatorState();

	for (int i = 0; i < 3; ++i) {
		mission.on_active();
		mission.publishItemsForTest();
		EXPECT_EQ(mission.currentSequenceForTest(), 0);
		EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
		EXPECT_TRUE(mission.transitionActiveForTest());
		EXPECT_FALSE(command_sub.updated());
	}

	// An explicit cursor change cancels the old alignment.
	front_transition.autocontinue = true;
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishLocalPosition(matrix::wrap_pi(yaw + M_PI_2_F));
	primeNavigatorState();
	prepareExecution(mission, {front_transition, target}, ++mission_id);
	ASSERT_TRUE(mission.transitionActiveForTest());
	ASSERT_TRUE(mission.set_current_mission_index(1));
	EXPECT_FALSE(mission.transitionActiveForTest());
	EXPECT_FALSE(command_sub.updated());
}

TEST_F(MissionRouteJoinTest, VtolTakeoffKeepsItsWaypointOnlyWhenMovementRemains)
{
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	uint32_t mission_id = 110;

	for (bool initially_distant : {false, true}) {
		for (bool movement_remains : {false, true}) {
			SCOPED_TRACE(::testing::Message() << initially_distant << ", " << movement_remains);
			auto takeoff = makePositionItemFromOffset(kBaseLat, kBaseLon, initially_distant ? 100.f : 0.f, 0.f, kAlt);
			takeoff.nav_cmd = NAV_CMD_VTOL_TAKEOFF;
			const auto next = makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt);
			publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
			publishGlobalPosition(position);
			publishLocalPosition(M_PI_2_F);
			publishLandDetected(false);
			publishHomePosition(position);
			primeNavigatorState();
			prepareExecution(mission, {takeoff, next}, ++mission_id, MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_CLIMB);
			ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
			ASSERT_EQ(mission.currentMissionItemForTest().nav_cmd, NAV_CMD_VTOL_TAKEOFF);
			const auto &triplet = *_navigator.get_position_setpoint_triplet();
			EXPECT_FLOAT_EQ(triplet.current.alt, kAlt);

			// Drift across the acceptance boundary while aligning; choose the continuation afterwards.
			const float final_alt = kAlt + 2.f * _navigator.get_altitude_acceptance_radius();
			publishGlobalPosition(makePositionFromOffset(takeoff.lat, takeoff.lon, movement_remains ? 100.f : 0.f, 0.f, final_alt));
			publishLocalPosition(mission.currentMissionItemForTest().yaw);
			primeNavigatorState();
			mission.on_active();
			// Takeoff accepts being above its altitude without returning to the alignment position.
			ASSERT_EQ(mission.currentMissionItemForTest().nav_cmd, NAV_CMD_DO_VTOL_TRANSITION);
			ASSERT_TRUE(mission.transitionActiveForTest());
			EXPECT_EQ(mission.workItemTypeForTest(), movement_remains
				  ? MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF
				  : MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
			// The aligned takeoff is reported reached with its command, as before the sequencer.
			EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
			publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true);
			primeNavigatorState();
			mission.on_active();
			EXPECT_EQ(mission.currentSequenceForTest(), 0);
			publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
			primeNavigatorState();
			mission.on_active();
			EXPECT_FALSE(mission.transitionActiveForTest());
			EXPECT_EQ(mission.currentSequenceForTest(), movement_remains ? 0 : 1);
			EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
			EXPECT_DOUBLE_EQ(triplet.current.lat, movement_remains ? takeoff.lat : next.lat);
			EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
		}
	}
}

TEST_F(MissionRouteJoinTest, ManualFrontTransitionDuringAlignmentStillSendsCommand)
{
	// As before the sequencer, the item is reported reached and commanded, then completes in fixed-wing.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto target = makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt);
	auto transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	transition.autocontinue = true;
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(position);
	publishLocalPosition(M_PI_2_F);
	publishLandDetected(false);
	publishHomePosition(position);
	primeNavigatorState();
	prepareExecution(mission, {transition, target}, 130);
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);

	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	primeNavigatorState();
	mission.on_active();
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission.currentSequenceForTest(), 0);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);

	mission.on_active();
	EXPECT_FALSE(command_sub.updated());
	EXPECT_FALSE(mission.transitionActiveForTest());
	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_EQ(_navigator.get_mission_result()->seq_current, 1);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
}

TEST_F(MissionRouteJoinTest, TransitionAlreadyInModeIsReportedAsCurrentItem)
{
	// A transition already in its mode is commanded and becomes the current item; it holds without autocontinue.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = false;
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishGlobalPosition(position);
	publishLocalPosition();
	publishLandDetected(false);
	publishHomePosition(position);
	primeNavigatorState();
	prepareExecution(mission, {makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt), front_transition,
				   makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt)
				  }, 140);
	ASSERT_EQ(mission.currentSequenceForTest(), 0);

	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_EQ(_navigator.get_mission_result()->seq_current, 1);
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);

	mission.on_active();
	EXPECT_FALSE(command_sub.updated());
	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_EQ(_navigator.get_mission_result()->seq_current, 1);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 1);
}

TEST_F(MissionRouteJoinTest, AlignedTransitionIsReportedReachedWithItsCommand)
{
	// As before the sequencer, the reached item changes when the alignment ends and the command is sent.
	// The report in fixed-wing repeats it, so MAVLink sends MISSION_ITEM_REACHED once.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto target = makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 200.f, kAlt);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = true;
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(position);
	publishLocalPosition();
	publishLandDetected(false);
	publishHomePosition(position);
	primeNavigatorState();
	prepareExecution(mission, {makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt), front_transition, target},
			 143);

	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	mission.on_active();
	ASSERT_EQ(mission.currentSequenceForTest(), 1);
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);

	publishLocalPosition(get_bearing_to_next_waypoint(position.lat, position.lon, target.lat, target.lon));
	primeNavigatorState();
	mission.on_active();
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 1);

	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true);
	primeNavigatorState();
	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 1);

	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	primeNavigatorState();
	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 1);
}

TEST_F(MissionRouteJoinTest, BackTransitionInHoverAlignsBeforeItsCommand)
{
	// As before the sequencer, a back transition reached in hover turns towards the next waypoint, then is commanded.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto target = makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 200.f, kAlt);
	const float yaw = get_bearing_to_next_waypoint(position.lat, position.lon, target.lat, target.lon);
	auto back_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	back_transition.autocontinue = true;
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(position);
	publishLocalPosition();
	publishLandDetected(false);
	publishHomePosition(position);
	primeNavigatorState();
	prepareExecution(mission, {makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt), back_transition, target},
			 144);

	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	mission.on_active();
	ASSERT_EQ(mission.currentSequenceForTest(), 1);
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
	EXPECT_NEAR(mission.currentMissionItemForTest().yaw, yaw, 1e-4f);
	mission.on_active();
	EXPECT_FALSE(command_sub.updated());
	EXPECT_EQ(mission.currentSequenceForTest(), 1);

	publishLocalPosition(yaw);
	primeNavigatorState();
	mission.on_active();
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 1);

	mission.on_active();
	EXPECT_FALSE(command_sub.updated());
	EXPECT_EQ(mission.currentSequenceForTest(), 2);
}

TEST_F(MissionRouteJoinTest, TransitionWaitingForBackTransitionIsReportedAsCurrentItem)
{
	// A front transition reached during a back transition becomes the current item while it waits.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false, true);
	publishGlobalPosition(position);
	publishLocalPosition();
	publishLandDetected(false);
	publishHomePosition(position);
	primeNavigatorState();
	prepareExecution(mission, {makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
				   makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
				   makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt)
				  }, 141);
	ASSERT_EQ(mission.currentSequenceForTest(), 0);

	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_WAIT_FOR_BACK_TRANSITION);
	EXPECT_EQ(_navigator.get_mission_result()->seq_current, 1);
}

TEST_F(MissionRouteJoinTest, FrontTransitionAfterBackTransitionAlignsWhereTheVehicleIs)
{
	// The back transition ends past the waypoint reached during it; alignment must not wait for a return to it.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto target = makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt);
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false, true);
	publishGlobalPosition(position);
	publishLocalPosition();
	publishLandDetected(false);
	publishHomePosition(position);
	primeNavigatorState();
	prepareExecution(mission, {makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
				   makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW), target
				  }, 142);
	mission.on_active();
	ASSERT_EQ(mission.currentSequenceForTest(), 1);
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_WAIT_FOR_BACK_TRANSITION);

	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	const auto stop = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 60.f, kAlt);
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(stop);
	publishLocalPosition(get_bearing_to_next_waypoint(stop.lat, stop.lon, target.lat, target.lon));
	primeNavigatorState();
	mission.on_active();
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
	EXPECT_FALSE(command_sub.updated());

	mission.on_active();
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
}

TEST_F(MissionRouteJoinTest, FeasibilityRefreshRestartsAligningFrontTransition)
{
	// A feasibility refresh restarts the item: the alignment starts again where the vehicle is, and the front
	// transition is still commanded. During the transition the restart sends the command once more.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto target = makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 100.f, kAlt);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = true;
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(position);
	publishLocalPosition();
	publishLandDetected(false);
	publishHomePosition(position);
	primeNavigatorState();
	prepareExecution(mission, {makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt), front_transition, target},
			 145);

	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	mission.on_active();
	ASSERT_EQ(mission.currentSequenceForTest(), 1);
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);

	// The vehicle drifts while aligning, then a pending feasibility check completes.
	const auto drifted = makePositionFromOffset(kBaseLat, kBaseLon, 40.f, 0.f, kAlt);
	const float yaw = get_bearing_to_next_waypoint(drifted.lat, drifted.lon, target.lat, target.lon);
	publishGlobalPosition(drifted);
	primeNavigatorState();
	mission.requestFeasibilityCheckForTest();
	mission.on_active();
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
	EXPECT_DOUBLE_EQ(mission.currentMissionItemForTest().lat, drifted.lat);
	EXPECT_NEAR(mission.currentMissionItemForTest().yaw, yaw, 1e-4f);
	EXPECT_FALSE(command_sub.updated());

	publishLocalPosition(yaw);
	primeNavigatorState();
	mission.on_active();
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission.currentSequenceForTest(), 1);

	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true);
	primeNavigatorState();
	mission.on_active();
	EXPECT_FALSE(command_sub.updated());
	mission.requestFeasibilityCheckForTest();
	mission.on_active();
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_FALSE(command_sub.updated());
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, -1);
	mission.on_active();
	EXPECT_FALSE(command_sub.updated());
	EXPECT_EQ(mission.currentSequenceForTest(), 1);

	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	primeNavigatorState();
	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 2);
}

// A rejoin skipped while the cache loads is reported; once the cache is ready the vehicle joins the
// leg it is next to.
TEST_F(MissionRouteJoinTest, MissionSmartRejoinWaitsForCacheThenJoinsNearestLeg)
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

	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishLandDetected(false);
	publishGlobalPosition(near_wp2);
	publishLocalPosition(0.f, 12.f, 0.f);
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -200.f, 0.f, kAlt));
	primeNavigatorState();

	mission.on_inactive();
	navigator_test::EventRecorder events;

	// The first start keeps the current item: an intentional skip, not a planning failure, even while the
	// cache is loading. The vehicle then reaches WP0 and is paused near WP2.
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

	// Rejoin targets WP2 with a valid JOIN_ROUTE work item and no VTOL transition.
	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_TRUE(mission.joinContextForTest().valid());
	EXPECT_EQ(mission.joinTransitionActionForTest(), MissionTestPeer::VtolTransitionAction::kNone);
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

		ASSERT_TRUE(mission.trySetRouteJoinOnActivation(false));
		EXPECT_EQ(mission.currentSequenceForTest(), fail_write ? 4 : 0);
		EXPECT_EQ(mission.jumpAnchorForTest().jump_item_index, fail_write ? -1 : 3);
		EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
		EXPECT_TRUE(mission.joinContextForTest().valid());
	}

	// A successful jump can land on another jump whose write fails. Keep the anchor
	// of the first jump: skipping the second still reaches the first jump's position target.
	prepareExecution(mission, {
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makeDoJump(0, 2, 0),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makeDoJump(1, 2, 0),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 100.f, kAlt),
	}, 53);
	ASSERT_TRUE(mission.set_current_mission_index(3));
	mission.setWriteFailureIndexForTest(1);

	ASSERT_TRUE(mission.advanceMissionForTest());
	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.jumpAnchorForTest().jump_item_index, 4);
	mission_item_s stored_jump{};
	ASSERT_TRUE(_dataman_client.readSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 4,
					     reinterpret_cast<uint8_t *>(&stored_jump), sizeof(stored_jump)));
	EXPECT_EQ(stored_jump.do_jump_current_count, 1);
	ASSERT_TRUE(_dataman_client.readSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 1,
					     reinterpret_cast<uint8_t *>(&stored_jump), sizeof(stored_jump)));
	EXPECT_EQ(stored_jump.do_jump_current_count, 0);
}

TEST_F(MissionRouteJoinTest, ResumingBeforeFirstFrontTransitionRequestsBackTransition)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	std::vector<mission_item_s> items;

	for (int index = 0; index <= 15; ++index) {
		items.push_back(index == 10 ? makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW)
				: makePositionItemFromOffset(kBaseLat, kBaseLon, index * 100.f, 0.f, kAlt));
	}

	const mission_route::Position near_wp2 = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 10.f, kAlt);
	const mission_s state = publishMissionItems(items, 53, 1);
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishLandDetected(false);
	publishGlobalPosition(near_wp2);
	publishLocalPosition(0.f, 15.f, 0.f);
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt));
	primeNavigatorState();
	updateRouteCacheUntilReady(state);
	mission.on_inactive();
	markMissionResultValid();
	// Paused after waypoint 1, before the FT; the vehicle is back near waypoint 2 in FW.
	startReachAndPause(mission, near_wp2);

	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_EQ(mission.joinTransitionActionForTest(), MissionTestPeer::VtolTransitionAction::kBackTransition);
}

// Selecting an item before the FT while in FW flies there in FW, as without the join: no back transition.
TEST_F(MissionRouteJoinTest, ItemSelectedBeforeFirstFrontTransitionIsFlownInFixedWing)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	std::vector<mission_item_s> items;

	for (int index = 0; index <= 15; ++index) {
		items.push_back(index == 10 ? makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW)
				: makePositionItemFromOffset(kBaseLat, kBaseLon, index * 100.f, 0.f, kAlt));
	}

	const mission_route::Position near_wp2 = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 10.f, kAlt);
	const mission_s state = publishMissionItems(items, 56, 14);
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishLandDetected(false);
	publishGlobalPosition(near_wp2);
	publishLocalPosition(0.f, 15.f, 0.f);
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt));
	primeNavigatorState();
	updateRouteCacheUntilReady(state);
	mission.on_inactive();
	markMissionResultValid();
	// After flying past the FT, the vehicle has returned near waypoint 2 while still in FW.
	startReachAndPause(mission, near_wp2);
	ASSERT_TRUE(mission.set_current_mission_index(2));
	uORB::Subscription commands{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (commands.update(&command)) {}

	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_FALSE(mission.joinContextForTest().valid());

	while (commands.update(&command)) {
		EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	}
}

TEST_F(MissionRouteJoinTest, DisarmBetweenActivationsKeepsCurrentItem)
{
	// A new flight after a landing flies to the current item: no join onto the last leg, and no restart.
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
	};
	const mission_route::Position beside_last_leg = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 10.f, kAlt);
	const mission_s mission_state = publishMissionItems(items, 51, 1);
	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishLandDetected(false);
	publishGlobalPosition(beside_last_leg);
	publishLocalPosition();
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt));
	primeNavigatorState();
	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();
	markMissionResultValid();
	startReachAndPause(mission, beside_last_leg);
	mission.setDisarmedSinceActivationForTest();

	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_FALSE(mission.joinContextForTest().valid());
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
	EXPECT_FALSE(mission.joinContextForTest().valid());
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
	mission.on_activation();

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
	EXPECT_FALSE(mission.joinContextForTest().valid());
	EXPECT_FALSE(mission.plannedItemValidForTest());
}

TEST_F(MissionRouteJoinTest, JoinOnAscendingSegmentFollowsRouteWithoutVerticalClimb)
{
	// The branch-in carries the interpolated route altitude; the target altitude is reached along the segment.
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt + 40.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt + 40.f),
	};
	const mission_route::Position beside_leg = makePositionFromOffset(kBaseLat, kBaseLon, 150.f, 10.f, kAlt);
	const mission_s mission_state = publishMissionItems(items, 61, 1);
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
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	ASSERT_EQ(mission.currentSequenceForTest(), 2);
	const mission_route::Position join = mission.joinContextForTest().projection;
	EXPECT_GT(join.alt, kAlt + 1.f);
	EXPECT_LT(join.alt, kAlt + 39.f);
	const auto &join_setpoint = _navigator.get_position_setpoint_triplet()->current;
	ASSERT_TRUE(join_setpoint.valid);
	EXPECT_DOUBLE_EQ(join_setpoint.lat, join.lat);
	EXPECT_DOUBLE_EQ(join_setpoint.lon, join.lon);
	EXPECT_FLOAT_EQ(join_setpoint.alt, join.alt);

	publishGlobalPosition(join);
	primeNavigatorState();
	mission.on_active();

	const auto &triplet = *_navigator.get_position_setpoint_triplet();
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_FALSE(mission.joinContextForTest().valid());
	EXPECT_EQ(mission.currentMissionItemForTest().nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_DOUBLE_EQ(triplet.current.lat, items[2].lat);
	EXPECT_DOUBLE_EQ(triplet.current.lon, items[2].lon);
	EXPECT_FLOAT_EQ(triplet.current.alt, items[2].altitude);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
}

TEST_F(MissionRouteJoinTest, RestartedMissionStartsAtFirstItemWithoutJoin)
{
	// A mission finished before the disarm flies again from item 0, even when airborne near its end.
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

	EXPECT_EQ(mission.currentSequenceForTest(), 0);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_FALSE(mission.joinContextForTest().valid());
}

// The first start of a mission flies to its current item, even from beside a later leg.
TEST_F(MissionRouteJoinTest, NeverActivatedMissionStartsAtCurrentItemWithoutJoin)
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

	EXPECT_EQ(mission.currentSequenceForTest(), 0);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_FALSE(mission.joinContextForTest().valid());
	// Keeping the current item is intended, not a planning failure.
	EXPECT_FALSE(events.sent(events::ID("mission_route_rejoin_unavailable")));

	// Paused and resumed before item 0 is reached, the mission still flies to it.
	mission.on_inactivation();
	mission.on_inactive();
	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 0);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_FALSE(mission.joinContextForTest().valid());
}

// After a landing and disarm, a new flight from beside the last leg flies to the current item instead of
// skipping the rest of the mission.
TEST_F(MissionRouteJoinTest, NewFlightAfterLandingKeepsCurrentItem)
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

	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_FALSE(mission.joinContextForTest().valid());
}

// An item selected while Mission is paused is flown to directly; the join must not override it.
TEST_F(MissionRouteJoinTest, ItemSelectedWhilePausedIsKeptOnResume)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	const std::vector<mission_item_s> items = surveyThenLanding();
	const mission_route::Position beside_leg_3_4 = makePositionFromOffset(kBaseLat, kBaseLon, 350.f, 10.f, kAlt + 50.f);
	publishAirborneMulticopter(beside_leg_3_4);
	uint32_t mission_id = 77;

	// Item 2 reached, then paused beside survey leg 3-4.
	const auto start_and_pause_on_survey = [&]() {
		const mission_s state = publishMissionItems(items, ++mission_id, 2, 8);
		updateRouteCacheUntilReady(state);
		mission.on_inactive();
		markMissionResultValid();
		startReachAndPause(mission, beside_leg_3_4);
		return state;
	};

	// Without a selection, the resume joins the survey leg beside the vehicle.
	start_and_pause_on_survey();
	mission.on_activation();
	ASSERT_EQ(mission.currentSequenceForTest(), 4);
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	mission.on_inactivation();

	// DO_SET_MISSION_CURRENT and MISSION_START (also sent for DO_LAND_START) select through
	// set_current_mission_index(), MISSION_SET_CURRENT through the mission topic. The current item counts too.
	const struct {
		int32_t index;
		bool from_mission_topic;
	} selections[] {{7, false}, {3, false}, {7, true}, {3, true}};

	for (const auto &selection : selections) {
		SCOPED_TRACE(::testing::Message() << "index=" << selection.index << ", topic=" << selection.from_mission_topic);
		const mission_s state = start_and_pause_on_survey();

		if (selection.from_mission_topic) {
			mission_s selected = state;
			selected.timestamp = hrt_absolute_time();
			selected.current_seq = selection.index;
			publishMission(selected);
			mission.on_inactive();

		} else {
			ASSERT_TRUE(mission.set_current_mission_index(selection.index));
		}

		mission.on_activation();

		EXPECT_EQ(mission.currentSequenceForTest(), selection.index);
		EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
		EXPECT_FALSE(mission.joinContextForTest().valid());
		mission.on_inactivation();
	}
}

// A selected item is flown to directly across further pauses until it is reached; later resumes join again.
TEST_F(MissionRouteJoinTest, SelectedItemIsKeptAcrossPausesUntilReached)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	const mission_route::Position beside_leg_3_4 = makePositionFromOffset(kBaseLat, kBaseLon, 350.f, 10.f, kAlt + 50.f);
	publishAirborneMulticopter(beside_leg_3_4);
	const mission_s state = publishMissionItems(surveyThenLanding(), 90, 2, 8);
	updateRouteCacheUntilReady(state);
	mission.on_inactive();
	markMissionResultValid();
	startReachAndPause(mission, beside_leg_3_4);

	// The landing approach is selected while paused.
	ASSERT_TRUE(mission.set_current_mission_index(7));
	mission.on_activation();
	ASSERT_EQ(mission.currentSequenceForTest(), 7);
	ASSERT_FALSE(mission.joinContextForTest().valid());

	// Paused again before reaching it, still beside the survey.
	mission.on_inactivation();
	mission.on_inactive();
	mission.on_activation();
	EXPECT_EQ(mission.currentSequenceForTest(), 7);
	EXPECT_FALSE(mission.joinContextForTest().valid());

	// Once it is reached, a resume beside the approach leg rejoins the route.
	reachCurrentItem(mission);
	ASSERT_EQ(mission.currentSequenceForTest(), 8);
	mission.on_inactivation();
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 2050.f, 1010.f, kAlt + 25.f));
	primeNavigatorState();
	mission.on_inactive();
	mission.on_activation();
	EXPECT_EQ(mission.currentSequenceForTest(), 8);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
}

// A selection handled while Mission is still active, e.g. DO_LAND_START in flight, is kept after a pause.
TEST_F(MissionRouteJoinTest, ItemSelectedInFlightIsKeptAfterPause)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	const mission_route::Position beside_leg_3_4 = makePositionFromOffset(kBaseLat, kBaseLon, 350.f, 10.f, kAlt + 50.f);
	publishAirborneMulticopter(beside_leg_3_4);
	const mission_s state = publishMissionItems(surveyThenLanding(), 91, 2, 8);
	updateRouteCacheUntilReady(state);
	// Run Mission as Navigator does, so that it is active while it handles the selection.
	mission.run(false);
	markMissionResultValid();
	mission.run(true);
	reachCurrentItem(mission);
	publishGlobalPosition(beside_leg_3_4);
	primeNavigatorState();
	mission.run(false);
	mission.run(false);
	mission.run(true);
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	mission.run(true);

	// Navigator handles commands before the modes run, so Mission is still active in the cycle that pauses it.
	// The selection cancels the armed join at once.
	ASSERT_TRUE(mission.isActive());
	ASSERT_TRUE(mission.set_current_mission_index(7));
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_FALSE(mission.joinContextForTest().valid());
	mission.run(false);
	mission.run(false);
	mission.run(true);

	EXPECT_EQ(mission.currentSequenceForTest(), 7);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_FALSE(mission.joinContextForTest().valid());
}

// A Return along the mission moves the index, and continuing the mission still rejoins the route.
// An operator selection during the Return is kept.
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
	uint32_t mission_id = 80;

	for (bool operator_selects : {false, true}) {
		SCOPED_TRACE(operator_selects ? "operator selection during Return" : "Return only");
		const mission_s state = publishMissionItems(items, ++mission_id, 4);
		updateRouteCacheUntilReady(state);
		publishNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
		mission.on_inactive();
		markMissionResultValid();
		mission.on_activation();
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

		if (operator_selects) {
			mission_s selected = state;
			selected.timestamp = hrt_absolute_time();
			selected.current_seq = 6;
			publishMission(selected);
			mission.on_inactive();
		}

		// The operator continues the mission.
		rtl.on_inactivation();
		publishNavState(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
		mission.on_activation();

		EXPECT_EQ(mission.currentSequenceForTest(), operator_selects ? 6 : 5);
		EXPECT_EQ(mission.joinContextForTest().valid(), !operator_selects);
		mission.on_inactivation();
	}
}

// A survey paused while the camera triggers retraces its last leg, even from beside a leg before the survey.
TEST_F(MissionRouteJoinTest, CameraTriggerResumeTakesPrecedenceOverJoin)
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

	// Pause on leg 3-4, resume beside leg 0-1.
	pauseMission(mission, items, 4, makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 40.f, kAlt));
	navigator_test::EventRecorder events;

	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 3);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_FALSE(mission.joinContextForTest().valid());
	EXPECT_FALSE(events.sent(events::ID("mission_route_rejoin")));
}

// A join back into a finished survey keeps its target: the camera was not triggering at the pause.
TEST_F(MissionRouteJoinTest, JoinIntoFinishedCameraSurveyKeepsTarget)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	mission_item_s speed{};
	speed.nav_cmd = NAV_CMD_DO_CHANGE_SPEED;
	speed.params[1] = 8.f;
	speed.autocontinue = true;
	const std::vector<mission_item_s> items{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makeCameraTriggerItem(10.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
		makeCameraTriggerItem(0.f),
		speed,
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt),
	};

	// Pause on leg 7-8 after the survey and a speed change, resume beside survey leg 3-4.
	pauseMission(mission, items, 8, makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 40.f, kAlt));
	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	navigator_test::EventRecorder events;

	mission.on_activation();

	EXPECT_EQ(mission.currentSequenceForTest(), 4);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_TRUE(mission.joinContextForTest().valid());
	EXPECT_TRUE(events.sent(events::ID("mission_route_rejoin")));

	// The speed change after the join target is not replayed.
	while (command_sub.update(&command)) {
		EXPECT_NE(command.command, NAV_CMD_DO_CHANGE_SPEED);
	}
}

// A join selecting a nearby landing executes that item directly, even if the old cursor was elsewhere.
TEST_F(MissionRouteJoinTest, MissionSmartRejoinNearLandingSelectsLandWithoutJoin)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);

	// Takeoff, two waypoints, land; vehicle close to the land item but 6 m below mission altitude.
	std::vector<mission_item_s> mission_items = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f,    0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,    0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, -300.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 300.f,    0.f, kAlt - 10.f),
	};

	mission_s mission_state = publishMissionItems(mission_items, 23, 1, 3);

	const mission_route::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 298.f, 0.f, kAlt - 6.f);

	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishLandDetected(false);
	publishGlobalPosition(vehicle_position);
	publishLocalPosition(0.f, 8.f, 0.f);
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -100.f, 0.f, kAlt - 10.f));
	primeNavigatorState();

	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();
	markMissionResultValid();
	// The first waypoint reached, then paused on the way to the second one.
	startReachAndPause(mission, vehicle_position);

	mission.on_activation();

	// Keep the planner's selected landing index and let normal FW landing handling execute it.
	EXPECT_EQ(mission.currentSequenceForTest(), 3);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_FALSE(mission.joinContextForTest().valid());
	const auto &sp = _navigator.get_position_setpoint_triplet()->current;
	EXPECT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_LAND);
	EXPECT_DOUBLE_EQ(sp.lat, mission_items[3].lat);
	EXPECT_DOUBLE_EQ(sp.lon, mission_items[3].lon);

	// A VTOL arriving in FW must still back-transition through normal landing handling.
	mission.on_inactivation();
	mission_items[0].nav_cmd = NAV_CMD_VTOL_TAKEOFF;
	mission_items[3].nav_cmd = NAV_CMD_VTOL_LAND;
	mission_state = publishMissionItems(mission_items, 24, 1, 3);
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	primeNavigatorState();
	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();
	startReachAndPause(mission, vehicle_position);
	uORB::Subscription commands{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (commands.update(&command)) {}

	mission.on_activation();
	EXPECT_EQ(mission.currentSequenceForTest(), 3);
	EXPECT_FALSE(mission.joinContextForTest().valid());
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND);
	EXPECT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
	EXPECT_FLOAT_EQ(sp.alt, vehicle_position.alt);
	mission.on_active();
	bool back_transition_sent = false;

	while (commands.update(&command)) {
		if (command.command == vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION) {
			EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
			back_transition_sent = true;
		}
	}

	EXPECT_TRUE(back_transition_sent);
	EXPECT_NE(sp.type, position_setpoint_s::SETPOINT_TYPE_LAND);
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	primeNavigatorState();
	mission.on_active();
	EXPECT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_LAND);
}

TEST_F(MissionRouteJoinTest, VtolLandDuringFrontTransitionKeepsMulticopterLanding)
{
	// Only route Return lands a VTOL in front transition as fixed-wing.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	auto land = makeLandItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	land.nav_cmd = NAV_CMD_VTOL_LAND;
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true);
	publishGlobalPosition(position);
	publishLocalPosition();
	publishLandDetected(false);
	publishHomePosition(position);
	primeNavigatorState();
	prepareExecution(mission, {land}, 143);

	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_EQ(_navigator.get_position_setpoint_triplet()->current.type, position_setpoint_s::SETPOINT_TYPE_LAND);
}

TEST_F(MissionRouteJoinTest, PausedLandingResumeUsesCurrentPosition)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	uORB::Subscription commands{ORB_ID(vehicle_command)};
	vehicle_command_s command{};
	const float descent_altitude = kAlt + 2.f;
	const float approach_altitude = kAlt + 50.f;
	uint32_t mission_id = 431;

	for (bool is_vtol : {false, true}) {
		// Resume at LAND, inside its radius, just outside it, or after a GoTo near an earlier leg.
		for (int resume_location = 0; resume_location < 4; ++resume_location) {
			SCOPED_TRACE(::testing::Message() << "VTOL=" << is_vtol << ", resume_location=" << resume_location);
			auto takeoff = makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, approach_altitude);
			auto land = makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt);

			if (is_vtol) {
				takeoff.nav_cmd = NAV_CMD_VTOL_TAKEOFF;
				land.nav_cmd = NAV_CMD_VTOL_LAND;
			}

			// VTOL_LAND's internal BT is absent from the uploaded route, which still implies FW.
			const std::vector<mission_item_s> items{
				takeoff, makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, approach_altitude), land
			};
			const mission_s state = publishMissionItems(items, ++mission_id, 1, 2);
			publishVehicleStatus(is_vtol, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
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
			mission.on_activation();
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
			EXPECT_FALSE(mission.transitionActiveForTest());
			const bool near_land = resume_location < 2;
			const int32_t target_index = resume_location == 3 ? 1 : 2;
			EXPECT_EQ(mission.currentSequenceForTest(), target_index);

			if (near_land) {
				EXPECT_FALSE(mission.joinContextForTest().valid());
				EXPECT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_LAND);
				EXPECT_DOUBLE_EQ(sp.lat, land.lat);
				EXPECT_DOUBLE_EQ(sp.lon, land.lon);

			} else {
				ASSERT_TRUE(mission.joinContextForTest().valid());
				EXPECT_FALSE(mission.joinContextForTest().skip_altitude_requirement);
				EXPECT_EQ(sp.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
				EXPECT_FLOAT_EQ(sp.alt, approach_altitude);
				EXPECT_FLOAT_EQ(mission.joinContextForTest().projection.alt, approach_altitude);
				EXPECT_EQ(mission.joinTransitionActionForTest(), is_vtol ? MissionTestPeer::VtolTransitionAction::kFrontTransition
					  : MissionTestPeer::VtolTransitionAction::kNone);
				// Reach the join horizontally while remaining at the paused descent altitude.
				const auto &join = mission.joinContextForTest().projection;
				publishGlobalPosition({join.lat, join.lon, descent_altitude});
				primeNavigatorState();
			}

			for (int i = 0; i < 3; ++i) {
				mission.on_active();
				EXPECT_EQ(mission.workItemTypeForTest(), near_land ? MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT
					  : MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
				EXPECT_EQ(sp.type, near_land ? position_setpoint_s::SETPOINT_TYPE_LAND : position_setpoint_s::SETPOINT_TYPE_POSITION);
				EXPECT_EQ(mission.currentSequenceForTest(), target_index);
			}

			while (commands.update(&command)) {
				EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
			}

			if (is_vtol && !near_land) {
				// The FT is still available, but only after climbing to the route and aligning.
				const auto join = mission.joinContextForTest().projection;
				const float yaw = get_bearing_to_next_waypoint(join.lat, join.lon, items[target_index].lat, items[target_index].lon);
				publishGlobalPosition(join);
				publishLocalPosition(yaw);
				primeNavigatorState();
				mission.on_active();
				mission.on_active();
				bool front_transition_sent = false;

				while (commands.update(&command)) {
					if (command.command == vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION) {
						EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
						front_transition_sent = true;
					}
				}

				EXPECT_TRUE(front_transition_sent);
				EXPECT_FLOAT_EQ(sp.alt, approach_altitude);
			}
		}
	}
}

// A fixed-wing landing abort resets the index to the land start: continuing the mission flies the approach
// again instead of joining the final leg near touchdown.
TEST_F(MissionRouteJoinTest, LandingAbortFliesTheApproachAgain)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	publishFixedWingLandingMission(104);
	mission.run(false);
	markMissionResultValid();
	mission.run(true);

	for (int step = 0; (step < 10) && (mission.currentSequenceForTest() < 4); ++step) {
		reachCurrentItem(mission);
	}

	ASSERT_EQ(mission.currentSequenceForTest(), 4);

	// On final, 300 m before touchdown at 15 m, the landing is aborted.
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 1500.f, kAlt + 15.f));
	primeNavigatorState();
	mission.run(true);
	publishLandingAbort(true);
	mission.run(true);
	ASSERT_EQ(mission.currentSequenceForTest(), 2);
	publishLandingAbort(false);

	// Hold loiters above the touchdown point, then the mission is continued.
	mission.run(false);
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 60.f, 1480.f, kAlt + 30.f));
	primeNavigatorState();
	mission.run(false);
	mission.run(true);

	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_FALSE(mission.joinContextForTest().valid());
}

// The same abort while a Return flies the mission landing (RTL_TYPE 1) is kept for Mission too.
TEST_F(MissionRouteJoinTest, ReturnLandingAbortFliesTheApproachAgain)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);
	publishFixedWingLandingMission(105);
	mission.run(false);
	markMissionResultValid();
	mission.run(true);
	reachCurrentItem(mission);

	// Return: Navigator builds the executor from the latest mission topic.
	uORB::SubscriptionData<mission_s> mission_topic{ORB_ID(mission)};
	mission_topic.update();
	RtlDirectMissionLandTestPeer rtl(&_navigator, mission_topic.get());
	rtl.setRtlAlt(kAlt + 100.f);
	rtl.run(false);
	mission.run(false);
	rtl.run(true);
	mission.run(false);

	for (int step = 0; (step < 40) && !((rtl.currentSequenceForTest() == 4) && (rtl.currentNavCmdForTest() == NAV_CMD_LAND));
	     ++step) {
		const position_setpoint_s setpoint = _navigator.get_position_setpoint_triplet()->current;

		if (setpoint.valid && PX4_ISFINITE(setpoint.lat)) {
			publishGlobalPosition({setpoint.lat, setpoint.lon, setpoint.alt});
			primeNavigatorState();
		}

		rtl.run(true);
		mission.run(false);
	}

	ASSERT_EQ(rtl.currentSequenceForTest(), 4);

	// On final, 300 m before touchdown at 15 m, the landing is aborted.
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 300.f, 1500.f, kAlt + 15.f));
	primeNavigatorState();
	rtl.run(true);
	mission.run(false);
	publishLandingAbort(true);
	rtl.run(true);
	mission.run(false);
	ASSERT_EQ(mission.currentSequenceForTest(), 2);
	publishLandingAbort(false);

	// Hold loiters above the touchdown point, then the mission is continued.
	rtl.run(false);
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 60.f, 1480.f, kAlt + 30.f));
	primeNavigatorState();
	mission.run(false);
	mission.run(true);

	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_FALSE(mission.joinContextForTest().valid());
}

class MissionRouteJoinTransitionTest : public MissionRouteJoinTest, public ::testing::WithParamInterface<bool> {};

TEST_P(MissionRouteJoinTransitionTest, MissionSmartRejoinAlignsHeadingBeforeFrontTransition)
{
	const bool initially_back_transitioning = GetParam();
	setIntParam("MIS_ROUTE_JOIN", 1);
	MissionTestPeer mission(&_navigator);

	std::vector<mission_item_s> mission_items = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 150.f, 0.f, kAlt + 20.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt + 20.f),
	};

	const mission_s mission_state = publishMissionItems(mission_items, 31, 2);
	const mission_route::Position beside_first_leg = makePositionFromOffset(kBaseLat, kBaseLon, 60.f, 15.f, kAlt + 5.f);

	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false, initially_back_transitioning);
	publishLandDetected(false);
	publishGlobalPosition(beside_first_leg);
	publishLocalPosition(0.f, 5.f, 0.f);
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt - 20.f));
	primeNavigatorState();

	updateRouteCacheUntilReady(mission_state);
	mission.on_inactive();
	markMissionResultValid();
	// Waypoint 2 reached, then back in hover beside the first leg.
	startReachAndPause(mission, beside_first_leg);

	mission.on_activation();
	EXPECT_EQ(mission.joinTransitionActionForTest(), MissionTestPeer::VtolTransitionAction::kFrontTransition);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);

	uORB::Subscription vehicle_command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	// Discard commands published before reaching the join waypoint.
	while (vehicle_command_sub.update(&command)) {}

	const int32_t target_index = mission.currentSequenceForTest();
	const mission_route::Position join_projection = mission.joinContextForTest().projection;
	float expected_yaw = get_bearing_to_next_waypoint(join_projection.lat, join_projection.lon,
			     mission_items[target_index].lat, mission_items[target_index].lon);
	publishGlobalPosition(join_projection);
	publishLocalPosition(matrix::wrap_pi(expected_yaw + M_PI_F / 2.f));
	primeNavigatorState();

	mission.on_active();

	if (initially_back_transitioning) {
		// Drift outside the join acceptance radius while the back transition finishes.
		const mission_route::Position drifted_position =
			makePositionFromOffset(join_projection.lat, join_projection.lon, 0.f, 50.f, join_projection.alt);
		publishGlobalPosition(drifted_position);
		primeNavigatorState();

		for (int i = 0; i < 3; ++i) {
			ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_WAIT_FOR_BACK_TRANSITION);
			EXPECT_FALSE(PX4_ISFINITE(_navigator.get_position_setpoint_triplet()->current.yaw));
			EXPECT_DOUBLE_EQ(_navigator.get_position_setpoint_triplet()->current.lat, join_projection.lat);
			EXPECT_DOUBLE_EQ(_navigator.get_position_setpoint_triplet()->current.lon, join_projection.lon);
			EXPECT_EQ(mission.currentSequenceForTest(), target_index);
			EXPECT_EQ(_navigator.get_mission_result()->seq_reached, -1);

			while (vehicle_command_sub.update(&command)) {
				EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
			}

			mission.on_active();
		}

		// Resume from vehicle state even though neither the join nor the mission target is reached.
		expected_yaw = get_bearing_to_next_waypoint(drifted_position.lat, drifted_position.lon,
				mission_items[target_index].lat, mission_items[target_index].lon);
		publishLocalPosition(matrix::wrap_pi(expected_yaw + M_PI_F / 2.f));
		publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
		primeNavigatorState();
		mission.on_active();
	}

	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
	const position_setpoint_triplet_s &triplet = *_navigator.get_position_setpoint_triplet();
	EXPECT_NEAR(triplet.current.yaw, expected_yaw, 1e-4f);
	EXPECT_DOUBLE_EQ(triplet.current.lat, join_projection.lat);
	EXPECT_DOUBLE_EQ(triplet.current.lon, join_projection.lon);
	EXPECT_FALSE(triplet.previous.valid);
	EXPECT_FALSE(triplet.next.valid);
	EXPECT_EQ(mission.currentSequenceForTest(), target_index);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, -1);

	publishGlobalPosition(join_projection);
	primeNavigatorState();

	// Repeated reach checks must withhold the command while heading is misaligned.
	for (int i = 0; i < 3; ++i) {
		mission.on_active();
		EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
		EXPECT_EQ(mission.currentSequenceForTest(), target_index);
		EXPECT_EQ(_navigator.get_mission_result()->seq_reached, -1);

		while (vehicle_command_sub.update(&command)) {
			EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
		}
	}

	// A feasibility refresh cannot restart the join, so its alignment keeps running.
	mission.requestFeasibilityCheckForTest();
	mission.on_active();
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
	EXPECT_EQ(mission.currentSequenceForTest(), target_index);
	EXPECT_NEAR(triplet.current.yaw, expected_yaw, 1e-4f);

	while (vehicle_command_sub.update(&command)) {
		EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	}

	publishLocalPosition(expected_yaw);
	primeNavigatorState();
	mission.on_active();

	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN);
	EXPECT_EQ(mission.currentSequenceForTest(), target_index);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, -1);
	EXPECT_NEAR(triplet.current.yaw, expected_yaw, 1e-4f);
	bool front_transition_sent = false;

	while (vehicle_command_sub.update(&command)) {
		if (command.command == vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION) {
			EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
			front_transition_sent = true;
		}
	}

	EXPECT_TRUE(front_transition_sent);

	// The same during the join's front transition: no second command.
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true);
	primeNavigatorState();
	mission.requestFeasibilityCheckForTest();
	mission.on_active();
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN);
	EXPECT_EQ(mission.currentSequenceForTest(), target_index);

	while (vehicle_command_sub.update(&command)) {
		EXPECT_NE(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	}

	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	primeNavigatorState();
	mission.on_active();
	EXPECT_NE(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
	EXPECT_NE(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN);
	EXPECT_EQ(mission.currentSequenceForTest(), target_index);
	EXPECT_FALSE(mission.joinContextForTest().valid());
}

INSTANTIATE_TEST_SUITE_P(WithAndWithoutBackTransition, MissionRouteJoinTransitionTest, ::testing::Bool());

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
	ASSERT_TRUE(mission.joinContextForTest().valid());
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);

	publishGlobalPosition(mission.joinContextForTest().projection);
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
