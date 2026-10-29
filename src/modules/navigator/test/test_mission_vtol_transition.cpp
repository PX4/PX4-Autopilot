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
 * @file test_mission_vtol_transition.cpp
 *
 * Mission VTOL transition tests. The transition sequence runs on every board, so these build without the
 * mission cache too.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <gtest/gtest.h>

#include "mission.h"
#include "navigator.h"
#include "support/mission_route_test_helpers.h"
#include "support/navigator_dataman_test.h"

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
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

#include <cmath>
#include <vector>

/* EVENT
 * @skip-file
 */

static constexpr double kBaseLat = 47.0;
static constexpr double kBaseLon = 8.0;
static constexpr float kAlt = 100.f;

static mission_item_s waypointAt(float north_m, float east_m)
{
	return makePositionItemFromOffset(kBaseLat, kBaseLon, north_m, east_m, kAlt);
}

// Navigator with its parameter update exposed.
class TestNavigator : public Navigator
{
public:
	using Navigator::updateParams;
};

/**
 * @brief Mission peer that exposes the work item and the current mission item.
 */
class MissionTestPeer : public Mission
{
public:
	explicit MissionTestPeer(Navigator *navigator) : Mission(navigator) {}

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

	// A pending check, as after a geofence upload; on_active() completes it with the current verdict.
	void requestFeasibilityCheckForTest() { _mission_check_state = MissionCheckState::Unchecked; }
	const mission_item_s &currentMissionItemForTest() const { return _mission_item; }

	WorkItemType workItemTypeForTest() const { return _work_item_type; }
	int32_t currentSequenceForTest() const { return _mission.current_seq; }
};

/**
 * @brief Fixture driving the real Mission class with dataman-backed mission data.
 */
class MissionVtolTransitionTest : public NavigatorDatamanTestBase
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

	void writeMissionItems(const std::vector<mission_item_s> &items)
	{
		for (size_t i = 0; i < items.size(); ++i) {
			mission_item_s item = items[i];
			ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, i,
							      reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		}
	}

	void primeNavigatorState()
	{
		*_navigator.get_vstatus() = _vehicle_status;
		*_navigator.get_land_detected() = _land_detected;
		*_navigator.get_global_position() = _global_position;
		*_navigator.get_local_position() = _local_position;
		*_navigator.get_home_position() = _home_position;
	}

	void publishMission(const mission_s &mission)
	{
		_mission_pub.publish(mission);
		_mission_state = mission;
	}

	void publishVehicleStatus(uint8_t vehicle_type, bool in_transition_to_fw = false, bool in_back_transition = false)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.is_vtol = true;
		status.vehicle_type = vehicle_type;
		status.in_transition_mode = in_transition_to_fw || in_back_transition;
		status.in_transition_to_fw = in_transition_to_fw;
		status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		_vehicle_status_pub.publish(status);
		_vehicle_status = status;
		primeNavigatorState();
	}

	void publishLandDetected(bool landed)
	{
		vehicle_land_detected_s land_detected{};
		land_detected.timestamp = hrt_absolute_time();
		land_detected.landed = landed;
		_land_detected_pub.publish(land_detected);
		_land_detected = land_detected;
		primeNavigatorState();
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
		primeNavigatorState();
	}

	void publishLocalPosition(float heading_rad = 0.f)
	{
		vehicle_local_position_s local_position{};
		local_position.timestamp = hrt_absolute_time();
		local_position.xy_valid = true;
		local_position.v_xy_valid = true;
		local_position.z_valid = true;
		local_position.heading = heading_rad;
		local_position.heading_good_for_control = true;
		_vehicle_local_position_pub.publish(local_position);
		_local_position = local_position;
		primeNavigatorState();
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
		primeNavigatorState();
	}

	// Publish the vehicle at position, airborne, then keep the navigator copies in sync.
	void setAirborneVehicle(const mission_route::Position &position, uint8_t vehicle_type, float heading_rad = 0.f,
				bool in_transition_to_fw = false, bool in_back_transition = false)
	{
		publishVehicleStatus(vehicle_type, in_transition_to_fw, in_back_transition);
		publishGlobalPosition(position);
		publishLocalPosition(heading_rad);
		publishLandDetected(false);
		publishHomePosition(position);
	}

	void markMissionResultValid()
	{
		*_navigator.get_mission_result() = mission_result_s{};
		_navigator.get_mission_result()->valid = true;
		_navigator.get_mission_result()->mission_id = _mission_state.mission_id;
		_navigator.get_mission_result()->geofence_id = _mission_state.geofence_id;
		_navigator.get_mission_result()->home_position_counter = _home_position.update_count;
	}

	void prepareExecution(MissionTestPeer &mission, const std::vector<mission_item_s> &items,
			      MissionTestPeer::WorkItemType work = MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT)
	{
		// Drain the commands published so far.
		vehicle_command_s command{};

		while (_command_sub.update(&command)) {}

		writeMissionItems(items);
		mission_s state{};
		state.timestamp = hrt_absolute_time();
		state.mission_id = _mission_state.mission_id + 1;
		state.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
		state.count = items.size();
		publishMission(state);
		mission.on_inactive();
		markMissionResultValid();
		_navigator.get_mission_result()->seq_reached = -1;
		mission.beginItemForTest(work);
	}

	// Drain the commands published so far and count the VTOL transition commands among them.
	int countTransitionCommands()
	{
		vehicle_command_s command{};
		int count = 0;

		while (_command_sub.update(&command)) {
			count += command.command == vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION ? 1 : 0;
		}

		return count;
	}

	mission_s _mission_state{};
	TestNavigator _navigator{};
	DatamanClient _dataman_client{};
	uORB::Publication<mission_s> _mission_pub{ORB_ID(mission)};
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_global_position_s> _vehicle_global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_local_position_s> _vehicle_local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<home_position_s> _home_position_pub{ORB_ID(home_position)};
	uORB::Subscription _command_sub{ORB_ID(vehicle_command)};

	vehicle_status_s _vehicle_status{};
	vehicle_land_detected_s _land_detected{};
	vehicle_global_position_s _global_position{};
	vehicle_local_position_s _local_position{};
	home_position_s _home_position{};
};

TEST_F(MissionVtolTransitionTest, NominalFrontTransitionAlignsBeforeItsCommand)
{
	// The alignment flies towards the next waypoint while it turns towards it; the command is sent once and the item
	// completes in fixed-wing.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	auto target = waypointAt(100.f, 0.f);
	target.yaw = M_PI_4_F;
	const float yaw = get_bearing_to_next_waypoint(position.lat, position.lon, target.lat, target.lon);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = true;
	setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, matrix::wrap_pi(yaw + M_PI_2_F));

	vehicle_command_s command{};
	prepareExecution(mission, {front_transition, target});

	const auto &triplet = *_navigator.get_position_setpoint_triplet();
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
	// The published setpoint is the alignment item: the next waypoint, not lower than now, heading towards it.
	EXPECT_DOUBLE_EQ(triplet.current.lat, target.lat);
	EXPECT_DOUBLE_EQ(triplet.current.lon, target.lon);
	EXPECT_FLOAT_EQ(triplet.current.alt, position.alt);
	EXPECT_NEAR(triplet.current.yaw, yaw, 1e-4f);
	EXPECT_NEAR(mission.currentMissionItemForTest().yaw, yaw, 1e-4f);
	mission.on_active();
	EXPECT_FALSE(_command_sub.updated());
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, -1);

	publishLocalPosition(yaw);
	mission.on_active();
	ASSERT_TRUE(_command_sub.update(&command));
	EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_DOUBLE_EQ(triplet.current.lat, position.lat);
	EXPECT_NEAR(triplet.current.yaw, yaw, 1e-4f);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);

	// The front transition runs without a second command and the item completes in fixed-wing.
	publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true);
	mission.on_active();
	EXPECT_FALSE(_command_sub.updated());
	EXPECT_EQ(mission.currentSequenceForTest(), 0);
	publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	mission.on_active();
	EXPECT_FALSE(_command_sub.updated());
	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
}

TEST_F(MissionVtolTransitionTest, BackTransitionKeepsItsParameterAndWaitsForMulticopter)
{
	// A back transition from fixed-wing keeps its uploaded parameter and completes in stable multicopter flight.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	auto back_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	back_transition.params[1] = 1.f;
	back_transition.autocontinue = true;
	setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

	vehicle_command_s command{};
	prepareExecution(mission, {back_transition, waypointAt(100.f, 0.f)});
	ASSERT_TRUE(_command_sub.update(&command));
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	EXPECT_FLOAT_EQ(command.param2, 1.f);
	publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false, true);
	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 0);
	EXPECT_FALSE(_command_sub.updated());
	publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 1);
}

TEST_F(MissionVtolTransitionTest, VtolTakeoffKeepsItsWaypointOnlyWhenMovementRemains)
{
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);

	for (bool initially_distant : {false, true}) {
		for (bool movement_remains : {false, true}) {
			SCOPED_TRACE(::testing::Message() << initially_distant << ", " << movement_remains);
			auto takeoff = waypointAt(initially_distant ? 100.f : 0.f, 0.f);
			takeoff.nav_cmd = NAV_CMD_VTOL_TAKEOFF;
			takeoff.loiter_radius = 25.f;
			takeoff.params[4] = 0.25f;
			takeoff.params[5] = 0.5f;
			const auto next = waypointAt(300.f, 0.f);
			setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, M_PI_2_F);
			vehicle_command_s command{};
			prepareExecution(mission, {takeoff, next}, MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_CLIMB);
			ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
			ASSERT_EQ(mission.currentMissionItemForTest().nav_cmd, NAV_CMD_VTOL_TAKEOFF);
			const auto &triplet = *_navigator.get_position_setpoint_triplet();
			EXPECT_FLOAT_EQ(triplet.current.alt, kAlt);
			EXPECT_DOUBLE_EQ(triplet.next.lat, next.lat);

			// Drift across the acceptance boundary while aligning; choose the continuation afterwards.
			const float final_alt = kAlt + 2.f * _navigator.get_altitude_acceptance_radius();
			publishGlobalPosition(makePositionFromOffset(takeoff.lat, takeoff.lon, movement_remains ? 100.f : 0.f, 0.f, final_alt));
			publishLocalPosition(mission.currentMissionItemForTest().yaw);
			mission.on_active();
			// Takeoff accepts being above its altitude without returning to the alignment position.
			ASSERT_EQ(mission.currentMissionItemForTest().nav_cmd, NAV_CMD_DO_VTOL_TRANSITION);
			EXPECT_EQ(mission.workItemTypeForTest(), movement_remains
				  ? MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF
				  : MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
			// The aligned takeoff is reported reached with its command, which keeps the takeoff item's other fields.
			EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
			ASSERT_TRUE(_command_sub.update(&command));
			EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
			EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
			EXPECT_FLOAT_EQ(command.param2, 0.f);
			EXPECT_FLOAT_EQ(command.param3, takeoff.loiter_radius);
			EXPECT_TRUE(std::isnan(command.param4));
			EXPECT_DOUBLE_EQ(command.param5, static_cast<double>(takeoff.params[4]));
			EXPECT_DOUBLE_EQ(command.param6, static_cast<double>(takeoff.params[5]));
			EXPECT_FLOAT_EQ(command.param7, takeoff.altitude);
			publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true);
			mission.on_active();
			EXPECT_EQ(mission.currentSequenceForTest(), 0);
			publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
			mission.on_active();
			EXPECT_EQ(mission.currentSequenceForTest(), movement_remains ? 0 : 1);
			EXPECT_EQ(mission.currentMissionItemForTest().nav_cmd, NAV_CMD_WAYPOINT);
			EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
			EXPECT_DOUBLE_EQ(triplet.current.lat, movement_remains ? takeoff.lat : next.lat);
			EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
		}
	}
}

TEST_F(MissionVtolTransitionTest, FrontTransitionByOperatorDuringTakeoffAlignmentFliesTheTakeoffWaypoint)
{
	// A front transition from the ground station or RC can end the takeoff alignment. The takeoff waypoint is then
	// flown in fixed-wing, whether the transition phase is seen or not.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	auto takeoff = waypointAt(100.f, 0.f);
	takeoff.nav_cmd = NAV_CMD_VTOL_TAKEOFF;
	const auto next = waypointAt(300.f, 0.f);

	for (bool transition_seen : {false, true}) {
		SCOPED_TRACE(transition_seen);
		setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, M_PI_2_F);

		vehicle_command_s command{};
		prepareExecution(mission, {takeoff, next}, MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_CLIMB);
		ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);

		if (transition_seen) {
			// Not aligned yet: the front transition is sent again at once and the takeoff is reported reached.
			publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true);
			mission.on_active();
			ASSERT_TRUE(_command_sub.update(&command));
			EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
			EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF);
			EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
		}

		publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
		mission.on_active();
		mission.on_active();
		EXPECT_FALSE(_command_sub.updated());
		EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
		EXPECT_EQ(mission.currentSequenceForTest(), 0);
		EXPECT_EQ(mission.currentMissionItemForTest().nav_cmd, NAV_CMD_WAYPOINT);
		EXPECT_DOUBLE_EQ(_navigator.get_position_setpoint_triplet()->current.lat, takeoff.lat);
	}
}

TEST_F(MissionVtolTransitionTest, FeasibilityRefreshRestartsAligningFrontTransition)
{
	// A feasibility refresh restarts the item: the alignment starts again from where the vehicle is, and the front
	// transition is still commanded.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto target = waypointAt(0.f, 100.f);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = true;
	setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	prepareExecution(mission, {waypointAt(0.f, 0.f), front_transition, target});

	vehicle_command_s command{};
	mission.on_active();
	ASSERT_EQ(mission.currentSequenceForTest(), 1);
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);

	// The vehicle drifts while aligning, then a pending feasibility check completes.
	const auto drifted = makePositionFromOffset(kBaseLat, kBaseLon, 40.f, 0.f, kAlt);
	const float yaw = get_bearing_to_next_waypoint(drifted.lat, drifted.lon, target.lat, target.lon);
	publishGlobalPosition(drifted);
	mission.requestFeasibilityCheckForTest();
	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, -1);
	EXPECT_NEAR(mission.currentMissionItemForTest().yaw, yaw, 1e-4f);
	EXPECT_DOUBLE_EQ(_navigator.get_position_setpoint_triplet()->current.lon, target.lon);
	EXPECT_FALSE(_command_sub.updated());

	publishLocalPosition(yaw);
	mission.on_active();
	ASSERT_TRUE(_command_sub.update(&command));
	EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 1);

	publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 2);
}

TEST_F(MissionVtolTransitionTest, FrontTransitionAfterBackTransitionAlignsTowardsTheNextWaypoint)
{
	// The back transition ends past the waypoint reached during it; alignment must not return to it.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto target = waypointAt(200.f, 0.f);
	const auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, 0.f, false, true);
	prepareExecution(mission, {waypointAt(0.f, 0.f), front_transition, target});
	mission.on_active();
	ASSERT_EQ(mission.currentSequenceForTest(), 1);
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_WAIT_FOR_BACK_TRANSITION);
	EXPECT_EQ(_navigator.get_mission_result()->seq_current, 1);

	vehicle_command_s command{};
	const auto stop = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 60.f, kAlt);
	publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(stop);
	publishLocalPosition(get_bearing_to_next_waypoint(stop.lat, stop.lon, target.lat, target.lon));
	mission.on_active();
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
	EXPECT_DOUBLE_EQ(mission.currentMissionItemForTest().lat, target.lat);
	EXPECT_FALSE(_command_sub.updated());

	mission.on_active();
	ASSERT_TRUE(_command_sub.update(&command));
	EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
}

TEST_F(MissionVtolTransitionTest, ImmediateFrontTransitionAlignsWithinTheNavigationAcceptanceRadius)
{
	// Param 2 (immediate transition) shares its slot with the acceptance radius: it must not delay the alignment,
	// and the command keeps it.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto target = waypointAt(100.f, 0.f);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = true;
	front_transition.params[1] = 1.f;
	setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	prepareExecution(mission, {front_transition, target});
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);

	vehicle_command_s command{};
	// Aligned, 2 m from where the alignment started.
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 2.f, 0.f, kAlt));
	mission.on_active();
	ASSERT_TRUE(_command_sub.update(&command));
	EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_FLOAT_EQ(command.param2, 1.f);
}

TEST_F(MissionVtolTransitionTest, BackTransitionDuringAlignmentWaitsAndAlignsAgain)
{
	// A back transition that starts during the alignment is not reversed: the item waits, then aligns again.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto target = waypointAt(100.f, 0.f);
	const float yaw = get_bearing_to_next_waypoint(position.lat, position.lon, target.lat, target.lon);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = true;
	setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, matrix::wrap_pi(yaw + M_PI_2_F));

	vehicle_command_s command{};
	prepareExecution(mission, {front_transition, target});
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);

	// Aligned during the back transition: no command.
	publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false, true);
	publishLocalPosition(yaw);
	mission.on_active();
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_WAIT_FOR_BACK_TRANSITION);
	EXPECT_FALSE(_command_sub.updated());
	EXPECT_EQ(mission.currentMissionItemForTest().nav_cmd, NAV_CMD_DO_VTOL_TRANSITION);
	mission.on_active();
	EXPECT_FALSE(_command_sub.updated());

	// In hover again, the item aligns before its command.
	publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishLocalPosition(matrix::wrap_pi(yaw + M_PI_2_F));
	mission.on_active();
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
	EXPECT_FALSE(_command_sub.updated());
	publishLocalPosition(yaw);
	mission.on_active();
	ASSERT_TRUE(_command_sub.update(&command));
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
}

TEST_F(MissionVtolTransitionTest, FrontTransitionItemDuringFrontTransitionIsSentAtOnce)
{
	// A front transition item that becomes current during a front transition is sent again at once, without an
	// alignment, and is reported reached in fixed-wing.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = true;
	setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, 0.f, true);

	vehicle_command_s command{};
	const auto target = waypointAt(0.f, 200.f);
	prepareExecution(mission, {front_transition, target});
	ASSERT_TRUE(_command_sub.update(&command));
	EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	// Without an alignment, the current setpoint is the next waypoint.
	const auto &triplet = *_navigator.get_position_setpoint_triplet();
	EXPECT_TRUE(triplet.current.valid);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
	EXPECT_DOUBLE_EQ(triplet.current.lat, target.lat);
	EXPECT_DOUBLE_EQ(triplet.current.lon, target.lon);
	mission.on_active();
	EXPECT_FALSE(_command_sub.updated());
	EXPECT_EQ(mission.currentSequenceForTest(), 0);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, -1);

	publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	mission.on_active();
	EXPECT_FALSE(_command_sub.updated());
	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
}

TEST_F(MissionVtolTransitionTest, BackTransitionItemDuringFrontTransitionIsSentAtOnce)
{
	// A back transition item that becomes current during a front transition ends it at once; the item completes
	// in multicopter flight.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	auto back_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	back_transition.autocontinue = true;
	setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, 0.f, true);

	vehicle_command_s command{};
	prepareExecution(mission, {back_transition, waypointAt(0.f, 200.f)});
	ASSERT_TRUE(_command_sub.update(&command));
	EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 0);

	publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	mission.on_active();
	EXPECT_FALSE(_command_sub.updated());
	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
}

TEST_F(MissionVtolTransitionTest, FrontTransitionByOperatorDuringTakeoffAlignmentHoldsWithoutAutocontinue)
{
	// Without autocontinue the aligned takeoff holds: a front transition from the ground station or RC does not
	// send it on to its waypoint.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	auto takeoff = waypointAt(100.f, 0.f);
	takeoff.nav_cmd = NAV_CMD_VTOL_TAKEOFF;
	takeoff.autocontinue = false;
	setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, M_PI_2_F);

	prepareExecution(mission, {takeoff, waypointAt(300.f, 0.f)},
			 MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_CLIMB);
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);

	publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

	for (int i = 0; i < 3; ++i) {
		mission.on_active();
		EXPECT_FALSE(_command_sub.updated());
		EXPECT_EQ(mission.currentSequenceForTest(), 0);
		EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
		EXPECT_EQ(mission.currentMissionItemForTest().nav_cmd, NAV_CMD_VTOL_TAKEOFF);
	}
}

TEST_F(MissionVtolTransitionTest, FrontTransitionAlignmentFliesOnAndStartsWhereTheVehicleIs)
{
	// The alignment flies towards the next waypoint, climbing to it but never descending, and is reached on heading
	// alone, far from that waypoint. The transition then starts where the vehicle is, at the alignment heading.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto start = waypointAt(0.f, 0.f);
	auto target = waypointAt(0.f, 300.f);
	target.altitude_is_relative = true;
	target.altitude = 50.f;
	const float yaw = get_bearing_to_next_waypoint(position.lat, position.lon, target.lat, target.lon);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = true;
	setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	prepareExecution(mission, {start, front_transition, target});
	mission.on_active();
	ASSERT_EQ(mission.currentSequenceForTest(), 1);
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);

	const auto &triplet = *_navigator.get_position_setpoint_triplet();
	EXPECT_DOUBLE_EQ(triplet.previous.lat, start.lat);
	EXPECT_DOUBLE_EQ(triplet.previous.lon, start.lon);
	EXPECT_DOUBLE_EQ(triplet.current.lat, target.lat);
	EXPECT_DOUBLE_EQ(triplet.current.lon, target.lon);
	EXPECT_FLOAT_EQ(triplet.current.alt, kAlt + 50.f);
	EXPECT_NEAR(triplet.current.yaw, yaw, 1e-4f);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);

	// On the way, not aligned yet: no command.
	const auto moved = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 120.f, kAlt + 10.f);
	publishGlobalPosition(moved);
	mission.on_active();
	EXPECT_FALSE(_command_sub.updated());

	vehicle_command_s command{};
	publishLocalPosition(yaw);
	mission.on_active();
	ASSERT_TRUE(_command_sub.update(&command));
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 1);
	EXPECT_DOUBLE_EQ(triplet.current.lat, moved.lat);
	EXPECT_DOUBLE_EQ(triplet.current.lon, moved.lon);
	EXPECT_FLOAT_EQ(triplet.current.alt, moved.alt);
	EXPECT_NEAR(triplet.current.yaw, yaw, 1e-4f);
	EXPECT_DOUBLE_EQ(triplet.previous.lon, moved.lon);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
	EXPECT_FALSE(mission.currentMissionItemForTest().heading_only);
}

TEST_F(MissionVtolTransitionTest, BackTransitionInHoverHoldsWhileAligning)
{
	// Only a front transition item flies on: a back transition item reached in hover turns where the vehicle is.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto target = waypointAt(0.f, 300.f);
	auto back_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	back_transition.autocontinue = true;
	setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	prepareExecution(mission, {back_transition, target});
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
	EXPECT_DOUBLE_EQ(_navigator.get_position_setpoint_triplet()->current.lat, position.lat);
	EXPECT_DOUBLE_EQ(_navigator.get_position_setpoint_triplet()->current.lon, position.lon);
	EXPECT_FALSE(mission.currentMissionItemForTest().heading_only);
}

TEST_F(MissionVtolTransitionTest, NextWaypointAfterFrontTransitionNeedsItsPosition)
{
	// Only the alignment item is reached on heading: the next waypoint is reached at its position.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto target = waypointAt(0.f, 300.f);
	const float yaw = get_bearing_to_next_waypoint(position.lat, position.lon, target.lat, target.lon);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = true;
	setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, yaw);
	prepareExecution(mission, {front_transition, target, waypointAt(0.f, 600.f)});
	mission.on_active();
	ASSERT_EQ(countTransitionCommands(), 1);

	publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	mission.on_active();
	ASSERT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_FALSE(mission.currentMissionItemForTest().heading_only);

	for (int i = 0; i < 3; ++i) {
		mission.on_active();
		EXPECT_EQ(mission.currentSequenceForTest(), 1);
	}
}

TEST_F(MissionVtolTransitionTest, FrontTransitionAlignmentKeepsTheHeadingTimeout)
{
	// The heading must still be reached: the yaw timeout applies while flying on.
	param_t param = param_find("MIS_YAW_TMT");
	float timeout = 1.f;
	ASSERT_EQ(param_set(param, &timeout), PX4_OK);
	_navigator.updateParams();

	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto target = waypointAt(0.f, 300.f);
	const float yaw = get_bearing_to_next_waypoint(position.lat, position.lon, target.lat, target.lon);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = true;
	setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, matrix::wrap_pi(yaw + M_PI_2_F));
	prepareExecution(mission, {front_transition, target});
	mission.on_active();
	EXPECT_FALSE(_navigator.get_mission_result()->failure);

	px4_usleep(1100_ms);
	mission.on_active();
	EXPECT_TRUE(_navigator.get_mission_result()->failure);
	EXPECT_EQ(countTransitionCommands(), 0);
}
