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

		// Each Mission reads the last published mission, so start every test from an empty one.
		mission_s mission{};
		mission.timestamp = hrt_absolute_time();
		_mission_pub.publish(mission);

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

	// Each helper writes the navigator's copy, which MissionBlock reads, and publishes it for Mission's subscriptions.
	void publishVehicleStatus(uint8_t vehicle_type, bool in_transition_to_fw = false, bool in_back_transition = false)
	{
		vehicle_status_s &status = *_navigator.get_vstatus();
		status = {};
		status.timestamp = hrt_absolute_time();
		status.is_vtol = true;
		status.vehicle_type = vehicle_type;
		status.in_transition_mode = in_transition_to_fw || in_back_transition;
		status.in_transition_to_fw = in_transition_to_fw;
		status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		_vehicle_status_pub.publish(status);
	}

	void publishGlobalPosition(const mission_route::Position &position)
	{
		vehicle_global_position_s &global_position = *_navigator.get_global_position();
		global_position = {};
		global_position.timestamp = hrt_absolute_time();
		global_position.lat = position.lat;
		global_position.lon = position.lon;
		global_position.alt = position.alt;
		_vehicle_global_position_pub.publish(global_position);
	}

	void publishLocalPosition(float heading_rad = 0.f)
	{
		vehicle_local_position_s &local_position = *_navigator.get_local_position();
		local_position = {};
		local_position.timestamp = hrt_absolute_time();
		local_position.xy_valid = true;
		local_position.v_xy_valid = true;
		local_position.z_valid = true;
		local_position.heading = heading_rad;
		local_position.heading_good_for_control = true;
		_vehicle_local_position_pub.publish(local_position);
	}

	// Publish the vehicle airborne at position, with Home at the same place.
	void setAirborneVehicle(const mission_route::Position &position, uint8_t vehicle_type, float heading_rad = 0.f,
				bool in_transition_to_fw = false, bool in_back_transition = false)
	{
		publishVehicleStatus(vehicle_type, in_transition_to_fw, in_back_transition);
		publishGlobalPosition(position);
		publishLocalPosition(heading_rad);

		vehicle_land_detected_s &land_detected = *_navigator.get_land_detected();
		land_detected = {};
		land_detected.timestamp = hrt_absolute_time();
		_land_detected_pub.publish(land_detected);

		home_position_s &home = *_navigator.get_home_position();
		home = {};
		home.timestamp = hrt_absolute_time();
		home.valid_hpos = true;
		home.valid_alt = true;
		home.lat = position.lat;
		home.lon = position.lon;
		home.alt = position.alt;
		_home_position_pub.publish(home);
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
		state.mission_id = ++_mission_id;
		state.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
		state.count = items.size();
		_mission_pub.publish(state);
		mission.on_inactive();

		// A valid check result for this mission, fence and Home.
		mission_result_s &result = *_navigator.get_mission_result();
		result = {};
		result.valid = true;
		result.mission_id = state.mission_id;
		result.geofence_id = state.geofence_id;
		result.home_position_counter = _navigator.get_home_position()->update_count;
		result.seq_reached = -1;
		mission.beginItemForTest(work);
	}

	uint32_t _mission_id{0};
	Navigator _navigator{};
	DatamanClient _dataman_client{};
	uORB::Publication<mission_s> _mission_pub{ORB_ID(mission)};
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_global_position_s> _vehicle_global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_local_position_s> _vehicle_local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<home_position_s> _home_position_pub{ORB_ID(home_position)};
	uORB::Subscription _command_sub{ORB_ID(vehicle_command)};
};

TEST_F(MissionVtolTransitionTest, NominalFrontTransitionAlignsBeforeItsCommand)
{
	// The alignment holds position and turns towards the next waypoint; the command is sent once and the item
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
	// The published setpoint is the alignment item: hold here, heading towards the next waypoint.
	EXPECT_DOUBLE_EQ(triplet.current.lat, position.lat);
	EXPECT_DOUBLE_EQ(triplet.current.lon, position.lon);
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
	// The hold point is also the previous setpoint while the transition starts.
	EXPECT_DOUBLE_EQ(triplet.previous.lat, position.lat);
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

TEST_F(MissionVtolTransitionTest, FrontTransitionWithoutLaterWaypointIsSentAtOnce)
{
	// With no position item after it there is nothing to turn towards: the command is sent at once.
	MissionTestPeer mission(&_navigator);
	setAirborneVehicle(makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt), vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

	vehicle_command_s command{};
	prepareExecution(mission, {makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW)});
	ASSERT_TRUE(_command_sub.update(&command));
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
}

TEST_F(MissionVtolTransitionTest, VtolTakeoffKeepsItsWaypointOnlyWhenMovementRemains)
{
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);

	for (bool movement_remains : {false, true}) {
		SCOPED_TRACE(movement_remains);
		// Starts away from the takeoff waypoint and drifts onto it while aligning, or the reverse.
		auto takeoff = waypointAt(0.f, movement_remains ? 0.f : 100.f);
		takeoff.nav_cmd = NAV_CMD_VTOL_TAKEOFF;
		const auto next = waypointAt(300.f, 0.f);
		setAirborneVehicle(position, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, M_PI_F);
		vehicle_command_s command{};
		prepareExecution(mission, {takeoff, next}, MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_CLIMB);
		ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
		ASSERT_EQ(mission.currentMissionItemForTest().nav_cmd, NAV_CMD_VTOL_TAKEOFF);
		const auto &triplet = *_navigator.get_position_setpoint_triplet();
		EXPECT_FLOAT_EQ(triplet.current.alt, kAlt);
		EXPECT_DOUBLE_EQ(triplet.next.lat, next.lat);
		// Turns in place towards the takeoff waypoint, not the next one (north).
		EXPECT_DOUBLE_EQ(triplet.current.lat, position.lat);
		EXPECT_DOUBLE_EQ(triplet.current.lon, position.lon);
		EXPECT_NEAR(mission.currentMissionItemForTest().yaw,
			    get_bearing_to_next_waypoint(position.lat, position.lon, takeoff.lat, takeoff.lon), 1e-4f);
		// Not aligned yet: no command.
		mission.on_active();
		EXPECT_FALSE(_command_sub.updated());

		// The continuation is chosen where the vehicle is when the command is sent.
		const float final_alt = kAlt + 2.f * _navigator.get_altitude_acceptance_radius();
		publishGlobalPosition(makePositionFromOffset(takeoff.lat, takeoff.lon, movement_remains ? 100.f : 0.f, 0.f, final_alt));
		publishLocalPosition(mission.currentMissionItemForTest().yaw);
		mission.on_active();
		// Takeoff accepts being above its altitude without returning to the alignment position.
		ASSERT_EQ(mission.currentMissionItemForTest().nav_cmd, NAV_CMD_DO_VTOL_TRANSITION);
		EXPECT_EQ(mission.workItemTypeForTest(), movement_remains
			  ? MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF
			  : MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
		// The aligned takeoff is reported reached when its command is sent.
		EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
		ASSERT_TRUE(_command_sub.update(&command));
		EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
		EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
		EXPECT_FLOAT_EQ(command.param2, 0.f);
		EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
		publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true);
		mission.on_active();
		EXPECT_EQ(mission.currentSequenceForTest(), 0);
		publishVehicleStatus(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
		mission.on_active();
		EXPECT_EQ(mission.currentSequenceForTest(), movement_remains ? 0 : 1);
		EXPECT_EQ(mission.currentMissionItemForTest().nav_cmd, NAV_CMD_WAYPOINT);
		EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
		EXPECT_DOUBLE_EQ(triplet.current.lat, movement_remains ? takeoff.lat : next.lat);
		EXPECT_DOUBLE_EQ(triplet.current.lon, movement_remains ? takeoff.lon : next.lon);
		EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
	}
}
