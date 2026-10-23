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
 * Mission VTOL transition tests. The transition sequencer runs on every board, so these build without the
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

#include <vector>

/* EVENT
 * @skip-file
 */

static constexpr double kBaseLat = 47.0;
static constexpr double kBaseLon = 8.0;
static constexpr float kAlt = 100.f;

/**
 * @brief Mission peer that exposes the VTOL transition state.
 */
class MissionTestPeer : public Mission
{
public:
	explicit MissionTestPeer(Navigator *navigator) : Mission(navigator) {}

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

TEST_F(MissionVtolTransitionTest, NominalFrontTransitionAlignsBeforeItsCommand)
{
	// The alignment tracks the next waypoint and waits out a back transition; the command is sent once.
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
}

TEST_F(MissionVtolTransitionTest, FrontTransitionInFixedWingIsStillCommanded)
{
	// A nominal transition already in the requested mode is still commanded, as before the sequencer.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = true;
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishGlobalPosition(position);
	publishLocalPosition();
	publishLandDetected(false);
	publishHomePosition(position);
	primeNavigatorState();

	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	prepareExecution(mission, {front_transition, makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt)}, 103);
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
}

TEST_F(MissionVtolTransitionTest, BackTransitionKeepsItsParameterAndWaitsForMulticopter)
{
	// A nominal immediate BT keeps its uploaded parameter and waits for stable MC.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	auto back_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	back_transition.params[1] = 1.f;
	back_transition.autocontinue = true;
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishGlobalPosition(position);
	publishLocalPosition();
	publishLandDetected(false);
	publishHomePosition(position);
	primeNavigatorState();

	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	prepareExecution(mission, {back_transition, makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt)}, 104);
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
}

TEST_F(MissionVtolTransitionTest, PausedTransitionIsReportedWithoutRepeatingItsCommand)
{
	// Report a paused command through the same active loop without issuing it again.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto target = makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);
	const float yaw = get_bearing_to_next_waypoint(position.lat, position.lon, target.lat, target.lon);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = false;
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(position);
	publishLocalPosition(yaw);
	publishLandDetected(false);
	publishHomePosition(position);
	primeNavigatorState();

	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	prepareExecution(mission, {front_transition, target}, 105);
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
}

TEST_F(MissionVtolTransitionTest, CursorChangeCancelsTheAlignment)
{
	// An explicit cursor change cancels the old alignment.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto target = makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);
	const float yaw = get_bearing_to_next_waypoint(position.lat, position.lon, target.lat, target.lon);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = true;
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(position);
	publishLocalPosition(matrix::wrap_pi(yaw + M_PI_2_F));
	publishLandDetected(false);
	publishHomePosition(position);
	primeNavigatorState();

	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	prepareExecution(mission, {front_transition, target}, 106);
	ASSERT_TRUE(mission.transitionActiveForTest());
	ASSERT_TRUE(mission.set_current_mission_index(1));
	EXPECT_FALSE(mission.transitionActiveForTest());
	EXPECT_FALSE(command_sub.updated());
}

TEST_F(MissionVtolTransitionTest, VtolTakeoffKeepsItsWaypointOnlyWhenMovementRemains)
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

TEST_F(MissionVtolTransitionTest, ManualFrontTransitionDuringAlignmentStillSendsCommand)
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

TEST_F(MissionVtolTransitionTest, TransitionAlreadyInModeIsReportedAsCurrentItem)
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

TEST_F(MissionVtolTransitionTest, AlignedTransitionIsReportedReachedWithItsCommand)
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

TEST_F(MissionVtolTransitionTest, BackTransitionInHoverAlignsBeforeItsCommand)
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

TEST_F(MissionVtolTransitionTest, TransitionWaitingForBackTransitionIsReportedAsCurrentItem)
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

TEST_F(MissionVtolTransitionTest, FrontTransitionAfterBackTransitionAlignsWhereTheVehicleIs)
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

TEST_F(MissionVtolTransitionTest, FeasibilityRefreshRestartsAligningFrontTransition)
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

TEST_F(MissionVtolTransitionTest, FrontTransitionItemDuringFrontTransitionTracksTheNextWaypoint)
{
	// Started during a front transition, the item is sent at once without an alignment, but still publishes the
	// next waypoint as target, so the fixed-wing side keeps tracking a waypoint.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const auto target = makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 200.f, kAlt);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = true;
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true);
	publishGlobalPosition(position);
	publishLocalPosition();
	publishLandDetected(false);
	publishHomePosition(position);
	primeNavigatorState();

	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	// Like an activation, prepareExecution() resets the triplet before it sets the item.
	prepareExecution(mission, {front_transition, target}, 146);
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	const auto &triplet = *_navigator.get_position_setpoint_triplet();
	EXPECT_TRUE(triplet.current.valid);
	EXPECT_EQ(triplet.current.type, position_setpoint_s::SETPOINT_TYPE_POSITION);
	EXPECT_DOUBLE_EQ(triplet.current.lat, target.lat);
	EXPECT_DOUBLE_EQ(triplet.current.lon, target.lon);

	mission.on_active();
	EXPECT_FALSE(command_sub.updated());
	EXPECT_TRUE(triplet.current.valid);
	EXPECT_EQ(mission.currentSequenceForTest(), 0);

	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	primeNavigatorState();
	mission.on_active();
	EXPECT_EQ(mission.currentSequenceForTest(), 1);
	EXPECT_EQ(_navigator.get_mission_result()->seq_reached, 0);
}

TEST_F(MissionVtolTransitionTest, LandingDetectedDuringAlignmentSendsNoTransition)
{
	// A landing detected during the alignment does not end it: no front transition is sent on the ground.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	auto front_transition = makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	front_transition.autocontinue = true;
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(position);
	publishLocalPosition();
	publishLandDetected(false);
	publishHomePosition(position);
	primeNavigatorState();
	prepareExecution(mission, {makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt), front_transition,
				   makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 200.f, kAlt)
				  }, 147);

	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	mission.on_active();
	ASSERT_EQ(mission.currentSequenceForTest(), 1);
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);

	publishLandDetected(true);
	primeNavigatorState();

	for (int i = 0; i < 3; ++i) {
		mission.on_active();
		EXPECT_FALSE(command_sub.updated());
		EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
	}
}

TEST_F(MissionVtolTransitionTest, LandingDetectedDuringTakeoffAlignmentSendsNoTransition)
{
	// A landing detected during the takeoff alignment sends no front transition: the alignment goes on, and once
	// aligned on the ground the takeoff continues as a takeoff item.
	MissionTestPeer mission(&_navigator);
	const auto position = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	auto takeoff = makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);
	takeoff.nav_cmd = NAV_CMD_VTOL_TAKEOFF;
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(position);
	publishLocalPosition(M_PI_2_F);
	publishLandDetected(false);
	publishHomePosition(position);
	primeNavigatorState();

	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	while (command_sub.update(&command)) {}

	prepareExecution(mission, {takeoff, makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt)}, 148,
			 MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_CLIMB);
	ASSERT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);
	mission.on_active();

	publishLandDetected(true);
	primeNavigatorState();
	mission.on_active();
	EXPECT_FALSE(command_sub.updated());
	EXPECT_EQ(mission.workItemTypeForTest(), MissionTestPeer::WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING);

	publishLocalPosition(mission.currentMissionItemForTest().yaw);
	primeNavigatorState();
	mission.on_active();
	EXPECT_FALSE(command_sub.updated());
	EXPECT_FALSE(mission.transitionActiveForTest());
	EXPECT_EQ(mission.currentMissionItemForTest().nav_cmd, NAV_CMD_VTOL_TAKEOFF);
	EXPECT_EQ(mission.currentSequenceForTest(), 0);
}

TEST_F(MissionVtolTransitionTest, VtolLandDuringFrontTransitionKeepsMulticopterLanding)
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
