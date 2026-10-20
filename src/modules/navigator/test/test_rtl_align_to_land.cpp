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
 * @file test_rtl_align_to_land.cpp
 *
 * WORK_ITEM_TYPE_ALIGN_TO_LAND state-machine tests for RtlDirectMissionLand.
 *
 * Kept in their own binary rather than test_RTL.cpp: dataman hands out a
 * fixed, never-reclaimed pool of client IDs per process (see
 * dataman.cpp:dataman_clients_count), and every fixture here contributes to
 * that count. Running alongside the rest of test_RTL.cpp's growing fixture
 * set pushed the shared binary over the cap, failing unrelated tests late in
 * that binary's run with "Max Dataman clients reached" instead of anything
 * about this feature. A separate binary gets its own budget.
 *
 */

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <parameters/param.h>
#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>

#include "navigator.h"
#include "rtl_direct_mission_land.h"
#include "support/navigator_dataman_test.h"
#include "support/vector_mission_item_store.h"

namespace
{

constexpr double kBaseLat = 47.397742;
constexpr double kBaseLon = 8.545594;
constexpr float kAlt = 500.f;

mission_item_s makeSafePointItem(double lat, double lon, float altitude, NAV_FRAME frame, NAV_CMD nav_cmd)
{
	mission_item_s item{};
	item.nav_cmd = nav_cmd;
	item.frame = frame;
	item.lat = lat;
	item.lon = lon;
	item.altitude = altitude;
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

} // namespace

class RtlDirectMissionLandTestPeer : public RtlDirectMissionLand
{
public:
	RtlDirectMissionLandTestPeer(Navigator *navigator, const mission_s &mission) :
		RtlDirectMissionLand(navigator, mission) {}

	void loadTestMission(const std::vector<mission_item_s> &items)
	{
		_mission_store.setItems(items);
		_mission.count = static_cast<int32_t>(_mission_store.itemCount());
	}

	// RTL::on_activation() refreshes the mode's mission copy right before it activates the mode
	void activateForTest()
	{
		_vehicle_status_sub.update();
		refreshMission();
		on_activation();
	}

	uint16_t activeNavCommand() const { return _mission_item.nav_cmd; }
	float activeYaw() const { return _mission_item.yaw; }

	using WorkItemType = MissionBase::WorkItemType;
	WorkItemType activeWorkItemType() const { return _work_item_type; }

protected:
	bool loadMissionItemFromCache(int32_t index, mission_item_s &mission_item) override
	{
		return _mission_store.loadItem(index, mission_item);
	}

private:
	navigator_test::VectorMissionItemStore _mission_store{};
};

class RtlAlignToLandTest : public NavigatorDatamanTestBase
{
protected:
	Navigator _navigator{};

	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
	}

	void TearDown() override
	{
		if (_vehicle_status_pub != nullptr) {
			orb_unadvertise(_vehicle_status_pub);
			_vehicle_status_pub = nullptr;
		}

		if (_global_position_pub != nullptr) {
			orb_unadvertise(_global_position_pub);
			_global_position_pub = nullptr;
		}

		if (_land_detected_pub != nullptr) {
			orb_unadvertise(_land_detected_pub);
			_land_detected_pub = nullptr;
		}

		if (_mission_pub != nullptr) {
			orb_unadvertise(_mission_pub);
			_mission_pub = nullptr;
		}

		param_control_autosave(true);
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

	orb_advert_t _vehicle_status_pub{nullptr};
	orb_advert_t _global_position_pub{nullptr};
	orb_advert_t _land_detected_pub{nullptr};
	orb_advert_t _mission_pub{nullptr};
};

// WHY: A VTOL landing with finite yaw, must rotate before proceeding with its landing, however it must
// also widend its acceptance heights for the landing waypoint to account for the back-transition
// WHAT: A VTOL_LAND item with a finite yaw is driven through MOVE_TO_LAND, the back-transition,
// ALIGN_TO_LAND, and back to VTOL_LAND.
TEST_F(RtlAlignToLandTest, DirectMissionLandAlignsToCommandedYawBeforeDescending)
{
	using WorkItemType = RtlDirectMissionLandTestPeer::WorkItemType;
	constexpr float kLandYaw = 1.2f;

	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.current_seq = 0;
	mission.count = 1;
	mission.land_start_index = 0;
	mission.land_index = 0;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;

	mission_item_s land = makeSafePointItem(kBaseLat, kBaseLon, kAlt, NAV_FRAME_GLOBAL, NAV_CMD_VTOL_LAND);
	land.autocontinue = true;
	land.yaw = kLandYaw;

	// GIVEN: A VTOL approaching its land point as a fixed wing, with a commanded landing yaw.
	// Publish the mission itself too: unlike a single activation check, this test calls
	// on_active() repeatedly, and MissionBase::updateMavlinkMission() reads the real `mission`
	// topic there. Without this, a fresh Subscription's first update() picks up whatever another
	// test last left on that (process-wide) topic and clobbers _mission with unrelated data.
	publishMission(mission);
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishGlobalPosition(kBaseLat, kBaseLon, kAlt);
	publishLandDetected(false);
	_navigator.get_mission_result()->valid = true;

	// is_mission_item_reached_or_completed() reads the navigator's own cached vehicle state
	// rather than MissionBase's subscriptions, so both must be kept in sync by the test.
	_navigator.get_global_position()->lat = kBaseLat;
	_navigator.get_global_position()->lon = kBaseLon;
	_navigator.get_global_position()->alt = kAlt;
	_navigator.get_global_position()->timestamp = hrt_absolute_time();
	_navigator.get_land_detected()->landed = false;
	_navigator.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	_navigator.get_vstatus()->is_vtol = true;
	_navigator.get_vstatus()->in_transition_mode = false;

	RtlDirectMissionLandTestPeer direct_mission_land{&_navigator, mission};
	direct_mission_land.loadTestMission({land});
	direct_mission_land.setRtlAlt(kAlt);

	// WHEN: The mission activates.
	direct_mission_land.activateForTest();

	// THEN: It moves to the land waypoint first as a fixed wing, with yaw ignored.
	ASSERT_EQ(direct_mission_land.activeWorkItemType(), WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND);
	EXPECT_EQ(direct_mission_land.activeNavCommand(), NAV_CMD_WAYPOINT);
	EXPECT_TRUE(std::isnan(direct_mission_land.activeYaw()));

	// WHEN: The vehicle reaches the land point and requests the back-transition to multicopter.
	direct_mission_land.on_active();

	ASSERT_EQ(direct_mission_land.activeWorkItemType(), WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION);
	EXPECT_EQ(direct_mission_land.activeNavCommand(), NAV_CMD_DO_VTOL_TRANSITION);

	// WHEN: The back-transition completes.
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	_navigator.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	_navigator.get_vstatus()->is_vtol = true;
	_navigator.get_vstatus()->in_transition_mode = false;
	direct_mission_land.on_active();

	// THEN: It aligns to the commanded landing yaw before descending, holding the current
	// altitude, and widens the altitude acceptance radius the same way MOVE_TO_LAND does.
	ASSERT_EQ(direct_mission_land.activeWorkItemType(), WorkItemType::WORK_ITEM_TYPE_ALIGN_TO_LAND);
	EXPECT_EQ(direct_mission_land.activeNavCommand(), NAV_CMD_WAYPOINT);
	EXPECT_FLOAT_EQ(direct_mission_land.activeYaw(), kLandYaw);
	EXPECT_FLOAT_EQ(_navigator.get_position_setpoint_triplet()->current.alt_acceptance_radius, FLT_MAX);

	// WHEN: The vehicle finishes rotating to the commanded landing yaw.
	_navigator.get_local_position()->heading = kLandYaw;
	_navigator.get_local_position()->heading_good_for_control = true;
	direct_mission_land.on_active();

	// THEN: It falls through to the real landing item with the commanded yaw intact.
	EXPECT_EQ(direct_mission_land.activeWorkItemType(), WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_EQ(direct_mission_land.activeNavCommand(), NAV_CMD_VTOL_LAND);
	EXPECT_FLOAT_EQ(direct_mission_land.activeYaw(), kLandYaw);
}

// WHY: With no commanded landing yaw there is nothing to align to, so the mission must
// go straight from the back-transition to the real landing item, never stopping to rotate.
// WHAT: A VTOL_LAND item with yaw = NaN skips ALIGN_TO_LAND entirely.
TEST_F(RtlAlignToLandTest, DirectMissionLandSkipsAlignWhenYawNotCommanded)
{
	using WorkItemType = RtlDirectMissionLandTestPeer::WorkItemType;

	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.current_seq = 0;
	mission.count = 1;
	mission.land_start_index = 0;
	mission.land_index = 0;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;

	mission_item_s land = makeSafePointItem(kBaseLat, kBaseLon, kAlt, NAV_FRAME_GLOBAL, NAV_CMD_VTOL_LAND);
	land.autocontinue = true;
	land.yaw = NAN;

	// GIVEN: A VTOL approaching its land point as a fixed wing, with no commanded landing yaw.
	publishMission(mission);
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishGlobalPosition(kBaseLat, kBaseLon, kAlt);
	publishLandDetected(false);
	_navigator.get_mission_result()->valid = true;

	_navigator.get_global_position()->lat = kBaseLat;
	_navigator.get_global_position()->lon = kBaseLon;
	_navigator.get_global_position()->alt = kAlt;
	_navigator.get_global_position()->timestamp = hrt_absolute_time();
	_navigator.get_land_detected()->landed = false;
	_navigator.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	_navigator.get_vstatus()->is_vtol = true;
	_navigator.get_vstatus()->in_transition_mode = false;

	RtlDirectMissionLandTestPeer direct_mission_land{&_navigator, mission};
	direct_mission_land.loadTestMission({land});
	direct_mission_land.setRtlAlt(kAlt);

	// WHEN: The mission activates and the vehicle reaches the land point to request the
	// back-transition, exactly as in the commanded-yaw case.
	direct_mission_land.activateForTest();
	ASSERT_EQ(direct_mission_land.activeWorkItemType(), WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND);

	direct_mission_land.on_active();
	ASSERT_EQ(direct_mission_land.activeWorkItemType(), WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION);
	EXPECT_EQ(direct_mission_land.activeNavCommand(), NAV_CMD_DO_VTOL_TRANSITION);

	// WHEN: The back-transition completes.
	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	_navigator.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	_navigator.get_vstatus()->is_vtol = true;
	_navigator.get_vstatus()->in_transition_mode = false;
	direct_mission_land.on_active();

	// THEN: With no yaw to align to, it lands directly -- no ALIGN_TO_LAND stop in between.
	EXPECT_EQ(direct_mission_land.activeWorkItemType(), WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_EQ(direct_mission_land.activeNavCommand(), NAV_CMD_VTOL_LAND);
	EXPECT_TRUE(std::isnan(direct_mission_land.activeYaw()));
}

// WHY: for a pure multicopter, if a mission item defines the yaw setpoint, the multicopter should
// rotate to the commanded yaw, before descending
// WHAT: A LAND item with a finite yaw is driven through MOVE_TO_LAND and ALIGN_TO_LAND to LAND
// for a pure multicopter, with no back-transition step in between.
TEST_F(RtlAlignToLandTest, DirectMissionLandMcOnlyAlignsToCommandedYawBeforeDescending)
{
	using WorkItemType = RtlDirectMissionLandTestPeer::WorkItemType;
	constexpr float kLandYaw = -0.7f;

	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.current_seq = 0;
	mission.count = 1;
	mission.land_start_index = 0;
	mission.land_index = 0;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;

	mission_item_s land = makeSafePointItem(kBaseLat, kBaseLon, kAlt, NAV_FRAME_GLOBAL, NAV_CMD_LAND);
	land.autocontinue = true;
	land.yaw = kLandYaw;

	// GIVEN: A pure multicopter (never VTOL) starting 100 m from the land point.
	const PositionYawSetpoint far_position = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);

	publishMission(mission);
	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(far_position.lat, far_position.lon, kAlt);
	publishLandDetected(false);
	_navigator.get_mission_result()->valid = true;

	_navigator.get_global_position()->lat = far_position.lat;
	_navigator.get_global_position()->lon = far_position.lon;
	_navigator.get_global_position()->alt = kAlt;
	_navigator.get_global_position()->timestamp = hrt_absolute_time();
	_navigator.get_land_detected()->landed = false;
	_navigator.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	_navigator.get_vstatus()->is_vtol = false;

	RtlDirectMissionLandTestPeer direct_mission_land{&_navigator, mission};
	direct_mission_land.loadTestMission({land});
	direct_mission_land.setRtlAlt(kAlt);

	// WHEN: The mission activates while the vehicle is still far from the land point.
	direct_mission_land.activateForTest();

	// THEN: It moves to the land waypoint first (too far away), with yaw ignored.
	ASSERT_EQ(direct_mission_land.activeWorkItemType(), WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND);
	EXPECT_EQ(direct_mission_land.activeNavCommand(), NAV_CMD_WAYPOINT);
	EXPECT_TRUE(std::isnan(direct_mission_land.activeYaw()));

	// WHEN: The vehicle arrives at the land point.
	publishGlobalPosition(kBaseLat, kBaseLon, kAlt);
	_navigator.get_global_position()->lat = kBaseLat;
	_navigator.get_global_position()->lon = kBaseLon;
	direct_mission_land.on_active();

	// THEN: With no VTOL back-transition to wait for, it goes straight from MOVE_TO_LAND to
	// aligning on the commanded landing yaw while holding the current altitude.
	ASSERT_EQ(direct_mission_land.activeWorkItemType(), WorkItemType::WORK_ITEM_TYPE_ALIGN_TO_LAND);
	EXPECT_EQ(direct_mission_land.activeNavCommand(), NAV_CMD_WAYPOINT);
	EXPECT_FLOAT_EQ(direct_mission_land.activeYaw(), kLandYaw);

	// WHEN: The vehicle finishes rotating to the commanded landing yaw.
	_navigator.get_local_position()->heading = kLandYaw;
	_navigator.get_local_position()->heading_good_for_control = true;
	direct_mission_land.on_active();

	// THEN: It falls through to the real landing item with the commanded yaw intact.
	EXPECT_EQ(direct_mission_land.activeWorkItemType(), WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_EQ(direct_mission_land.activeNavCommand(), NAV_CMD_LAND);
	EXPECT_FLOAT_EQ(direct_mission_land.activeYaw(), kLandYaw);
}

// WHY: for a pure multicopter: no commanded yaw means no ALIGN_TO_LAND stop.
// WHAT: A LAND item with yaw = NaN skips ALIGN_TO_LAND and lands directly after MOVE_TO_LAND.
TEST_F(RtlAlignToLandTest, DirectMissionLandMcOnlySkipsAlignWhenYawNotCommanded)
{
	using WorkItemType = RtlDirectMissionLandTestPeer::WorkItemType;

	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.current_seq = 0;
	mission.count = 1;
	mission.land_start_index = 0;
	mission.land_index = 0;
	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_1;

	mission_item_s land = makeSafePointItem(kBaseLat, kBaseLon, kAlt, NAV_FRAME_GLOBAL, NAV_CMD_LAND);
	land.autocontinue = true;
	land.yaw = NAN;

	// GIVEN: A pure multicopter (never VTOL) starting 100 m from the land point, with no
	// commanded landing yaw.
	const PositionYawSetpoint far_position = makePositionYawSetpointFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt);

	publishMission(mission);
	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishGlobalPosition(far_position.lat, far_position.lon, kAlt);
	publishLandDetected(false);
	_navigator.get_mission_result()->valid = true;

	_navigator.get_global_position()->lat = far_position.lat;
	_navigator.get_global_position()->lon = far_position.lon;
	_navigator.get_global_position()->alt = kAlt;
	_navigator.get_global_position()->timestamp = hrt_absolute_time();
	_navigator.get_land_detected()->landed = false;
	_navigator.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	_navigator.get_vstatus()->is_vtol = false;

	RtlDirectMissionLandTestPeer direct_mission_land{&_navigator, mission};
	direct_mission_land.loadTestMission({land});
	direct_mission_land.setRtlAlt(kAlt);

	// WHEN: The mission activates while still far from the land point.
	direct_mission_land.activateForTest();
	ASSERT_EQ(direct_mission_land.activeWorkItemType(), WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND);

	// WHEN: The vehicle arrives at the land point.
	publishGlobalPosition(kBaseLat, kBaseLon, kAlt);
	_navigator.get_global_position()->lat = kBaseLat;
	_navigator.get_global_position()->lon = kBaseLon;
	direct_mission_land.on_active();

	// THEN: With no yaw to align to, it lands directly -- no ALIGN_TO_LAND stop in between.
	EXPECT_EQ(direct_mission_land.activeWorkItemType(), WorkItemType::WORK_ITEM_TYPE_DEFAULT);
	EXPECT_EQ(direct_mission_land.activeNavCommand(), NAV_CMD_LAND);
	EXPECT_TRUE(std::isnan(direct_mission_land.activeYaw()));
}
