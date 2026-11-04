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
 * Precision Land activation tests.
 */

#include <gtest/gtest.h>

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

#include "mission_base.h"
#include "navigator.h"
#include "precland.h"
#include "support/navigator_dataman_test.h"

namespace
{

constexpr double kBaseLat = 47.397742;
constexpr double kBaseLon = 8.545594;
constexpr float kAlt = 500.f;

// about one kilometre north of the vehicle
constexpr double kFarLat = kBaseLat + 0.009;

// a mission land point about 6 m north, inside the default acceptance radius
constexpr double kLandLat = kBaseLat + 0.000054;

// where the vehicle is when the mission reaches its land item, about 200 m south
constexpr double kApproachLat = kBaseLat - 0.0018;

} // namespace

// Runs MissionBase::handleLanding() the way Mission::setActiveMissionItems() does
class MissionLandingTestPeer : public MissionBase
{
public:
	explicit MissionLandingTestPeer(Navigator *navigator) :
		MissionBase(navigator, 8, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {}

	void setActiveMissionItems() override {}
	bool setNextMissionItem() override { return false; }

	// returns true once the precision land work item started
	bool runLandItem(const mission_item_s &land_item)
	{
		_land_detected_sub.update();
		_vehicle_status_sub.update();
		_global_pos_sub.update();

		_mission_item = land_item;
		WorkItemType new_work_item_type{WorkItemType::WORK_ITEM_TYPE_DEFAULT};
		mission_item_s next_mission_items[2] {};
		size_t num_found_items{0};
		handleLanding(new_work_item_type, next_mission_items, num_found_items);

		if (new_work_item_type != WorkItemType::WORK_ITEM_TYPE_PRECISION_LAND) {
			mission_item_to_position_setpoint(_mission_item, &_navigator->get_position_setpoint_triplet()->current);
		}

		_work_item_type = new_work_item_type;
		return new_work_item_type == WorkItemType::WORK_ITEM_TYPE_PRECISION_LAND;
	}
};

class PrecLandTest : public NavigatorDatamanTestBase
{
protected:
	Navigator _navigator{};
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_global_position_s> _global_position_pub{ORB_ID(vehicle_global_position)};

	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();

		_navigator.get_local_position()->ref_lat = kBaseLat;
		_navigator.get_local_position()->ref_lon = kBaseLon;
		*_navigator.get_position_setpoint_triplet() = {};
		setVehicle(kBaseLat, vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND);
	}

	void TearDown() override
	{
		param_control_autosave(true);
	}

	void setVehicle(double lat, uint8_t nav_state)
	{
		_navigator.get_global_position()->lat = lat;
		_navigator.get_global_position()->lon = kBaseLon;
		_navigator.get_global_position()->alt = kAlt;
		_navigator.get_vstatus()->nav_state = nav_state;
		_navigator.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

		vehicle_status_s vehicle_status{};
		vehicle_status.timestamp = hrt_absolute_time();
		vehicle_status.nav_state = nav_state;
		vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		_vehicle_status_pub.publish(vehicle_status);

		vehicle_land_detected_s land_detected{};
		land_detected.timestamp = hrt_absolute_time();
		land_detected.landed = false;
		_land_detected_pub.publish(land_detected);

		vehicle_global_position_s global_position{};
		global_position.timestamp = hrt_absolute_time();
		global_position.lat = lat;
		global_position.lon = kBaseLon;
		global_position.alt = kAlt;
		_global_position_pub.publish(global_position);
	}

	// leave a setpoint behind the way a previous mode would
	void leaveSetpoint(uint8_t type, double lat)
	{
		position_setpoint_s &current = _navigator.get_position_setpoint_triplet()->current;
		current.timestamp = hrt_absolute_time();
		current.valid = true;
		current.type = type;
		current.lat = lat;
		current.lon = kBaseLon;
		current.alt = kAlt;
	}

	const position_setpoint_s &current() { return _navigator.get_position_setpoint_triplet()->current; }

	// Runs a mission land item with precision landing while vehicle_status reports nav_state, the
	// move to land waypoint first, then the precision landing once it is accepted a few metres short
	void runMissionPrecisionLanding(uint8_t nav_state)
	{
		MissionLandingTestPeer mission{&_navigator};

		mission_item_s land_item{};
		land_item.nav_cmd = NAV_CMD_LAND;
		land_item.lat = kLandLat;
		land_item.lon = kBaseLon;
		land_item.altitude = kAlt;
		land_item.land_precision = 2;
		land_item.autocontinue = true;

		setVehicle(kApproachLat, nav_state);
		ASSERT_FALSE(mission.runLandItem(land_item)); // move to land first

		setVehicle(kBaseLat, nav_state);
		ASSERT_TRUE(mission.runLandItem(land_item));
	}
};

// Switching to Precision Land while a mission waypoint far away is the current setpoint
// must land at the current position rather than fly to that waypoint first
TEST_F(PrecLandTest, EnteredAsAModeLandsAtTheCurrentPosition)
{
	leaveSetpoint(position_setpoint_s::SETPOINT_TYPE_POSITION, kFarLat);

	_navigator.get_precland()->on_activation();

	EXPECT_TRUE(current().valid);
	EXPECT_NEAR(current().lat, kBaseLat, 1e-9);
	EXPECT_NEAR(current().lon, kBaseLon, 1e-9);
}

// Without any setpoint it lands at the current position
TEST_F(PrecLandTest, EnteredAsAModeWithoutASetpointLandsAtTheCurrentPosition)
{
	_navigator.get_precland()->on_activation();

	EXPECT_TRUE(current().valid);
	EXPECT_NEAR(current().lat, kBaseLat, 1e-9);
	EXPECT_NEAR(current().lon, kBaseLon, 1e-9);
}

// A mission land item with precision landing keeps its land point when the move to land
// waypoint is accepted short of it, as it did before each mode owned its triplet reset
TEST_F(PrecLandTest, StartedByAMissionKeepsTheLandPoint)
{
	ASSERT_NO_FATAL_FAILURE(runMissionPrecisionLanding(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION));

	EXPECT_TRUE(current().valid);
	EXPECT_NEAR(current().lat, kLandLat, 1e-9);
	EXPECT_NEAR(current().lon, kBaseLon, 1e-9);
}

// Return commanded during a mission landing keeps Mission running, so the mission starts
// Precision Land while vehicle_status already reports Return, and it keeps the land point
TEST_F(PrecLandTest, StartedByAMissionLandingDuringReturnKeepsTheLandPoint)
{
	ASSERT_NO_FATAL_FAILURE(runMissionPrecisionLanding(vehicle_status_s::NAVIGATION_STATE_AUTO_RTL));

	EXPECT_TRUE(current().valid);
	EXPECT_NEAR(current().lat, kLandLat, 1e-9);
	EXPECT_NEAR(current().lon, kBaseLon, 1e-9);
}

// Started by a mission while no setpoint is valid, for example a mission resumed at its land
// item above the land point, it lands at the current position
TEST_F(PrecLandTest, StartedByAMissionWithoutASetpointLandsAtTheCurrentPosition)
{
	setVehicle(kBaseLat, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);

	_navigator.get_precland()->on_activation();

	EXPECT_TRUE(current().valid);
	EXPECT_NEAR(current().lat, kBaseLat, 1e-9);
	EXPECT_NEAR(current().lon, kBaseLon, 1e-9);
}
