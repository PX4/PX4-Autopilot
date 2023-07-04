/****************************************************************************
 *
 *   Copyright (C) 2022-2023 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include "FeasibilityChecker.hpp"
#include <lib/geo/geo.h>


// to run: make tests TESTFILTER=FeasibilityChecker
class FeasibilityCheckerTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
	}
};

class TestFeasibilityChecker : public FeasibilityChecker
{
public:
	TestFeasibilityChecker() : FeasibilityChecker() {}
	void paramsChanged() {FeasibilityChecker::updateParamsImpl();}

	void publishHomePosition(double lat, double lon, float alt)
	{
		home_position_s home = {};
		home.alt = alt;
		home.valid_alt = true;
		home.lat = lat;
		home.lon = lon;
		home.valid_hpos = true;
		orb_advert_t home_pub = orb_advertise(ORB_ID(home_position), &home);
		orb_publish(ORB_ID(home_position), home_pub, &home);
	}

	void publishCurrentPosition(double lat, double lon)
	{
		vehicle_global_position_s gpos = {};
		gpos.lat = lat;
		gpos.lon = lon;
		orb_advert_t gpos_pub = orb_advertise(ORB_ID(vehicle_global_position), &gpos);
		orb_publish(ORB_ID(vehicle_global_position), gpos_pub, &gpos);
	}

	void publishLanded(bool landed)
	{
		vehicle_land_detected_s land_detected = {};
		land_detected.landed = true;
		orb_advert_t landed_pub = orb_advertise(ORB_ID(vehicle_land_detected), &land_detected);
		orb_publish(ORB_ID(vehicle_land_detected), landed_pub, &land_detected);
	}

	void publishVehicleType(int vehicle_type)
	{
		vehicle_status_s status = {};
		status.vehicle_type = vehicle_type;
		orb_advert_t status_pub = orb_advertise(ORB_ID(vehicle_status), &status);
		orb_publish(ORB_ID(vehicle_status), status_pub, &status);
	}

};

TEST_F(FeasibilityCheckerTest, instantiation)
{
	FeasibilityChecker checker;
}

TEST_F(FeasibilityCheckerTest, mission_item_validity)
{
	TestFeasibilityChecker checker;


	checker.publishLanded(true);

	// supported mission item should pass
	mission_item_s mission_item = {};
	mission_item.nav_cmd = NAV_CMD_WAYPOINT;
	checker.processNextItem(mission_item, 0, 1);
	ASSERT_EQ(checker.someCheckFailed(), false);

	// when landed the first item cannot be a land item
	checker.reset();
	checker.publishHomePosition(0, 0, 100.f);
	checker.publishLanded(true);
	mission_item.nav_cmd = NAV_CMD_LAND;
	bool ret = checker.processNextItem(mission_item, 0, 1);
	ASSERT_EQ(checker.someCheckFailed(), true);

	// mission item validity failed
	ASSERT_EQ(ret, false);

	checker.reset();
	mission_item.nav_cmd = NAV_CMD_TAKEOFF;
	mission_item.altitude_is_relative = true;
	ret = checker.processNextItem(mission_item, 0, 5);

	// home alt is not valid but we have a relative mission item, should fail immediately
	ASSERT_EQ(ret, false);
}

TEST_F(FeasibilityCheckerTest, check_dist_first_waypoint)
{
	// GIVEN: MIS_DIST_1WP set to 500m
	TestFeasibilityChecker checker;
	checker.publishLanded(true);
	param_t param = param_handle(px4::params::MIS_DIST_1WP);
	float max_dist = 500.0f;
	param_set(param, &max_dist);
	checker.paramsChanged();
	mission_item_s mission_item = {};
	mission_item.nav_cmd = NAV_CMD_WAYPOINT;
	double lat_new, lon_new;

	// GIVEN: no valid Current position

	// THEN: always pass
	checker.processNextItem(mission_item, 0, 1);
	ASSERT_EQ(checker.someCheckFailed(), false);

	// BUT WHEN: valid current position, first WP 501m away from current pos
	checker.reset();
	checker.publishLanded(true);
	checker.publishCurrentPosition(0, 0);
	waypoint_from_heading_and_distance(0, 0, 0, 501, &lat_new, &lon_new);
	mission_item.lat = lat_new;
	mission_item.lon = lon_new;

	// THEN: fail
	checker.processNextItem(mission_item, 0, 1);
	ASSERT_EQ(checker.someCheckFailed(), true);

	// BUT WHEN: valid current position fist WP 499m away from current
	checker.reset();
	checker.publishLanded(true);
	checker.publishCurrentPosition(0, 0);
	waypoint_from_heading_and_distance(0, 0, 0, 499, &lat_new, &lon_new);
	mission_item.lat = lat_new;
	mission_item.lon = lon_new;

	// THEN: pass
	checker.processNextItem(mission_item, 0, 1);
	ASSERT_EQ(checker.someCheckFailed(), false);
}

TEST_F(FeasibilityCheckerTest, check_below_home)
{
	TestFeasibilityChecker checker;
	checker.publishLanded(true);
	//checker.publishHomePosition(0,0,100);


	mission_item_s mission_item = {};
	mission_item.nav_cmd = NAV_CMD_WAYPOINT;
	mission_item.altitude = 50;
	mission_item.altitude_is_relative = true;

	checker.processNextItem(mission_item, 0, 1);

	// this is done to invalidate the home position
	checker.reset();
	checker.publishLanded(true);
	checker.processNextItem(mission_item, 0, 1);

	// cannot have relative altitude without valid home position
	ASSERT_EQ(checker.someCheckFailed(), true);
}

TEST_F(FeasibilityCheckerTest, check_takeoff)
{
	TestFeasibilityChecker checker;
	checker.publishLanded(true);
	checker.publishHomePosition(0, 0, 100);


	// takeoff altitude is smaller than acceptance radius, should fail
	mission_item_s mission_item = {};
	mission_item.nav_cmd = NAV_CMD_TAKEOFF;
	mission_item.altitude = -5.0f;
	mission_item.altitude_is_relative = true;

	checker.processNextItem(mission_item, 0, 1);

	ASSERT_EQ(checker.someCheckFailed(), true);

	checker.reset();
	checker.publishLanded(true);
	checker.publishHomePosition(0, 0, 100);


	// takeoff altitude is larger than home altitude, should pass
	mission_item.altitude = 0.1f;
	checker.processNextItem(mission_item, 0, 1);

	ASSERT_EQ(checker.someCheckFailed(), false);


	// takeoff item needs to be first item
	checker.reset();
	checker.publishLanded(true);
	checker.publishHomePosition(0, 0, 100);

	mission_item.nav_cmd = NAV_CMD_WAYPOINT;

	checker.processNextItem(mission_item, 0, 2);

	mission_item.nav_cmd = NAV_CMD_TAKEOFF;

	checker.processNextItem(mission_item, 1, 2);

	ASSERT_EQ(checker.someCheckFailed(), true);
}

TEST_F(FeasibilityCheckerTest, fixed_wing_land_approach)
{
	TestFeasibilityChecker checker;
	checker.publishLanded(true);
	checker.publishHomePosition(0, 0, 100);
	checker.publishVehicleType(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

	mission_item_s mission_item = {};
	mission_item.nav_cmd = NAV_CMD_WAYPOINT;
	mission_item.altitude = 50;
	mission_item.altitude_is_relative = true;

	checker.processNextItem(mission_item, 0, 2);

	mission_item.nav_cmd = NAV_CMD_LAND;
	mission_item.altitude = 60;
	checker.processNextItem(mission_item, 1, 2);

	ASSERT_EQ(checker.someCheckFailed(), true);


	// landing point should not be within loiter radius of previous waypoint
	checker.reset();
	checker.publishLanded(true);
	checker.publishVehicleType(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	checker.publishHomePosition(0, 0, 100);
	mission_item.nav_cmd = NAV_CMD_LOITER_TO_ALT;
	mission_item.altitude = 50;
	mission_item.loiter_radius = 100;

	checker.processNextItem(mission_item, 0, 2);

	double lat_new, lon_new;
	waypoint_from_heading_and_distance(mission_item.lat, mission_item.lon, 0, 99, &lat_new, &lon_new);
	mission_item.lat = lat_new;
	mission_item.lon = lon_new;
	mission_item.nav_cmd = NAV_CMD_LAND;
	mission_item.altitude = 40;

	checker.processNextItem(mission_item, 1, 2);

	ASSERT_EQ(checker.someCheckFailed(), true);

	// only loiter to alt or plain waypoint type are allowed before landing
	checker.reset();
	checker.publishLanded(true);
	checker.publishVehicleType(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	checker.publishHomePosition(0, 0, 100);
	mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
	mission_item.altitude = 50;
	mission_item.loiter_radius = 50;

	checker.processNextItem(mission_item, 0, 2);

	waypoint_from_heading_and_distance(mission_item.lat, mission_item.lon, 0, 99, &lat_new, &lon_new);
	mission_item.lat = lat_new;
	mission_item.lon = lon_new;
	mission_item.nav_cmd = NAV_CMD_LAND;
	mission_item.altitude = 40;

	checker.processNextItem(mission_item, 1, 2);

	ASSERT_EQ(checker.someCheckFailed(), true);

	// fail the glide slope
	checker.reset();
	checker.publishLanded(true);
	checker.publishVehicleType(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	checker.publishHomePosition(0, 0, 100);
	mission_item.nav_cmd = NAV_CMD_WAYPOINT;
	mission_item.altitude = 50;
	mission_item.loiter_radius = 50;

	checker.processNextItem(mission_item, 0, 2);

	waypoint_from_heading_and_distance(mission_item.lat, mission_item.lon, 0, 1, &lat_new, &lon_new);
	mission_item.lat = lat_new;
	mission_item.lon = lon_new;
	mission_item.nav_cmd = NAV_CMD_LAND;
	mission_item.altitude = 40;

	checker.processNextItem(mission_item, 1, 2);

	ASSERT_EQ(checker.someCheckFailed(), true);

	// fixed wing land approach checks should not be executed for rotary wing
	checker.publishVehicleType(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	checker.reset();
	checker.publishLanded(true);
	checker.publishHomePosition(0, 0, 100);
	mission_item.nav_cmd = NAV_CMD_WAYPOINT;
	mission_item.altitude = 50;
	mission_item.loiter_radius = 50;

	checker.processNextItem(mission_item, 0, 2);

	waypoint_from_heading_and_distance(mission_item.lat, mission_item.lon, 0, 1, &lat_new, &lon_new);
	mission_item.lat = lat_new;
	mission_item.lon = lon_new;
	mission_item.nav_cmd = NAV_CMD_LAND;
	mission_item.altitude = 40;

	checker.processNextItem(mission_item, 1, 2);

	ASSERT_EQ(checker.someCheckFailed(), false);
}

TEST_F(FeasibilityCheckerTest, fixed_wing_landing)
{
	// multiple do land start are not allowed
	TestFeasibilityChecker checker;
	checker.publishLanded(true);
	checker.publishHomePosition(0, 0, 100);
	checker.publishVehicleType(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

	mission_item_s mission_item = {};
	mission_item.nav_cmd = NAV_CMD_DO_LAND_START;

	checker.processNextItem(mission_item, 0, 2);
	checker.processNextItem(mission_item, 1, 2);

	ASSERT_EQ(checker.someCheckFailed(), true);


	// cannot start with land waypoint
	checker.reset();
	checker.publishVehicleType(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	mission_item.nav_cmd = NAV_CMD_LAND;
	checker.processNextItem(mission_item, 0, 2);
	ASSERT_EQ(checker.someCheckFailed(), true);

	// cannot have land start before RTL
	checker.reset();
	checker.publishVehicleType(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	mission_item.nav_cmd = NAV_CMD_DO_LAND_START;
	checker.processNextItem(mission_item, 0, 2);

	mission_item.nav_cmd = NAV_CMD_RETURN_TO_LAUNCH;
	checker.processNextItem(mission_item, 1, 2);
	ASSERT_EQ(checker.someCheckFailed(), true);

	// cannot have land start after landing waypoint
	checker.reset();
	checker.publishHomePosition(0, 0, 100);
	checker.publishVehicleType(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	mission_item.nav_cmd = NAV_CMD_WAYPOINT;
	mission_item.altitude = 10;
	mission_item.altitude_is_relative = true;
	checker.processNextItem(mission_item, 0, 3);

	mission_item.nav_cmd = NAV_CMD_LAND;
	mission_item.altitude = 0;
	double lat_new, lon_new;
	waypoint_from_heading_and_distance(mission_item.lat, mission_item.lon, 0, 200, &lat_new, &lon_new);
	mission_item.lat = lat_new;
	mission_item.lon = lon_new;
	checker.processNextItem(mission_item, 1, 3);

	mission_item.nav_cmd = NAV_CMD_DO_LAND_START;
	checker.processNextItem(mission_item, 2, 3);
	ASSERT_EQ(checker.someCheckFailed(), true);


	checker.reset();
	param_t param = param_handle(px4::params::MIS_TKO_LAND_REQ);
	// not takeoff land requiremntes, check should pass
	int param_val = 0;
	param_set(param, &param_val);
	checker.paramsChanged();
	checker.publishHomePosition(0, 0, 100);
	checker.publishVehicleType(vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	mission_item.nav_cmd = NAV_CMD_WAYPOINT;
	checker.processNextItem(mission_item, 0, 3);
	mission_item.nav_cmd = NAV_CMD_DO_LAND_START;
	checker.processNextItem(mission_item, 1, 3);
	mission_item.nav_cmd = NAV_CMD_LAND;
	checker.processNextItem(mission_item, 2, 3);
	ASSERT_EQ(checker.someCheckFailed(), false);
}

TEST_F(FeasibilityCheckerTest, check_takeoff_land_requirements)
{

	TestFeasibilityChecker checker;
	checker.publishLanded(true);
	checker.publishHomePosition(0, 0, 0);

	param_t param = param_handle(px4::params::MIS_TKO_LAND_REQ);

	// not takeoff land requiremntes, check should pass
	int param_val = 0;
	param_set(param, &param_val);
	checker.paramsChanged();

	mission_item_s mission_item = {};
	mission_item.nav_cmd = NAV_CMD_WAYPOINT;

	checker.processNextItem(mission_item, 0, 1);
	ASSERT_EQ(checker.someCheckFailed(), false);

	// require takeoff, check should fail
	param_val = 1;
	param_set(param, &param_val);
	checker.paramsChanged();
	checker.processNextItem(mission_item, 0, 1);
	ASSERT_EQ(checker.someCheckFailed(), true);

	// require landing, check should fail
	param_val = 2;
	param_set(param, &param_val);
	checker.paramsChanged();
	checker.processNextItem(mission_item, 0, 1);
	ASSERT_EQ(checker.someCheckFailed(), true);

	// require both takeoff and landing, check should fail
	param_val = 3;
	param_set(param, &param_val);
	checker.paramsChanged();
	checker.processNextItem(mission_item, 0, 1);
	ASSERT_EQ(checker.someCheckFailed(), true);

	// require takeoff and land or none, this should pass as we don't have either
	param_val = 4;
	param_set(param, &param_val);
	checker.paramsChanged();
	checker.processNextItem(mission_item, 0, 1);
	ASSERT_EQ(checker.someCheckFailed(), false);
}
