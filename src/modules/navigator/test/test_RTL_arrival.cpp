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

#include <gtest/gtest.h>

#include "mission_block.h"
#include "navigator.h"
#include "rtl_arrival.h"
#include "support/navigator_dataman_test.h"

class RtlArrivalTest : public NavigatorDatamanTestBase
{
protected:
	void SetUp() override
	{
		config.approach.lat = 47.0;
		config.approach.lon = 8.0;
		config.approach.height_m = 530.f;
		config.approach.loiter_radius_m = -70.f;
		config.transit_altitude = 590.f;
		config.land_delay = 5.f;
		config.yaw = 1.f;
		vehicle.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	}

	Navigator navigator{};
	MissionBlock builder{&navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL};
	RtlArrival arrival{};
	RtlArrival::ItemConfig config{};
	mission_item_s item{};
	vehicle_status_s vehicle{};
};

TEST_F(RtlArrivalTest, MovePreservesTransitAltitudeAndUsesVehicleArrivalGeometry)
{
	arrival.startDirectReturn();
	arrival.buildMissionItem(builder, item, config, vehicle);
	EXPECT_EQ(item.nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_DOUBLE_EQ(item.lat, config.approach.lat);
	EXPECT_DOUBLE_EQ(item.lon, config.approach.lon);
	EXPECT_FLOAT_EQ(item.altitude, config.transit_altitude);
	EXPECT_FALSE(item.altitude_is_relative);
	EXPECT_FLOAT_EQ(item.yaw, config.yaw);
	EXPECT_FLOAT_EQ(item.acceptance_radius, navigator.get_acceptance_radius());

	vehicle.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	arrival.buildMissionItem(builder, item, config, vehicle);
	EXPECT_EQ(item.nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
	EXPECT_FLOAT_EQ(item.altitude, config.transit_altitude);
	EXPECT_FLOAT_EQ(item.loiter_radius, config.approach.loiter_radius_m);
	EXPECT_FLOAT_EQ(item.time_inside, 0.f);
	EXPECT_TRUE(item.autocontinue);
	EXPECT_FALSE(PX4_ISFINITE(item.yaw));
}

TEST_F(RtlArrivalTest, ArrivalDescendsThenHoldsBeforeHandingOverLanding)
{
	arrival.startDirectReturn();
	arrival.advance(vehicle, config.land_delay);
	arrival.buildMissionItem(builder, item, config, vehicle);
	EXPECT_EQ(item.nav_cmd, NAV_CMD_LOITER_TO_ALT);
	EXPECT_FLOAT_EQ(item.altitude, config.approach.height_m);
	EXPECT_FLOAT_EQ(item.yaw, config.yaw);

	arrival.advance(vehicle, config.land_delay);
	arrival.buildMissionItem(builder, item, config, vehicle);
	EXPECT_EQ(item.nav_cmd, NAV_CMD_LOITER_TIME_LIMIT);
	EXPECT_FLOAT_EQ(item.altitude, config.approach.height_m);
	EXPECT_FLOAT_EQ(item.time_inside, config.land_delay);
	EXPECT_TRUE(item.autocontinue);
	EXPECT_FALSE(PX4_ISFINITE(item.yaw));

	arrival.advance(vehicle, config.land_delay);
	EXPECT_EQ(arrival.phase(), RtlArrival::Phase::Land);
}

TEST_F(RtlArrivalTest, ImmediateLandingSkipsDescentAndHold)
{
	arrival.startDirectReturn();
	arrival.advance(vehicle, 0.f);
	EXPECT_EQ(arrival.phase(), RtlArrival::Phase::Land);

	// Final landing belongs to the caller, including the uploaded command and its settings.
	item.nav_cmd = NAV_CMD_VTOL_LAND;
	item.altitude = 500.f;
	item.land_precision = 2;
	arrival.buildMissionItem(builder, item, config, vehicle);
	EXPECT_EQ(item.nav_cmd, NAV_CMD_VTOL_LAND);
	EXPECT_FLOAT_EQ(item.altitude, 500.f);
	EXPECT_EQ(item.land_precision, 2);
}

TEST_F(RtlArrivalTest, DescentAndHoldNeverRaiseTheCapturedTransitAltitude)
{
	config.transit_altitude = config.approach.height_m - 10.f;
	arrival.startRouteArrival(true);
	vehicle.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	arrival.buildMissionItem(builder, item, config, vehicle);
	EXPECT_FLOAT_EQ(item.altitude, config.transit_altitude);
	EXPECT_FLOAT_EQ(item.loiter_radius, config.approach.loiter_radius_m);

	arrival.advance(vehicle, config.land_delay);
	vehicle.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	arrival.buildMissionItem(builder, item, config, vehicle);
	EXPECT_FLOAT_EQ(item.altitude, config.transit_altitude);
	EXPECT_FLOAT_EQ(item.loiter_radius, config.approach.loiter_radius_m);
}

TEST_F(RtlArrivalTest, NegativeDelayCreatesAnIndefiniteHold)
{
	config.land_delay = -1.f;
	arrival.startRouteArrival(true);
	arrival.advance(vehicle, config.land_delay);
	vehicle.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	arrival.buildMissionItem(builder, item, config, vehicle);
	EXPECT_EQ(item.nav_cmd, NAV_CMD_LOITER_UNLIMITED);
	EXPECT_FALSE(item.autocontinue);
	EXPECT_FLOAT_EQ(item.time_inside, 0.f);
}

TEST_F(RtlArrivalTest, BuildingArrivalItemsPreservesCallerOwnedFields)
{
	item.force_heading = true;
	item.land_precision = 2;
	arrival.startRouteArrival(true);
	arrival.buildMissionItem(builder, item, config, vehicle);
	EXPECT_TRUE(item.force_heading);
	EXPECT_EQ(item.land_precision, 2);
}

TEST_F(RtlArrivalTest, RouteApproachStartsAtDescentAndLeavesYawUnspecified)
{
	arrival.startRouteArrival(true);
	arrival.buildMissionItem(builder, item, config, vehicle);
	EXPECT_EQ(item.nav_cmd, NAV_CMD_LOITER_TO_ALT);
	EXPECT_FALSE(PX4_ISFINITE(item.yaw));

	arrival.startDirectReturn();
	arrival.buildMissionItem(builder, item, config, vehicle);
	EXPECT_EQ(item.nav_cmd, NAV_CMD_WAYPOINT);
	EXPECT_FLOAT_EQ(item.yaw, config.yaw);
}

TEST_F(RtlArrivalTest, RouteArrivalKeepsTransitioningVehiclesOutOfImmediateLanding)
{
	vehicle.is_vtol = true;
	vehicle.in_transition_mode = true;

	for (bool front_transition : {false, true}) {
		vehicle.in_transition_to_fw = front_transition;
		arrival.startRouteArrival(false);
		arrival.buildMissionItem(builder, item, config, vehicle);
		EXPECT_EQ(item.nav_cmd, front_transition ? NAV_CMD_LOITER_TIME_LIMIT : NAV_CMD_WAYPOINT);
		arrival.advance(vehicle, 0.f);
		EXPECT_EQ(arrival.phase(), RtlArrival::Phase::Descend);

		// Direct RTL retains its existing vehicle-type-only shortcut.
		arrival.startDirectReturn();
		arrival.buildMissionItem(builder, item, config, vehicle);
		EXPECT_EQ(item.nav_cmd, NAV_CMD_WAYPOINT);
		arrival.advance(vehicle, 0.f);
		EXPECT_EQ(arrival.phase(), RtlArrival::Phase::Land);
	}

	vehicle.in_transition_mode = false;
	vehicle.in_transition_to_fw = false;
	arrival.startRouteArrival(false);
	arrival.advance(vehicle, 0.f);
	EXPECT_EQ(arrival.phase(), RtlArrival::Phase::Land);
}

TEST_F(RtlArrivalTest, NewEntryResetsPreviousArrivalProgress)
{
	arrival.startRouteArrival(true);
	arrival.advance(vehicle, config.land_delay);
	EXPECT_EQ(arrival.phase(), RtlArrival::Phase::Hold);
	arrival.reset();
	EXPECT_EQ(arrival.phase(), RtlArrival::Phase::Land);
	arrival.startDirectReturn();
	EXPECT_EQ(arrival.phase(), RtlArrival::Phase::Move);
}
