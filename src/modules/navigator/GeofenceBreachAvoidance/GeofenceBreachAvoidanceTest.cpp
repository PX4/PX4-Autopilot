/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
#include "geofence_breach_avoidance.h"
#include "fake_geofence.hpp"

using namespace matrix;
using Vector2d = matrix::Vector2<double>;

TEST(GeofenceBreachAvoidanceTest, waypointFromBearingAndDistance)
{

	GeofenceBreachAvoidance gf_avoidance;
	struct map_projection_reference_s ref = {};
	Vector2d home_global(42.1, 8.2);
	map_projection_init(&ref, home_global(0), home_global(1));

	Vector2d waypoint_north_east_local(1.0, 1.0);
	waypoint_north_east_local = waypoint_north_east_local.normalized() * 10.5;
	Vector2d waypoint_north_east_global = gf_avoidance.waypointFromBearingAndDistance(home_global, M_PI_F * 0.25f, 10.5);

	float x, y;
	map_projection_project(&ref, waypoint_north_east_global(0), waypoint_north_east_global(1), &x, &y);
	Vector2d waypoint_north_east_reprojected(x, y);

	EXPECT_FLOAT_EQ(waypoint_north_east_local(0), waypoint_north_east_reprojected(0));
	EXPECT_FLOAT_EQ(waypoint_north_east_local(1), waypoint_north_east_reprojected(1));


	Vector2d waypoint_south_west_local = -waypoint_north_east_local;

	Vector2d waypoint_southwest_global = gf_avoidance.waypointFromBearingAndDistance(home_global, M_PI_F * 0.25f, -10.5);

	map_projection_project(&ref, waypoint_southwest_global(0), waypoint_southwest_global(1), &x, &y);
	Vector2d waypoint_south_west_reprojected(x, y);

	EXPECT_FLOAT_EQ(waypoint_south_west_local(0), waypoint_south_west_reprojected(0));
	EXPECT_FLOAT_EQ(waypoint_south_west_local(1), waypoint_south_west_reprojected(1));


	Vector2d same_as_home_global = gf_avoidance.waypointFromBearingAndDistance(home_global, M_PI_F * 0.25f, 0.0);

	EXPECT_EQ(home_global, same_as_home_global);
}

TEST(GeofenceBreachAvoidanceTest, generateLoiterPointForFixedWing)
{
	GeofenceBreachAvoidance gf_avoidance;
	FakeGeofence geo;
	struct map_projection_reference_s ref = {};
	Vector2d home_global(42.1, 8.2);
	map_projection_init(&ref, home_global(0), home_global(1));

	geofence_violation_type_u gf_violation;
	gf_violation.flags.fence_violation = true;

	gf_avoidance.setTestPointDistance(20.0f);
	gf_avoidance.setTestPointBearing(0.0f);
	gf_avoidance.setCurrentPosition(home_global(0), home_global(1), 0);


	Vector2d loiter_point_lat_lon = gf_avoidance.generateLoiterPointForFixedWing(gf_violation, &geo);

	// the expected loiter point is located test_point_distance behind
	Vector2d loiter_point_lat_lon_expected = gf_avoidance.waypointFromBearingAndDistance(home_global, 0.0f, -20.0f);

	EXPECT_FLOAT_EQ(loiter_point_lat_lon(0), loiter_point_lat_lon_expected(0));
	EXPECT_FLOAT_EQ(loiter_point_lat_lon(1), loiter_point_lat_lon_expected(1));


	geo.setProbeFunctionBehavior(FakeGeofence::LEFT_INSIDE_RIGHT_OUTSIDE);
	loiter_point_lat_lon = gf_avoidance.generateLoiterPointForFixedWing(gf_violation, &geo);

	loiter_point_lat_lon_expected = gf_avoidance.waypointFromBearingAndDistance(home_global, -M_PI_F * 0.5f, 20.0f);

	EXPECT_FLOAT_EQ(loiter_point_lat_lon(0), loiter_point_lat_lon_expected(0));
	EXPECT_FLOAT_EQ(loiter_point_lat_lon(1), loiter_point_lat_lon_expected(1));

	geo.setProbeFunctionBehavior(FakeGeofence::RIGHT_INSIDE_LEFT_OUTSIDE);
	loiter_point_lat_lon = gf_avoidance.generateLoiterPointForFixedWing(gf_violation, &geo);

	loiter_point_lat_lon_expected = gf_avoidance.waypointFromBearingAndDistance(home_global, M_PI_F * 0.5f, 20.0f);

	EXPECT_FLOAT_EQ(loiter_point_lat_lon(0), loiter_point_lat_lon_expected(0));
	EXPECT_FLOAT_EQ(loiter_point_lat_lon(1), loiter_point_lat_lon_expected(1));

	gf_violation.flags.fence_violation = false;
	loiter_point_lat_lon = gf_avoidance.generateLoiterPointForFixedWing(gf_violation, &geo);

	EXPECT_FLOAT_EQ(loiter_point_lat_lon(0), home_global(0));
	EXPECT_FLOAT_EQ(loiter_point_lat_lon(1), home_global(1));
}

TEST(GeofenceBreachAvoidanceTest, generateLoiterPointForMultirotor)
{
	GeofenceBreachAvoidance gf_avoidance;
	FakeGeofence geo;
	struct map_projection_reference_s ref = {};
	Vector2d home_global(42.1, 8.2);
	map_projection_init(&ref, home_global(0), home_global(1));

	param_t param = param_handle(px4::params::MPC_ACC_HOR);

	float value = 3;
	param_set(param, &value);

	param = param_handle(px4::params::MPC_ACC_HOR_MAX);
	value = 5;
	param_set(param, &value);

	param = param_handle(px4::params::MPC_JERK_AUTO);
	value = 8;
	param_set(param, &value);

	geofence_violation_type_u gf_violation;
	gf_violation.flags.fence_violation = true;

	gf_avoidance.setTestPointDistance(30.0f);
	gf_avoidance.setTestPointBearing(0.0f);
	gf_avoidance.setCurrentPosition(home_global(0), home_global(1), 0);
	// vehicle is approaching the fence at a crazy velocity
	gf_avoidance.setHorizontalVelocity(1000.0f);
	gf_avoidance.computeBrakingDistanceMultirotor();

	geo.setProbeFunctionBehavior(FakeGeofence::GF_BOUNDARY_20M_AHEAD);

	Vector2d loiter_point = gf_avoidance.generateLoiterPointForMultirotor(gf_violation, &geo);

	Vector2d loiter_point_predicted = gf_avoidance.waypointFromBearingAndDistance(home_global, 0.0f,
					  20.0f - gf_avoidance.getMinDistToFenceMulticopter());

	float error = get_distance_to_next_waypoint(loiter_point(0), loiter_point(1), loiter_point_predicted(0),
			loiter_point_predicted(1));

	EXPECT_LE(error, 0.5f);

	// vehicle is approaching fenc slowly, plenty of time to brake
	gf_avoidance.setHorizontalVelocity(0.1f);
	gf_avoidance.computeBrakingDistanceMultirotor();

	loiter_point = gf_avoidance.generateLoiterPointForMultirotor(gf_violation, &geo);
	loiter_point_predicted = gf_avoidance.waypointFromBearingAndDistance(home_global, 0.0f,
				 gf_avoidance.computeBrakingDistanceMultirotor());


	error = get_distance_to_next_waypoint(loiter_point(0), loiter_point(1), loiter_point_predicted(0),
					      loiter_point_predicted(1));

	EXPECT_LE(error, 0.0f);

	gf_violation.flags.fence_violation = false;
	loiter_point = gf_avoidance.generateLoiterPointForMultirotor(gf_violation, &geo);

	EXPECT_EQ(loiter_point, home_global);
}

TEST(GeofenceBreachAvoidanceTest, generateLoiterAltitudeForFixedWing)
{
	GeofenceBreachAvoidance gf_avoidance;
	const float current_alt_amsl = 100.0f;
	const float vertical_test_point_dist = 10.0f;

	gf_avoidance.setVerticalTestPointDistance(vertical_test_point_dist);
	gf_avoidance.setCurrentPosition(0, 0, current_alt_amsl); // just care about altitude
	geofence_violation_type_u gf_violation;
	gf_violation.flags.max_altitude_exceeded = true;

	float loiter_alt = gf_avoidance.generateLoiterAltitudeForFixedWing(gf_violation);

	EXPECT_EQ(loiter_alt, current_alt_amsl - 2 * vertical_test_point_dist);

	gf_violation.flags.max_altitude_exceeded = false;

	loiter_alt = gf_avoidance.generateLoiterAltitudeForFixedWing(gf_violation);

	EXPECT_EQ(loiter_alt, current_alt_amsl);
}

TEST(GeofenceBreachAvoidanceTest, generateLoiterAltitudeForMulticopter)
{
	GeofenceBreachAvoidance gf_avoidance;
	const float climbrate = 10.0f;
	const float current_alt_amsl = 100.0f;
	geofence_violation_type_u gf_violation;
	gf_violation.flags.max_altitude_exceeded = true;


	gf_avoidance.setClimbRate(climbrate);
	gf_avoidance.setCurrentPosition(0, 0, current_alt_amsl);
	gf_avoidance.computeVerticalBrakingDistanceMultirotor();

	float loiter_alt_amsl = gf_avoidance.generateLoiterAltitudeForMulticopter(gf_violation);

	EXPECT_EQ(loiter_alt_amsl, current_alt_amsl + gf_avoidance.computeVerticalBrakingDistanceMultirotor() -
		  gf_avoidance.getMinVertDistToFenceMultirotor());

	gf_violation.flags.max_altitude_exceeded = false;

	loiter_alt_amsl = gf_avoidance.generateLoiterAltitudeForMulticopter(gf_violation);

	EXPECT_EQ(loiter_alt_amsl, current_alt_amsl);
}
