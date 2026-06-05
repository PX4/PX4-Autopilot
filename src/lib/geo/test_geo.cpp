/****************************************************************************
 *
 *   Copyright (c) 2020 ECL Development Team. All rights reserved.
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
#include <math.h>
#include <mathlib/mathlib.h>
#include <memory>
#include <lib/geo/geo.h>

class GeoTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		proj.initReference(473566094 / 1e7, 85190237 / 1e7, 0);
	}

protected:
	MapProjection proj;

};


TEST_F(GeoTest, reprojectProject)
{
	// GIVEN: x and y coordinates in the local cartesian frame
	float x = 0.5;
	float y = 1;
	double lat;
	double lon;
	proj.reproject(x, y, lat, lon);
	float x_new;
	float y_new;
	proj.project(lat, lon, x_new, y_new);
	double lat_new;
	double lon_new;
	proj.reproject(x_new, y_new, lat_new, lon_new);
	EXPECT_FLOAT_EQ(x, x_new);
	EXPECT_FLOAT_EQ(y, y_new);
	EXPECT_FLOAT_EQ(lat, lat_new);
	EXPECT_FLOAT_EQ(lon, lon_new);
}

TEST_F(GeoTest, projectReproject)
{
	// GIVEN: lat and lon coordinates in the geographic coordinate system
	double lat = 47.356616973876953;
	double lon = 8.5190505981445313;
	float x;
	float y;
	proj.project(lat, lon, x, y);
	double lat_new;
	double lon_new;
	proj.reproject(x, y, lat_new, lon_new);
	float x_new;
	float y_new;
	proj.project(lat_new, lon_new, x_new, y_new);
	// WHEN: apply the mapping and its inverse the output should stay the same
	EXPECT_FLOAT_EQ(x, x_new);
	EXPECT_FLOAT_EQ(y, y_new);
	EXPECT_FLOAT_EQ(lat, lat_new);
	EXPECT_FLOAT_EQ(lon, lon_new);
}

TEST_F(GeoTest, ReprojectNanPrevention)
{
	// GIVEN: A reference point very close to the North Pole using the fixture's 'proj' object
	proj.initReference(89.9999, 0.0, 0ULL);

	// WHEN: We reproject a tiny offset that pushes the math right against the mathematical boundary
	double lat;
	double lon;
	proj.reproject(10.0f, 10.0f, lat, lon);

	// THEN: It should return a valid constrained number, not a NaN
	EXPECT_FALSE(std::isnan(lat));
	EXPECT_FALSE(std::isnan(lon));
}

TEST_F(GeoTest, VectorToNextWaypointFastAntimeridian)
{
	// GIVEN: A starting point at +179 (East) and a target at -179 (West)
	double lat_now = 0.0;
	double lon_now = 179.0;
	double lat_next = 0.0;
	double lon_next = -179.0;

	float v_n = 0.0f;
	float v_e = 0.0f;

	// WHEN: We calculate the vector
	get_vector_to_next_waypoint_fast(lat_now, lon_now, lat_next, lon_next, &v_n, &v_e);

	// THEN: Because the shortest path is 2 degrees East, the East velocity (v_e)
	// MUST be a positive number. Without wrapping, this would be highly negative.
	EXPECT_FLOAT_EQ(v_n, 0.0f); // Latitude didn't change
	EXPECT_GT(v_e, 0.0f);       // Eastward velocity should be Greater Than (GT) zero
}

TEST_F(GeoTest, DistanceToLineBeforeStart)
{
	struct crosstrack_error_s ct_error;

	// GIVEN: A line from (0,0) to (0,10). The drone is at (0,-5), which is BEHIND the start line.
	get_distance_to_line(ct_error, 0.0, -5.0, 0.0, 0.0, 0.0, 10.0);

	// THEN: It should flag past_start as true, and past_end as false.
	EXPECT_TRUE(ct_error.past_start);
	EXPECT_FALSE(ct_error.past_end);
}

TEST_F(GeoTest, DistanceToLinePastEnd)
{
	struct crosstrack_error_s ct_error;

	// GIVEN: A line from (0,0) to (0,10). The drone is at (0,15), which is BEYOND the end line.
	get_distance_to_line(ct_error, 0.0, 15.0, 0.0, 0.0, 0.0, 10.0);

	// THEN: It should flag past_end as true, and past_start as false.
	EXPECT_TRUE(ct_error.past_end);
	EXPECT_FALSE(ct_error.past_start);
}

TEST_F(GeoTest, WaypointDistanceAndBearing)
{
	// GIVEN: Starting at the equator (0,0) and moving exactly 1 degree North.
	double lat_now = 0.0;
	double lon_now = 0.0;
	double lat_next = 1.0;
	double lon_next = 0.0;

	// 1 degree of latitude converted to radians, multiplied by the radius.
	float expected_dist = static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * (M_PI / 180.0));

	// THEN: The calculated distance should match our expected math (within 1 meter).
	float dist = get_distance_to_next_waypoint(lat_now, lon_now, lat_next, lon_next);
	EXPECT_NEAR(dist, expected_dist, 1.0f);

	// THEN: The bearing for moving straight North must be exactly 0.0 radians.
	float bearing = get_bearing_to_next_waypoint(lat_now, lon_now, lat_next, lon_next);
	EXPECT_FLOAT_EQ(bearing, 0.0f);
}

TEST_F(GeoTest, AddVectorToGlobal)
{
	// GIVEN: Starting at the equator and moving 1000 meters straight North.
	double lat_now = 0.0;
	double lon_now = 0.0;
	float v_n = 1000.0f; // 1000m North
	float v_e = 0.0f;    // 0m East
	double lat_res = 0.0;
	double lon_res = 0.0;

	// WHEN: We calculate the new global position
	add_vector_to_global_position(lat_now, lon_now, v_n, v_e, &lat_res, &lon_res);

	// THEN: Latitude must be a positive number, and longitude must remain exactly 0.
	EXPECT_GT(lat_res, 0.0);
	EXPECT_DOUBLE_EQ(lon_res, 0.0);
}

TEST_F(GeoTest, VectorToNextWaypointStandard)
{
	// 1. STANDARD VECTOR: Moving 1 degree North from the equator
	float v_n = 0.0f;
	float v_e = 0.0f;
	get_vector_to_next_waypoint(0.0, 0.0, 1.0, 0.0, &v_n, &v_e);

	float expected_dist = static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * (M_PI / 180.0));
	EXPECT_NEAR(v_n, expected_dist, 10.0f); // North velocity should match Earth's arc
	EXPECT_FLOAT_EQ(v_e, 0.0f);            // East velocity should be perfectly zero
}

TEST_F(GeoTest, DistanceToPointGlobalSphericalAbsolute)
{
	// 2. SPHERICAL DISTANCE: The WGS84 haversine rename
	float dist_xy = 0.0f;
	float dist_z = 0.0f;

	// Moving 1 degree North, maintaining 100m altitude
	float total_dist = get_distance_to_point_global_spherical(0.0, 0.0, 100.0f, 1.0, 0.0, 100.0f, &dist_xy, &dist_z);

	float expected_xy = static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * (M_PI / 180.0));
	EXPECT_NEAR(dist_xy, expected_xy, 1.0f);
	EXPECT_FLOAT_EQ(dist_z, 0.0f);           // Altitude didn't change
	EXPECT_NEAR(total_dist, expected_xy, 1.0f);
}

TEST_F(GeoTest, MavlinkLocalDistance)
{
	// 3. MAVLINK LOCAL: A standard 3D Pythagorean theorem test
	float dist_xy = 0.0f;
	float dist_z = 0.0f;

	// A 3-4-12 triangle in 3D space: sqrt(3^2 + 4^2 + 12^2) = 13
	// We pass the memory addresses of dist_xy and dist_z so the function can write to them
	float dist = mavlink_wpm_distance_to_point_local(0.0f, 0.0f, 0.0f, 3.0f, 4.0f, 12.0f, &dist_xy, &dist_z);

	EXPECT_FLOAT_EQ(dist, 13.0f);
	EXPECT_FLOAT_EQ(dist_xy, 5.0f); // 3-4-5 triangle for the XY plane
	EXPECT_FLOAT_EQ(dist_z, 12.0f); // Vertical distance is exactly 12
}

TEST_F(GeoTest, MapProjectionAbsolute)
{
	// 4. MAP PROJECTION: Verifying absolute output, not just round-trip invertibility
	MapProjection ref{};
	ref.initReference(0.0, 0.0); // Set reference to the equator

	float x = 0.0f;
	float y = 0.0f;
	ref.project(1.0, 0.0, x, y); // Project 1 degree North

	// 1 degree North on the WGS84 ellipsoid is approximately 110,574 meters.
	EXPECT_NEAR(x, 111195.0f, 2.0f); // Assert we get the correct absolute Cartesian distance
	EXPECT_FLOAT_EQ(y, 0.0f);
}

TEST_F(GeoTest, waypoint_from_heading_and_zero_distance)
{
	// GIVEN: a starting waypoint, a heading and a distance of 0
	double lat_start = -33;
	double lon_start = 18;
	float bearing = 0;
	float dist = 0;

	double lat_target = 0;
	double lon_target = 0;

	// WHEN: we get the next waypoint
	waypoint_from_heading_and_distance(lat_start, lon_start, bearing, dist, &lat_target, &lon_target);

	// THEN: it should be the same
	EXPECT_DOUBLE_EQ(lat_start, lat_target);
	EXPECT_DOUBLE_EQ(lon_start, lon_target);
}


TEST_F(GeoTest, waypoint_from_heading_and_negative_distance)
{
	// GIVEN: a starting waypoint, a heading and negative distance
	double lat_start = -33;
	double lon_start = 18;
	float bearing = 0;
	float lat_offset = -0.01f;
	float dist = CONSTANTS_RADIUS_OF_EARTH_F * M_PI_F / 180.f * lat_offset;

	double lat_target = 0;
	double lon_target = 0;

	// WHEN: we get the next waypoint
	waypoint_from_heading_and_distance(lat_start, lon_start, bearing, dist, &lat_target, &lon_target);

	// THEN: it should be the same
	EXPECT_FLOAT_EQ(static_cast<float>(lat_start) + lat_offset, lat_target);
	EXPECT_DOUBLE_EQ(lon_start, lon_target);
}

TEST_F(GeoTest, waypoint_from_heading_and_positive_distance)
{
	// GIVEN: a starting waypoint, a heading and positive distance
	double lat_start = -33;
	double lon_start = 18;
	float bearing = 0;
	float lat_offset = 0.01f;
	float dist = CONSTANTS_RADIUS_OF_EARTH_F * M_PI_F / 180.f * lat_offset;

	double lat_target = 0;
	double lon_target = 0;

	// WHEN: we get the next waypoint
	waypoint_from_heading_and_distance(lat_start, lon_start, bearing, dist, &lat_target, &lon_target);

	// THEN: it should be the same
	EXPECT_FLOAT_EQ(static_cast<float>(lat_start) + lat_offset, lat_target);
	EXPECT_DOUBLE_EQ(lon_start, lon_target);
}

TEST_F(GeoTest, waypoint_from_line_and_zero_distance)
{
	// GIVEN: a starting waypoint, a heading and a distance of 0
	double lat_start = -33;
	double lon_start = 18;
	double lat_end = -32;
	double lon_end = 18;
	float dist = 0;

	double lat_target = 0;
	double lon_target = 0;

	// WHEN: we get the next waypoint
	create_waypoint_from_line_and_dist(lat_start, lon_start, lat_end, lon_end, dist, &lat_target, &lon_target);

	// THEN: it should be the same
	EXPECT_DOUBLE_EQ(lat_start, lat_target);
	EXPECT_DOUBLE_EQ(lon_start, lon_target);
}

TEST_F(GeoTest, waypoint_from_line_and_positive_distance)
{
	// GIVEN: a starting waypoint, a heading and a positive distance
	double lat_start = -33;
	double lon_start = 18;
	double lat_offset = 0.01;
	double lat_end = lat_start + lat_offset;
	double lon_end = 18;
	float dist = CONSTANTS_RADIUS_OF_EARTH_F * M_PI_F / 180.f * static_cast<float>(lat_offset);

	double lat_target = 0;
	double lon_target = 0;

	// WHEN: we get the next waypoint
	create_waypoint_from_line_and_dist(lat_start, lon_start, lat_end, lon_end, dist, &lat_target, &lon_target);

	// THEN: it should be the same
	EXPECT_FLOAT_EQ(lat_start + lat_offset, lat_target);
	EXPECT_DOUBLE_EQ(lon_start, lon_target);
}


TEST_F(GeoTest, waypoint_from_line_and_negative_distance)
{
	// GIVEN: a starting waypoint, a heading and a negative distance
	double lat_start = -33;
	double lon_start = 18;
	double lat_offset = 0.01;
	double lat_end = lat_start + lat_offset;
	double lon_end = 18;
	float dist = -CONSTANTS_RADIUS_OF_EARTH_F * M_PI_F / 180.f * static_cast<float>(lat_offset);

	double lat_target = 0;
	double lon_target = 0;

	// WHEN: we get the next waypoint
	create_waypoint_from_line_and_dist(lat_start, lon_start, lat_end, lon_end, dist, &lat_target, &lon_target);

	// THEN: it should be the same
	EXPECT_FLOAT_EQ(lat_start - lat_offset, lat_target);
	EXPECT_DOUBLE_EQ(lon_start, lon_target);
}
