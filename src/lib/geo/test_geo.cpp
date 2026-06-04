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

TEST_F(GeoTest, get_distance_to_arc_outside_sector)
{
	// GIVEN: an arc centered at a mid-latitude (so the longitude cosine scaling
	// matters), sweeping from due-North of the center (start) to due-East (end).
	const double lat_center = 47.0;
	const double lon_center = 8.0;
	const float radius = 10000.f;            // [m]
	const float arc_start_bearing = 0.f;     // start point due North of the center
	const float arc_sweep = M_PI_F / 2.f;    // end point due East of the center

	// Endpoints placed with the same equirectangular approximation the implementation
	// uses: 1 deg latitude ~= 111111 m, 1 deg longitude ~= 111111 m * cos(latitude).
	const double meters_per_deg = 111111.0;
	const double radius_m = static_cast<double>(radius);
	const double cos_lat = cos(math::radians(lat_center));
	const double lat_start = lat_center + radius_m / meters_per_deg;
	const double lon_start = lon_center;
	const double lat_end = lat_center;
	const double lon_end = lon_center + radius_m / (meters_per_deg * cos_lat);

	crosstrack_error_s err{};

	// WHEN: the vehicle sits exactly on the arc start point (outside the sector)
	get_distance_to_arc(&err, lat_start, lon_start, lat_center, lon_center, radius, arc_start_bearing, arc_sweep);
	// THEN: the crosstrack distance is ~zero and we are not past the end
	EXPECT_NEAR(err.distance, 0.f, 0.1f);
	EXPECT_FALSE(err.past_end);

	// WHEN: the vehicle sits exactly on the arc end point (outside the sector)
	get_distance_to_arc(&err, lat_end, lon_end, lat_center, lon_center, radius, arc_start_bearing, arc_sweep);
	// THEN: the crosstrack distance is ~zero and we are past the end
	EXPECT_NEAR(err.distance, 0.f, 0.1f);
	EXPECT_TRUE(err.past_end);

	// WHEN: the vehicle is offset radially outward from the start point
	const double lat_offset = 500.0 / meters_per_deg;
	get_distance_to_arc(&err, lat_start + lat_offset, lon_start, lat_center, lon_center, radius, arc_start_bearing,
			    arc_sweep);
	// THEN: the crosstrack distance equals the distance to the (nearer) start point
	EXPECT_NEAR(err.distance, get_distance_to_next_waypoint(lat_start + lat_offset, lon_start, lat_start, lon_start), 0.1f);
	EXPECT_FALSE(err.past_end);
}
