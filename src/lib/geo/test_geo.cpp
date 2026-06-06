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

static constexpr float one_degree_m = CONSTANTS_RADIUS_OF_EARTH_F * M_PI_F / 180.f;


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

TEST_F(GeoTest, mapProjectionAbsoluteValues)
{
	MapProjection local_proj{0., 0.};

	float x = NAN;
	float y = NAN;
	local_proj.project(1., 0., x, y);
	EXPECT_NEAR(x, one_degree_m, 0.01f);
	EXPECT_NEAR(y, 0.f, 0.01f);

	local_proj.project(0., 1., x, y);
	EXPECT_NEAR(x, 0.f, 0.01f);
	EXPECT_NEAR(y, one_degree_m, 0.01f);

	double lat = NAN;
	double lon = NAN;
	local_proj.reproject(one_degree_m, 0.f, lat, lon);
	EXPECT_NEAR(lat, 1., 1e-6);
	EXPECT_NEAR(lon, 0., 1e-6);

	local_proj.reproject(0.f, one_degree_m, lat, lon);
	EXPECT_NEAR(lat, 0., 1e-6);
	EXPECT_NEAR(lon, 1., 1e-6);
}

TEST_F(GeoTest, distanceAndBearingToNextWaypoint)
{
	EXPECT_NEAR(get_distance_to_next_waypoint(0., 0., 1., 0.), one_degree_m, 0.01f);
	EXPECT_NEAR(get_distance_to_next_waypoint(0., 0., 0., 1.), one_degree_m, 0.01f);

	EXPECT_NEAR(get_bearing_to_next_waypoint(0., 0., 1., 0.), 0.f, 1e-6f);
	EXPECT_NEAR(get_bearing_to_next_waypoint(0., 0., 0., 1.), M_PI_2_F, 1e-6f);
}

TEST_F(GeoTest, vectorToNextWaypoint)
{
	float v_n = NAN;
	float v_e = NAN;
	get_vector_to_next_waypoint(0., 0., 1., 0., &v_n, &v_e);
	EXPECT_NEAR(v_n, CONSTANTS_RADIUS_OF_EARTH_F * sinf(M_PI_F / 180.f), 0.01f);
	EXPECT_NEAR(v_e, 0.f, 0.01f);

	get_vector_to_next_waypoint(0., 0., 0., 1., &v_n, &v_e);
	EXPECT_NEAR(v_n, 0.f, 0.01f);
	EXPECT_NEAR(v_e, CONSTANTS_RADIUS_OF_EARTH_F * sinf(M_PI_F / 180.f), 0.01f);
}

TEST_F(GeoTest, fastVectorToNextWaypointWrapsAntimeridian)
{
	float v_n = NAN;
	float v_e = NAN;
	get_vector_to_next_waypoint_fast(0., 179.9, 0., -179.9, &v_n, &v_e);

	EXPECT_NEAR(v_n, 0.f, 0.01f);
	EXPECT_NEAR(v_e, CONSTANTS_RADIUS_OF_EARTH_F * math::radians(0.2f), 0.1f);
}

TEST_F(GeoTest, addVectorToGlobalPositionWrapsAntimeridian)
{
	double lat = NAN;
	double lon = NAN;
	add_vector_to_global_position(0., 179.9, 0.f, CONSTANTS_RADIUS_OF_EARTH_F * math::radians(0.2f), &lat, &lon);

	EXPECT_NEAR(lat, 0., 1e-6);
	EXPECT_NEAR(lon, -179.9, 1e-6);
}

TEST_F(GeoTest, distanceToLine)
{
	crosstrack_error_s crosstrack_error{};
	EXPECT_EQ(get_distance_to_line(crosstrack_error, 0.1, 0.5, 0., 0., 0., 1.), 0);
	EXPECT_FALSE(crosstrack_error.past_end);
	EXPECT_NEAR(fabsf(crosstrack_error.distance), one_degree_m / 10.f, 1.f);

	EXPECT_EQ(get_distance_to_line(crosstrack_error, 0., 1.1, 0., 0., 0., 1.), 0);
	EXPECT_TRUE(crosstrack_error.past_end);
}

TEST_F(GeoTest, distanceToPointGlobalWgs84)
{
	float dist_xy = NAN;
	float dist_z = NAN;
	const float distance = get_distance_to_point_global_wgs84(0., 0., 100.f, 0., 1., 70.f, &dist_xy, &dist_z);

	EXPECT_NEAR(dist_xy, one_degree_m, 0.01f);
	EXPECT_NEAR(dist_z, 30.f, 1e-6f);
	EXPECT_NEAR(distance, sqrtf(one_degree_m * one_degree_m + 30.f * 30.f), 0.01f);
}

TEST_F(GeoTest, mavlinkWpmDistanceToPointLocal)
{
	float dist_xy = NAN;
	float dist_z = NAN;
	const float distance = mavlink_wpm_distance_to_point_local(3.f, 4.f, 12.f, 0.f, 0.f, 0.f, &dist_xy, &dist_z);

	EXPECT_FLOAT_EQ(dist_xy, 5.f);
	EXPECT_FLOAT_EQ(dist_z, 12.f);
	EXPECT_FLOAT_EQ(distance, 13.f);
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
