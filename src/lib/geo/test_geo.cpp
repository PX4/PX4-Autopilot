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

TEST_F(GeoTest, getDistanceToArcOutsideSector)
{
	// GIVEN: an arc at a mid-latitude (so the longitude cosine scaling matters), sweeping from
	// due-North of the center (start) to due-East (end).
	const double lat_center = 47.0;
	const double lon_center = 8.0;
	const float radius = 10000.f;            // [m]
	const float arc_start_bearing = 0.f;     // start point due North of the center
	const float arc_sweep = M_PI_F / 2.f;    // end point due East of the center
	const double mpd = 111111.0;
	const double cos_lat = cos(math::radians(lat_center));

	// The implementation locates the endpoints with an equirectangular approximation; mirror it so we
	// can predict which endpoint is nearer and at what distance.
	auto endpoint = [&](double bearing, double & lat, double & lon) {
		lat = lat_center + static_cast<double>(radius) * cos(bearing) / mpd;
		lon = lon_center + static_cast<double>(radius) * sin(bearing) / (mpd * cos_lat);
	};
	double lat_start, lon_start, lat_end, lon_end;
	endpoint(static_cast<double>(arc_start_bearing), lat_start, lon_start);
	endpoint(static_cast<double>(arc_start_bearing + arc_sweep), lat_end, lon_end);

	crosstrack_error_s err{};
	double lat_v, lon_v;

	// WHEN: the vehicle is outside the wedge on the start side (0.5 rad before the start bearing)
	waypoint_from_heading_and_distance(lat_center, lon_center, arc_start_bearing - 0.5f, radius, &lat_v, &lon_v);
	get_distance_to_arc(&err, lat_v, lon_v, lat_center, lon_center, radius, arc_start_bearing, arc_sweep);
	// THEN: the result is the distance to the nearer (start) endpoint, not past the end
	EXPECT_LT(get_distance_to_next_waypoint(lat_v, lon_v, lat_start, lon_start),
		  get_distance_to_next_waypoint(lat_v, lon_v, lat_end, lon_end));
	EXPECT_NEAR(err.distance, get_distance_to_next_waypoint(lat_v, lon_v, lat_start, lon_start), 0.5f);
	EXPECT_FALSE(err.past_end);

	// WHEN: the vehicle is outside the wedge on the end side (0.5 rad past the end bearing)
	waypoint_from_heading_and_distance(lat_center, lon_center, arc_start_bearing + arc_sweep + 0.5f, radius, &lat_v, &lon_v);
	get_distance_to_arc(&err, lat_v, lon_v, lat_center, lon_center, radius, arc_start_bearing, arc_sweep);
	// THEN: the result is the distance to the nearer (end) endpoint, past the end
	EXPECT_LT(get_distance_to_next_waypoint(lat_v, lon_v, lat_end, lon_end),
		  get_distance_to_next_waypoint(lat_v, lon_v, lat_start, lon_start));
	EXPECT_NEAR(err.distance, get_distance_to_next_waypoint(lat_v, lon_v, lat_end, lon_end), 0.5f);
	EXPECT_TRUE(err.past_end);
}

TEST_F(GeoTest, getDistanceToArcInsideSector)
{
	// GIVEN: an arc sweeping from due-North (start) to due-East (end) of the center
	const double lat_center = 47.0;
	const double lon_center = 8.0;
	const float radius = 10000.f;
	const float arc_start_bearing = 0.f;
	const float arc_sweep = M_PI_F / 2.f;
	const float mid_bearing = arc_start_bearing + arc_sweep / 2.f;   // pi/4, inside the wedge

	crosstrack_error_s err{};
	double lat_v, lon_v;

	// WHEN: the vehicle is exactly on the arc (on the circle, inside the wedge). This is the case the
	// previous in-sector logic mis-classified and reported kilometers off.
	waypoint_from_heading_and_distance(lat_center, lon_center, mid_bearing, radius, &lat_v, &lon_v);
	get_distance_to_arc(&err, lat_v, lon_v, lat_center, lon_center, radius, arc_start_bearing, arc_sweep);
	// THEN: distance to the arc is ~zero
	EXPECT_NEAR(err.distance, 0.f, 1.f);
	EXPECT_FALSE(err.past_end);

	// WHEN: the vehicle is inside the circle, within the wedge (half a radius from the center)
	waypoint_from_heading_and_distance(lat_center, lon_center, mid_bearing, radius / 2.f, &lat_v, &lon_v);
	get_distance_to_arc(&err, lat_v, lon_v, lat_center, lon_center, radius, arc_start_bearing, arc_sweep);
	// THEN: the distance is the radial gap to the arc (~radius/2)
	EXPECT_NEAR(err.distance, radius / 2.f, 1.f);
	EXPECT_FALSE(err.past_end);

	// WHEN: the vehicle is outside the circle, within the wedge (1.5 radius from the center)
	waypoint_from_heading_and_distance(lat_center, lon_center, mid_bearing, 1.5f * radius, &lat_v, &lon_v);
	get_distance_to_arc(&err, lat_v, lon_v, lat_center, lon_center, radius, arc_start_bearing, arc_sweep);
	// THEN: the distance is again the radial gap (~radius/2)
	EXPECT_NEAR(err.distance, radius / 2.f, 1.f);
	EXPECT_FALSE(err.past_end);
}

TEST_F(GeoTest, getDistanceToArcNegativeSweep)
{
	// GIVEN: the same NE-quadrant arc, but defined from due-East (start) sweeping negatively to North (end)
	const double lat_center = 47.0;
	const double lon_center = 8.0;
	const float radius = 10000.f;
	const float arc_start_bearing = M_PI_F / 2.f;
	const float arc_sweep = -M_PI_F / 2.f;

	crosstrack_error_s err{};
	double lat_v, lon_v;

	// WHEN: the vehicle is on the arc midpoint (bearing pi/4 from the center) -> in sector
	waypoint_from_heading_and_distance(lat_center, lon_center, M_PI_F / 4.f, radius, &lat_v, &lon_v);
	get_distance_to_arc(&err, lat_v, lon_v, lat_center, lon_center, radius, arc_start_bearing, arc_sweep);
	// THEN: distance to the arc is ~zero
	EXPECT_NEAR(err.distance, 0.f, 1.f);
	EXPECT_FALSE(err.past_end);

	// WHEN: the vehicle is due South of the center, far outside the wedge -> out of sector
	waypoint_from_heading_and_distance(lat_center, lon_center, M_PI_F, radius, &lat_v, &lon_v);
	get_distance_to_arc(&err, lat_v, lon_v, lat_center, lon_center, radius, arc_start_bearing, arc_sweep);
	// THEN: it is NOT treated as on the arc (the old negative-sweep sector math returned ~zero here)
	EXPECT_GT(err.distance, radius);
}


TEST_F(GeoTest, mavlinkLocalPointDistance3d)
{
	float dist_xy = 0.f;
	float dist_z = 0.f;
	const float d = mavlink_wpm_distance_to_point_local(0.f, 0.f, 0.f, 3.f, 4.f, 12.f, &dist_xy, &dist_z);
	EXPECT_NEAR(d, 13.f, 1e-4f);
	EXPECT_NEAR(dist_xy, 5.f, 1e-4f);
	EXPECT_NEAR(dist_z, 12.f, 1e-4f);

	const float d0 = mavlink_wpm_distance_to_point_local(1.f, 2.f, 3.f, 1.f, 2.f, 3.f, &dist_xy, &dist_z);
	EXPECT_NEAR(d0, 0.f, 1e-6f);
	EXPECT_NEAR(dist_xy, 0.f, 1e-6f);
	EXPECT_NEAR(dist_z, 0.f, 1e-6f);
}

TEST_F(GeoTest, globalWgs84DistanceHorizontal)
{
	const double lat0 = 47.0;
	const double lon0 = 8.0;
	double lat1, lon1;
	waypoint_from_heading_and_distance(lat0, lon0, 0.f, 200.f, &lat1, &lon1);
	float dist_xy = 0.f;
	float dist_z = 0.f;
	float dist = get_distance_to_point_global_wgs84(lat0, lon0, 100.f, lat1, lon1, 100.f, &dist_xy, &dist_z);
	EXPECT_NEAR(dist_xy, 200.f, 2.0f);
	EXPECT_NEAR(dist_z, 0.f, 0.1f);
	EXPECT_NEAR(dist, 200.f, 2.0f);
}
