/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
#include "geofence_avoidance_planner.h"
#include <lib/geofence/geofence_utils.h>
#include <lib/geo/geo.h>
#include <navigator/navigation.h>

class FakeGeofence : public GeofenceInterface
{
public:
	FakeGeofence(const matrix::Vector2<double> *vertices, int num_vertices, uint16_t fence_type)
		: _vertices(vertices), _num_vertices(num_vertices), _fence_type(fence_type) {}

	bool checkIfLineViolatesAnyFence(const matrix::Vector2f &start_local,
					 const matrix::Vector2f &end_local,
					 const matrix::Vector2<double> &reference) override
	{
		MapProjection ref{reference(0), reference(1)};

		matrix::Vector2f vertices_local[_num_vertices];

		for (int i = 0; i < _num_vertices; i++) {
			vertices_local[i] = ref.project(_vertices[i](0), _vertices[i](1));
		}

		const bool is_inclusion_zone = (_fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);
		return geofence_utils::lineSegmentIntersectsPolygon(start_local, end_local,
				vertices_local, _num_vertices, is_inclusion_zone);
	}

	PolygonInfo getPolygonInfoByIndex(int index) override
	{
		PolygonInfo info{};
		info.fence_type = _fence_type;
		info.vertex_count = _num_vertices;
		return info;
	}

	matrix::Vector2<double> getPolygonVertexByIndex(int poly_idx, int vertex_idx) override
	{
		return _vertices[vertex_idx];
	}

	int getNumPolygons() const override { return 1; }

private:
	const matrix::Vector2<double> *_vertices;
	int _num_vertices;
	uint16_t _fence_type;
};

class GeofenceAvoidancePlannerTest : public ::testing::Test
{
public:
	GeofenceAvoidancePlanner _planner;
};

TEST_F(GeofenceAvoidancePlannerTest, StartEqualsDestination)
{
	using namespace matrix;
	Vector2<double> point(47.3977, 8.5456);

	FakeGeofence fake(nullptr, 0, NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);
	_planner.update_vertices(fake, 0.f);
	_planner.update_destination(point, fake);

	const int num_waypoints = _planner.set_start_and_plan_path_to_destination(point, fake);

	ASSERT_EQ(num_waypoints, 0);
}

TEST_F(GeofenceAvoidancePlannerTest, DirectPathNoFence)
{
	using namespace matrix;
	Vector2<double> start(47.3977, 8.5456);
	Vector2<double> destination(47.3984, 8.5470);

	FakeGeofence fake(nullptr, 0, NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);
	_planner.update_vertices(fake, 0.f);
	_planner.update_destination(destination, fake);

	const int num_waypoints = _planner.set_start_and_plan_path_to_destination(start, fake);

	ASSERT_EQ(num_waypoints, 0);
}

TEST_F(GeofenceAvoidancePlannerTest, PathAroundExclusionZone)
{
	// in this test there is an exclusion zone between start and destination. The shortest path
	// is chosen to be via vertex nr 1 and vertex nr 2, so the path should have two waypoints
	using namespace matrix;
	Vector2<double> start(47.3559582, 8.5192064);
	Vector2<double> destination(47.3546153, 8.5193195);

	static const Vector2<double> vertices[] = {
		{47.3552420, 8.5192293},
		{47.3555843, 8.5201174},
		{47.3551382, 8.5209143},
		{47.3550828, 8.5171901},
	};

	FakeGeofence fake(vertices, 4, NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION);
	_planner.update_vertices(fake, 0.f);
	_planner.update_destination(destination, fake);

	const int num_waypoints = _planner.set_start_and_plan_path_to_destination(start, fake);

	// start is the current vehicle position, so it is not part of the returned path.
	// path = vertex 1 + vertex 2 (2 points)
	ASSERT_EQ(num_waypoints, 2);

	// index 0 is the first waypoint after start
	const Vector2d first_wp = _planner.get_point_at_index(0);
	EXPECT_NEAR(first_wp(0), vertices[1](0), 1e-3);
	EXPECT_NEAR(first_wp(1), vertices[1](1), 1e-3);

	// index num_waypoints - 1 is the last waypoint before destination
	const Vector2d last_wp = _planner.get_point_at_index(num_waypoints - 1);
	EXPECT_NEAR(last_wp(0), vertices[2](0), 1e-3);
	EXPECT_NEAR(last_wp(1), vertices[2](1), 1e-3);
}

TEST_F(GeofenceAvoidancePlannerTest, PathInsideInclusionZone)
{
	using namespace matrix;
	Vector2<double> start(47.3560041, 8.5191872);
	Vector2<double> destination(47.3553241, 8.5177686);

	// inclusion geofence which forms an L, where start and end don't have a direct connection
	static const Vector2<double> vertices[] = {
		{47.3562241, 8.5186720},
		{47.3562479, 8.5195691},
		{47.3550948, 8.5196762},
		{47.3549660, 8.5171633},
		{47.3555168, 8.5171278},
		{47.3554507, 8.5187742}
	};

	FakeGeofence fake(vertices, 6, NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);
	_planner.update_vertices(fake, 0.f);
	_planner.update_destination(destination, fake);

	const int num_waypoints = _planner.set_start_and_plan_path_to_destination(start, fake);

	// start is the current vehicle position, so it is not part of the returned path.
	// path = vertex 5 (1 point)
	ASSERT_EQ(num_waypoints, 1);

	const Vector2d only_wp = _planner.get_point_at_index(0);
	EXPECT_NEAR(only_wp(0), vertices[5](0), 1e-3);
	EXPECT_NEAR(only_wp(1), vertices[5](1), 1e-3);
}

TEST_F(GeofenceAvoidancePlannerTest, DestinationOutsideInclusion)
{
	using namespace matrix;
	Vector2<double> start(47.3560041, 8.5191872);
	Vector2<double> destination(0, 0);

	// inclusion geofence which forms an L, where start and end don't have a direct connection
	static const Vector2<double> vertices[] = {
		{47.3562241, 8.5186720},
		{47.3562479, 8.5195691},
		{47.3550948, 8.5196762},
		{47.3549660, 8.5171633},
		{47.3555168, 8.5171278},
		{47.3554507, 8.5187742}
	};

	FakeGeofence fake(vertices, 6, NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);
	_planner.update_vertices(fake, 0.f);
	_planner.update_destination(destination, fake);

	const int num_waypoints = _planner.set_start_and_plan_path_to_destination(start, fake);

	ASSERT_EQ(num_waypoints, 0);
}

TEST_F(GeofenceAvoidancePlannerTest, DestinationInsideExclusion)
{
	using namespace matrix;
	Vector2<double> start(47.3559582, 8.5192064);

	// DESTINATION is inside the exclusion zone
	Vector2<double> destination(47.3551506, 8.5193859);

	static const Vector2<double> vertices[] = {
		{47.3552420, 8.5192293},
		{47.3555843, 8.5201174},
		{47.3551382, 8.5209143},
		{47.3550828, 8.5171901},
	};

	FakeGeofence fake(vertices, 4, NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION);
	_planner.update_vertices(fake, 0.f);
	_planner.update_destination(destination, fake);

	const int num_waypoints = _planner.set_start_and_plan_path_to_destination(start, fake);

	ASSERT_EQ(num_waypoints, 0);
}

TEST_F(GeofenceAvoidancePlannerTest, StartInsideExclusion)
{
	using namespace matrix;
	// START is inside the exclusion zone
	Vector2<double> start(47.3551506, 8.5193859);

	Vector2<double> destination(47.3559582, 8.5192064);

	static const Vector2<double> vertices[] = {
		{47.3552420, 8.5192293},
		{47.3555843, 8.5201174},
		{47.3551382, 8.5209143},
		{47.3550828, 8.5171901},
	};

	FakeGeofence fake(vertices, 4, NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION);
	_planner.update_vertices(fake, 0.f);
	_planner.update_destination(destination, fake);

	const int num_waypoints = _planner.set_start_and_plan_path_to_destination(start, fake);

	ASSERT_EQ(num_waypoints, 0);
}

TEST_F(GeofenceAvoidancePlannerTest, StartInsideGeofence)
{
	using namespace matrix;
	// Start and destination are both in valid airspace (north of the exclusion zone), so the
	// destination is directly reachable from the start without any detour.
	Vector2<double> start(47.3559582, 8.5192064);
	Vector2<double> destination(47.3560100, 8.5192300);

	static const Vector2<double> vertices[] = {
		{47.3552420, 8.5192293},
		{47.3555843, 8.5201174},
		{47.3551382, 8.5209143},
		{47.3550828, 8.5171901},
	};

	FakeGeofence fake(vertices, 4, NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION);
	_planner.update_vertices(fake, 0.f);
	_planner.update_destination(destination, fake);

	// Vehicle was outside the fence at plan time, so the start is used as an anchor the
	// vehicle must fly back to. Even though the destination is directly visible from there,
	// the path still has to include the anchor as the first (and only) waypoint.
	const int num_waypoints =
		_planner.set_start_and_plan_path_to_destination(start, fake, /*start_is_current_position=*/false);

	ASSERT_EQ(num_waypoints, 1);

	const Vector2d anchor_wp = _planner.get_point_at_index(0);
	EXPECT_NEAR(anchor_wp(0), start(0), 1e-3);
	EXPECT_NEAR(anchor_wp(1), start(1), 1e-3);
}

TEST_F(GeofenceAvoidancePlannerTest, NanStartOrDestination)
{
	using namespace matrix;
	Vector2<double> valid(47.3977, 8.5456);
	Vector2<double> nan_lat(NAN, 8.5456);
	Vector2<double> nan_lon(47.3977, NAN);

	FakeGeofence fake(nullptr, 0, NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);
	_planner.update_vertices(fake, 0.f);

	auto plan = [&](const Vector2<double> &s, const Vector2<double> &d) {
		_planner.update_destination(d, fake);
		return _planner.set_start_and_plan_path_to_destination(s, fake);
	};

	EXPECT_EQ(plan(nan_lat, valid), 0);
	EXPECT_EQ(plan(nan_lon, valid), 0);
	EXPECT_EQ(plan(valid, nan_lat), 0);
	EXPECT_EQ(plan(valid, nan_lon), 0);
}

TEST_F(GeofenceAvoidancePlannerTest, LatLonOutOfBounds)
{
	using namespace matrix;
	Vector2<double> valid(47.3977, 8.5456);
	Vector2<double> lat_too_high(91.0, 8.5456);
	Vector2<double> lat_too_low(-91.0, 8.5456);
	Vector2<double> lon_too_high(47.3977, 181.0);
	Vector2<double> lon_too_low(47.3977, -181.0);

	FakeGeofence fake(nullptr, 0, NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);
	_planner.update_vertices(fake, 0.f);

	auto plan = [&](const Vector2<double> &s, const Vector2<double> &d) {
		_planner.update_destination(d, fake);
		return _planner.set_start_and_plan_path_to_destination(s, fake);
	};

	EXPECT_EQ(plan(lat_too_high, valid), 0);
	EXPECT_EQ(plan(lat_too_low, valid), 0);
	EXPECT_EQ(plan(lon_too_high, valid), 0);
	EXPECT_EQ(plan(lon_too_low, valid), 0);
	EXPECT_EQ(plan(valid, lat_too_high), 0);
	EXPECT_EQ(plan(valid, lat_too_low), 0);
	EXPECT_EQ(plan(valid, lon_too_high), 0);
	EXPECT_EQ(plan(valid, lon_too_low), 0);
}

TEST_F(GeofenceAvoidancePlannerTest, DuplicateNeighborVertex)
{
	using namespace matrix;

	// WE HAVE start end end inside the inclusion fence
	Vector2<double> start(47.3560041, 8.5191872);
	Vector2<double> destination(47.3553241, 8.5177686);

	// BUT we have a fence with a duplicate vertex
	static const Vector2<double> vertices[] = {
		{47.3562241, 8.5186720},
		{47.3562479, 8.5195691},
		{47.3550948, 8.5196762},
		{47.3549660, 8.5171633},
		{47.3555168, 8.5171278},
		{47.3554507, 8.5187742},
		{47.3554507, 8.5187742}, // last vertex is a duplicate of the previous one
	};

	FakeGeofence fake(vertices, 7, NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);
	_planner.update_vertices(fake, 0.f);
	_planner.update_destination(destination, fake);

	const int num_waypoints = _planner.set_start_and_plan_path_to_destination(start, fake);

	// THEN the pathplanner should fail
	ASSERT_EQ(num_waypoints, 0);
}
