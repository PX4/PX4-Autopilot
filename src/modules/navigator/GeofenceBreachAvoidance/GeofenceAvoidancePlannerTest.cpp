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

		for (int i = 0; i < _num_vertices; i++) {
			matrix::Vector2f v1_local = ref.project(_vertices[i](0), _vertices[i](1));
			matrix::Vector2f v2_local = ref.project(_vertices[(i + 1) % _num_vertices](0),
								_vertices[(i + 1) % _num_vertices](1));

			if (geofence_utils::segmentsIntersect(start_local, end_local, v1_local, v2_local)) {
				return true;
			}
		}

		return false;
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
	_planner.update_vertices(fake);
	_planner.update_start(point, fake);
	_planner.update_destination(point, fake);

	PlannedPath path = _planner.planPath();

	ASSERT_EQ(path.num_points, 0);
}

TEST_F(GeofenceAvoidancePlannerTest, DirectPathNoFence)
{
	using namespace matrix;
	Vector2<double> start(47.3977, 8.5456);
	Vector2<double> destination(47.3984, 8.5470);

	FakeGeofence fake(nullptr, 0, NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);
	_planner.update_vertices(fake);
	_planner.update_start(start, fake);
	_planner.update_destination(destination, fake);

	PlannedPath path = _planner.planPath();

	ASSERT_EQ(path.num_points, 0);
}

TEST_F(GeofenceAvoidancePlannerTest, PathAroundExclusionZone)
{
	// in this test there is an exclusion zone between start and destination. The shortest path
	// is chosen to be via vertex nr 2 and vertex nr 3, so the path should have two points
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
	_planner.update_vertices(fake);
	_planner.update_start(start, fake);
	_planner.update_destination(destination, fake);

	PlannedPath path = _planner.planPath();

	ASSERT_EQ(path.num_points, 2);
	// The optimal path is via vertex 1 and 2
	EXPECT_NEAR(path.points[0](0), vertices[1](0), 1e-3);
	EXPECT_NEAR(path.points[0](1), vertices[1](1), 1e-3);
	EXPECT_NEAR(path.points[1](0), vertices[2](0), 1e-3);
	EXPECT_NEAR(path.points[1](1), vertices[2](1), 1e-3);
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
	_planner.update_vertices(fake);
	_planner.update_start(start, fake);
	_planner.update_destination(destination, fake);

	PlannedPath path = _planner.planPath();

	ASSERT_EQ(path.num_points, 1);
	ASSERT_NEAR(path.points[0](0), vertices[5](0), 1e-3);
	ASSERT_NEAR(path.points[0](1), vertices[5](1), 1e-3);
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
	_planner.update_vertices(fake);
	_planner.update_start(start, fake);
	_planner.update_destination(destination, fake);

	PlannedPath path = _planner.planPath();

	ASSERT_EQ(path.num_points, 0);
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
	_planner.update_vertices(fake);
	_planner.update_start(start, fake);
	_planner.update_destination(destination, fake);

	PlannedPath path = _planner.planPath();

	ASSERT_EQ(path.num_points, 0);
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
	_planner.update_vertices(fake);
	_planner.update_start(start, fake);
	_planner.update_destination(destination, fake);

	PlannedPath path = _planner.planPath();

	ASSERT_EQ(path.num_points, 0);
}
TEST_F(GeofenceAvoidancePlannerTest, NanStartOrDestination)
{
	using namespace matrix;
	Vector2<double> valid(47.3977, 8.5456);
	Vector2<double> nan_lat(NAN, 8.5456);
	Vector2<double> nan_lon(47.3977, NAN);

	FakeGeofence fake(nullptr, 0, NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);
	_planner.update_vertices(fake);

	auto plan = [&](const Vector2<double> &s, const Vector2<double> &d) {
		_planner.update_start(s, fake);
		_planner.update_destination(d, fake);
		return _planner.planPath();
	};

	EXPECT_EQ(plan(nan_lat, valid).num_points, 0);
	EXPECT_EQ(plan(nan_lon, valid).num_points, 0);
	EXPECT_EQ(plan(valid, nan_lat).num_points, 0);
	EXPECT_EQ(plan(valid, nan_lon).num_points, 0);
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
	_planner.update_vertices(fake);

	auto plan = [&](const Vector2<double> &s, const Vector2<double> &d) {
		_planner.update_start(s, fake);
		_planner.update_destination(d, fake);
		return _planner.planPath();
	};

	EXPECT_EQ(plan(lat_too_high, valid).num_points, 0);
	EXPECT_EQ(plan(lat_too_low, valid).num_points, 0);
	EXPECT_EQ(plan(lon_too_high, valid).num_points, 0);
	EXPECT_EQ(plan(lon_too_low, valid).num_points, 0);
	EXPECT_EQ(plan(valid, lat_too_high).num_points, 0);
	EXPECT_EQ(plan(valid, lat_too_low).num_points, 0);
	EXPECT_EQ(plan(valid, lon_too_high).num_points, 0);
	EXPECT_EQ(plan(valid, lon_too_low).num_points, 0);
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
	_planner.update_vertices(fake);
	_planner.update_start(start, fake);
	_planner.update_destination(destination, fake);

	PlannedPath path = _planner.planPath();

	// THEN the pathplanner should fail
	ASSERT_EQ(path.num_points, 0);
}
