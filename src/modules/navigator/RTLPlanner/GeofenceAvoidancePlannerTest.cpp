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
#include <navigator/navigation.h>

class FakeGeofence : public GeofenceInterface
{
public:
	FakeGeofence(const matrix::Vector2<double> *vertices, int num_vertices, uint16_t fence_type)
		: _vertices(vertices), _num_vertices(num_vertices), _fence_type(fence_type) {}

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
	_planner.updateGraphFromGeofence(fake, 0.f);
	_planner.updateDestination(point);

	const int num_waypoints = _planner.updateStartAndFillPath(point);

	ASSERT_EQ(num_waypoints, 0);
}

TEST_F(GeofenceAvoidancePlannerTest, DirectPathNoFence)
{
	using namespace matrix;
	Vector2<double> start(47.3977, 8.5456);
	Vector2<double> destination(47.3984, 8.5470);

	FakeGeofence fake(nullptr, 0, NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);
	_planner.updateGraphFromGeofence(fake, 0.f);
	_planner.updateDestination(destination);

	const int num_waypoints = _planner.updateStartAndFillPath(start);

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
	_planner.updateGraphFromGeofence(fake, 0.f);
	_planner.updateDestination(destination);

	const int num_waypoints = _planner.updateStartAndFillPath(start);

	// start is the current vehicle position, so it is not part of the returned path.
	// path = vertex 1 + vertex 2 (2 points)
	ASSERT_EQ(num_waypoints, 2);

	// index 0 is the first waypoint after start
	const Vector2d first_wp = _planner.waypointAtIndex(0);
	EXPECT_NEAR(first_wp(0), vertices[1](0), 1e-3);
	EXPECT_NEAR(first_wp(1), vertices[1](1), 1e-3);

	// index num_waypoints - 1 is the last waypoint before destination
	const Vector2d last_wp = _planner.waypointAtIndex(num_waypoints - 1);
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
	_planner.updateGraphFromGeofence(fake, 0.f);
	_planner.updateDestination(destination);

	const int num_waypoints = _planner.updateStartAndFillPath(start);

	// start is the current vehicle position, so it is not part of the returned path.
	// path = vertex 5 (1 point)
	ASSERT_EQ(num_waypoints, 1);

	const Vector2d only_wp = _planner.waypointAtIndex(0);
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
	_planner.updateGraphFromGeofence(fake, 0.f);
	_planner.updateDestination(destination);

	const int num_waypoints = _planner.updateStartAndFillPath(start);

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
	_planner.updateGraphFromGeofence(fake, 0.f);
	_planner.updateDestination(destination);

	const int num_waypoints = _planner.updateStartAndFillPath(start);

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
	_planner.updateGraphFromGeofence(fake, 0.f);
	_planner.updateDestination(destination);

	const int num_waypoints = _planner.updateStartAndFillPath(start);

	ASSERT_EQ(num_waypoints, 0);
}

TEST_F(GeofenceAvoidancePlannerTest, FallsBackToSavedAnchorWhenStartViolatesFence)
{
	using namespace matrix;
	// `anchor` is in valid airspace (north of the exclusion polygon) and `destination`
	// sits just next to it so it's directly reachable from there. `inside_exclusion`
	// is inside the exclusion polygon, i.e. it violates the fence and cannot be used
	// as a planner start.
	Vector2<double> anchor(47.3559582, 8.5192064);
	Vector2<double> destination(47.3560100, 8.5192300);
	Vector2<double> inside_exclusion(47.3553000, 8.5197000);

	static const Vector2<double> vertices[] = {
		{47.3552420, 8.5192293},
		{47.3555843, 8.5201174},
		{47.3551382, 8.5209143},
		{47.3550828, 8.5171901},
	};

	FakeGeofence fake(vertices, 4, NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION);
	_planner.updateGraphFromGeofence(fake, 0.f);
	_planner.updateDestination(destination);

	// Seed the planner with the anchor as a known-good fallback start by planning from it:
	// an in-fence start is latched internally as the fallback for future calls.
	_planner.updateStartAndFillPath(anchor);

	// Plan from a position that violates the fence. The planner should detect that the
	// provided start has no visible polygon node, fall back to the latched anchor, and
	// include the anchor as the first (and only) waypoint since the destination is
	// directly reachable from there.
	const int num_waypoints = _planner.updateStartAndFillPath(inside_exclusion);

	ASSERT_EQ(num_waypoints, 1);

	const Vector2d anchor_wp = _planner.waypointAtIndex(0);
	EXPECT_NEAR(anchor_wp(0), anchor(0), 1e-3);
	EXPECT_NEAR(anchor_wp(1), anchor(1), 1e-3);
}

TEST_F(GeofenceAvoidancePlannerTest, NanStartOrDestination)
{
	using namespace matrix;
	Vector2<double> valid(47.3977, 8.5456);
	Vector2<double> nan_lat(NAN, 8.5456);
	Vector2<double> nan_lon(47.3977, NAN);

	FakeGeofence fake(nullptr, 0, NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);
	_planner.updateGraphFromGeofence(fake, 0.f);

	auto plan = [&](const Vector2<double> &s, const Vector2<double> &d) {
		_planner.updateDestination(d);
		return _planner.updateStartAndFillPath(s);
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
	_planner.updateGraphFromGeofence(fake, 0.f);

	auto plan = [&](const Vector2<double> &s, const Vector2<double> &d) {
		_planner.updateDestination(d);
		return _planner.updateStartAndFillPath(s);
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
	_planner.updateGraphFromGeofence(fake, 0.f);
	_planner.updateDestination(destination);

	const int num_waypoints = _planner.updateStartAndFillPath(start);

	// THEN the pathplanner should fail, printing:
	// WARN  [geofence_avoidance_planner] geofence avoidance: polygon 0 (7 vertices) rejected by planner
	ASSERT_EQ(num_waypoints, 0);
}

TEST_F(GeofenceAvoidancePlannerTest, CursorInterface)
{
	using namespace matrix;
	// Reuse the exclusion-zone scenario which produces a 2-waypoint path.
	Vector2<double> start(47.3559582, 8.5192064);
	Vector2<double> destination(47.3546153, 8.5193195);

	static const Vector2<double> vertices[] = {
		{47.3552420, 8.5192293},
		{47.3555843, 8.5201174},
		{47.3551382, 8.5209143},
		{47.3550828, 8.5171901},
	};

	FakeGeofence fake(vertices, 4, NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION);
	_planner.updateGraphFromGeofence(fake, 0.f);
	_planner.updateDestination(destination);
	_planner.updateStartAndFillPath(start);

	ASSERT_EQ(_planner.get_num_waypoints(), 2);

	// --- cursor starts at 0 ---
	EXPECT_TRUE(_planner.hasMore());
	EXPECT_EQ(_planner.getPathCursor(), 0);

	// getCurrentWaypoint() and waypointAtIndex(0) agree
	const Vector2d cur0 = _planner.getCurrentWaypoint();
	const Vector2d idx0 = _planner.waypointAtIndex(0);
	EXPECT_NEAR(cur0(0), idx0(0), 1e-6);
	EXPECT_NEAR(cur0(1), idx0(1), 1e-6);

	// getNextWaypoint() matches waypointAtIndex(1)
	const Vector2d nxt0 = _planner.getNextWaypoint();
	const Vector2d idx1 = _planner.waypointAtIndex(1);
	EXPECT_NEAR(nxt0(0), idx1(0), 1e-6);
	EXPECT_NEAR(nxt0(1), idx1(1), 1e-6);

	// --- advance to cursor 1 ---
	_planner.advanceWaypoint();
	EXPECT_TRUE(_planner.hasMore());
	EXPECT_EQ(_planner.getPathCursor(), 1);

	// current is now the second waypoint
	const Vector2d cur1 = _planner.getCurrentWaypoint();
	EXPECT_NEAR(cur1(0), idx1(0), 1e-6);
	EXPECT_NEAR(cur1(1), idx1(1), 1e-6);

	// no next after the last waypoint
	EXPECT_FALSE(_planner.getNextWaypoint().isAllFinite());

	// --- advance past the end ---
	_planner.advanceWaypoint();
	EXPECT_FALSE(_planner.hasMore());
	EXPECT_FALSE(_planner.getCurrentWaypoint().isAllFinite());
	EXPECT_EQ(_planner.getPathCursor(), 2);
}
