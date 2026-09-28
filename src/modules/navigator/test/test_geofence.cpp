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

#include "geofence.h"
#include "navigator.h"
#include "support/navigator_dataman_test.h"

#include <lib/geo/geo.h>
#include <px4_platform_common/posix.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/geofence_status.h>

#include <vector>

class GeofenceTest : public NavigatorDatamanTestBase
{
protected:
	static constexpr double kReferenceLat = 47.0;
	static constexpr double kReferenceLon = 8.0;

	void SetUp() override
	{
		ASSERT_TRUE(_dataman_client.clearSync(DM_KEY_FENCE_POINTS_0));
		_navigator.get_land_detected()->landed = true;
	}

	static mission_fence_point_s makePointFromOffset(uint16_t command, float north_m, float east_m)
	{
		mission_fence_point_s point{};
		point.nav_cmd = command;
		point.frame = NAV_FRAME_GLOBAL;
		add_vector_to_global_position(kReferenceLat, kReferenceLon, north_m, east_m, &point.lat, &point.lon);
		return point;
	}

	::testing::AssertionResult loadFence(const std::vector<mission_fence_point_s> &points)
	{
		for (size_t i = 0; i < points.size(); ++i) {
			mission_fence_point_s point = points[i];

			if (!_dataman_client.writeSync(DM_KEY_FENCE_POINTS_0, static_cast<uint32_t>(i),
						       reinterpret_cast<uint8_t *>(&point), sizeof(point))) {
				return ::testing::AssertionFailure() << "failed to write fence item " << i;
			}
		}

		mission_stats_entry_s stats{};
		stats.num_items = static_cast<uint16_t>(points.size());
		stats.opaque_id = ++_fence_id;
		stats.dataman_id = DM_KEY_FENCE_POINTS_0;

		if (!_dataman_client.writeSync(DM_KEY_FENCE_POINTS_STATE, 0,
					       reinterpret_cast<uint8_t *>(&stats), sizeof(stats))) {
			return ::testing::AssertionFailure() << "failed to write fence metadata";
		}

		geofence_status_s status{};

		while (_status_sub.update(&status)) {}

		_fence.updateFence();
		const hrt_abstime start = hrt_absolute_time();

		while (hrt_elapsed_time(&start) < 5_s) {
			_fence.run();

			if (_status_sub.update(&status) && status.geofence_id == _fence_id) {
				if (status.status == geofence_status_s::GF_STATUS_READY) {
					return ::testing::AssertionSuccess();
				}

				if (status.status == geofence_status_s::GF_STATUS_FAILED) {
					return ::testing::AssertionFailure() << "fence load failed";
				}
			}

			px4_usleep(1000);
		}

		return ::testing::AssertionFailure() << "fence load timed out";
	}

	Navigator _navigator{};
	Geofence &_fence = _navigator.get_geofence();
	DatamanClient _dataman_client{};
	uORB::Subscription _status_sub{ORB_ID(geofence_status)};
	uint32_t _fence_id{0};
};

TEST_F(GeofenceTest, CircleAndPolygonLoadInEitherOrder)
{
	// A 20 m radius exclusion circle 75 m east of the rectangle's centre.
	mission_fence_point_s circle = makePointFromOffset(NAV_CMD_FENCE_CIRCLE_EXCLUSION, 0.f, 75.f);
	circle.circle_radius = 20.f;
	std::vector<mission_fence_point_s> polygon {
		makePointFromOffset(NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION, -100.f, -150.f),
		makePointFromOffset(NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION, 100.f, -150.f),
		makePointFromOffset(NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION, 100.f, 150.f),
		makePointFromOffset(NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION, -100.f, 150.f)
	};

	double outside_lat = kReferenceLat;
	double outside_lon = kReferenceLon;
	add_vector_to_global_position(kReferenceLat, kReferenceLon, 0.f, 250.f, &outside_lat, &outside_lon);

	for (mission_fence_point_s &vertex : polygon) {
		vertex.vertex_count = static_cast<uint16_t>(polygon.size());
	}

	for (bool circle_first : {true, false}) {
		SCOPED_TRACE(circle_first);
		std::vector<mission_fence_point_s> points = polygon;
		points.insert(circle_first ? points.begin() : points.end(), circle);
		ASSERT_TRUE(loadFence(points));

		GeofenceInterface &fence_interface = _fence;
		ASSERT_EQ(fence_interface.getNumPolygons(), 2);
		const PolygonInfo polygon_info = fence_interface.getPolygonInfoByIndex(circle_first ? 1 : 0);
		EXPECT_EQ(polygon_info.fence_type, NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);
		EXPECT_EQ(polygon_info.vertex_count, polygon.size());
		EXPECT_EQ(polygon_info.dataman_index, circle_first ? 1 : 0);

		const PolygonInfo circle_info = fence_interface.getPolygonInfoByIndex(circle_first ? 0 : 1);
		EXPECT_EQ(circle_info.fence_type, NAV_CMD_FENCE_CIRCLE_EXCLUSION);
		EXPECT_EQ(circle_info.dataman_index, circle_first ? 0 : polygon.size());
		EXPECT_FLOAT_EQ(circle_info.circle_radius, circle.circle_radius);

		EXPECT_TRUE(_fence.checkPointAgainstAllGeofences(kReferenceLat, kReferenceLon, 500.f));
		EXPECT_FALSE(_fence.checkPointAgainstAllGeofences(circle.lat, circle.lon, 500.f));
		EXPECT_FALSE(_fence.checkPointAgainstAllGeofences(outside_lat, outside_lon, 500.f));
	}
}
