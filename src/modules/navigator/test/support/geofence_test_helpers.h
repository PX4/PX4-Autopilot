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

#pragma once

#include "navigator_dataman_test.h"

#include "geofence.h"
#include "navigator.h"

#include <initializer_list>
#include <vector>

#include <lib/geo/geo.h>
#include <px4_platform_common/posix.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/geofence_status.h>

class GeofenceTestPeer
{
public:
	static void reset(Geofence &fence)
	{
		fence._clearFence();
		fence._dataman_cache.invalidate();
		fence._dataman_state = Geofence::DatamanState::UpdateRequestWait;
		fence._fence_loaded = false;
		fence._projection_reference = MapProjection{};
		fence._altitude_min = 0.f;
		fence._altitude_max = 0.f;
		fence.updateFence();
	}

	static void invalidateCache(Geofence &fence) { fence._dataman_cache.invalidate(); }

	static bool replaceCachedPoint(Geofence &fence, uint32_t index, const mission_fence_point_s &point)
	{
		return fence._dataman_cache.updateCachedItem(static_cast<dm_item_t>(fence._stats.dataman_id), index,
				reinterpret_cast<const uint8_t *>(&point), sizeof(point));
	}

	static void setAltitudeBand(Geofence &fence, float minimum, float maximum)
	{
		fence._altitude_min = minimum;
		fence._altitude_max = maximum;
	}

	static void setVertexCount(Geofence &fence, uint16_t count) { fence._polygons[0].vertex_count = count; }

	static void useEquatorProjection(Geofence &fence) { fence._projection_reference.initReference(0.0, 0.0); }

	static void finishUpdate(Geofence &fence) { fence._finishFenceUpdate(true); }

	static bool waitForPendingRead(Geofence &fence)
	{
		return DatamanClientTestPeer::waitForOperation(fence._dataman_client, 1_s);
	}

	static bool failPendingRead(Geofence &fence)
	{
		return DatamanClientTestPeer::completeOperationWithFailure(fence._dataman_client);
	}
};

namespace navigator_test
{

class GeofenceTestNavigator : public Navigator
{
public:
	using Navigator::updateParams;
};

class GeofenceTestBase : public NavigatorDatamanTestBase
{
protected:
	using FencePoints = std::vector<mission_fence_point_s>;

	// Geometry uses north/east metre offsets from this reference.
	explicit GeofenceTestBase(double reference_lat = 47.0, double reference_lon = 8.0) :
		_reference(reference_lat, reference_lon) {}

	::testing::AssertionResult resetFence(float home_altitude = 400.f)
	{
		if (!_dataman_client.clearSync(DM_KEY_FENCE_POINTS_0)) {
			return ::testing::AssertionFailure() << "failed to clear fence items";
		}

		GeofenceTestPeer::reset(_fence);
		_navigator.get_land_detected()->landed = true;
		home_position_s *home = _navigator.get_home_position();
		home->timestamp = hrt_absolute_time();
		home->lat = _reference(0);
		home->lon = _reference(1);
		home->alt = home_altitude;
		home->valid_hpos = true;
		home->valid_alt = true;
		return ::testing::AssertionSuccess();
	}

	matrix::Vector2d position(float north_m, float east_m) const
	{
		double lat, lon;
		add_vector_to_global_position(_reference(0), _reference(1), north_m, east_m, &lat, &lon);
		return {lat, lon};
	}

	Geofence::PathCheck path(const matrix::Vector2f &start, const matrix::Vector2f &end) const
	{
		return {position(start(0), start(1)), position(end(0), end(1))};
	}

	FencePoints polygon(bool inclusion, std::initializer_list<matrix::Vector2f> vertices) const
	{
		FencePoints points;

		for (const matrix::Vector2f &vertex : vertices) {
			const auto coordinate = position(vertex(0), vertex(1));
			mission_fence_point_s point{};
			point.nav_cmd = inclusion ? NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION : NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION;
			point.frame = NAV_FRAME_GLOBAL;
			point.lat = coordinate(0);
			point.lon = coordinate(1);
			point.vertex_count = static_cast<uint16_t>(vertices.size());
			points.push_back(point);
		}

		return points;
	}

	FencePoints exclusionSquare() const
	{
		// A 100 m square centred 300 m east of Home.
		return polygon(false, {{-50.f, 250.f}, {50.f, 250.f}, {50.f, 350.f}, {-50.f, 350.f}});
	}

	FencePoints circle(bool inclusion, const matrix::Vector2f &centre, float radius) const
	{
		const auto coordinate = position(centre(0), centre(1));
		mission_fence_point_s point{};
		point.nav_cmd = inclusion ? NAV_CMD_FENCE_CIRCLE_INCLUSION : NAV_CMD_FENCE_CIRCLE_EXCLUSION;
		point.frame = NAV_FRAME_GLOBAL;
		point.lat = coordinate(0);
		point.lon = coordinate(1);
		point.circle_radius = radius;
		return {point};
	}

	::testing::AssertionResult waitForFence(uint8_t expected_status = geofence_status_s::GF_STATUS_READY)
	{
		const hrt_abstime start = hrt_absolute_time();

		// The deadline only bounds failures; successful loads return immediately.
		while (hrt_elapsed_time(&start) < 1_s) {
			_fence.run();
			geofence_status_s status{};

			if (_fence_status_sub.update(&status) && status.geofence_id == _fence_id
			    && status.status != geofence_status_s::GF_STATUS_LOADING) {
				if (status.status != expected_status) {
					return ::testing::AssertionFailure() << "fence status " << int(status.status)
					       << ", expected " << int(expected_status);
				}

				return ::testing::AssertionSuccess();
			}

			px4_usleep(1000);
		}

		return ::testing::AssertionFailure() << "fence load timed out";
	}

	::testing::AssertionResult loadFence(const FencePoints &points)
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
		_fence_id = ++runtime().next_fence_id;
		stats.opaque_id = _fence_id;
		stats.dataman_id = DM_KEY_FENCE_POINTS_0;

		if (!_dataman_client.writeSync(DM_KEY_FENCE_POINTS_STATE, 0,
					       reinterpret_cast<uint8_t *>(&stats), sizeof(stats))) {
			return ::testing::AssertionFailure() << "failed to write fence metadata";
		}

		_fence.updateFence();
		return waitForFence();
	}

	struct Runtime {
		GeofenceTestNavigator navigator;
		DatamanClient dataman_client;
		uint32_t next_fence_id{0};
	};

	static Runtime &runtime()
	{
		// Dataman client IDs are not reclaimed, so reuse clients across test cases.
		static Runtime instance;
		return instance;
	}

	GeofenceTestNavigator &_navigator = runtime().navigator;
	Geofence &_fence = _navigator.get_geofence();
	DatamanClient &_dataman_client = runtime().dataman_client;
	uORB::Subscription _fence_status_sub{ORB_ID(geofence_status)};
	uint32_t _fence_id{0};
	const matrix::Vector2d _reference;
};

} // namespace navigator_test
