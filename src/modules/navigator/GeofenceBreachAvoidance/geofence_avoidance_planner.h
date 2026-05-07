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

/**
 * @file geofence_avoidance_planner.h
 * Ensures vehicle waypoints during RTL and autonomous modes remain
 * inside inclusion fences and outside exclusion fences.
 */

#include <cstdint>
#include <matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/geofence/geofence_utils.h>
#include "geofence_interface.h"

static constexpr int kMaxNodes = 100;
static constexpr int kMaxCircles = 16;
static constexpr int kCircleApproxVertices = 8;

struct PlannedPath {
	// points[0] is the start (latched valid position); intermediate waypoints follow; destination is not included
	matrix::Vector2<double> points[kMaxNodes];
	int num_points{0};
	int current_index{0};

	PlannedPath()
	{
		num_points = 0;
		current_index = 0;
	}

	matrix::Vector2d getPoint(int index)
	{
		if (index < num_points) {
			return points[index];

		} else {
			return matrix::Vector2d{(double)NAN, (double)NAN};
		}
	}

	bool hasNextPoint()
	{
		return current_index < num_points;

	}

	matrix::Vector2<double> getAndPopCurrentPoint()
	{
		if (current_index < num_points) {
			return points[current_index++];

		} else {
			return matrix::Vector2d{(double)NAN, (double)NAN};
		}

	}

	matrix::Vector2d getCurrentPoint()
	{
		// returns the current points but does not increase the index
		if (current_index < num_points) {
			return points[current_index];
		}

		return matrix::Vector2d{(double)NAN, (double)NAN};

	}
};

struct Node {
	enum Type : uint8_t {
		START,
		DESTINATION,
		VERTEX
	};

	float best_distance{FLT_MAX};
	int previous_index{-1};
	matrix::Vector2f position;
	Type type{VERTEX};
	bool visited = false;
};


class GeofenceAvoidancePlanner
{
public:
	GeofenceAvoidancePlanner() = default;
	~GeofenceAvoidancePlanner();


	int set_start_and_plan_path_to_destination(matrix::Vector2d start, GeofenceInterface &geofence,
			bool start_is_current_position = true);

	matrix::Vector2d get_point_at_index(int index) const;

	// True if the start passed to the last plan call was the vehicle's current position. False means
	// planning was anchored to a stored point (e.g. last valid in-fence position when the vehicle is
	// outside the fence), so the leg from current position to point 0 is unaccounted for in the path.
	bool start_is_current_position() const { return _start_is_current_position; }

	bool update_vertices(GeofenceInterface &geofence, float margin = 50.0f);

	void update_destination(const matrix::Vector2d &destination, GeofenceInterface &geofence);

private:

	static constexpr int num_distances_in_graph = (kMaxNodes) * (kMaxNodes - 1) / 2;

	float _best_distance[kMaxNodes];
	float _distances[num_distances_in_graph];
	int _next_node_buffer[kMaxNodes];
	bool _visited_buffer[kMaxNodes];

	int _best_starting_index{-1};
	int _num_path_points{0};

	matrix::Vector2f _start_local;

	int _num_nodes{0};
	int _num_vertices{0};
	int _dest_idx{-1};
	matrix::Vector2<double> _reference; // lat/lon anchor of the local frame

	// Cached, read-only fence representation. _polygons owns the master
	// node buffer (polygon vertices + circle approximation vertices +
	// destination); _circles holds the analytical circle metadata for the
	// line-vs-circle violation check. Built once per `update_vertices`
	// (= geofence geometry change); reused by every line-violates-fence
	// query during distance setup, destination updates, and start-anchor
	// selection.
	struct CachedCircle {
		matrix::Vector2f center;
		float radius;
	};
	geofence_utils::PlannerPolygons _polygons;
	CachedCircle _circles[kMaxCircles];
	int _num_circles{0};

	bool _polygons_healthy{false};
	bool _destination_healthy{false};
	bool _start_is_current_position{true};

	perf_counter_t _setup_perf{perf_alloc(PC_ELAPSED, "rtl_planner: setup")};
	perf_counter_t _setup_distances_perf{perf_alloc(PC_ELAPSED, "rtl_planner: setup distances")};
	perf_counter_t _update_destination_perf{perf_alloc(PC_ELAPSED, "rtl_planner: update destination")};
	perf_counter_t _plan_path_perf{perf_alloc(PC_ELAPSED, "rtl_planner: plan path")};

	bool update_graph_nodes_without_start_and_destination(GeofenceInterface &geofence, float margin);
	void update_distances_between_vertices();
	void planPath();

	// Read-only query against the cached fence: union of polygon (index-based)
	// and circle (analytic, float) checks. No geofence dataman roundtrips.
	bool lineViolatesAnyCachedFenceBetweenNodes(int a, int b) const;
	bool lineViolatesAnyCachedFenceFromPoint(const matrix::Vector2f &p, int node_idx) const;

	bool lat_lon_within_bounds(const matrix::Vector2<double> &lat_lon);

};
