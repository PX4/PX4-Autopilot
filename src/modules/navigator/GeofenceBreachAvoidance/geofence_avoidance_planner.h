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
#include "geofence_interface.h"

static constexpr int kMaxNodes = 100;
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

	const PlannedPath &planPath();

	bool update_vertices(GeofenceInterface &geofence, float margin = 10.0f);
	void update_start(const matrix::Vector2d &start, GeofenceInterface &geofence);
	void update_destination(const matrix::Vector2d &destination, GeofenceInterface &geofence);

private:

	static constexpr int num_distances_in_graph = (kMaxNodes) * (kMaxNodes - 1) / 2;

	Node _graph_nodes[kMaxNodes];
	float _distances[num_distances_in_graph];
	PlannedPath _planned_path{};
	int _num_nodes{0};
	int _num_vertices{0};
	matrix::Vector2<double> _reference; // lat/lon anchor of the local frame

	bool _polygons_healthy{false};
	bool _start_healthy{false};
	bool _destination_healthy{false};

	perf_counter_t _setup_perf{perf_alloc(PC_ELAPSED, "rtl_planner: setup")};
	perf_counter_t _setup_distances_perf{perf_alloc(PC_ELAPSED, "rtl_planner: setup distances")};
	perf_counter_t _update_start_perf{perf_alloc(PC_ELAPSED, "rtl_planner: update start")};
	perf_counter_t _update_destination_perf{perf_alloc(PC_ELAPSED, "rtl_planner: update destination")};
	perf_counter_t _plan_path_perf{perf_alloc(PC_ELAPSED, "rtl_planner: plan path")};

	bool update_graph_nodes_without_start_and_destination(GeofenceInterface &geofence, float margin);
	void update_distances_between_vertices(GeofenceInterface &geofence);

	void reset_graph_state();

	bool lat_lon_within_bounds(const matrix::Vector2<double> &lat_lon);

};
