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
#include "geofence_interface.h"

static constexpr int kMaxNodes = 100;
static constexpr int kCircleApproxVertices = 8;

struct PlannedPath {
	// does not include the start and the end, just intermediate points
	matrix::Vector2<double> points[kMaxNodes];
	int num_points{0};
	int current_index{0};

	PlannedPath()
	{
		num_points = 0;
		current_index = 0;
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
			return matrix::Vector2d{NAN, NAN};
		}

	}

	matrix::Vector2d getCurrentPoint()
	{
		// returns the current points but does not increase the index
		if (current_index < num_points) {
			return points[current_index];
		}

		return matrix::Vector2d{NAN, NAN};

	}
};

struct VisibleVertex {
	int index;
	float distance;
};

struct VisibleVertices {
	VisibleVertex items[kMaxNodes];
	int count{0};
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
	~GeofenceAvoidancePlanner() = default;

	PlannedPath planPath(const matrix::Vector2<double> &start,
			     const matrix::Vector2<double> &destination,
			     GeofenceInterface *geofence,
			     float margin = 10.0f);


private:

	Node _graph_nodes[kMaxNodes];
	matrix::Vector2<double> _reference; // lat/lon of start, used as local frame origin

	bool calculate_graph_nodes(const matrix::Vector2<double> &start,
				   const matrix::Vector2<double> &destination,
				   GeofenceInterface *geofence,
				   float margin);

	bool lat_lon_within_bounds(const matrix::Vector2<double> &lat_lon);

};
