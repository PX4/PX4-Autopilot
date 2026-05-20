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
static constexpr int kCircleApproxVertices = 8;

class GeofenceAvoidancePlanner
{
public:
	GeofenceAvoidancePlanner() = default;
	~GeofenceAvoidancePlanner();


	/**
	 * @brief Plan a path from `start` to the previously-set destination, falling back to a
	 *        saved in-fence anchor if `start` itself violates the fences.
	 *
	 * Pass the vehicle's current position as `start`. The planner first tries to plan from
	 * there; if no polygon node is visible (i.e. the position lies outside the planner's
	 * margined polygons), it transparently retries from the most recent in-fence position
	 * that was latched on a prior call. When `start` is itself in-fence it is latched as
	 * the new fallback for future calls — so calling this periodically with the current
	 * vehicle position keeps a usable anchor ready the moment one is needed. The result of
	 * which start was actually used is queryable via start_is_current_position().
	 *
	 * @return Number of waypoints the caller should consume from get_point_at_index().
	 *         When the planner falls back to the saved anchor, index 0 is that anchor; the
	 *         vehicle has to fly to it first.
	 */
	int set_start_and_plan_path_to_destination(matrix::Vector2d start, GeofenceInterface &geofence);

	matrix::Vector2d get_point_at_index(int index) const;

	// True if the start passed to the last plan call was the vehicle's current position. False means
	// planning was anchored to a stored point (e.g. last valid in-fence position when the vehicle is
	// outside the fence), so the leg from current position to point 0 is unaccounted for in the path.
	bool start_is_current_position() const { return _start_is_current_position; }

	bool update_vertices(GeofenceInterface &geofence, float margin = 10.0f);

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

	matrix::Vector2<double> _reference; // lat/lon anchor of the local frame

	// Cached, read-only fence representation. _polygons owns the master
	// node buffer (polygon vertices + circle k-gon approximation vertices +
	// destination). Built once per `update_vertices` (= geofence geometry
	// change); reused by every line-violates-fence query during distance
	// setup, destination updates, and start-anchor selection.
	geofence_utils::PlannerPolygons _polygons;

	bool _polygons_healthy{false};
	bool _destination_healthy{false};
	bool _start_is_current_position{true};

	// Latched fallback planner-start: most recent in-fence position passed to
	// set_start_and_plan_path_to_destination. Used by subsequent calls when the
	// caller-provided start has no visible node.
	matrix::Vector2<double> _saved_valid_start{(double)NAN, (double)NAN};

	perf_counter_t _setup_perf{perf_alloc(PC_ELAPSED, "rtl_planner: setup")};
	perf_counter_t _setup_distances_perf{perf_alloc(PC_ELAPSED, "rtl_planner: setup distances")};
	perf_counter_t _update_destination_perf{perf_alloc(PC_ELAPSED, "rtl_planner: update destination")};
	perf_counter_t _plan_path_perf{perf_alloc(PC_ELAPSED, "rtl_planner: plan path")};

	bool update_graph_nodes_without_start_and_destination(GeofenceInterface &geofence, float margin);
	void update_distances_between_vertices();
	void planPath();

	bool lat_lon_within_bounds(const matrix::Vector2<double> &lat_lon) const;

	/**
	 * @brief Search for the best polygon node visible from start_local.
	 *
	 * @param start_local             Candidate start in the planner's local frame.
	 * @param destination_directly_reachable Set true if the destination itself is visible from
	 *                                start_local (no detour needed).
	 * @return Index of the cheapest visible intermediate node (>= 1), or -1 if none is visible.
	 *         Note: destination_directly_reachable being true with a returned -1 means the
	 *         destination is reachable but no intermediate node is cheaper.
	 */
	int findBestStartingNode(const matrix::Vector2f &start_local, bool &destination_directly_reachable) const;

};
