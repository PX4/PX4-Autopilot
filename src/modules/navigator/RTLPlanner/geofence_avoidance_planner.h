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

class GeofenceAvoidancePlanner
{
public:
	GeofenceAvoidancePlanner() = default;
	~GeofenceAvoidancePlanner();

	/**
	 * Result of the latest graph build & dijkstra run.
	 * Is converted to user-facing warning in navigator_main.
	 */
	enum class Status {
		Success,                     // No error
		NoFence,                     // No fence polygons -- avoidance not applicable.
		DijkstraFailed,              // Dijkstra solve exited due to invalid input data.
		DestinationInvalid,          // Destination lat/lon non-finite, out of [-90,90]/[-180,180], or out of fixed-point range (internal).
		DestinationBreachesGeofence, // Destination breaches geofence
		BudgetExceeded,              // Node/polygon storage budget exceeded (should not normally happen, see static_assert).
		OutOfRange,                  // A zone's vertex/extent fell outside the usable fixed-point range.
		Degenerate,                  // A zone was degenerate (< 3 vertices, self-intersecting, antiparallel/zero-length edge, empty circle, negative margin).
	};

	Status status() const { return _status; }

	void resetStatus() { _status = Status::Success; }

	/**
	 * True if the latest updateStartAndFillPath() found neither a routed path nor a direct
	 * line to the destination -- the caller will fly directly and ignore geofences.
	 * Stays false when no fence is loaded (Status::NoFence), as there is nothing to avoid.
	 */
	bool needsStraightLineFallback() const { return _straight_line_fallback; }

	/**
	 * Fill the path from `start` to the destination. If `start` is
	 * breaching a (margin-expanded) geofence, start with the last
	 * non-breaching vehicle position.
	 *
	 * Does not re-plan - that only happens after updateGraphFromGeofence
	 * and updateDestination.
	 *
	 * The resulting path is available for external consumers through
	 * getCurrentWaypoint, getNextWaypoint, advanceWaypoint.
	 *
	 * @return Number of waypoints in the path. Only used in tests.
	 */
	int updateStartAndFillPath(matrix::Vector2d start);

	// --- Cursor-based path-following interface ---
	// Call set_start_and_plan_path_to_destination() first to plan; it resets the cursor to 0.

	// True while there are waypoints left to fly.
	bool hasMore() const { return _path_cursor < _path_length; }

	// The waypoint to fly to now. NaN if hasMore() is false.
	matrix::Vector2d getCurrentWaypoint() const
	{
		return hasMore() ? _path[_path_cursor] : matrix::Vector2d{(double)NAN, (double)NAN};
	}

	// The waypoint after current (for populating triplet.next). NaN if current is the last.
	matrix::Vector2d getNextWaypoint() const
	{
		return (_path_cursor + 1 < _path_length) ? _path[_path_cursor + 1] : matrix::Vector2d{(double)NAN, (double)NAN};
	}

	// Mark current waypoint reached; advance to next. No-op when hasMore() is false.
	void advanceWaypoint() { if (_path_cursor < _path_length) { ++_path_cursor; } }

	// Number of waypoints to fly.
	int get_num_waypoints() const { return _path_length; }

	// 0-indexed access to waypoints. Primarily used by time estimators.
	matrix::Vector2d waypointAtIndex(int index) const
	{
		if (index < 0 || index >= _path_length) {
			return matrix::Vector2d{(double)NAN, (double)NAN};
		}

		return _path[index];
	}

	// Current cursor position (0-indexed).
	int getPathCursor() const { return _path_cursor; }

	void updateGraphFromGeofence(GeofenceInterface &geofence, float margin = 10.0f);

	void updateDestination(const matrix::Vector2d &destination);

private:

	static constexpr int kMaxNodes = geofence_utils::PlannerPolygons::kMaxNodes;
	static constexpr int num_distances_in_graph = kMaxNodes * (kMaxNodes - 1) / 2;

	float _best_distance[kMaxNodes];
	float _distances[num_distances_in_graph];
	int _next_node_buffer[kMaxNodes];
	bool _visited_buffer[kMaxNodes];

	// Stored flat path. Worst case: 1 anchor + kMaxNodes DAG vertices.
	matrix::Vector2d _path[kMaxNodes + 1];
	int _path_length{0};
	int _path_cursor{0};

	matrix::Vector2<double> _reference; // lat/lon anchor of the local frame

	// Cached fence representation.
	//  - Stores polygons plus safety margin in fixed-point (for robust geometry calculations)
	//  - Abstracts away geometry, provides edge cost between any two nodes
	//  - Updated on updateGraphFromGeofence (through updatePolygonsFromGeofence), otherwise read only
	geofence_utils::PlannerPolygons _polygons;

	bool _polygons_healthy{false};
	bool _destination_healthy{false};
	Status _status{Status::NoFence};
	bool _straight_line_fallback{false};

	// Most recent in-fence position passed to updateStartAndFillPath. Used
	// by subsequent calls as fallback when we have breached a geofence.
	matrix::Vector2<double> _saved_valid_start{(double)NAN, (double)NAN};

	perf_counter_t _update_polygons_perf{perf_alloc(PC_ELAPSED, "rtl_planner: polygons")};
	perf_counter_t _update_edge_costs_perf{perf_alloc(PC_ELAPSED, "rtl_planner: edge costs")};
	perf_counter_t _plan_path_perf{perf_alloc(PC_ELAPSED, "rtl_planner: plan path")};
	perf_counter_t _lookup_path_perf{perf_alloc(PC_ELAPSED, "rtl_planner: lookup path")};

	// Sets _status on failure (matching planPath); returns false to abort the build.
	bool updatePolygonsFromGeofence(GeofenceInterface &geofence, float margin);
	void updateEdgeCosts();
	void planPath();


	static bool latLonWithinBounds(const matrix::Vector2<double> &lat_lon)
	{
		return lat_lon(0) >= -90.0 && lat_lon(0) <= 90.0
		       && lat_lon(1) >= -180.0 && lat_lon(1) <= 180.0;
	}

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
