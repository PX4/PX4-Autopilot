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

#include "geofence_avoidance_planner.h"
#include <lib/geo/geo.h>
#include <lib/geofence/geofence_utils.h>
#include <lib/dijkstra/dijkstra.h>

GeofenceAvoidancePlanner::~GeofenceAvoidancePlanner()
{
	perf_free(_update_polygons_perf);
	perf_free(_update_edge_costs_perf);
	perf_free(_plan_path_perf);
	perf_free(_lookup_path_perf);
}

static matrix::Vector2f get_vertex_local_position(int poly_index, int vertex_idx,
		GeofenceInterface &geofence,
		const matrix::Vector2<double> &reference)
{
	matrix::Vector2<double> latlon = geofence.getPolygonVertexByIndex(poly_index, vertex_idx);
	MapProjection ref{reference(0), reference(1)};
	matrix::Vector2f local;
	ref.project(latlon(0), latlon(1), local(0), local(1));
	return local;
}

void GeofenceAvoidancePlanner::planPath()
{
	if (!_polygons_healthy || !_destination_healthy) {
		// Do not change _status, it already reflects the reason for the unhealthy state
		return;
	}

	perf_begin(_plan_path_perf);

	const bool ret = dijkstra::solveBackward(_polygons.numNodes(), _polygons.destIndex(), _distances, true, _best_distance,
			 _next_node_buffer, _visited_buffer);

	perf_end(_plan_path_perf);

	_status = ret ? Status::Success : Status::DijkstraFailed;
}

void GeofenceAvoidancePlanner::updateGraphFromGeofence(GeofenceInterface &geofence, float margin)
{
	// Polygons are about to change; any previously latched fallback start may no longer be valid.
	_saved_valid_start = matrix::Vector2<double> {(double)NAN, (double)NAN};

	_polygons_healthy = true;

	const int num_polygons = geofence.getNumPolygons();
	int num_vertices{0};

	// Before copying the information, sanity check if we have space
	for (int poly_idx = 0; poly_idx < num_polygons; poly_idx++) {
		PolygonInfo info = geofence.getPolygonInfoByIndex(poly_idx);

		if (info.fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION || info.fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION) {
			// Worst case: the maximum possible number of corners are sharp and have to be split
			num_vertices += geofence_utils::maxVerticesAfterSplitting(info.vertex_count);

		} else if (info.fence_type == NAV_CMD_FENCE_CIRCLE_INCLUSION || info.fence_type == NAV_CMD_FENCE_CIRCLE_EXCLUSION) {
			num_vertices += geofence_utils::PlannerPolygons::kCircleApproxVertices;
		}
	}

	if (num_vertices == 0) {
		_polygons_healthy = false;
		_status = Status::NoFence;
		return;
	}

	if (num_vertices > kMaxNodes - 1) { // -1 to reserve the destination slot
		// Fence larger than the per-board budget (kMaxNodes). On boards where
		// kMaxNodes >= kMaxNodesForAnyStorableFence this cannot happen.
		_polygons_healthy = false;
		_status = Status::BudgetExceeded;
		return;
	}

	_reference = geofence.getPolygonVertexByIndex(0, 0);

	if (!updatePolygonsFromGeofence(geofence, margin)) {
		_polygons_healthy = false;
		return;
	}

	updateEdgeCosts();

	// The graph has changed, so we need to re-plan.
	_status = Status::Success;
	planPath();
}

bool GeofenceAvoidancePlanner::updatePolygonsFromGeofence(
	GeofenceInterface &geofence, float margin)
{
	perf_begin(_update_polygons_perf);

	const int num_polygons = geofence.getNumPolygons();

	_polygons.reset();

	for (int poly_index = 0; poly_index < num_polygons; poly_index++) {

		PolygonInfo info = geofence.getPolygonInfoByIndex(poly_index);

		auto add_result = geofence_utils::PlannerPolygons::AddResult::Success;

		if (info.fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION || info.fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION) {

			// Could skip this local copy and pass e.g. a reference to geofence into addPolygon, so it could access directly
			matrix::Vector2f local_in[info.vertex_count];

			for (int vertex_idx = 0; vertex_idx < info.vertex_count; vertex_idx++) {
				local_in[vertex_idx] = get_vertex_local_position(poly_index, vertex_idx, geofence, _reference);
			}

			const bool is_inclusion = (info.fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);

			add_result = _polygons.addPolygon(local_in, info.vertex_count, is_inclusion, margin);

		} else if (info.fence_type == NAV_CMD_FENCE_CIRCLE_INCLUSION || info.fence_type == NAV_CMD_FENCE_CIRCLE_EXCLUSION) {

			const matrix::Vector2f center = get_vertex_local_position(poly_index, 0, geofence, _reference);

			const bool is_inclusion = (info.fence_type == NAV_CMD_FENCE_CIRCLE_INCLUSION);

			add_result = _polygons.addApproxCircle(center, info.circle_radius, margin, is_inclusion);
		}

		switch (add_result) {
		case geofence_utils::PlannerPolygons::AddResult::Success:
			break;

		case geofence_utils::PlannerPolygons::AddResult::BudgetExceeded:
			_status = Status::BudgetExceeded;
			perf_cancel(_update_polygons_perf);
			return false;

		case geofence_utils::PlannerPolygons::AddResult::OutOfRange:
			_status = Status::OutOfRange;
			perf_cancel(_update_polygons_perf);
			return false;

		case geofence_utils::PlannerPolygons::AddResult::Degenerate:
			_status = Status::Degenerate;
			perf_cancel(_update_polygons_perf);
			return false;
		}
	}

	perf_end(_update_polygons_perf);

	return true;
}

void GeofenceAvoidancePlanner::updateEdgeCosts()
{
	perf_begin(_update_edge_costs_perf);

	// All edges in the upper triangle, INCLUDING destination-incident ones (i==0).
	// Polygon vertices occupy indices 1..numNodes()-1; destination is at 0.
	for (int i = 0; i < _polygons.numNodes(); i++) {
		for (int j = i + 1; j < _polygons.numNodes(); j++) {
			const size_t idx = dijkstra::symmetricPairIndex(i, j, _polygons.numNodes());
			_distances[idx] = _polygons.edgeCost(i, j);
		}
	}

	perf_end(_update_edge_costs_perf);
}

void GeofenceAvoidancePlanner::updateDestination(const matrix::Vector2d &destination)
{
	if (!_polygons_healthy) {
		// Polygons unhealthy -- _status already reflects the root cause, do not overwrite.
		return;
	}

	if (!destination.isAllFinite() || !latLonWithinBounds(destination)) {
		_destination_healthy = false;
		_status = Status::DestinationInvalid;
		return;
	}

	MapProjection ref{_reference(0), _reference(1)};
	matrix::Vector2f dest_local;
	ref.project(destination(0), destination(1), dest_local(0), dest_local(1));

	if (!geofence_utils::inFixedPointRange(dest_local(0))
	    || !geofence_utils::inFixedPointRange(dest_local(1))) {
		_destination_healthy = false;
		_status = Status::DestinationInvalid;
		return;
	}

	// PlannerPolygons stores positions in int32-cm, so a setDestination/getDestination
	// roundtrip introduces up to 0.5cm of error -- hence the comparison tolerance.
	if ((dest_local - _polygons.getDestination()).norm() < 0.1f) {
		return;
	}

	_destination_healthy = _polygons.setDestination(dest_local);

	if (!_destination_healthy) {
		// Keep the (breaching) destination so RTL can still fall back to a straight
		// line; just flag it for the user-facing warning.
		_status = Status::DestinationBreachesGeofence;
		return;
	}

	// Destination changed -- only the destination-incident edges (node 0) need
	// refreshing; polygon-polygon edges are independent of the destination. This is
	// linear in the number of nodes, unlike the quadratic full rebuild, so it is not
	// worth a perf counter.
	for (int j = 1; j < _polygons.numNodes(); j++) {
		_distances[dijkstra::symmetricPairIndex(0, j, _polygons.numNodes())] = _polygons.edgeCost(0, j);
	}

	planPath();
}

int GeofenceAvoidancePlanner::findBestStartingNode(const matrix::Vector2f &start_local,
		bool &destination_directly_reachable) const
{
	destination_directly_reachable = false;
	float best_cost = INFINITY;
	int best_index = -1;

	for (int node_idx = 0; node_idx < _polygons.numNodes(); node_idx++) {
		if (!_polygons.edgeVisible(start_local, node_idx)) {
			continue;
		}

		if (node_idx == _polygons.destIndex()) {
			destination_directly_reachable = true;
			continue;
		}

		const float cost = (start_local - _polygons.node(node_idx)).norm() + _best_distance[node_idx];

		if (cost < best_cost) {
			best_cost = cost;
			best_index = node_idx;
		}
	}

	return best_index;
}

int GeofenceAvoidancePlanner::updateStartAndFillPath(matrix::Vector2d start)
{
	// Populate _path so consumers can blindly follow it:
	//  - append the saved valid start (anchor) if the current `start` is outside the fence
	//  - then fill the RTL return path by walking the DAG produced by the dijkstra solver

	// Could also avoid storing _path entirely and build it ad-hoc from _next_node_buffer.
	// Pro: less state, easier implementation of replanning on fence change during RTL
	// Con: introduces a lot of new edge cases (safe anchoring / replanning conflict somewhat)

	int path_index = 0;  // points to the first empty slot, increment after writing

	if (!_polygons_healthy || !_destination_healthy) {
		_path_length = 0;
		_path_cursor = 0;
		// Without any fence there is nothing to avoid, so flying directly is not a fallback.
		_straight_line_fallback = (_status != Status::NoFence);
		return 0;
	}

	perf_begin(_lookup_path_perf);

	MapProjection ref{_reference(0), _reference(1)};
	matrix::Vector2f start_local;
	ref.project(start(0), start(1), start_local(0), start_local(1));

	// Default: plan from given start
	bool direct_path_feasible = false;
	int best_starting_index = findBestStartingNode(start_local, direct_path_feasible);
	const bool path_feasible = best_starting_index >= 0 || direct_path_feasible;

	// Save for future fallback
	if (path_feasible) {
		_saved_valid_start = start;
	}

	// If planning failed, use previous fallback
	if (!path_feasible && _saved_valid_start.isAllFinite()) {
		matrix::Vector2f fallback_local;
		ref.project(_saved_valid_start(0), _saved_valid_start(1), fallback_local(0), fallback_local(1));
		const int fallback_best = findBestStartingNode(fallback_local, direct_path_feasible);

		if (fallback_best >= 0 || direct_path_feasible) {
			start_local = fallback_local;
			best_starting_index = fallback_best;
		}
	}

	// Start with the last valid start if outside the fence and a path exists.
	// EXCEPT when no path was found (no point flying to an out-of-fence position).
	const bool start_with_anchor = !path_feasible && (best_starting_index >= 0 || direct_path_feasible);

	if (start_with_anchor) {
		ref.reproject(start_local(0), start_local(1), _path[path_index](0), _path[path_index](1));
		path_index++;
	}

	if (!direct_path_feasible && best_starting_index >= 0) {
		int node = best_starting_index;

		for (int idx = 0; idx < _polygons.numNodes(); idx++) {
			const int next = _next_node_buffer[node];
			const matrix::Vector2f p = _polygons.node(node);
			ref.reproject(p(0), p(1), _path[path_index](0), _path[path_index](1));
			path_index++;

			if (next == _polygons.destIndex() || next < 0) {
				break;
			}

			node = next;
		}
	}

	_path_length = path_index;
	_path_cursor = 0;

	// No path from current position or saved anchor, and destination not
	// directly reachable from either. Fall back to flying directly
	_straight_line_fallback = (_path_length == 0) && !direct_path_feasible;

	perf_end(_lookup_path_perf);

	return path_index;
}
