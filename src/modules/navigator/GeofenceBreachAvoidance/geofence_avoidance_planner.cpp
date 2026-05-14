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
	perf_free(_setup_perf);
	perf_free(_setup_distances_perf);
	perf_free(_update_destination_perf);
	perf_free(_plan_path_perf);
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
		// we are not in a state where we can reliably plan a path, return an empty one
		return;
	}

	perf_begin(_plan_path_perf);

	dijkstra::solveBackward(_polygons.numNodes(), _polygons.destIndex(), _distances, true, _best_distance,
				_next_node_buffer, _visited_buffer);

	perf_end(_plan_path_perf);
	return;
}

bool GeofenceAvoidancePlanner::update_vertices(GeofenceInterface &geofence, float margin)
{
	// Polygons are about to change; any previously latched fallback start may no longer be valid.
	_saved_valid_start = matrix::Vector2<double> {(double)NAN, (double)NAN};

	_polygons_healthy = true;
	margin = math::max(margin, 1.f); // margin should be non-zero otherwise polygon expansion breaks down

	const int num_polygons = geofence.getNumPolygons();
	int num_vertices{0};

	for (int poly_idx = 0; poly_idx < num_polygons; poly_idx++) {
		PolygonInfo info = geofence.getPolygonInfoByIndex(poly_idx);

		if (info.fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION || info.fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION) {
			num_vertices += info.vertex_count;

		} else if (info.fence_type == NAV_CMD_FENCE_CIRCLE_INCLUSION || info.fence_type == NAV_CMD_FENCE_CIRCLE_EXCLUSION) {
			num_vertices += kCircleApproxVertices;
		}
	}

	if (num_vertices == 0 || num_vertices > kMaxNodes - 1) { // -1 to reserve the destination slot
		_polygons_healthy = false;
		return false;
	}

	perf_begin(_setup_perf);

	_reference = geofence.getPolygonVertexByIndex(0, 0);

	if (!update_graph_nodes_without_start_and_destination(geofence, margin)) {
		_polygons_healthy = false;
		perf_cancel(_setup_perf);
		return false;
	}

	update_distances_between_vertices();

	// invalidate destination to force a refresh on the next update_destination()
	_destination_healthy = false;

	perf_end(_setup_perf);

	planPath();
	return true;
}

bool GeofenceAvoidancePlanner::update_graph_nodes_without_start_and_destination(
	GeofenceInterface &geofence, float margin)
{
	const int num_polygons = geofence.getNumPolygons();

	_polygons.reset();

	for (int poly_index = 0; poly_index < num_polygons; poly_index++) {

		PolygonInfo info = geofence.getPolygonInfoByIndex(poly_index);

		if (info.fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION || info.fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION) {

			matrix::Vector2f local_in[info.vertex_count];

			for (int vertex_idx = 0; vertex_idx < info.vertex_count; vertex_idx++) {
				local_in[vertex_idx] = get_vertex_local_position(poly_index, vertex_idx, geofence, _reference);
			}

			const bool is_inclusion = (info.fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);

			if (!_polygons.addPolygon(local_in, info.vertex_count, is_inclusion, margin)) {
				return false;
			}

		} else if (info.fence_type == NAV_CMD_FENCE_CIRCLE_INCLUSION || info.fence_type == NAV_CMD_FENCE_CIRCLE_EXCLUSION) {

			const matrix::Vector2f center = get_vertex_local_position(poly_index, 0, geofence, _reference);

			const bool is_inclusion = (info.fence_type == NAV_CMD_FENCE_CIRCLE_INCLUSION);

			if (!_polygons.addApproxCircle(center, info.circle_radius, margin, kCircleApproxVertices, is_inclusion)) {
				return false;
			}
		}
	}

	return true;
}

void GeofenceAvoidancePlanner::update_distances_between_vertices()
{
	perf_begin(_setup_distances_perf);

	// Polygon vertices occupy indices 1..numNodes()-1; destination is at 0.
	for (int i = 1; i < _polygons.numNodes(); i++) {
		for (int j = i + 1; j < _polygons.numNodes(); j++) {
			const size_t idx = geofence_utils::symmetricPairIndex(i, j, _polygons.numNodes());
			_distances[idx] = _polygons.edgeCost(i, j);
		}
	}

	perf_end(_setup_distances_perf);
}

void GeofenceAvoidancePlanner::update_destination(const matrix::Vector2d &destination, GeofenceInterface &geofence)
{
	if (!destination.isAllFinite() || !lat_lon_within_bounds(destination) || !_polygons_healthy) {
		_destination_healthy = false;
		return;
	}

	MapProjection ref{_reference(0), _reference(1)};
	matrix::Vector2f dest_local;
	ref.project(destination(0), destination(1), dest_local(0), dest_local(1));

	if (_destination_healthy && (dest_local - _polygons.getDestination()).norm() < FLT_EPSILON) {
		return;
	}

	perf_begin(_update_destination_perf);
	_polygons.setDestination(dest_local);

	for (int graph_idx = 1; graph_idx < _polygons.numNodes(); graph_idx++) {
		const size_t dist_idx = geofence_utils::symmetricPairIndex(graph_idx, _polygons.destIndex(), _polygons.numNodes());
		_distances[dist_idx] = _polygons.edgeCost(graph_idx, _polygons.destIndex());
	}

	_destination_healthy = true;
	perf_end(_update_destination_perf);

	planPath();
}

bool GeofenceAvoidancePlanner::lat_lon_within_bounds(const matrix::Vector2<double> &lat_lon)
{
	if (lat_lon(0) > 90.0 || lat_lon(0) < -90.0 || lat_lon(1) > 180.0 || lat_lon(1) < -180.0) {
		return false;
	}

	return true;
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

void GeofenceAvoidancePlanner::save_position_if_no_fence_violation(const matrix::Vector2<double> &position)
{
	if (!_polygons_healthy || !position.isAllFinite()) {
		return;
	}

	matrix::Vector2f position_local;
	MapProjection ref{_reference(0), _reference(1)};
	ref.project(position(0), position(1), position_local(0), position_local(1));

	// "Doesn't violate any fence" (for planner purposes) = at least one polygon node is
	// edge-visible from this position. If the point were outside an inclusion polygon or
	// inside an exclusion polygon, every line from it to a polygon vertex would cross a fence.
	for (int node_idx = 0; node_idx < _polygons.numNodes(); node_idx++) {
		if (_polygons.edgeVisible(position_local, node_idx)) {
			_saved_valid_start = position;
			return;
		}
	}
}

int GeofenceAvoidancePlanner::set_start_and_plan_path_to_destination(matrix::Vector2d start, GeofenceInterface &geofence)
{
	if (!_polygons_healthy || !_destination_healthy) {
		return 0;
	}

	MapProjection ref{_reference(0), _reference(1)};
	ref.project(start(0), start(1), _start_local(0), _start_local(1));

	// Try the caller-provided position (typically the vehicle's current position) first.
	bool destination_directly_reachable = false;
	_best_starting_index = findBestStartingNode(_start_local, destination_directly_reachable);
	_start_is_current_position = (_best_starting_index >= 0) || destination_directly_reachable;

	// If the provided position has no visible node and the destination isn't directly reachable,
	// fall back to the most recently latched in-fence position (see save_position_if_no_fence_violation).
	if (!_start_is_current_position && _saved_valid_start.isAllFinite()) {
		matrix::Vector2f fallback_local;
		ref.project(_saved_valid_start(0), _saved_valid_start(1), fallback_local(0), fallback_local(1));
		const int fallback_best = findBestStartingNode(fallback_local, destination_directly_reachable);

		if (fallback_best >= 0 || destination_directly_reachable) {
			_start_local = fallback_local;
			_best_starting_index = fallback_best;
			// _start_is_current_position stays false: the anchor is not where the vehicle is.
		}
	}

	if (destination_directly_reachable) {
		// Destination is directly reachable from the chosen start: no detour planning needed.
		// If the vehicle is already at the start, no waypoints at all are needed.
		// Otherwise we still need the anchor (point 0) so the vehicle flies back into the fence first.
		_best_starting_index = -1;
		_num_path_points = 0;
		return _start_is_current_position ? 0 : 1;
	}

	int current_index = _best_starting_index;
	_num_path_points = 0;

	if (current_index < 0) {
		// no reachable path to destination
		return 0;
	}

	for (int idx = 0; idx < _polygons.numNodes(); idx++) {
		int next = _next_node_buffer[current_index];
		_num_path_points++;

		if (next == _polygons.destIndex() || next < 0) {
			break;
		}

		current_index = next;
	}

	// When the vehicle is already at the start, the start itself is not a waypoint to fly to.
	// Otherwise the start is the anchor (point 0) the vehicle has to fly back to first.
	// Use the member here: the fallback path may have flipped it from the caller's value.
	return _start_is_current_position ? _num_path_points : _num_path_points + 1;
}

matrix::Vector2d GeofenceAvoidancePlanner::get_point_at_index(int index) const
{
	const matrix::Vector2d nan_point{(double)NAN, (double)NAN};

	// When start_is_current_position == false, the start (anchor) occupies index 0 of the path
	// and intermediate vertices follow at index 1..N. When true, the start is omitted and the
	// intermediate vertices start at index 0.
	const int anchor_offset = _start_is_current_position ? 0 : 1;
	const int total_points = _num_path_points + anchor_offset;

	if (index < 0 || index >= total_points) {
		return nan_point;
	}

	if (anchor_offset == 1 && index == 0) {
		matrix::Vector2d start_global;
		MapProjection ref{_reference(0), _reference(1)};
		ref.reproject(_start_local(0), _start_local(1), start_global(0), start_global(1));
		return start_global;
	}

	if (_best_starting_index < 0) {
		return nan_point;
	}

	int next = _best_starting_index;

	for (int i = 0; i < index - anchor_offset; i++) {
		next = _next_node_buffer[next];
	}

	if (next < 0 || next >= _polygons.numNodes()) {
		return nan_point;
	}

	matrix::Vector2d global;
	MapProjection ref{_reference(0), _reference(1)};
	const matrix::Vector2f p = _polygons.node(next);
	ref.reproject(p(0), p(1), global(0), global(1));
	return global;
}
