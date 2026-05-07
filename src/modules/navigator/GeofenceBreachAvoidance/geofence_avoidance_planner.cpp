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


	dijkstra::solveBackward(_num_nodes, _num_nodes - 1, _distances, true, _best_distance,
				_next_node_buffer, _visited_buffer);


	perf_end(_plan_path_perf);
	return;
}

bool GeofenceAvoidancePlanner::update_vertices(GeofenceInterface &geofence, float margin)
{
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

	update_distances_between_vertices(geofence);

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

	int node_index = 0;

	for (int poly_index = 0; poly_index < num_polygons; poly_index++) {

		PolygonInfo info = geofence.getPolygonInfoByIndex(poly_index);

		if (info.fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION || info.fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION) {

			matrix::Vector2f local_in[info.vertex_count];
			matrix::Vector2f local_out[info.vertex_count];

			for (int vertex_idx = 0; vertex_idx < info.vertex_count; vertex_idx++) {
				local_in[vertex_idx] = get_vertex_local_position(poly_index, vertex_idx, geofence, _reference);
			}

			const bool should_expand = (info.fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION);

			if (!geofence_utils::expandOrShrinkPolygon(local_in, info.vertex_count, margin, should_expand,
					local_out)) {
				return false;
			}

			for (int vertex_idx = 0; vertex_idx < info.vertex_count; vertex_idx++) {
				_positions[node_index] = local_out[vertex_idx];
				node_index++;
			}

		} else if (info.fence_type == NAV_CMD_FENCE_CIRCLE_INCLUSION || info.fence_type == NAV_CMD_FENCE_CIRCLE_EXCLUSION) {
			const matrix::Vector2f center = get_vertex_local_position(poly_index, 0, geofence, _reference);
			const float delta_angle = 2.f * M_PI_F / kCircleApproxVertices;

			float adjusted_radius;

			if (info.fence_type == NAV_CMD_FENCE_CIRCLE_EXCLUSION) {
				adjusted_radius = info.circle_radius / cosf(M_PI_F / kCircleApproxVertices) + margin;

			} else {
				adjusted_radius = info.circle_radius - margin;

				if (adjusted_radius <= 0.f) {
					return false;
				}
			}

			for (int i = 0; i < kCircleApproxVertices; i++) {
				const float angle = i * delta_angle;
				_positions[node_index] = center + matrix::Vector2f{adjusted_radius * cosf(angle), adjusted_radius * sinf(angle)};
				node_index++;
			}
		}
	}

	// node_index equals the polygon vertex count; the destination is reserved one slot beyond
	_num_vertices = node_index;
	_num_nodes = node_index + 1;	// +1 for the destination

	return true;
}

void GeofenceAvoidancePlanner::update_distances_between_vertices(GeofenceInterface &geofence)
{
	perf_begin(_setup_distances_perf);

	// loop through all possible vertex to vertex combinations and store the distance between them
	for (int i = 0; i < _num_vertices; i++) {
		for (int j = i + 1; j < _num_vertices; j++) {
			const size_t idx = geofence_utils::symmetricPairIndex(i, j, _num_nodes);

			const bool clear = !geofence.checkIfLineViolatesAnyFence(_positions[i],
					   _positions[j], _reference);

			if (clear) {
				_distances[idx] = (_positions[i] - _positions[j]).norm();

			} else {
				_distances[idx] = INFINITY;
			}
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

	const int dest_idx = _num_nodes - 1;

	if (_destination_healthy && (dest_local - _positions[dest_idx]).norm() < FLT_EPSILON) {
		// no change in destination, skip the update
		return;
	}

	perf_begin(_update_destination_perf);
	_positions[dest_idx] = dest_local;

	// distances from each vertex to destination
	for (int graph_idx = 0; graph_idx < _num_vertices; graph_idx++) {
		const size_t dist_idx = geofence_utils::symmetricPairIndex(graph_idx, dest_idx, _num_nodes);

		const bool clear = !geofence.checkIfLineViolatesAnyFence(_positions[graph_idx],
				   _positions[dest_idx], _reference);

		if (clear) {
			_distances[dist_idx] = (_positions[graph_idx] - _positions[dest_idx]).norm();

		} else {
			_distances[dist_idx] = INFINITY;
		}
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

int GeofenceAvoidancePlanner::set_start_and_plan_path_to_destination(matrix::Vector2d start, GeofenceInterface &geofence,
		bool start_is_current_position)
{
	_start_is_current_position = start_is_current_position;

	if (!_polygons_healthy || !_destination_healthy) {
		return 0;
	}

	MapProjection ref{_reference(0), _reference(1)};
	ref.project(start(0), start(1), _start_local(0), _start_local(1));

	float best_cost = INFINITY;

	_best_starting_index = -1;

	for (int node_idx = 0; node_idx < _num_nodes; node_idx++) {
		// check if node is reachable from this position
		if (!geofence.checkIfLineViolatesAnyFence(_start_local, _positions[node_idx], _reference)) {

			if (node_idx == _num_nodes - 1) {
				// destination is directly reachable from start: no detour planning needed.
				// If the vehicle is already at the start, no waypoints at all are needed.
				// Otherwise we still need the anchor (point 0) so the vehicle flies back into the fence first.
				_best_starting_index = -1;
				_num_path_points = 0;
				return start_is_current_position ? 0 : 1;
			}

			const float cost = (_start_local - _positions[node_idx]).norm() + _best_distance[node_idx];

			if (cost < best_cost) {
				best_cost = cost;
				_best_starting_index = node_idx;
			}
		}
	}

	int current_index = _best_starting_index;
	_num_path_points = 0;

	if (current_index < 0) {
		// no reachable path to destination
		return 0;
	}

	for (int idx = 0; idx < _num_nodes; idx++) {
		int next = _next_node_buffer[current_index];
		_num_path_points++;

		if (next == _num_nodes - 1 || next < 0) {
			break;
		}

		current_index = next;
	}

	// When the vehicle is already at the start, the start itself is not a waypoint to fly to.
	// Otherwise the start is the anchor (point 0) the vehicle has to fly back to first.
	return start_is_current_position ? _num_path_points : _num_path_points + 1;
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

	if (next < 0 || next >= _num_nodes) {
		return nan_point;
	}

	matrix::Vector2d global;
	MapProjection ref{_reference(0), _reference(1)};
	ref.reproject(_positions[next](0), _positions[next](1), global(0), global(1));
	return global;
}
