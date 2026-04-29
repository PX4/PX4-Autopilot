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

GeofenceAvoidancePlanner::~GeofenceAvoidancePlanner()
{
	perf_free(_setup_perf);
	perf_free(_setup_distances_perf);
	perf_free(_update_start_perf);
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

const PlannedPath &GeofenceAvoidancePlanner::planPath()
{
	// The navigator task stack is only ~2 KB, so we keep the result as a member
	// and return it by reference to avoid overflowing the stack on RTL entry.
	_planned_path.num_points = 0;
	_planned_path.current_index = 0;

	if (!_polygons_healthy || !_start_healthy || !_destination_healthy) {
		// we are not in a state where we can reliably plan a path, return an empty one
		return _planned_path;
	}

	perf_begin(_plan_path_perf);

	reset_graph_state();

	// reset to the start location
	int graph_index = 0;
	int last_graph_index = 0;

	while (true) {
		// get visible vertices from current node

		VisibleVertices visible_vertices = {};
		int visible_count = 0;

		for (int i = 0; i < _num_nodes; i++) {

			if (i == graph_index) {
				continue;
			}

			const float distance = _distances[geofence_utils::symmetricPairIndex(graph_index, i, _num_nodes)];

			if (distance < INFINITY) {
				visible_vertices.items[visible_count].index = i;
				visible_vertices.items[visible_count].distance = distance;
				visible_count++;
			}
		}

		visible_vertices.count = visible_count;

		// update the best distances to all neighbouring nodes
		for (int i = 0; i < visible_vertices.count; i++) {
			const int idx = visible_vertices.items[i].index;
			const float distance = visible_vertices.items[i].distance + _graph_nodes[graph_index].best_distance;

			if (distance < _graph_nodes[idx].best_distance) {
				_graph_nodes[idx].best_distance = distance;
				_graph_nodes[idx].previous_index = graph_index;
			}
		}

		// find the unvisited node with the smallest best distance
		last_graph_index = graph_index;
		float min_dist = INFINITY;

		for (int i = 0; i < _num_nodes; i++) {
			if (!_graph_nodes[i].visited && _graph_nodes[i].best_distance < min_dist) {
				min_dist = _graph_nodes[i].best_distance;
				graph_index = i;
			}
		}

		if (last_graph_index == graph_index) {
			// we are stuck, destination does not seem to be reachable
			perf_end(_plan_path_perf);
			return _planned_path;
		}

		if (_graph_nodes[graph_index].type == Node::DESTINATION) {
			break;
		}

		_graph_nodes[graph_index].visited = true;
	}

	// backtrack from destination to start and count the nodes along the way
	int count = 0;
	int idx = graph_index;

	while (idx != 0) {
		if (idx < 0) {
			// can happen if the destination is not reachable
			perf_end(_plan_path_perf);
			return _planned_path;
		}

		count++;
		idx = _graph_nodes[idx].previous_index;
	}

	_planned_path.num_points = count;

	// Walk backwards and fill points in reverse, converting local to lat/lon.
	// The last iteration lands on the start node, which becomes points[0].
	MapProjection ref{_reference(0), _reference(1)};
	idx = _graph_nodes[graph_index].previous_index; // skip destination

	for (int i = _planned_path.num_points - 1; i >= 0; i--) {
		if (idx < 0) {
			// this should never happen, but just in case, we return an empty path
			_planned_path.num_points = 0;
			break;
		}

		ref.reproject(_graph_nodes[idx].position(0), _graph_nodes[idx].position(1),
			      _planned_path.points[i](0), _planned_path.points[i](1));
		idx = _graph_nodes[idx].previous_index;
	}

	perf_end(_plan_path_perf);
	return _planned_path;
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

	if (num_vertices == 0 || num_vertices > kMaxNodes - 2) { // -2 to reserve start and destination slots
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
	perf_end(_setup_perf);
	return true;
}

bool GeofenceAvoidancePlanner::update_graph_nodes_without_start_and_destination(
	GeofenceInterface &geofence, float margin)
{
	// local frame is anchored at _reference (set by update_vertices); this can be computed
	// once up-front; start and destination nodes are filled in later when they are known
	const int num_polygons = geofence.getNumPolygons();

	int node_index = 1; // reserve index 0 for the start

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
				_graph_nodes[node_index].type = Node::Type::VERTEX;
				_graph_nodes[node_index].position = local_out[vertex_idx];
				_graph_nodes[node_index].best_distance = FLT_MAX;
				_graph_nodes[node_index].previous_index = -1;
				_graph_nodes[node_index].visited = false;
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
				_graph_nodes[node_index].type = Node::Type::VERTEX;
				_graph_nodes[node_index].position = center + matrix::Vector2f{adjusted_radius * cosf(angle), adjusted_radius * sinf(angle)};
				_graph_nodes[node_index].best_distance = FLT_MAX;
				_graph_nodes[node_index].previous_index = -1;
				_graph_nodes[node_index].visited = false;
				node_index++;
			}
		}
	}

	// node_index now points at the reserved destination slot; total count includes it
	_num_vertices = node_index - 1;
	_num_nodes = node_index + 1;	// +1 for the destination

	return true;
}

void GeofenceAvoidancePlanner::update_distances_between_vertices(GeofenceInterface &geofence)
{
	perf_begin(_setup_distances_perf);
	// vertices occupy indices 1 .. _num_vertices; index 0 and the last slot hold the
	// start and destination, which are handled by update_start / update_destination
	const int last_vertex = _num_vertices;

	// loop through all possible vertex to vertex combinations and store the distance between them
	for (int i = 1; i <= last_vertex; i++) {
		for (int j = i + 1; j <= last_vertex; j++) {
			const size_t idx = geofence_utils::symmetricPairIndex(i, j, _num_nodes);

			const bool clear = !geofence.checkIfLineViolatesAnyFence(_graph_nodes[i].position,
					   _graph_nodes[j].position, _reference);

			if (clear) {
				_distances[idx] = (_graph_nodes[i].position - _graph_nodes[j].position).norm();

			} else {
				_distances[idx] = INFINITY;
			}
		}
	}

	perf_end(_setup_distances_perf);
}

void GeofenceAvoidancePlanner::update_start(const matrix::Vector2d &start, GeofenceInterface &geofence)
{
	if (!start.isAllFinite() || !lat_lon_within_bounds(start) || !_polygons_healthy) {
		_start_healthy = false;
		return;
	}

	perf_begin(_update_start_perf);

	MapProjection ref{_reference(0), _reference(1)};
	matrix::Vector2f start_local;
	ref.project(start(0), start(1), start_local(0), start_local(1));

	_graph_nodes[0].position = start_local;

	// distances from start to each vertex
	for (int graph_idx = 1; graph_idx <= _num_vertices; graph_idx++) {
		const size_t dist_idx = geofence_utils::symmetricPairIndex(0, graph_idx, _num_nodes);

		const bool clear = !geofence.checkIfLineViolatesAnyFence(_graph_nodes[0].position,
				   _graph_nodes[graph_idx].position, _reference);

		if (clear) {
			_distances[dist_idx] = (_graph_nodes[0].position - _graph_nodes[graph_idx].position).norm();

		} else {
			_distances[dist_idx] = INFINITY;
		}
	}

	// refresh start <-> destination edge if destination is already known
	if (_destination_healthy) {
		const int dest_idx = _num_nodes - 1;
		const size_t dist_idx = geofence_utils::symmetricPairIndex(0, dest_idx, _num_nodes);

		const bool clear = !geofence.checkIfLineViolatesAnyFence(_graph_nodes[0].position,
				   _graph_nodes[dest_idx].position, _reference);

		if (clear) {
			_distances[dist_idx] = (_graph_nodes[0].position - _graph_nodes[dest_idx].position).norm();

		} else {
			_distances[dist_idx] = INFINITY;
		}
	}

	_start_healthy = true;
	perf_end(_update_start_perf);
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

	if (_destination_healthy && (dest_local - _graph_nodes[dest_idx].position).norm() < FLT_EPSILON) {
		// no change in destination, skip the update
		return;
	}

	perf_begin(_update_destination_perf);
	_graph_nodes[dest_idx].position = dest_local;

	// distances from each vertex to destination
	for (int graph_idx = 1; graph_idx <= _num_vertices; graph_idx++) {
		const size_t dist_idx = geofence_utils::symmetricPairIndex(graph_idx, dest_idx, _num_nodes);

		const bool clear = !geofence.checkIfLineViolatesAnyFence(_graph_nodes[graph_idx].position,
				   _graph_nodes[dest_idx].position, _reference);

		if (clear) {
			_distances[dist_idx] = (_graph_nodes[graph_idx].position - _graph_nodes[dest_idx].position).norm();

		} else {
			_distances[dist_idx] = INFINITY;
		}
	}

	// refresh start <-> destination edge if start is already known
	if (_start_healthy) {
		const size_t dist_idx = geofence_utils::symmetricPairIndex(0, dest_idx, _num_nodes);

		const bool clear = !geofence.checkIfLineViolatesAnyFence(_graph_nodes[0].position,
				   _graph_nodes[dest_idx].position, _reference);

		if (clear) {
			_distances[dist_idx] = (_graph_nodes[0].position - _graph_nodes[dest_idx].position).norm();

		} else {
			_distances[dist_idx] = INFINITY;
		}
	}

	_destination_healthy = true;
	perf_end(_update_destination_perf);
}

bool GeofenceAvoidancePlanner::lat_lon_within_bounds(const matrix::Vector2<double> &lat_lon)
{
	if (lat_lon(0) > 90.0 || lat_lon(0) < -90.0 || lat_lon(1) > 180.0 || lat_lon(1) < -180.0) {
		return false;
	}

	return true;
}

void GeofenceAvoidancePlanner::reset_graph_state()
{

	_graph_nodes[0].best_distance = 0.0f;
	_graph_nodes[0].previous_index = 0;
	_graph_nodes[0].visited = true;

	for (int i = 1; i <= _num_vertices; i++) {
		_graph_nodes[i].best_distance = FLT_MAX;
		_graph_nodes[i].previous_index = -1;
		_graph_nodes[i].visited = false;
	}

	_graph_nodes[_num_nodes - 1].best_distance = FLT_MAX;
	_graph_nodes[_num_nodes - 1].previous_index = -1;
	_graph_nodes[_num_nodes - 1].visited = false;
	_graph_nodes[_num_nodes - 1].type = Node::Type::DESTINATION;
}
