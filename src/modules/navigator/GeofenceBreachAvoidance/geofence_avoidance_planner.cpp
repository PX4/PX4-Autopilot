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

static matrix::Vector2f get_vertex_local_position(int poly_index, int vertex_idx,
		GeofenceInterface *geofence,
		const matrix::Vector2<double> &reference)
{
	matrix::Vector2<double> latlon = geofence->getPolygonVertexByIndex(poly_index, vertex_idx);
	MapProjection ref{reference(0), reference(1)};
	matrix::Vector2f local;
	ref.project(latlon(0), latlon(1), local(0), local(1));
	return local;
}

PlannedPath GeofenceAvoidancePlanner::planPath(const matrix::Vector2<double> &start,
		const matrix::Vector2<double> &destination,
		GeofenceInterface *geofence,
		float margin)
{
	PlannedPath path;

	margin = math::max(margin, 1.f); // margin should be non-zero as otherwhise the algorithm breaks down

	if (!start.isAllFinite() || !destination.isAllFinite()) {
		path.num_points = 0;
		return path;
	}

	if (!lat_lon_within_bounds(start) || !lat_lon_within_bounds(destination)) {
		path.num_points = 0;
		return path;
	}

	if ((start - destination).norm() < 1e-10) {
		path.num_points = 0;
		return path;
	}

	const int num_polygons = geofence->getNumPolygons();
	int num_vertices_total = 0;

	for (int poly_index = 0; poly_index < num_polygons; poly_index++) {
		const PolygonInfo info = geofence->getPolygonInfoByIndex(poly_index);

		if (info.fence_type == NAV_CMD_FENCE_CIRCLE_INCLUSION || info.fence_type == NAV_CMD_FENCE_CIRCLE_EXCLUSION) {
			num_vertices_total += kCircleApproxVertices;

		} else {
			num_vertices_total += info.vertex_count;
		}
	}

	if (num_vertices_total + 2 > kMaxNodes) {
		path.num_points = 0;
		return path;
	}

	if (!calculate_graph_nodes(start, destination, geofence, margin)) {
		path.num_points = 0;
		return path;
	}

	// reset to the start location
	int graph_index = 0;
	int last_graph_index = 0;

	while (true) {
		// get visible vertices from current node

		VisibleVertices visible_vertices = {};
		int visible_count = 0;

		for (int i = 0; i < num_vertices_total + 2; i++) { // +2 accounts for start and destination node

			if (i == graph_index) {
				continue;
			}

			// check if the line from our current node to any other node is clear
			const bool clear = !geofence->checkIfLineViolatesAnyFence(_graph_nodes[graph_index].position,
					   _graph_nodes[i].position, _reference);

			if (clear) {
				visible_vertices.items[visible_count].index = i;
				visible_vertices.items[visible_count].distance =
					(_graph_nodes[graph_index].position - _graph_nodes[i].position).norm();
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
		float min_dist = INFINITY;

		for (int i = 0; i < num_vertices_total + 2; i++) {	// +2 accounts for start and destination
			if (!_graph_nodes[i].visited && _graph_nodes[i].best_distance < min_dist) {
				min_dist = _graph_nodes[i].best_distance;
				graph_index = i;
			}
		}

		if (last_graph_index == graph_index) {
			// we are stuck, destination does not seem to be reachable
			path.num_points = 0;
			return path;
		}

		if (_graph_nodes[graph_index].type == Node::DESTINATION) {
			break;
		}

		_graph_nodes[graph_index].visited = true;
	}

	// figure out the path by backtracking from destination to start
	int count = 0;
	int idx = graph_index;

	while (idx != 0) {
		if (idx < 0) {
			// can happen if the destination is not reachable
			path.num_points = 0;
			return path;
		}

		count++;
		idx = _graph_nodes[idx].previous_index;
	}


	path.num_points = count - 1;

	// Walk backwards again and fill points in reverse, converting local to lat/lon
	MapProjection ref{_reference(0), _reference(1)};
	idx = _graph_nodes[graph_index].previous_index; // skip destination

	for (int i = path.num_points - 1; i >= 0; i--) {
		if (idx < 0) {
			// this should never happen, but just in case, we return an empty path
			path.num_points = 0;
			break;
		}

		ref.reproject(_graph_nodes[idx].position(0), _graph_nodes[idx].position(1),
			      path.points[i](0), path.points[i](1));
		idx = _graph_nodes[idx].previous_index;
	}

	return path;
}

bool GeofenceAvoidancePlanner::calculate_graph_nodes(const matrix::Vector2<double> &start,
		const matrix::Vector2<double> &destination,
		GeofenceInterface *geofence,
		float margin)
{
	_reference = start;
	MapProjection ref{start(0), start(1)};

	const int num_polygons = geofence->getNumPolygons();

	int node_index = 0;
	// Node 0: START at local origin
	_graph_nodes[node_index].type = Node::Type::START;
	_graph_nodes[node_index].position = {0.f, 0.f};
	_graph_nodes[node_index].best_distance = 0.0f;
	_graph_nodes[node_index].previous_index = 0;
	_graph_nodes[node_index].visited = true;

	node_index++;

	for (int poly_index = 0; poly_index < num_polygons; poly_index++) {
		PolygonInfo info = geofence->getPolygonInfoByIndex(poly_index);


		if (info.fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION || info.fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION) {

			// Project vertices to local frame
			matrix::Vector2f local_in[info.vertex_count];
			matrix::Vector2f local_out[info.vertex_count];

			for (int vertex_idx = 0; vertex_idx < info.vertex_count; vertex_idx++) {
				local_in[vertex_idx] = get_vertex_local_position(poly_index, vertex_idx, geofence, start);
			}

			// In order to avoid moving too close to the edges of the polygons, we either expand (inclusion polygon)
			// or shrink (exclusion) the polygon by a margin and use the resulting vertices as graph nodes.
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
			const matrix::Vector2f center = get_vertex_local_position(poly_index, 0, geofence, start);
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

	// Last node: DESTINATION in local frame
	matrix::Vector2f dest_local;
	ref.project(destination(0), destination(1), dest_local(0), dest_local(1));

	_graph_nodes[node_index].type = Node::Type::DESTINATION;
	_graph_nodes[node_index].position = dest_local;
	_graph_nodes[node_index].best_distance = FLT_MAX;
	_graph_nodes[node_index].previous_index = -1;
	_graph_nodes[node_index].visited = false;

	return true;
}

bool GeofenceAvoidancePlanner::lat_lon_within_bounds(const matrix::Vector2<double> &lat_lon)
{
	if (lat_lon(0) > 90.0 || lat_lon(0) < -90.0 || lat_lon(1) > 180.0 || lat_lon(1) < -180.0) {
		return false;
	}

	return true;
}
