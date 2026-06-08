/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
/**
 * @file corridor_graph.cpp
 */

#include "corridor_graph.h"
#include "navigator.h"

#include <inttypes.h>

#include <lib/geo/geo.h>
#include <px4_platform_common/events.h>
#include <systemlib/mavlink_log.h>

CorridorGraph::CorridorGraph(Navigator *navigator) :
	_navigator(navigator)
{
	if (_navigator != nullptr) {
		updateGraph();
	}
}

CorridorGraph::~CorridorGraph()
{
	delete[] _nodes;
	delete[] _edges;
}

void CorridorGraph::updateGraph()
{
	_update_requested = true;
}

void CorridorGraph::run()
{
	bool success;

	switch (_dataman_state) {

	case DatamanState::UpdateRequestWait:

		if (_update_requested) {
			_update_requested = false;
			_graph_loaded = false;
			_dataman_state = DatamanState::ReadNodeState;
		}

		break;

	case DatamanState::ReadNodeState:

		_dataman_state = DatamanState::ReadNodeStateWait;
		success = _dataman_client.readAsync(DM_KEY_CORRIDOR_NODES_STATE, 0,
						    reinterpret_cast<uint8_t *>(&_node_stats),
						    sizeof(mission_stats_entry_s));

		if (!success) {
			_error_state = DatamanState::ReadNodeState;
			_dataman_state = DatamanState::Error;
		}

		break;

	case DatamanState::ReadNodeStateWait:

		_dataman_client.update();

		if (_dataman_client.lastOperationCompleted(success)) {
			if (!success) {
				_error_state = DatamanState::ReadNodeStateWait;
				_dataman_state = DatamanState::Error;

			} else {
				_dataman_state = DatamanState::ReadEdgeState;
			}
		}

		break;

	case DatamanState::ReadEdgeState:
		_dataman_state = DatamanState::ReadEdgeStateWait;
		success = _dataman_client.readAsync(DM_KEY_CORRIDOR_EDGES_STATE, 0,
						    reinterpret_cast<uint8_t *>(&_edge_stats),
						    sizeof(mission_stats_entry_s));

		if (!success) {
			_error_state = DatamanState::ReadEdgeState;
			_dataman_state = DatamanState::Error;
		}

		break;

	case DatamanState::ReadEdgeStateWait:

		_dataman_client.update();

		if (_dataman_client.lastOperationCompleted(success)) {
			if (!success) {
				_error_state = DatamanState::ReadEdgeStateWait;
				_dataman_state = DatamanState::Error;

			} else if (_node_stats.opaque_id != 0 && _node_stats.opaque_id == _loaded_graph_id) {
				// Graph unchanged — already current
				_graph_loaded = true;
				_dataman_state = DatamanState::UpdateRequestWait;
				PX4_INFO("CorridorGraph: graph unchanged (id=%" PRIu32 "), skipping reload", _loaded_graph_id);

			} else if (_node_stats.num_items == 0) {
				// Empty graph: discard any previously loaded data
				delete[] _nodes; _nodes = nullptr; _num_nodes = 0;
				delete[] _edges; _edges = nullptr; _num_edges = 0;
				_loaded_graph_id = _node_stats.opaque_id;
				_graph_loaded = true;
				_dataman_state = DatamanState::UpdateRequestWait;
				PX4_INFO("CorridorGraph: cleared (0 nodes, 0 edges)");

			} else {
				// New graph available — queue async node loads
				_node_cache.invalidate();

				if (_node_cache.size() != (int)_node_stats.num_items) {
					_node_cache.resize(_node_stats.num_items);
				}

				dm_item_t node_key = static_cast<dm_item_t>(_node_stats.dataman_id);

				for (int i = 0; i < (int)_node_stats.num_items; i++) {
					_node_cache.load(node_key, i);
				}

				_dataman_state = DatamanState::LoadNodes;
			}
		}

		break;

	case DatamanState::LoadNodes:

		_node_cache.update();

		if (!_node_cache.isLoading()) {
			_loadNodes();

			if (_nodes == nullptr) {
				_error_state = DatamanState::LoadNodes;
				_dataman_state = DatamanState::Error;
				break;
			}

			// Queue async edge loads
			_edge_cache.invalidate();

			if (_edge_cache.size() != (int)_edge_stats.num_items) {
				_edge_cache.resize(_edge_stats.num_items);
			}

			dm_item_t edge_key = static_cast<dm_item_t>(_edge_stats.dataman_id);

			for (int i = 0; i < (int)_edge_stats.num_items; i++) {
				_edge_cache.load(edge_key, i);
			}

			_dataman_state = DatamanState::LoadEdges;
		}

		break;

	case DatamanState::LoadEdges:

		_edge_cache.update();

		if (!_edge_cache.isLoading()) {
			_loadEdges();

			if (_edges == nullptr) {
				_error_state = DatamanState::LoadEdges;
				_dataman_state = DatamanState::Error;
				break;
			}

			_loaded_graph_id = _node_stats.opaque_id;
			_graph_loaded = true;
			_dataman_state = DatamanState::UpdateRequestWait;

			PX4_INFO("CorridorGraph: loaded %" PRIu16 " nodes, %" PRIu16 " edges", _num_nodes, _num_edges);
		}

		break;

	case DatamanState::Error:
		PX4_ERR("CorridorGraph update failed! state: %" PRIu8, static_cast<uint8_t>(_error_state));
		_dataman_state = DatamanState::UpdateRequestWait;
		break;

	default:
		break;
	}

}

void CorridorGraph::_loadNodes()
{
	delete[] _nodes;
	_nodes = nullptr;
	_num_nodes = 0;

	const uint16_t count = _node_stats.num_items;

	if (count == 0) { return; }

	_nodes = new mission_corridor_node_s[count];

	if (!_nodes) {
		PX4_ERR("CorridorGraph: node alloc failed");
		return;
	}

	dm_item_t key = static_cast<dm_item_t>(_node_stats.dataman_id);

	for (uint16_t i = 0; i < count; i++) {
		if (!_node_cache.loadWait(key, i,
					  reinterpret_cast<uint8_t *>(&_nodes[i]),
					  sizeof(mission_corridor_node_s))) {
			PX4_ERR("CorridorGraph: loadWait failed for node %" PRIu16, i);
			delete[] _nodes;
			_nodes = nullptr;
			return;
		}
	}

	_num_nodes = count;
}

void CorridorGraph::_loadEdges()
{
	delete[] _edges;
	_edges = nullptr;
	_num_edges = 0;

	const uint16_t count = _edge_stats.num_items;

	if (count == 0) { return; }

	_edges = new mission_corridor_edge_s[count];

	if (!_edges) {
		PX4_ERR("CorridorGraph: edge alloc failed");
		return;
	}

	dm_item_t key = static_cast<dm_item_t>(_edge_stats.dataman_id);

	for (uint16_t i = 0; i < count; i++) {
		if (!_edge_cache.loadWait(key, i,
					  reinterpret_cast<uint8_t *>(&_edges[i]),
					  sizeof(mission_corridor_edge_s))) {
			PX4_ERR("CorridorGraph: loadWait failed for edge %" PRIu16, i);
			delete[] _edges;
			_edges = nullptr;
			return;
		}
	}

	_num_edges = count;
}

int CorridorGraph::_findClosestNode(double lat, double lon, float alt) const
{
	if (!isLoaded() || _num_nodes == 0) { return -1; }

	float min_dist = INFINITY;
	int closest_node = -1;

	for (int i = 0; i < _num_nodes; i++) {
		float h_dist = get_distance_to_next_waypoint(lat, lon, _nodes[i].lat, _nodes[i].lon);
		float v_dist = alt - _nodes[i].alt;
		float dist = sqrtf(h_dist * h_dist + v_dist * v_dist);

		if (dist < min_dist) {
			min_dist = dist;
			closest_node = i;
		}
	}

	return closest_node;
}

float CorridorGraph::_edgeCost(uint16_t from_idx, uint16_t to_idx, float static_cost) const
{
	// An external static cost overrides distance; the two are never combined.
	if (static_cost > 0.0f) {
		return static_cost;
	}

	// No static cost provided: fall back to the 3D Euclidean distance between nodes.
	const float h_dist = get_distance_to_next_waypoint(_nodes[from_idx].lat, _nodes[from_idx].lon,
			     _nodes[to_idx].lat, _nodes[to_idx].lon);
	const float v_dist = _nodes[from_idx].alt - _nodes[to_idx].alt;

	return sqrtf(h_dist * h_dist + v_dist * v_dist);
}


bool CorridorGraph::findPath(double cur_lat, double cur_lon, float cur_alt,
			     double home_lat, double home_lon, float home_alt,
			     mission_corridor_node_s *waypoints_out, uint8_t &num_waypoints,
			     uint8_t max_waypoints) const
{
	// TODO: implement RTL path search using the dijkstra library.
	//
	// Steps:
	//   1. Call _findClosestNode(cur_lat, cur_lon, cur_alt) to get the start node index.
	//   2. Identify landing node indices (nodes flagged as acceptable landing points).
	//   3. Call dijkstra::solveBackward(_num_nodes, start, _cost, false, ...) with
	//      start as the goal so that best_cost[landing_k] gives the path cost from
	//      each landing node back to the drone's position.
	//   4. Pick the landing node with the lowest best_cost[].
	//   5. Walk next_node[] from that landing node toward start, collecting indices.
	//   6. Reverse the collected indices and copy the corresponding _nodes[] entries
	//      into waypoints_out[], capped at max_waypoints.
	//   7. Set num_waypoints and return true. Return false if no reachable landing node.
	(void)cur_lat; (void)cur_lon; (void)cur_alt;
	(void)home_lat; (void)home_lon; (void)home_alt;
	(void)waypoints_out; (void)max_waypoints;
	num_waypoints = 0;
	return false;
}
