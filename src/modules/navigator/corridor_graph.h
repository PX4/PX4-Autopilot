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
 * @file corridor_graph.h
 * Flight corridor graph navigator plan component.
 *
 * Loads a directed weighted graph of pre-approved flight corridors from
 * dataman and provides shortest-path queries via Dijkstra.
 */

#pragma once

#include <float.h>

#include <dataman_client/DatamanClient.hpp>
#include <lib/geo/geo.h>
#include <px4_platform_common/defines.h>

#include "navigation.h"

class Navigator;

class CorridorGraph
{
public:
	CorridorGraph(Navigator *navigator);
	CorridorGraph(const CorridorGraph &) = delete;
	CorridorGraph &operator=(const CorridorGraph &) = delete;
	~CorridorGraph();

	/**
	 * @brief Background work loop — call from the navigator main loop every iteration.
	 */
	void run();

	/**
	 * @brief Trigger a reload from dataman on the next run() call.
	 *
	 * Call this whenever corridor_graph_id changes in the mission topic.
	 */
	void updateGraph();

	/**
	 * @brief Find the shortest corridor path from cur to home via Dijkstra.
	 *
	 * The nearest corridor node to cur_pos is the start; nearest to home_pos
	 * is the goal.  The output array contains the full ordered sequence of
	 * corridor nodes to visit (start node first, goal node last).
	 *
	 * Edge cost = horizontal distance + static_cost.
	 *
	 * @param waypoints_out  Output array of nodes to visit in order.
	 * @param num_waypoints  Number of entries written to waypoints_out.
	 * @param max_waypoints  Capacity of waypoints_out.
	 * @return true if a path was found; false if not loaded, empty, or no path.
	 */
	bool findPath(double cur_lat, double cur_lon, float cur_alt,
		      double home_lat, double home_lon, float home_alt,
		      mission_corridor_node_s *waypoints_out, uint8_t &num_waypoints,
		      uint8_t max_waypoints) const;

	bool isLoaded() const { return _graph_loaded; }
	bool isEmpty() const { return _num_nodes == 0 || _num_edges == 0; }

	uint16_t numNodes() const { return _num_nodes; }
	uint16_t numEdges() const { return _num_edges; }

	mission_corridor_node_s node(uint16_t i) const
	{
		return (_nodes && i < _num_nodes) ? _nodes[i] : mission_corridor_node_s{};
	}

private:

	enum class DatamanState {
		UpdateRequestWait,
		ReadNodeState,
		ReadNodeStateWait,
		ReadEdgeState,
		ReadEdgeStateWait,
		LoadNodes,
		LoadEdges,
		Error
	};

	void _loadNodes();
	void _loadEdges();

	Navigator *_navigator{nullptr};

	mission_stats_entry_s _node_stats{};
	mission_stats_entry_s _edge_stats{};

	DatamanCache  _node_cache{"corridor_node_dm_miss", 0};
	DatamanCache  _edge_cache{"corridor_edge_dm_miss", 0};
	DatamanClient &_dataman_client{_node_cache.client()};

	DatamanState _dataman_state{DatamanState::UpdateRequestWait};
	DatamanState _error_state{DatamanState::UpdateRequestWait};

	mission_corridor_node_s *_nodes{nullptr};
	mission_corridor_edge_s *_edges{nullptr};
	uint16_t _num_nodes{0};
	uint16_t _num_edges{0};

	uint32_t _loaded_graph_id{0};
	bool _graph_loaded{false};
	bool _update_requested{false};
};
