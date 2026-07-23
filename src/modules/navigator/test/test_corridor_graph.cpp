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

/**
 * @file test_corridor_graph.cpp
 *
 * Unit tests for CorridorGraph::findPath(): nest-first / rally-fallback goal
 * selection and directed-edge correctness.
 */

#include <gtest/gtest.h>

#include "corridor_graph.h"
#include "rtl.h"

#include <dataman_client/DatamanClient.hpp>
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>

#include <cmath>
#include <unistd.h>
#include <vector>

extern "C" int dataman_main(int argc, char *argv[]);

namespace
{

class CorridorGraphDatamanRuntime
{
public:
	CorridorGraphDatamanRuntime()
	{
		param_control_autosave(false);
		px4::WorkQueueManagerStart();

		char name[] = "dataman";
		char start[] = "start";
		char ram[] = "-r";
		char *argv[] = {name, start, ram};
		dataman_main(3, argv);
	}

	~CorridorGraphDatamanRuntime()
	{
		param_control_autosave(true);

		char name[] = "dataman";
		char stop[] = "stop";
		char *argv[] = {name, stop};
		dataman_main(2, argv);

		px4::WorkQueueManagerStop();
	}
};

CorridorGraphDatamanRuntime &corridorGraphDatamanRuntime()
{
	static CorridorGraphDatamanRuntime runtime{};
	return runtime;
}

// CorridorGraph's constructor creates DatamanClient members that register with the
// dataman work queue immediately; dataman must already be running before that happens,
// not just before the first read/write. A global environment guarantees this runs once,
// before any TEST body (and its CorridorGraph objects) executes.
class CorridorGraphDatamanEnvironment : public ::testing::Environment
{
public:
	void SetUp() override { corridorGraphDatamanRuntime(); }
};

testing::Environment *const kCorridorGraphDatamanEnv =
	testing::AddGlobalTestEnvironment(new CorridorGraphDatamanEnvironment());

mission_corridor_node_s makeNode(double lat, double lon, float alt, CorridorNodeType type)
{
	mission_corridor_node_s node{};
	node.lat = lat;
	node.lon = lon;
	node.alt = alt;
	node.type = static_cast<uint8_t>(type);
	return node;
}

mission_corridor_edge_s makeEdge(uint16_t from, uint16_t to, float static_cost = 1.0f)
{
	mission_corridor_edge_s edge{};
	edge.node_from = from;
	edge.node_to = to;
	edge.static_cost = static_cost;
	return edge;
}

// Writes a corridor graph into dataman buffer 0 and returns once CorridorGraph
// has finished (re)loading it.
void loadGraph(CorridorGraph &graph, const std::vector<mission_corridor_node_s> &nodes,
	       const std::vector<mission_corridor_edge_s> &edges)
{
	corridorGraphDatamanRuntime();

	DatamanClient client;

	for (size_t i = 0; i < nodes.size(); i++) {
		mission_corridor_node_s node = nodes[i];
		ASSERT_TRUE(client.writeSync(DM_KEY_CORRIDOR_NODES_0, i, reinterpret_cast<uint8_t *>(&node),
					     sizeof(node)));
	}

	for (size_t i = 0; i < edges.size(); i++) {
		mission_corridor_edge_s edge = edges[i];
		ASSERT_TRUE(client.writeSync(DM_KEY_CORRIDOR_EDGES_0, i, reinterpret_cast<uint8_t *>(&edge),
					     sizeof(edge)));
	}

	static uint32_t graph_id = 0;
	graph_id++;

	mission_stats_entry_s node_stats{};
	node_stats.num_items = static_cast<uint16_t>(nodes.size());
	node_stats.dataman_id = static_cast<uint8_t>(DM_KEY_CORRIDOR_NODES_0);
	node_stats.opaque_id = graph_id;
	ASSERT_TRUE(client.writeSync(DM_KEY_CORRIDOR_NODES_STATE, 0, reinterpret_cast<uint8_t *>(&node_stats),
				     sizeof(node_stats)));

	mission_stats_entry_s edge_stats{};
	edge_stats.num_items = static_cast<uint16_t>(edges.size());
	edge_stats.dataman_id = static_cast<uint8_t>(DM_KEY_CORRIDOR_EDGES_0);
	edge_stats.opaque_id = graph_id;
	ASSERT_TRUE(client.writeSync(DM_KEY_CORRIDOR_EDGES_STATE, 0, reinterpret_cast<uint8_t *>(&edge_stats),
				     sizeof(edge_stats)));

	graph.updateGraph();

	for (int i = 0; i < 2000 && !graph.isLoaded(); i++) {
		graph.run();
		usleep(1000);
	}

	ASSERT_TRUE(graph.isLoaded());
	ASSERT_EQ(graph.numNodes(), nodes.size());
	ASSERT_EQ(graph.numEdges(), edges.size());
}

} // namespace

// Small triangle: 0 = waypoint (start), 1 = nest, 2 = rally point.
// Coordinates are ~100m apart so nearest-node search is unambiguous.
static const mission_corridor_node_s kWaypoint = makeNode(47.000, 8.000, 50.f, CorridorNodeType::Waypoint);
static const mission_corridor_node_s kNest = makeNode(47.001, 8.000, 50.f, CorridorNodeType::Nest);
static const mission_corridor_node_s kRally = makeNode(47.002, 8.000, 50.f, CorridorNodeType::RallyPoint);

TEST(CorridorGraphTest, ReturnsFalseWhenNotLoaded)
{
	CorridorGraph graph(nullptr);

	mission_corridor_node_s waypoints[8];
	uint8_t num_waypoints = 0;
	EXPECT_FALSE(graph.findPath(47.0, 8.0, 50.f, waypoints, num_waypoints, 8));
	EXPECT_EQ(num_waypoints, 0);
}

TEST(CorridorGraphTest, FindsNestWhenReachable)
{
	CorridorGraph graph(nullptr);
	loadGraph(graph, {kWaypoint, kNest, kRally}, {makeEdge(0, 1, 10.f)});

	mission_corridor_node_s waypoints[8];
	uint8_t num_waypoints = 0;
	ASSERT_TRUE(graph.findPath(kWaypoint.lat, kWaypoint.lon, kWaypoint.alt, waypoints, num_waypoints, 8));

	ASSERT_EQ(num_waypoints, 2);
	EXPECT_EQ(waypoints[0].type, static_cast<uint8_t>(CorridorNodeType::Waypoint));
	EXPECT_EQ(waypoints[1].type, static_cast<uint8_t>(CorridorNodeType::Nest));
}

TEST(CorridorGraphTest, FallsBackToNearestReachableRallyPointWhenNestUnreachable)
{
	CorridorGraph graph(nullptr);
	// No edge leads to the nest (index 1); only the rally point (index 2) is reachable.
	loadGraph(graph, {kWaypoint, kNest, kRally}, {makeEdge(0, 2, 10.f)});

	mission_corridor_node_s waypoints[8];
	uint8_t num_waypoints = 0;
	ASSERT_TRUE(graph.findPath(kWaypoint.lat, kWaypoint.lon, kWaypoint.alt, waypoints, num_waypoints, 8));

	ASSERT_EQ(num_waypoints, 2);
	EXPECT_EQ(waypoints[0].type, static_cast<uint8_t>(CorridorNodeType::Waypoint));
	EXPECT_EQ(waypoints[1].type, static_cast<uint8_t>(CorridorNodeType::RallyPoint));
}

TEST(CorridorGraphTest, ReturnsFalseWhenNothingReachable)
{
	CorridorGraph graph(nullptr);
	// Edges only lead back to the waypoint, not out of it.
	loadGraph(graph, {kWaypoint, kNest, kRally}, {makeEdge(1, 0, 10.f), makeEdge(2, 0, 10.f)});

	mission_corridor_node_s waypoints[8];
	uint8_t num_waypoints = 0;
	EXPECT_FALSE(graph.findPath(kWaypoint.lat, kWaypoint.lon, kWaypoint.alt, waypoints, num_waypoints, 8));
	EXPECT_EQ(num_waypoints, 0);
}

TEST(CorridorGraphTest, RespectsDirectedEdgeDirection)
{
	CorridorGraph graph(nullptr);
	// Only a nest -> waypoint edge exists (the reverse of what RTL needs); there is no
	// waypoint -> nest edge. A direction-agnostic (or backwards) Dijkstra call would
	// incorrectly report the nest as reachable from the waypoint here.
	loadGraph(graph, {kWaypoint, kNest}, {makeEdge(1, 0, 5.f)});

	mission_corridor_node_s waypoints[8];
	uint8_t num_waypoints = 0;
	EXPECT_FALSE(graph.findPath(kWaypoint.lat, kWaypoint.lon, kWaypoint.alt, waypoints, num_waypoints, 8));
	EXPECT_EQ(num_waypoints, 0);
}

TEST(CorridorGraphTest, FindPathToRallyPointIgnoresReachableNest)
{
	CorridorGraph graph(nullptr);
	// Both the nest and the rally point are reachable; findPathToRallyPoint() must
	// still target the rally point, unlike findPath()'s nest-preferred behavior.
	loadGraph(graph, {kWaypoint, kNest, kRally}, {makeEdge(0, 1, 10.f), makeEdge(0, 2, 5.f)});

	mission_corridor_node_s waypoints[8];
	uint8_t num_waypoints = 0;
	ASSERT_TRUE(graph.findPathToRallyPoint(kWaypoint.lat, kWaypoint.lon, kWaypoint.alt, waypoints, num_waypoints, 8));

	ASSERT_EQ(num_waypoints, 2);
	EXPECT_EQ(waypoints[0].type, static_cast<uint8_t>(CorridorNodeType::Waypoint));
	EXPECT_EQ(waypoints[1].type, static_cast<uint8_t>(CorridorNodeType::RallyPoint));
}

TEST(CorridorGraphTest, FindPathToRallyPointReturnsFalseWhenNoneReachable)
{
	CorridorGraph graph(nullptr);
	// Only the nest has an inbound edge; no rally point is reachable from the waypoint.
	loadGraph(graph, {kWaypoint, kNest, kRally}, {makeEdge(0, 1, 10.f)});

	mission_corridor_node_s waypoints[8];
	uint8_t num_waypoints = 0;
	EXPECT_FALSE(graph.findPathToRallyPoint(kWaypoint.lat, kWaypoint.lon, kWaypoint.alt, waypoints, num_waypoints, 8));
	EXPECT_EQ(num_waypoints, 0);
}

TEST(CorridorGraphTest, MultiHopPathIncludesIntermediateWaypoints)
{
	CorridorGraph graph(nullptr);
	// The nest (index 3) is only reachable via a two-hop path through an intermediate
	// waypoint (index 1); a direct, cheaper edge to the rally point (index 2) exists
	// too, but the nest must still win since it's reachable. This pins down that the
	// reconstructed path includes the intermediate hop rather than just start+goal.
	const mission_corridor_node_s waypoint2 = makeNode(47.0005, 8.000, 50.f, CorridorNodeType::Waypoint);
	loadGraph(graph, {kWaypoint, waypoint2, kRally, kNest}, {
		makeEdge(0, 1, 1.f),
		makeEdge(1, 3, 1.f),
		makeEdge(0, 2, 1.f), // reaches only the rally point, but not the nest
	});

	mission_corridor_node_s waypoints[8];
	uint8_t num_waypoints = 0;
	ASSERT_TRUE(graph.findPath(kWaypoint.lat, kWaypoint.lon, kWaypoint.alt, waypoints, num_waypoints, 8));

	ASSERT_EQ(num_waypoints, 3);
	EXPECT_EQ(waypoints[0].type, static_cast<uint8_t>(CorridorNodeType::Waypoint));
	EXPECT_EQ(waypoints[1].type, static_cast<uint8_t>(CorridorNodeType::Waypoint));
	EXPECT_EQ(waypoints[2].type, static_cast<uint8_t>(CorridorNodeType::Nest));
}

// RTL::isWithinBatteryBudget() is the pure decision that gates RTL_TYPE=7's nest-vs-rally
// choice on remaining battery time (see RTL::tryFindCorridorPath()). It's covered here
// directly with fabricated inputs since driving a precise battery time estimate through
// SITL isn't practical.
TEST(RtlIsWithinBatteryBudgetTest, WithinBudgetWhenEstimateIsLessThanRemaining)
{
	rtl_time_estimate_s estimate{};
	estimate.valid = true;
	estimate.safe_time_estimate = 100.f;
	EXPECT_TRUE(RTL::isWithinBatteryBudget(estimate, 150.f));
}

TEST(RtlIsWithinBatteryBudgetTest, NotWithinBudgetWhenEstimateExceedsRemaining)
{
	rtl_time_estimate_s estimate{};
	estimate.valid = true;
	estimate.safe_time_estimate = 200.f;
	EXPECT_FALSE(RTL::isWithinBatteryBudget(estimate, 150.f));
}

TEST(RtlIsWithinBatteryBudgetTest, NotWithinBudgetWhenEstimateIsInvalid)
{
	rtl_time_estimate_s estimate{};
	estimate.valid = false;
	estimate.safe_time_estimate = 10.f;
	EXPECT_FALSE(RTL::isWithinBatteryBudget(estimate, 150.f));
}

TEST(RtlIsWithinBatteryBudgetTest, NotWithinBudgetWhenBatteryRemainingIsNan)
{
	rtl_time_estimate_s estimate{};
	estimate.valid = true;
	estimate.safe_time_estimate = 10.f;
	EXPECT_FALSE(RTL::isWithinBatteryBudget(estimate, std::nanf("")));
}
