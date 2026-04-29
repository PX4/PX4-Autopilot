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

#include "dijkstra.h"

namespace dijkstra
{

bool solveBackward(int num_nodes, int goal, const float *cost,
		   float *best_cost, int *next_node, bool *visited)
{
	if (num_nodes <= 0 || goal < 0 || goal >= num_nodes
	    || cost == nullptr || best_cost == nullptr || next_node == nullptr || visited == nullptr) {
		return false;
	}

	for (int i = 0; i < num_nodes; ++i) {
		best_cost[i] = kUnreachable;
		next_node[i] = -1;
		visited[i] = false;
	}

	best_cost[goal] = 0.f;

	// Standard O(N^2) Dijkstra on the reverse graph: pick the unvisited node u with
	// the smallest current best_cost, then relax every incoming edge v -> u using
	// cost(v, u). This yields, for each v, the shortest cost from v to goal and the
	// next hop next_node[v] = u along that path.
	for (int iter = 0; iter < num_nodes; ++iter) {
		int u = -1;
		float min_cost = kUnreachable;

		for (int i = 0; i < num_nodes; ++i) {
			if (!visited[i] && best_cost[i] < min_cost) {
				min_cost = best_cost[i];
				u = i;
			}
		}

		if (u < 0) {
			// No unvisited node is reachable; remaining nodes stay at kUnreachable.
			break;
		}

		visited[u] = true;

		const float u_cost = best_cost[u];

		for (int v = 0; v < num_nodes; ++v) {
			if (visited[v]) {
				continue;
			}

			const float edge = cost[v * num_nodes + u];

			// Treat +INFINITY and NaN as missing edges. `edge < kUnreachable` is false for
			// both because NaN is unordered and INFINITY < INFINITY is false.
			if (!(edge < kUnreachable)) {
				continue;
			}

			const float candidate = u_cost + edge;

			if (candidate < best_cost[v]) {
				best_cost[v] = candidate;
				next_node[v] = u;
			}
		}
	}

	return true;
}

} // namespace dijkstra
