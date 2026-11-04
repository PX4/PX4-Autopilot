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

bool solveBackward(const int num_nodes, const int goal, const float *cost, const bool symmetric,
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
	visited[goal] = true;

	// O(N^2) Dijkstra on the reverse graph. Each iteration relaxes incoming edges
	// v -> u using cost(v, u) and, in the same sweep over v, tracks the smallest
	// best_cost among still-unvisited nodes — that argmin is the u for the next
	// iteration. The very first u is `goal` (the only node with best_cost == 0).
	int u = goal;
	float u_cost = 0.f;

	for (int iter = 0; iter < num_nodes - 1; ++iter) {
		int next_u = -1;
		float next_cost = kUnreachable;

		for (int v = 0; v < num_nodes; ++v) {
			if (visited[v]) {
				continue;
			}

			// Edge v -> u. For asymmetric layout, look up the (v, u) entry directly.
			// For symmetric packed upper-triangular: index by the unordered pair {v, u}.
			float edge;

			if (symmetric) {
				edge = cost[symmetricPairIndex(v, u, num_nodes)];

			} else {
				edge = cost[v * num_nodes + u];
			}

			// Treat +INFINITY and NaN as missing edges. `edge < kUnreachable` is false for
			// both because NaN is unordered and INFINITY < INFINITY is false.
			if (edge < kUnreachable) {
				const float candidate = u_cost + edge;

				if (candidate < best_cost[v]) {
					best_cost[v] = candidate;
					next_node[v] = u;
				}
			}

			// Same sweep: track argmin of best_cost over unvisited nodes for the next iter.
			if (best_cost[v] < next_cost) {
				next_cost = best_cost[v];
				next_u = v;
			}
		}

		if (next_u < 0) {
			// No unvisited node is reachable; remaining nodes stay at kUnreachable.
			break;
		}

		visited[next_u] = true;
		u = next_u;
		u_cost = next_cost;
	}

	return true;
}

} // namespace dijkstra
