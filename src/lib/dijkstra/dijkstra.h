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
 * @file dijkstra.h
 *
 * Backward single-source Dijkstra over a precomputed asymmetric cost matrix.
 *
 * `solveBackward` computes the shortest path from every node to a fixed `goal`
 * and returns, for each node `i`, the next node to step toward on its shortest
 * path to the goal. The intended usage is:
 *
 *   1. Whenever the graph (cost matrix) or the goal changes, call solveBackward
 *      once to fill the next-hop table.
 *   2. As long as both stay fixed, querying the path from any new start `s` is
 *      a constant-time table lookup followed by a forward walk:
 *         u = s; while (u != goal) { emit(u); u = next_node[u]; }
 *      No re-planning, no backtracking, no temporary buffers.
 *
 * Two cost-matrix layouts are supported, selected by the `symmetric` flag:
 *
 *   - Asymmetric (default, symmetric = false): full N*N row-major matrix.
 *     `cost[i * num_nodes + j]` is the cost of the directed edge i -> j.
 *
 *   - Symmetric (symmetric = true): packed upper triangle, no diagonal.
 *     Buffer length is N*(N-1)/2 floats. The cost of the (undirected) edge
 *     between i and j (i != j) is at:
 *         offset(i, j) = a*(2*N - a - 1)/2 + (b - a - 1),  where a = min(i,j), b = max(i,j)
 *     Self-loops are not stored; Dijkstra ignores them anyway.
 *
 * Entries equal to +INFINITY or NaN are treated as missing edges. Negative
 * costs are not supported (Dijkstra assumes non-negative edge weights).
 */

#pragma once

#include <math.h>

namespace dijkstra
{

/** Sentinel cost for unreachable nodes / missing edges. */
static constexpr float kUnreachable = INFINITY;

/**
 * Compute backward shortest paths from every node to `goal`.
 *
 * @param num_nodes  number of nodes.
 * @param goal       target node index in [0, num_nodes).
 * @param cost       cost buffer; layout depends on `symmetric` (see above).
 *                   Missing edges are encoded as +INFINITY or NaN.
 * @param symmetric  if true, `cost` is the packed upper triangle of a symmetric
 *                   matrix (length N*(N-1)/2). If false, `cost` is the full
 *                   N*N row-major matrix and edges may be asymmetric.
 * @param best_cost  out, length num_nodes: best_cost[i] = shortest cost from i
 *                   to goal, or kUnreachable. best_cost[goal] = 0.
 * @param next_node  out, length num_nodes: next_node[i] = the node to step to
 *                   from i on the shortest path to goal, or -1 if i == goal or
 *                   i has no path to goal.
 * @param visited    scratch buffer, length num_nodes; contents on return are
 *                   not meaningful.
 *
 * @return true if inputs are valid (non-null, num_nodes > 0, goal in range);
 *         false otherwise. A `true` return does not imply that goal is
 *         reachable from any particular node — check best_cost[i] for that.
 */
bool solveBackward(int num_nodes, int goal, const float *cost, bool symmetric,
		   float *best_cost, int *next_node, bool *visited);

} // namespace dijkstra
