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

#include <gtest/gtest.h>
#include <cstdlib>
#include <math.h>
#include "dijkstra.h"

namespace
{

// Helper: assemble cost matrix entries into a row-major buffer. Use INFINITY for
// missing edges; explicit list of finite (i, j, cost) triples is concise in tests.
struct Edge { int from, to; float cost; };

template <int N>
void makeMatrix(float (&m)[N * N], std::initializer_list<Edge> edges)
{
	for (int i = 0; i < N * N; ++i) {
		m[i] = INFINITY;
	}

	for (const auto &e : edges) {
		m[e.from * N + e.to] = e.cost;
	}
}

} // namespace

TEST(DijkstraTest, SingleNodeAtGoal)
{
	constexpr int N = 1;
	float cost[N * N] = { INFINITY };
	float best[N];
	int next[N];
	bool vis[N];

	ASSERT_TRUE(dijkstra::solveBackward(N, 0, cost, false, best, next, vis));
	EXPECT_FLOAT_EQ(best[0], 0.f);
	EXPECT_EQ(next[0], -1);
}

TEST(DijkstraTest, AsymmetricLineGraphForward)
{
	// 0 -> 1 -> 2, going forward is cheap, going backward is expensive.
	constexpr int N = 3;
	float cost[N * N];
	makeMatrix<N>(cost, {
		{0, 1, 1.f},
		{1, 2, 1.f},
		{1, 0, 100.f},
		{2, 1, 100.f},
	});

	float best[N];
	int next[N];
	bool vis[N];

	ASSERT_TRUE(dijkstra::solveBackward(N, 2, cost, false, best, next, vis));

	EXPECT_FLOAT_EQ(best[0], 2.f); // 0 -> 1 -> 2 costs 1 + 1
	EXPECT_FLOAT_EQ(best[1], 1.f); // 1 -> 2
	EXPECT_FLOAT_EQ(best[2], 0.f);
	EXPECT_EQ(next[0], 1);
	EXPECT_EQ(next[1], 2);
	EXPECT_EQ(next[2], -1);
}

TEST(DijkstraTest, AsymmetricLineGraphReverse)
{
	// Same matrix as above; with goal = 0, paths must use the expensive back-edges,
	// confirming the solver respects edge direction (uses cost[i][j] = i -> j, not j -> i).
	constexpr int N = 3;
	float cost[N * N];
	makeMatrix<N>(cost, {
		{0, 1, 1.f},
		{1, 2, 1.f},
		{1, 0, 100.f},
		{2, 1, 100.f},
	});

	float best[N];
	int next[N];
	bool vis[N];

	ASSERT_TRUE(dijkstra::solveBackward(N, 0, cost, false, best, next, vis));

	EXPECT_FLOAT_EQ(best[0], 0.f);
	EXPECT_FLOAT_EQ(best[1], 100.f);
	EXPECT_FLOAT_EQ(best[2], 200.f);
	EXPECT_EQ(next[1], 0);
	EXPECT_EQ(next[2], 1);
	EXPECT_EQ(next[0], -1);
}

TEST(DijkstraTest, TriangleWithBlockedEdge)
{
	// Direct edge 0 -> 2 is cheap; remove it and the solver must route via 1.
	constexpr int N = 3;
	float cost[N * N];
	makeMatrix<N>(cost, {
		{0, 1, 1.f},
		{1, 2, 1.f},
		{0, 2, 0.5f},
	});

	float best[N];
	int next[N];
	bool vis[N];

	ASSERT_TRUE(dijkstra::solveBackward(N, 2, cost, false, best, next, vis));
	EXPECT_FLOAT_EQ(best[0], 0.5f);
	EXPECT_EQ(next[0], 2);

	// Block the direct edge and re-solve; path must detour through 1.
	cost[0 * N + 2] = INFINITY;
	ASSERT_TRUE(dijkstra::solveBackward(N, 2, cost, false, best, next, vis));
	EXPECT_FLOAT_EQ(best[0], 2.f);
	EXPECT_EQ(next[0], 1);
}

TEST(DijkstraTest, GoalUnreachableNoInfiniteLoop)
{
	// Two disconnected components: {0, 1} and {2}. Goal = 2, so 0 and 1 must end
	// up unreachable and the algorithm must not loop.
	constexpr int N = 3;
	float cost[N * N];
	makeMatrix<N>(cost, {
		{0, 1, 1.f},
		{1, 0, 1.f},
	});

	float best[N];
	int next[N];
	bool vis[N];

	ASSERT_TRUE(dijkstra::solveBackward(N, 2, cost, false, best, next, vis));
	EXPECT_FLOAT_EQ(best[2], 0.f);
	EXPECT_FALSE(best[0] < dijkstra::kUnreachable);
	EXPECT_FALSE(best[1] < dijkstra::kUnreachable);
	EXPECT_EQ(next[0], -1);
	EXPECT_EQ(next[1], -1);
}

TEST(DijkstraTest, NaNTreatedAsMissingEdge)
{
	constexpr int N = 3;
	float cost[N * N];
	makeMatrix<N>(cost, {
		{0, 1, 1.f},
		{1, 2, 1.f},
		{0, 2, NAN}, // NaN must be treated like INFINITY, not as a 0 / negative edge
	});

	float best[N];
	int next[N];
	bool vis[N];

	ASSERT_TRUE(dijkstra::solveBackward(N, 2, cost, false, best, next, vis));
	EXPECT_FLOAT_EQ(best[0], 2.f);
	EXPECT_EQ(next[0], 1);
}

TEST(DijkstraTest, RejectsInvalidInputs)
{
	constexpr int N = 2;
	float cost[N * N] = { INFINITY, 1.f, 1.f, INFINITY };
	float best[N];
	int next[N];
	bool vis[N];

	EXPECT_FALSE(dijkstra::solveBackward(0, 0, cost, false, best, next, vis));
	EXPECT_FALSE(dijkstra::solveBackward(N, -1, cost, false, best, next, vis));
	EXPECT_FALSE(dijkstra::solveBackward(N, N, cost, false, best, next, vis));
	EXPECT_FALSE(dijkstra::solveBackward(N, 0, nullptr, false, best, next, vis));
	EXPECT_FALSE(dijkstra::solveBackward(N, 0, cost, false, nullptr, next, vis));
}

TEST(DijkstraTest, SymmetricPackedMatchesAsymmetric)
{
	// Same undirected weighted graph expressed two ways: full N*N matrix with both
	// halves filled, and packed upper triangle. Results must match for both layouts.
	constexpr int N = 5;

	// Edges (a, b, w) with a < b.
	struct UndirEdge { int a, b; float w; };
	const UndirEdge edges[] = {
		{0, 1, 2.f},
		{0, 2, 4.f},
		{1, 2, 1.f},
		{1, 3, 7.f},
		{2, 3, 3.f},
		{3, 4, 2.f},
	};

	float full[N * N];

	for (int i = 0; i < N * N; ++i) { full[i] = INFINITY; }

	constexpr int kPacked = N * (N - 1) / 2;
	float packed[kPacked];

	for (int i = 0; i < kPacked; ++i) { packed[i] = INFINITY; }

	for (const auto &e : edges) {
		full[e.a * N + e.b] = e.w;
		full[e.b * N + e.a] = e.w;
		packed[e.a * (2 * N - e.a - 1) / 2 + (e.b - e.a - 1)] = e.w;
	}

	float best_full[N], best_pack[N];
	int next_full[N], next_pack[N];
	bool vis[N];

	for (int goal = 0; goal < N; ++goal) {
		ASSERT_TRUE(dijkstra::solveBackward(N, goal, full, false, best_full, next_full, vis));
		ASSERT_TRUE(dijkstra::solveBackward(N, goal, packed, true, best_pack, next_pack, vis));

		for (int i = 0; i < N; ++i) {
			EXPECT_FLOAT_EQ(best_full[i], best_pack[i]) << "goal=" << goal << " i=" << i;
			EXPECT_EQ(next_full[i], next_pack[i]) << "goal=" << goal << " i=" << i;
		}
	}
}

TEST(DijkstraTest, ForwardWalkReachesGoal)
{
	// 5-node graph: 0 -> 1 -> 2 -> 3 -> 4, plus a long detour 0 -> 4 directly.
	// Solving once with goal = 4, then for any start we should reach 4 by following
	// next_node without re-solving.
	constexpr int N = 5;
	float cost[N * N];
	makeMatrix<N>(cost, {
		{0, 1, 1.f},
		{1, 2, 1.f},
		{2, 3, 1.f},
		{3, 4, 1.f},
		{0, 4, 100.f}, // direct but expensive
	});

	float best[N];
	int next[N];
	bool vis[N];
	ASSERT_TRUE(dijkstra::solveBackward(N, 4, cost, false, best, next, vis));

	for (int start = 0; start < N - 1; ++start) {
		int u = start;
		int hops = 0;

		while (u != 4 && hops < N) {
			ASSERT_GE(next[u], 0) << "stuck at " << u << " from start " << start;
			u = next[u];
			++hops;
		}

		EXPECT_EQ(u, 4) << "did not reach goal from " << start;
	}
}

TEST(DijkstraTest, SymmetricPairIndex)
{
	// create a NxN matrix and fill with random data. Then pack the upper triangle into a 1D array.
	// Then loop through the upper triangle and read the values from the array using  symmetricPairIndex
	// and verify that the values match.
	constexpr size_t N = 21;
	constexpr size_t kPairs = N * (N - 1) / 2;

	float matrix[N][N];
	float packed[kPairs];
	std::srand(42);
	size_t idx = 0;

	for (size_t i = 0; i < N; ++i) {
		for (size_t j = i + 1; j < N; ++j) {
			matrix[i][j] = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
			matrix[j][i] = matrix[i][j]; // symmetric matrix
			packed[idx++] = matrix[i][j];
		}
	}

	size_t counter = 0;

	for (size_t i = 0; i < N; ++i) {
		for (size_t j = i + 1; j < N; ++j) {
			counter++;
			EXPECT_FLOAT_EQ(matrix[i][j], packed[dijkstra::symmetricPairIndex(i, j, N)]);
			EXPECT_FLOAT_EQ(matrix[i][j], packed[dijkstra::symmetricPairIndex(j, i, N)]);
		}
	}

	// verify that we have the expected number of pairs
	EXPECT_EQ(counter, kPairs);
}
