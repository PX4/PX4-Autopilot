/****************************************************************************
 *
 *   Copyright (c) 2013-2026 PX4 Development Team. All rights reserved.
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
 * @file geofence_utils.h
 * Pure geometry functions for geofence point-in-polygon and point-in-circle tests.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>

#include <matrix/math.hpp>

#include <dataman/dataman.h>
#include <px4_platform_common/px4_config.h> // CONFIG_NAVIGATOR_GEOFENCE_MAX_NODES

namespace geofence_utils
{

static constexpr float CM_PER_M  = 100.f;

// True iff `m_value` (metres) fits into the planner's fixed-point
// representation. intersectsInsideOf doubles cm coordinates internally, so the
// usable range is 2^30 cm = 10,737 km from the planner's reference.
inline bool inFixedPointRange(float m_value) { return fabsf(m_value) < (1u << 30) / CM_PER_M; }

/**
 * Sign of the signed area of triangle abc -- the fundamental 2D orientation
 * predicate. Returns:
 *   +1, if c is left of a->b (CCW turn)
 *   -1, if c is right of a->b (CW turn)
 *    0, if collinear
 *
 * Refs:
 *
 * Fundamental ideas, usage of orient2d for higher level geometry:
 * Lecture Notes on Geometric Robustness, Jonathan Richard Shewchuk
 * https://perso.uclouvain.be/jean-francois.remacle/LMECA2170/robnotes.pdf
 *
 * The fact that in fixed-point orient2d is automatically exact:
 * Computational Geometry in C, Joseph O'Rourke
 * https://www.science.smith.edu/~jorourke/books/compgeom.html
 *
 */
inline int orient2d(int32_t ax, int32_t ay,
		    int32_t bx, int32_t by,
		    int32_t cx, int32_t cy)
{
	const int64_t det = (static_cast<int64_t>(bx) - ax) * (static_cast<int64_t>(cy) - ay)
			    - (static_cast<int64_t>(by) - ay) * (static_cast<int64_t>(cx) - ax);

	if (det > 0) { return  1; }

	if (det < 0) { return -1; }

	return 0;
}

/**
 * For a, b, c collinear (orient2d == 0), is c on the open segment ab?
 */
inline bool collinearBetween(int32_t ax, int32_t ay, int32_t bx, int32_t by, int32_t cx, int32_t cy)
{
	if (std::abs(ax - bx) >= std::abs(ay - by)) {
		return (ax < cx && cx < bx) || (bx < cx && cx < ax);

	} else {
		return (ay < cy && cy < by) || (by < cy && cy < ay);
	}
}

/**
 * Classification of the relative position of two segments ab and cd.
 *
 *   Cross       proper interior crossing (each segment strictly straddles
 *               the other's supporting line)
 *   AInsideCD   a strictly between c and d (open interior of segment cd)
 *   BInsideCD   b strictly between c and d
 *   CInsideAB   c strictly between a and b
 *   DInsideAB   d strictly between a and b
 *   Collinear   ab and cd lie on the same supporting line (overlap or not)
 *   Disjoint    none of the above
 *
 * Ref: SegSegInt implementation from:
 * Computational Geometry in C, Joseph O'Rourke
 * https://www.science.smith.edu/~jorourke/books/compgeom.html
 */
enum class SegSegResult {
	Disjoint,
	Cross,
	AInsideCD, BInsideCD,
	CInsideAB, DInsideAB,
	Collinear,
};

inline SegSegResult segmentsIntersect(int32_t ax, int32_t ay, int32_t bx, int32_t by,
				      int32_t cx, int32_t cy, int32_t dx, int32_t dy)
{
	// These two early returns are logically not necessary (would catch the
	// case in the end) but improve performance, as most segment pairs are
	// expected to be completely disjoint. If both c and d are strictly on
	// the same side of a-b (first check, second vice versa) we can already
	// conclude the segments are disjoint. This does not detect _all_
	// disjoint cases so the final return stays.

	const int o1 = orient2d(ax, ay, bx, by, cx, cy);
	const int o2 = orient2d(ax, ay, bx, by, dx, dy);

	if (o1 != 0 && o1 == o2) { return SegSegResult::Disjoint; }

	const int o3 = orient2d(cx, cy, dx, dy, ax, ay);
	const int o4 = orient2d(cx, cy, dx, dy, bx, by);

	if (o3 != 0 && o3 == o4) { return SegSegResult::Disjoint; }

	if (o1 && o2 && o3 && o4 && o1 != o2 && o3 != o4) { return SegSegResult::Cross; }

	if (!o1 && !o2 && !o3 && !o4) { return SegSegResult::Collinear; }

	// Endpoint strictly on the open interior of the other segment iff:
	//  - orient2d is zero (point on the supporting line)
	//  - the point lies strictly between the other two via collinearBetween
	if (o3 == 0 && collinearBetween(cx, cy, dx, dy, ax, ay)) { return SegSegResult::AInsideCD; }

	if (o4 == 0 && collinearBetween(cx, cy, dx, dy, bx, by)) { return SegSegResult::BInsideCD; }

	if (o1 == 0 && collinearBetween(ax, ay, bx, by, cx, cy)) { return SegSegResult::CInsideAB; }

	if (o2 == 0 && collinearBetween(ax, ay, bx, by, dx, dy)) { return SegSegResult::DInsideAB; }

	return SegSegResult::Disjoint;
}

/**
 * Check if a polygon's vertices are ordered counter-clockwise using the shoelace formula.
 * Works in local Cartesian coordinates. Returns false for CW or zero-area polygons.
 *
 * @param vertices      polygon vertices in local frame
 * @param num_vertices  number of vertices
 * @return true if the polygon is counter-clockwise
 */
bool isPolygonCCW(const matrix::Vector2f *vertices, int num_vertices);

/**
 * Upper bound on the number of vertices addPolygon() emits for a simple polygon:
 *  - Sharp convex corners (interior angle < 60 deg) are split in two vertices
 *  - Interior angles of a simple polygon sum to (n - 2) * 180 deg
 *  - With k corners below 60 deg and the rest below 360 deg, we have
 *     (n - 2) * 180 < k * 60 + (n - k) * 360
 *     =>  k < (180 n + 360) / 300
 * so k <= ceil(.) - 1, computed as (a - 1) / b in integer arithmetic.

 * Degenerate input (n < 3) is rejected by addPolygon and emits nothing, so
 * the bound is 0.
 */
constexpr int maxVerticesAfterSplitting(int num_vertices)
{
	return num_vertices < 3 ? 0 : num_vertices + (180 * num_vertices + 360 - 1) / 300;
}


/**
 * Fence geometry cache for the geofence avoidance planner. Owns a flat
 * int32-cm node buffer that holds polygon vertices, circle k-gon vertices,
 * and one destination slot.
 *
 * Polygons are stored in canonical orientation: the polygon INSIDE is always to
 * the left of each edge (CCW for exclusion zones, CW for inclusion zones).
 *
 * Typical lifecycle:
 *   1. addPolygon / addApproxCircle  (one per geofence zone)
 *   2. setDestination(p)             (called whenever destination changes)
 *   3. edgeCost(a, b) / edgeVisible  (queried during planning)
 */
class PlannerPolygons
{
public:

	// By default, limit to 100 nodes, which was found to limit the full update
	// on geofence / margin change to about 20 ms. Lower for weaker or RAM-constrained boards.

#ifndef CONFIG_NAVIGATOR_GEOFENCE_MAX_NODES
#define CONFIG_NAVIGATOR_GEOFENCE_MAX_NODES 100
#endif

	static constexpr int kMaxNodes = CONFIG_NAVIGATOR_GEOFENCE_MAX_NODES;
	static constexpr int kMaxPolygons = 16;
	static constexpr int kCircleApproxVertices = 8;

	// Number of nodes needed to hold the largest geofence storable in dataman.
	// Worst case:
	//  - kMaxPolygons-1 circles (most nodes per dataman item)
	//  - one remaining polygon using all remaining fence points, with max amount of split vertices
	//  - 1 destination slot
	// If kMaxNodes > kMaxNodesForAnyStorableFence, we always have enough space.
	// Otherwise we may fail with AddResult::BudgetExceeded.
	static constexpr int kMaxNodesForAnyStorableFence =
		(kMaxPolygons - 1) * kCircleApproxVertices
		+ maxVerticesAfterSplitting(DM_KEY_FENCE_POINTS_MAX - (kMaxPolygons - 1))
		+ 1;

	// Return code specifying why a polygon was not added.
	enum class AddResult {
		Success,
		BudgetExceeded,  // node or polygon buffer full (kMaxNodes / kMaxPolygons)
		OutOfRange,      // vertex or circle extent outside usable fixed-point range
		Degenerate,      // <3 vertices, zero-length/antiparallel edge, self-intersecting, empty circle, negative margin
	};

	PlannerPolygons() { reset(); }

	// Slot 0 is always reserved for the destination; polygon vertices start at index 1.
	void reset()
	{
		_num_nodes = 1;
		_x_cm[0] = 0;
		_y_cm[0] = 0;
		_num_polygons = 0;
		_node_not_on_optimal_path[0] = false;
	}

	// Append a polygon:
	//  - canonicalize orientation,
	//  - offset each vertex outward (exclusion) or inward (inclusion) by `margin` meters,
	//  - quantize to cm, append as nodes.
	AddResult addPolygon(const matrix::Vector2f *vertices_in, int num_vertices,
			     bool is_inclusion_zone, float margin = 0.f);

	// Append an approximate circle (k-gon over/underapproximation).
	//  - shrink/expand according to margin and is_inclusion_zone
	//  - quantize to cm, append as nodes.
	AddResult addApproxCircle(const matrix::Vector2f &center, const float radius, float margin, const bool is_inclusion_zone);

	int numNodes() const { return _num_nodes; }

	// Number of polygon nodes (excludes the destination at index 0).
	int numVertices() const { return _num_nodes - 1; }

	matrix::Vector2f node(int idx) const
	{
		return matrix::Vector2f{_x_cm[idx] / CM_PER_M, _y_cm[idx] / CM_PER_M};
	}

	// Update the destination position. Always safe to call after reset().
	// Returns false if the destination violates the geofence, but still updates destionation.
	bool setDestination(const matrix::Vector2f &p);

	// Get the current destination position.
	matrix::Vector2f getDestination() const;

	// Destination is always at index 0; polygon vertices start at index 1.
	static constexpr int destIndex() { return 0; }

	// True if the segment (a, b) crosses no geofence interior.
	bool edgeVisible(int a, int b) const;
	bool edgeVisible(const matrix::Vector2f &a, int b) const;          // One float endpoint (e.g. vehicle position).
	bool edgeVisible(const matrix::Vector2f &a, const matrix::Vector2f &b) const; // Both float endpoints.

	// Traversal cost for edge (a, b): INFINITY if the edge crosses any fence,
	// Euclidean distance (metres) otherwise.
	float edgeCost(int a, int b) const;

private:
	int32_t _x_cm[kMaxNodes];
	int32_t _y_cm[kMaxNodes];

	struct PolygonInfo {
		int start_index;
		int num_vertices;
		bool is_inclusion;
		int32_t min_x, max_x, min_y, max_y; // Bouding box
	};

	PolygonInfo _polygons[kMaxPolygons];
	int _num_polygons{0};

	/**
	 * Reduced visibility graph: we can exclude some vertices a priori
	 * because they cannot be part of the shortest path. For convex
	 * inclusion corners or reflex exclusion corners at node B, the path
	 *   (A -> B -> C)
	 * is a detour compared to
	 *   (A -> just before B -> just after B -> C)
	 * and so it cannot be globally optimal.
	 */
	bool _node_not_on_optimal_path[kMaxNodes];
	int _num_nodes{0};

	/**
	 * Line-segment vs polygon intersection check.
	 *
	 * Only detects intersection with the _interior_ of the polygon in
	 * canonical orientation, i.e. the forbidden region. Intersection with
	 * only the boundary is allowed (necessary for shortest path to be well
	 * defined)
	 */
	bool intersectsInsideOf(const PolygonInfo &poly, int32_t s_x, int32_t s_y, int32_t e_x, int32_t e_y) const;

	/**
	 * Return true iff the line segment s-e intersects the inside of _any_ stored polygon.
	 */
	bool intersectsAnyInside(int32_t s_x, int32_t s_y, int32_t e_x, int32_t e_y) const;

	/**
	 * Consider extending the triangle poly[v-1] -> poly[v] -> poly[v+1]
	 * outward from poly[v] (the inside always being left of the points).
	 * This cone is the local interior of the forbidden region at the given
	 * vertex.
	 *
	 * Returns true if the point p is in the interior of that cone, which is
	 * the case iff the line segment poly[v] -> p pierces the interior at
	 * vertex curr.
	 */
	bool pointInsideInteriorCone(const PolygonInfo &poly, int32_t px, int32_t py, int v) const;

	/**
	 * Return true if the edge ab is bitangent at a: neither the edge nor
	 * its mirror about `a` enters the forbidden interior there.
	 *
	 * Only bitangent edges can be part of a shortest path [*]. Checking
	 * this first in edgeVisible also throws out invalid (interior-entering)
	 * edges before the general intersection checks.
	 *
	 * * Stephen LaValle, 2006: Planning Algorithms, 6.2.4 - Shortest-Path Roadmaps
	 */
	bool edgeBitangent(int a, int32_t bx, int32_t by) const;
	bool edgeBitangent(int a, int b) const;                        // Other endpoint is node index b.
	bool edgeBitangent(int a, const matrix::Vector2f &b) const;    // Other endpoint is an arbitrary position.

	/**
	 * Update {min,max}_{x,y} with the min/max coordinates of the polygon with vertices
	 * [start_index, start_index + num_vertices).
	 */
	void computeBoundingBox(const int start_index, const int num_vertices,
				int32_t &min_x, int32_t &max_x, int32_t &min_y, int32_t &max_y);

	// Write float-metre coordinates into an existing node slot (used internally by
	// addPolygon, addApproxCircle, and setDestination).
	void setNode(int idx, const matrix::Vector2f &p);

};

// Convenience for unit tests and one-shot callers: build a transient
// PlannerPolygons with the given polygon and query the segment.
inline bool lineSegmentIntersectsPolygon(const matrix::Vector2f &start, const matrix::Vector2f &end,
		const matrix::Vector2f *vertices, int num_vertices, bool is_inclusion_zone)
{
	PlannerPolygons polys;

	if (polys.addPolygon(vertices, num_vertices, is_inclusion_zone) != PlannerPolygons::AddResult::Success) {
		return false;
	}

	return !polys.edgeVisible(start, end);
}

} // namespace geofence_utils
