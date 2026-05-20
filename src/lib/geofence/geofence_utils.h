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

namespace geofence_utils
{

/**
 * Sign of the signed area of triangle abc -- the fundamental 2D orientation
 * predicate. +1 if c is left of a->b (CCW turn), -1 if right (CW turn),
 * 0 if collinear. Reference: O'Rourke, "Computational Geometry in C", 1.5.
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
 * For a, b, c known to be collinear (orient2d == 0), is c on the closed
 * segment ab?
 */
inline bool collinearBetween(int32_t ax, int32_t ay, int32_t bx, int32_t by, int32_t cx, int32_t cy)
{
	if (std::abs(ax - bx) >= std::abs(ay - by)) {
		return (ax <= cx && cx <= bx) || (bx <= cx && cx <= ax);

	} else {
		return (ay <= cy && cy <= by) || (by <= cy && cy <= ay);
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
 * Reference: O'Rourke, "Computational Geometry in C" (2nd ed.), SegSegInt,
 * section 1.5.
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

	// Endpoint strictly on the open interior of the other segment: orient2d
	// is zero (point on the supporting line), the point lies between the
	// other two via collinearBetween, and is not coincident with either.
	if (o3 == 0 && collinearBetween(cx, cy, dx, dy, ax, ay)
	    && !(ax == cx && ay == cy) && !(ax == dx && ay == dy)) { return SegSegResult::AInsideCD; }

	if (o4 == 0 && collinearBetween(cx, cy, dx, dy, bx, by)
	    && !(bx == cx && by == cy) && !(bx == dx && by == dy)) { return SegSegResult::BInsideCD; }

	if (o1 == 0 && collinearBetween(ax, ay, bx, by, cx, cy)
	    && !(cx == ax && cy == ay) && !(cx == bx && cy == by)) { return SegSegResult::CInsideAB; }

	if (o2 == 0 && collinearBetween(ax, ay, bx, by, dx, dy)
	    && !(dx == ax && dy == ay) && !(dx == bx && dy == by)) { return SegSegResult::DInsideAB; }

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
	static constexpr int kMaxNodes = 100;
	static constexpr int kMaxPolygons = 16;

	PlannerPolygons() { reset(); }

	// Slot 0 is always reserved for the destination; polygon vertices start at index 1.
	void reset() { _num_nodes = 1; _x_cm[0] = 0; _y_cm[0] = 0; _num_polygons = 0; }

	// Append a polygon:
	//  - canonicalize orientation,
	//  - optionally offset each vertex outward (exclusion) or inward (inclusion) by `margin` meters
	//  - quantize to cm, append as nodes.
	// Returns false on budget overflow, fewer than 3 vertices, or a
	// degenerate edge/antiparallel corner when margin != 0.
	bool addPolygon(const matrix::Vector2f *vertices_in, int num_vertices,
			bool is_inclusion_zone, float margin = 0.f);

	// Append an approximate circle (k-gon over/underapproximation).
	//  - shrink/expand according to margin and is_inclusion_zone
	//  - quantize to cm, append as nodes.
	bool addApproxCircle(const matrix::Vector2f &center, const float radius, float margin, const int num_vertices,
			     const bool is_inclusion_zone);

	int numNodes() const { return _num_nodes; }

	// Number of polygon/circle nodes (excludes the destination at index 0).
	int numVertices() const { return _num_nodes - 1; }

	matrix::Vector2f node(int idx) const
	{
		return matrix::Vector2f{_x_cm[idx] / 100.f, _y_cm[idx] / 100.f};
	}

	// Update the destination position. Always safe to call after reset().
	void setDestination(const matrix::Vector2f &p);

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
	struct PolygonInfo {
		int start_index;
		int num_vertices;
		bool is_inclusion;
		int32_t min_x, max_x, min_y, max_y; // Bouding box
	};

	int32_t _x_cm[kMaxNodes];
	int32_t _y_cm[kMaxNodes];
	int _num_nodes{0};

	PolygonInfo _polygons[kMaxPolygons];
	int _num_polygons{0};

	/// Write float-metre coordinates into an existing node slot (used internally by
	/// addPolygon, addApproxCircle, and setDestination).
	void setNode(int idx, const matrix::Vector2f &p);

	bool intersectsAnyInside(int32_t s_x, int32_t s_y, int32_t e_x, int32_t e_y) const;
	bool intersectsInsideOf(const PolygonInfo &poly,
				int32_t s_x, int32_t s_y, int32_t e_x, int32_t e_y) const;
	bool pointInsideInteriorCone(const PolygonInfo &poly, int32_t px, int32_t py, int v) const;
	void computeBoundingBox(const int start_index, const int num_vertices,
				int32_t &min_x, int32_t &max_x, int32_t &min_y, int32_t &max_y);

};

/// Convenience for unit tests and one-shot callers: build a transient
/// PlannerPolygons with the given polygon and query the segment.
inline bool lineSegmentIntersectsPolygon(const matrix::Vector2f &start, const matrix::Vector2f &end,
		const matrix::Vector2f *vertices, int num_vertices, bool is_inclusion_zone)
{
	PlannerPolygons polys;

	if (!polys.addPolygon(vertices, num_vertices, is_inclusion_zone)) {
		return false;
	}

	return !polys.edgeVisible(start, end);
}


/**
 * Map the upper triangular matrix WITHOUT the diagonal into a flat array. Caller must ensure i != j.
 *
 *
 * Required array size is num_nodes * (num_nodes - 1) / 2.
 *
 * @param i          first node index, in [0, num_nodes)
 * @param j          second node index, in [0, num_nodes), must differ from i
 * @param num_nodes  total number of nodes
 * @return flat array index in [0, num_nodes * (num_nodes - 1) / 2)
 */
size_t symmetricPairIndex(size_t i, size_t j, size_t num_nodes);

} // namespace geofence_utils
