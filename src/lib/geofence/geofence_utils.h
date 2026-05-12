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
 * Update the PNPOLY inside/outside state for a single polygon edge.
 * Templated on the coordinate scalar type so the same test can be used
 * with either float (local frame, meters) or double (lat/lon degrees).
 *
 * @param last_state  state passed in on each call as reference, init to false
 * @param v1          edge start vertex
 * @param v2          edge end vertex
 * @param point       test point
 */
template <typename T>
void insidePolygonUpdateState(bool &last_state,
			      const matrix::Vector2<T> &v1,
			      const matrix::Vector2<T> &v2,
			      const matrix::Vector2<T> &point)
{
	/**
	 * Adaptation of algorithm originally presented as
	 * PNPOLY - Point Inclusion in Polygon Test
	 * W. Randolph Franklin (WRF)
	 * Only supports non-complex polygons (not self intersecting)
	 */
	if (((v1(1) >= point(1)) != (v2(1) >= point(1))) &&
	    (point(0) <= (v2(0) - v1(0)) * (point(1) - v1(1)) /
	     (v2(1) - v1(1)) + v1(0))) {
		last_state = !last_state;
	}
}


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
	const int o1 = orient2d(ax, ay, bx, by, cx, cy);
	const int o2 = orient2d(ax, ay, bx, by, dx, dy);
	const int o3 = orient2d(cx, cy, dx, dy, ax, ay);
	const int o4 = orient2d(cx, cy, dx, dy, bx, by);

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
 * Read-only fence cache. Owns a flat int32-cm node buffer that holds every
 * position the planner cares about (polygon vertices, circle-approximation
 * vertices, destination, ...) and a polygon metadata table that slices into
 * it. Polygons are stored in canonical orientation: walking vertices in
 * stored order, the polygon's INSIDE region is always to the left of each
 * edge (CCW for exclusion zones, CW for inclusion zones).
 *
 * Three layers of intersection query:
 *   (1) Free int32 predicates (orient2d, segmentsIntersect, collinearBetween)
 *       at the top of this header -- raw scalar coordinates, no cm semantics.
 *   (2) `isLineBetweenNodesIntersectingAnyInside(a, b)` -- node-to-node
 *       visibility for the planner's V^2 hot path; returns false if a == b.
 *   (3) `isLineFromPointToNodeIntersectingAnyInside(p, idx)` -- the only
 *       float bridge, intended for the vehicle's current position.
 */
class PlannerPolygons
{
public:
	static constexpr int kMaxNodes = 100;
	static constexpr int kMaxPolygons = 16;

	PlannerPolygons() = default;

	void reset() { _num_nodes = 0; _num_polygons = 0; }

	/// Append a single point to the node buffer. Returns its index, or -1 if full.
	int addNode(const matrix::Vector2f &p);

	/// Replace the node at `idx` (used to update the destination slot in place).
	void setNode(int idx, const matrix::Vector2f &p);

	/// Append a polygon:
	///  - canonicalize orientation,
	///  - optionally offset each vertex outward (exclusion) or inward (inclusion) by `margin` meters
	///  - quantize to cm, append as nodes.
	/// Returns false on budget overflow, fewer than 3 vertices, or a
	/// degenerate edge/antiparallel corner when margin != 0.
	bool addPolygon(const matrix::Vector2f *vertices_in, int num_vertices,
			bool is_inclusion_zone, float margin = 0.f);

	/// Append an approximate circle (k-gon over/underapproximation).
	///  - shrink/expand according to margin and is_inclusion_zone
	///  - quantize to cm, append as nodes.
	bool addApproxCircle(const matrix::Vector2f &center, const float radius, float margin, const int num_vertices,
			     const bool is_inclusion_zone);

	int numNodes() const { return _num_nodes; }

	matrix::Vector2f node(int idx) const
	{
		return matrix::Vector2f{_x_cm[idx] / 100.f, _y_cm[idx] / 100.f};
	}

	bool isLineBetweenNodesIntersectingAnyInside(int a, int b) const;
	bool isLineFromPointToNodeIntersectingAnyInside(const matrix::Vector2f &p, int node_idx) const;

private:
	struct PolygonInfo {
		int start_index;
		int num_vertices;
		bool is_inclusion;
	};

	int32_t _x_cm[kMaxNodes];
	int32_t _y_cm[kMaxNodes];
	int _num_nodes{0};

	PolygonInfo _polygons[kMaxPolygons];
	int _num_polygons{0};

	bool intersectsAnyInside(int32_t s_x, int32_t s_y, int32_t e_x, int32_t e_y) const;
	bool intersectsInsideOf(const PolygonInfo &poly,
				int32_t s_x, int32_t s_y, int32_t e_x, int32_t e_y) const;
	bool pointInsideInteriorCone(const PolygonInfo &poly, int32_t px, int32_t py, int v) const;
};

/// Convenience for unit tests and one-shot callers: build a transient
/// PlannerPolygons with the given polygon and two extra nodes for the segment
/// endpoints, then run the node-to-node query.
inline bool lineSegmentIntersectsPolygon(const matrix::Vector2f &start, const matrix::Vector2f &end,
		const matrix::Vector2f *vertices, int num_vertices, bool is_inclusion_zone)
{
	PlannerPolygons polys;

	if (!polys.addPolygon(vertices, num_vertices, is_inclusion_zone)) {
		return false;
	}

	return polys.isLineBetweenNodesIntersectingAnyInside(polys.addNode(start), polys.addNode(end));
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
