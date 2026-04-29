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

#include <matrix/math.hpp>

namespace geofence_utils
{

/**
 * Scale factor applied to the planner margin when inflating polygons / circles for the
 * line-vs-fence visibility check. The planning graph is built from polygons inflated by
 * `margin`; the visibility check uses polygons inflated by `kVisibilityCheckMarginScale * margin`
 * (slightly less) so that edges riding exactly on the planning-inflated boundary are not
 * rejected by float noise.
 */
static constexpr float kVisibilityCheckMarginScale = 0.99f;

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
 * Check if a point is inside a circle.
 *
 * @param center  circle center (lat, lon) in degrees
 * @param radius  circle radius in meters
 * @param point   test point (lat, lon) in degrees
 * @return true if the point is inside the circle
 */
bool insideCircle(const matrix::Vector2<double> &center, float radius,
		  const matrix::Vector2<double> &point);

/**
 * Check if two line segments intersect (excluding endpoints).
 * Works in local Cartesian coordinates (meters).
 *
 * @param p1  first segment start in local frame
 * @param p2  first segment end in local frame
 * @param v1  second segment start in local frame
 * @param v2  second segment end in local frame
 * @return true if the segments intersect
 */
bool segmentsIntersect(const matrix::Vector2f &p1, const matrix::Vector2f &p2,
		       const matrix::Vector2f &v1, const matrix::Vector2f &v2);

/**
 * Check if a line segment intersects a circle.
 * Works in local Cartesian coordinates (meters).
 *
 * @param start   segment start in local frame
 * @param end     segment end in local frame
 * @param center  circle center in local frame
 * @param radius  circle radius in meters
 * @return true if the segment intersects the circle
 */
bool lineSegmentIntersectsCircle(const matrix::Vector2f &start, const matrix::Vector2f &end,
				 const matrix::Vector2f &center, float radius);

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
 * Offset polygon vertices outward (expand) or inward (shrink) so that every face is at exactly
 * `margin` distance from the corresponding original face.
 *
 * At each vertex the offset point is moved by `margin / sin(alpha/2)` along the unit bisector,
 * which keeps both adjacent faces at distance `margin`. For interior angles below 90 degrees the
 * single bisector point would lie further than sqrt(2) * margin from the original vertex, so the
 * vertex is replaced by two vertices on the tangent line at distance `margin` from the original
 * vertex (one for each adjacent edge). This caps the maximum vertex displacement at sqrt(2) *
 * margin and keeps the resulting polygon a piecewise-linear approximation of the Minkowski offset
 * curve.
 *
 * Output buffer must hold at least `2 * num_vertices` elements; the actual count is written to
 * *num_vertices_out.
 *
 * Returns false for degenerate polygons: fewer than 3 vertices, zero-length edges,
 * or antiparallel adjacent edges (polygon doubles back).
 *
 * @param vertices_in       input vertices in local frame
 * @param num_vertices      number of input vertices
 * @param margin            offset distance in meters
 * @param expand            true to expand, false to shrink
 * @param vertices_out      output array (must hold at least 2 * num_vertices elements)
 * @param num_vertices_out  output: number of vertices actually written
 */
bool expandOrShrinkPolygon(const matrix::Vector2f *vertices_in, int num_vertices,
			   float margin, bool expand,
			   matrix::Vector2f *vertices_out, int *num_vertices_out);

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
