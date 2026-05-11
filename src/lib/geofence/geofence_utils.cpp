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

#include "geofence_utils.h"
#include <lib/geo/geo.h>

namespace geofence_utils
{

namespace
{

// Float-meter to int32-cm: the only conversion at the float/cm boundary.
int32_t metersToCm(float m) { return static_cast<int32_t>(std::llroundf(m * 100.f)); }

} // namespace

bool insideCircle(const matrix::Vector2<double> &center, float radius,
		  const matrix::Vector2<double> &point)
{
	float dist = get_distance_to_next_waypoint(point(0), point(1), center(0), center(1));
	return dist < radius;
}

int PlannerPolygons::addNode(const matrix::Vector2f &p)
{
	if (_num_nodes >= kMaxNodes) { return -1; }

	setNode(_num_nodes, p);
	return _num_nodes++;
}

void PlannerPolygons::setNode(int idx, const matrix::Vector2f &p)
{
	_x_cm[idx] = metersToCm(p(0));
	_y_cm[idx] = metersToCm(p(1));
}

bool PlannerPolygons::addPolygon(const matrix::Vector2f *vertices_in, int num_vertices, bool is_inclusion_zone)
{
	if (num_vertices < 3
	    || _num_polygons >= kMaxPolygons
	    || _num_nodes + num_vertices > kMaxNodes) {
		return false;
	}

	// Canonical orientation: walking vertices in stored order, INSIDE is on
	// the left of each edge. CCW for exclusion (bounded interior is forbidden),
	// CW for inclusion (unbounded exterior is forbidden). Reverse iff input
	// doesn't already match.
	const bool reverse = (isPolygonCCW(vertices_in, num_vertices) == is_inclusion_zone);

	PolygonInfo &poly = _polygons[_num_polygons];
	poly.start_index = _num_nodes;
	poly.num_vertices = num_vertices;
	poly.inside_is_bounded = !is_inclusion_zone;

	for (int i = 0; i < num_vertices; i++) {
		setNode(poly.start_index + i, vertices_in[reverse ? (num_vertices - 1 - i) : i]);
	}

	_num_nodes += num_vertices;
	++_num_polygons;
	return true;
}

// Standard convex/reflex vertex test: at a convex vertex Inside is the small
// wedge between the two incident edges (left of BOTH); at a reflex vertex it
// is the large arc (left of EITHER).
bool PlannerPolygons::pointInsideInteriorCone(const PolygonInfo &poly,
		int32_t px, int32_t py, int v) const
{
	const int prev = poly.start_index + ((v == 0) ? poly.num_vertices - 1 : v - 1);
	const int curr = poly.start_index + v;
	const int next = poly.start_index + ((v + 1) % poly.num_vertices);

	const int o_in     = orient2d(_x_cm[prev], _y_cm[prev], _x_cm[curr], _y_cm[curr], px, py);
	const int o_out    = orient2d(_x_cm[curr], _y_cm[curr], _x_cm[next], _y_cm[next], px, py);
	const int is_convex = orient2d(_x_cm[prev], _y_cm[prev], _x_cm[curr], _y_cm[curr], _x_cm[next], _y_cm[next]);

	return is_convex > 0              // If convex == 0 (exact 180deg vertex) the below cases are equivalent
	       ? (o_in > 0 && o_out > 0)  // Inside = intersection of both half-planes
	       : (o_in > 0 || o_out > 0); // Inside = union of both half-planes
}

bool PlannerPolygons::intersectsInsideOf(const PolygonInfo &poly,
		int32_t s_x, int32_t s_y, int32_t e_x, int32_t e_y) const
{
	// Single pass over polygon edges, doing two jobs at once:
	//   (1) classify the test segment vs each polygon edge -- early-return on
	//       a proper crossing, wedge-check polygon vertices that sit strictly
	//       on the open test segment;
	//   (2) accumulate Sunday's winding-number contribution for the midpoint,
	//       used (with poly.inside_is_bounded) to classify the no-crossing
	//       case as Inside or Outside.
	int wn = 0;
	bool mid_on_boundary = false;

	const int32_t twice_mid_x = s_x + e_x;
	const int32_t twice_mid_y = s_y + e_y;

	for (int i = 0; i < poly.num_vertices; i++) {
		const int prev_idx = poly.start_index + ((i == 0) ? poly.num_vertices - 1 : i - 1);
		const int curr_idx = poly.start_index + i;
		const int32_t ax = _x_cm[prev_idx], ay = _y_cm[prev_idx];
		const int32_t bx = _x_cm[curr_idx], by = _y_cm[curr_idx];

		// (1) test segment vs polygon edge (a, b)
		switch (segmentsIntersect(ax, ay, bx, by, s_x, s_y, e_x, e_y)) {
		case SegSegResult::Cross:
			return true;

		case SegSegResult::BInsideCD:

			// polygon vertex `b` strictly on the open test segment -- a graze
			// unless the wedge classifies the two endpoints differently
			if (pointInsideInteriorCone(poly, s_x, s_y, i) !=
			    pointInsideInteriorCone(poly, e_x, e_y, i)) {
				return true;
			}

			break;

		default:
			break;
		}

		// (2) midpoint winding contribution (Sunday)

		// Rather than calculating the midpoint (mid = (s + e)/2)
		// explicitly, which will lead to rounding error for uneven
		// numbers, we instead scale all other inputs by two, reducing
		// the range by a factor of two (2^31 cm = 21400 km -> 2^30 cm =
		// 10700 km)

		const int side = orient2d(2 * ax, 2 * ay, 2 * bx, 2 * by, twice_mid_x, twice_mid_y);

		if (side == 0 && collinearBetween(2 * ax, 2 * ay, 2 * bx, 2 * by, twice_mid_x, twice_mid_y)) {
			mid_on_boundary = true;

		} else if (2 * ay <= twice_mid_y) {
			if (2 * by > twice_mid_y && side > 0) { wn++; }

		} else if (2 * by <= twice_mid_y && side < 0) { wn--; }
	}

	// Midpoint on the boundary: segment runs along it; non-violating in either zone.
	if (mid_on_boundary) {
		return false;
	}

	// wn != 0 means the midpoint is in the bounded region. The only place
	// orientation polarity surfaces.
	const bool bounded = (wn != 0);
	return bounded == poly.inside_is_bounded;
}

bool PlannerPolygons::intersectsAnyInside(int32_t s_x, int32_t s_y, int32_t e_x, int32_t e_y) const
{
	for (int p = 0; p < _num_polygons; p++) {
		if (intersectsInsideOf(_polygons[p], s_x, s_y, e_x, e_y)) {
			return true;
		}
	}

	return false;
}

bool PlannerPolygons::isLineBetweenNodesIntersectingAnyInside(int a, int b) const
{
	if (a == b) { return false; }

	return intersectsAnyInside(_x_cm[a], _y_cm[a], _x_cm[b], _y_cm[b]);
}

bool PlannerPolygons::isLineFromPointToNodeIntersectingAnyInside(const matrix::Vector2f &p, int node_idx) const
{
	return intersectsAnyInside(metersToCm(p(0)), metersToCm(p(1)), _x_cm[node_idx], _y_cm[node_idx]);
}

bool lineSegmentIntersectsCircle(const matrix::Vector2f &start, const matrix::Vector2f &end,
				 const matrix::Vector2f &center, float radius)
{
	// Algorithm borrowed from here: https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
	// Basic idea: Find the closest point on the line segment from the circle center.
	// If that point is outside the circle, we don't have an intersection. If the point is inside the
	// radius then we just need to check if either the start or the end of the line segment is outside the circle.


	// let vector from start to circle center be A and vector from start to end be B
	// then the projection of A onto B gives the closest point on the line from the circle center.
	// By constraining the projection length to [0, |B|] we make sure that we find the closest point on the
	// line segment and not the infinite line that coincides with B.
	const matrix::Vector2f A = center - start;
	const matrix::Vector2f B = end - start;
	const float B_length = B.norm();

	if (B_length < FLT_EPSILON) {
		return false;
	}

	const float projection_A_on_B = math::constrain(A.dot(B) / B_length, 0.f, B_length);
	const matrix::Vector2f closest = start + projection_A_on_B * B.normalized();

	// closest point is not even inside the radius, so no intersection
	if ((closest - center).norm() >= radius) {
		return false;
	}

	// we have an intersection if at least one of start or end is further away than radius from center
	return (start - center).norm() >= radius || (end - center).norm() >= radius;
}

bool isPolygonCCW(const matrix::Vector2f *vertices, int num_vertices)
{
	// https://en.wikipedia.org/wiki/Shoelace_formula
	// Triangle formula
	float signed_area_2x = 0.f;

	for (int i = 0; i < num_vertices; i++) {
		const int j = (i + 1) % num_vertices;
		signed_area_2x += vertices[i](0) * vertices[j](1)
				  - vertices[j](0) * vertices[i](1);
	}

	return signed_area_2x > 0.f;
}

bool expandOrShrinkPolygon(const matrix::Vector2f *vertices_in, int num_vertices,
			   float margin, bool expand,
			   matrix::Vector2f *vertices_out)
{
	// Algorithm which shrinks or expands a polygon with given vertices.
	// It calculates the inward vector at each vertex as the sum of the inward normals of the two adjacent edges
	// The vertex of the new polygon is then moved by margin along the inward vector, either positively (shrink) or
	// negatively (expand). The inward vector of an edge can be easily found if the polygon orientation (CW or CCW),
	// is known. We use the triangle formula to find the polygon orientation.

	if (num_vertices < 3) {
		return false;
	}

	// If polygon is CCW we rotate the edge vector to the left to get the inward normal.
	const float rot_sign = isPolygonCCW(vertices_in, num_vertices) ? 1.f : -1.f;

	// Expand pushes vertices outward, shrink pushes them inward.
	const float step_sign = expand ? -1.f : 1.f;

	for (int i = 0; i < num_vertices; i++) {
		const int prev = (i + num_vertices - 1) % num_vertices;
		const int next = (i + 1) % num_vertices;

		const matrix::Vector2f edge_in  = vertices_in[i] - vertices_in[prev];
		const matrix::Vector2f edge_out = vertices_in[next] - vertices_in[i];

		if (edge_in.norm() < FLT_EPSILON || edge_out.norm() < FLT_EPSILON) {
			return false;
		}

		const matrix::Vector2f edge_in_unit  = edge_in.normalized();
		const matrix::Vector2f edge_out_unit = edge_out.normalized();

		// Unit inward normals of the two adjacent edges.
		const matrix::Vector2f n_in {-rot_sign * edge_in_unit(1), rot_sign * edge_in_unit(0)};
		const matrix::Vector2f n_out{-rot_sign * edge_out_unit(1), rot_sign * edge_out_unit(0)};

		matrix::Vector2f bisector = n_in + n_out;
		const float bisector_len = bisector.length();

		if (bisector_len < FLT_EPSILON) {
			// degenerate case, edges are antiparallel
			return false;
		}

		bisector.normalize();

		vertices_out[i] = vertices_in[i] + bisector * margin * step_sign;
	}

	return true;
}

size_t symmetricPairIndex(size_t i, size_t j, size_t num_nodes)
{
	// Pack the strict upper triangle (i < j) of an N x N symmetric matrix into a flat array.
	// https://stackoverflow.com/questions/27086195/linear-index-upper-triangular-matrix
	if (i > j) {
		const size_t tmp = i;
		i = j;
		j = tmp;
	}

	return i * (2 * num_nodes - i - 1) / 2 + (j - i - 1);
}

} // namespace geofence_utils
