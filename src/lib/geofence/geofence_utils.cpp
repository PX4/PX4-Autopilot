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

bool PlannerPolygons::addPolygon(const matrix::Vector2f *vertices_in, int num_vertices,
				 bool is_inclusion_zone, float margin)
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

	// Index into vertices_in in stored order (accounts for possible reversal).
	auto vertex = [&](int i) -> const matrix::Vector2f & {
		return vertices_in[reverse ? (num_vertices - 1 - i) : i];
	};

	PolygonInfo &poly = _polygons[_num_polygons];
	poly.start_index    = _num_nodes;
	poly.num_vertices   = num_vertices;
	poly.is_inclusion = is_inclusion_zone;

	if (margin < -FLT_EPSILON) {
		// Negative margin makes no sense, would move vertices into illegal region
		// Zero margin might be desired sometimes
		return false;
	}

	// Canonical stored orientation: left normal of each edge points inward for
	// exclusion (CCW), outward for inclusion (CW) -- flip for inclusion.
	const float inward_sign = is_inclusion_zone ? -1.f : 1.f;

	for (int i = 0; i < num_vertices; i++) {
		const int prev = (i + num_vertices - 1) % num_vertices;
		const int next = (i + 1) % num_vertices;

		const matrix::Vector2f edge_in  = vertex(i) - vertex(prev);
		const matrix::Vector2f edge_out = vertex(next) - vertex(i);

		const float edge_in_norm = edge_in.norm();
		const float edge_out_norm = edge_out.norm();

		if (edge_in_norm < FLT_EPSILON || edge_out_norm < FLT_EPSILON) {
			return false;
		}

		const matrix::Vector2f edge_in_normalized = edge_in / edge_in_norm;
		const matrix::Vector2f edge_out_normalized = edge_out / edge_out_norm;

		// Inward normals of the two adjacent edges.
		const matrix::Vector2f n_in  = inward_sign * matrix::Vector2f{-edge_in_normalized(1), edge_in_normalized(0)};
		const matrix::Vector2f n_out = inward_sign * matrix::Vector2f{-edge_out_normalized(1), edge_out_normalized(0)};

		matrix::Vector2f bisector = n_in + n_out;
		const float bisector_len  = bisector.length();

		if (bisector_len < FLT_EPSILON) {
			return false; // antiparallel edges - polygon doubles back
		}

		bisector /= bisector_len;
		// Exclusion: expand outward (against inward bisector). Inclusion: shrink inward (along it).
		setNode(poly.start_index + i, vertex(i) - inward_sign * margin * bisector);
	}


	_num_nodes += num_vertices;
	++_num_polygons;
	return true;
}

bool PlannerPolygons::addApproxCircle(const matrix::Vector2f &center, const float circle_radius, float margin, const int num_vertices,
				      const bool is_inclusion_zone)
{
	// For planning we approximate circles by regular k-gons. This adds
	// quite some nodes and additional restricted area. However, planning
	// properly around circles would be more complicated. Read these if that
	// becomes a priority regardless:
	//
	//  - https://dl.acm.org/doi/epdf/10.1145/323233.323261
	//  - https://www.sciencedirect.com/science/article/pii/S0925772106000496
	//
	// Keep in mind that the added complexity would extend beyond the
	// planner into all controllers, which would then have to also fly
	// loiter segments, not just pure waypoint sequences.

	if (_num_nodes + num_vertices > kMaxNodes || _num_polygons >= kMaxPolygons || circle_radius <= 0.1f) {
		return false;  // Not enough space
	}

	// ensures that the k-gon is, w.r.t the real circle:
	//  - over-approximation, if exclusion
	//  - under-approximation, if inclusion
	const float k_gon_radius = is_inclusion_zone
				   ? circle_radius - margin
				   : circle_radius / cosf(M_PI_F / num_vertices) + margin;

	if (k_gon_radius <= FLT_EPSILON) {
		return false;
	}

	// CCW for exclusion, CW for inclusion — same convention as addPolygon.
	const float delta_angle = (is_inclusion_zone ? -1.f : 1.f) * 2.f * M_PI_F / num_vertices;

	PolygonInfo &poly = _polygons[_num_polygons];
	poly.start_index = _num_nodes;
	poly.num_vertices = num_vertices;
	poly.is_inclusion = is_inclusion_zone;

	for (int i = 0; i < num_vertices; i++) {
		const float angle = i * delta_angle;
		const matrix::Vector2f p = center + matrix::Vector2f{
			k_gon_radius * cosf(angle), k_gon_radius * sinf(angle)
		};
		setNode(poly.start_index + i, p);
	}

	_num_nodes += num_vertices;
	++_num_polygons;
	return true;
}


/**
 * Consider the cone obtained by extending the triangle poly[prev] -> poly[curr]
 * -> poly[next] outward from poly[curr] (the inside always being left of the
 * points). This cone is the local interior of the forbidden region at the given
 * vertex.
 *
 * Returns true if the point p is in the interior of that cone.
 */
bool PlannerPolygons::pointInsideInteriorCone(const PolygonInfo &poly,
		int32_t px, int32_t py, int v) const
{
	const int prev = poly.start_index + ((v + poly.num_vertices - 1) % poly.num_vertices);
	const int curr = poly.start_index + v;
	const int next = poly.start_index + ((v + 1) % poly.num_vertices);

	const int p_left_of_incoming  = orient2d(_x_cm[prev], _y_cm[prev], _x_cm[curr], _y_cm[curr], px, py);
	const int p_left_of_outgoing  = orient2d(_x_cm[curr], _y_cm[curr], _x_cm[next], _y_cm[next], px, py);
	const int is_convex           = orient2d(_x_cm[prev], _y_cm[prev], _x_cm[curr], _y_cm[curr], _x_cm[next], _y_cm[next]);

	return is_convex > 0  // If convex==0 (exact 180 deg "corner") the two cases are equivalent
	       ? (p_left_of_incoming > 0 && p_left_of_outgoing > 0)   // Convex: interior = intersection of two half-planes
	       : (p_left_of_incoming > 0 || p_left_of_outgoing > 0);  // Reflex: interior = union of two half-planes
}

bool PlannerPolygons::intersectsInsideOf(const PolygonInfo &poly,
		int32_t s_x, int32_t s_y, int32_t e_x, int32_t e_y) const
{
	// Single pass over polygon edges, doing two jobs at once:
	//   (1) classify the test segment vs each polygon edge -- early-return on
	//       a proper crossing, wedge-check polygon vertices that sit strictly
	//       on the open test segment;
	//   (2) accumulate Sunday's winding-number contribution for the midpoint,
	//       used (with poly.is_inclusion) to classify the no-crossing
	//       case as Inside or Outside.
	int wn = 0;
	bool mid_on_boundary = false;

	const int32_t twice_mid_x = s_x + e_x;
	const int32_t twice_mid_y = s_y + e_y;

	for (int i = 0; i < poly.num_vertices; i++) {

		const int prev_idx = poly.start_index + (i + poly.num_vertices - 1) % poly.num_vertices;
		const int curr_idx = poly.start_index + i;

		const int32_t ax = _x_cm[prev_idx], ay = _y_cm[prev_idx];
		const int32_t bx = _x_cm[curr_idx], by = _y_cm[curr_idx];

		// (1) test segment vs polygon edge (a, b)
		switch (segmentsIntersect(ax, ay, bx, by, s_x, s_y, e_x, e_y)) {
		case SegSegResult::Cross:
			return true;

		// skip AInsideCD - A from this segment is B from the adjacent segment
		case SegSegResult::BInsideCD: {

				// Polygon vertex b is exactly on the interior of the test segment.
				// Detect if it passes from outside to inside through the inside.
				//  - If yes, conclusive
				//  - If no, record that we have grazed a vertex
				const bool s_inside = pointInsideInteriorCone(poly, s_x, s_y, i);
				const bool e_inside = pointInsideInteriorCone(poly, e_x, e_y, i);

				if (s_inside || e_inside) {
					// If either endpoint inside interior cone, interior is crossed
					return true;
				}

				break;
			}

		default:
			break;
		}

		// (2) midpoint winding contribution (Sunday)

		// Rather than calculating the midpoint (mid = (s + e)/2)
		// explicitly, which will lead to rounding error for uneven
		// numbers, we instead scale all other inputs by two, reducing
		// the range by a factor of two (2^31 cm = 21400 km -> 2^30 cm =
		// 10700 km)

		// TODO it would be awesome to have this part more readable

		const int side = orient2d(2 * ax, 2 * ay, 2 * bx, 2 * by, twice_mid_x, twice_mid_y);

		if (side == 0 && collinearBetween(2 * ax, 2 * ay, 2 * bx, 2 * by, twice_mid_x, twice_mid_y)) {
			mid_on_boundary = true;

		} else if (2 * ay <= twice_mid_y) {
			if (2 * by > twice_mid_y && side > 0) { wn++; }

		} else if (2 * by <= twice_mid_y && side < 0) { wn--; }
	}


	// Midpoint on boundary, segment runs along it (strict crossing detected earlier).
	// Non-intersecting in either zone.
	if (mid_on_boundary) {
		return false;
	}

	// wn != 0 means the midpoint is in the bounded region. The only place
	// orientation polarity surfaces.
	const bool bounded = (wn != 0);
	// Exclusion: violation when midpoint is inside (bounded). Inclusion: when outside (unbounded).
	return bounded != poly.is_inclusion;
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
