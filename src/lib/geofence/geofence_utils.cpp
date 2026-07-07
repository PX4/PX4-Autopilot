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

int32_t metersToCm(float m) { return roundf(m * CM_PER_M); }

} // namespace


void PlannerPolygons::setNode(int idx, const matrix::Vector2f &p)
{
	idx = math::constrain(idx, 0, kMaxNodes - 1);

	_x_cm[idx] = metersToCm(p(0));
	_y_cm[idx] = metersToCm(p(1));
}

PlannerPolygons::AddResult PlannerPolygons::addPolygon(const matrix::Vector2f *vertices_in, int num_vertices,
		bool is_inclusion_zone, float margin)
{
	if (_num_polygons >= kMaxPolygons || _num_nodes + num_vertices > kMaxNodes) {
		return AddResult::BudgetExceeded;
	}

	if (margin < -FLT_EPSILON || num_vertices < 3) { return AddResult::Degenerate; }

	for (int i = 0; i < num_vertices; ++i) {
		// Reject self-intersecting (non-simple) polygons.
		const matrix::Vector2f &a = vertices_in[i];
		const matrix::Vector2f &b = vertices_in[(i + 1) % num_vertices];

		for (int j = i + 2; j < num_vertices; ++j) {
			if (i == 0 && j == num_vertices - 1) {
				continue; // (last edge, first edge) are adjacent
			}

			const matrix::Vector2f &c = vertices_in[j];
			const matrix::Vector2f &d = vertices_in[(j + 1) % num_vertices];

			if (segmentsIntersect(
				    metersToCm(a(0)), metersToCm(a(1)), metersToCm(b(0)), metersToCm(b(1)),
				    metersToCm(c(0)), metersToCm(c(1)), metersToCm(d(0)), metersToCm(d(1))) != SegSegResult::Disjoint) {
				return AddResult::Degenerate;
			}
		}

		// Reject vertices outside the usable fixed-point range
		if (!inFixedPointRange(vertices_in[i](0)) || !inFixedPointRange(vertices_in[i](1))) {
			return AddResult::OutOfRange;
		}
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
	poly.start_index  = _num_nodes;
	poly.is_inclusion = is_inclusion_zone;

	int out_idx = 0;

	for (int i = 0; i < num_vertices; i++) {
		const int prev = (i + num_vertices - 1) % num_vertices;
		const int next = (i + 1) % num_vertices;

		const matrix::Vector2f edge_in  = vertex(i) - vertex(prev);
		const matrix::Vector2f edge_out = vertex(next) - vertex(i);

		const float edge_in_norm = edge_in.norm();
		const float edge_out_norm = edge_out.norm();

		if (edge_in_norm < FLT_EPSILON || edge_out_norm < FLT_EPSILON) {
			return AddResult::Degenerate;
		}

		const matrix::Vector2f edge_in_normalized = edge_in / edge_in_norm;
		const matrix::Vector2f edge_out_normalized = edge_out / edge_out_norm;

		// Inward normals of the two adjacent edges.
		// Canonical stored orientation: left side = forbidden region, right side = allowed region
		const matrix::Vector2f n_in  = matrix::Vector2f{-edge_in_normalized(1), edge_in_normalized(0)};
		const matrix::Vector2f n_out = matrix::Vector2f{-edge_out_normalized(1), edge_out_normalized(0)};

		matrix::Vector2f bisector = n_in + n_out;
		const float bisector_len  = bisector.length();

		if (bisector_len < FLT_EPSILON) {
			return AddResult::Degenerate; // antiparallel edges - polygon doubles back
		}

		const matrix::Vector2f normalized_bisector = bisector / bisector_len;

		// The following bit ensures that:
		//  - The original edges are moved away from the forbidden interior by `margin`
		//  - Sharp convex corners (<60deg) are "cut off" with a new edge that is exactly
		//    `margin` away from the original vertex. For this the vertex is replaced by
		//    two new ones.

		// Only split convex corners - others are ignored, see _node_not_on_optimal_path
		const bool corner_convex = edge_in_normalized(0) * edge_out_normalized(1)
					   - edge_in_normalized(1) * edge_out_normalized(0) > 0.f;

		// Scaling needed wrt the normalised bisector to achieve parallel edges
		const float offset_scale = 2.f / bisector_len;

		// bisector_len = 2*sin(interior angle/2); split when interior angle < 60 deg <=> bisector_len < 1.
		const bool angle_sharp = bisector_len < 1;

		// If margin zero, do not split - would output the same vertex twice.
		const bool margin_nonzero = margin > FLT_EPSILON;

		// If we do not have enough space, do not split the vertex.
		// This never happens if the planner-internal node buffer is 2x the original buffer.
		const bool space_for_split_vertices = _num_nodes + out_idx + 1 < kMaxNodes;

		if (corner_convex && angle_sharp && margin_nonzero && space_for_split_vertices) {

			// Sharp corner: cut it off along the line
			//  - perpendicular to normalized_bisector
			//  - going through base := vertex(i) - normalized_bisector * margin

			const matrix::Vector2f base = vertex(i) - normalized_bisector * margin;
			const matrix::Vector2f perp = {-normalized_bisector(1), normalized_bisector(0)};

			// Construct s such that:
			//  base + perp*s is on the offset incoming edge
			//  base - perp*s is on the offset outgoing edge
			const float e_out_dot_b = edge_out_normalized.dot(normalized_bisector);
			const float s = margin * (1.f - n_out.dot(normalized_bisector)) / e_out_dot_b;

			setNode(poly.start_index + out_idx++, base + perp * s);
			setNode(poly.start_index + out_idx++, base - perp * s);

		} else {
			// Regular corner or no space for split: single bisector vertex displaced by margin / sin(interior angle/2).
			setNode(poly.start_index + out_idx++, vertex(i) - normalized_bisector * (margin * offset_scale));
		}
	}

	const int num_out = out_idx;
	poly.num_vertices = num_out;

	computeBoundingBox(poly.start_index, num_out, poly.min_x, poly.max_x, poly.min_y, poly.max_y);

	// In canonical orientation, the node is not possibly on the shortest
	// path if the vertex curves right or straight (forbidden region >= 180
	// deg)
	for (int i = 0; i < num_out; i++) {
		const int prev = poly.start_index + (i + num_out - 1) % num_out;
		const int curr = poly.start_index + i;
		const int next = poly.start_index + (i + 1) % num_out;
		const bool curves_right_or_straight = orient2d(_x_cm[prev], _y_cm[prev],
						      _x_cm[curr], _y_cm[curr],
						      _x_cm[next], _y_cm[next]) <= 0;
		_node_not_on_optimal_path[curr] = curves_right_or_straight;
	}

	_num_nodes += num_out;
	++_num_polygons;
	return AddResult::Success;
}

PlannerPolygons::AddResult PlannerPolygons::addApproxCircle(const matrix::Vector2f &center, const float circle_radius,
		float margin, const bool is_inclusion_zone)
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

	if (_num_polygons >= kMaxPolygons || _num_nodes + kCircleApproxVertices > kMaxNodes) {
		return AddResult::BudgetExceeded;
	}

	if (circle_radius <= 0.1f) { return AddResult::Degenerate; }

	// ensures that the k-gon is, w.r.t the real circle:
	//  - over-approximation of circle grown by margin, if exclusion
	//  - under-approximation of circle shrunk by margin, if inclusion
	const float k_gon_radius = is_inclusion_zone
				   ? circle_radius - margin
				   : circle_radius / cosf(M_PI_F / kCircleApproxVertices) + margin;

	if (k_gon_radius <= FLT_EPSILON) { // Negative radius - circle empty
		return AddResult::Degenerate;
	}

	if (!inFixedPointRange(fabsf(center(0)) + k_gon_radius)
	    || !inFixedPointRange(fabsf(center(1)) + k_gon_radius)) {
		return AddResult::OutOfRange;
	}

	// Canonical orientation: CCW for exclusion, CW for inclusion
	const float delta_angle = (is_inclusion_zone ? -1.f : 1.f) * 2.f * M_PI_F / kCircleApproxVertices;

	PolygonInfo &poly = _polygons[_num_polygons];
	poly.start_index = _num_nodes;
	poly.num_vertices = kCircleApproxVertices;
	poly.is_inclusion = is_inclusion_zone;

	for (int i = 0; i < kCircleApproxVertices; i++) {
		const float angle = (i + 0.5f) * delta_angle;
		const matrix::Vector2f p = center + matrix::Vector2f{
			k_gon_radius * cosf(angle), k_gon_radius * sinf(angle)
		};
		setNode(poly.start_index + i, p);
	}

	computeBoundingBox(poly.start_index, kCircleApproxVertices, poly.min_x, poly.max_x, poly.min_y, poly.max_y);

	// For regular k-gons, all corners are either convex or reflex based on
	// is_inclusion, so the reduced visibility graph criterion is simplified
	for (int i = 0; i < kCircleApproxVertices; i++) {
		_node_not_on_optimal_path[poly.start_index + i] = is_inclusion_zone;
	}

	_num_nodes += kCircleApproxVertices;
	++_num_polygons;
	return AddResult::Success;
}

void PlannerPolygons::computeBoundingBox(const int start_index, const int num_vertices,
		int32_t &min_x, int32_t &max_x, int32_t &min_y, int32_t &max_y)
{

	// Store the bounding box, so we can do a very cheap check against the
	// bounding box, and only do the full line-polygon intersection check if
	// the line intersects the bounding box.

	min_x = max_x = _x_cm[start_index];
	min_y = max_y = _y_cm[start_index];

	for (int i = 1; i < num_vertices; i++) {
		const int32_t x = _x_cm[start_index + i];
		const int32_t y = _y_cm[start_index + i];

		if (x < min_x) { min_x = x; }

		if (x > max_x) { max_x = x; }

		if (y < min_y) { min_y = y; }

		if (y > max_y) { max_y = y; }
	}
}

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
	// Single pass over polygon edges, doing two jobs:
	//  (1) classify the test segment vs each polygon edge. early-return on:
	//       - strict crossings (line segment - polygon edge)
	//       - interior crossing at vertex
	//  (2) accumulate midpoint winding contribution -- if (1) is inconclusive,
	//      this decides if the line is inside or outside

	int wn = 0;
	bool mid_on_boundary = false;

	// Rather than calculating the midpoint (mid = (s + e)/2) explicitly,
	// which will lead to rounding error for uneven numbers, we instead
	// scale all other inputs by two, reducing the range by a factor of two
	// (2^31 cm = 21400 km -> 2^30 cm = 10700 km)
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

		// skip AInsideCD - a from this segment is b from the adjacent segment
		case SegSegResult::BInsideCD:

			// Polygon vertex b is exactly on the interior of the test segment.
			// Return intersecting if it pokes into the polygon interior through vertex b.
			if (pointInsideInteriorCone(poly, s_x, s_y, i) || pointInsideInteriorCone(poly, e_x, e_y, i)) {
				return true;
			}

			break;

		default:
			break;
		}

		// (2) midpoint winding contribution
		// Ref: Dan Sunday, Inclusion of a Point in a Polygon
		// https://web.archive.org/web/20130126163405/http://geomalgorithms.com/a03-_inclusion.html

		const int side = orient2d(2 * ax, 2 * ay, 2 * bx, 2 * by, twice_mid_x, twice_mid_y);

		if (side == 0) {

			const bool mid_on_open_ab = collinearBetween(2 * ax, 2 * ay, 2 * bx, 2 * by, twice_mid_x, twice_mid_y);
			const bool mid_is_b = 2 * bx == twice_mid_x && 2 * by == twice_mid_y;

			// Skip point a - a from this segment is b from the adjacent segment
			if (mid_on_open_ab || mid_is_b) {
				mid_on_boundary = true;
			}

		} else if (2 * ay <= twice_mid_y) {
			if (2 * by > twice_mid_y && side > 0) { wn++; }

		} else if (2 * by <= twice_mid_y && side < 0) { wn--; }
	}


	// Midpoint on boundary, segment runs along it (strict crossing detected earlier).
	// Non-intersecting in either zone.
	if (mid_on_boundary) {
		return false;
	}

	// Exclusion: violation when midpoint is inside (bounded). Inclusion: when outside (unbounded).
	// This is the only place we depart from the inside = left = forbidden canonical orientation,
	// necessary because of the point membership check.
	const bool midpoint_in_bounded_inside = (wn != 0);
	return midpoint_in_bounded_inside != poly.is_inclusion;
}

bool PlannerPolygons::intersectsAnyInside(int32_t s_x, int32_t s_y, int32_t e_x, int32_t e_y) const
{
	const int32_t seg_min_x = s_x < e_x ? s_x : e_x;
	const int32_t seg_max_x = s_x > e_x ? s_x : e_x;
	const int32_t seg_min_y = s_y < e_y ? s_y : e_y;
	const int32_t seg_max_y = s_y > e_y ? s_y : e_y;

	for (int p = 0; p < _num_polygons; p++) {
		const PolygonInfo &poly = _polygons[p];

		if (seg_max_x < poly.min_x || seg_min_x > poly.max_x ||
		    seg_max_y < poly.min_y || seg_min_y > poly.max_y) {
			// Segment is entirely outside this polygon.
			// For exclusion zones: outside = safe, skip.
			// For inclusion zones: outside = forbidden region, immediate violation.
			if (poly.is_inclusion) { return true; }

			continue;
		}

		if (intersectsInsideOf(poly, s_x, s_y, e_x, e_y)) {
			return true;
		}
	}

	return false;
}

bool PlannerPolygons::edgeBitangent(int a, int32_t bx, int32_t by) const
{
	// Find the polygon owning vertex index a. Indices not belonging to any
	// polygon (in particular the destination at index 0) are trivially bitangent.

	if (a == destIndex()) { return true; }

	for (int p = 0; p < _num_polygons; p++) {
		const PolygonInfo &poly = _polygons[p];

		if (a >= poly.start_index && a < poly.start_index + poly.num_vertices) {
			// Define point c = a + (a-b) = 2a - b, so that ac = ab mirrored about a
			// Then: Edge is bitangent at a <=> neither ab nor ac pokes into the interior
			int32_t cx = 2 * _x_cm[a] - bx;
			int32_t cy = 2 * _y_cm[a] - by;
			return !pointInsideInteriorCone(poly, bx, by, a - poly.start_index)
			       && !pointInsideInteriorCone(poly, cx, cy, a - poly.start_index);
		}
	}

	return true;
}

bool PlannerPolygons::edgeBitangent(int a, int b) const
{
	return edgeBitangent(a, _x_cm[b], _y_cm[b]);
}

bool PlannerPolygons::edgeBitangent(int a, const matrix::Vector2f &b) const
{
	return edgeBitangent(a, metersToCm(b(0)), metersToCm(b(1)));
}

bool PlannerPolygons::setDestination(const matrix::Vector2f &p)
{
	setNode(destIndex(), p);

	const int32_t x_cm = metersToCm(p(0));
	const int32_t y_cm = metersToCm(p(1));

	if (intersectsAnyInside(x_cm, y_cm, x_cm, y_cm)) {
		// If the destination breaches a geofence, still use it but
		// signal to the user that it is invalid.
		return false;
	}

	return true;
}

matrix::Vector2f PlannerPolygons::getDestination() const
{
	return node(destIndex());
}

bool PlannerPolygons::edgeVisible(int a, int b) const
{
	if (a == b) { return false; }

	if (_node_not_on_optimal_path[a] || _node_not_on_optimal_path[b]) { return false; }

	// Only bitangent edges can be on a shortest path; this check also cheaply
	// discards edges that violate the polygon at the vertex they come from.
	if (!edgeBitangent(a, b) || !edgeBitangent(b, a)) { return false; }

	return !intersectsAnyInside(_x_cm[a], _y_cm[a], _x_cm[b], _y_cm[b]);
}

bool PlannerPolygons::edgeVisible(const matrix::Vector2f &a, int b) const
{
	if (_node_not_on_optimal_path[b]) { return false; }

	if (!edgeBitangent(b, a)) { return false; }

	return !intersectsAnyInside(metersToCm(a(0)), metersToCm(a(1)), _x_cm[b], _y_cm[b]);
}

bool PlannerPolygons::edgeVisible(const matrix::Vector2f &a, const matrix::Vector2f &b) const
{
	return !intersectsAnyInside(metersToCm(a(0)), metersToCm(a(1)), metersToCm(b(0)), metersToCm(b(1)));
}

float PlannerPolygons::edgeCost(int a, int b) const
{
	return edgeVisible(a, b) ? (node(a) - node(b)).norm() : INFINITY;
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

} // namespace geofence_utils
