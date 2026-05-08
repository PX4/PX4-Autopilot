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

// Ternary point-in-polygon classification used internally. OnBoundary is
// returned when the point lies exactly on a polygon edge or vertex.
enum class PointPolygonRelation { Inside, Outside, OnBoundary };

// Winding-number point-in-polygon test built on orient2d. Robust on the
// boundary (returns OnBoundary deterministically) and orientation-independent.
//
// Reference: Dan Sunday, "Inclusion of a Point in a Polygon"
// (https://web.archive.org/web/20130126163405/http://geomalgorithms.com/a03-_inclusion.html).
static PointPolygonRelation classifyPointInPolygon(const matrix::Vector2f *vs, int n,
		const matrix::Vector2f &p)
{
	int wn = 0;

	for (int i = 0; i < n; i++) {
		const matrix::Vector2f &a = vs[i];
		const matrix::Vector2f &b = vs[(i + 1) % n];

		const int side = orient2d(a, b, p);

		if (side == 0 && collinearBetween(a, b, p)) {
			return PointPolygonRelation::OnBoundary;
		}

		if (a(1) <= p(1)) {
			if (b(1) > p(1) && side > 0) { wn++; }

		} else if (b(1) <= p(1) && side < 0) { wn--; }
	}

	return wn != 0 ? PointPolygonRelation::Inside : PointPolygonRelation::Outside;
}

bool insideCircle(const matrix::Vector2<double> &center, float radius,
		  const matrix::Vector2<double> &point)
{
	float dist = get_distance_to_next_waypoint(point(0), point(1), center(0), center(1));
	return dist < radius;
}

// O'Rourke, "Computational Geometry in C" (2nd ed.), section 1.5: SegSegInt.
// True iff each segment strictly straddles the other's supporting line; this
// excludes endpoint-touching and collinear overlap.
bool segmentsIntersect(const matrix::Vector2f &a, const matrix::Vector2f &b,
		       const matrix::Vector2f &c, const matrix::Vector2f &d)
{
	const int o1 = orient2d(a, b, c);
	const int o2 = orient2d(a, b, d);
	const int o3 = orient2d(c, d, a);
	const int o4 = orient2d(c, d, b);

	return o1 && o2 && o3 && o4 && o1 != o2 && o3 != o4;
}

// Is point P strictly inside the CCW/CW interior wedge of polygon vertex V,
// where Vp and Vn are the previous and next polygon vertices?
//
// Reference: standard convex/reflex vertex test built from orient2d. P lies
// strictly to the left of both incident edges (Vp->V and V->Vn) for a convex
// vertex, or to the left of either one for a reflex vertex.
static bool pointInsideWedge(const matrix::Vector2f &P,
			     const matrix::Vector2f &V,
			     const matrix::Vector2f &Vp,
			     const matrix::Vector2f &Vn)
{
	const int o_in     = orient2d(Vp, V,  P);
	const int o_out    = orient2d(V,  Vn, P);
	const int is_convex = orient2d(Vp, V, Vn);

	return is_convex > 0 ? (o_in > 0 && o_out > 0) : (o_in > 0 || o_out > 0);
}

bool lineSegmentIntersectsPolygon(const matrix::Vector2f &start, const matrix::Vector2f &end,
				  const matrix::Vector2f *vertices, int num_vertices, bool is_inclusion_zone)
{
	// Walk polygon edges, looking for proper boundary crossings. A proper
	// crossing is a definite transition between inside and outside, which
	// violates either zone type.
	for (int i = 0; i < num_vertices; i++) {
		const int prev = (i == 0) ? num_vertices - 1 : i - 1;

		if (segmentsIntersect(vertices[prev], vertices[i], start, end)) {
			return true;
		}
	}

	// Polygon vertices touching the open segment do not by themselves prove a
	// crossing -- the segment may merely graze (run along an incident edge,
	// or skim a corner). For each such vertex, classify each segment endpoint
	// against the wedge: if start is strictly inside the wedge and end is
	// strictly outside (or vice versa), the segment really does cross the
	// boundary at V.
	for (int i = 0; i < num_vertices; i++) {
		if (orient2d(start, end, vertices[i]) != 0 ||
		    !collinearBetween(start, end, vertices[i]) ||
		    vertices[i] == start || vertices[i] == end) {
			continue;
		}

		const int prev = (i == 0) ? num_vertices - 1 : i - 1;
		const int next = (i + 1) % num_vertices;

		if (pointInsideWedge(start, vertices[i], vertices[prev], vertices[next]) !=
		    pointInsideWedge(end,   vertices[i], vertices[prev], vertices[next])) {
			return true;
		}
	}

	// No crossings: the segment lies entirely on one side of the boundary
	// (or along it). Sample start, midpoint, end -- one of them will reveal
	// the side unless the segment runs exactly along the boundary throughout
	// (in which case OnBoundary at all three is the correct, allowed answer).
	const matrix::Vector2f samples[3] = {
		start, 0.5f * (start + end), end
	};

	for (const matrix::Vector2f &p : samples) {
		const PointPolygonRelation r = classifyPointInPolygon(vertices, num_vertices, p);

		if ((is_inclusion_zone && r == PointPolygonRelation::Outside) ||
		    (!is_inclusion_zone && r == PointPolygonRelation::Inside)) {
			return true;
		}
	}

	return false;
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
