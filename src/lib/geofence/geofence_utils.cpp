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
// Classifies a segment-segment intersection from the four orient2d signs of
// the endpoints. No asymmetric strict/non-strict tolerance convention is
// baked in -- the caller decides which variants count as "intersecting".
SegSegResult segmentsIntersect(const matrix::Vector2f &a, const matrix::Vector2f &b,
			       const matrix::Vector2f &c, const matrix::Vector2f &d)
{
	const int o1 = orient2d(a, b, c);
	const int o2 = orient2d(a, b, d);
	const int o3 = orient2d(c, d, a);
	const int o4 = orient2d(c, d, b);

	// Each segment strictly straddles the other's supporting line.
	if (o1 && o2 && o3 && o4 && o1 != o2 && o3 != o4) {
		return SegSegResult::Proper;
	}

	// All four points collinear: overlap iff any endpoint of one lies on the other.
	if (!o1 && !o2 && !o3 && !o4) {
		if (collinearBetween(a, b, c) || collinearBetween(a, b, d) ||
		    collinearBetween(c, d, a) || collinearBetween(c, d, b)) {
			return SegSegResult::CollinearOverlap;
		}

		return SegSegResult::None;
	}

	// One endpoint sits on the other segment.
	if ((!o1 && collinearBetween(a, b, c)) ||
	    (!o2 && collinearBetween(a, b, d)) ||
	    (!o3 && collinearBetween(c, d, a)) ||
	    (!o4 && collinearBetween(c, d, b))) {
		return SegSegResult::Touching;
	}

	return SegSegResult::None;
}

bool lineSegmentIntersectsPolygon(const matrix::Vector2f &start, const matrix::Vector2f &end,
				  const matrix::Vector2f *vertices, int num_vertices, bool is_inclusion_zone)
{
	// Walk polygon edges, looking for proper boundary crossings. A proper
	// crossing is a definite transition between inside and outside, which
	// violates either zone type.
	for (int i = 0; i < num_vertices; i++) {
		const int prev = (i == 0) ? num_vertices - 1 : i - 1;

		if (segmentsIntersect(vertices[prev], vertices[i], start, end) == SegSegResult::Proper) {
			return true;
		}
	}

	// Polygon vertices touching the open segment do not by themselves prove a
	// crossing -- the segment may merely graze (run along an incident edge,
	// or skim a corner from outside). Record those as split points and below
	// classify each sub-segment by sampling its midpoint.
	const matrix::Vector2f delta = end - start;
	const float delta_norm_sq = delta.dot(delta);

	float vertex_hit_params[num_vertices];
	int num_hits = 0;

	if (delta_norm_sq >= FLT_EPSILON) {
		for (int i = 0; i < num_vertices; i++) {
			if (orient2d(start, end, vertices[i]) != 0) { continue; }

			const float s = delta.dot(vertices[i] - start) / delta_norm_sq;

			if (s > FLT_EPSILON && s < 1.0f - FLT_EPSILON) {
				vertex_hit_params[num_hits++] = s;
			}
		}
	}

	// Insertion sort -- num_hits is small.
	for (int i = 1; i < num_hits; i++) {
		const float k = vertex_hit_params[i];
		int j = i - 1;

		while (j >= 0 && vertex_hit_params[j] > k) {
			vertex_hit_params[j + 1] = vertex_hit_params[j];
			j--;
		}

		vertex_hit_params[j + 1] = k;
	}

	// Each sub-segment between consecutive split points lies wholly on one
	// side of the polygon boundary (or along it). Sample the midpoint.
	// OnBoundary is treated as allowed for both zone types -- a segment
	// running exactly along the fence line is not a violation.
	float prev_s = 0.0f;

	for (int i = 0; i <= num_hits; i++) {
		const float next_s = (i == num_hits) ? 1.0f : vertex_hit_params[i];
		const matrix::Vector2f sample = start + (0.5f * (prev_s + next_s)) * delta;
		const PointPolygonRelation r = classifyPointInPolygon(vertices, num_vertices, sample);

		if ((is_inclusion_zone && r == PointPolygonRelation::Outside) ||
		    (!is_inclusion_zone && r == PointPolygonRelation::Inside)) {
			return true;
		}

		prev_s = next_s;
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
