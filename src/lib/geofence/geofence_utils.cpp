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

bool insidePolygon(const matrix::Vector2f *vertices, int num_vertices,
		   const matrix::Vector2f &point)
{
	bool c = false;

	for (int i = 0, j = num_vertices - 1; i < num_vertices; j = i++) {
		insidePolygonUpdateState(c, vertices[i], vertices[j], point);
	}

	return c;
}

bool insideCircle(const matrix::Vector2<double> &center, float radius,
		  const matrix::Vector2<double> &point)
{
	float dist = get_distance_to_next_waypoint(point(0), point(1), center(0), center(1));
	return dist < radius;
}

bool segmentsIntersect(const matrix::Vector2f &p1, const matrix::Vector2f &p2,
		       const matrix::Vector2f &v1, const matrix::Vector2f &v2)
{
	// line intersection algorithm from wikipedia
	// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
	// Basic idea: a 2D vector formula for each line is created, consisting of a start point e.g. p1
	// and a direction vector pointing from p1 to p2. A running variable t [0,1] defines the position
	// on the line betwen p1 and p2. So in this case the formula for the line is P = p1 + t * (p2-p1).
	// The same is done for the second line and then both lines are set to be equal (to find the intersection).
	// We get two equations with two unknowns (t and u) which can be solved. If the denominator is zero, the lines are parallel
	// or coinciding, which means we can stop. If the solution for t and u is between 0 and 1, the line segments intersect, otherwise they don't.

	float x1 = p1(0), y1 = p1(1);
	float x2 = p2(0), y2 = p2(1);
	float x3 = v1(0), y3 = v1(1);
	float x4 = v2(0), y4 = v2(1);

	float denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

	if (fabsf(denominator) < FLT_EPSILON) {
		return false;
	}

	float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator;
	float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator;

	// lines intersect if both running variables are between 0 and 1
	return t > 0.0f && t < 1.0f && u > 0.0f && u < 1.0f;
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



} // namespace geofence_utils
