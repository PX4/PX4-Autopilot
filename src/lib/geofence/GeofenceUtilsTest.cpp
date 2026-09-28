/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include "geofence_utils.h"

using namespace matrix;
using SS = geofence_utils::SegSegResult;
using AddResult = geofence_utils::PlannerPolygons::AddResult;

// Low level primitives operating on fixed-point int32 coordinates.

TEST(GeofenceUtilsTest, Orient2d)
{
	// CCW turn -> +1, CW turn -> -1, collinear -> 0 (in front of, behind, between).
	EXPECT_EQ(1,  geofence_utils::orient2d(0, 0, 1, 0, 0, 1));
	EXPECT_EQ(-1, geofence_utils::orient2d(0, 0, 1, 0, 0, -1));
	EXPECT_EQ(0,  geofence_utils::orient2d(0, 0, 2, 2, 1, 1));
	EXPECT_EQ(0,  geofence_utils::orient2d(0, 0, 2, 2, 3, 3));
}

TEST(GeofenceUtilsTest, SegmentsCross)
{
	// Vertical ab vs horizontal cd just above the x-axis; strict interior crossing.
	EXPECT_EQ(SS::Cross, geofence_utils::segmentsIntersect(0, 0, 0, 100, -1, 1, 100, 1));

	// Crossing at very shallow angle - not a problem in fixed point
	const int D = 10'000'000;  // 100 km
	const int d = 1;           // 1 cm
	EXPECT_EQ(SS::Cross, geofence_utils::segmentsIntersect(0, 0, D, 0, 0, -d, D, d));
}

TEST(GeofenceUtilsTest, SegmentsEndpointOnInterior)
{
	// Endpoint c of cd lies strictly on the open ab (axis-aligned).
	EXPECT_EQ(SS::CInsideAB, geofence_utils::segmentsIntersect(0, 0, 0, 200, 0, 100, 100, 100));
	EXPECT_EQ(SS::AInsideCD, geofence_utils::segmentsIntersect(0, 100, 100, 100, 0, 0, 0, 200));

	// Same configuration on a slanted line.
	EXPECT_EQ(SS::CInsideAB, geofence_utils::segmentsIntersect(-100, 0, 100, 200, 0, 100, 100, 100));
	EXPECT_EQ(SS::AInsideCD, geofence_utils::segmentsIntersect(0, 100, 100, 100, -100, 0, 100, 200));
}

TEST(GeofenceUtilsTest, SegmentsCollinear)
{
	// Identical segments and sub-intervals are reported as Collinear, not Cross.
	EXPECT_EQ(SS::Collinear, geofence_utils::segmentsIntersect(0, 0, 2, 2, 1, 1, 2, 2));
	EXPECT_EQ(SS::Collinear, geofence_utils::segmentsIntersect(0, 0, 300, 0, 0, 0, 300, 0));
	EXPECT_EQ(SS::Collinear, geofence_utils::segmentsIntersect(1000, 1000, 4000, 2000, 1000, 1000, 4000, 2000));
	// Same supporting line, non-overlapping intervals.
	EXPECT_EQ(SS::Collinear, geofence_utils::segmentsIntersect(0, 0, 100, 0, 200, 0, 300, 0));
}

TEST(GeofenceUtilsTest, SegmentsDisjoint)
{
	// Parallel, non-collinear, disjoint.
	EXPECT_EQ(SS::Disjoint, geofence_utils::segmentsIntersect(0, 0, 300, 0, 1000, 1000, 4000, 2000));
}

TEST(GeofenceUtilsTest, PolygonIsCCW)
{
	const Vector2f square[4] = {{0.f, 0.f}, {1.f, 0.f}, {1.f, 1.f}, {0.f, 1.f}};
	EXPECT_TRUE(geofence_utils::isPolygonCCW(square, 4));
}

// ===========================================================================
// Segment vs polygon: each test names one geometric configuration and asserts
// the violation flag for both inclusion (outside disallowed) and exclusion
// (inside disallowed), under both polygon winding orders.
// ===========================================================================

constexpr int kMaxN = 8;

// Check segment/polygon interior intersection under both winding direction &
// both zone types. Note that the used lineSegmentIntersectsPolygon first adds
// the polygon to the internal representation which canonicalises orientation.
// So the internal convention (left = illegal) should hold, this is also what we
// test here.
void expectSegmentVsPolygon(const Vector2f *vertices, int n,
			    const Vector2f &a, const Vector2f &b,
			    bool intersects_inclusion, bool intersects_exclusion)
{
	ASSERT_LE(n, kMaxN);
	Vector2f rev[kMaxN];

	for (int i = 0; i < n; ++i) { rev[i] = vertices[n - 1 - i]; }

	EXPECT_EQ(intersects_inclusion, geofence_utils::lineSegmentIntersectsPolygon(a, b, vertices, n, true));
	EXPECT_EQ(intersects_exclusion, geofence_utils::lineSegmentIntersectsPolygon(a, b, vertices, n, false));
	EXPECT_EQ(intersects_inclusion, geofence_utils::lineSegmentIntersectsPolygon(a, b, rev,      n, true));
	EXPECT_EQ(intersects_exclusion, geofence_utils::lineSegmentIntersectsPolygon(a, b, rev,      n, false));
}

// Convex polygon (unit square)
const Vector2f kSquare[4] = {{0.f, 0.f}, {1.f, 0.f}, {1.f, 1.f}, {0.f, 1.f}};

TEST(GeofenceUtilsTest, SquareSegmentOutside)
{
	// Far outside: inclusion is violated (must stay inside), exclusion is fine.
	expectSegmentVsPolygon(kSquare, 4, {4.f, 5.f}, {5.f, 4.f}, true, false);
}

TEST(GeofenceUtilsTest, SquareSegmentInside)
{
	// Strict interior: exclusion violated, inclusion fine.
	expectSegmentVsPolygon(kSquare, 4, {0.2f, 0.2f}, {0.6f, 0.5f}, false, true);
}

TEST(GeofenceUtilsTest, SquareSegmentStrictCrossing)
{
	// Interior endpoint to exterior endpoint via a single proper edge crossing.
	expectSegmentVsPolygon(kSquare, 4, {0.5f, 0.5f}, {0.5f, 1.5f}, true, true);
}

TEST(GeofenceUtilsTest, SquareSegmentCrossesTwoEdges)
{
	// Exterior endpoints with two proper edge crossings.
	expectSegmentVsPolygon(kSquare, 4, {0.5f, -0.5f}, {0.5f, 1.5f}, true, true);
}

TEST(GeofenceUtilsTest, SquareSegmentEdgeToEdgeThroughInterior)
{
	// Endpoints on two opposite edges; the segment body lies in the interior.
	expectSegmentVsPolygon(kSquare, 4, {0.5f, 0.f}, {0.5f, 1.f}, false, true);
}

TEST(GeofenceUtilsTest, SquareSegmentDiagonalThroughOppositeVertices)
{
	// Endpoints are two opposite polygon vertices; segment lies in the interior.
	expectSegmentVsPolygon(kSquare, 4, {0.f, 0.f}, {1.f, 1.f}, false, true);
	// Segment passes through opposite vertices - both zones violated.
	expectSegmentVsPolygon(kSquare, 4, {-0.5f, 0.5f}, {1.5f, 1.5f}, true, true);
	// Diagonal through only one vertex - still both zones.
	expectSegmentVsPolygon(kSquare, 4, {2.f, 2.f}, {0.5f, 0.5f}, true, true);
}

TEST(GeofenceUtilsTest, SquareSegmentAlongEdge)
{
	// Segment coincides with one edge (pure graze, no strict interior or exterior).
	expectSegmentVsPolygon(kSquare, 4, {0.f, 0.f}, {1.f, 0.f}, false, false);
}

TEST(GeofenceUtilsTest, SquareSegmentExtendsBeyondEdge)
{
	// Segment overlaps edge but is longer. For exclusion this is ok
	// (non-intersecting), for inclusion this violates the outside region.
	expectSegmentVsPolygon(kSquare, 4, {0.f, 0.f}, {2.f, 0.f}, true, false);
	expectSegmentVsPolygon(kSquare, 4, {-1.f, 0.f}, {2.f, 0.f}, true, false);
}

TEST(GeofenceUtilsTest, SquareSegmentTangentThroughVertex)
{
	// Tangent line touching the (1,1) corner with both endpoints outside.
	// Inclusion violated (line outside), exclusion not (only boundary).
	// One case with midpoint = vertex, one !=
	expectSegmentVsPolygon(kSquare, 4, {2.f, 0.f}, {0.f, 2.f},  true, false);
	expectSegmentVsPolygon(kSquare, 4, {2.f, 0.f}, {-1.f, 3.f}, true, false);

	// If we nudge the point inward just a bit, we have a strict intersection
	// and both zones are violated.
	expectSegmentVsPolygon(kSquare, 4, {2.f, 0.f}, {0.f, 1.98f}, true, true);
}






// Nonconvex polygon (L-shape with a reflex vertex at (1,1))
const Vector2f kLShape[6] = {{0.f, 0.f}, {2.f, 0.f}, {2.f, 1.f}, {1.f, 1.f}, {1.f, 2.f}, {0.f, 2.f}};

TEST(GeofenceUtilsTest, LShapeSegmentDisjointOutside)
{
	// Entirely in the notch (the reflex cone, outside the polygon).
	expectSegmentVsPolygon(kLShape, 6, {1.5f, 1.5f}, {1.8f, 1.8f}, true, false);
}

TEST(GeofenceUtilsTest, LShapeSegmentEntirelyInside)
{
	// Horizontal line inside bottom rectangle - inclusion non-intersecting, exclusion intersecting
	expectSegmentVsPolygon(kLShape, 6, {0.5f, 0.5f}, {1.5f, 0.5f}, false, true);

	// Skewed line inside upper rectangle - same
	expectSegmentVsPolygon(kLShape, 6, {0.3f, 1.2f}, {0.7f, 1.8f}, false, true);
}

TEST(GeofenceUtilsTest, LShapeSegmentCrossesNotchEdge)
{
	// Single proper crossing from the upper-rect interior into the notch.
	expectSegmentVsPolygon(kLShape, 6, {0.5f, 1.5f}, {1.5f, 1.5f}, true, true);
}

TEST(GeofenceUtilsTest, LShapeSegmentThroughReflexVertexInsideToInside)
{
	// Both endpoints inside, segment passes through the reflex (1,1) but
	// stays inside. Inclusion non-intersecting, exclusion intersecting.
	expectSegmentVsPolygon(kLShape, 6, {0.5f, 1.5f}, {2.f, 0.f}, false, true);
	expectSegmentVsPolygon(kLShape, 6, {0.5f, 1.5f}, {1.5f, 0.5f}, false, true);
}

TEST(GeofenceUtilsTest, LShapeSegmentThroughReflexVertexIntoNotch)
{
	// One endpoint inside the polygon, the other in the notch; the segment
	// crosses the boundary at the reflex vertex.
	expectSegmentVsPolygon(kLShape, 6, {0.5f, 0.5f}, {2.f, 2.f}, true, true);
}

TEST(GeofenceUtilsTest, LShapeSegmentBetweenTwoVerticesAcrossNotch)
{
	// Endpoints are two convex polygon vertices, (2,1) and (1,2). The chord
	// between them crosses the notch (outside the polygon), grazing the
	// polygon only at those two vertices.
	expectSegmentVsPolygon(kLShape, 6, {2.f, 1.f}, {1.f, 2.f}, true, false);
}

TEST(GeofenceUtilsTest, LShapeSegmentTangentThroughConvexVertex)
{
	// These lines touch a vertex from outside. Inclusion -> intersection,
	// exclusion -> no intersection.

	// Tangent at (2, 0)
	expectSegmentVsPolygon(kLShape, 6, {3.f, 1.f}, {1.5f, -0.5f}, true, false);
	// Tangent at (0, 0)
	expectSegmentVsPolygon(kLShape, 6, {-1.f, 1.f}, {1.f, -1.f}, true, false);
	// Tangent at both (1, 2) and (2, 1)
	expectSegmentVsPolygon(kLShape, 6, {0.f, 3.f}, {3.f, 0.f}, true, false);
}

// ===========================================================================
// addPolygon rejects input out of bounds
// ===========================================================================

TEST(GeofenceUtilsTest, AddPolygonRejectsOutOfBounds)
{
	// Square inside of bounds, 10000 km.
	float extent = 10000 * 1000; // km to m
	const Vector2f square[4] = {{0.f, 0.f}, {extent, 0.f}, {extent, extent}, {0.f, extent}};

	geofence_utils::PlannerPolygons polys;
	EXPECT_EQ(polys.addPolygon(square, 4, /*is_inclusion_zone=*/false, /*margin=*/0.f), AddResult::Success);

	// Square out of bounds, 12000 km.
	extent = 12000 * 1000;
	const Vector2f square_larger[4] = {{0.f, 0.f}, {extent, 0.f}, {extent, extent}, {0.f, extent}};

	geofence_utils::PlannerPolygons polys_larger;
	EXPECT_EQ(polys_larger.addPolygon(square_larger, 4, /*is_inclusion_zone=*/false, /*margin=*/0.f), AddResult::OutOfRange);
}

TEST(GeofenceUtilsTest, AddApproxCircleRejectsOutOfBounds)
{
	// 100m circle at origin with 10m margin - works
	geofence_utils::PlannerPolygons polys0;
	EXPECT_EQ(
		polys0.addApproxCircle(matrix::Vector2f(0, 0), 100.0f, 10.0f, false),
		AddResult::Success
	);

	// 5000km circle -- same
	EXPECT_EQ(
		polys0.addApproxCircle(matrix::Vector2f(0, 0), 5000.f * 1000.f, 10.0f, false),
		AddResult::Success
	);

	// 10000km circle -- fails (circle is within range but approx circle not)
	EXPECT_EQ(
		polys0.addApproxCircle(matrix::Vector2f(0, 0), 10000.f * 1000.f, 10.0f, false),
		AddResult::OutOfRange
	);

	// 12000km circle -- fails
	EXPECT_EQ(
		polys0.addApproxCircle(matrix::Vector2f(0, 0), 12000.f * 1000.f, 10.0f, false),
		AddResult::OutOfRange
	);

	// 10000km circle, 2000 km margin (outwards, exclusion zone) -- fails
	EXPECT_EQ(
		polys0.addApproxCircle(matrix::Vector2f(0, 0), 12000.f * 1000.f, 2000.f * 1000.f, false),
		AddResult::OutOfRange
	);

	// 2000km circle 10000km out - fails
	EXPECT_EQ(
		polys0.addApproxCircle(matrix::Vector2f(0, 10000.f * 1000.f), 2000.f * 1000.f, 10.f, false),
		AddResult::OutOfRange
	);

	// 100km circle 10000km out - works
	EXPECT_EQ(
		polys0.addApproxCircle(matrix::Vector2f(0, 10000.f * 1000.f), 100.f * 1000.f, 10.f, false),
		AddResult::Success
	);

	// 100km circle 10000km out, 1000km margin - fails
	EXPECT_EQ(
		polys0.addApproxCircle(matrix::Vector2f(0, 10000.f * 1000.f), 100.f * 1000.f, 1000.f * 1000.f, false),
		AddResult::OutOfRange
	);

}

// ===========================================================================
// addPolygon rejects non-simple input
// ===========================================================================

TEST(GeofenceUtilsTest, AddPolygonRejectsSelfIntersecting)
{
	// Figure-eight quadrilateral: 0->1->2->3 with edges (0,1) and (2,3) crossing.
	const Vector2f figure_eight[4] = {{0.f, 0.f}, {10.f, 10.f}, {10.f, 0.f}, {0.f, 10.f}};

	geofence_utils::PlannerPolygons polys;
	EXPECT_EQ(polys.addPolygon(figure_eight, 4, /*is_inclusion_zone=*/false, /*margin=*/0.f), AddResult::Degenerate);
}

TEST(GeofenceUtilsTest, AddPolygonAcceptsSimpleQuad)
{
	// Convex simple quadrilateral; must succeed.
	const Vector2f square[4] = {{0.f, 0.f}, {10.f, 0.f}, {10.f, 10.f}, {0.f, 10.f}};

	geofence_utils::PlannerPolygons polys;
	EXPECT_EQ(polys.addPolygon(square, 4, /*is_inclusion_zone=*/false, /*margin=*/0.f), AddResult::Success);
}
