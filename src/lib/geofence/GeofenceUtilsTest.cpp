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
#include <cstdlib>
#include "geofence_utils.h"

using namespace matrix;


// Layer-1 primitives are tested directly in int32 (no float, no cm semantics).

TEST(GeofenceUtilsTest, Orient2d)
{
	// CCW turn -> +1, CW turn -> -1, collinear -> 0.
	EXPECT_EQ(1,  geofence_utils::orient2d(0, 0, 1, 0, 0, 1));
	EXPECT_EQ(-1, geofence_utils::orient2d(0, 0, 1, 0, 0, -1));
	EXPECT_EQ(0,  geofence_utils::orient2d(0, 0, 2, 2, 1, 1));
	EXPECT_EQ(0,  geofence_utils::orient2d(0, 0, 2, 2, 3, 3));
}

using SS = geofence_utils::SegSegResult;

TEST(GeofenceUtilsTest, SegmentsSharedEndpointNoIntersection)
{
	// Collinear, second segment is a sub-interval of the first.
	EXPECT_EQ(SS::Collinear, geofence_utils::segmentsIntersect(0, 0, 2, 2, 1, 1, 2, 2));
}

TEST(GeofenceUtilsTest, SegmentsCross)
{
	// Vertical ab vs horizontal cd just above the x-axis; strict interior crossing.
	EXPECT_EQ(SS::Cross, geofence_utils::segmentsIntersect(0, 0, 0, 100, -1, 1, 100, 1));
}

TEST(GeofenceUtilsTest, SegmentsTouching)
{
	// Endpoint c of cd lies strictly on the open ab.
	EXPECT_EQ(SS::CInsideAB, geofence_utils::segmentsIntersect(0, 0, 0, 200, 0, 100, 100, 100));
	// Argument order swapped: now endpoint a of the first segment is on cd.
	EXPECT_EQ(SS::AInsideCD, geofence_utils::segmentsIntersect(0, 100, 100, 100, 0, 0, 0, 200));

	// Slanted variant.
	EXPECT_EQ(SS::CInsideAB, geofence_utils::segmentsIntersect(-100, 0, 100, 200, 0, 100, 100, 100));
	EXPECT_EQ(SS::AInsideCD, geofence_utils::segmentsIntersect(0, 100, 100, 100, -100, 0, 100, 200));
}

TEST(GeofenceUtilsTest, SegmentsParallel)
{
	// Disjoint, non-collinear.
	EXPECT_EQ(SS::Disjoint, geofence_utils::segmentsIntersect(0, 0, 300, 0, 1000, 1000, 4000, 2000));

	// A segment with itself is collinear, not a proper crossing.
	EXPECT_EQ(SS::Collinear, geofence_utils::segmentsIntersect(0, 0, 300, 0, 0, 0, 300, 0));
	EXPECT_EQ(SS::Collinear,
		  geofence_utils::segmentsIntersect(1000, 1000, 4000, 2000, 1000, 1000, 4000, 2000));
}

TEST(GeofenceUtilsTest, SegmentsCollinearDisjoint)
{
	// Two segments on the same line but with non-overlapping intervals.
	EXPECT_EQ(SS::Collinear, geofence_utils::segmentsIntersect(0, 0, 100, 0, 200, 0, 300, 0));
}

TEST(GeofenceUtilsTest, SegmentPolygonExclusionOutside)
{
	static constexpr int N = 4;

	// Unit square (exclusion zone: inside disallowed)
	Vector2f p1(0.f, 0.f);
	Vector2f p2(1.f, 0.f);
	Vector2f p3(1.f, 1.f);
	Vector2f p4(0.f, 1.f);

	Vector2f vertices[N] = {p1, p2, p3, p4};
	Vector2f vertices_rev[N] = {p4, p3, p2, p1};

	// Line in a completely random place
	Vector2f l1(4.f, 5.f);
	Vector2f l2(5.f, 4.f);

	// Outside an exclusion zone is allowed -> no intersection
	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices, N, false));
	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices_rev, N, false));
}

TEST(GeofenceUtilsTest, SegmentPolygonExclusionThroughEdge)
{
	static constexpr int N = 4;

	// Unit square (exclusion zone)
	Vector2f p1(0.f, 0.f);
	Vector2f p2(1.f, 0.f);
	Vector2f p3(1.f, 1.f);
	Vector2f p4(0.f, 1.f);

	Vector2f vertices[N] = {p1, p2, p3, p4};
	Vector2f vertices_rev[N] = {p4, p3, p2, p1};

	// Line obviously passing through an edge
	Vector2f l1(0.5f, 0.5f);
	Vector2f l2(0.5f, 1.5f);

	// Crossing an edge always counts as intersection regardless of zone type
	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices, N, false));
	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices_rev, N, false));

	// Line through several edges
	Vector2f l3(0.5f, -0.5f);
	Vector2f l4(0.5f, 1.5f);

	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l3, l4, vertices, N, false));
	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l3, l4, vertices_rev, N, false));
}

TEST(GeofenceUtilsTest, SegmentPolygonExclusionInside)
{
	static constexpr int N = 4;

	// Unit square (exclusion zone)
	Vector2f p1(0.f, 0.f);
	Vector2f p2(1.f, 0.f);
	Vector2f p3(1.f, 1.f);
	Vector2f p4(0.f, 1.f);

	Vector2f vertices[N] = {p1, p2, p3, p4};
	Vector2f vertices_rev[N] = {p4, p3, p2, p1};

	// Line going exactly between opposite vertices
	Vector2f l1(0.f, 0.f);
	Vector2f l2(1.f, 1.f);

	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices, N, false));
	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices_rev, N, false));

	// Line exactly touching sides
	Vector2f l3(0.5f, 0.0f);
	Vector2f l4(0.5f, 1.0f);

	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l3, l4, vertices, N, false));
	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l3, l4, vertices_rev, N, false));

	// Line completely in the interior of an exclusion zone -> disallowed
	Vector2f l5(0.2f, 0.2f);
	Vector2f l6(0.6f, 0.5f);

	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l5, l6, vertices, N, false));
	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l5, l6, vertices_rev, N, false));
}

TEST(GeofenceUtilsTest, SegmentPolygonExclusionTouching)
{
	static constexpr int N = 4;

	// Unit square (exclusion zone)
	Vector2f p1(0.f, 0.f);
	Vector2f p2(1.f, 0.f);
	Vector2f p3(1.f, 1.f);
	Vector2f p4(0.f, 1.f);

	Vector2f vertices[N] = {p1, p2, p3, p4};
	Vector2f vertices_rev[N] = {p4, p3, p2, p1};

	// Line exactly equal to one of the edges
	Vector2f l1(0.f, 0.f);
	Vector2f l2(1.f, 0.f);

	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices, N, false));
	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices_rev, N, false));

	// Same line but longer at one end - fails
	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsPolygon(l1, 2.f * l2, vertices, N, false));
	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsPolygon(l1, 2.f * l2, vertices_rev, N, false));

	// A line just skimming the (1, 1) vertex but staying outside
	Vector2f l3(2.f, 0.f);
	Vector2f l4(0.f, 2.f);

	// Should be no intersection
	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsPolygon(l3, l4, vertices, N, false));
	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsPolygon(l3, l4, vertices_rev, N, false));

	// But if we move the line a couple of cm towards the square
	l4(1) = 1.98f;

	// We have a (strict) intersection
	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l3, l4, vertices, N, false));
	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l3, l4, vertices_rev, N, false));
}

TEST(GeofenceUtilsTest, SegmentPolygonInclusionOutside)
{
	static constexpr int N = 4;

	// Unit square (inclusion zone: outside disallowed)
	Vector2f p1(0.f, 0.f);
	Vector2f p2(1.f, 0.f);
	Vector2f p3(1.f, 1.f);
	Vector2f p4(0.f, 1.f);

	Vector2f vertices[N] = {p1, p2, p3, p4};
	Vector2f vertices_rev[N] = {p4, p3, p2, p1};

	// Line in a completely random place outside the polygon
	Vector2f l1(4.f, 5.f);
	Vector2f l2(5.f, 4.f);

	// Outside an inclusion zone is disallowed -> intersection
	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices, N, true));
	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices_rev, N, true));
}

TEST(GeofenceUtilsTest, SegmentPolygonInclusionThroughEdge)
{
	static constexpr int N = 4;

	// Unit square (inclusion zone)
	Vector2f p1(0.f, 0.f);
	Vector2f p2(1.f, 0.f);
	Vector2f p3(1.f, 1.f);
	Vector2f p4(0.f, 1.f);

	Vector2f vertices[N] = {p1, p2, p3, p4};
	Vector2f vertices_rev[N] = {p4, p3, p2, p1};

	// Line crossing an edge from inside to outside
	Vector2f l1(0.5f, 0.5f);
	Vector2f l2(0.5f, 1.5f);

	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices, N, true));
	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices_rev, N, true));

	// Line through several edges
	Vector2f l3(0.5f, -0.5f);
	Vector2f l4(0.5f, 1.5f);

	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l3, l4, vertices, N, true));
	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l3, l4, vertices_rev, N, true));
}

TEST(GeofenceUtilsTest, SegmentPolygonInclusionInside)
{
	static constexpr int N = 4;

	// Unit square (inclusion zone)
	Vector2f p1(0.f, 0.f);
	Vector2f p2(1.f, 0.f);
	Vector2f p3(1.f, 1.f);
	Vector2f p4(0.f, 1.f);

	Vector2f vertices[N] = {p1, p2, p3, p4};
	Vector2f vertices_rev[N] = {p4, p3, p2, p1};

	// Line completely in the interior of an inclusion zone -> allowed
	Vector2f l1(0.2f, 0.2f);
	Vector2f l2(0.6f, 0.5f);

	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices, N, true));
	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices_rev, N, true));
}


TEST(GeofenceUtilsTest, SegmentAndCircleIntersect)
{
	// vertical line from origin straight up
	Vector2f p1(0.f, 0.f);
	Vector2f p2(0.f, 1.f);
	Vector2f center(0.0f, 1.0f);

	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsCircle(p1, p2, center, 0.5f));

	p1 = Vector2f(-1.0f, 0.0f);
	p2 = Vector2f(1.f, 0.0f);
	center = Vector2f(0.f, 1.f);

	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsCircle(p1, p2, center, 1.1f));

	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsCircle(p1, p2, center, 0.9f));

}
TEST(GeofenceUtilsTest, PolygonIsCCW)
{
	// square with vertices in CCW order
	Vector2f p1(0.f, 0.f);
	Vector2f p2(1.f, 0.f);
	Vector2f p3(1.f, 1.f);
	Vector2f p4(0.f, 1.f);

	Vector2f vertices[4] = {p1, p2, p3, p4};

	EXPECT_TRUE(geofence_utils::isPolygonCCW(vertices, 4));
}
TEST(GeofenceUtilsTest, SymmetricPairIndex)
{

	// create a NxN matrix and fill with random data. Then pack the upper triangle into a 1D array.
	// Then loop through the upper triangle and read the values from the array using  symmetricPairIndex
	// and verify that the values match.
	constexpr size_t N = 21;
	constexpr size_t kPairs = N * (N - 1) / 2;

	float matrix[N][N];
	float packed[kPairs];

	std::srand(42);

	int idx = 0;

	for (size_t i = 0; i < N; ++i) {
		for (size_t j = i + 1; j < N; ++j) {
			matrix[i][j] = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
			matrix[j][i] = matrix[i][j]; // symmetric
			packed[idx++] = matrix[i][j];
		}
	}

	int counter = 0;

	for (size_t i = 0; i < N; ++i) {
		for (size_t j = i + 1; j < N; ++j) {
			counter++;
			EXPECT_FLOAT_EQ(matrix[i][j], packed[geofence_utils::symmetricPairIndex(i, j, N)]);
			EXPECT_FLOAT_EQ(matrix[i][j], packed[geofence_utils::symmetricPairIndex(j, i, N)]);
		}
	}

	// verify that we have the expected number of pairs
	EXPECT_EQ(counter, kPairs);
}
