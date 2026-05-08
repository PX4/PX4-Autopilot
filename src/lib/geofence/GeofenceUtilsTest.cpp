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


TEST(GeofenceUtilsTest, Orient2d)
{
	// CCW turn -> +1, CW turn -> -1, collinear -> 0.
	EXPECT_EQ(1,  geofence_utils::orient2d({0.f, 0.f}, {1.f, 0.f}, {0.f, 1.f}));
	EXPECT_EQ(-1, geofence_utils::orient2d({0.f, 0.f}, {1.f, 0.f}, {0.f, -1.f}));
	EXPECT_EQ(0,  geofence_utils::orient2d({0.f, 0.f}, {2.f, 2.f}, {1.f, 1.f}));
	EXPECT_EQ(0,  geofence_utils::orient2d({0.f, 0.f}, {2.f, 2.f}, {3.f, 3.f}));
}

TEST(GeofenceUtilsTest, SegmentsSharedEndpointNoIntersection)
{
	// Collinear, second segment is a sub-interval of the first.
	Vector2f p1(0.f, 0.f);
	Vector2f p2(2.f, 2.f);

	Vector2f v1(1.f, 1.f);
	Vector2f v2(2.f, 2.f);

	EXPECT_FALSE(geofence_utils::segmentsIntersect(p1, p2, v1, v2));
}

TEST(GeofenceUtilsTest, SegmentsCross)
{
	// vertical line from origin straight up
	Vector2f p1(0.f, 0.f);
	Vector2f p2(0.f, 1.f);

	Vector2f v1(-0.0001f, 0.0001f);
	Vector2f v2(1.f, 0.0001f);

	EXPECT_TRUE(geofence_utils::segmentsIntersect(p1, p2, v1, v2));
}

TEST(GeofenceUtilsTest, SegmentsTouching)
{
	// Vertical line
	Vector2f p1(0.f, 0.f);
	Vector2f p2(0.f, 2.f);

	// Horizontal line starting on interior of vertical line
	Vector2f v1(0.f, 1.0f);
	Vector2f v2(1.f, 1.0f);

	// Endpoint v1 of segment cd lies on the open segment ab. segmentsIntersect
	// only flags proper crossings, so an endpoint touch returns false.
	EXPECT_FALSE(geofence_utils::segmentsIntersect(p1, p2, v1, v2));
	EXPECT_FALSE(geofence_utils::segmentsIntersect(v1, v2, p1, p2));

	// Same, but with vertical line slanted for good measure
	p1(0) = -1.0f;
	p2(0) = 1.0f;

	EXPECT_FALSE(geofence_utils::segmentsIntersect(p1, p2, v1, v2));
	EXPECT_FALSE(geofence_utils::segmentsIntersect(v1, v2, p1, p2));
}

TEST(GeofenceUtilsTest, SegmentsParallel)
{
	Vector2f p1(0.f, 0.f);
	Vector2f p2(3.0f, 0.f);

	Vector2f v1(10.f, 10.f);
	Vector2f v2(40.f, 20.f);

	// Disjoint, non-collinear: no intersection.
	EXPECT_FALSE(geofence_utils::segmentsIntersect(p1, p2, v1, v2));

	// A segment with itself is collinear-overlapping, not a proper crossing.
	EXPECT_FALSE(geofence_utils::segmentsIntersect(p1, p2, p1, p2));
	EXPECT_FALSE(geofence_utils::segmentsIntersect(v1, v2, v1, v2));
}

TEST(GeofenceUtilsTest, SegmentsCollinearDisjoint)
{
	// Two segments on the same line but with non-overlapping intervals.
	Vector2f a(0.f, 0.f), b(1.f, 0.f);
	Vector2f c(2.f, 0.f), d(3.f, 0.f);

	EXPECT_FALSE(geofence_utils::segmentsIntersect(a, b, c, d));
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

	// Line in a completely random place
	Vector2f l1(4.f, 5.f);
	Vector2f l2(5.f, 4.f);

	// Outside an exclusion zone is allowed -> no intersection
	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices, N, false));
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

	// Line obviously passing through an edge
	Vector2f l1(0.5f, 0.5f);
	Vector2f l2(0.5f, 1.5f);

	// Crossing an edge always counts as intersection regardless of zone type
	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices, N, false));

	// Line through several edges
	Vector2f l3(0.5f, -0.5f);
	Vector2f l4(0.5f, 1.5f);

	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l3, l4, vertices, N, false));
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

	// Line going exactly between opposite vertices
	Vector2f l1(0.f, 0.f);
	Vector2f l2(1.f, 1.f);

	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices, N, false));

	// Line exactly touching sides
	Vector2f l3(0.5f, 0.0f);
	Vector2f l4(0.5f, 1.0f);

	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l3, l4, vertices, N, false));

	// Line completely in the interior of an exclusion zone -> disallowed
	Vector2f l5(0.2f, 0.2f);
	Vector2f l6(0.6f, 0.5f);

	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l5, l6, vertices, N, false));
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

	// Line exactly equal to one of the edges
	Vector2f l1(0.f, 0.f);
	Vector2f l2(1.f, 0.f);

	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices, N, false));

	// Same line but longer at one end - fails
	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsPolygon(l1, 2.f * l2, vertices, N, false));

	// A line just skimming the (1, 1) vertex but staying outside
	Vector2f l3(2.f, 0.f);
	Vector2f l4(0.f, 2.f);

	// Should be no intersection
	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsPolygon(l3, l4, vertices, N, false));
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

	// Line in a completely random place outside the polygon
	Vector2f l1(4.f, 5.f);
	Vector2f l2(5.f, 4.f);

	// Outside an inclusion zone is disallowed -> intersection
	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices, N, true));
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

	// Line crossing an edge from inside to outside
	Vector2f l1(0.5f, 0.5f);
	Vector2f l2(0.5f, 1.5f);

	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices, N, true));

	// Line through several edges
	Vector2f l3(0.5f, -0.5f);
	Vector2f l4(0.5f, 1.5f);

	EXPECT_TRUE(geofence_utils::lineSegmentIntersectsPolygon(l3, l4, vertices, N, true));
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

	// Line completely in the interior of an inclusion zone -> allowed
	Vector2f l1(0.2f, 0.2f);
	Vector2f l2(0.6f, 0.5f);

	EXPECT_FALSE(geofence_utils::lineSegmentIntersectsPolygon(l1, l2, vertices, N, true));
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
