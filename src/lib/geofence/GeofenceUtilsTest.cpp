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

TEST(GeofenceUtilsTest, SegmentsSharedEndpointNoIntersection)
{
	// Two segments share endpoint (1,1) but go in different directions — no crossing
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
