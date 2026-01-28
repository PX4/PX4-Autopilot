/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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
#include <matrix/math.hpp>
#include <lib/geo/geo.h>

#include "lat_lon_alt.hpp"

using namespace matrix;
using math::radians;
using math::degrees;

TEST(TestLatLonAlt, init)
{
	LatLonAlt lla(5.7, -2.3, 420);
	ASSERT_FLOAT_EQ(lla.latitude_deg(), 5.7);
	ASSERT_FLOAT_EQ(lla.longitude_deg(), -2.3);
	ASSERT_EQ(lla.altitude(), 420);
}

TEST(TestLatLonAlt, set)
{
	LatLonAlt lla(0.0, 0.0, 0);
	ASSERT_EQ(lla.latitude_rad(), 0.0);
	ASSERT_EQ(lla.longitude_rad(), 0.0);
	ASSERT_EQ(lla.altitude(), 0);

	lla.setLatLonRad(0.1, -0.5);
	lla.setAltitude(420);
	ASSERT_EQ(lla.latitude_rad(), 0.1);
	ASSERT_EQ(lla.longitude_rad(), -0.5);
	ASSERT_EQ(lla.altitude(), 420);
}

TEST(TestLatLonAlt, copy)
{
	LatLonAlt lla(-0.8, -0.1, 500);

	LatLonAlt lla_copy = lla;
	ASSERT_EQ(lla_copy.latitude_deg(), -0.8);
	ASSERT_EQ(lla_copy.longitude_deg(), -0.1);
	ASSERT_EQ(lla_copy.altitude(), 500);
}

TEST(TestLatLonAlt, addDeltaPos)
{
	MapProjection pos_ref(60.0, 5.0);
	LatLonAlt lla(pos_ref.getProjectionReferenceLat(), pos_ref.getProjectionReferenceLon(), 400.f);

	Vector3f delta_pos(5.f, -2.f, 3.f);
	lla += delta_pos;

	double lat_new, lon_new;
	pos_ref.reproject(delta_pos(0), delta_pos(1), lat_new, lon_new);

	EXPECT_NEAR(lla.latitude_deg(), lat_new, 1e-6);
	EXPECT_NEAR(lla.longitude_deg(), lon_new, 1e-6);
	EXPECT_EQ(lla.altitude(), 397.f);
}

TEST(TestLatLonAlt, subLatLonAlt)
{
	MapProjection pos_ref(60.0, 5.0);
	LatLonAlt lla(pos_ref.getProjectionReferenceLat(), pos_ref.getProjectionReferenceLon(), 0.f);

	const Vector3f delta_pos_true(1.f, -2.f, 3.f);

	double lat_new, lon_new;
	pos_ref.reproject(delta_pos_true(0), delta_pos_true(1), lat_new, lon_new);
	LatLonAlt lla_new(lat_new, lon_new, -3.f);
	Vector3f delta_pos = lla_new - lla;

	EXPECT_NEAR(delta_pos(0), delta_pos_true(0), 1e-2);
	EXPECT_NEAR(delta_pos(1), delta_pos_true(1), 1e-2);
	EXPECT_EQ(delta_pos(2), delta_pos_true(2));
}

TEST(TestLatLonAlt, fromAndToECEF)
{
	for (double lat = -M_PI; lat < M_PI; lat += M_PI / 4.0) {
		for (double lon = -M_PI; lon < M_PI; lon += M_PI / 4.0) {
			for (float alt = -500.f; alt < 8000.f; alt += 500.f) {
				LatLonAlt lla(lat, lon, alt);

				LatLonAlt res = LatLonAlt::fromEcef(lla.toEcef());
				EXPECT_TRUE(!(lla - res).longerThan(10e-6f)) << "lat: " << lat << ", lon: " << lon << ", alt: " << alt;
			}
		}
	}
}
