/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

// The synthetic terrain behind the terrain interface answer the same as terrain_sim asked in meters.

#define MODULE_NAME "TerrainSyntheticTest"

#include <gtest/gtest.h>
#include <cstdarg>
#include <cstdio>
#include "TerrainSynthetic.hpp"

// The terrain classes log with PX4_INFO and PX4_ERR. The two stubs below stand in for them, as
// TerrainDemTest does, so the host test links without px4_platform.
__BEGIN_DECLS

void px4_log_info(const char *moduleName, const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vfprintf(stdout, fmt, args);
	va_end(args);
	fputc('\n', stdout);
}

void px4_log_error(const char *moduleName, const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);
	fputc('\n', stderr);
}

__END_DECLS

namespace
{
constexpr double kLat0 = 47.397742; // SIH_LOC_LAT0 default
constexpr double kLon0 = 8.545594;
constexpr float kH0 = 488.f;

struct Fixture : public ::testing::Test {
	TerrainSynthetic terrain;
	MapProjection ref{kLat0, kLon0};

	void SetUp() override
	{
		ASSERT_TRUE(terrain_sim::set_formula(terrain_sim::kFormula, 206, true));
		terrain.setReference(kLat0, kLon0);
		terrain.setDefaultAltitude(kH0);
	}

	void lla(float north, float east, double &lat, double &lon) { ref.reproject(north, east, lat, lon); }
};

const float kPoints[][2] = {{0.f, 0.f}, {37.5f, -12.f}, {-210.f, 95.f}, {640.f, 480.f}, {-1500.f, -900.f}, {2400.f, 100.f}};
}

TEST_F(Fixture, homeIsOnTheGround)
{
	float alt;
	EXPECT_TRUE(terrain.altitude(kLat0, kLon0, alt));
	EXPECT_FLOAT_EQ(alt, kH0);
	EXPECT_FLOAT_EQ(terrain.vehicleGroundAltitude(kLat0, kLon0), kH0);
}

TEST_F(Fixture, altitudeMatchesTheLibraryInMeters)
{
	for (const auto &p : kPoints) {
		double lat, lon;
		lla(p[0], p[1], lat, lon);
		float alt;
		EXPECT_TRUE(terrain.altitude(lat, lon, alt));
		// the projection round trip is good to millimeters, the terrain slopes less than one to one
		EXPECT_NEAR(alt, kH0 + terrain_sim::height(p[0], p[1]), 2e-3f) << p[0] << "," << p[1];
		EXPECT_NEAR(terrain.vehicleGroundAltitude(lat, lon), alt, 0.f);
	}
}

TEST_F(Fixture, straightDownReadsHeightAboveGround)
{
	for (const auto &p : kPoints) {
		double lat, lon;
		lla(p[0], p[1], lat, lon);
		const float ground = kH0 + terrain_sim::height(p[0], p[1]);

		for (float h : {0.5f, 2.f, 25.f, 65.f}) {
			const float d = terrain.rayDistance(lat, lon, ground + h, matrix::Vector3f(0.f, 0.f, 1.f), 100.f);
			EXPECT_NEAR(d, h, 0.02f) << p[0] << "," << p[1] << " at " << h;
		}
	}
}

TEST_F(Fixture, tiltedBeamMatchesTheLibraryRaycast)
{
	const matrix::Vector3f dir = matrix::Vector3f(sinf(0.35f), 0.2f, cosf(0.35f)).unit();

	for (const auto &p : kPoints) {
		double lat, lon;
		lla(p[0], p[1], lat, lon);
		const float alt = kH0 + terrain_sim::height(p[0], p[1]) + 30.f;
		const float expected = terrain_sim::raycast(p[0], p[1], alt - kH0, dir(0), dir(1), -dir(2), 101.f);
		const float d = terrain.rayDistance(lat, lon, alt, dir, 100.f);
		// the projected origin differs from the meters by the round trip, well under a step of the march
		EXPECT_NEAR(d, expected, 0.02f) << p[0] << "," << p[1];
	}
}

TEST_F(Fixture, missAndUpwardBeam)
{
	double lat, lon;
	lla(640.f, 480.f, lat, lon);
	const float alt = kH0 + terrain_sim::height(640.f, 480.f) + 30.f;
	EXPECT_TRUE(std::isnan(terrain.rayDistance(lat, lon, alt, matrix::Vector3f(0.f, 0.f, 1.f), 20.f)));
	EXPECT_TRUE(std::isnan(terrain.rayDistance(lat, lon, alt, matrix::Vector3f(0.f, 0.f, -1.f), 100.f)));
	EXPECT_TRUE(std::isnan(terrain.rayDistance(lat, lon, alt, matrix::Vector3f(1.f, 0.f, 0.f), 100.f)));
}

TEST_F(Fixture, disabledIsFlatLikeTerrainFlat)
{
	ASSERT_TRUE(terrain_sim::set_formula(terrain_sim::kFormula, 206, false));
	TerrainFlat flat;
	flat.setDefaultAltitude(kH0);
	const matrix::Vector3f dir = matrix::Vector3f(0.3f, -0.1f, 0.9f).unit();

	for (const auto &p : kPoints) {
		double lat, lon;
		lla(p[0], p[1], lat, lon);
		float alt;
		terrain.altitude(lat, lon, alt);
		EXPECT_FLOAT_EQ(alt, kH0);
		EXPECT_NEAR(terrain.rayDistance(lat, lon, kH0 + 12.f, dir, 100.f), flat.rayDistance(lat, lon, kH0 + 12.f, dir, 100.f), 0.02f);
	}
}
