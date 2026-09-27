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

/**
 * make tests TESTFILTER=terrain_sim
 *
 * Every test configures the library first. The configuration is static and
 * gtest does not order tests.
 */

#include <gtest/gtest.h>
#include <lib/terrain_sim/terrain_sim.h>

using terrain_sim::height;
using terrain_sim::raycast;
using terrain_sim::surface_band;
using terrain_sim::set_params;
using terrain_sim::set_octaves;

#include <chrono>
#include <cstdio>
#include <cmath>

namespace
{

/* the shipped world, the library's own constants, so the tests run where the firmware does */
constexpr float kWavelength = terrain_sim::kWavelengthM;

void configure_flat()
{
	set_params(0.f, kWavelength, 0);
}

void configure_hilly(int seed, float amp = terrain_sim::kAmplitudeM)
{
	set_params(amp, kWavelength, seed);
}

} // namespace

TEST(TestTerrain, HomeOriginIsExactlyZero)
{
	for (int seed : {0, 1, 7, 42, 1000}) {
		configure_hilly(seed);
		EXPECT_NEAR(height(0.f, 0.f), 0.f, 1e-3f)
				<< "seed=" << seed;
	}
}

TEST(TestTerrain, HomeOriginIsZeroWhenAmpZero)
{
	configure_flat();
	EXPECT_FLOAT_EQ(height(0.f, 0.f),   0.f);
	EXPECT_FLOAT_EQ(height(100.f, 50.f), 0.f);
	EXPECT_FLOAT_EQ(height(-1234.f, 5678.f), 0.f);
}

TEST(TestTerrain, IsBitExactRepeatableForSameSeed)
{
	configure_hilly(42);

	const float a1 = height(17.5f, -89.25f);
	const float a2 = height(17.5f, -89.25f);
	EXPECT_EQ(a1, a2);

	/* a different seed and back, the recomputed home offset must match */
	configure_hilly(7);
	configure_hilly(42);

	const float a3 = height(17.5f, -89.25f);
	EXPECT_EQ(a1, a3);
}

TEST(TestTerrain, OctavesChangeDetailAndClamp)
{
	configure_hilly(3);
	set_octaves(6);
	const float six = height(123.f, -456.f);
	set_octaves(1);
	const float one = height(123.f, -456.f);
	EXPECT_NE(six, one);
	EXPECT_NEAR(height(0.f, 0.f), 0.f, 1e-3f);
	set_octaves(99);
	const float nine = height(123.f, -456.f);
	set_octaves(9);
	EXPECT_EQ(nine, height(123.f, -456.f));
	set_octaves(6);
	EXPECT_EQ(six, height(123.f, -456.f));
}

TEST(TestTerrain, DifferentSeedsProduceDifferentValues)
{
	configure_hilly(0);
	const float h0 = height(123.f, -456.f);

	configure_hilly(1);
	const float h1 = height(123.f, -456.f);

	EXPECT_NE(h0, h1);
}


TEST(TestTerrain, RaycastFlatStraightDownReturnsAltitude)
{
	configure_flat();
	const float t = raycast(0.f, 0.f, 42.5f, 0.f, 0.f, -1.f, 1000.f);
	EXPECT_FLOAT_EQ(t, 42.5f);
}

TEST(TestTerrain, RaycastTiltedBeamReturnsSlantRange)
{
	configure_flat();
	/* 30 deg off vertical over flat ground, slant range is alt / cos(30 deg) */
	const float t = raycast(0.f, 0.f, 50.f, 0.f, 0.5f, -0.866025f, 1000.f);
	EXPECT_NEAR(t, 50.f / 0.866025f, 0.01f);
}

TEST(TestTerrain, RaycastTiltedBeamLandsOnTerrain)
{
	/* the shipped 5 m terrain and 20 m terrain as a stress case, 20 deg off vertical toward east, the hit must sit on the surface */
	for (float amp : {terrain_sim::kAmplitudeM, 20.f}) {
		set_params(amp, 200.f, 7);
		const float dir_e = 0.34202f, dir_alt = -0.93969f;
		const float t = raycast(30.f, -40.f, 60.f, 0.f, dir_e, dir_alt, 500.f);
		ASSERT_LT(t, 500.f) << "amp " << amp;
		const float hit_alt = 60.f + t * dir_alt;
		EXPECT_NEAR(hit_alt, height(30.f, -40.f + t * dir_e), 0.1f) << "amp " << amp;
	}
}

TEST(TestTerrain, RaycastSteepTiltFindsFirstCrossing)
{
	/* beams tilted 20 to 30 deg over the shipped 5 m terrain and over 20 m terrain as the stress case, where
	 * a beam crosses a crest before the ground it hits, against an independent 1 cm march along the
	 * same beam */
	for (float amp : {terrain_sim::kAmplitudeM, 20.f}) {
		set_params(amp, 200.f, 3);
		set_octaves(6);
		int checked = 0;

		for (int k = 0; k < 40; ++k) {
			const float n = -300.f + 15.f * k, e = 200.f - 11.f * k;
			const float tilt = (20.f + 0.25f * k) * 3.14159265f / 180.f, az = 0.7f * k;
			const float dn = sinf(tilt) * cosf(az), de = sinf(tilt) * sinf(az), dalt = -cosf(tilt);
			const float alt = height(n, e) + 25.f;

			float ref = 0.f;

			while (ref < 200.f && alt + ref * dalt > height(n + ref * dn, e + ref * de)) {
				ref += 0.01f;
			}

			if (ref >= 200.f) {
				continue;
			}

			EXPECT_NEAR(raycast(n, e, alt, dn, de, dalt, 200.f), ref, 0.02f) << "amp " << amp;
			checked++;
		}

		EXPECT_GT(checked, 30) << "amp " << amp;
	}
}

TEST(TestTerrain, SurfaceBandsCoverAndRepeat)
{
	/* the shipped world, every band appears, low ground is the majority, and a point always gives the same band.
	 * The counts are over a 200 by 200 grid at 4 m, 800 m across, in seed 3 */
	set_params(terrain_sim::kAmplitudeM, 200.f, 3);
	set_octaves(6);
	int count[5] = {0, 0, 0, 0, 0};

	for (int i = 0; i < 200; ++i) {
		for (int j = 0; j < 200; ++j) {
			const int band = surface_band(-400.f + 4.f * i, -400.f + 4.f * j);
			ASSERT_GE(band, 0);
			ASSERT_LE(band, 4);
			count[band]++;
		}
	}

	EXPECT_GT(count[0], 28000);   /* grass */
	EXPECT_GT(count[1], 300);     /* dry sand, the scarce one at 5 m */
	EXPECT_GT(count[2], 2000);    /* rock, the outcrops, gentle terrain has no slope rock */
	EXPECT_GT(count[3], 1500);    /* melting snow */
	EXPECT_GT(count[4], 1000);    /* road */
	EXPECT_EQ(surface_band(123.f, -45.f), surface_band(123.f, -45.f));

	/* all five surfaces lie within 300 m of home in the seeds the tests and the CI fly, on a 5 m grid */
	for (int seed : {3, 80, 206}) {
		set_params(terrain_sim::kAmplitudeM, 200.f, seed);
		int near[5] = {0, 0, 0, 0, 0};

		for (int n = -300; n <= 300; n += 5) {
			for (int e = -300; e <= 300; e += 5) {
				if (n * n + e * e <= 300 * 300) {
					near[surface_band((float)n, (float)e)]++;
				}
			}
		}

		for (int b = 0; b < 5; ++b) {
			EXPECT_GT(near[b], 100) << "seed " << seed << " band " << b;
		}
	}

	set_params(0.f, 200.f, 3);
	EXPECT_EQ(surface_band(123.f, -45.f), 0);
}

TEST(TestTerrain, RaycastBelowSurfaceReturnsZero)
{
	configure_flat();
	const float t = raycast(0.f, 0.f, -0.02f, 0.f, 0.f, -1.f, 100.f);
	EXPECT_FLOAT_EQ(t, 0.f);
}

TEST(TestTerrain, RaycastNearlyVerticalInsideEnvelopeReturnsHit)
{
	configure_flat();
	/* 3 deg off vertical, hit at t = origin_alt / cos(3 deg) */
	const float dir_e   =  0.05234f;
	const float dir_alt = -0.99863f;
	const float t = raycast(0.f, 0.f, 30.f, 0.f, dir_e, dir_alt, 1000.f);
	EXPECT_NEAR(t, 30.f / 0.99863f, 0.01f);
}

TEST(TestTerrain, RaycastUpwardBeamReturnsMaxT)
{
	configure_flat();
	const float max_t = 250.f;
	/* upward beam from above flat ground never hits */
	const float t = raycast(0.f, 0.f, 10.f, 0.f, 0.f, 1.f, max_t);
	EXPECT_FLOAT_EQ(t, max_t);
}

TEST(TestTerrain, RaycastTooShortRangeReturnsMaxT)
{
	configure_flat();
	const float max_t = 5.f;
	/* 100 m up with a 5 m range, no hit */
	const float t = raycast(0.f, 0.f, 100.f, 0.f, 0.f, -1.f, max_t);
	EXPECT_FLOAT_EQ(t, max_t);
}

TEST(TestTerrain, ThousandCallsTimed)
{
	configure_hilly(123);

	volatile float sink = 0.f;
	const auto t0 = std::chrono::steady_clock::now();

	for (int i = 0; i < 1000; ++i) {
		/* different input every call so it cannot be hoisted */
		const float n = static_cast<float>(i) * 0.317f;
		const float e = static_cast<float>(1000 - i) * 0.213f;
		sink += height(n, e);
	}

	const auto t1 = std::chrono::steady_clock::now();
	const auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

	(void)sink;
	/* printed, a loaded runner must not fail a unit test on wall clock time */
	printf("1000 height() calls took %lld us\n", static_cast<long long>(us));
}

TEST(TerrainSim, RaycastStepLongerThanBeam)
{
	/* 1000 m swells over 2 octaves give a 125 m march step, longer than a 101 m beam, the ground 5 m under
	 * the sensor must still be found */
	set_params(10.f, 1000.f, 7);
	set_octaves(2);

	for (int i = -5; i <= 5; i++) {
		for (int j = -5; j <= 5; j++) {
			const float n = 100.f * (float)i;
			const float e = 100.f * (float)j;
			const float ground = height(n, e);
			EXPECT_NEAR(raycast(n, e, ground + 5.f, 0.f, 0.f, -1.f, 101.f), 5.f, 0.01f) << n << " " << e;
		}
	}

	/* ground past the limit is still a miss */
	EXPECT_FLOAT_EQ(raycast(100.f, 100.f, height(100.f, 100.f) + 150.f, 0.f, 0.f, -1.f, 101.f), 101.f);
	set_params(0.f, 200.f, 0);
	set_octaves(6);
}

TEST(TestTerrain, FormulaLooksUpOneRow)
{
	const terrain_sim::Rules *r = terrain_sim::rules_for(terrain_sim::kFormula);
	ASSERT_NE(r, nullptr);
	EXPECT_EQ(r->formula, terrain_sim::kFormula);
	EXPECT_EQ(terrain_sim::rules_for(0x0102), nullptr);
	EXPECT_EQ(terrain_sim::rules_for(0x0201), nullptr);

	EXPECT_FALSE(terrain_sim::set_formula(0x0102, 1));
	EXPECT_TRUE(terrain_sim::set_formula(terrain_sim::kFormula, 1));
	EXPECT_EQ(terrain_sim::formula(), terrain_sim::kFormula);

	/* disabled keeps the formula and flattens the field */
	EXPECT_TRUE(terrain_sim::set_formula(terrain_sim::kFormula, 1, false));
	EXPECT_EQ(terrain_sim::formula(), terrain_sim::kFormula);
	EXPECT_FLOAT_EQ(height(120.f, 120.f), 0.f);
	set_params(0.f, 200.f, 0);
	set_octaves(6);
}

/* formula 0x0101 is frozen. These heights and bands were recorded from this code at points that between
 * them touch every band. A change to any value in that row, or to the rule behind it, fails here, and the
 * fix is the next row under the next formula, never an edit to this one. */
TEST(TestTerrain, Formula0101IsFrozen)
{
	struct Point { int seed; float n, e, height; int band; };
	static const Point kRecorded[] = {
		{206, 0.0f, 2.0f, 0.074707f, 2},
		{206, 10.0f, -5.0f, 0.679417f, 2},
		{206, -37.5f, 61.0f, -2.338217f, 0},
		{206, 120.0f, 120.0f, -8.254951f, 0},
		{206, -250.0f, 40.0f, -4.688935f, 0},
		{206, 300.0f, -300.0f, -1.814588f, 3},
		{206, 55.0f, 410.0f, -2.744693f, 0},
		{206, -500.0f, -500.0f, 1.255113f, 3},
		{206, -600.0f, -495.0f, -1.222327f, 1},
		{206, -600.0f, -480.0f, -2.111593f, 2},
		{206, -600.0f, -127.5f, 0.122935f, 4},
		{3, 0.0f, 2.0f, -0.057977f, 0},
		{3, 10.0f, -5.0f, -0.075489f, 0},
		{3, -37.5f, 61.0f, 1.712295f, 0},
		{3, 120.0f, 120.0f, 2.599012f, 0},
		{3, -250.0f, 40.0f, 5.594074f, 0},
		{3, 300.0f, -300.0f, 4.370723f, 0},
		{3, -600.0f, -540.0f, 3.368534f, 4},
		{3, -600.0f, -285.0f, 6.785832f, 3},
		{3, -600.0f, -157.5f, 6.326584f, 1},
		{3, -600.0f, 22.5f, -1.899146f, 2},
	};

	int seed = -1;

	for (const Point &p : kRecorded) {
		if (p.seed != seed) {
			seed = p.seed;
			ASSERT_TRUE(terrain_sim::set_formula(0x0101, seed));
		}

		EXPECT_NEAR(height(p.n, p.e), p.height, 1e-4f) << "seed " << p.seed << " at " << p.n << ", " << p.e;
		EXPECT_EQ(surface_band(p.n, p.e), p.band) << "seed " << p.seed << " at " << p.n << ", " << p.e;
	}

	set_params(0.f, 200.f, 0);
	set_octaves(6);
}

TEST(TestRangefinder, NoiseIsSquareInRange)
{
	using terrain_sim::rangefinder_reading;
	/* one standard deviation at 25 m is the constant, a quarter of it at 12.5 m and four times at 50 m */
	EXPECT_NEAR(rangefinder_reading(25.f, 0, 1.f, 0.f, 0.10f, 100.f) - 25.f, 0.10f, 1e-5f);
	EXPECT_NEAR(rangefinder_reading(12.5f, 0, 1.f, 0.f, 0.10f, 100.f) - 12.5f, 0.025f, 1e-5f);
	EXPECT_NEAR(rangefinder_reading(50.f, 0, 1.f, 0.f, 0.10f, 100.f) - 50.f, 0.40f, 1e-5f);
	EXPECT_NEAR(rangefinder_reading(2.f, 0, 1.f, 0.f, 0.10f, 100.f) - 2.f, 0.00064f, 1e-6f);
	/* no draw, no noise, and the constant at zero is the exact reading */
	EXPECT_FLOAT_EQ(rangefinder_reading(37.25f, 0, 0.f, 0.f, 0.10f, 100.f), 37.25f);
	EXPECT_FLOAT_EQ(rangefinder_reading(37.25f, 0, 3.f, 0.f, 0.f, 100.f), 37.25f);
	/* the floor adds to the law, 1 cm at 2 m is the floor plus 0.64 mm, at 25 m it is 11 cm */
	EXPECT_NEAR(rangefinder_reading(2.f, 0, 1.f, 0.01f, 0.10f, 100.f) - 2.f, 0.01064f, 1e-6f);
	EXPECT_NEAR(rangefinder_reading(25.f, 0, 1.f, 0.01f, 0.10f, 100.f) - 25.f, 0.11f, 1e-5f);
	/* the floor alone when the 25 m constant is zero */
	EXPECT_NEAR(rangefinder_reading(40.f, 0, 1.f, 0.01f, 0.f, 100.f) - 40.f, 0.01f, 1e-5f);
	/* a draw that would take the reading below zero is clamped */
	EXPECT_FLOAT_EQ(rangefinder_reading(1.f, 0, -1e6f, 0.f, 0.10f, 100.f), 0.f);
}

TEST(TestRangefinder, UniformDrawIsInRangeAndRepeatable)
{
	uint32_t a = 0x9e3779b9u, b = 0x9e3779b9u;

	for (int i = 0; i < 100000; ++i) {
		const float u = terrain_sim::rand_uniform(a);
		EXPECT_GE(u, 0.f);
		EXPECT_LT(u, 1.f);
		EXPECT_EQ(u, terrain_sim::rand_uniform(b));
	}
}

TEST(TestRangefinder, SurfaceStretchesTheRange)
{
	using terrain_sim::rangefinder_reading;
	/* the five factors, read as the spread at a true 25 m with one standard deviation drawn */
	const float factor[5] = {1.f, 1.37f, 1.82f, 0.95f, 2.12f};

	for (int band = 0; band < 5; ++band) {
		const float expected = 0.10f * factor[band] * factor[band];
		EXPECT_NEAR(rangefinder_reading(25.f, band, 1.f, 0.f, 0.10f, 100.f) - 25.f, expected, 1e-5f) << "band " << band;
	}

	/* the reading is centered on the true range whatever the surface */
	EXPECT_FLOAT_EQ(rangefinder_reading(40.f, 2, 0.f, 0.f, 0.10f, 100.f), 40.f);
}

TEST(TestRangefinder, DarkSurfaceDropsOutSooner)
{
	using terrain_sim::rangefinder_reading;
	/* asphalt at 2.12 passes the 100 m limit at a true 47.2 m, rock at 55 m, grass at 100 m, snow at 105 m */
	EXPECT_FLOAT_EQ(rangefinder_reading(48.f, 4, 0.f, 0.f, 0.10f, 100.f), 101.f);
	EXPECT_FLOAT_EQ(rangefinder_reading(47.f, 4, 0.f, 0.f, 0.10f, 100.f), 47.f);
	EXPECT_FLOAT_EQ(rangefinder_reading(56.f, 2, 0.f, 0.f, 0.10f, 100.f), 101.f);
	EXPECT_FLOAT_EQ(rangefinder_reading(99.f, 0, 0.f, 0.f, 0.10f, 100.f), 99.f);
	EXPECT_FLOAT_EQ(rangefinder_reading(104.f, 3, 0.f, 0.f, 0.10f, 100.f), 104.f);
	EXPECT_FLOAT_EQ(rangefinder_reading(106.f, 3, 0.f, 0.f, 0.10f, 100.f), 101.f);
}
