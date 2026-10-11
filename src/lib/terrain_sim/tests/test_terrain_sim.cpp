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
using terrain_sim::set_params;
using terrain_sim::set_octaves;

#include <chrono>
#include <cstdio>
#include <cmath>

namespace
{

constexpr float kWavelength = 200.f;

void configure_flat()
{
	set_params(0.f, kWavelength, 0);
}

void configure_hilly(int seed, float amp = 25.f)
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

TEST(TestTerrain, RaycastTiltedBeamLandsOnHills)
{
	set_params(20.f, 200.f, 7);
	/* 20 deg off vertical toward east, the hit must sit on the surface */
	const float dir_e = 0.34202f, dir_alt = -0.93969f;
	const float t = raycast(30.f, -40.f, 60.f, 0.f, dir_e, dir_alt, 500.f);
	ASSERT_LT(t, 500.f);
	const float hit_alt = 60.f + t * dir_alt;
	EXPECT_NEAR(hit_alt, height(30.f, -40.f + t * dir_e), 0.1f);
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
	/* printed rather than asserted, a loaded runner must not fail a unit test on wall clock time */
	printf("1000 height() calls took %lld us\n", static_cast<long long>(us));
}
