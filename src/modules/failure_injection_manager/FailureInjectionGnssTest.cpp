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

// Unit tests for the pure GNSS spoof/jam mutator cores. These take explicit
// params + now_us (no uORB, params or hrt), so they run as a plain unit gtest.

#include <gtest/gtest.h>

#include <cmath>

#include <lib/failure_injection/FailureInjection.hpp>
#include <lib/geo/geo.h>

using namespace failure_injection;

namespace
{

sensor_gps_s clean_gps()
{
	sensor_gps_s gps{};
	gps.device_id = 0x1234;
	gps.fix_type = sensor_gps_s::FIX_TYPE_3D;
	gps.latitude_deg = 47.0;
	gps.longitude_deg = 8.0;
	gps.altitude_msl_m = 500.0;
	gps.eph = 0.9f;
	gps.epv = 1.7f;
	gps.hdop = 0.7f;
	gps.vdop = 1.1f;
	gps.s_variance_m_s = 0.5f;
	gps.satellites_used = 25;
	gps.vel_ned_valid = true;
	gps.vel_n_m_s = 0.f;
	gps.vel_e_m_s = 0.f;
	gps.spoofing_state = sensor_gps_s::SPOOFING_STATE_OK;
	gps.jamming_state = sensor_gps_s::JAMMING_STATE_OK;
	return gps;
}

} // namespace

TEST(FailureInjectionGnss, WrongSmoothDriftZeroAtOnset)
{
	sensor_gps_s gps = clean_gps();
	const sensor_gps_s ref = gps;
	GnssFailureState st;

	// At onset (now == captured onset), the expanding-drift offset and velocity are 0.
	apply_gnss_wrong(gps, st, 2, 1000);

	EXPECT_NEAR(gps.latitude_deg, ref.latitude_deg, 1e-12);
	EXPECT_NEAR(gps.longitude_deg, ref.longitude_deg, 1e-12);
	EXPECT_NEAR(gps.vel_n_m_s, 0.f, 1e-4f);
	EXPECT_NEAR(gps.vel_e_m_s, 0.f, 1e-4f);
	// Stealthy: fix quality and the report flag are left untouched.
	EXPECT_EQ(gps.fix_type, sensor_gps_s::FIX_TYPE_3D);
	EXPECT_FLOAT_EQ(gps.eph, ref.eph);
	EXPECT_EQ(gps.spoofing_state, sensor_gps_s::SPOOFING_STATE_OK);
}

TEST(FailureInjectionGnss, WrongConstantDriftEastCoherent)
{
	sensor_gps_s gps = clean_gps();
	const sensor_gps_s ref = gps;
	GnssFailureState st;

	apply_gnss_wrong(gps, st, 1, 0); // constant drift, onset at t=0

	// The mutator is applied to a fresh truth sample each cycle; offset is absolute from onset.
	gps = clean_gps();
	apply_gnss_wrong(gps, st, 1, 10000000); // +10 s

	// Drift is due east (bearing hardcoded to 90 deg): latitude unchanged, longitude grows.
	EXPECT_NEAR(gps.latitude_deg, ref.latitude_deg, 1e-9);
	EXPECT_GT(gps.longitude_deg, ref.longitude_deg);
	EXPECT_GT(gps.vel_e_m_s, 0.f);
	EXPECT_NEAR(gps.vel_n_m_s, 0.f, 1e-3f);

	// Coherence: over 10 s the position offset equals reported east velocity * 10 s.
	const double lat_rad = ref.latitude_deg * M_PI / 180.0;
	const double expected_dlon = ((double)gps.vel_e_m_s * 10.0 / (CONSTANTS_RADIUS_OF_EARTH * cos(lat_rad)))
				     * (180.0 / M_PI);
	EXPECT_NEAR(gps.longitude_deg - ref.longitude_deg, expected_dlon, 1e-6);
}

TEST(FailureInjectionGnss, WrongStaticJumpEast)
{
	sensor_gps_s gps = clean_gps();
	const sensor_gps_s ref = gps;
	GnssFailureState st;

	apply_gnss_wrong(gps, st, 0, 5000); // static jump

	// Jump is east and does not move latitude, with no velocity injected (teleport).
	EXPECT_GT(gps.longitude_deg, ref.longitude_deg);
	EXPECT_NEAR(gps.latitude_deg, ref.latitude_deg, 1e-9);
	EXPECT_NEAR(gps.vel_e_m_s, 0.f, 1e-4f);
	EXPECT_NEAR(gps.vel_n_m_s, 0.f, 1e-4f);
}

TEST(FailureInjectionGnss, WrongAltitudeDrift)
{
	sensor_gps_s gps = clean_gps();
	const sensor_gps_s ref = gps;
	GnssFailureState st;

	apply_gnss_wrong(gps, st, 3, 0); // altitude drift, onset at t=0

	gps = clean_gps();
	apply_gnss_wrong(gps, st, 3, 20000000); // +20 s

	EXPECT_LT(gps.altitude_msl_m, ref.altitude_msl_m); // altitude walked down
	EXPECT_GT(gps.vel_d_m_s, 0.f);                      // coherent downward rate
	// Horizontal position left untouched.
	EXPECT_NEAR(gps.latitude_deg, ref.latitude_deg, 1e-12);
	EXPECT_NEAR(gps.longitude_deg, ref.longitude_deg, 1e-12);
}

TEST(FailureInjectionGnss, GarbageFullLoss)
{
	sensor_gps_s gps = clean_gps();
	GnssFailureState st;

	apply_gnss_garbage(gps, st, 1, 0); // full loss

	EXPECT_EQ(gps.fix_type, 0);
	EXPECT_EQ(gps.satellites_used, 0);
	EXPECT_GT(gps.eph, 1.0e5f);
	EXPECT_FALSE(gps.vel_ned_valid);
	EXPECT_GE(gps.noise_per_ms, 90); // fluctuates around the jammed baseline
	EXPECT_EQ(gps.jamming_state, sensor_gps_s::JAMMING_STATE_DETECTED);
}

TEST(FailureInjectionGnss, GarbageDegradedJittersPositionDropsQuality)
{
	sensor_gps_s gps = clean_gps();
	const sensor_gps_s ref = gps;
	GnssFailureState st;

	apply_gnss_garbage(gps, st, 0, 12345); // degraded fix

	EXPECT_EQ(gps.fix_type, sensor_gps_s::FIX_TYPE_3D);
	EXPECT_GE(gps.satellites_used, 1);
	EXPECT_LE(gps.satellites_used, 6);
	EXPECT_GT(gps.s_variance_m_s, 70.f);
	EXPECT_GE(gps.eph, 20.f);
	EXPECT_GE(gps.automatic_gain_control, 5000);
	EXPECT_LE(gps.automatic_gain_control, 7000);
	EXPECT_EQ(gps.jamming_state, sensor_gps_s::JAMMING_STATE_DETECTED);

	// Position jitters within ~eph (bounded noise), not a coherent teleport.
	const double lat_rad = ref.latitude_deg * M_PI / 180.0;
	const double bound_lat = ((double)gps.eph / CONSTANTS_RADIUS_OF_EARTH) * (180.0 / M_PI);
	EXPECT_LT(fabs(gps.latitude_deg - ref.latitude_deg), bound_lat + 1e-12);
	EXPECT_LT(fabs(gps.longitude_deg - ref.longitude_deg), bound_lat / cos(lat_rad) + 1e-12);
}

TEST(FailureInjectionGnss, GarbageTwoPhaseLossThenDegraded)
{
	sensor_gps_s gps = clean_gps();
	GnssFailureState st;

	// Phase 1 at onset: no fix (blackout is 1..6 s, so elapsed 0 is always inside it).
	apply_gnss_garbage(gps, st, 2, 1000000);
	EXPECT_EQ(gps.fix_type, 0);

	// Phase 2 after 7 s: degraded fix returns (7 s > max 6 s blackout).
	gps = clean_gps();
	apply_gnss_garbage(gps, st, 2, 1000000 + 7000000);
	EXPECT_EQ(gps.fix_type, sensor_gps_s::FIX_TYPE_3D);
}

TEST(FailureInjectionGnss, GarbageBlackoutWithinOneToSixSeconds)
{
	// The blackout is 1..6 s, so under 1 s from onset it is always the loss phase...
	{
		sensor_gps_s gps = clean_gps();
		GnssFailureState st;
		apply_gnss_garbage(gps, st, 2, 500000);            // onset
		apply_gnss_garbage(gps, st, 2, 500000 + 500000);   // +0.5 s
		EXPECT_EQ(gps.fix_type, 0);
	}

	// ...and over 6 s it is always past the blackout (degraded fix).
	{
		sensor_gps_s gps = clean_gps();
		GnssFailureState st;
		apply_gnss_garbage(gps, st, 2, 500000);            // onset
		gps = clean_gps();
		apply_gnss_garbage(gps, st, 2, 500000 + 6500000);  // +6.5 s
		EXPECT_EQ(gps.fix_type, sensor_gps_s::FIX_TYPE_3D);
	}
}

TEST(FailureInjectionGnss, ResetClearsState)
{
	GnssFailureState st;
	st.active = Mode::Wrong;
	st.onset_us = 12345;

	gnss_reset(st);

	EXPECT_EQ(st.active, Mode::Ok);
	EXPECT_EQ(st.onset_us, 0u);
}
