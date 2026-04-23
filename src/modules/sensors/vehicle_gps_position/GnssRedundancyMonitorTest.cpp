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

#include <gtest/gtest.h>

#include "GnssRedundancyMonitor.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/gnss_redundancy_status.h>
#include <uORB/topics/sensor_gps.h>

// to run: make tests TESTFILTER=GnssRedundancyMonitor

// Base position (PX4 SITL default home)
static constexpr double BASE_LAT = 47.397742;
static constexpr double BASE_LON = 8.545594;

// 1 deg latitude ≈ 111111 m
// AGREEING:  ~0.4m north  → below gate of 0.785m (0.70m lever arm + 0.085m eph tolerance)
// DIVERGING: ~2.0m north  → above gate
static constexpr double AGREEING_LAT  = BASE_LAT + 3.6e-6;
static constexpr double DIVERGING_LAT = BASE_LAT + 1.8e-5;

class GnssRedundancyMonitorTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);

		int required = 0;
		param_set(param_find("SYS_HAS_NUM_GNSS"), &required);

		_monitor = new sensors::GnssRedundancyMonitor(nullptr);

		// Lever arm: GPS0 at +35cm forward, GPS1 at -35cm → expected_d = 0.70m
		// With both RTK Fixed (eph=0.02m): gate = 0.70 + 3*√(0.02²+0.02²) = 0.785m
		_offsets[0] = matrix::Vector3f(0.35f, 0.f, 0.f);
		_offsets[1] = matrix::Vector3f(-0.35f, 0.f, 0.f);

		_states[0] = sensor_gps_s{};
		_states[1] = sensor_gps_s{};
	}

	void TearDown() override
	{
		delete _monitor;
		_monitor = nullptr;
	}

	sensor_gps_s makeGps(double lat, double lon, float eph = 0.02f, uint8_t fix_type = 6)
	{
		sensor_gps_s gps{};
		gps.timestamp            = hrt_absolute_time();
		gps.device_id            = 1;
		gps.latitude_deg         = lat;
		gps.longitude_deg        = lon;
		gps.altitude_ellipsoid_m = 100.0;
		gps.eph                  = eph;
		gps.epv                  = 0.05f;
		gps.fix_type             = fix_type;
		return gps;
	}

	gnss_redundancy_status_s runUpdate(bool armed)
	{
		_monitor->update(_states, _offsets, 2, armed);
		gnss_redundancy_status_s status{};
		_status_sub.copy(&status);
		return status;
	}

	sensor_gps_s _states[2] {};
	matrix::Vector3f _offsets[2] {};
	sensors::GnssRedundancyMonitor *_monitor{nullptr};
	uORB::Subscription _status_sub{ORB_ID(gnss_redundancy_status)};
};

// No data → no divergence
TEST_F(GnssRedundancyMonitorTest, NoGnss)
{
	auto status = runUpdate(false);
	EXPECT_FALSE(status.divergence_detected);
	EXPECT_EQ(status.num_receivers_fixed, 0);
}

// Only one fixed receiver → divergence check requires ≥ 2
TEST_F(GnssRedundancyMonitorTest, SingleGnss)
{
	_states[0] = makeGps(BASE_LAT, BASE_LON);
	auto status = runUpdate(false);
	EXPECT_FALSE(status.divergence_detected);
	EXPECT_EQ(status.num_receivers_fixed, 1);
}

// Two receivers within gate → no divergence
TEST_F(GnssRedundancyMonitorTest, TwoGnssAgreeingPrearm)
{
	_states[0] = makeGps(BASE_LAT, BASE_LON);
	_states[1] = makeGps(AGREEING_LAT, BASE_LON);
	auto status = runUpdate(false);
	EXPECT_FALSE(status.divergence_detected);
}

// Two receivers beyond gate, pre-arm → immediate divergence (sustain=0 pre-arm)
TEST_F(GnssRedundancyMonitorTest, TwoGnssDivergingPrearm)
{
	_states[0] = makeGps(BASE_LAT, BASE_LON);
	_states[1] = makeGps(DIVERGING_LAT, BASE_LON);
	auto status = runUpdate(false);
	EXPECT_TRUE(status.divergence_detected);
}

// Two receivers beyond gate, in-flight, first call → sustain not elapsed
TEST_F(GnssRedundancyMonitorTest, TwoGnssDivergingInflightNotYetSustained)
{
	_states[0] = makeGps(BASE_LAT, BASE_LON);
	_states[1] = makeGps(DIVERGING_LAT, BASE_LON);
	auto status = runUpdate(true);
	EXPECT_FALSE(status.divergence_detected);
}

// fix_type < 3 → treated as inactive → only 1 fixed receiver → no divergence check
TEST_F(GnssRedundancyMonitorTest, FixTypeBelow3TreatedAsInactive)
{
	_states[0] = makeGps(BASE_LAT, BASE_LON);
	_states[1] = makeGps(DIVERGING_LAT, BASE_LON, 0.02f, 2);
	auto status = runUpdate(false);
	EXPECT_FALSE(status.divergence_detected);
	EXPECT_EQ(status.num_receivers_fixed, 1);
}

// Divergence then recovery: flag clears when positions agree again
TEST_F(GnssRedundancyMonitorTest, DivergenceClearsAfterRecovery)
{
	_states[0] = makeGps(BASE_LAT, BASE_LON);
	_states[1] = makeGps(DIVERGING_LAT, BASE_LON);
	EXPECT_TRUE(runUpdate(false).divergence_detected);

	_states[0] = makeGps(BASE_LAT, BASE_LON);
	_states[1] = makeGps(AGREEING_LAT, BASE_LON);
	EXPECT_FALSE(runUpdate(false).divergence_detected);
}

// Below-required is reported when fixed count < SYS_HAS_NUM_GNSS
TEST_F(GnssRedundancyMonitorTest, BelowRequiredFlag)
{
	int required = 2;
	param_set(param_find("SYS_HAS_NUM_GNSS"), &required);
	delete _monitor;
	_monitor = new sensors::GnssRedundancyMonitor(nullptr);

	_states[0] = makeGps(BASE_LAT, BASE_LON);
	auto status = runUpdate(false);
	EXPECT_TRUE(status.below_required);
	EXPECT_EQ(status.num_receivers_fixed, 1);
	EXPECT_EQ(status.num_receivers_required, 2);
}

// Peak tracking: once we've seen 2 fixed, losing one is reported as dropped_below_peak
TEST_F(GnssRedundancyMonitorTest, DroppedBelowPeakFlag)
{
	_states[0] = makeGps(BASE_LAT, BASE_LON);
	_states[1] = makeGps(AGREEING_LAT, BASE_LON);
	auto status = runUpdate(false);
	EXPECT_EQ(status.peak_receivers_fixed, 2);

	_states[1] = sensor_gps_s{};
	status = runUpdate(false);
	EXPECT_TRUE(status.dropped_below_peak);
	EXPECT_EQ(status.num_receivers_fixed, 1);
	EXPECT_EQ(status.peak_receivers_fixed, 2);
}
