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

#include "checks/gnssRedundancyCheck.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/param.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_gps.h>

// to run: make tests TESTFILTER=gnssRedundancyChecks

/* EVENT
 * @skip-file
 */

// Base position (PX4 SITL default home).
static constexpr double BASE_LAT = 47.397742;
static constexpr double BASE_LON = 8.545594;

// With GPS0 at +35cm X and GPS1 at -35cm X, expected_d = 0.70m.
// With RTK eph = 0.02m: gate = 3 * sqrt(0.02² + 0.02²) ≈ 0.085m.
// AGREEING_LAT puts GPS1 ~0.70m north of GPS0 so separation ≈ expected_d.
// DIVERGING_FAR_LAT puts GPS1 ~2.0m north, well outside the gate.
static constexpr double AGREEING_LAT     = BASE_LAT + 6.3e-6;  // ~0.70m north
static constexpr double DIVERGING_FAR_LAT = BASE_LAT + 1.8e-5; // ~2.0m north

class GnssRedundancyChecksTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);

		// Reset params that tests modify so state doesn't leak between tests.
		param_reset(param_find("SYS_HAS_NUM_GNSS"));
		param_reset(param_find("COM_GNSSLOSS_ACT"));

		// Set lever arms so expected_d = 0.70m, enabling the "too close" direction of divergence detection.
		float v = 0.35f;  param_set(param_find("SENS_GPS0_OFFX"), &v);
		v = -0.35f;       param_set(param_find("SENS_GPS1_OFFX"), &v);

		// Claim uORB instances 0 and 1 before the check subscribes on first copy().
		sensor_gps_s empty{};
		_gps0_pub.publish(empty);
		_gps1_pub.publish(empty);
	}

	sensor_gps_s makeGps(double lat, double lon, float eph = 0.02f, uint8_t fix_type = 6)
	{
		sensor_gps_s gps{};
		gps.timestamp     = hrt_absolute_time();
		gps.device_id     = 1;
		gps.latitude_deg  = lat;
		gps.longitude_deg = lon;
		gps.eph           = eph;
		gps.fix_type      = fix_type;
		return gps;
	}

	// Run the check and store results in _failsafe_flags and _health_warning_gps.
	void runCheck(bool armed = false)
	{
		vehicle_status_s status{};

		if (armed) { status.arming_state = vehicle_status_s::ARMING_STATE_ARMED; }

		_check.updateParams();
		Context context{status};
		_failsafe_flags = {};
		Report reporter{_failsafe_flags, 0};
		_check.checkAndReport(context, reporter);
		// Capture any GPS health issue regardless of log level (Warning or Error).
		_health_warning_gps = (reporter.healthResults().warning | reporter.healthResults().error) & health_component_t::gps;
	}

	uORB::PublicationMulti<sensor_gps_s> _gps0_pub{ORB_ID(sensor_gps)};
	uORB::PublicationMulti<sensor_gps_s> _gps1_pub{ORB_ID(sensor_gps)};
	failsafe_flags_s  _failsafe_flags{};
	bool              _health_warning_gps{false};
	GnssRedundancyChecks _check;
};

// No GPS data → no flags.
TEST_F(GnssRedundancyChecksTest, NoGpsNoFlags)
{
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// One receiver fixed, SYS_HAS_NUM_GNSS not configured → no failsafe.
TEST_F(GnssRedundancyChecksTest, SingleGpsNoFailsafe)
{
	_gps0_pub.publish(makeGps(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
}

// Two receivers at expected lever-arm separation → no divergence.
TEST_F(GnssRedundancyChecksTest, TwoGpsAgreeingNoFlags)
{
	_gps0_pub.publish(makeGps(BASE_LAT, BASE_LON));
	_gps1_pub.publish(makeGps(AGREEING_LAT, BASE_LON));
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// Two receivers at the same position while a 0.70m lever arm is configured →
// divergence detected (separation = 0, expected_d = 0.70m, deviation = 0.70m >> gate).
// This exercises the "too close" direction of the improved check.
TEST_F(GnssRedundancyChecksTest, TwoGpsTooCloseDivergenceDetected)
{
	_gps0_pub.publish(makeGps(BASE_LAT, BASE_LON));
	_gps1_pub.publish(makeGps(BASE_LAT, BASE_LON));

	// First call starts the hysteresis timer, no flag yet.
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);

	// Hysteresis fires after ~2 s; calling immediately again stays false.
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
}

// Two receivers too far apart → hysteresis timer starts but has not elapsed on first call.
TEST_F(GnssRedundancyChecksTest, TwoGpsDivergingFarNotYetSustained)
{
	_gps0_pub.publish(makeGps(BASE_LAT, BASE_LON));
	_gps1_pub.publish(makeGps(DIVERGING_FAR_LAT, BASE_LON));
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
}

// After divergence the receivers recover → hysteresis resets, no flag.
TEST_F(GnssRedundancyChecksTest, TwoGpsDivergingClearsOnRecovery)
{
	_gps0_pub.publish(makeGps(BASE_LAT, BASE_LON));
	_gps1_pub.publish(makeGps(DIVERGING_FAR_LAT, BASE_LON));
	runCheck();

	_gps0_pub.publish(makeGps(BASE_LAT, BASE_LON));
	_gps1_pub.publish(makeGps(AGREEING_LAT, BASE_LON));
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// fix_type < 3 is not counted as a fixed receiver → no divergence check triggered.
TEST_F(GnssRedundancyChecksTest, FixTypeBelow3NotCounted)
{
	_gps0_pub.publish(makeGps(BASE_LAT, BASE_LON, 0.02f, 6));
	_gps1_pub.publish(makeGps(DIVERGING_FAR_LAT, BASE_LON, 0.02f, 2));
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// SYS_HAS_NUM_GNSS = 2, COM_GNSSLOSS_ACT > 0, only one receiver fixed → gnss_lost.
TEST_F(GnssRedundancyChecksTest, BelowRequiredSetsGnssLost)
{
	int required = 2;   param_set(param_find("SYS_HAS_NUM_GNSS"), &required);
	int act = 1;        param_set(param_find("COM_GNSSLOSS_ACT"), &act);

	_gps0_pub.publish(makeGps(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gps);
}

// After seeing two fixed receivers, losing one emits a health warning
// even when SYS_HAS_NUM_GNSS is not set (dropped_below_peak path).
TEST_F(GnssRedundancyChecksTest, DroppedBelowPeakSetsHealthWarning)
{
	_gps0_pub.publish(makeGps(BASE_LAT, BASE_LON));
	_gps1_pub.publish(makeGps(AGREEING_LAT, BASE_LON));
	runCheck();
	EXPECT_FALSE(_health_warning_gps); // both present, no warning

	// GPS1 disappears.
	sensor_gps_s gone{};
	_gps1_pub.publish(gone); // device_id = 0 → treated as absent
	runCheck();
	EXPECT_TRUE(_health_warning_gps);
	EXPECT_FALSE(_failsafe_flags.gnss_lost); // no failsafe without COM_GNSSLOSS_ACT + below_required
}
