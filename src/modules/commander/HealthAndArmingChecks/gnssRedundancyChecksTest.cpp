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
#include <px4_platform_common/time.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_gnss.h>
#include <uORB/topics/sensors_status_gnss.h>

// to run: make tests TESTFILTER=gnssRedundancyChecks

/* EVENT
 * @skip-file
 */

// Base position (PX4 SITL default home).
static constexpr double BASE_LAT = 47.397742;
static constexpr double BASE_LON = 8.545594;

// The sensors module reports how far the receivers disagree after their lever arms. With RTK eph = 0.02 m the gate is
// 3 * sqrt(0.02² + 0.02²) ≈ 0.085 m.
static constexpr float AGREEING_M = 0.01f;
static constexpr float DIVERGING_FAR_M = 1.3f;

class GnssRedundancyChecksTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);

		// Reset params that tests modify so state doesn't leak between tests.
		param_reset(param_find("SYS_HAS_NUM_GNSS"));
		param_reset(param_find("COM_GNSSLOSS_ACT"));

		// Receiver 0 is the selected one
		_status.device_id_selected = 1;

		for (float &inconsistency : _status.inconsistency) {
			inconsistency = NAN;
		}

		// Claim uORB instances 0 and 1 before the check subscribes on first copy().
		sensor_gnss_s empty{};
		_gnss0_pub.publish(empty);
		_gnss1_pub.publish(empty);
	}

	sensor_gnss_s makeGnss(double lat, double lon, float eph = 0.02f, uint8_t fix_type = 6)
	{
		sensor_gnss_s gnss{};
		gnss.timestamp     = hrt_absolute_time();
		gnss.device_id     = 1;
		gnss.latitude      = lat;
		gnss.longitude     = lon;
		gnss.eph           = eph;
		gnss.fix_type      = fix_type;
		return gnss;
	}

	// Publish a receiver and the sensors module's status for it, healthy with a 3D fix, and how far it disagrees with
	// the selected receiver 0. Instances 0 and 1 get device IDs 1 and 2.
	void publishGnss(int instance, sensor_gnss_s gnss, float inconsistency = 0.f)
	{
		gnss.device_id = instance + 1;
		(instance == 0 ? _gnss0_pub : _gnss1_pub).publish(gnss);

		_status.device_ids[instance] = gnss.device_id;
		_status.healthy[instance] = (gnss.fix_type >= 3);
		_status.inconsistency[instance] = (instance == 0) ? 0.f : inconsistency;
		_status.timestamp = hrt_absolute_time();
		_status_pub.publish(_status);
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

	uORB::PublicationMulti<sensor_gnss_s> _gnss0_pub{ORB_ID(sensor_gnss)};
	uORB::PublicationMulti<sensor_gnss_s> _gnss1_pub{ORB_ID(sensor_gnss)};
	uORB::Publication<sensors_status_gnss_s> _status_pub{ORB_ID(sensors_status_gnss)};
	sensors_status_gnss_s _status{};
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
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
}

// Two receivers that agree after their lever arms → no divergence.
TEST_F(GnssRedundancyChecksTest, TwoGpsAgreeingNoFlags)
{
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(BASE_LAT, BASE_LON), AGREEING_M);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// Two receivers that disagree → hysteresis timer starts but has not elapsed on first call.
TEST_F(GnssRedundancyChecksTest, TwoGpsDivergingFarNotYetSustained)
{
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(BASE_LAT, BASE_LON), DIVERGING_FAR_M);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);

	// Hysteresis fires after ~2 s; calling immediately again stays false.
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
}

// A sustained divergence warns, and with SYS_HAS_NUM_GNSS = 2 it sets gnss_lost.
TEST_F(GnssRedundancyChecksTest, SustainedDivergenceSetsGnssLost)
{
	int required = 2;
	param_set(param_find("SYS_HAS_NUM_GNSS"), &required);

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(BASE_LAT, BASE_LON), DIVERGING_FAR_M);
	runCheck();
	ASSERT_FALSE(_failsafe_flags.gnss_lost);

	px4_usleep(2100000);
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(BASE_LAT, BASE_LON), DIVERGING_FAR_M);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gps);
}

// The gate scales with the reported accuracy.
TEST_F(GnssRedundancyChecksTest, InaccurateReceiversMayDisagreeMore)
{
	int required = 2;
	param_set(param_find("SYS_HAS_NUM_GNSS"), &required);

	// eph 1 m each: gate = 3 * sqrt(2) ≈ 4.2 m
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON, 1.f));
	publishGnss(1, makeGnss(BASE_LAT, BASE_LON, 1.f), DIVERGING_FAR_M);
	runCheck();
	px4_usleep(2100000);
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON, 1.f));
	publishGnss(1, makeGnss(BASE_LAT, BASE_LON, 1.f), DIVERGING_FAR_M);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// After divergence the receivers recover → hysteresis resets, no flag.
TEST_F(GnssRedundancyChecksTest, TwoGpsDivergingClearsOnRecovery)
{
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(BASE_LAT, BASE_LON), DIVERGING_FAR_M);
	runCheck();

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(BASE_LAT, BASE_LON), AGREEING_M);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// An unknown inconsistency is no divergence.
TEST_F(GnssRedundancyChecksTest, UnknownInconsistencyNotADivergence)
{
	int required = 2;
	param_set(param_find("SYS_HAS_NUM_GNSS"), &required);

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(BASE_LAT, BASE_LON), NAN);
	runCheck();
	px4_usleep(2100000);
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(BASE_LAT, BASE_LON), NAN);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
}

// A receiver that fails its checks is not counted → no divergence check triggered.
TEST_F(GnssRedundancyChecksTest, FixTypeBelow3NotCounted)
{
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON, 0.02f, 6));
	publishGnss(1, makeGnss(BASE_LAT, BASE_LON, 0.02f, 2), DIVERGING_FAR_M);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// SYS_HAS_NUM_GNSS = 2, COM_GNSSLOSS_ACT > 0, only one receiver fixed → gnss_lost.
TEST_F(GnssRedundancyChecksTest, BelowRequiredSetsGnssLost)
{
	int required = 2;   param_set(param_find("SYS_HAS_NUM_GNSS"), &required);
	int act = 1;        param_set(param_find("COM_GNSSLOSS_ACT"), &act);

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gps);
}

// After seeing two fixed receivers, losing one emits a health warning
// even when SYS_HAS_NUM_GNSS is not set (dropped_below_peak path).
TEST_F(GnssRedundancyChecksTest, DroppedBelowPeakSetsHealthWarning)
{
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(BASE_LAT, BASE_LON), AGREEING_M);
	runCheck();
	EXPECT_FALSE(_health_warning_gps); // both present, no warning

	// GPS1 disappears.
	sensor_gnss_s gone{};
	_gnss1_pub.publish(gone); // device_id = 0 → treated as absent
	runCheck();
	EXPECT_TRUE(_health_warning_gps);
	EXPECT_FALSE(_failsafe_flags.gnss_lost); // no failsafe without COM_GNSSLOSS_ACT + below_required
}

// A receiver counts only while the status names it and says it passes its checks.
TEST_F(GnssRedundancyChecksTest, StatusOfAnotherReceiverDoesNotCount)
{
	int required = 1;
	param_set(param_find("SYS_HAS_NUM_GNSS"), &required);

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);

	_status.device_ids[0] = 99;
	_status_pub.publish(_status);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
}

// A status older than 1 s doesn't count, however fresh the receiver's own data is.
TEST_F(GnssRedundancyChecksTest, StaleStatusDoesNotCount)
{
	int required = 1;
	param_set(param_find("SYS_HAS_NUM_GNSS"), &required);

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	ASSERT_FALSE(_failsafe_flags.gnss_lost);

	_status.timestamp = hrt_absolute_time() - 2000000; // 2 s
	_status_pub.publish(_status);
	_gnss0_pub.publish(makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
}
