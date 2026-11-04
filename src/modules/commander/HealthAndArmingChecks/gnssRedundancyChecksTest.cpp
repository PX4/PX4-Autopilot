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

// With GPS0 at +35cm X and GPS1 at -35cm X, expected_d = 0.70m.
// The reported eph (RTK, 0.02m) is floored with EKF2_GPS_P_NOISE = 0.5m:
// gate = 3 * sqrt(0.5² + 0.5²) ≈ 2.12m.
// AGREEING_LAT puts GPS1 ~0.70m north of GPS0 so separation ≈ expected_d.
// DIVERGING_FAR_LAT puts GPS1 ~5.0m north, well outside the gate.
static constexpr double AGREEING_LAT     = BASE_LAT + 6.3e-6;  // ~0.70m north
static constexpr double DIVERGING_FAR_LAT = BASE_LAT + 4.5e-5; // ~5.0m north

// Divergence has to be sustained for the 2s hysteresis time before it is reported.
static constexpr useconds_t HYSTERESIS_WAIT_US = 2100000;

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
		float v = 0.35f;  param_set(param_find("SENS_GNSS0_OFFX"), &v);
		v = -0.35f;       param_set(param_find("SENS_GNSS1_OFFX"), &v);

		// Floor for the reported accuracy, the test geometry above assumes 0.5m.
		v = 0.5f;         param_set(param_find("EKF2_GPS_P_NOISE"), &v);

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

	// Publish a receiver and the sensors module's status for it, healthy with a 3D fix. Instances 0 and 1 get device IDs
	// 1 and 2.
	void publishGnss(int instance, sensor_gnss_s gnss)
	{
		gnss.device_id = instance + 1;
		(instance == 0 ? _gnss0_pub : _gnss1_pub).publish(gnss);

		_status.device_ids[instance] = gnss.device_id;
		_status.healthy[instance] = (gnss.fix_type >= 3);
		_status.timestamp = hrt_absolute_time();
		_status_pub.publish(_status);
	}

	// Run the check and store results in _failsafe_flags, _health_warning_gps and _can_arm_all_modes.
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
		_can_arm_all_modes = reporter.armingCheckResults().can_arm == NavModes::All;
	}

	void setGnssLossAct(int32_t com_gnssloss_act)
	{
		param_set(param_find("COM_GNSSLOSS_ACT"), &com_gnssloss_act);
	}

	// Two fixed receivers diverging well beyond the gate with SYS_HAS_NUM_GNSS = 2, held past the hysteresis time.
	void runSustainedDivergence()
	{
		int required = 2;
		param_set(param_find("SYS_HAS_NUM_GNSS"), &required);

		publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
		publishGnss(1, makeGnss(DIVERGING_FAR_LAT, BASE_LON));
		runCheck();

		px4_usleep(HYSTERESIS_WAIT_US);
		publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
		publishGnss(1, makeGnss(DIVERGING_FAR_LAT, BASE_LON));
		runCheck();
	}

	uORB::PublicationMulti<sensor_gnss_s> _gnss0_pub{ORB_ID(sensor_gnss)};
	uORB::PublicationMulti<sensor_gnss_s> _gnss1_pub{ORB_ID(sensor_gnss)};
	uORB::Publication<sensors_status_gnss_s> _status_pub{ORB_ID(sensors_status_gnss)};
	sensors_status_gnss_s _status{};
	failsafe_flags_s  _failsafe_flags{};
	bool              _health_warning_gps{false};
	bool              _can_arm_all_modes{false};
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

// Two receivers at expected lever-arm separation → no divergence.
TEST_F(GnssRedundancyChecksTest, TwoGpsAgreeingNoFlags)
{
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(AGREEING_LAT, BASE_LON));
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// Two receivers at the same position while a 0.70m lever arm is configured → 0.70m deviation.
// Far outside the gate from the reported RTK accuracy (≈0.085m) but within the floored gate (≈2.12m),
// so it is tolerated even once the hysteresis time has elapsed.
TEST_F(GnssRedundancyChecksTest, TwoGpsTooCloseToleratedByAccuracyFloor)
{
	int required = 2;   param_set(param_find("SYS_HAS_NUM_GNSS"), &required);

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(BASE_LAT, BASE_LON));
	runCheck();

	px4_usleep(HYSTERESIS_WAIT_US);
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// Two receivers too far apart → hysteresis timer starts but has not elapsed on first call.
TEST_F(GnssRedundancyChecksTest, TwoGpsDivergingFarNotYetSustained)
{
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(DIVERGING_FAR_LAT, BASE_LON));
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
}

// Sustained divergence with SYS_HAS_NUM_GNSS = 2 → gnss_lost, but COM_GNSSLOSS_ACT = Warning does not block arming.
TEST_F(GnssRedundancyChecksTest, TwoGpsDivergingFarSustainedSetsGnssLost)
{
	runSustainedDivergence();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gps);
	EXPECT_TRUE(_can_arm_all_modes);
}

// Sustained divergence with COM_GNSSLOSS_ACT = Return → gnss_lost and arming blocked.
TEST_F(GnssRedundancyChecksTest, TwoGpsDivergingFarReturnBlocksArming)
{
	setGnssLossAct(1); // Return
	runSustainedDivergence();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gps);
	EXPECT_FALSE(_can_arm_all_modes);
}

// After divergence the receivers recover → hysteresis resets, no flag.
TEST_F(GnssRedundancyChecksTest, TwoGpsDivergingClearsOnRecovery)
{
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(DIVERGING_FAR_LAT, BASE_LON));
	runCheck();

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(AGREEING_LAT, BASE_LON));
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// A receiver that fails its checks is not counted → no divergence check triggered.
TEST_F(GnssRedundancyChecksTest, FixTypeBelow3NotCounted)
{
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON, 0.02f, 6));
	publishGnss(1, makeGnss(DIVERGING_FAR_LAT, BASE_LON, 0.02f, 2));
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// SYS_HAS_NUM_GNSS = 2, COM_GNSSLOSS_ACT = Return, only one receiver fixed → gnss_lost and arming blocked.
TEST_F(GnssRedundancyChecksTest, BelowRequiredSetsGnssLost)
{
	int required = 2;   param_set(param_find("SYS_HAS_NUM_GNSS"), &required);
	setGnssLossAct(1); // Return

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gps);
	EXPECT_FALSE(_can_arm_all_modes);
}

// COM_GNSSLOSS_ACT = "Warning, block arming" blocks arming without any in-flight action (gnss_lost is still set for the warning).
TEST_F(GnssRedundancyChecksTest, BelowRequiredBlockArmingBlocksArming)
{
	int required = 2;   param_set(param_find("SYS_HAS_NUM_GNSS"), &required);
	setGnssLossAct(4); // Warning, block arming

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gps);
	EXPECT_FALSE(_can_arm_all_modes);
}

// COM_GNSSLOSS_ACT = Warning reports the issue but never blocks arming.
TEST_F(GnssRedundancyChecksTest, BelowRequiredWarningDoesNotBlockArming)
{
	int required = 2;   param_set(param_find("SYS_HAS_NUM_GNSS"), &required);
	setGnssLossAct(0); // Warning

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gps);
	EXPECT_TRUE(_can_arm_all_modes);
}

// After seeing two fixed receivers, losing one emits a health warning
// even when SYS_HAS_NUM_GNSS is not set (dropped_below_peak path).
TEST_F(GnssRedundancyChecksTest, DroppedBelowPeakSetsHealthWarning)
{
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	publishGnss(1, makeGnss(AGREEING_LAT, BASE_LON));
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
