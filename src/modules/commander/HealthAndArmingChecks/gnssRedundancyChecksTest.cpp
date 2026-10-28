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
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/param.h>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_gnss.h>
#include <uORB/topics/sensors_status_gnss.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

// to run: make tests TESTFILTER=gnssRedundancyChecks

/* EVENT
 * @skip-file
 */

// Base position (PX4 SITL default home).
static constexpr double BASE_LAT = 47.397742;
static constexpr double BASE_LON = 8.545594;
static constexpr double BASE_ALT = 488.0;

// With GNSS0 at +35cm X and GNSS1 at -35cm X, GNSS1 is expected 0.70m behind GNSS0.
// The vehicle is level and heading north with an aligned heading unless a test sets otherwise.
// The divergence limits COM_GNSS_H_MAX and COM_GNSS_V_MAX default to 2m.
// AGREEING_LAT puts GNSS1 ~0.70m south of GNSS0, matching the expected baseline.
// DIVERGING_FAR_LAT puts GNSS1 ~5.0m south, ~4.3m off the expected position, above the limit.
// SWAPPED_LAT puts GNSS1 ~0.70m north, i.e. where it is expected when heading south.
static constexpr double AGREEING_LAT      = BASE_LAT - 6.3e-6; // ~0.70m south
static constexpr double DIVERGING_FAR_LAT = BASE_LAT - 4.5e-5; // ~5.0m south
static constexpr double SWAPPED_LAT       = BASE_LAT + 6.3e-6; // ~0.70m north

// In flight a divergence has to be sustained for the 2s hysteresis time before it is reported.
// Before arming it is reported immediately and only clears after 2s of continuous agreement.
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
		param_reset(param_find("COM_GNSS_H_MAX"));
		param_reset(param_find("COM_GNSS_V_MAX"));
		param_reset(param_find("SENS_GNSS0_ID"));
		param_reset(param_find("SENS_GNSS1_ID"));
		param_reset(param_find("SENS_GNSS0_OFFZ"));
		param_reset(param_find("SENS_GNSS1_OFFZ"));

		// Set lever arms so the expected separation is 0.70m, enabling the "too close" direction of divergence detection.
		float v = 0.35f;  param_set(param_find("SENS_GNSS0_OFFX"), &v);
		v = -0.35f;       param_set(param_find("SENS_GNSS1_OFFX"), &v);

		// Claim uORB instances 0 and 1 before the check subscribes on first copy().
		sensor_gnss_s empty{};
		_gnss0_pub.publish(empty);
		_gnss1_pub.publish(empty);
	}

	sensor_gnss_s makeGnss(double lat, double lon, float eph = 0.02f, uint8_t fix_type = 6, double alt = BASE_ALT)
	{
		sensor_gnss_s gnss{};
		gnss.timestamp    = hrt_absolute_time();
		gnss.device_id    = 1;
		gnss.latitude     = lat;
		gnss.longitude    = lon;
		gnss.altitude_msl = alt;
		gnss.eph          = eph;
		gnss.fix_type     = fix_type;
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

	// Publish GNSS0 at the base position and GNSS1 at the given latitude/altitude, both healthy.
	void publishBoth(double gnss1_lat, double gnss1_alt = BASE_ALT)
	{
		publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
		publishGnss(1, makeGnss(gnss1_lat, BASE_LON, 0.02f, 6, gnss1_alt));
	}

	void setRequiredGnss(int32_t sys_has_num_gnss)
	{
		param_set(param_find("SYS_HAS_NUM_GNSS"), &sys_has_num_gnss);
	}

	void setFloatParam(const char *name, float value)
	{
		param_set(param_find(name), &value);
	}

	void setAttitude(float roll, float pitch, float yaw)
	{
		_attitude_q = matrix::Quatf(matrix::Eulerf(roll, pitch, yaw));
	}

	// Run the check and store results in _failsafe_flags, _health_warning_gps and _can_arm_all_modes.
	void runCheck(bool armed = false)
	{
		// Publish the attitude and heading state on every run so they are never stale after a hysteresis wait.
		vehicle_attitude_s attitude{};
		attitude.timestamp = hrt_absolute_time();
		_attitude_q.copyTo(attitude.q);
		_attitude_pub.publish(attitude);

		vehicle_local_position_s local_position{};
		local_position.timestamp = hrt_absolute_time();
		local_position.heading_good_for_control = _heading_aligned;
		_local_position_pub.publish(local_position);

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

	// Two healthy receivers diverging well beyond the limit with SYS_HAS_NUM_GNSS = 2, held past the hysteresis time.
	void runSustainedDivergence(bool armed = false)
	{
		setRequiredGnss(2);

		publishBoth(DIVERGING_FAR_LAT);
		runCheck(armed);

		px4_usleep(HYSTERESIS_WAIT_US);
		publishBoth(DIVERGING_FAR_LAT);
		runCheck(armed);
	}

	uORB::PublicationMulti<sensor_gnss_s> _gnss0_pub{ORB_ID(sensor_gnss)};
	uORB::PublicationMulti<sensor_gnss_s> _gnss1_pub{ORB_ID(sensor_gnss)};
	uORB::Publication<sensors_status_gnss_s> _status_pub{ORB_ID(sensors_status_gnss)};
	uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_local_position_s> _local_position_pub{ORB_ID(vehicle_local_position)};
	sensors_status_gnss_s _status{};
	matrix::Quatf     _attitude_q{}; // level, heading north
	bool              _heading_aligned{true};
	failsafe_flags_s  _failsafe_flags{};
	bool              _health_warning_gps{false};
	bool              _can_arm_all_modes{false};
	GnssRedundancyChecks _check;
};

// No GNSS data → no flags.
TEST_F(GnssRedundancyChecksTest, NoGpsNoFlags)
{
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// One healthy receiver, SYS_HAS_NUM_GNSS not configured → no failsafe.
TEST_F(GnssRedundancyChecksTest, SingleGpsNoFailsafe)
{
	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
}

// Two receivers at the expected lever-arm separation → no divergence.
TEST_F(GnssRedundancyChecksTest, TwoGpsAgreeingNoFlags)
{
	publishBoth(AGREEING_LAT);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// Two receivers at the same position while a 0.70m lever arm is configured → 0.70m deviation.
// Far outside the reported RTK accuracy (0.02m) but within the 2m limit,
// so it is tolerated even once the hysteresis time has elapsed.
TEST_F(GnssRedundancyChecksTest, TwoGpsTooCloseWithinLimit)
{
	setRequiredGnss(2);

	publishBoth(BASE_LAT);
	runCheck();

	px4_usleep(HYSTERESIS_WAIT_US);
	publishBoth(BASE_LAT);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// In flight, two receivers too far apart → hysteresis timer starts but has not elapsed on first call.
TEST_F(GnssRedundancyChecksTest, TwoGpsDivergingFarInFlightNotYetSustained)
{
	setRequiredGnss(2);

	publishBoth(DIVERGING_FAR_LAT);
	runCheck(true);
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// In flight, sustained divergence with SYS_HAS_NUM_GNSS = 2 → gnss_lost, but COM_GNSSLOSS_ACT = Warning does not block arming.
TEST_F(GnssRedundancyChecksTest, TwoGpsDivergingFarSustainedSetsGnssLost)
{
	runSustainedDivergence(true);
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gps);
	EXPECT_TRUE(_can_arm_all_modes);
}

// Before arming, divergence is reported immediately and blocks arming without waiting for the hysteresis time.
TEST_F(GnssRedundancyChecksTest, TwoGpsDivergingFarBeforeArmingImmediate)
{
	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming

	publishBoth(DIVERGING_FAR_LAT);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gps);
	EXPECT_FALSE(_can_arm_all_modes);
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

// In flight, a divergence that recovers before the hysteresis time → no flag.
TEST_F(GnssRedundancyChecksTest, TwoGpsDivergingInFlightShortGlitchIgnored)
{
	setRequiredGnss(2);

	publishBoth(DIVERGING_FAR_LAT);
	runCheck(true);

	publishBoth(AGREEING_LAT);
	runCheck(true);
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
}

// Before arming, a divergence only clears after the receivers agree continuously for the hysteresis time.
TEST_F(GnssRedundancyChecksTest, TwoGpsDivergingBeforeArmingClearsAfterContinuousAgreement)
{
	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming

	publishBoth(DIVERGING_FAR_LAT);
	runCheck();
	EXPECT_FALSE(_can_arm_all_modes);

	// Agreeing again, but not yet for long enough.
	publishBoth(AGREEING_LAT);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_can_arm_all_modes);

	px4_usleep(HYSTERESIS_WAIT_US);
	publishBoth(AGREEING_LAT);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
	EXPECT_TRUE(_can_arm_all_modes);
}

// Before arming, a new disagreement while waiting restarts the 2s wait.
TEST_F(GnssRedundancyChecksTest, TwoGpsDivergingBeforeArmingDisagreementRestartsWait)
{
	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming

	publishBoth(DIVERGING_FAR_LAT);
	runCheck();
	publishBoth(AGREEING_LAT);
	runCheck();

	px4_usleep(1500000);
	publishBoth(DIVERGING_FAR_LAT); // restarts the wait
	runCheck();
	publishBoth(AGREEING_LAT);
	runCheck();

	px4_usleep(1500000);
	publishBoth(AGREEING_LAT);
	runCheck();
	EXPECT_FALSE(_can_arm_all_modes); // only 1.5s of agreement since the last disagreement

	px4_usleep(600000);
	publishBoth(AGREEING_LAT);
	runCheck();
	EXPECT_TRUE(_can_arm_all_modes);
}

// Vertical disagreement above COM_GNSS_V_MAX (default 2m) → divergence, even if horizontally agreeing.
TEST_F(GnssRedundancyChecksTest, VerticalDivergenceBlocksArming)
{
	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming

	publishBoth(AGREEING_LAT, BASE_ALT + 2.5);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gps);
	EXPECT_FALSE(_can_arm_all_modes);
}

// Vertical disagreement below COM_GNSS_V_MAX → no flags.
TEST_F(GnssRedundancyChecksTest, VerticalWithinLimitNoFlags)
{
	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming

	publishBoth(AGREEING_LAT, BASE_ALT + 1.5);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gps);
	EXPECT_TRUE(_can_arm_all_modes);
}

// The expected antenna height difference (SENS_GNSSx_OFFZ) is removed before applying the vertical limit.
TEST_F(GnssRedundancyChecksTest, VerticalOffsetCompensated)
{
	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming
	setFloatParam("SENS_GNSS1_OFFZ", -1.f); // GNSS1 antenna 1m above GNSS0 (FRD frame)

	// 2.8m measured - 1m expected = 1.8m < 2m
	publishBoth(AGREEING_LAT, BASE_ALT + 2.8);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_can_arm_all_modes);
}

// The sign of the height difference is accounted for: GNSS1 reporting 1.5m below GNSS0 while mounted 1m above is 2.5m off.
TEST_F(GnssRedundancyChecksTest, VerticalOffsetSignAccounted)
{
	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming
	setFloatParam("SENS_GNSS1_OFFZ", -1.f); // GNSS1 antenna 1m above GNSS0 (FRD frame)

	publishBoth(AGREEING_LAT, BASE_ALT - 1.5);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_can_arm_all_modes);
}

// Heading south, the rear antenna (GNSS1) is expected north of GNSS0.
TEST_F(GnssRedundancyChecksTest, HeadingAccountedFor)
{
	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming
	setFloatParam("COM_GNSS_H_MAX", 1.f);
	setAttitude(0.f, 0.f, math::radians(180.f));

	publishBoth(SWAPPED_LAT);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_can_arm_all_modes);
}

// Heading north, GNSS1 reported in front of GNSS0 instead of behind is 1.4m off, even though the distance matches.
TEST_F(GnssRedundancyChecksTest, SwappedReceiversDetected)
{
	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming
	setFloatParam("COM_GNSS_H_MAX", 1.f);

	publishBoth(SWAPPED_LAT);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_can_arm_all_modes);
}

// Without an aligned heading (e.g. no compass before takeoff), the unaligned heading estimate is not trusted:
// only the horizontal distances are compared, so large disagreements are still detected.
TEST_F(GnssRedundancyChecksTest, HeadingNotAlignedDistanceOnly)
{
	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming
	setFloatParam("COM_GNSS_H_MAX", 1.f);
	_heading_aligned = false;

	// Vehicle actually heading south while the unaligned estimate says north: no false 1.4m divergence.
	publishBoth(SWAPPED_LAT);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_can_arm_all_modes);

	// 5.0m measured - 0.7m expected = 4.3m > 1m
	publishBoth(DIVERGING_FAR_LAT);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_can_arm_all_modes);
}

// Pitched 30° nose up, the rear antenna (GNSS1) is 0.61m behind and 0.35m below GNSS0.
TEST_F(GnssRedundancyChecksTest, TiltAccountedFor)
{
	static constexpr double TILTED_LAT = BASE_LAT - 5.45e-6; // ~0.61m south

	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming
	setFloatParam("COM_GNSS_V_MAX", .2f);
	setAttitude(0.f, math::radians(30.f), 0.f);

	publishBoth(TILTED_LAT, BASE_ALT - 0.35);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_can_arm_all_modes);

	// The same positions on a level vehicle are 0.35m off vertically.
	setAttitude(0.f, 0.f, 0.f);
	publishBoth(TILTED_LAT, BASE_ALT - 0.35);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_can_arm_all_modes);
}

// With SENS_GNSSx_ID set, the antenna offsets follow the device ID, not the sensor_gnss instance.
TEST_F(GnssRedundancyChecksTest, OffsetsMatchedByDeviceId)
{
	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming
	setFloatParam("COM_GNSS_H_MAX", 1.f);

	// Instance 0 (device ID 1) is the rear receiver (slot 1) and instance 1 (device ID 2) the front one (slot 0),
	// so instance 1 is expected 0.70m north.
	int32_t id = 2; param_set(param_find("SENS_GNSS0_ID"), &id);
	id = 1;         param_set(param_find("SENS_GNSS1_ID"), &id);

	publishBoth(SWAPPED_LAT);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_can_arm_all_modes);
}

// The reported accuracy does not widen the limit.
TEST_F(GnssRedundancyChecksTest, ReportedAccuracyIgnored)
{
	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON, 10.f));
	publishGnss(1, makeGnss(DIVERGING_FAR_LAT, BASE_LON, 10.f));
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_can_arm_all_modes);
}

// The limits are configurable.
TEST_F(GnssRedundancyChecksTest, LimitsConfigurable)
{
	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming

	// ~4.3m horizontal deviation tolerated with a 5m limit.
	setFloatParam("COM_GNSS_H_MAX", 5.f);
	publishBoth(DIVERGING_FAR_LAT);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_can_arm_all_modes);

	// 1m vertical deviation flagged with a 0.5m limit.
	setFloatParam("COM_GNSS_V_MAX", .5f);
	publishBoth(AGREEING_LAT, BASE_ALT + 1.0);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_can_arm_all_modes);
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

// SYS_HAS_NUM_GNSS = 2, COM_GNSSLOSS_ACT = Return, only one receiver healthy → gnss_lost and arming blocked.
TEST_F(GnssRedundancyChecksTest, BelowRequiredSetsGnssLost)
{
	setRequiredGnss(2);
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
	setRequiredGnss(2);
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
	setRequiredGnss(2);
	setGnssLossAct(0); // Warning

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gps);
	EXPECT_TRUE(_can_arm_all_modes);
}

// After seeing two healthy receivers, losing one emits a health warning
// even when SYS_HAS_NUM_GNSS is not set (dropped_below_peak path).
TEST_F(GnssRedundancyChecksTest, DroppedBelowPeakSetsHealthWarning)
{
	publishBoth(AGREEING_LAT);
	runCheck();
	EXPECT_FALSE(_health_warning_gps); // both present, no warning

	// GNSS1 disappears.
	sensor_gnss_s gone{};
	_gnss1_pub.publish(gone); // device_id = 0 → treated as absent
	runCheck();
	EXPECT_TRUE(_health_warning_gps);
	EXPECT_FALSE(_failsafe_flags.gnss_lost); // no failsafe without COM_GNSSLOSS_ACT + below_required
}

// A receiver counts only while the status names it and says it passes its checks.
TEST_F(GnssRedundancyChecksTest, StatusOfAnotherReceiverDoesNotCount)
{
	setRequiredGnss(1);

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
	setRequiredGnss(1);

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	ASSERT_FALSE(_failsafe_flags.gnss_lost);

	_status.timestamp = hrt_absolute_time() - 2000000; // 2 s
	_status_pub.publish(_status);
	_gnss0_pub.publish(makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
}
