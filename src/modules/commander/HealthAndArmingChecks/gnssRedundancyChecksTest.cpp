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

// The sensors module compares the receivers; these tests publish its result. GNSS0 (device ID 1) is the selected
// receiver, and the limits COM_GNSS_H_MAX and COM_GNSS_V_MAX default to 2 m.
static constexpr float RTK_ACCURACY = 0.02f;
static constexpr float AGREEING = 0.3f;
static constexpr float WITHIN_LIMIT = 1.5f;
static constexpr float DIVERGING = 5.f;

// In flight a divergence has to be sustained for the 2 s hysteresis time before it is reported. Before arming it is
// reported at once and only clears after 2 s of continuous agreement.
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

		// Claim uORB instances 0 and 1 before the check subscribes on first copy().
		sensor_gnss_s empty{};
		_gnss0_pub.publish(empty);
		_gnss1_pub.publish(empty);

		_status.device_id_selected = 1;
	}

	// Publish a receiver and the sensors module's status for it, healthy with a 3D fix. Instances 0 and 1 get device IDs
	// 1 and 2.
	void publishGnss(int instance, float eph = RTK_ACCURACY, float epv = RTK_ACCURACY, uint8_t fix_type = 6)
	{
		sensor_gnss_s gnss{};
		gnss.timestamp = hrt_absolute_time();
		gnss.device_id = instance + 1;
		gnss.eph = eph;
		gnss.epv = epv;
		gnss.fix_type = fix_type;
		(instance == 0 ? _gnss0_pub : _gnss1_pub).publish(gnss);

		_status.device_ids[instance] = gnss.device_id;
		_status.healthy[instance] = (fix_type >= 3);
		_status.timestamp = hrt_absolute_time();
		_status_pub.publish(_status);
	}

	// Publish both receivers, GNSS1 the given distance from where GNSS0 and the antenna positions put it
	void publishBoth(float horizontal, float vertical = 0.f, float accuracy = RTK_ACCURACY)
	{
		_status.inconsistency_horizontal[0] = 0.f;
		_status.inconsistency_vertical[0] = 0.f;
		_status.inconsistency_horizontal[1] = horizontal;
		_status.inconsistency_vertical[1] = vertical;
		publishGnss(0, accuracy, accuracy);
		publishGnss(1, accuracy, accuracy);
	}

	void setRequiredGnss(int32_t sys_has_num_gnss)
	{
		param_set(param_find("SYS_HAS_NUM_GNSS"), &sys_has_num_gnss);
	}

	void setGnssLossAct(int32_t com_gnssloss_act)
	{
		param_set(param_find("COM_GNSSLOSS_ACT"), &com_gnssloss_act);
	}

	void setFloatParam(const char *name, float value)
	{
		param_set(param_find(name), &value);
	}

	// Run the check and store results in _failsafe_flags, _health_warning_gnss and _can_arm_all_modes.
	void runCheck(bool armed = false)
	{
		vehicle_status_s status{};

		if (armed) { status.arming_state = vehicle_status_s::ARMING_STATE_ARMED; }

		_check.updateParams();
		Context context{status};
		_failsafe_flags = {};
		Report reporter{_failsafe_flags, 0};
		_check.checkAndReport(context, reporter);
		// Capture any GNSS health issue regardless of log level (Warning or Error).
		_health_warning_gnss = (reporter.healthResults().warning | reporter.healthResults().error) & health_component_t::gps;
		_can_arm_all_modes = reporter.armingCheckResults().can_arm == NavModes::All;
		_arming_warning_gnss = reporter.armingCheckResults().warning & health_component_t::gps;
		_arming_error_gnss = reporter.armingCheckResults().error & health_component_t::gps;
		_can_arm = reporter.armingCheckResults().can_arm;
	}

	// The receivers report the given disagreement past the hysteresis time
	void runSustained(float horizontal, float vertical = 0.f, bool armed = false, float accuracy = RTK_ACCURACY)
	{
		publishBoth(horizontal, vertical, accuracy);
		runCheck(armed);

		px4_usleep(HYSTERESIS_WAIT_US);
		publishBoth(horizontal, vertical, accuracy);
		runCheck(armed);
	}

	uORB::PublicationMulti<sensor_gnss_s> _gnss0_pub{ORB_ID(sensor_gnss)};
	uORB::PublicationMulti<sensor_gnss_s> _gnss1_pub{ORB_ID(sensor_gnss)};
	uORB::Publication<sensors_status_gnss_s> _status_pub{ORB_ID(sensors_status_gnss)};
	sensors_status_gnss_s _status{};
	failsafe_flags_s  _failsafe_flags{};
	bool              _health_warning_gnss{false};
	bool              _can_arm_all_modes{false};
	bool              _arming_warning_gnss{false};
	bool              _arming_error_gnss{false};
	NavModes          _can_arm{NavModes::None};
	GnssRedundancyChecks _check;
};

// No GNSS data → no flags.
TEST_F(GnssRedundancyChecksTest, NoGnssNoFlags)
{
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gnss);
}

// One receiver, SYS_HAS_NUM_GNSS not configured → no failsafe.
TEST_F(GnssRedundancyChecksTest, SingleGnssNoFailsafe)
{
	publishGnss(0);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
}

// Receivers that agree → no flags.
TEST_F(GnssRedundancyChecksTest, AgreeingNoFlags)
{
	setRequiredGnss(2);
	runSustained(AGREEING, AGREEING);
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gnss);
}

// A disagreement far outside the reported RTK accuracy but within the limits is tolerated.
TEST_F(GnssRedundancyChecksTest, WithinLimitsTolerated)
{
	setRequiredGnss(2);
	runSustained(WITHIN_LIMIT, WITHIN_LIMIT);
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gnss);
}

// In flight, a divergence starts the hysteresis timer but isn't reported yet.
TEST_F(GnssRedundancyChecksTest, InFlightNotYetSustained)
{
	setRequiredGnss(2);
	publishBoth(DIVERGING);
	runCheck(true);
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gnss);
}

// In flight, a sustained divergence with SYS_HAS_NUM_GNSS = 2 → gnss_lost, but COM_GNSSLOSS_ACT = Warning doesn't block
// arming.
TEST_F(GnssRedundancyChecksTest, InFlightSustainedSetsGnssLost)
{
	setRequiredGnss(2);
	runSustained(DIVERGING, 0.f, true);
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gnss);
	EXPECT_TRUE(_can_arm_all_modes);
}

// Without SYS_HAS_NUM_GNSS = 2 a divergence is a warning only.
TEST_F(GnssRedundancyChecksTest, DivergenceWarnsWithoutRequiredReceivers)
{
	setGnssLossAct(1); // Return
	runSustained(DIVERGING, 0.f, true);
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gnss);
}

// In flight, a divergence that recovers before the hysteresis time → no flag.
TEST_F(GnssRedundancyChecksTest, InFlightShortGlitchIgnored)
{
	setRequiredGnss(2);
	publishBoth(DIVERGING);
	runCheck(true);

	publishBoth(AGREEING);
	runCheck(true);

	px4_usleep(HYSTERESIS_WAIT_US);
	publishBoth(AGREEING);
	runCheck(true);
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gnss);
}

// Before arming, a divergence blocks arming at once.
TEST_F(GnssRedundancyChecksTest, BeforeArmingImmediate)
{
	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming
	publishBoth(DIVERGING);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gnss);
	EXPECT_FALSE(_can_arm_all_modes);
}

// A sustained divergence with COM_GNSSLOSS_ACT = Return → gnss_lost and arming blocked.
TEST_F(GnssRedundancyChecksTest, ReturnBlocksArming)
{
	setRequiredGnss(2);
	setGnssLossAct(1); // Return
	runSustained(DIVERGING);
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gnss);
	EXPECT_FALSE(_can_arm_all_modes);
}

// Before arming, a divergence clears only after the receivers agree continuously for the hysteresis time, and a new
// disagreement restarts the wait.
TEST_F(GnssRedundancyChecksTest, BeforeArmingClearsAfterContinuousAgreement)
{
	setRequiredGnss(2);
	publishBoth(DIVERGING);
	runCheck();
	ASSERT_TRUE(_failsafe_flags.gnss_lost);

	publishBoth(AGREEING);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);

	px4_usleep(1500000);
	publishBoth(DIVERGING);
	runCheck();
	publishBoth(AGREEING);
	runCheck();

	px4_usleep(1500000);
	publishBoth(AGREEING);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);

	px4_usleep(700000);
	publishBoth(AGREEING);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gnss);
}

// A vertical disagreement above COM_GNSS_V_MAX diverges, even when the receivers agree horizontally.
TEST_F(GnssRedundancyChecksTest, VerticalDivergence)
{
	setRequiredGnss(2);
	runSustained(0.f, DIVERGING);
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gnss);
}

// Receivers that report a poor accuracy get a wider limit: 3 times their combined accuracy.
TEST_F(GnssRedundancyChecksTest, ReportedAccuracyWidensTheLimits)
{
	setRequiredGnss(2);

	// 1.5 m accuracy each: about 6.4 m
	runSustained(DIVERGING, DIVERGING, false, 1.5f);
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gnss);

	runSustained(7.f, 0.f, false, 1.5f);
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
}

// The limits are configurable.
TEST_F(GnssRedundancyChecksTest, LimitsConfigurable)
{
	setRequiredGnss(2);
	setFloatParam("COM_GNSS_H_MAX", 1.f);
	runSustained(WITHIN_LIMIT);
	EXPECT_TRUE(_failsafe_flags.gnss_lost);

	setFloatParam("COM_GNSS_H_MAX", 6.f);
	setFloatParam("COM_GNSS_V_MAX", 1.f);
	runSustained(DIVERGING, 0.5f);
	EXPECT_FALSE(_failsafe_flags.gnss_lost);

	runSustained(0.f, WITHIN_LIMIT);
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
}

// Receivers the sensors module can't compare clear a pre-arm divergence at once.
TEST_F(GnssRedundancyChecksTest, UnknownDisagreementNotCompared)
{
	setRequiredGnss(2);
	publishBoth(DIVERGING);
	runCheck();
	ASSERT_TRUE(_failsafe_flags.gnss_lost);

	publishBoth(NAN, NAN);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gnss);
}

// Without a selected receiver among the healthy ones there is nothing to compare with.
TEST_F(GnssRedundancyChecksTest, NoSelectedReceiverNotCompared)
{
	setRequiredGnss(2);
	_status.device_id_selected = 0;
	runSustained(DIVERGING);
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gnss);
}

// A receiver that fails its checks is neither counted nor compared.
TEST_F(GnssRedundancyChecksTest, UnhealthyReceiverNotCompared)
{
	_status.inconsistency_horizontal[1] = DIVERGING;
	publishGnss(0);
	publishGnss(1, RTK_ACCURACY, RTK_ACCURACY, 2);
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);
	EXPECT_FALSE(_health_warning_gnss);
}

// SYS_HAS_NUM_GNSS = 2, COM_GNSSLOSS_ACT = Return, only one receiver healthy → gnss_lost and arming blocked.
TEST_F(GnssRedundancyChecksTest, BelowRequiredSetsGnssLost)
{
	setRequiredGnss(2);
	setGnssLossAct(1); // Return

	publishGnss(0);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gnss);
	EXPECT_FALSE(_can_arm_all_modes);
}

// COM_GNSSLOSS_ACT = "Warning, block arming" blocks arming without any in-flight action (gnss_lost is still set for the warning).
TEST_F(GnssRedundancyChecksTest, BelowRequiredBlockArmingBlocksArming)
{
	setRequiredGnss(2);
	setGnssLossAct(4); // Warning, block arming

	publishGnss(0);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gnss);
	EXPECT_FALSE(_can_arm_all_modes);
}

// COM_GNSSLOSS_ACT = Warning reports the issue but never blocks arming.
TEST_F(GnssRedundancyChecksTest, BelowRequiredWarningDoesNotBlockArming)
{
	setRequiredGnss(2);
	setGnssLossAct(0); // Warning

	publishGnss(0);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gnss);
	EXPECT_TRUE(_can_arm_all_modes);
}

// After seeing two healthy receivers, losing one emits a health warning
// even when SYS_HAS_NUM_GNSS is not set (dropped_below_peak path).
TEST_F(GnssRedundancyChecksTest, DroppedBelowPeakSetsHealthWarning)
{
	publishBoth(AGREEING);
	runCheck();
	EXPECT_FALSE(_health_warning_gnss); // both present, no warning

	// GNSS1 disappears.
	sensor_gnss_s gone{};
	_gnss1_pub.publish(gone); // device_id = 0 → treated as absent
	runCheck();
	EXPECT_TRUE(_health_warning_gnss);
	EXPECT_FALSE(_failsafe_flags.gnss_lost); // no failsafe without COM_GNSSLOSS_ACT + below_required
}

// A receiver counts only while the status names it and says it passes its checks.
TEST_F(GnssRedundancyChecksTest, StatusOfAnotherReceiverDoesNotCount)
{
	setRequiredGnss(1);

	publishGnss(0);
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

	publishGnss(0);
	runCheck();
	ASSERT_FALSE(_failsafe_flags.gnss_lost);

	_status.timestamp = hrt_absolute_time() - 2000000; // 2 s
	_status_pub.publish(_status);

	sensor_gnss_s gnss{};
	gnss.timestamp = hrt_absolute_time();
	gnss.device_id = 1;
	gnss.fix_type = 6;
	_gnss0_pub.publish(gnss);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
}

// A configured primary receiver that is not publishing is a warning before arming, not a reason to refuse it.
TEST_F(GnssRedundancyChecksTest, PrimaryOfflineWarnsBeforeArming)
{
	publishGnss(1);
	runCheck();
	ASSERT_FALSE(_arming_warning_gnss);
	const NavModes can_arm = _can_arm;

	_status.primary_offline = true;
	publishGnss(1);
	runCheck();
	EXPECT_TRUE(_arming_warning_gnss);
	EXPECT_FALSE(_arming_error_gnss);
	EXPECT_EQ(_can_arm, can_arm);
	EXPECT_FALSE(_failsafe_flags.gnss_lost);

	// in flight the switch away from it is reported instead
	runCheck(true);
	EXPECT_FALSE(_arming_warning_gnss);
}
