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

#include "checks/escCheck.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/param.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/esc_status.h>

// to run: make tests TESTFILTER=escChecks

/* EVENT
 * @skip-file
 */

class EscChecksTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);

		// Make sure the other ESC checks stay disabled so only the error count check is exercised.
		param_reset(param_find("COM_ARM_CHK_ESCS"));
		param_reset(param_find("FD_ACT_EN"));
	}

	// Publish a quad ESC status where every ESC is mapped, online and reports `error_counts[i]` errors of `type`.
	void publishEscStatus(const uint32_t error_counts[4], uint8_t type = esc_report_s::ERRORCOUNT_TYPE_CAN_TEC)
	{
		esc_status_s esc_status{};
		esc_status.timestamp = hrt_absolute_time();
		esc_status.esc_count = 4;
		esc_status.esc_online_flags = 0b1111;
		esc_status.esc_armed_flags = 0b1111;

		for (int i = 0; i < 4; i++) {
			esc_status.esc[i].timestamp = esc_status.timestamp;
			esc_status.esc[i].actuator_function = esc_report_s::ACTUATOR_FUNCTION_MOTOR1 + i;
			esc_status.esc[i].esc_errorcount = error_counts[i];
			esc_status.esc[i].esc_errorcount_type = type;
		}

		_esc_status_pub.publish(esc_status);
	}

	void runCheck(bool armed = false)
	{
		vehicle_status_s status{};

		if (armed) { status.arming_state = vehicle_status_s::ARMING_STATE_ARMED; }

		_check.updateParams();
		Context context{status};
		_failsafe_flags = {};
		Report reporter{_failsafe_flags, 0};
		_check.checkAndReport(context, reporter);

		_can_arm = reporter.armingCheckResults().can_arm == NavModes::All;
		_health_error_escs = reporter.healthResults().error & health_component_t::motors_escs;
		_health_warning_escs = reporter.healthResults().warning & health_component_t::motors_escs;
	}

	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};
	failsafe_flags_s _failsafe_flags{};
	bool _can_arm{true};
	bool _health_error_escs{false};
	bool _health_warning_escs{false};
	EscChecks _check;
};

static constexpr uint32_t TH_CAN = EscChecks::ESC_CAN_ERROR_COUNTER_THRESHOLD;

// Live CAN error counters at or below the threshold are fine.
TEST_F(EscChecksTest, LiveCanCountersAtThresholdCanArm)
{
	const uint8_t live_types[3] = {esc_report_s::ERRORCOUNT_TYPE_CAN_TEC, esc_report_s::ERRORCOUNT_TYPE_CAN_REC,
				       esc_report_s::ERRORCOUNT_TYPE_CAN_TEC_REC_MAX
				      };

	for (uint8_t type : live_types) {
		const uint32_t counts[4] = {0, TH_CAN / 2, TH_CAN, TH_CAN};
		publishEscStatus(counts, type);
		runCheck();
		EXPECT_TRUE(_can_arm) << "type " << type;
		EXPECT_FALSE(_health_error_escs) << "type " << type;
		EXPECT_FALSE(_health_warning_escs) << "type " << type;
	}
}

// A single ESC above the threshold blocks arming while disarmed.
TEST_F(EscChecksTest, LiveCanCounterAboveThresholdBlocksArming)
{
	const uint32_t counts[4] = {0, TH_CAN + 1, 0, 0};
	publishEscStatus(counts, esc_report_s::ERRORCOUNT_TYPE_CAN_TEC);
	runCheck(false);
	EXPECT_FALSE(_can_arm);
	EXPECT_TRUE(_health_error_escs);
}

// The same condition while armed is only a warning and does not clear any arming bits.
TEST_F(EscChecksTest, LiveCanCounterAboveThresholdWarnsInFlight)
{
	const uint32_t counts[4] = {0, TH_CAN + 1, 0, 0};
	publishEscStatus(counts, esc_report_s::ERRORCOUNT_TYPE_CAN_REC);
	runCheck(true);
	EXPECT_TRUE(_can_arm);
	EXPECT_FALSE(_health_error_escs);
	EXPECT_TRUE(_health_warning_escs);
	EXPECT_FALSE(_failsafe_flags.fd_motor_failure);
}

// Packed live CAN counters (TX in the upper, RX in the lower 16 bit): each half is checked on its own.
TEST_F(EscChecksTest, PackedCanCountersEvaluateBothHalves)
{
	const uint32_t both_at_limit = (TH_CAN << 16) | TH_CAN;
	const uint32_t rx_above = TH_CAN + 1;
	const uint32_t tx_above = (TH_CAN + 1) << 16;

	{
		const uint32_t counts[4] = {both_at_limit, 0, 0, 0};
		publishEscStatus(counts, esc_report_s::ERRORCOUNT_TYPE_CAN_TEC_REC_PACKED);
		runCheck();
		EXPECT_TRUE(_can_arm);
		EXPECT_FALSE(_health_error_escs);
	}

	{
		const uint32_t counts[4] = {rx_above, 0, 0, 0};
		publishEscStatus(counts, esc_report_s::ERRORCOUNT_TYPE_CAN_TEC_REC_PACKED);
		runCheck();
		EXPECT_FALSE(_can_arm);
		EXPECT_TRUE(_health_error_escs);
	}

	{
		const uint32_t counts[4] = {tx_above, 0, 0, 0};
		publishEscStatus(counts, esc_report_s::ERRORCOUNT_TYPE_CAN_TEC_REC_PACKED);
		runCheck();
		EXPECT_FALSE(_can_arm);
		EXPECT_TRUE(_health_error_escs);
	}
}

// Cumulative counts and unknown types are informational only: never acted upon, however large.
TEST_F(EscChecksTest, CumulativeAndUnknownTypesIgnored)
{
	const uint8_t ignored_types[6] = {esc_report_s::ERRORCOUNT_TYPE_UNKNOWN, esc_report_s::ERRORCOUNT_TYPE_ESC_FAULTS,
					  esc_report_s::ERRORCOUNT_TYPE_TELEMETRY_ERRORS, esc_report_s::ERRORCOUNT_TYPE_CAN_ERRORS,
					  esc_report_s::ERRORCOUNT_TYPE_SERIAL_TELEMETRY_ERRORS, esc_report_s::ERRORCOUNT_TYPE_BDSHOT_TELEMETRY_ERRORS
					 };

	for (uint8_t type : ignored_types) {
		const uint32_t counts[4] = {TH_CAN * 100, TH_CAN * 100, 0, 0};
		publishEscStatus(counts, type);
		runCheck();
		EXPECT_TRUE(_can_arm) << "type " << type;
		EXPECT_FALSE(_health_error_escs) << "type " << type;
		EXPECT_FALSE(_health_warning_escs) << "type " << type;
	}
}

// Unmapped ESC entries (no motor actuator function) are ignored.
TEST_F(EscChecksTest, UnmappedEscIgnored)
{
	esc_status_s esc_status{};
	esc_status.timestamp = hrt_absolute_time();
	esc_status.esc_count = 1;
	esc_status.esc_online_flags = 0b1;
	esc_status.esc[0].timestamp = esc_status.timestamp;
	esc_status.esc[0].actuator_function = esc_report_s::ACTUATOR_FUNCTION_MOTOR1;
	esc_status.esc[0].esc_errorcount_type = esc_report_s::ERRORCOUNT_TYPE_CAN_TEC;
	esc_status.esc[1].actuator_function = 0; // unmapped
	esc_status.esc[1].esc_errorcount = TH_CAN * 100;
	esc_status.esc[1].esc_errorcount_type = esc_report_s::ERRORCOUNT_TYPE_CAN_TEC;
	_esc_status_pub.publish(esc_status);
	runCheck();
	EXPECT_TRUE(_can_arm);
	EXPECT_FALSE(_health_error_escs);
}
