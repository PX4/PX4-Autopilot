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
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/esc_status.h>

using namespace time_literals;

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

		const int32_t one = 1;
		param_set_no_notification(param_find("COM_ARM_CHK_ESCS"), &one);
		param_set_no_notification(param_find("FD_ACT_EN"), &one);

		const float trigger_time = 0.2f; // keeps the motor failure tests short
		param_set_no_notification(param_find("MOTFAIL_UND_TIME"), &trigger_time);
	}

	void TearDown() override
	{
		param_reset(param_find("COM_ARM_CHK_ESCS"));
		param_reset(param_find("FD_ACT_EN"));
		param_reset(param_find("MOTFAIL_UND_TIME"));
	}

	// Publish a quad ESC status with every ESC fresh except `stale_index` whose last report is `age` old
	void publishWithStaleEsc(int stale_index, hrt_abstime age, uint8_t armed_flags = 0b1111)
	{
		esc_status_s esc_status{};
		esc_status.timestamp = hrt_absolute_time();
		esc_status.esc_count = 4;
		esc_status.esc_online_flags = 0b1111; // driver-side flags stay online, only the timestamp ages
		esc_status.esc_armed_flags = armed_flags;

		for (int i = 0; i < 4; i++) {
			esc_status.esc[i].timestamp = (i == stale_index) ? esc_status.timestamp - age : esc_status.timestamp;
			esc_status.esc[i].actuator_function = esc_report_s::ACTUATOR_FUNCTION_MOTOR1 + i;
		}

		_esc_status_pub.publish(esc_status);
	}

	// Fly armed for `duration` of simulated time: the motor failure detector runs on the ESC sample
	// timestamps, so they advance in kTick steps without waiting. All motors are commanded kCommand,
	// each ESC reports the current its motor draws at the default model (MOTFAIL_C2T * kCommand),
	// except ESC `failed_index` reporting 0 A. `functions` maps the ESC slots to motors,
	// `reversible_flags` is published per motor.
	void fly(hrt_abstime duration, int failed_index, uint16_t reversible_flags = 0,
		 const uint8_t (&functions)[4] = kQuadFunctions)
	{
		if (_esc_time == 0) { _esc_time = hrt_absolute_time(); }

		for (hrt_abstime t = 0; t < duration; t += kTick) {
			_esc_time += kTick;

			actuator_motors_s actuator_motors{};
			actuator_motors.timestamp = _esc_time;
			actuator_motors.reversible_flags = reversible_flags;

			for (int i = 0; i < 4; i++) { actuator_motors.control[i] = kCommand; }

			_actuator_motors_pub.publish(actuator_motors);

			esc_status_s esc_status{};
			esc_status.timestamp = _esc_time;
			esc_status.esc_count = 4;
			esc_status.esc_online_flags = 0b1111;
			esc_status.esc_armed_flags = 0b1111;

			for (int i = 0; i < 4; i++) {
				esc_status.esc[i].timestamp = esc_status.timestamp - i; // each ESC reports at its own time
				esc_status.esc[i].actuator_function = functions[i];
				esc_status.esc[i].esc_current = (i == failed_index) ? 0.f : kHealthyCurrent;
			}

			_esc_status_pub.publish(esc_status);
			runCheck(true);
		}
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
	}

	static constexpr hrt_abstime kTick = 50_ms;
	static constexpr float kCommand = 0.5f;
	static constexpr float kHealthyCurrent = 35.f * kCommand; // MOTFAIL_C2T default, MOTFAIL_IDLE 0
	static constexpr uint8_t kMotor1 = esc_report_s::ACTUATOR_FUNCTION_MOTOR1;
	static constexpr uint8_t kQuadFunctions[4] {kMotor1, kMotor1 + 1, kMotor1 + 2, kMotor1 + 3};

	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	hrt_abstime _esc_time{0}; // simulated ESC sample time, starts at the real time of the first fly()
	failsafe_flags_s _failsafe_flags{};
	bool _can_arm{true};
	bool _health_error_escs{false};
	EscChecks _check;
};

// An ESC report older than ESC_OFFLINE_TIMEOUT_US marks the ESC offline, blocks arming, and sets the motor failure mask
TEST_F(EscChecksTest, EscOfflineAfterOfflineTimeout)
{
	publishWithStaleEsc(2, EscChecks::ESC_OFFLINE_TIMEOUT_US + 50_ms);
	runCheck();
	EXPECT_FALSE(_can_arm);
	EXPECT_TRUE(_health_error_escs);
	EXPECT_EQ(_check.getMotorFailureMask(), 0b0100u);
	EXPECT_TRUE(_failsafe_flags.fd_motor_failure);
}

TEST_F(EscChecksTest, EscOnlineInsideOfflineTimeout)
{
	publishWithStaleEsc(2, EscChecks::ESC_OFFLINE_TIMEOUT_US - 50_ms);
	runCheck();
	EXPECT_TRUE(_can_arm);
	EXPECT_FALSE(_health_error_escs);
	EXPECT_EQ(_check.getMotorFailureMask(), 0u);
	EXPECT_FALSE(_failsafe_flags.fd_motor_failure);
}

// Right after arming the ESCs have not reported armed yet, which must not fail before the arming timeout
TEST_F(EscChecksTest, EscsNotYetArmedTolerated)
{
	static_assert(EscChecks::ESC_ARMING_TIMEOUT_US > EscChecks::ESC_OFFLINE_TIMEOUT_US, "arming timeout must outlast the offline timeout");
	publishWithStaleEsc(-1, 0, 0b0000); // all fresh, none armed yet
	runCheck(true);
	px4_usleep(EscChecks::ESC_ARMING_TIMEOUT_US - 50_ms);
	publishWithStaleEsc(-1, 0, 0b0000);
	runCheck(true);
	EXPECT_FALSE(_failsafe_flags.fd_esc_arming_failure);
	EXPECT_FALSE(_check.getEscArmStatus());
	EXPECT_EQ(_check.getMotorFailureMask(), 0u);
}

// ESCs still not reporting armed once the arming timeout passed triggers the ESC arming failure
TEST_F(EscChecksTest, EscsNotArmedFailAfterArmingTimeout)
{
	publishWithStaleEsc(-1, 0, 0b0000); // all fresh, none armed yet
	runCheck(true);
	px4_usleep(EscChecks::ESC_ARMING_TIMEOUT_US + 50_ms);
	publishWithStaleEsc(-1, 0, 0b1011); // ESC 3 still not armed
	runCheck(true);
	EXPECT_TRUE(_failsafe_flags.fd_esc_arming_failure);
	EXPECT_TRUE(_check.getEscArmStatus());
	EXPECT_TRUE(_health_error_escs);
}

// A stale ESC is ignored when COM_ARM_CHK_ESCS is disabled
TEST_F(EscChecksTest, EscOfflineIgnoredWithChecksDisabled)
{
	const int32_t zero = 0;
	param_set_no_notification(param_find("COM_ARM_CHK_ESCS"), &zero);
	publishWithStaleEsc(2, EscChecks::ESC_OFFLINE_TIMEOUT_US + 50_ms);
	runCheck();
	EXPECT_TRUE(_can_arm);
	EXPECT_FALSE(_health_error_escs);
	EXPECT_EQ(_check.getMotorFailureMask(), 0u);
	EXPECT_FALSE(_failsafe_flags.fd_motor_failure);
}

// A motor whose current collapses while commanded is flagged once it stays under the band for the
// trigger time; the failure is latched until disarm.
TEST_F(EscChecksTest, MotorUndercurrentLatchesUntilDisarm)
{
	fly(200_ms, -1); // healthy: every ESC reports current, so every ESC gets judged
	ASSERT_EQ(_check.getMotorFailureMask(), 0u);

	fly(600_ms, 2);
	EXPECT_EQ(_check.getMotorFailureMask(), 0b0100u);
	EXPECT_TRUE(_failsafe_flags.fd_motor_failure);
	EXPECT_TRUE(_health_error_escs);

	fly(300_ms, -1); // current back: still latched
	EXPECT_EQ(_check.getMotorFailureMask(), 0b0100u);

	publishWithStaleEsc(-1, 0);
	runCheck(false); // disarm resets the detector
	EXPECT_EQ(_check.getMotorFailureMask(), 0u);
	EXPECT_FALSE(_failsafe_flags.fd_motor_failure);

	fly(200_ms, -1); // the next flight starts without the old failure
	EXPECT_EQ(_check.getMotorFailureMask(), 0u);
	EXPECT_FALSE(_failsafe_flags.fd_motor_failure);
}

// The current model only describes forward thrust: a reversible motor is not judged.
TEST_F(EscChecksTest, ReversibleMotorNotJudged)
{
	fly(200_ms, -1);
	fly(600_ms, 2, 0b0100);
	EXPECT_EQ(_check.getMotorFailureMask(), 0u);
	EXPECT_FALSE(_failsafe_flags.fd_motor_failure);
}

// Every ESC is judged on its own, also when two ESCs share a motor function.
TEST_F(EscChecksTest, EscSharingMotorFunctionJudgedSeparately)
{
	const uint8_t functions[4] {kMotor1, kMotor1 + 1, kMotor1 + 2, kMotor1 + 2}; // ESCs 3 and 4 both drive motor 3
	fly(200_ms, -1, 0, functions);
	fly(600_ms, 2, 0, functions); // the first of the two fails, the second stays healthy
	EXPECT_EQ(_check.getMotorFailureMask(), 0b0100u);
	EXPECT_TRUE(_failsafe_flags.fd_motor_failure);
}
