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

#include "checks/batteryCheck.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/param.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/battery_status.h>

// to run: make tests TESTFILTER=batteryChecks

/* EVENT
 * @skip-file
 */

class BatteryChecksTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);

		// Reset params the check reads so state doesn't leak between tests.
		param_reset(param_find("COM_ARM_BAT_VDIF")); // delta check disabled (0) by default
		param_reset(param_find("COM_ARM_BAT_MIN"));  // -1: low-battery arming path stays off
		param_reset(param_find("CBRK_SUPPLY_CHK"));  // 0: supply check (and this check) runs

		// Claim uORB instances 0,1,2 before the check subscribes on first copy().
		battery_status_s empty{};
		_bat0_pub.publish(empty);
		_bat1_pub.publish(empty);
		_bat2_pub.publish(empty);
	}

	// Healthy, connected battery at a given voltage. Defaults keep every other
	// battery check quiet so only the voltage-delta path can set a battery error.
	battery_status_s makeBattery(float voltage_v)
	{
		battery_status_s battery{};
		battery.timestamp   = hrt_absolute_time();
		battery.connected   = true;
		battery.voltage_v   = voltage_v;
		battery.remaining   = 1.f;
		battery.warning     = battery_status_s::WARNING_NONE;
		battery.faults      = 0;
		battery.is_required = false;
		return battery;
	}

	void setMaxDelta(float volts) { param_set(param_find("COM_ARM_BAT_VDIF"), &volts); }

	// Run the check and capture whether the battery component failed an arming check.
	void runCheck(bool armed = false)
	{
		vehicle_status_s status{};

		if (armed) { status.arming_state = vehicle_status_s::ARMING_STATE_ARMED; }

		_check.updateParams();
		Context context{status};
		_failsafe_flags = {};
		Report reporter{_failsafe_flags, 0};
		_check.checkAndReport(context, reporter);
		_arming_error_battery =
			(reporter.armingCheckResults().error | reporter.armingCheckResults().warning) & health_component_t::battery;
	}

	uORB::PublicationMulti<battery_status_s> _bat0_pub{ORB_ID(battery_status)};
	uORB::PublicationMulti<battery_status_s> _bat1_pub{ORB_ID(battery_status)};
	uORB::PublicationMulti<battery_status_s> _bat2_pub{ORB_ID(battery_status)};
	failsafe_flags_s _failsafe_flags{};
	bool             _arming_error_battery{false};
	BatteryChecks    _check;
};

// Delta check disabled by default (COM_ARM_BAT_VDIF = 0): it never fails on voltage delta.
TEST_F(BatteryChecksTest, Disabled_NoFailureEvenWhenDiverging)
{
	_bat0_pub.publish(makeBattery(16.0f));
	_bat1_pub.publish(makeBattery(20.0f));
	runCheck();
	EXPECT_FALSE(_arming_error_battery);
}

// A single connected battery never trips the delta check (needs >= 2).
TEST_F(BatteryChecksTest, SingleBattery_NoFailure)
{
	setMaxDelta(0.5f);
	_bat0_pub.publish(makeBattery(16.0f));
	runCheck();
	EXPECT_FALSE(_arming_error_battery);
}

// Two batteries within the allowed delta: no failure.
TEST_F(BatteryChecksTest, TwoBatteries_WithinDelta_NoFailure)
{
	setMaxDelta(0.5f);
	_bat0_pub.publish(makeBattery(16.0f));
	_bat1_pub.publish(makeBattery(16.3f));
	runCheck();
	EXPECT_FALSE(_arming_error_battery);
}

// Two batteries exceeding the allowed delta: arming failure on the battery component.
TEST_F(BatteryChecksTest, TwoBatteries_ExceedsDelta_Failure)
{
	setMaxDelta(0.5f);
	_bat0_pub.publish(makeBattery(16.0f));
	_bat1_pub.publish(makeBattery(17.0f));
	runCheck();
	EXPECT_TRUE(_arming_error_battery);
}

// Three batteries all within the allowed spread: no failure.
TEST_F(BatteryChecksTest, ThreeBatteries_WithinDelta_NoFailure)
{
	setMaxDelta(0.5f);
	_bat0_pub.publish(makeBattery(16.0f));
	_bat1_pub.publish(makeBattery(16.2f));
	_bat2_pub.publish(makeBattery(16.4f));
	runCheck();
	EXPECT_FALSE(_arming_error_battery);
}

// Three batteries where a single outlier pushes max-min past the limit: failure.
// (delta is taken across all connected batteries, not just adjacent pairs.)
TEST_F(BatteryChecksTest, ThreeBatteries_OneOutlier_Failure)
{
	setMaxDelta(0.5f);
	_bat0_pub.publish(makeBattery(16.0f));
	_bat1_pub.publish(makeBattery(16.2f));
	_bat2_pub.publish(makeBattery(17.0f));
	runCheck();
	EXPECT_TRUE(_arming_error_battery);
}

// Three batteries diverging beyond the limit, but armed: no failure (gated on disarmed).
TEST_F(BatteryChecksTest, ThreeBatteries_ExceedsDelta_ArmedSkipped)
{
	setMaxDelta(0.5f);
	_bat0_pub.publish(makeBattery(16.0f));
	_bat1_pub.publish(makeBattery(16.2f));
	_bat2_pub.publish(makeBattery(17.0f));
	runCheck(/*armed=*/true);
	EXPECT_FALSE(_arming_error_battery);
}
