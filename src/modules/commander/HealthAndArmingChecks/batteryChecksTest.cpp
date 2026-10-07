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
	static constexpr int kBatteryCount = 2;
	static constexpr uint16_t kCharging = 1 << battery_status_s::FLAG_CHARGING;

	// advertised once for the whole suite so they keep instances 0 and 1
	static void SetUpTestSuite()
	{
		for (int i = 0; i < kBatteryCount; i++) {
			ASSERT_TRUE(_battery_pubs[i].advertise());
			ASSERT_EQ(_battery_pubs[i].get_instance(), i);
		}
	}

	void SetUp() override
	{
		param_control_autosave(false);
		_failsafe_flags = {};
		publish(0, battery_status_s::WARNING_NONE);
		publish(1, battery_status_s::WARNING_NONE);
	}

	static void publish(int index, uint8_t warning, uint16_t flags = 0)
	{
		battery_status_s battery{};
		battery.timestamp = hrt_absolute_time();
		battery.connected = true;
		battery.remaining = 0.5f;
		battery.time_remaining_s = NAN;
		battery.warning = warning;
		battery.flags = flags;
		_battery_pubs[index].publish(battery);
	}

	// the failsafe flags carry over between runs, as they do in commander, which is where the in-flight latch lives
	void runCheck(bool armed)
	{
		vehicle_status_s status{};

		if (armed) { status.arming_state = vehicle_status_s::ARMING_STATE_ARMED; }

		_check.updateParams();
		Context context{status};
		Report reporter{_failsafe_flags, 0};
		_check.checkAndReport(context, reporter);
		_can_arm = reporter.armingCheckResults().can_arm == NavModes::All;
	}

	static uORB::PublicationMulti<battery_status_s> _battery_pubs[kBatteryCount];
	failsafe_flags_s _failsafe_flags{};
	bool _can_arm{false};
	BatteryChecks _check;
};

uORB::PublicationMulti<battery_status_s> BatteryChecksTest::_battery_pubs[kBatteryCount] {
	{ORB_ID(battery_status)}, {ORB_ID(battery_status)}
};

// a charging pack must not hide another pack's emergency
TEST_F(BatteryChecksTest, ChargingDoesNotOutrankAnEmergency)
{
	publish(0, battery_status_s::WARNING_NONE, kCharging);
	publish(1, battery_status_s::WARNING_EMERGENCY);
	runCheck(true);
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_EMERGENCY);
}

// charging when the vehicle arms, on a dock for example, must not latch and block a later low battery
TEST_F(BatteryChecksTest, ChargingAtArmingDoesNotLatchOverALaterWarning)
{
	publish(0, battery_status_s::WARNING_NONE, kCharging);
	runCheck(false);
	runCheck(true);
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_NONE);

	publish(0, battery_status_s::WARNING_LOW, kCharging);
	runCheck(true);
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_LOW);

	publish(0, battery_status_s::WARNING_CRITICAL, kCharging);
	runCheck(true);
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_CRITICAL);
}

// a charging pack on its own is no warning and doesn't stop arming
TEST_F(BatteryChecksTest, ChargingAloneIsNoWarning)
{
	publish(0, battery_status_s::WARNING_NONE, kCharging);
	runCheck(false);
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_NONE);
	EXPECT_TRUE(_can_arm);
}
