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

#include <algorithm>
#include <vector>

#include "checks/batteryCheck.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/param.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/event.h>

// to run: make tests TESTFILTER=batteryChecks

/* EVENT
 * @skip-file
 */

class BatteryChecksTest : public ::testing::Test
{
public:
	static constexpr int kBatteryCount = 2;

	// advertised once for the whole suite so they keep instances 0 and 1
	static void SetUpTestSuite()
	{
		// ensure the topic exists, otherwise the first reported events are lost
		orb_advertise(ORB_ID(event), nullptr);

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

	static void publish(int index, uint8_t warning, bool connected = true, float remaining = 0.5f, uint8_t cell_count = 0,
			    uint8_t source = battery_status_s::SOURCE_POWER_MODULE, uint8_t id = 1)
	{
		battery_status_s battery{};
		battery.timestamp = hrt_absolute_time();
		battery.connected = connected;
		battery.remaining = remaining;
		battery.cell_count = cell_count;
		battery.source = source;
		battery.id = id;
		battery.time_remaining_s = NAN;
		battery.warning = warning;
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

		// publish the reported events and keep the battery checks' own ones
		event_s event;

		while (_event_sub.update(&event)) {}

		reporter.finalize();
		reporter.report(true);
		_events.clear();

		while (_event_sub.update(&event)) {
			_events.push_back(event);
		}
	}

	// the battery number argument follows the modes (uint32) and the health component (uint8)
	bool reported(uint32_t event_id, int battery = -1) const
	{
		return std::any_of(_events.begin(), _events.end(), [&](const event_s & event) {
			return (event.id == event_id) && ((battery < 0) || (event.arguments[5] == battery));
		});
	}

	static uORB::PublicationMulti<battery_status_s> _battery_pubs[kBatteryCount];
	failsafe_flags_s _failsafe_flags{};
	bool _can_arm{false};
	BatteryChecks _check;
	uORB::Subscription _event_sub{ORB_ID(event)};
	std::vector<event_s> _events;
};

uORB::PublicationMulti<battery_status_s> BatteryChecksTest::_battery_pubs[kBatteryCount] {
	{ORB_ID(battery_status)}, {ORB_ID(battery_status)}
};

// a charging pack must not hide another pack's emergency
TEST_F(BatteryChecksTest, ChargingDoesNotOutrankAnEmergency)
{
	publish(0, battery_status_s::WARNING_CHARGING);
	publish(1, battery_status_s::WARNING_EMERGENCY);
	runCheck(true);
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_EMERGENCY);
}

// charging when the vehicle arms, on a dock for example, must not latch and block a later low battery
TEST_F(BatteryChecksTest, ChargingAtArmingDoesNotLatchOverALaterWarning)
{
	publish(0, battery_status_s::WARNING_CHARGING);
	runCheck(false);
	runCheck(true);
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_NONE);

	publish(0, battery_status_s::WARNING_LOW);
	runCheck(true);
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_LOW);

	publish(0, battery_status_s::WARNING_CRITICAL);
	runCheck(true);
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_CRITICAL);
}

// a charging pack on its own is no warning and doesn't stop arming
TEST_F(BatteryChecksTest, ChargingAloneIsNoWarning)
{
	publish(0, battery_status_s::WARNING_CHARGING);
	runCheck(false);
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_NONE);
	EXPECT_TRUE(_can_arm);
}

// a pack that disconnects in flight raises a critical warning over a lower one, and it must not
// hold back another pack's later emergency
TEST_F(BatteryChecksTest, DisconnectedPackIsCriticalAndDoesNotHoldBackALaterEmergency)
{
	runCheck(false);
	runCheck(true);

	publish(0, battery_status_s::WARNING_LOW);
	publish(1, battery_status_s::WARNING_NONE, false);
	runCheck(true);
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_CRITICAL);

	publish(0, battery_status_s::WARNING_EMERGENCY);
	runCheck(true);
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_EMERGENCY);
}

// a pack that disconnects in flight must not hide another pack's emergency
TEST_F(BatteryChecksTest, DisconnectedPackDoesNotHideAnotherPacksEmergency)
{
	runCheck(false);
	runCheck(true);

	publish(0, battery_status_s::WARNING_EMERGENCY);
	publish(1, battery_status_s::WARNING_NONE, false);
	runCheck(true);
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_EMERGENCY);
}

// a battery without a configured cell count has an unknown state of charge, which still counts as an emergency,
// and arming names the parameter of the BATn index rather than the topic instance
TEST_F(BatteryChecksTest, UnknownChargeWithoutCellCountNamesTheParameter)
{
	publish(0, battery_status_s::WARNING_EMERGENCY, true, -1.f, 0, battery_status_s::SOURCE_POWER_MODULE, 2);
	runCheck(false);
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_EMERGENCY);
	EXPECT_FALSE(_can_arm);
	EXPECT_TRUE(reported(events::ID("check_battery_cell_count_unknown"), 2));
	EXPECT_FALSE(reported(events::ID("check_battery_charge_unknown")));

	publish(0, battery_status_s::WARNING_NONE, true, 0.9f, 3, battery_status_s::SOURCE_POWER_MODULE, 2);
	runCheck(false);
	EXPECT_TRUE(_can_arm);
	EXPECT_FALSE(reported(events::ID("check_battery_cell_count_unknown")));
}

// an external battery without a charge blocks arming without a warning, so a single unknown value in flight
// doesn't latch a failsafe, and a cell count doesn't make the charge known
TEST_F(BatteryChecksTest, ExternalBatteryWithoutChargeBlocksArming)
{
	publish(1, battery_status_s::WARNING_NONE, true, -1.f, 1, battery_status_s::SOURCE_EXTERNAL, 0);
	runCheck(false);
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_NONE);
	EXPECT_FALSE(_can_arm);
	EXPECT_TRUE(reported(events::ID("check_battery_charge_unknown"), 2));
	EXPECT_FALSE(reported(events::ID("check_battery_cell_count_unknown")));
}

// an unknown state of charge still counts as below the configured arming threshold
TEST_F(BatteryChecksTest, UnknownChargeIsBelowArmThreshold)
{
	float arm_bat_min = 0.2f;
	param_set(param_find("COM_ARM_BAT_MIN"), &arm_bat_min);

	publish(0, battery_status_s::WARNING_NONE, true, -1.f, 4, battery_status_s::SOURCE_EXTERNAL);
	runCheck(false);
	EXPECT_FALSE(_can_arm);
	EXPECT_TRUE(reported(events::ID("check_battery_low")));

	param_reset(param_find("COM_ARM_BAT_MIN"));
}

// the unknown charge failure only matters for arming, so it isn't reported in flight
TEST_F(BatteryChecksTest, UnknownChargeIsNotReportedWhileArmed)
{
	runCheck(false);
	runCheck(true);

	publish(1, battery_status_s::WARNING_NONE, true, -1.f, 1, battery_status_s::SOURCE_EXTERNAL, 0);
	runCheck(true);
	EXPECT_FALSE(reported(events::ID("check_battery_charge_unknown")));
	EXPECT_EQ(_failsafe_flags.battery_warning, battery_status_s::WARNING_NONE);
}
