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

#include "checks/fixedWingPropulsionCheck.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/param.h>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

using namespace time_literals;

// to run: make tests TESTFILTER=fixedWingPropulsionChecks

/* EVENT
 * @skip-file
 */

// With COM_EF_CURR 5 A, the propulsion fails below 5 A at any throttle above COM_EF_THROT (0.5).
class FixedWingPropulsionChecksTest : public ::testing::Test
{
public:
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
		setParam("COM_EF_CURR", 5.f);
		setParam("COM_EF_TIME", 0.2f);
	}

	void TearDown() override
	{
		_battery_age = 0;
		_battery_period = 0;

		for (hrt_abstime &last_publish : _last_battery_publish) {
			last_publish = 0;
		}

		param_reset(param_find("COM_EF_CURR"));
		param_reset(param_find("COM_EF_TIME"));
		param_reset(param_find("COM_EF_THROT"));
	}

	static void setParam(const char *name, float value)
	{
		param_set_no_notification(param_find(name), &value);
	}

	// Fly for this time with the throttle and the battery currents (negative: not measured), returns the last result
	bool fly(hrt_abstime duration, float throttle, float current0, float current1 = NAN,
		 uint8_t vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING, bool landed = false)
	{
		const hrt_abstime start = hrt_absolute_time();

		do {
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			vehicle_thrust_setpoint.xyz[_thrust_axis] = _thrust_sign * throttle;
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_land_detected_s vehicle_land_detected{};
			vehicle_land_detected.timestamp = hrt_absolute_time();
			vehicle_land_detected.landed = landed;
			_vehicle_land_detected_pub.publish(vehicle_land_detected);

			publishBattery(0, current0);
			publishBattery(1, current1);

			runCheck(vehicle_type);
			px4_usleep(20_ms);
		} while (hrt_elapsed_time(&start) < duration);

		return _failure;
	}

	void runCheck(uint8_t vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING)
	{
		vehicle_status_s status{};
		status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		status.vehicle_type = vehicle_type;

		_check.updateParams();
		Context context{status};
		Report reporter{_failsafe_flags, 0}; // commander keeps the flags between runs
		_check.checkAndReport(context, reporter);

		_failure = _failsafe_flags.fd_fw_motor;
		_can_arm = reporter.armingCheckResults().can_arm == NavModes::All;
	}

	static constexpr int kBatteryCount = 2;
	static uORB::PublicationMulti<battery_status_s> _battery_pubs[kBatteryCount];

	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_land_detected_s> _vehicle_land_detected_pub{ORB_ID(vehicle_land_detected)};
	FixedWingPropulsionChecks _check;
	failsafe_flags_s _failsafe_flags{};
	int _thrust_axis{0}; // tiltrotors and tailsitters push along -z
	float _thrust_sign{1.f};
	static hrt_abstime _battery_age;
	static hrt_abstime _battery_period; // smart batteries report every few seconds
	static hrt_abstime _last_battery_publish[kBatteryCount];
	bool _failure{false};
	bool _can_arm{true};

private:
	// a NAN current is a disconnected battery
	static void publishBattery(int index, float current)
	{
		if (_battery_period > 0 && _last_battery_publish[index] != 0
		    && hrt_elapsed_time(&_last_battery_publish[index]) < _battery_period) {
			return;
		}

		_last_battery_publish[index] = hrt_absolute_time();
		battery_status_s battery_status{};
		battery_status.timestamp = hrt_absolute_time() - _battery_age;
		battery_status.connected = PX4_ISFINITE(current);
		battery_status.current_a = PX4_ISFINITE(current) ? current : -1.f;
		_battery_pubs[index].publish(battery_status);
	}
};

uORB::PublicationMulti<battery_status_s> FixedWingPropulsionChecksTest::_battery_pubs[kBatteryCount] {
	{ORB_ID(battery_status)}, {ORB_ID(battery_status)}
};
hrt_abstime FixedWingPropulsionChecksTest::_battery_age{0};
hrt_abstime FixedWingPropulsionChecksTest::_battery_period{0};
hrt_abstime FixedWingPropulsionChecksTest::_last_battery_publish[kBatteryCount] {};

TEST_F(FixedWingPropulsionChecksTest, LowCurrentAtHighThrottleIsFailure)
{
	EXPECT_FALSE(fly(100_ms, 0.9f, 2.f)); // not before COM_EF_TIME
	EXPECT_TRUE(fly(300_ms, 0.9f, 2.f));
	EXPECT_FALSE(_can_arm);
}

TEST_F(FixedWingPropulsionChecksTest, CurrentAboveThresholdIsNoFailure)
{
	// the threshold does not grow with the throttle
	EXPECT_FALSE(fly(500_ms, 1.f, 6.f));
	EXPECT_TRUE(_can_arm);
}

TEST_F(FixedWingPropulsionChecksTest, CruiseThrottleIsChecked)
{
	// without airspeed sensor the controller may keep the cruise throttle after a failure, and the threshold does not
	// shrink with the throttle: 4 A would pass a threshold of 5 A per unit throttle at 0.6
	EXPECT_TRUE(fly(500_ms, 0.6f, 4.f));
}

TEST_F(FixedWingPropulsionChecksTest, ThrottleBelowThresholdIsNoFailure)
{
	// COM_EF_THROT is 0.5 by default
	EXPECT_FALSE(fly(500_ms, 0.45f, 2.f));

	setParam("COM_EF_THROT", 0.4f);
	EXPECT_TRUE(fly(500_ms, 0.45f, 2.f));
}

TEST_F(FixedWingPropulsionChecksTest, BatteryCurrentsAreSummed)
{
	EXPECT_FALSE(fly(500_ms, 0.9f, 3.f, 3.f));
	EXPECT_TRUE(fly(500_ms, 0.9f, 3.f));
}

TEST_F(FixedWingPropulsionChecksTest, UnmeasuredCurrentIsNoFailure)
{
	EXPECT_FALSE(fly(500_ms, 0.9f, -1.f));

	// a battery without current measurement could feed the propulsion
	EXPECT_FALSE(fly(500_ms, 0.9f, 2.f, -1.f));
}

TEST_F(FixedWingPropulsionChecksTest, SlowBatteryReportsAreUsed)
{
	_battery_period = 1_s;
	EXPECT_TRUE(fly(1500_ms, 0.9f, 2.f));
}

TEST_F(FixedWingPropulsionChecksTest, CurrentFromBeforeTheThrottleRoseIsNotUsed)
{
	// gliding at low throttle and current, the battery reports once
	_battery_period = 5_s;
	EXPECT_FALSE(fly(100_ms, 0.1f, 2.f));

	// the climb raises the throttle, the old sample shows the current of the glide
	EXPECT_FALSE(fly(1_s, 0.9f, 2.f));
}

TEST_F(FixedWingPropulsionChecksTest, CurrentFromAnEarlierGlideIsNotUsed)
{
	EXPECT_FALSE(fly(100_ms, 0.9f, 12.f));

	// gliding at low throttle, the slow battery reports a low current once
	_battery_period = 5_s;
	_last_battery_publish[0] = 0;
	_last_battery_publish[1] = 0;
	EXPECT_FALSE(fly(100_ms, 0.1f, 2.f));

	// the next climb starts a new wait for a current measured at its throttle
	EXPECT_FALSE(fly(1_s, 0.9f, 2.f));
}

TEST_F(FixedWingPropulsionChecksTest, OutdatedCurrentIsNoFailure)
{
	setParam("COM_EF_TIME", 6.f);
	EXPECT_FALSE(fly(100_ms, 0.9f, 2.f));

	// the battery stops reporting: its last sample gets older than 5 s before COM_EF_TIME
	_battery_period = 10_s;
	_last_battery_publish[0] = hrt_absolute_time();
	EXPECT_FALSE(fly(7_s, 0.9f, 2.f));
}

TEST_F(FixedWingPropulsionChecksTest, ThrustAlongZIsChecked)
{
	// tiltrotors and tailsitters in fixed-wing flight
	_thrust_axis = 2;
	_thrust_sign = -1.f;
	EXPECT_TRUE(fly(500_ms, 0.9f, 2.f));
}

TEST_F(FixedWingPropulsionChecksTest, MulticopterFlightIsNoFailure)
{
	// a VTOL in hover or in a transition also reports rotary wing
	EXPECT_FALSE(fly(500_ms, 0.9f, 2.f, NAN, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING));
}

TEST_F(FixedWingPropulsionChecksTest, LandedIsNoFailure)
{
	EXPECT_FALSE(fly(500_ms, 0.9f, 2.f, NAN, vehicle_status_s::VEHICLE_TYPE_FIXED_WING, true));
}

TEST_F(FixedWingPropulsionChecksTest, FailureIsLatchedUntilDisabled)
{
	ASSERT_TRUE(fly(500_ms, 0.9f, 2.f));
	EXPECT_TRUE(fly(300_ms, 0.9f, 12.f));

	// a VTOL quad-chutes into multicopter flight
	EXPECT_TRUE(fly(100_ms, 0.f, 12.f, NAN, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING));

	// disabling the check in flight clears a false positive
	setParam("COM_EF_CURR", 0.f);
	EXPECT_FALSE(fly(100_ms, 0.9f, 12.f));
	setParam("COM_EF_CURR", 5.f);
	EXPECT_FALSE(fly(100_ms, 0.9f, 12.f));
	EXPECT_TRUE(_can_arm);
}

TEST_F(FixedWingPropulsionChecksTest, DisabledByDefault)
{
	param_reset(param_find("COM_EF_CURR"));
	EXPECT_FALSE(fly(500_ms, 0.9f, 2.f));
}
