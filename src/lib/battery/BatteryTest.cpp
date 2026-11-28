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

#include "battery.h"

#include <px4_platform_common/param.h>
#include <uORB/Publication.hpp>

// to run: make tests TESTFILTER=Battery

using namespace time_literals;

class BatteryTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset(param_find("BAT1_N_CELLS"));
		param_reset(param_find("BAT1_CAPACITY"));
		// topics persist across tests
		publishArmed(false);
		publishMotorFailureMask(0);
		publishFlightPhase(flight_phase_estimation_s::FLIGHT_PHASE_UNKNOWN);
	}

	void TearDown() override
	{
		param_reset(param_find("BAT1_N_CELLS"));
		param_reset(param_find("BAT1_CAPACITY"));
	}

	// feed a connected battery for long enough to pass the initialization delay
	static battery_status_s run(Battery &battery, float voltage_v, float current_a)
	{
		hrt_abstime timestamp = 1_s;

		for (int i = 0; i < 8 * 50; i++) {
			battery.setConnected(true);
			battery.updateVoltage(voltage_v);
			battery.updateCurrent(current_a);
			battery.updateBatteryStatus(timestamp);
			timestamp += 20_ms;
		}

		return battery.getBatteryStatus();
	}

	static void publishArmed(bool armed, bool fixed_wing = false)
	{
		vehicle_status_s vehicle_status{};
		vehicle_status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
		vehicle_status.vehicle_type = fixed_wing ? vehicle_status_s::VEHICLE_TYPE_FIXED_WING :
					      vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		vehicle_status.timestamp = hrt_absolute_time();
		uORB::Publication<vehicle_status_s> pub{ORB_ID(vehicle_status)};
		pub.publish(vehicle_status);
	}

	static void publishMotorFailureMask(uint16_t mask)
	{
		failure_detector_status_s failure_detector_status{};
		failure_detector_status.fd_motor = (mask != 0);
		failure_detector_status.motor_failure_mask = mask;
		failure_detector_status.timestamp = hrt_absolute_time();
		uORB::Publication<failure_detector_status_s> pub{ORB_ID(failure_detector_status)};
		pub.publish(failure_detector_status);
	}

	static void publishFlightPhase(uint8_t flight_phase)
	{
		flight_phase_estimation_s flight_phase_estimation{};
		flight_phase_estimation.flight_phase = flight_phase;
		flight_phase_estimation.timestamp = hrt_absolute_time();
		uORB::Publication<flight_phase_estimation_s> pub{ORB_ID(flight_phase_estimation)};
		pub.publish(flight_phase_estimation);
	}

	// feed a current alternating by +-noise_a, updating the current average every sample like the drivers do
	static void fly(Battery &battery, float current_a, hrt_abstime duration, hrt_abstime &timestamp, float noise_a = 0.f)
	{
		const hrt_abstime end = timestamp + duration;
		float noise_sign = 1.f;

		while (timestamp < end) {
			battery.setConnected(true);
			battery.updateVoltage(12.4f);
			battery.updateCurrent(current_a + noise_sign * noise_a);
			noise_sign = -noise_sign;
			battery.updateBatteryStatus(timestamp);
			battery.getBatteryStatus();
			timestamp += 20_ms;
		}
	}
};

// without a cell count the state of charge is unknown, which is treated as empty
TEST_F(BatteryTest, UnknownCellCountIsAnEmergency)
{
	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	const battery_status_s status = run(battery, 12.4f, 1.f);
	EXPECT_EQ(status.cell_count, 0);
	EXPECT_FLOAT_EQ(status.remaining, -1.f);
	EXPECT_EQ(status.warning, battery_status_s::WARNING_EMERGENCY);
}

// a configured capacity can't make up for the unknown starting charge, so it stays reported as unknown
TEST_F(BatteryTest, UnknownCellCountWithCapacityStaysUnknown)
{
	float capacity = 5000.f;
	param_set(param_find("BAT1_CAPACITY"), &capacity);

	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	const battery_status_s status = run(battery, 12.4f, 1.f);
	EXPECT_FLOAT_EQ(status.remaining, -1.f);
	EXPECT_EQ(status.warning, battery_status_s::WARNING_EMERGENCY);
	EXPECT_FALSE(PX4_ISFINITE(status.time_remaining_s));
}

// a NaN voltage sample is skipped and doesn't directly trigger a false positive warning,
TEST_F(BatteryTest, NanVoltageIsSkipped)
{
	int32_t n_cells = 3;
	param_set(param_find("BAT1_N_CELLS"), &n_cells);
	float capacity = 5000.f;
	param_set(param_find("BAT1_CAPACITY"), &capacity);

	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_ESCS);
	const battery_status_s status = run(battery, NAN, 1.f);
	EXPECT_EQ(status.warning, battery_status_s::WARNING_NONE);
}

TEST_F(BatteryTest, KnownCellCountEstimatesCharge)
{
	int32_t n_cells = 3;
	param_set(param_find("BAT1_N_CELLS"), &n_cells);

	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	const battery_status_s status = run(battery, 12.4f, 1.f);
	EXPECT_EQ(status.cell_count, 3);
	EXPECT_GT(status.remaining, 0.5f);
	EXPECT_EQ(status.warning, battery_status_s::WARNING_NONE);

	// an empty battery still raises the emergency warning
	Battery empty_battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	const battery_status_s empty_status = run(empty_battery, 9.f, 1.f);
	EXPECT_EQ(empty_status.warning, battery_status_s::WARNING_EMERGENCY);
}

// a motor failure in flight holds the current average through the spike, then averages the new load over a growing window
TEST_F(BatteryTest, MotorFailureAveragesNewLoad)
{
	int32_t n_cells = 3;
	param_set(param_find("BAT1_N_CELLS"), &n_cells);

	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	hrt_abstime timestamp = 1_s;
	publishArmed(true);
	fly(battery, 10.f, 10_s, timestamp);
	const float average_before = battery.getCurrentAverage();

	publishMotorFailureMask(1);
	// the remaining motors saturate while catching the vehicle
	fly(battery, 60.f, 1900_ms, timestamp);
	EXPECT_NEAR(battery.getCurrentAverage(), average_before, 0.1f);

	fly(battery, 20.f, 2_s, timestamp, 5.f);
	EXPECT_NEAR(battery.getCurrentAverage(), 20.f, 0.5f);
}

// a lower load after a motor failure (e.g. braking) doesn't lower the average below the one before
TEST_F(BatteryTest, MotorFailureKeepsAverageBeforeAsMinimum)
{
	int32_t n_cells = 3;
	param_set(param_find("BAT1_N_CELLS"), &n_cells);

	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	hrt_abstime timestamp = 1_s;
	publishArmed(true);
	fly(battery, 10.f, 10_s, timestamp);
	const float average_before = battery.getCurrentAverage();

	publishMotorFailureMask(1);
	fly(battery, 5.f, 10_s, timestamp);
	EXPECT_NEAR(battery.getCurrentAverage(), average_before, 0.01f);
}

// a failure flag that clears and comes back doesn't restart the hold
TEST_F(BatteryTest, MotorFailureFlappingDoesNotRestart)
{
	int32_t n_cells = 3;
	param_set(param_find("BAT1_N_CELLS"), &n_cells);

	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	hrt_abstime timestamp = 1_s;
	publishArmed(true);
	fly(battery, 10.f, 10_s, timestamp);

	publishMotorFailureMask(1);
	fly(battery, 20.f, 3_s, timestamp);
	publishMotorFailureMask(0);
	fly(battery, 20.f, 100_ms, timestamp);
	publishMotorFailureMask(1);
	fly(battery, 60.f, 1_s, timestamp);
	// still averaging: one second at 60 A after 1.1 s at 20 A
	EXPECT_GT(battery.getCurrentAverage(), 30.f);
}

// a transition to fixed-wing ends the reset, the average restarts from the configured prior
TEST_F(BatteryTest, MotorFailureResetEndsInFixedWing)
{
	int32_t n_cells = 3;
	param_set(param_find("BAT1_N_CELLS"), &n_cells);

	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	hrt_abstime timestamp = 1_s;
	publishArmed(true);
	fly(battery, 30.f, 10_s, timestamp);

	publishMotorFailureMask(1);
	fly(battery, 30.f, 3_s, timestamp);
	publishArmed(true, true);
	fly(battery, 30.f, 3_s, timestamp);
	// BAT_AVRG_CURRENT, not updated outside of level fixed-wing flight
	EXPECT_NEAR(battery.getCurrentAverage(), 15.f, 0.01f);
}

// without a motor failure the average follows a current step slowly
TEST_F(BatteryTest, CurrentAverageFiltersWithoutMotorFailure)
{
	int32_t n_cells = 3;
	param_set(param_find("BAT1_N_CELLS"), &n_cells);

	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	hrt_abstime timestamp = 1_s;
	publishArmed(true);
	fly(battery, 10.f, 10_s, timestamp);
	const float average_before = battery.getCurrentAverage();

	fly(battery, 20.f, 3_s, timestamp);
	EXPECT_GT(battery.getCurrentAverage(), average_before + 0.3f);
	EXPECT_LT(battery.getCurrentAverage(), average_before + 2.f);
}

// a motor fault already present on the ground (e.g. ESC not yet online) is not taken for an in-flight failure
TEST_F(BatteryTest, MotorFailurePresentBeforeArmingIsIgnored)
{
	int32_t n_cells = 3;
	param_set(param_find("BAT1_N_CELLS"), &n_cells);

	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	hrt_abstime timestamp = 1_s;
	fly(battery, 1.f, 1_s, timestamp);
	publishMotorFailureMask(1);
	fly(battery, 1.f, 1_s, timestamp);

	publishArmed(true);
	publishMotorFailureMask(1);
	fly(battery, 20.f, 5_s, timestamp);
	EXPECT_LT(battery.getCurrentAverage(), 17.f);
}

// after a back-transition the hover load is averaged over a growing window, starting from no less than the configured prior
TEST_F(BatteryTest, BackTransitionAveragesHoverLoad)
{
	int32_t n_cells = 3;
	param_set(param_find("BAT1_N_CELLS"), &n_cells);

	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	hrt_abstime timestamp = 1_s;
	publishArmed(true);
	fly(battery, 30.f, 10_s, timestamp);
	publishArmed(true, true);
	publishFlightPhase(flight_phase_estimation_s::FLIGHT_PHASE_LEVEL);
	fly(battery, 5.f, 30_s, timestamp);
	ASSERT_LT(battery.getCurrentAverage(), 12.f);

	publishArmed(true);
	// the back-transition peak is held out, the fixed-wing average is raised to BAT_AVRG_CURRENT
	fly(battery, 60.f, 1900_ms, timestamp);
	EXPECT_NEAR(battery.getCurrentAverage(), 15.f, 0.01f);

	fly(battery, 40.f, 2_s, timestamp, 5.f);
	EXPECT_NEAR(battery.getCurrentAverage(), 40.f, 0.5f);
}

// a low current after a back-transition doesn't lower the average below the configured prior
TEST_F(BatteryTest, BackTransitionKeepsConfiguredPriorAsMinimum)
{
	int32_t n_cells = 3;
	param_set(param_find("BAT1_N_CELLS"), &n_cells);

	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	hrt_abstime timestamp = 1_s;
	publishArmed(true);
	fly(battery, 30.f, 10_s, timestamp);
	publishArmed(true, true);
	fly(battery, 10.f, 5_s, timestamp);

	publishArmed(true);
	fly(battery, 8.f, 10_s, timestamp);
	EXPECT_NEAR(battery.getCurrentAverage(), 15.f, 0.01f);
}

// a fixed-wing average above the configured prior is kept as minimum after a back-transition
TEST_F(BatteryTest, BackTransitionKeepsFixedWingAverageAsMinimum)
{
	int32_t n_cells = 3;
	param_set(param_find("BAT1_N_CELLS"), &n_cells);

	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	hrt_abstime timestamp = 1_s;
	publishArmed(true);
	fly(battery, 30.f, 10_s, timestamp);
	publishArmed(true, true);
	publishFlightPhase(flight_phase_estimation_s::FLIGHT_PHASE_LEVEL);
	fly(battery, 25.f, 30_s, timestamp);
	const float fixed_wing_average = battery.getCurrentAverage();
	ASSERT_GT(fixed_wing_average, 18.f);

	publishArmed(true);
	fly(battery, 8.f, 10_s, timestamp);
	EXPECT_NEAR(battery.getCurrentAverage(), fixed_wing_average, 0.01f);
}

// disarming ends the reset, after re-arming the average filters normally again
TEST_F(BatteryTest, MotorFailureResetEndsOnDisarm)
{
	int32_t n_cells = 3;
	param_set(param_find("BAT1_N_CELLS"), &n_cells);

	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	hrt_abstime timestamp = 1_s;
	publishArmed(true);
	fly(battery, 10.f, 10_s, timestamp);
	const float average_before = battery.getCurrentAverage();

	publishMotorFailureMask(1);
	fly(battery, 10.f, 1_s, timestamp);
	publishArmed(false);
	fly(battery, 0.f, 1_s, timestamp);

	publishArmed(true);
	// filter: about 1.5 A higher, continued reset: averaging 40 A after the hold
	fly(battery, 40.f, 3_s, timestamp);
	EXPECT_LT(battery.getCurrentAverage(), average_before + 3.f);
}

// once the averaging window reaches the filter time constant, the average filters normally without a minimum
TEST_F(BatteryTest, MotorFailureResetHandsOverToFilter)
{
	int32_t n_cells = 3;
	param_set(param_find("BAT1_N_CELLS"), &n_cells);

	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	hrt_abstime timestamp = 1_s;
	publishArmed(true);
	fly(battery, 10.f, 10_s, timestamp);
	const float average_before = battery.getCurrentAverage();

	publishMotorFailureMask(1);
	fly(battery, 20.f, 53_s, timestamp);
	EXPECT_NEAR(battery.getCurrentAverage(), 20.f, 0.1f);

	// filter: about 7.4 A, continued averaging: held at the average before
	fly(battery, 2.f, 60_s, timestamp);
	EXPECT_LT(battery.getCurrentAverage(), average_before - 1.f);
}

// samples without a valid current don't count towards the hold
TEST_F(BatteryTest, MotorFailureHoldIgnoresInvalidCurrent)
{
	int32_t n_cells = 3;
	param_set(param_find("BAT1_N_CELLS"), &n_cells);

	Battery battery(1, nullptr, 20_ms, battery_status_s::SOURCE_POWER_MODULE);
	hrt_abstime timestamp = 1_s;
	publishArmed(true);
	fly(battery, 10.f, 10_s, timestamp);
	const float average_before = battery.getCurrentAverage();

	publishMotorFailureMask(1);
	fly(battery, -1.f, 5_s, timestamp);
	EXPECT_FLOAT_EQ(battery.getCurrentAverage(), average_before);

	fly(battery, 60.f, 1900_ms, timestamp);
	EXPECT_NEAR(battery.getCurrentAverage(), average_before, 0.1f);
}
