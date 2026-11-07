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
