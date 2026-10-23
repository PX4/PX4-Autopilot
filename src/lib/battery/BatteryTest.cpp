/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

// to run: make tests TESTFILTER=Battery

using namespace time_literals;

static constexpr int N_CELLS = 4;
static constexpr float FULL_VOLTAGE = 4.2f * N_CELLS;
static constexpr hrt_abstime SAMPLE_INTERVAL = 50_ms;

class BatteryTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();

		// The battery only reads its parameters on construction
		const int32_t n_cells = N_CELLS;
		param_set(param_find("BAT1_N_CELLS"), &n_cells);

		_battery = new Battery(1, nullptr, SAMPLE_INTERVAL, battery_status_s::SOURCE_POWER_MODULE);
		_battery->setCapacityMah(1000.f);
		_battery->setConnected(true);
	}

	void TearDown() override
	{
		delete _battery;
	}

	// Feed constant samples at the battery sample rate for the given duration
	void run(const float current_a, const hrt_abstime duration, const float voltage_v = FULL_VOLTAGE)
	{
		const hrt_abstime end = _now + duration;

		while (_now < end) {
			_now += SAMPLE_INTERVAL;
			_battery->updateVoltage(voltage_v);
			_battery->updateCurrent(current_a);
			_battery->updateBatteryStatus(_now);
		}
	}

	float remaining() { return _battery->getBatteryStatus().remaining; }

	Battery *_battery{nullptr};
	hrt_abstime _now{1_s};
};

TEST_F(BatteryTest, CoulombCountIntegratesCurrent)
{
	// GIVEN: an initialized battery
	run(0.f, 3_s);
	EXPECT_FLOAT_EQ(_battery->getDischargedMah(), 0.f);

	// WHEN: drawing 36 A for 50 s
	run(36.f, 50_s);

	// THEN: 36 A * 50 s / 3600 s/h = 0.5 Ah are counted
	EXPECT_NEAR(_battery->getDischargedMah(), 500.f, 1.f);
}

TEST_F(BatteryTest, ZeroOrInvalidCurrentStopsCount)
{
	// The battery simulator holds the charge at SIM_BAT_MIN_PCT by publishing 0 A
	// GIVEN: a partially discharged battery
	run(0.f, 3_s);
	run(36.f, 50_s);
	const float discharged_mah = _battery->getDischargedMah();

	// WHEN: no current is drawn
	run(0.f, 10_s);

	// THEN: the count does not change
	EXPECT_FLOAT_EQ(_battery->getDischargedMah(), discharged_mah);

	// WHEN: the current is invalid
	run(-1.f, 10_s);

	// THEN: the count does not change either
	EXPECT_FLOAT_EQ(_battery->getDischargedMah(), discharged_mah);
}

TEST_F(BatteryTest, ResetDischargedMahSetsCount)
{
	// GIVEN: a partially discharged battery
	run(0.f, 3_s);
	run(36.f, 50_s);

	// WHEN: reset to a given discharged capacity
	_battery->resetDischargedMah(200.f);

	// THEN: the count restarts from that value
	EXPECT_FLOAT_EQ(_battery->getDischargedMah(), 200.f);
	run(36.f, 10_s);
	EXPECT_NEAR(_battery->getDischargedMah(), 300.f, 1.f);

	// WHEN: reset with no argument
	_battery->resetDischargedMah();

	// THEN: the count restarts from empty
	EXPECT_FLOAT_EQ(_battery->getDischargedMah(), 0.f);

	// WHEN: reset to a negative value
	_battery->resetDischargedMah(-100.f);

	// THEN: the count is clamped to 0
	EXPECT_FLOAT_EQ(_battery->getDischargedMah(), 0.f);
}

TEST_F(BatteryTest, ResetDischargedMahReinitializesStateOfCharge)
{
	// GIVEN: a battery discharged to half its capacity, with the voltage still reading full
	run(0.f, 3_s);
	EXPECT_NEAR(remaining(), 1.f, 0.01f);
	run(36.f, 50_s);
	run(0.f, 3_s);
	EXPECT_NEAR(remaining(), 0.5f, 0.01f);

	// WHEN: the count is reset, e.g. a fresh battery after disarming in simulation
	_battery->resetDischargedMah();
	run(0.f, 3_s);

	// THEN: the state of charge restarts from the voltage instead of keeping the old estimate
	EXPECT_NEAR(remaining(), 1.f, 0.01f);

	// AND: the battery drains again from full
	run(36.f, 25_s);
	run(0.f, 3_s);
	EXPECT_NEAR(remaining(), 0.75f, 0.01f);
}
