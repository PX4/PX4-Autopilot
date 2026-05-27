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

#include "UavcanBridgeTestFixture.hpp"

#include <sensors/battery.hpp>

#include <ardupilot/equipment/power/BatteryContinuous.hpp>
#include <ardupilot/equipment/power/BatteryPeriodic.hpp>
#include <ardupilot/equipment/power/BatteryCells.hpp>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/battery_info.h>
#include <uORB/Subscription.hpp>
#include <uORB/uORB.h>

#include <cmath>
#include <cstring>

namespace
{

constexpr unsigned kSpinMs = 50;

// Single publisher node ID across the suite — bridge reuses one
// _battery_status slot, so uORB only advertises one instance per topic.
constexpr uint8_t kPublisherNodeId = uavcan_bridge_test::kFirstPublisherNodeId;

bool find_battery_status_for_node(uint8_t wanted_node_id, battery_status_s &out)
{
	hrt_abstime newest = 0;
	bool found = false;

	for (uint8_t inst = 0; inst < ORB_MULTI_MAX_INSTANCES; inst++) {
		uORB::Subscription sub{ORB_ID(battery_status), inst};
		battery_status_s data{};

		if (!sub.copy(&data)) {
			continue;
		}

		if (data.id == wanted_node_id && data.timestamp >= newest) {
			newest = data.timestamp;
			out = data;
			found = true;
		}
	}

	return found;
}

bool find_battery_info_for_node(uint8_t wanted_node_id, battery_info_s &out)
{
	hrt_abstime newest = 0;
	bool found = false;

	for (uint8_t inst = 0; inst < ORB_MULTI_MAX_INSTANCES; inst++) {
		uORB::Subscription sub{ORB_ID(battery_info), inst};
		battery_info_s data{};

		if (!sub.copy(&data)) {
			continue;
		}

		if (data.id == wanted_node_id && data.timestamp >= newest) {
			newest = data.timestamp;
			out = data;
			found = true;
		}
	}

	return found;
}

ardupilot::equipment::power::BatteryPeriodic make_default_periodic()
{
	ardupilot::equipment::power::BatteryPeriodic per;
	per.cells_in_series      = 6;
	per.nominal_voltage      = 22.2f;
	per.full_charge_capacity = 6.6f;
	per.design_capacity      = 6.6f;
	per.cycle_count          = 42;
	per.state_of_health      = 98;
	return per;
}

ardupilot::equipment::power::BatteryContinuous make_default_continuous()
{
	ardupilot::equipment::power::BatteryContinuous cont;
	cont.voltage         = 22.2f;
	cont.current         = 4.0f;
	cont.state_of_charge = 80.0f;
	cont.slot_id         = 1;
	cont.status_flags    = 0;
	return cont;
}

// Bridge + network shared across the suite — uORB has no unadvertise path,
// so a fresh bridge per test would exhaust ORB_MULTI_MAX_INSTANCES.
// reset_to_baseline() in SetUp() decouples per-test state.
class BatteryBridgeTest : public ::testing::Test
{
protected:
	static constexpr unsigned kNumNodes = 2;

	static void SetUpTestSuite()
	{
		_shared_network = std::make_unique<TestNetwork<kNumNodes>>(uavcan_bridge_test::kSubscriberNodeId);
		_shared_bridge  = std::make_unique<UavcanBatteryBridge>((*_shared_network)[0], nullptr);
		ASSERT_EQ(_shared_bridge->init(), 0);
	}

	static void TearDownTestSuite()
	{
		_shared_bridge.reset();
		_shared_network.reset();
	}

	void SetUp() override
	{
		// Substitute for a test-only state-reset hook into the bridge.
		reset_to_baseline();
	}

	TestNode &publisher_node() { return (*_shared_network)[1]; }

	int spin(unsigned ms)
	{
		return _shared_network->spinAll(uavcan::MonotonicDuration::fromMSec(ms));
	}

	template <typename Msg>
	int broadcast(const Msg &msg)
	{
		uavcan::Publisher<Msg> pub(publisher_node());
		const int init_res = pub.init();

		if (init_res < 0) {
			return init_res;
		}

		return pub.broadcast(msg);
	}

	UavcanBatteryBridge *bridge() { return _shared_bridge.get(); }

	static std::unique_ptr<TestNetwork<kNumNodes>> _shared_network;
	static std::unique_ptr<UavcanBatteryBridge>    _shared_bridge;

private:
	void reset_to_baseline()
	{
		// Periodic with 0 cycle_count / 0 state_of_health stamps known values
		// past the bridge's UINT16_MAX / UINT8_MAX skip conditions.
		ardupilot::equipment::power::BatteryPeriodic per;
		per.cells_in_series      = 0;
		per.nominal_voltage      = 22.2f;
		per.full_charge_capacity = NAN;
		per.design_capacity      = 0.0f;
		per.cycle_count          = 0;
		per.state_of_health      = 0;
		ASSERT_GE(broadcast(per), 0);
		ASSERT_GE(spin(kSpinMs), 0);

		// 14 zero-volt entries wipe voltage_cell_v left by prior tests.
		ardupilot::equipment::power::BatteryCells cells;
		cells.index = 0;
		constexpr size_t kMaxCells = 14;

		for (size_t i = 0; i < kMaxCells; ++i) {
			cells.voltages.push_back(0.0f);
		}

		ASSERT_GE(broadcast(cells), 0);
		ASSERT_GE(spin(kSpinMs), 0);

		// Continuous flushes merged state to battery_status uORB.
		ASSERT_GE(broadcast(make_default_continuous()), 0);
		ASSERT_GE(spin(kSpinMs), 0);
	}
};

std::unique_ptr<TestNetwork<BatteryBridgeTest::kNumNodes>> BatteryBridgeTest::_shared_network;
std::unique_ptr<UavcanBatteryBridge>                       BatteryBridgeTest::_shared_bridge;

TEST_F(BatteryBridgeTest, BridgeReportsName)
{
	EXPECT_STREQ(bridge()->get_name(), "battery");
}

TEST_F(BatteryBridgeTest, ContinuousFieldsMapToBatteryStatus)
{
	ardupilot::equipment::power::BatteryContinuous msg;
	msg.voltage          = 23.1f;
	msg.current          = 8.2f;
	msg.state_of_charge  = 72.0f;
	msg.capacity_consumed = 1.8f;
	msg.slot_id          = 7;
	msg.status_flags     = 0;

	ASSERT_GE(broadcast(msg), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	battery_status_s data{};
	ASSERT_TRUE(find_battery_status_for_node(kPublisherNodeId, data)) << "no battery_status published";

	EXPECT_FLOAT_EQ(data.voltage_v, 23.1f);
	EXPECT_FLOAT_EQ(data.current_a, 8.2f);
	EXPECT_NEAR(data.remaining, 0.72f, 1e-3f);
	EXPECT_TRUE(data.connected);
	EXPECT_EQ(data.source, battery_status_s::SOURCE_EXTERNAL);
}

TEST_F(BatteryBridgeTest, PeriodicSetsCellCountAndNominalVoltage)
{
	// Continuous first so the node enters Multi mode and owns its slot.
	ASSERT_GE(broadcast(make_default_continuous()), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	ardupilot::equipment::power::BatteryPeriodic per;
	per.cells_in_series       = 6;
	per.nominal_voltage       = 22.2f;
	per.full_charge_capacity  = 6.6f; // Ah -> 6600 mAh
	per.design_capacity       = 6.6f;
	per.cycle_count           = 42;
	per.state_of_health       = 98;

	ASSERT_GE(broadcast(per), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	// Periodic updates the bridge cache but only publishes battery_info;
	// follow-up Continuous flushes to battery_status uORB.
	ASSERT_GE(broadcast(make_default_continuous()), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	battery_status_s data{};
	ASSERT_TRUE(find_battery_status_for_node(kPublisherNodeId, data)) << "no battery_status published";

	EXPECT_EQ(data.cell_count, 6);
	// float16 wire format — loose tolerance for quantisation
	EXPECT_NEAR(data.nominal_voltage, 22.2f, 0.05f);
	EXPECT_EQ(data.capacity, 6600);
	EXPECT_EQ(data.cycle_count, 42);
	EXPECT_EQ(data.state_of_health, 98);
}

TEST_F(BatteryBridgeTest, CellsPopulateVoltagesAndDelta)
{
	ASSERT_GE(broadcast(make_default_continuous()), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	ardupilot::equipment::power::BatteryCells cells;
	cells.index = 0;
	cells.voltages.push_back(3.85f);
	cells.voltages.push_back(3.84f);
	cells.voltages.push_back(3.86f);
	cells.voltages.push_back(3.85f);
	cells.voltages.push_back(3.83f);
	cells.voltages.push_back(3.85f);

	ASSERT_GE(broadcast(cells), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	// Cells doesn't publish on its own; follow-up Continuous flushes to uORB.
	ASSERT_GE(broadcast(make_default_continuous()), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	battery_status_s data{};
	ASSERT_TRUE(find_battery_status_for_node(kPublisherNodeId, data)) << "no battery_status published";

	EXPECT_NEAR(data.voltage_cell_v[0], 3.85f, 1e-2f);
	EXPECT_NEAR(data.voltage_cell_v[2], 3.86f, 1e-2f);
	EXPECT_NEAR(data.voltage_cell_v[4], 3.83f, 1e-2f);
	// float16 quantisation tolerance
	EXPECT_NEAR(data.max_cell_voltage_delta, 0.03f, 1e-2f);
}

TEST_F(BatteryBridgeTest, PeriodicSerialNumberPublishedToBatteryInfo)
{
	auto per = make_default_periodic();
	const char *serial = "ABC123";

	for (const char *p = serial; *p; ++p) {
		per.serial_number.push_back((uint8_t)*p);
	}

	ASSERT_GE(broadcast(per), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	battery_info_s info{};
	ASSERT_TRUE(find_battery_info_for_node(kPublisherNodeId, info)) << "no battery_info published";
	EXPECT_STREQ(info.serial_number, serial);
}

TEST_F(BatteryBridgeTest, BatteryInfoIdMatchesBatteryStatusId)
{
	auto per = make_default_periodic();
	per.serial_number.push_back('S');
	ASSERT_GE(broadcast(per), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	ASSERT_GE(broadcast(make_default_continuous()), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	battery_status_s status{};
	battery_info_s   info{};
	ASSERT_TRUE(find_battery_status_for_node(kPublisherNodeId, status));
	ASSERT_TRUE(find_battery_info_for_node(kPublisherNodeId, info));
	EXPECT_EQ(status.id, info.id);
}

TEST_F(BatteryBridgeTest, PeriodicWithZeroNominalVoltageYieldsNan)
{
	auto per = make_default_periodic();
	per.nominal_voltage = 0.0f;

	ASSERT_GE(broadcast(per), 0);
	ASSERT_GE(spin(kSpinMs), 0);
	ASSERT_GE(broadcast(make_default_continuous()), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	battery_status_s data{};
	ASSERT_TRUE(find_battery_status_for_node(kPublisherNodeId, data));
	EXPECT_TRUE(std::isnan(data.nominal_voltage)) << "expected NaN, got " << data.nominal_voltage;
}

TEST_F(BatteryBridgeTest, PeriodicWithoutCapacityKeepsZeroCapacity)
{
	auto per = make_default_periodic();
	per.full_charge_capacity = NAN;
	per.design_capacity      = 0.0f;

	ASSERT_GE(broadcast(per), 0);
	ASSERT_GE(spin(kSpinMs), 0);
	ASSERT_GE(broadcast(make_default_continuous()), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	battery_status_s data{};
	ASSERT_TRUE(find_battery_status_for_node(kPublisherNodeId, data));
	EXPECT_EQ(data.capacity, 0);
}

TEST_F(BatteryBridgeTest, PeriodicFallsBackToDesignCapacityWhenFullChargeNan)
{
	auto per = make_default_periodic();
	per.full_charge_capacity = NAN;
	per.design_capacity      = 5.0f; // 5 Ah -> 5000 mAh

	ASSERT_GE(broadcast(per), 0);
	ASSERT_GE(spin(kSpinMs), 0);
	ASSERT_GE(broadcast(make_default_continuous()), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	battery_status_s data{};
	ASSERT_TRUE(find_battery_status_for_node(kPublisherNodeId, data));
	EXPECT_EQ(data.capacity, 5000);
}

TEST_F(BatteryBridgeTest, PeriodicSkipsInvalidCycleCountAndStateOfHealth)
{
	auto good = make_default_periodic();
	good.cycle_count     = 100;
	good.state_of_health = 50;
	ASSERT_GE(broadcast(good), 0);
	ASSERT_GE(spin(kSpinMs), 0);
	ASSERT_GE(broadcast(make_default_continuous()), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	auto sentinel = make_default_periodic();
	sentinel.cycle_count     = UINT16_MAX;
	sentinel.state_of_health = UINT8_MAX;
	ASSERT_GE(broadcast(sentinel), 0);
	ASSERT_GE(spin(kSpinMs), 0);
	ASSERT_GE(broadcast(make_default_continuous()), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	battery_status_s data{};
	ASSERT_TRUE(find_battery_status_for_node(kPublisherNodeId, data));
	EXPECT_EQ(data.cycle_count, 100);
	EXPECT_EQ(data.state_of_health, 50);
}

TEST_F(BatteryBridgeTest, ContinuousChargingFlagSetsChargingWarning)
{
	auto cont = make_default_continuous();
	cont.status_flags = ardupilot::equipment::power::BatteryContinuous::STATUS_FLAG_CHARGING;

	ASSERT_GE(broadcast(cont), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	battery_status_s data{};
	ASSERT_TRUE(find_battery_status_for_node(kPublisherNodeId, data));
	EXPECT_EQ(data.warning, battery_status_s::STATE_CHARGING);
	EXPECT_EQ(data.faults, 0u);
}

TEST_F(BatteryBridgeTest, ContinuousFaultFlagsMapToFaultBits)
{
	using BC = ardupilot::equipment::power::BatteryContinuous;
	auto cont = make_default_continuous();
	cont.status_flags = BC::STATUS_FLAG_FAULT_OVER_CURRENT | BC::STATUS_FLAG_FAULT_OVER_TEMP;

	ASSERT_GE(broadcast(cont), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	battery_status_s data{};
	ASSERT_TRUE(find_battery_status_for_node(kPublisherNodeId, data));
	EXPECT_NE(data.faults & (1u << battery_status_s::FAULT_OVER_CURRENT), 0u);
	EXPECT_NE(data.faults & (1u << battery_status_s::FAULT_OVER_TEMPERATURE), 0u);
	EXPECT_EQ(data.warning, battery_status_s::STATE_UNHEALTHY);
}

} // namespace
