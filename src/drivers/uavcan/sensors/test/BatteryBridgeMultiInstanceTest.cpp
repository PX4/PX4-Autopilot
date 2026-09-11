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

// Own gtest binary so multi-instance uORB advertises don't leak into the
// single-bridge BatteryBridgeTest suite.

#include "UavcanBridgeTestFixture.hpp"

#include <sensors/battery.hpp>

#include <ardupilot/equipment/power/BatteryContinuous.hpp>

#include <uORB/topics/battery_status.h>
#include <uORB/Subscription.hpp>
#include <uORB/uORB.h>

namespace
{

constexpr unsigned kSpinMs = 50;

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

class BatteryBridgeMultiInstanceTest : public UavcanBridgeTestFixtureT<3>
{
protected:
	void SetUp() override
	{
		UavcanBridgeTestFixtureT<3>::SetUp();
		_bridge = std::make_unique<UavcanBatteryBridge>(subscriber_node(), nullptr);
		ASSERT_EQ(_bridge->init(), 0);
	}

	void TearDown() override
	{
		_bridge.reset();
		UavcanBridgeTestFixtureT<3>::TearDown();
	}

	std::unique_ptr<UavcanBatteryBridge> _bridge;
};

TEST_F(BatteryBridgeMultiInstanceTest, TwoNodesFillSeparateInstances)
{
	ardupilot::equipment::power::BatteryContinuous msg_a;
	msg_a.voltage = 22.0f;
	msg_a.current = 3.0f;
	msg_a.state_of_charge = 60.0f;
	msg_a.slot_id = 1;

	ardupilot::equipment::power::BatteryContinuous msg_b;
	msg_b.voltage = 24.5f;
	msg_b.current = 5.5f;
	msg_b.state_of_charge = 85.0f;
	msg_b.slot_id = 2;

	ASSERT_GE(broadcast(msg_a, /*publisher_idx*/ 0), 0);
	ASSERT_GE(broadcast(msg_b, /*publisher_idx*/ 1), 0);
	ASSERT_GE(spin(kSpinMs), 0);

	battery_status_s data_a{};
	battery_status_s data_b{};

	constexpr uint8_t kNodeA = uavcan_bridge_test::kFirstPublisherNodeId;
	constexpr uint8_t kNodeB = uavcan_bridge_test::kFirstPublisherNodeId + 1;

	ASSERT_TRUE(find_battery_status_for_node(kNodeA, data_a))
			<< "battery_status instance for node " << (int)kNodeA << " not populated";
	ASSERT_TRUE(find_battery_status_for_node(kNodeB, data_b))
			<< "battery_status instance for node " << (int)kNodeB << " not populated";

	EXPECT_FLOAT_EQ(data_a.voltage_v, 22.0f);
	EXPECT_FLOAT_EQ(data_b.voltage_v, 24.5f);
}

} // namespace
