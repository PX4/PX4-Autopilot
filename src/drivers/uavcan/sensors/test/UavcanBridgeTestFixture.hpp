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

/**
 * @file UavcanBridgeTestFixture.hpp
 *
 * N libuavcan nodes on a loopback bus (PairableCanDriver): index 0 hosts the
 * bridge under test, indices >=1 publish synthetic messages. Template stays
 * because TestNetwork<N> is compile-time parameterised.
 */

#pragma once

#include <gtest/gtest.h>
#include <memory>

#include <libdronecan/libuavcan/test/clock.hpp>
#include <libdronecan/libuavcan/test/node/test_node.hpp>

#include <uavcan/uavcan.hpp>

namespace uavcan_bridge_test
{

// Subscriber=1, publishers contiguous from 2.
static constexpr uint8_t kSubscriberNodeId     = 1;
static constexpr uint8_t kFirstPublisherNodeId = 2;

} // namespace uavcan_bridge_test

template <unsigned NumNodes = 2>
class UavcanBridgeTestFixtureT : public ::testing::Test
{
protected:
	static_assert(NumNodes >= 2, "Need at least one subscriber + one publisher");

	void SetUp() override
	{
		_network = std::make_unique<TestNetwork<NumNodes>>(uavcan_bridge_test::kSubscriberNodeId);
	}

	void TearDown() override
	{
		_network.reset();
	}

	TestNode &subscriber_node() { return (*_network)[0]; }
	TestNode &publisher_node(unsigned idx = 0) { return (*_network)[1 + idx]; }

	int spin(unsigned ms)
	{
		return _network->spinAll(uavcan::MonotonicDuration::fromMSec(ms));
	}

	template <typename Msg>
	int broadcast(const Msg &msg, unsigned publisher_idx = 0)
	{
		uavcan::Publisher<Msg> pub(publisher_node(publisher_idx));
		const int init_res = pub.init();

		if (init_res < 0) {
			return init_res;
		}

		return pub.broadcast(msg);
	}

	std::unique_ptr<TestNetwork<NumNodes>> _network;
};

using UavcanBridgeTestFixture = UavcanBridgeTestFixtureT<>;
