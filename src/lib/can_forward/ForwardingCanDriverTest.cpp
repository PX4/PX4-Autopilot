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

/**
 * ForwardingCanDriver against the real libuavcan CanIOManager.
 *
 * Test matrix rows D1-D13, A2, A4 (see TEST_MATRIX.md)
 */

#include <gtest/gtest.h>

#include "ForwardingCanDriver.hpp"
#include "CanForwardTestHelpers.hpp"
#include "FakeCan.hpp"

#include <uavcan/dynamic_memory.hpp>
#include <uavcan/transport/can_io.hpp>

#include <vector>

using namespace can_forward;
using namespace can_forward_test;

namespace
{

struct SinkEntry {
	uint8_t iface;
	uavcan::CanFrame frame;
};

class VectorSink : public IFrameSink
{
public:
	void onFrameToGcs(uint8_t iface_index, const uavcan::CanFrame &frame) override
	{
		frames.push_back({iface_index, frame});
	}

	std::vector<SinkEntry> frames;
};

static uavcan::CanFrame makeFrame(uint32_t id, uint8_t len = 8, uint8_t seed = 0)
{
	uint8_t data[8];

	for (uint8_t i = 0; i < 8; i++) { data[i] = static_cast<uint8_t>(seed + i); }

	return uavcan::CanFrame(id, data, len);
}

static Frame toFrame(const uavcan::CanFrame &f)
{
	Frame out{};
	out.id = f.id;
	out.len = f.dlc;
	memcpy(out.data, f.data, f.dlc);
	return out;
}

static constexpr uint8_t LocalNode = 10;
static constexpr uint8_t TargetNode = 42;
static constexpr uint8_t GcsNode = 127;

} // namespace

class ForwardingCanDriverTest : public ::testing::Test
{
protected:
	ForwardingCanDriverTest() :
		phys(clock, 2),
		fwd(phys, clock),
		canio(fwd, pool, clock)
	{
		phys.iface(0).setBus(&bus0);
		phys.iface(1).setBus(&bus1);
		peer0.setBus(&bus0);
		peer1.setBus(&bus1);
	}

	void activate(int8_t bus = 0, bool inject_allowed = true)
	{
		fwd.configure(bus, inject_allowed, &filter, &sink, LocalNode);
	}

	// one receive() call as the node's spinOnce() would do it, returns the number of frames received
	int drainRx(std::vector<uavcan::CanRxFrame> *out = nullptr)
	{
		int n = 0;

		while (true) {
			uavcan::CanRxFrame frame;
			uavcan::CanIOFlags flags = 0;
			const int res = canio.receive(frame, uavcan::MonotonicTime(), flags);

			if (res <= 0) { break; }

			n++;

			if (out) { out->push_back(frame); }
		}

		return n;
	}

	uavcan::MonotonicTime deadline(uint64_t ms = 100) const
	{
		return clock.getMonotonic() + uavcan::MonotonicDuration::fromMSec(ms);
	}

	FakeClock clock;
	FakeBus bus0;
	FakeBus bus1;
	FakeIface peer0{clock};
	FakeIface peer1{clock};
	FakeCanDriver phys;
	ForwardingCanDriver fwd;
	uavcan::PoolAllocator<uavcan::MemPoolBlockSize * 512, uavcan::MemPoolBlockSize> pool;
	uavcan::CanIOManager canio;
	CanForwardFilter filter;
	VectorSink sink;
};

TEST_F(ForwardingCanDriverTest, PassesThroughInterfaces)
{
	EXPECT_EQ(fwd.getNumIfaces(), 2);
	EXPECT_NE(fwd.getIface(0), nullptr);
	EXPECT_NE(fwd.getIface(1), nullptr);
	EXPECT_EQ(fwd.getIface(2), nullptr);
}

// D1
TEST_F(ForwardingCanDriverTest, BusRxMirroredOnce)
{
	activate();
	peer0.send(makeFrame(messageId(NodeStatus, TargetNode)), deadline(), 0);

	EXPECT_EQ(drainRx(), 1);
	ASSERT_EQ(sink.frames.size(), 1u);
	EXPECT_EQ(sink.frames[0].iface, 0);
	EXPECT_EQ(sink.frames[0].frame, makeFrame(messageId(NodeStatus, TargetNode)));
	EXPECT_EQ(fwd.counters().frames_to_gcs, 1u);
}

// D2
TEST_F(ForwardingCanDriverTest, OwnTxMirroredOnlyAfterTransmit)
{
	activate();
	phys.iface(0).mailboxes_free = 0;

	const uavcan::CanFrame own = makeFrame(messageId(NodeStatus, LocalNode));
	canio.send(own, deadline(), uavcan::MonotonicTime(), 1, uavcan::CanTxQueue::Persistent, 0);
	EXPECT_TRUE(sink.frames.empty()); // queued, not transmitted yet

	phys.iface(0).mailboxes_free = 1;
	drainRx(); // the node's spin also flushes the TX queue

	ASSERT_EQ(sink.frames.size(), 1u);
	EXPECT_EQ(sink.frames[0].frame, own);
	ASSERT_EQ(bus0.wire.size(), 1u);
}

// D3
TEST_F(ForwardingCanDriverTest, LoopbackNotMirroredTwice)
{
	activate();
	const uavcan::CanFrame sync = makeFrame(messageId(4, LocalNode)); // GlobalTimeSync uses loopback
	canio.send(sync, deadline(), uavcan::MonotonicTime(), 1, uavcan::CanTxQueue::Persistent, uavcan::CanIOFlagLoopback);

	std::vector<uavcan::CanRxFrame> rx;
	EXPECT_EQ(drainRx(&rx), 1); // loopback is delivered to the node
	EXPECT_EQ(sink.frames.size(), 1u); // but only mirrored once, at send time
}

// D4
TEST_F(ForwardingCanDriverTest, GcsFrameOnWireOnceNotEchoed)
{
	activate();
	const uavcan::CanFrame f = makeFrame(serviceId(FileRead, false, TargetNode, GcsNode));
	const Frame frames[] = {toFrame(f)};

	EXPECT_EQ(pumpToBus(canio, 0, frames, 1, deadline()), 1);
	drainRx();

	ASSERT_EQ(bus0.wire.size(), 1u);
	EXPECT_EQ(bus0.wire[0], f);
	ASSERT_EQ(phys.iface(0).sent.size(), 1u);
	EXPECT_EQ(phys.iface(0).sent[0].flags & CanIOFlagForwarded, 0) << "private flag must not reach the platform driver";
	EXPECT_TRUE(sink.frames.empty());
	EXPECT_EQ(fwd.counters().frames_to_bus, 1u);
}

// D5
TEST_F(ForwardingCanDriverTest, GcsRequestToLocalNodeIsInjected)
{
	activate();
	const uavcan::CanFrame request = makeFrame(serviceId(GetNodeInfo, true, LocalNode, GcsNode), 1);
	const uavcan::CanFrame broadcast = makeFrame(messageId(NodeStatus, GcsNode), 7);
	const Frame frames[] = {toFrame(request), toFrame(broadcast)};

	EXPECT_EQ(pumpToBus(canio, 0, frames, 2, deadline()), 2);

	std::vector<uavcan::CanRxFrame> rx;
	EXPECT_EQ(drainRx(&rx), 2);
	EXPECT_EQ(rx[0], request);
	EXPECT_EQ(rx[0].iface_index, 0);
	EXPECT_EQ(rx[1], broadcast);
	EXPECT_TRUE(sink.frames.empty()) << "injected frames must not go back to the GCS";

	// and they went out on the wire as well
	EXPECT_EQ(bus0.wire.size(), 2u);
}

TEST_F(ForwardingCanDriverTest, AnonymousGcsFrameIsInjected)
{
	activate();
	const Frame frames[] = {toFrame(makeFrame(anonymousId(0x77, 1)))};
	pumpToBus(canio, 0, frames, 1, deadline());
	EXPECT_EQ(drainRx(), 1);
}

// D6
TEST_F(ForwardingCanDriverTest, GcsFrameForOtherNodeIsNotInjected)
{
	activate();
	std::vector<Frame> frames;

	for (uint8_t i = 0; i < 37; i++) {
		frames.push_back(toFrame(makeFrame(serviceId(FileRead, false, TargetNode, GcsNode), 8, i)));
	}

	EXPECT_EQ(pumpToBus(canio, 0, frames.data(), frames.size(), deadline()), frames.size());
	EXPECT_EQ(drainRx(), 0);
	EXPECT_EQ(bus0.wire.size(), 37u);
	EXPECT_EQ(fwd.counters().dropped_inject_queue, 0u);
}

TEST_F(ForwardingCanDriverTest, LocalNodeIdZeroInjectsNoServices)
{
	fwd.configure(0, true, &filter, &sink, 0);
	const Frame frames[] = {toFrame(makeFrame(serviceId(GetNodeInfo, true, 0, GcsNode)))};
	pumpToBus(canio, 0, frames, 1, deadline());
	EXPECT_EQ(drainRx(), 0);
}

TEST_F(ForwardingCanDriverTest, InjectQueueOverflowIsCounted)
{
	activate();
	std::vector<Frame> frames;

	for (uint8_t i = 0; i < ForwardingCanDriver::InjectQueueLength + 3; i++) {
		frames.push_back(toFrame(makeFrame(messageId(NodeStatus, GcsNode), 8, i)));
	}

	pumpToBus(canio, 0, frames.data(), frames.size(), deadline());
	EXPECT_EQ(fwd.counters().dropped_inject_queue, 3u);
	EXPECT_EQ(drainRx(), ForwardingCanDriver::InjectQueueLength);
	EXPECT_EQ(bus0.wire.size(), frames.size()) << "the bus side is not affected";
}

// D7
TEST_F(ForwardingCanDriverTest, SelectOnlyReportsRequestedReads)
{
	activate();
	const Frame frames[] = {toFrame(makeFrame(messageId(NodeStatus, GcsNode)))};
	pumpToBus(canio, 0, frames, 1, deadline());

	const uavcan::CanFrame *pending[uavcan::MaxCanIfaces] = {};
	uavcan::CanSelectMasks masks;
	masks.read = 0;
	masks.write = 1;
	EXPECT_GE(fwd.select(masks, pending, deadline()), 0);
	EXPECT_EQ(masks.read, 0) << "CanIOManager::send() asserts that read stays 0";

	masks.read = 0b10; // only iface 1 requested
	masks.write = 0;
	fwd.select(masks, pending, deadline());
	EXPECT_EQ(masks.read, 0);

	masks.read = 0b11;
	masks.write = 0;
	EXPECT_EQ(fwd.select(masks, pending, deadline()), 1);
	EXPECT_EQ(masks.read, 0b01);
	EXPECT_EQ(phys.last_blocking_deadline, uavcan::MonotonicTime()) << "must not block with injected frames pending";

	// a send while an injected frame is pending works and does not lose it
	canio.send(makeFrame(messageId(NodeStatus, LocalNode)), deadline(), uavcan::MonotonicTime(), 1,
		   uavcan::CanTxQueue::Persistent, 0);
	EXPECT_EQ(drainRx(), 1);
}

TEST_F(ForwardingCanDriverTest, SelectPassesBlockingDeadlineWithoutInjected)
{
	activate();
	const uavcan::CanFrame *pending[uavcan::MaxCanIfaces] = {};
	uavcan::CanSelectMasks masks;
	masks.read = 0b11;
	fwd.select(masks, pending, deadline(50));
	EXPECT_EQ(phys.last_blocking_deadline, deadline(50));
}

// D8 + D9: a burst of four pipelined 256 byte file read responses with one free mailbox per tick
TEST_F(ForwardingCanDriverTest, BurstKeepsOrderWithSlowMailbox)
{
	activate();
	std::vector<Frame> frames;

	for (int transfer = 0; transfer < 4; transfer++) {
		for (int i = 0; i < 37; i++) {
			// same CAN ID for all frames of a transfer, the payload tells them apart
			frames.push_back(toFrame(makeFrame(serviceId(FileRead, false, TargetNode, GcsNode), 8,
							   static_cast<uint8_t>(transfer * 37 + i))));
		}
	}

	phys.iface(0).mailboxes_free = 0;
	size_t handed_over = 0;
	int ticks = 0;

	while (handed_over < frames.size() || bus0.wire.size() < frames.size()) {
		ASSERT_LT(ticks++, 2000);
		handed_over += pumpToBus(canio, 0, frames.data() + handed_over, frames.size() - handed_over, deadline());
		EXPECT_LE(canio.getTxQueuePerfCounters(0).peak_used_blocks, 1) << "at most one forwarded frame queued";
		phys.iface(0).mailboxes_free = 1;
		drainRx();
		clock.advance(150);
	}

	ASSERT_EQ(bus0.wire.size(), frames.size());

	for (size_t i = 0; i < frames.size(); i++) {
		EXPECT_EQ(bus0.wire[i], uavcan::CanFrame(frames[i].id, frames[i].data, frames[i].len)) << i;
	}

	EXPECT_EQ(canio.getTxQueuePerfCounters(0).rejected_frames, 0u);
	EXPECT_EQ(fwd.counters().frames_to_bus, frames.size());
}

TEST_F(ForwardingCanDriverTest, BurstWithFreeMailboxGoesOutImmediately)
{
	activate();
	std::vector<Frame> frames(150);

	for (size_t i = 0; i < frames.size(); i++) {
		frames[i] = toFrame(makeFrame(serviceId(FileRead, false, TargetNode, GcsNode), 8, static_cast<uint8_t>(i)));
	}

	EXPECT_EQ(pumpToBus(canio, 0, frames.data(), frames.size(), deadline()), frames.size());
	EXPECT_EQ(bus0.wire.size(), frames.size());
}

// D10
TEST_F(ForwardingCanDriverTest, ExpiredFrameIsDroppedWithoutLeak)
{
	activate();
	phys.iface(0).mailboxes_free = 0;
	const Frame frames[] = {toFrame(makeFrame(serviceId(FileRead, false, TargetNode, GcsNode)))};

	EXPECT_EQ(pumpToBus(canio, 0, frames, 1, deadline(100)), 1);
	EXPECT_GT(pool.getNumUsedBlocks(), 0);

	clock.advance(200'000);
	phys.iface(0).mailboxes_free = 1;
	drainRx();

	EXPECT_TRUE(bus0.wire.empty());
	EXPECT_EQ(canio.getTxQueuePerfCounters(0).expired_frames, 1u);
	EXPECT_EQ(pool.getNumUsedBlocks(), 0);
}

// D11
TEST_F(ForwardingCanDriverTest, InactiveIsPassThrough)
{
	fwd.configure(-1, false, &filter, &sink, LocalNode);
	peer0.send(makeFrame(messageId(NodeStatus, TargetNode)), deadline(), 0);
	canio.send(makeFrame(messageId(NodeStatus, LocalNode)), deadline(), uavcan::MonotonicTime(), 1,
		   uavcan::CanTxQueue::Persistent, 0);

	EXPECT_EQ(drainRx(), 1);
	EXPECT_TRUE(sink.frames.empty());
	EXPECT_EQ(phys.iface(0).sent.size(), 1u);
}

TEST_F(ForwardingCanDriverTest, InactiveDropsForwardedFrames)
{
	fwd.configure(-1, false, &filter, &sink, LocalNode);
	const Frame frames[] = {toFrame(makeFrame(messageId(NodeStatus, GcsNode)))};
	pumpToBus(canio, 0, frames, 1, deadline());
	EXPECT_TRUE(bus0.wire.empty());
	EXPECT_EQ(fwd.counters().dropped_blocked, 1u);
	EXPECT_EQ(drainRx(), 0);
}

TEST_F(ForwardingCanDriverTest, DeactivateClearsInjectQueue)
{
	activate();
	const Frame frames[] = {toFrame(makeFrame(messageId(NodeStatus, GcsNode)))};
	pumpToBus(canio, 0, frames, 1, deadline());
	fwd.configure(-1, false, &filter, &sink, LocalNode);
	EXPECT_EQ(drainRx(), 0);
}

// D12
TEST_F(ForwardingCanDriverTest, OtherInterfaceNotForwarded)
{
	activate(1);
	peer0.send(makeFrame(messageId(NodeStatus, TargetNode)), deadline(), 0);
	peer1.send(makeFrame(messageId(NodeStatus, 43)), deadline(), 0);
	EXPECT_EQ(drainRx(), 2);
	ASSERT_EQ(sink.frames.size(), 1u);
	EXPECT_EQ(sink.frames[0].iface, 1);

	// a forwarded frame for the wrong bus is dropped
	const Frame frames[] = {toFrame(makeFrame(messageId(NodeStatus, GcsNode)))};
	pumpToBus(canio, 0, frames, 1, deadline());
	EXPECT_TRUE(phys.iface(0).sent.empty());
	EXPECT_EQ(fwd.counters().dropped_blocked, 1u);

	pumpToBus(canio, 1, frames, 1, deadline());
	EXPECT_EQ(phys.iface(1).sent.size(), 1u);
}

TEST_F(ForwardingCanDriverTest, FilterApplies)
{
	activate();
	const uint16_t ids[] = {NodeStatus};
	filter.apply(0, ids, 1);
	peer0.send(makeFrame(messageId(NodeStatus, TargetNode)), deadline(), 0);
	peer0.send(makeFrame(messageId(1063, TargetNode)), deadline(), 0);
	EXPECT_EQ(drainRx(), 2);
	ASSERT_EQ(sink.frames.size(), 1u);
	EXPECT_EQ(sink.frames[0].frame.id, messageId(NodeStatus, TargetNode));
}

// D13
TEST_F(ForwardingCanDriverTest, InterfaceCounters)
{
	activate();
	const Frame frames[] = {
		toFrame(makeFrame(messageId(NodeStatus, GcsNode))),
		toFrame(makeFrame(serviceId(FileRead, false, TargetNode, GcsNode))),
	};
	pumpToBus(canio, 0, frames, 2, deadline());
	drainRx();

	// forwarded frames count as transmitted, injected ones as received (documented behavior)
	EXPECT_EQ(canio.getIfacePerfCounters(0).frames_tx, 2u);
	EXPECT_EQ(canio.getIfacePerfCounters(0).frames_rx, 1u);
}

// A2 / A4: arming while forwarded frames wait for a mailbox
TEST_F(ForwardingCanDriverTest, NothingReachesTheBusAfterArming)
{
	activate();
	phys.iface(0).mailboxes_free = 0;
	const Frame frames[] = {toFrame(makeFrame(serviceId(FileRead, false, TargetNode, GcsNode)))};
	EXPECT_EQ(pumpToBus(canio, 0, frames, 1, deadline()), 1); // queued in libuavcan

	activate(0, false); // armed: monitoring only
	phys.iface(0).mailboxes_free = 10;
	drainRx();

	EXPECT_TRUE(phys.iface(0).sent.empty());
	EXPECT_EQ(fwd.counters().dropped_blocked, 1u);
	EXPECT_EQ(pool.getNumUsedBlocks(), 0);

	// bus to GCS still works while armed
	peer0.send(makeFrame(messageId(NodeStatus, TargetNode)), deadline(), 0);
	drainRx();
	EXPECT_EQ(sink.frames.size(), 1u);

	// the autopilot's own traffic is unaffected
	canio.send(makeFrame(messageId(NodeStatus, LocalNode)), deadline(), uavcan::MonotonicTime(), 1,
		   uavcan::CanTxQueue::Persistent, 0);
	EXPECT_EQ(phys.iface(0).sent.size(), 1u);
}
