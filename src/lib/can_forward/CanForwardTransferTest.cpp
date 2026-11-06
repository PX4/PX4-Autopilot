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
 * End to end through the MAVLink tunnel model, at transfer level, with three real libuavcan
 * dispatchers:
 *
 *   GCS node 127 --(tunnel: frames to/from GCS)--> autopilot node 10 (ForwardingCanDriver) --(bus)--> target node 42
 *
 * The tunnel stands in for MAVLink CAN_FRAME: frames the GCS dispatcher transmits are handed to
 * pumpToBus() like the uavcan driver does with frames from the can_forward_tx topic, frames the
 * IFrameSink receives are delivered to the GCS dispatcher.
 *
 * Test matrix rows E1, E2, E4, E5, E15, F9, A5 (see TEST_MATRIX.md)
 */

#include <gtest/gtest.h>

#include "ForwardingCanDriver.hpp"
#include "CanForwardTestHelpers.hpp"
#include "FakeCan.hpp"

#include <uavcan/dynamic_memory.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>
#include <uavcan/protocol/file/Read.hpp>
#include <uavcan/transport/dispatcher.hpp>
#include <uavcan/transport/transfer_listener.hpp>
#include <uavcan/transport/transfer_sender.hpp>

#include <deque>
#include <memory>
#include <vector>

using namespace can_forward;
using namespace can_forward_test;

namespace
{

static constexpr uint8_t AutopilotNode = 10;
static constexpr uint8_t TargetNode = 42;
static constexpr uint8_t GcsNode = 127;

template<typename T>
uavcan::DataTypeDescriptor descriptor(uavcan::DataTypeKind kind)
{
	return uavcan::DataTypeDescriptor(kind, T::DefaultDataTypeID, T::getDataTypeSignature(), T::getDataTypeFullName());
}

struct Received {
	uint8_t src;
	uint8_t tid;
	std::vector<uint8_t> payload;
};

class RecordingListener : public uavcan::TransferListener
{
public:
	RecordingListener(uavcan::Dispatcher &dispatcher, const uavcan::DataTypeDescriptor &type, uavcan::IPoolAllocator &pool) :
		uavcan::TransferListener(dispatcher.getTransferPerfCounter(), type, 512, pool) {}

	void handleIncomingTransfer(uavcan::IncomingTransfer &transfer) override
	{
		Received r;
		r.src = transfer.getSrcNodeID().get();
		r.tid = transfer.getTransferID().get();
		uint8_t buf[512];
		const int len = transfer.read(0, buf, sizeof(buf));

		if (len > 0) {
			r.payload.assign(buf, buf + len);
		}

		received.push_back(r);
	}

	std::vector<Received> received;
};

/// Frames from the autopilot to the GCS
class TunnelSink : public IFrameSink
{
public:
	explicit TunnelSink(FakeIface &gcs_rx) : _gcs_rx(gcs_rx) {}

	void onFrameToGcs(uint8_t, const uavcan::CanFrame &frame) override
	{
		frames_to_gcs++;

		if (!link_down) {
			_gcs_rx.receiveFromBus(frame);
		}
	}

	bool link_down{false};
	unsigned frames_to_gcs{0};

private:
	FakeIface &_gcs_rx;
};

using Pool = uavcan::PoolAllocator<uavcan::MemPoolBlockSize * 1024, uavcan::MemPoolBlockSize>;

} // namespace

class CanForwardTransferTest : public ::testing::Test
{
protected:
	CanForwardTransferTest() :
		autopilot_phys(clock),
		target_phys(clock),
		gcs_phys(clock),
		fwd(autopilot_phys, clock),
		sink(gcs_phys.iface(0)),
		autopilot(fwd, autopilot_pool, clock),
		target(target_phys, target_pool, clock),
		gcs(gcs_phys, gcs_pool, clock)
	{
		autopilot_phys.iface(0).setBus(&bus);
		target_phys.iface(0).setBus(&bus);
		// the GCS dispatcher is on no bus: what it sends is picked up from its iface by tunnelToBus()

		autopilot.setNodeID(AutopilotNode);
		target.setNodeID(TargetNode);
		gcs.setNodeID(GcsNode);

		fwd.configure(0, true, &filter, &sink, AutopilotNode);
	}

	/// one scheduling round: every node spins, frames cross the tunnel
	void step()
	{
		autopilot.spinOnce();
		target.spinOnce();
		gcs.spinOnce();

		// GCS -> MAVLink -> autopilot
		auto &gcs_sent = gcs_phys.iface(0).sent;

		while (tunnel_index < gcs_sent.size()) {
			tunnel_queue.push_back(toFrame(gcs_sent[tunnel_index++].frame));
		}

		while (!tunnel_queue.empty()) {
			const Frame f = tunnel_queue.front();

			if (pumpToBus(autopilot.getCanIOManager(), 0, &f, 1, deadline(1000)) == 0) {
				break;
			}

			tunnel_queue.pop_front();
		}

		clock.advance(500);
	}

	void run(int steps)
	{
		for (int i = 0; i < steps; i++) { step(); }
	}

	uavcan::MonotonicTime deadline(uint64_t ms) const
	{
		return clock.getMonotonic() + uavcan::MonotonicDuration::fromMSec(ms);
	}

	static Frame toFrame(const uavcan::CanFrame &f)
	{
		Frame out{};
		out.id = f.id;
		out.len = f.dlc;
		memcpy(out.data, f.data, f.dlc);
		return out;
	}

	int send(uavcan::Dispatcher &from, const uavcan::DataTypeDescriptor &type, const std::vector<uint8_t> &payload,
		 uavcan::TransferType transfer_type, uint8_t dst, int tid = -1)
	{
		uavcan::TransferSender sender(from, type, uavcan::CanTxQueue::Persistent);

		if (tid >= 0) {
			return sender.send(payload.data(), payload.size(), deadline(1000), uavcan::MonotonicTime(), transfer_type,
					   uavcan::NodeID(dst), uavcan::TransferID(static_cast<uint8_t>(tid)));
		}

		return sender.send(payload.data(), payload.size(), deadline(1000), uavcan::MonotonicTime(), transfer_type,
				   uavcan::NodeID(dst));
	}

	static std::vector<uint8_t> pattern(size_t len, uint8_t seed)
	{
		std::vector<uint8_t> v(len);

		for (size_t i = 0; i < len; i++) { v[i] = static_cast<uint8_t>(seed + i * 7); }

		return v;
	}

	FakeClock clock;
	FakeBus bus;
	FakeCanDriver autopilot_phys;
	FakeCanDriver target_phys;
	FakeCanDriver gcs_phys;
	ForwardingCanDriver fwd;
	CanForwardFilter filter;
	TunnelSink sink;
	Pool autopilot_pool;
	Pool target_pool;
	Pool gcs_pool;
	uavcan::Dispatcher autopilot;
	uavcan::Dispatcher target;
	uavcan::Dispatcher gcs;

	size_t tunnel_index{0};
	std::deque<Frame> tunnel_queue;

	const uavcan::DataTypeDescriptor read_type{descriptor<uavcan::protocol::file::Read>(uavcan::DataTypeKindService)};
	const uavcan::DataTypeDescriptor info_type{descriptor<uavcan::protocol::GetNodeInfo>(uavcan::DataTypeKindService)};
	const uavcan::DataTypeDescriptor begin_type{descriptor<uavcan::protocol::file::BeginFirmwareUpdate>(uavcan::DataTypeKindService)};
	const uavcan::DataTypeDescriptor status_type{descriptor<uavcan::protocol::NodeStatus>(uavcan::DataTypeKindMessage)};
};

// E1: the GCS sees the target's NodeStatus
TEST_F(CanForwardTransferTest, DiscoveryThroughTunnel)
{
	RecordingListener gcs_status(gcs, status_type, gcs_pool);
	ASSERT_TRUE(gcs.registerMessageListener(&gcs_status));

	ASSERT_GT(send(target, status_type, pattern(7, 1), uavcan::TransferTypeMessageBroadcast, 0), 0);
	run(10);

	ASSERT_EQ(gcs_status.received.size(), 1u);
	EXPECT_EQ(gcs_status.received[0].src, TargetNode);
	EXPECT_EQ(gcs_status.received[0].payload, pattern(7, 1));
}

// E2: the GCS talks to the autopilot's own node
TEST_F(CanForwardTransferTest, GetNodeInfoFromAutopilot)
{
	RecordingListener autopilot_server(autopilot, info_type, autopilot_pool);
	RecordingListener gcs_client(gcs, info_type, gcs_pool);
	RecordingListener target_server(target, info_type, target_pool);
	ASSERT_TRUE(autopilot.registerServiceRequestListener(&autopilot_server));
	ASSERT_TRUE(gcs.registerServiceResponseListener(&gcs_client));
	ASSERT_TRUE(target.registerServiceRequestListener(&target_server));

	ASSERT_GT(send(gcs, info_type, {}, uavcan::TransferTypeServiceRequest, AutopilotNode), 0);
	run(10);

	ASSERT_EQ(autopilot_server.received.size(), 1u);
	EXPECT_EQ(autopilot_server.received[0].src, GcsNode);
	EXPECT_TRUE(target_server.received.empty());

	// multi-frame response (like a real GetNodeInfo with a name) goes back through the mirror path
	const std::vector<uint8_t> info = pattern(60, 3);
	ASSERT_GT(send(autopilot, info_type, info, uavcan::TransferTypeServiceResponse, GcsNode,
		       autopilot_server.received[0].tid), 0);
	run(20);

	ASSERT_EQ(gcs_client.received.size(), 1u);
	EXPECT_EQ(gcs_client.received[0].src, AutopilotNode);
	EXPECT_EQ(gcs_client.received[0].payload, info);
}

// E4 / E5: the core of a firmware update: the node pulls the image from the GCS with file.Read
class FileReadSizes : public CanForwardTransferTest, public ::testing::WithParamInterface<size_t> {};

TEST_P(FileReadSizes, ReadResponseArrivesIntact)
{
	RecordingListener gcs_server(gcs, read_type, gcs_pool);
	RecordingListener target_client(target, read_type, target_pool);
	ASSERT_TRUE(gcs.registerServiceRequestListener(&gcs_server));
	ASSERT_TRUE(target.registerServiceResponseListener(&target_client));

	ASSERT_GT(send(target, read_type, pattern(12, 9), uavcan::TransferTypeServiceRequest, GcsNode), 0);
	run(10);
	ASSERT_EQ(gcs_server.received.size(), 1u);
	EXPECT_EQ(gcs_server.received[0].src, TargetNode);
	EXPECT_EQ(gcs_server.received[0].payload, pattern(12, 9));

	const std::vector<uint8_t> chunk = pattern(GetParam(), 0x5A);
	ASSERT_GT(send(gcs, read_type, chunk, uavcan::TransferTypeServiceResponse, TargetNode,
		       gcs_server.received[0].tid), 0);
	run(200);

	ASSERT_EQ(target_client.received.size(), 1u);
	EXPECT_EQ(target_client.received[0].src, GcsNode);
	EXPECT_EQ(target_client.received[0].payload, chunk);
	EXPECT_EQ(fwd.counters().dropped_blocked, 0u);
}

INSTANTIATE_TEST_SUITE_P(Sizes, FileReadSizes, ::testing::Values(0, 1, 6, 7, 8, 13, 14, 255, 256, 258, 300));

// E5: four pipelined reads answered back to back (~150 frames) with a slow mailbox
TEST_F(CanForwardTransferTest, PipelinedReadsWithSlowMailbox)
{
	RecordingListener gcs_server(gcs, read_type, gcs_pool);
	RecordingListener target_client(target, read_type, target_pool);
	ASSERT_TRUE(gcs.registerServiceRequestListener(&gcs_server));
	ASSERT_TRUE(target.registerServiceResponseListener(&target_client));

	for (uint8_t i = 0; i < 4; i++) {
		ASSERT_GT(send(target, read_type, pattern(12, i), uavcan::TransferTypeServiceRequest, GcsNode), 0);
	}

	run(20);
	ASSERT_EQ(gcs_server.received.size(), 4u);

	for (const auto &request : gcs_server.received) {
		ASSERT_GT(send(gcs, read_type, pattern(256, request.tid), uavcan::TransferTypeServiceResponse, TargetNode,
			       request.tid), 0);
	}

	// the autopilot hardware takes one frame per step
	for (int i = 0; i < 1000 && target_client.received.size() < 4; i++) {
		autopilot_phys.iface(0).mailboxes_free = 1;
		step();
	}

	ASSERT_EQ(target_client.received.size(), 4u);

	for (const auto &response : target_client.received) {
		EXPECT_EQ(response.payload, pattern(256, response.tid));
	}

	EXPECT_EQ(autopilot.getCanIOManager().getTxQueuePerfCounters(0).rejected_frames, 0u);
}

// F9: the minimal firmware update filter still lets the whole update through, other traffic not
TEST_F(CanForwardTransferTest, FirmwareUpdateFilter)
{
	const uint16_t ids[] = {0, GetNodeInfo, BeginFirmwareUpdate, FileGetInfo, FileRead, NodeStatus};
	ASSERT_EQ(filter.apply(0, ids, 6), CanForwardFilter::Result::Ok);

	RecordingListener gcs_begin(gcs, begin_type, gcs_pool);
	RecordingListener gcs_read(gcs, read_type, gcs_pool);
	RecordingListener gcs_status(gcs, status_type, gcs_pool);
	RecordingListener target_begin(target, begin_type, target_pool);
	ASSERT_TRUE(gcs.registerServiceResponseListener(&gcs_begin));
	ASSERT_TRUE(gcs.registerServiceRequestListener(&gcs_read));
	ASSERT_TRUE(gcs.registerMessageListener(&gcs_status));
	ASSERT_TRUE(target.registerServiceRequestListener(&target_begin));

	// GCS: BeginFirmwareUpdate to the target
	ASSERT_GT(send(gcs, begin_type, pattern(20, 1), uavcan::TransferTypeServiceRequest, TargetNode), 0);
	run(20);
	ASSERT_EQ(target_begin.received.size(), 1u);
	EXPECT_EQ(target_begin.received[0].src, GcsNode);

	// target: response, then a read request and its NodeStatus
	ASSERT_GT(send(target, begin_type, {0}, uavcan::TransferTypeServiceResponse, GcsNode, target_begin.received[0].tid), 0);
	ASSERT_GT(send(target, read_type, pattern(12, 2), uavcan::TransferTypeServiceRequest, GcsNode), 0);
	ASSERT_GT(send(target, status_type, pattern(7, 3), uavcan::TransferTypeMessageBroadcast, 0), 0);

	// unrelated traffic is filtered out (e.g. a param GetSet response from another node)
	const unsigned before = sink.frames_to_gcs;
	target_phys.iface(0).send(uavcan::CanFrame(messageId(1063, TargetNode), pattern(8, 0).data(), 8), deadline(10), 0);
	run(20);

	EXPECT_EQ(gcs_begin.received.size(), 1u);
	EXPECT_EQ(gcs_read.received.size(), 1u);
	EXPECT_EQ(gcs_status.received.size(), 1u);
	EXPECT_EQ(sink.frames_to_gcs - before, 1u + 2u + 1u) << "begin response (1 frame), read request (2), status (1)";
}

// E15: after BeginFirmwareUpdate a bootloader may come up without a node ID
TEST_F(CanForwardTransferTest, AnonymousBootloaderFramesReachGcs)
{
	const uint16_t ids[] = {0, NodeStatus};
	filter.apply(0, ids, 2);

	// anonymous allocation request: single frame message with source node 0
	const uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0xC0};
	target_phys.iface(0).send(uavcan::CanFrame(anonymousId(0x1234, 1), data, 7), deadline(10), 0);
	// and a frame of a type that is not in the filter
	target_phys.iface(0).send(uavcan::CanFrame(messageId(1063, TargetNode), data, 7), deadline(10), 0);
	run(5);

	EXPECT_EQ(sink.frames_to_gcs, 1u);
}

// A5: arming in the middle of a read response, nothing more reaches the bus
TEST_F(CanForwardTransferTest, ArmingDuringTransferStopsInjection)
{
	RecordingListener gcs_server(gcs, read_type, gcs_pool);
	RecordingListener target_client(target, read_type, target_pool);
	ASSERT_TRUE(gcs.registerServiceRequestListener(&gcs_server));
	ASSERT_TRUE(target.registerServiceResponseListener(&target_client));

	ASSERT_GT(send(target, read_type, pattern(12, 1), uavcan::TransferTypeServiceRequest, GcsNode), 0);
	run(10);
	ASSERT_EQ(gcs_server.received.size(), 1u);
	ASSERT_GT(send(gcs, read_type, pattern(256, 1), uavcan::TransferTypeServiceResponse, TargetNode,
		       gcs_server.received[0].tid), 0);

	// a few frames make it out, then the vehicle is armed
	for (int i = 0; i < 5; i++) {
		autopilot_phys.iface(0).mailboxes_free = 1;
		step();
	}

	const size_t sent_before_arming = autopilot_phys.iface(0).sent.size();
	EXPECT_GT(sent_before_arming, 0u);

	fwd.configure(0, false, &filter, &sink, AutopilotNode);
	autopilot_phys.iface(0).mailboxes_free = 1000;
	run(200);

	EXPECT_EQ(autopilot_phys.iface(0).sent.size(), sent_before_arming);
	EXPECT_TRUE(target_client.received.empty());
	EXPECT_GT(fwd.counters().dropped_blocked, 0u);

	// monitoring still works: the target's next request reaches the GCS
	ASSERT_GT(send(target, read_type, pattern(12, 2), uavcan::TransferTypeServiceRequest, GcsNode), 0);
	run(10);
	EXPECT_EQ(gcs_server.received.size(), 2u);
}

// E8: link loss in the middle of a transfer, the autopilot keeps running and a retry works
TEST_F(CanForwardTransferTest, LinkLossAndRetry)
{
	RecordingListener gcs_server(gcs, read_type, gcs_pool);
	RecordingListener target_client(target, read_type, target_pool);
	ASSERT_TRUE(gcs.registerServiceRequestListener(&gcs_server));
	ASSERT_TRUE(target.registerServiceResponseListener(&target_client));

	sink.link_down = true;
	ASSERT_GT(send(target, read_type, pattern(12, 1), uavcan::TransferTypeServiceRequest, GcsNode), 0);
	run(10);
	EXPECT_TRUE(gcs_server.received.empty());

	sink.link_down = false;
	clock.advance(1'000'000);
	ASSERT_GT(send(target, read_type, pattern(12, 1), uavcan::TransferTypeServiceRequest, GcsNode), 0);
	run(10);
	ASSERT_EQ(gcs_server.received.size(), 1u);

	ASSERT_GT(send(gcs, read_type, pattern(256, 4), uavcan::TransferTypeServiceResponse, TargetNode,
		       gcs_server.received[0].tid), 0);
	run(200);
	ASSERT_EQ(target_client.received.size(), 1u);
	EXPECT_EQ(target_client.received[0].payload, pattern(256, 4));
}
