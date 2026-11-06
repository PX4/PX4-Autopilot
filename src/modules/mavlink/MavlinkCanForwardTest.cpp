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
 * MAVLink side of CAN forwarding, with real uORB topics.
 *
 * Test matrix rows S1-S3, S5, S8, C2, C6-C8, M1, M2, M4-M6, M9, F5, F8 (see src/lib/can_forward/TEST_MATRIX.md)
 */

#include <gtest/gtest.h>

#include "mavlink_can_forward.h"

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_command.h>

#include <vector>

namespace
{

static constexpr uint8_t GcsSys = 250;
static constexpr uint8_t GcsComp = 189; // MAV_COMP_ID_MAVCAN
static constexpr uint32_t ExtId = 0x80000000U | 0x1014D2AU;

template<typename T>
mavlink_message_t makeMessage(uint32_t msgid, uint8_t sysid, uint8_t compid, const T &payload)
{
	mavlink_message_t msg{};
	msg.msgid = msgid;
	msg.sysid = sysid;
	msg.compid = compid;
	msg.len = sizeof(T);
	memcpy(_MAV_PAYLOAD_NON_CONST(&msg), &payload, sizeof(T));
	return msg;
}

mavlink_message_t canFrame(uint8_t bus, uint32_t id, uint8_t len, uint8_t seed = 0, uint8_t sysid = GcsSys,
			   uint8_t compid = GcsComp)
{
	mavlink_can_frame_t frame{};
	frame.bus = bus;
	frame.id = id;
	frame.len = len;

	for (uint8_t i = 0; i < sizeof(frame.data); i++) { frame.data[i] = seed + i; }

	return makeMessage(MAVLINK_MSG_ID_CAN_FRAME, sysid, compid, frame);
}

mavlink_message_t canFdFrame(uint8_t bus, uint32_t id, uint8_t len)
{
	mavlink_canfd_frame_t frame{};
	frame.bus = bus;
	frame.id = id;
	frame.len = len;
	return makeMessage(MAVLINK_MSG_ID_CANFD_FRAME, GcsSys, GcsComp, frame);
}

mavlink_message_t filterModify(uint8_t bus_1based, uint8_t op, std::vector<uint16_t> ids, uint8_t sysid = GcsSys,
			       uint8_t num_ids = 255)
{
	mavlink_can_filter_modify_t filter{};
	filter.bus = bus_1based;
	filter.operation = op;
	filter.num_ids = (num_ids == 255) ? ids.size() : num_ids;

	for (size_t i = 0; i < ids.size() && i < 16; i++) { filter.ids[i] = ids[i]; }

	return makeMessage(MAVLINK_MSG_ID_CAN_FILTER_MODIFY, sysid, GcsComp, filter);
}

vehicle_command_ack_s ack(uint8_t result, uint32_t sysid = GcsSys, uint8_t compid = GcsComp)
{
	vehicle_command_ack_s a{};
	a.timestamp = hrt_absolute_time();
	a.command = vehicle_command_s::VEHICLE_CMD_CAN_FORWARD;
	a.result = result;
	a.target_system = sysid;
	a.target_component = compid;
	return a;
}

class FakeSender : public MavlinkCanForward::Sender
{
public:
	unsigned freeTxBuf() override
	{
		return (budget > 0) ? budget * MavlinkCanForward::CanFrameMessageSize : 0;
	}

	void sendCanFrame(mavlink_can_frame_t &msg, uint32_t target_system) override
	{
		sent.push_back(msg);
		targets.push_back(target_system);

		if (budget > 0 && budget != Unlimited) { budget--; }
	}

	static constexpr int Unlimited = 1000000;
	int budget{Unlimited};
	std::vector<mavlink_can_frame_t> sent;
	std::vector<uint32_t> targets;
};

} // namespace

class MavlinkCanForwardTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		// start every test from the current end of the topics
		can_forward_frames_s frames;

		while (tx_sub.update(&frames)) {}

		can_forward_filter_s filter;

		while (filter_sub.update(&filter)) {}
	}

	void bind(MavlinkCanForward &cf, float bus = 1.f)
	{
		cf.handleCommand(GcsSys, GcsComp, bus);
		cf.handleAck(ack(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED));
		ASSERT_TRUE(cf.bound());
	}

	void publishStatus(uint8_t state, uint8_t bus = 0, uint32_t sysid = GcsSys, uint8_t compid = GcsComp,
			   hrt_abstime timestamp = 0)
	{
		can_forward_status_s status{};
		status.timestamp = timestamp ? timestamp : hrt_absolute_time();
		status.state = state;
		status.bus = bus;
		status.requester_sysid = sysid;
		status.requester_compid = compid;
		status_pub.publish(status);
	}

	void publishRx(uint8_t bus, uint8_t count, uint8_t seed)
	{
		can_forward_frames_s frames{};
		frames.timestamp = hrt_absolute_time();
		frames.bus = bus;
		frames.count = count;

		for (uint8_t i = 0; i < count; i++) {
			frames.id[i] = ExtId + seed + i;
			frames.len[i] = 8;

			for (uint8_t j = 0; j < 8; j++) { frames.data[i * 8 + j] = seed + i; }
		}

		rx_pub.publish(frames);
	}

	std::vector<can_forward_frames_s> takeTx()
	{
		std::vector<can_forward_frames_s> out;
		can_forward_frames_s frames;

		while (tx_sub.update(&frames)) { out.push_back(frames); }

		return out;
	}

	uORB::Subscription tx_sub{ORB_ID(can_forward_tx)};
	uORB::Subscription filter_sub{ORB_ID(can_forward_filter)};
	uORB::Publication<can_forward_frames_s> rx_pub{ORB_ID(can_forward_rx)};
	uORB::Publication<can_forward_status_s> status_pub{ORB_ID(can_forward_status)};
};

// S1
TEST_F(MavlinkCanForwardTest, AcceptedAckBinds)
{
	MavlinkCanForward cf;
	EXPECT_FALSE(cf.bound());
	bind(cf);
}

// S3 / A1: a refused request does not bind
TEST_F(MavlinkCanForwardTest, RefusedAckDoesNotBind)
{
	MavlinkCanForward cf;

	for (uint8_t result : {
		     vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED,
		     vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
		     vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED
	     }) {
		cf.handleCommand(GcsSys, GcsComp, 1.f);
		cf.handleAck(ack(result));
		EXPECT_FALSE(cf.bound());
	}
}

TEST_F(MavlinkCanForwardTest, AckForSomeoneElseIsIgnored)
{
	MavlinkCanForward cf;
	cf.handleCommand(GcsSys, GcsComp, 1.f);
	cf.handleAck(ack(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 42, 1));
	EXPECT_FALSE(cf.bound());

	// and without a pending request on this instance
	MavlinkCanForward other;
	other.handleAck(ack(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED));
	EXPECT_FALSE(other.bound());
}

// S2
TEST_F(MavlinkCanForwardTest, StopUnbinds)
{
	MavlinkCanForward cf;
	bind(cf);
	cf.handleCommand(GcsSys, GcsComp, 0.f);
	cf.handleAck(ack(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED));
	EXPECT_FALSE(cf.bound());
}

// S5: the driver ended the session (timeout)
TEST_F(MavlinkCanForwardTest, IdleStatusUnbinds)
{
	MavlinkCanForward cf;
	FakeSender sender;
	bind(cf);

	publishStatus(can_forward_status_s::STATE_ACTIVE);
	cf.update(sender);
	EXPECT_TRUE(cf.bound());

	publishStatus(can_forward_status_s::STATE_IDLE);
	cf.update(sender);
	EXPECT_FALSE(cf.bound());
}

TEST_F(MavlinkCanForwardTest, StatusFromBeforeBindingIsIgnored)
{
	MavlinkCanForward cf;
	FakeSender sender;
	const hrt_abstime before = hrt_absolute_time();
	px4_usleep(1000);
	bind(cf);

	publishStatus(can_forward_status_s::STATE_IDLE, 0, GcsSys, GcsComp, before);
	cf.update(sender);
	EXPECT_TRUE(cf.bound());
}

TEST_F(MavlinkCanForwardTest, MonitorOnlyKeepsBinding)
{
	MavlinkCanForward cf;
	FakeSender sender;
	bind(cf);
	publishStatus(can_forward_status_s::STATE_MONITOR_ONLY);
	cf.update(sender);
	EXPECT_TRUE(cf.bound());
}

// S8: another client took over the session in the driver
TEST_F(MavlinkCanForwardTest, TakeoverByOtherClientUnbinds)
{
	MavlinkCanForward cf;
	FakeSender sender;
	bind(cf);
	publishStatus(can_forward_status_s::STATE_ACTIVE, 0, 42, 1);
	cf.update(sender);
	EXPECT_FALSE(cf.bound());
}

// M6: only the instance with the latest request gets the frames
TEST_F(MavlinkCanForwardTest, LatestInstanceWins)
{
	MavlinkCanForward a;
	MavlinkCanForward b;
	a.handleCommand(GcsSys, GcsComp, 1.f);
	b.handleCommand(GcsSys, GcsComp, 1.f);

	// both instances see the same ack
	a.handleAck(ack(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED));
	b.handleAck(ack(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED));
	EXPECT_FALSE(a.bound());
	EXPECT_TRUE(b.bound());

	publishRx(0, 3, 1);
	FakeSender sa;
	FakeSender sb;
	a.update(sa);
	b.update(sb);
	EXPECT_TRUE(sa.sent.empty());
	EXPECT_EQ(sb.sent.size(), 3u);
}

// M2
TEST_F(MavlinkCanForwardTest, FramesWithoutSessionAreDropped)
{
	MavlinkCanForward cf;
	cf.handleMessage(canFrame(0, ExtId, 8));
	cf.flushToBus(hrt_absolute_time());
	EXPECT_TRUE(takeTx().empty());
	EXPECT_EQ(cf.counters().dropped_not_owner, 1u);
}

// M1
TEST_F(MavlinkCanForwardTest, FramesFromOtherClientsAreDropped)
{
	MavlinkCanForward cf;
	bind(cf);
	cf.handleMessage(canFrame(0, ExtId, 8, 0, 42, GcsComp));
	cf.handleMessage(canFrame(0, ExtId, 8, 0, GcsSys, 1));
	cf.flushToBus(hrt_absolute_time());
	EXPECT_TRUE(takeTx().empty());
	EXPECT_EQ(cf.counters().dropped_not_owner, 2u);
}

// M9: frames are batched and published per receive buffer
TEST_F(MavlinkCanForwardTest, FramesAreBatched)
{
	MavlinkCanForward cf;
	bind(cf);

	cf.handleMessage(canFrame(0, ExtId, 3, 10));
	EXPECT_TRUE(takeTx().empty()) << "not before the receive buffer is done";

	cf.flushToBus(hrt_absolute_time());
	auto batches = takeTx();
	ASSERT_EQ(batches.size(), 1u);
	EXPECT_EQ(batches[0].count, 1);
	EXPECT_EQ(batches[0].bus, 0);
	EXPECT_EQ(batches[0].id[0], ExtId);
	EXPECT_EQ(batches[0].len[0], 3);
	EXPECT_EQ(batches[0].data[0], 10);
	EXPECT_EQ(batches[0].data[2], 12);
	EXPECT_EQ(batches[0].data[3], 0) << "bytes past len are zero";

	// a full batch goes out right away, the rest with the flush
	for (uint8_t i = 0; i < 20; i++) {
		cf.handleMessage(canFrame(0, ExtId, 8, i));
	}

	batches = takeTx();
	ASSERT_EQ(batches.size(), 1u);
	EXPECT_EQ(batches[0].count, 16);

	cf.flushToBus(hrt_absolute_time());
	batches = takeTx();
	ASSERT_EQ(batches.size(), 1u);
	EXPECT_EQ(batches[0].count, 4);
	EXPECT_EQ(batches[0].data[0], 16) << "order kept across batches";

	EXPECT_EQ(cf.counters().frames_from_gcs, 21u);

	// flushing with nothing collected publishes nothing
	cf.flushToBus(hrt_absolute_time());
	EXPECT_TRUE(takeTx().empty());
}

// C2 / C6 / C7 / C8
TEST_F(MavlinkCanForwardTest, InvalidFramesAreDropped)
{
	MavlinkCanForward cf;
	bind(cf, 2.f); // bus index 1

	cf.handleMessage(canFrame(0, ExtId, 8));          // wrong bus
	cf.handleMessage(canFrame(1, ExtId, 9));          // too long
	cf.handleMessage(canFrame(1, ExtId | (1U << 30), 0)); // RTR
	cf.handleMessage(canFrame(1, 0x800, 1));          // standard ID too large
	cf.handleMessage(canFdFrame(1, ExtId, 12));       // CAN FD payload
	cf.handleMessage(canFdFrame(1, ExtId, 64));
	cf.flushToBus(hrt_absolute_time());
	EXPECT_TRUE(takeTx().empty());
	EXPECT_EQ(cf.counters().dropped_invalid, 6u);

	cf.handleMessage(canFdFrame(1, ExtId, 8));        // classic payload in CANFD_FRAME
	cf.handleMessage(canFrame(1, 0x7FF, 0));          // standard frame, empty
	cf.flushToBus(hrt_absolute_time());
	auto batches = takeTx();
	ASSERT_EQ(batches.size(), 1u);
	EXPECT_EQ(batches[0].count, 2);
	EXPECT_EQ(batches[0].bus, 1);
}

TEST_F(MavlinkCanForwardTest, BusFollowsStatus)
{
	MavlinkCanForward cf;
	FakeSender sender;
	bind(cf, 1.f);
	publishStatus(can_forward_status_s::STATE_ACTIVE, 1);
	cf.update(sender);

	cf.handleMessage(canFrame(0, ExtId, 8));
	cf.handleMessage(canFrame(1, ExtId, 8));
	cf.flushToBus(hrt_absolute_time());
	auto batches = takeTx();
	ASSERT_EQ(batches.size(), 1u);
	EXPECT_EQ(batches[0].bus, 1);
	EXPECT_EQ(batches[0].count, 1);
}

// filter: F5, F8 and the filter sent right after the request, before the ack
TEST_F(MavlinkCanForwardTest, Filter)
{
	MavlinkCanForward cf;
	can_forward_filter_s filter;

	// from a stranger
	cf.handleMessage(filterModify(1, 0, {341}, 42));
	EXPECT_FALSE(filter_sub.update(&filter));

	// pending request, not acked yet
	cf.handleCommand(GcsSys, GcsComp, 1.f);
	cf.handleMessage(filterModify(1, 0, {0, 1, 40, 48, 341}));
	ASSERT_TRUE(filter_sub.update(&filter));
	EXPECT_EQ(filter.bus, 0);
	EXPECT_EQ(filter.operation, 0);
	EXPECT_EQ(filter.num_ids, 5);
	EXPECT_EQ(filter.ids[4], 341);

	// invalid: bus 0 (1-based field), too many IDs
	cf.handleMessage(filterModify(0, 0, {1}));
	cf.handleMessage(filterModify(1, 0, {1}, GcsSys, 17));
	EXPECT_FALSE(filter_sub.update(&filter));
	EXPECT_EQ(cf.counters().dropped_invalid, 2u);

	// the operation is passed on, the driver validates it
	cf.handleMessage(filterModify(2, 7, {1}));
	ASSERT_TRUE(filter_sub.update(&filter));
	EXPECT_EQ(filter.bus, 1);
	EXPECT_EQ(filter.operation, 7);
}

TEST_F(MavlinkCanForwardTest, FramesToGcs)
{
	MavlinkCanForward cf;
	FakeSender sender;
	bind(cf);

	publishRx(0, 5, 20);
	cf.update(sender);

	ASSERT_EQ(sender.sent.size(), 5u);

	for (size_t i = 0; i < 5; i++) {
		EXPECT_EQ(sender.targets[i], GcsSys);
		EXPECT_EQ(sender.sent[i].target_component, GcsComp);
		EXPECT_EQ(sender.sent[i].bus, 0);
		EXPECT_EQ(sender.sent[i].id, ExtId + 20 + i);
		EXPECT_EQ(sender.sent[i].len, 8);
		EXPECT_EQ(sender.sent[i].data[7], 20 + i);
	}

	EXPECT_EQ(cf.counters().frames_to_gcs, 5u);
}

// M4: a full link pauses in the middle of a batch and resumes at the same frame
TEST_F(MavlinkCanForwardTest, FullLinkResumesWithoutGapsOrDuplicates)
{
	MavlinkCanForward cf;
	FakeSender sender;
	bind(cf);

	publishRx(0, 16, 0);
	publishRx(0, 16, 16);

	sender.budget = 5;
	cf.update(sender);
	EXPECT_EQ(sender.sent.size(), 5u);

	sender.budget = 0;
	cf.update(sender);
	EXPECT_EQ(sender.sent.size(), 5u);

	sender.budget = 20;
	cf.update(sender);
	EXPECT_EQ(sender.sent.size(), 25u);

	sender.budget = FakeSender::Unlimited;
	cf.update(sender);
	ASSERT_EQ(sender.sent.size(), 32u);

	for (uint32_t i = 0; i < 32; i++) {
		EXPECT_EQ(sender.sent[i].id, ExtId + i) << i;
	}
}

// M5
TEST_F(MavlinkCanForwardTest, RxOverrunIsCounted)
{
	MavlinkCanForward cf;
	FakeSender sender;
	bind(cf);
	cf.update(sender);

	for (uint8_t i = 0; i < can_forward_frames_s::ORB_QUEUE_LENGTH + 3; i++) {
		publishRx(0, 1, i);
	}

	cf.update(sender);
	EXPECT_EQ(sender.sent.size(), can_forward_frames_s::ORB_QUEUE_LENGTH);
	EXPECT_EQ(cf.counters().dropped_to_gcs, 3u);
}

TEST_F(MavlinkCanForwardTest, NothingSentWhileUnboundAndNoOldBurstLater)
{
	MavlinkCanForward cf;
	FakeSender sender;
	publishRx(0, 4, 0);
	cf.update(sender);
	EXPECT_TRUE(sender.sent.empty());

	bind(cf);
	cf.update(sender);
	EXPECT_TRUE(sender.sent.empty()) << "frames from before the binding are not sent";

	publishRx(0, 2, 8);
	cf.update(sender);
	EXPECT_EQ(sender.sent.size(), 2u);
}
