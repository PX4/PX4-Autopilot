/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/node_status_monitor.hpp>
#include <uavcan/protocol/node_status_provider.hpp>
#include "helpers.hpp"

static void publishNodeStatus(CanDriverMock& can, uavcan::NodeID node_id, uavcan::uint8_t status_code,
                              uavcan::uint32_t uptime_sec, uavcan::TransferID tid)
{
    uavcan::StaticTransferBuffer<100> buffer;
    uavcan::BitStream bitstream(buffer);
    uavcan::ScalarCodec codec(bitstream);

    uavcan::protocol::NodeStatus msg;
    msg.status_code = status_code;
    msg.uptime_sec = uptime_sec;

    // Manual message publication
    ASSERT_LT(0, uavcan::protocol::NodeStatus::encode(msg, codec));
    ASSERT_GE(7, buffer.getMaxWritePos());

    // DataTypeID data_type_id, TransferType transfer_type, NodeID src_node_id, NodeID dst_node_id,
    // uint_fast8_t frame_index, TransferID transfer_id, bool last_frame
    uavcan::Frame frame(uavcan::protocol::NodeStatus::DefaultDataTypeID, uavcan::TransferTypeMessageBroadcast,
                        node_id, uavcan::NodeID::Broadcast, 0, tid, true);

    ASSERT_EQ(buffer.getMaxWritePos(), frame.setPayload(buffer.getRawPtr(), buffer.getMaxWritePos()));

    uavcan::CanFrame can_frame;
    ASSERT_TRUE(frame.compile(can_frame));

    for (uint8_t i = 0; i < can.getNumIfaces(); i++)
    {
        can.ifaces.at(i).pushRx(can_frame);
    }
}


static void shortSpin(TestNode& node)
{
    ASSERT_LE(0, node.spin(uavcan::MonotonicDuration::fromMSec(10)));
}


TEST(NodeStatusMonitor, Basic)
{
    using uavcan::protocol::NodeStatus;
    using uavcan::NodeID;

    SystemClockMock clock_mock(100);
    clock_mock.monotonic_auto_advance = 1000;

    CanDriverMock can(2, clock_mock);

    TestNode node(can, clock_mock, 64);

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg1;

    uavcan::NodeStatusMonitor nsm(node);
    ASSERT_LE(0, nsm.start());

    ASSERT_LE(0, node.spin(uavcan::MonotonicDuration::fromMSec(10)));

    /*
     * Empty NSM, no nodes were registered yet
     */
    ASSERT_FALSE(nsm.findNodeWithWorstStatus().isValid());

    uavcan::NodeStatusMonitor::NodeStatus st = nsm.getNodeStatus(uavcan::NodeID(123));
    ASSERT_FALSE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_OFFLINE, st.status_code);

    /*
     * Some new status messages
     */
    publishNodeStatus(can, 10, NodeStatus::STATUS_OK, 12, 0);
    shortSpin(node);
    ASSERT_EQ(NodeID(10), nsm.findNodeWithWorstStatus());

    publishNodeStatus(can, 9,  NodeStatus::STATUS_INITIALIZING, 0, 0);
    shortSpin(node);
    ASSERT_EQ(NodeID(9), nsm.findNodeWithWorstStatus());

    publishNodeStatus(can, 11, NodeStatus::STATUS_CRITICAL, 999, 0);
    shortSpin(node);
    ASSERT_EQ(NodeID(11), nsm.findNodeWithWorstStatus());

    st = nsm.getNodeStatus(uavcan::NodeID(10));
    ASSERT_TRUE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_OK, st.status_code);

    st = nsm.getNodeStatus(uavcan::NodeID(9));
    ASSERT_TRUE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_INITIALIZING, st.status_code);

    st = nsm.getNodeStatus(uavcan::NodeID(11));
    ASSERT_TRUE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_CRITICAL, st.status_code);

    /*
     * Timeout
     */
    std::cout << "Starting timeout test, current monotime is " << clock_mock.monotonic << std::endl;

    clock_mock.advance(500000);
    shortSpin(node);
    st = nsm.getNodeStatus(uavcan::NodeID(10));
    ASSERT_TRUE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_OK, st.status_code);

    clock_mock.advance(500000);
    shortSpin(node);
    st = nsm.getNodeStatus(uavcan::NodeID(9));
    ASSERT_TRUE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_INITIALIZING, st.status_code);

    clock_mock.advance(500000);
    shortSpin(node);
    st = nsm.getNodeStatus(uavcan::NodeID(11));
    ASSERT_TRUE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_CRITICAL, st.status_code);

    /*
     * Will timeout now
     */
    clock_mock.advance(4000000);
    shortSpin(node);

    st = nsm.getNodeStatus(uavcan::NodeID(10));
    ASSERT_TRUE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_OFFLINE, st.status_code);

    st = nsm.getNodeStatus(uavcan::NodeID(9));
    ASSERT_TRUE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_OFFLINE, st.status_code);

    st = nsm.getNodeStatus(uavcan::NodeID(11));
    ASSERT_TRUE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_OFFLINE, st.status_code);

    /*
     * Recovering one node, adding two extra
     */
    publishNodeStatus(can, 11, NodeStatus::STATUS_WARNING, 999, 0);
    shortSpin(node);

    publishNodeStatus(can, 127, NodeStatus::STATUS_WARNING, 9999, 0);
    shortSpin(node);

    publishNodeStatus(can, 1, NodeStatus::STATUS_OK, 1234, 0);
    shortSpin(node);

    /*
     * Making sure OFFLINE is still worst status
     */
    ASSERT_EQ(NodeID(9), nsm.findNodeWithWorstStatus());

    /*
     * Final validation
     */
    st = nsm.getNodeStatus(uavcan::NodeID(10));
    ASSERT_TRUE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_OFFLINE, st.status_code);

    st = nsm.getNodeStatus(uavcan::NodeID(9));
    ASSERT_TRUE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_OFFLINE, st.status_code);

    st = nsm.getNodeStatus(uavcan::NodeID(11));
    ASSERT_TRUE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_WARNING, st.status_code);

    st = nsm.getNodeStatus(uavcan::NodeID(127));
    ASSERT_TRUE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_WARNING, st.status_code);

    st = nsm.getNodeStatus(uavcan::NodeID(1));
    ASSERT_TRUE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_OK, st.status_code);

    /*
     * Forgetting
     */
    nsm.forgetNode(127);
    st = nsm.getNodeStatus(uavcan::NodeID(127));
    ASSERT_FALSE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_OFFLINE, st.status_code);

    nsm.forgetNode(9);
    st = nsm.getNodeStatus(uavcan::NodeID(9));
    ASSERT_FALSE(st.known);
    ASSERT_EQ(NodeStatus::STATUS_OFFLINE, st.status_code);
}
