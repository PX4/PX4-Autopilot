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
    uavcan::protocol::NodeStatus msg;
    msg.status_code = status_code;
    msg.uptime_sec = uptime_sec;
    emulateSingleFrameBroadcastTransfer(can, node_id, msg, tid);
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
