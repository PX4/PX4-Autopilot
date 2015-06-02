/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include "helpers.hpp"


TEST(DynamicNodeIDClient, Basic)
{
    // Node A is Allocator, Node B is Allocatee
    InterlinkedTestNodesWithSysClock nodes(uavcan::NodeID(10), uavcan::NodeID::Broadcast);

    uavcan::DynamicNodeIDClient dnidac(nodes.b);

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::dynamic_node_id::Allocation> _reg1;
    (void)_reg1;

    /*
     * Client initialization
     */
    uavcan::protocol::HardwareVersion hwver;

    ASSERT_LE(-uavcan::ErrInvalidParam, dnidac.start(hwver));  // Empty hardware version is not allowed

    for (uavcan::uint8_t i = 0; i < hwver.unique_id.size(); i++)
    {
        hwver.unique_id[i] = i;
    }

    ASSERT_LE(-uavcan::ErrInvalidParam, dnidac.start(hwver, uavcan::NodeID()));

    const uavcan::NodeID PreferredNodeID = 42;
    ASSERT_LE(0, dnidac.start(hwver, PreferredNodeID));

    ASSERT_FALSE(dnidac.getAllocatedNodeID().isValid());
    ASSERT_FALSE(dnidac.getAllocatorNodeID().isValid());
    ASSERT_FALSE(dnidac.isAllocationComplete());

    /*
     * Subscriber (server emulation)
     */
    SubscriberWithCollector<uavcan::protocol::dynamic_node_id::Allocation> dynid_sub(nodes.a);
    ASSERT_LE(0, dynid_sub.start());
    dynid_sub.subscriber.allowAnonymousTransfers();

    /*
     * Monitoring requests at 1Hz
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1100));
    ASSERT_TRUE(dynid_sub.collector.msg.get());
    std::cout << "First-stage request:\n" << *dynid_sub.collector.msg << std::endl;
    ASSERT_EQ(PreferredNodeID.get(), dynid_sub.collector.msg->node_id);
    ASSERT_TRUE(dynid_sub.collector.msg->first_part_of_unique_id);
    ASSERT_TRUE(uavcan::equal(dynid_sub.collector.msg->unique_id.begin(),
                              dynid_sub.collector.msg->unique_id.end(),
                              hwver.unique_id.begin()));
    dynid_sub.collector.msg.reset();

    // Rate validation
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(500));
    ASSERT_FALSE(dynid_sub.collector.msg.get());

    // Second - rate is 1 Hz
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(500));
    ASSERT_TRUE(dynid_sub.collector.msg.get());
    dynid_sub.collector.msg.reset();

    ASSERT_FALSE(dnidac.getAllocatedNodeID().isValid());
    ASSERT_FALSE(dnidac.getAllocatorNodeID().isValid());
    ASSERT_FALSE(dnidac.isAllocationComplete());

    /*
     * Publisher (server emulation)
     */
    uavcan::Publisher<uavcan::protocol::dynamic_node_id::Allocation> dynid_pub(nodes.a);
    ASSERT_LE(0, dynid_pub.init());

    /*
     * Sending some some Allocation messages - the timer will keep restarting
     */
    for (int i = 0; i < 10; i++)
    {
        uavcan::protocol::dynamic_node_id::Allocation msg;  // Contents of the message doesn't matter
        ASSERT_LE(0, dynid_pub.broadcast(msg));
        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(210));
        ASSERT_FALSE(dynid_sub.collector.msg.get());
    }

    /*
     * Responding with partially matching unique ID - the client will respond with second-stage request immediately
     */
    {
        uavcan::protocol::dynamic_node_id::Allocation msg;
        msg.unique_id.resize(7);
        uavcan::copy(hwver.unique_id.begin(), hwver.unique_id.begin() + 7, msg.unique_id.begin());

        std::cout << "First-stage offer:\n" << msg << std::endl;

        ASSERT_FALSE(dynid_sub.collector.msg.get());
        ASSERT_LE(0, dynid_pub.broadcast(msg));
        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(100));

        ASSERT_TRUE(dynid_sub.collector.msg.get());
        std::cout << "Second-stage request:\n" << *dynid_sub.collector.msg << std::endl;
        ASSERT_EQ(PreferredNodeID.get(), dynid_sub.collector.msg->node_id);
        ASSERT_FALSE(dynid_sub.collector.msg->first_part_of_unique_id);
        ASSERT_TRUE(uavcan::equal(dynid_sub.collector.msg->unique_id.begin(),
                                  dynid_sub.collector.msg->unique_id.end(),
                                  hwver.unique_id.begin() + 7));
        dynid_sub.collector.msg.reset();
    }

    /*
     * Responding with second-stage offer, expecting the last request back
     */
    {
        uavcan::protocol::dynamic_node_id::Allocation msg;
        msg.unique_id.resize(14);
        uavcan::copy(hwver.unique_id.begin(), hwver.unique_id.begin() + 14, msg.unique_id.begin());

        std::cout << "Second-stage offer:\n" << msg << std::endl;

        ASSERT_FALSE(dynid_sub.collector.msg.get());
        ASSERT_LE(0, dynid_pub.broadcast(msg));
        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(100));

        ASSERT_TRUE(dynid_sub.collector.msg.get());
        std::cout << "Last request:\n" << *dynid_sub.collector.msg << std::endl;
        ASSERT_EQ(PreferredNodeID.get(), dynid_sub.collector.msg->node_id);
        ASSERT_FALSE(dynid_sub.collector.msg->first_part_of_unique_id);
        ASSERT_TRUE(uavcan::equal(dynid_sub.collector.msg->unique_id.begin(),
                                  dynid_sub.collector.msg->unique_id.end(),
                                  hwver.unique_id.begin() + 14));
        dynid_sub.collector.msg.reset();
    }

    ASSERT_FALSE(dnidac.getAllocatedNodeID().isValid());
    ASSERT_FALSE(dnidac.getAllocatorNodeID().isValid());
    ASSERT_FALSE(dnidac.isAllocationComplete());

    /*
     * Now we have full unique ID for this client received, and it is possible to grant allocation
     */
    {
        uavcan::protocol::dynamic_node_id::Allocation msg;
        msg.unique_id.resize(16);
        msg.node_id = 72;
        uavcan::copy(hwver.unique_id.begin(), hwver.unique_id.end(), msg.unique_id.begin());

        ASSERT_FALSE(dynid_sub.collector.msg.get());
        ASSERT_LE(0, dynid_pub.broadcast(msg));
        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1100));
        ASSERT_FALSE(dynid_sub.collector.msg.get());
    }

    ASSERT_EQ(uavcan::NodeID(72), dnidac.getAllocatedNodeID());
    ASSERT_EQ(uavcan::NodeID(10), dnidac.getAllocatorNodeID());
    ASSERT_TRUE(dnidac.isAllocationComplete());
}


TEST(DynamicNodeIDClient, NonPassiveMode)
{
    InterlinkedTestNodesWithSysClock nodes;

    uavcan::DynamicNodeIDClient dnidac(nodes.b);

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::dynamic_node_id::Allocation> _reg1;
    (void)_reg1;

    uavcan::protocol::HardwareVersion hwver;
    for (uavcan::uint8_t i = 0; i < hwver.unique_id.size(); i++)
    {
        hwver.unique_id[i] = i;
    }

    ASSERT_LE(-uavcan::ErrLogic, dnidac.start(hwver));
}
