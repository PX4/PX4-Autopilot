/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/dynamic_node_id_allocation_client.hpp>
#include "helpers.hpp"


TEST(DynamicNodeIDAllocationClient, Basic)
{
    // Node A is Allocator, Node B is Allocatee
    InterlinkedTestNodesWithSysClock nodes(uavcan::NodeID(10), uavcan::NodeID::Broadcast);

    uavcan::DynamicNodeIDAllocationClient dnidac(nodes.b);

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::DynamicNodeIDAllocation> _reg1;

    uavcan::protocol::HardwareVersion hwver;

    ASSERT_LE(-uavcan::ErrInvalidParam, dnidac.start(hwver));  // Empty hardware version is not allowed

    /*
     * Initializing.
     * Reduced signature was calculated as follows:
     * >>> crc = pyuavcan.dsdl.signature.Signature()
     * >>> crc.add(range(16))
     * >>> crc.get_value()
     * 4539764000456687298L
     * >>> hex(crc.get_value())
     * '0x3f007b4e4353bec2L'
     */
    const uavcan::uint64_t UniqueID64Bit = 0x3f007b4e4353bec2ULL;
    const uavcan::uint64_t UniqueID57Bit = UniqueID64Bit & ((1ULL << 57) - 1U);
    for (uavcan::uint8_t i = 0; i < hwver.unique_id.size(); i++)
    {
        hwver.unique_id[i] = i;
    }

    // More incorrect inputs
    ASSERT_LE(-uavcan::ErrInvalidParam, dnidac.start(0));
    ASSERT_LE(-uavcan::ErrInvalidParam, dnidac.start(hwver, uavcan::NodeID::Broadcast)); // Bad node ID
    ASSERT_LE(-uavcan::ErrInvalidParam, dnidac.start(hwver, uavcan::NodeID()));          // Ditto

    const uavcan::NodeID PreferredNodeID = 42;
    ASSERT_LE(0, dnidac.start(hwver, PreferredNodeID));

    // Making sure the signature reduction was performed correctly
    ASSERT_EQ(UniqueID57Bit, dnidac.getShortUniqueNodeID());

    ASSERT_FALSE(dnidac.getAllocatedNodeID().isValid());
    ASSERT_FALSE(dnidac.getAllocatorNodeID().isValid());

    /*
     * Initializing subscriber
     * Anonymous transfers must be enabled
     */
    SubscriberWithCollector<uavcan::protocol::DynamicNodeIDAllocation> dynid_sub(nodes.a);
    ASSERT_LE(0, dynid_sub.start());
    dynid_sub.subscriber.allowAnonymousTransfers();

    /*
     * Monitoring requests at 1Hz
     * First request will be sent immediately
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1100));
    ASSERT_TRUE(dynid_sub.collector.msg.get());
    ASSERT_EQ(UniqueID57Bit, dynid_sub.collector.msg->short_unique_id);
    ASSERT_EQ(PreferredNodeID.get(), dynid_sub.collector.msg->node_id);
    dynid_sub.collector.msg.reset();

    // Rate validation
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(100));
    ASSERT_FALSE(dynid_sub.collector.msg.get());

    // Second - rate is 1 Hz
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(900));
    ASSERT_TRUE(dynid_sub.collector.msg.get());
    ASSERT_EQ(UniqueID57Bit, dynid_sub.collector.msg->short_unique_id);
    ASSERT_EQ(PreferredNodeID.get(), dynid_sub.collector.msg->node_id);
    dynid_sub.collector.msg.reset();

    ASSERT_FALSE(dnidac.getAllocatedNodeID().isValid());
    ASSERT_FALSE(dnidac.getAllocatorNodeID().isValid());

    /*
     * Sending a bunch of responses
     * Note that response transfers are NOT anonymous
     */
    uavcan::Publisher<uavcan::protocol::DynamicNodeIDAllocation> dynid_pub(nodes.a);
    ASSERT_LE(0, dynid_pub.init());

    uavcan::protocol::DynamicNodeIDAllocation msg;

    msg.short_unique_id = 123;          // garbage
    msg.node_id = 100;                  // oh whatever
    ASSERT_LE(0, dynid_pub.broadcast(msg));
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(50));

    msg.short_unique_id = 0;            // garbage
    msg.node_id = 101;
    ASSERT_LE(0, dynid_pub.broadcast(msg));
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(50));

    msg.short_unique_id = UniqueID57Bit;  // correct ID
    msg.node_id = 102;                    // THIS NODE ID WILL BE USED
    ASSERT_LE(0, dynid_pub.broadcast(msg));
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(50));

    msg.short_unique_id = UniqueID57Bit;  // repeating, will be ignored
    msg.node_id = 103;
    ASSERT_LE(0, dynid_pub.broadcast(msg));
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(50));

    /*
     * Validating the results
     */
    ASSERT_FALSE(dynid_sub.collector.msg.get());

    ASSERT_TRUE(dnidac.getAllocatedNodeID().isUnicast());
    ASSERT_TRUE(dnidac.getAllocatorNodeID().isUnicast());

    ASSERT_EQ(102, dnidac.getAllocatedNodeID().get());
    ASSERT_EQ(10,  dnidac.getAllocatorNodeID().get());

    // Making sure requests have stopped
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1100));
    ASSERT_FALSE(dynid_sub.collector.msg.get());
}


TEST(DynamicNodeIDAllocationClient, NonPassiveMode)
{
    InterlinkedTestNodesWithSysClock nodes;

    uavcan::DynamicNodeIDAllocationClient dnidac(nodes.b);

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::DynamicNodeIDAllocation> _reg1;

    ASSERT_LE(-uavcan::ErrLogic, dnidac.start(123456789));
}
