/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/node_initializer.hpp>
#include <uavcan/protocol/node_status_provider.hpp>
#include <uavcan/protocol/data_type_info_provider.hpp>
#include <uavcan/protocol/GlobalDiscoveryRequest.hpp>
#include <memory>
#include "helpers.hpp"


static void registerTypes()
{
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GlobalDiscoveryRequest> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg2;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::ComputeAggregateTypeSignature> _reg3;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetNodeInfo> _reg4;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetDataTypeInfo> _reg5;
}


struct NodeInitializerRemoteContext
{
    uavcan::NodeStatusProvider node_status_provider;
    uavcan::DataTypeInfoProvider data_type_info_provider;

    NodeInitializerRemoteContext(uavcan::INode& node)
        : node_status_provider(node)
        , data_type_info_provider(node)
    {
        node_status_provider.setName("com.example");
        uavcan::protocol::SoftwareVersion swver;
        swver.build = 1;
        node_status_provider.setSoftwareVersion(swver);
    }

    void start()
    {
        ASSERT_LE(0, node_status_provider.startAndPublish());
        ASSERT_LE(0, data_type_info_provider.start());
    }
};


TEST(NodeInitializer, Size)
{
    std::cout << "sizeof(uavcan::NodeInitializer): " << sizeof(uavcan::NodeInitializer) << std::endl;
    ASSERT_TRUE(sizeof(uavcan::NodeInitializer) < 1024);
}


TEST(NodeInitializer, EmptyNetwork)
{
    registerTypes();
    InterlinkedTestNodesWithSysClock nodes;

    ASSERT_LE(0, uavcan::NodeInitializer::publishGlobalDiscoveryRequest(nodes.a));

    uavcan::NodeInitializer ni(nodes.a);
    ASSERT_LE(0, ni.execute());
    ASSERT_TRUE(ni.getResult().isOk());
}


TEST(NodeInitializer, Success)
{
    registerTypes();
    InterlinkedTestNodesWithSysClock nodes;
    NodeInitializerRemoteContext remote(nodes.b);
    remote.start();

    BackgroundSpinner bgspinner(nodes.b, nodes.a);
    bgspinner.startPeriodic(uavcan::MonotonicDuration::fromMSec(10));

    ASSERT_LE(0, uavcan::NodeInitializer::publishGlobalDiscoveryRequest(nodes.a));

    uavcan::NodeInitializer ni(nodes.a);
    ASSERT_LE(0, ni.execute());
    ASSERT_TRUE(ni.getResult().isOk());
}


TEST(NodeInitializer, RequestTimeout)
{
    registerTypes();
    InterlinkedTestNodesWithSysClock nodes;
    NodeInitializerRemoteContext remote(nodes.b);
    remote.start();

    ASSERT_LE(0, uavcan::NodeInitializer::publishGlobalDiscoveryRequest(nodes.a));

    uavcan::NodeInitializer ni(nodes.a);
    ASSERT_GT(0, ni.execute());            // There is no background spinner, so CATS request will time out
}


TEST(NodeInitializer, NodeIDCollision)
{
    registerTypes();
    InterlinkedTestNodesWithSysClock nodes(8, 8);   // Same NID
    NodeInitializerRemoteContext remote(nodes.b);
    remote.start();

    BackgroundSpinner bgspinner(nodes.b, nodes.a);
    bgspinner.startPeriodic(uavcan::MonotonicDuration::fromMSec(10));

    ASSERT_LE(0, uavcan::NodeInitializer::publishGlobalDiscoveryRequest(nodes.a));

    uavcan::NodeInitializer ni(nodes.a);
    ASSERT_LE(0, ni.execute());
    ASSERT_FALSE(ni.getResult().isOk());
    ASSERT_EQ(8, ni.getResult().conflicting_node.get());
}
