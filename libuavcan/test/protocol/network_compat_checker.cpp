/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/network_compat_checker.hpp>
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


struct NetworkCompatibilityCheckerRemoteContext
{
    uavcan::NodeStatusProvider node_status_provider;
    uavcan::DataTypeInfoProvider data_type_info_provider;

    NetworkCompatibilityCheckerRemoteContext(uavcan::INode& node)
        : node_status_provider(node)
        , data_type_info_provider(node)
    {
        node_status_provider.setName("com.example");
        uavcan::protocol::SoftwareVersion swver;
        swver.vcs_commit = 1;
        node_status_provider.setSoftwareVersion(swver);
    }

    void start()
    {
        ASSERT_LE(0, node_status_provider.startAndPublish());
        ASSERT_LE(0, data_type_info_provider.start());
    }
};


TEST(NetworkCompatibilityChecker, Size)
{
    // Objects are subject for stack allocation, hence the size matters
    std::cout << "sizeof(uavcan::NetworkCompatibilityChecker): "
              << sizeof(uavcan::NetworkCompatibilityChecker) << std::endl;
    ASSERT_TRUE(sizeof(uavcan::NetworkCompatibilityChecker) < 2048);
}


TEST(NetworkCompatibilityChecker, EmptyNetwork)
{
    registerTypes();
    InterlinkedTestNodesWithSysClock nodes;

    ASSERT_LE(0, uavcan::NetworkCompatibilityChecker::publishGlobalDiscoveryRequest(nodes.a));

    uavcan::NetworkCompatibilityChecker ni(nodes.a);
    ASSERT_LE(0, ni.execute());
    ASSERT_TRUE(ni.getResult().isOk());
}


TEST(NetworkCompatibilityChecker, Success)
{
    registerTypes();
    InterlinkedTestNodesWithSysClock nodes;
    NetworkCompatibilityCheckerRemoteContext remote(nodes.b);
    remote.start();

    BackgroundSpinner bgspinner(nodes.b, nodes.a);
    bgspinner.startPeriodic(uavcan::MonotonicDuration::fromMSec(10));

    ASSERT_LE(0, uavcan::NetworkCompatibilityChecker::publishGlobalDiscoveryRequest(nodes.a));

    uavcan::NetworkCompatibilityChecker ni(nodes.a);
    ASSERT_LE(0, ni.execute());
    ASSERT_TRUE(ni.getResult().isOk());
}


TEST(NetworkCompatibilityChecker, RequestTimeout)
{
    registerTypes();
    InterlinkedTestNodesWithSysClock nodes;
    NetworkCompatibilityCheckerRemoteContext remote(nodes.b);
    remote.start();

    ASSERT_LE(0, uavcan::NetworkCompatibilityChecker::publishGlobalDiscoveryRequest(nodes.a));

    uavcan::NetworkCompatibilityChecker ni(nodes.a);
    // There is no background spinner, so CATS request will time out
    // Despite the time out, the checker will not report failure
    ASSERT_EQ(0, ni.execute());

    // The one (and only) node has failed
    ASSERT_EQ(1, ni.getResult().num_failed_nodes);
    ASSERT_TRUE(ni.getResult().isOk());
}


TEST(NetworkCompatibilityChecker, NodeIDCollision)
{
    registerTypes();
    InterlinkedTestNodesWithSysClock nodes(8, 8);   // Same NID
    NetworkCompatibilityCheckerRemoteContext remote(nodes.b);
    remote.start();

    BackgroundSpinner bgspinner(nodes.b, nodes.a);
    bgspinner.startPeriodic(uavcan::MonotonicDuration::fromMSec(10));

    ASSERT_LE(0, uavcan::NetworkCompatibilityChecker::publishGlobalDiscoveryRequest(nodes.a));

    uavcan::NetworkCompatibilityChecker ni(nodes.a);
    ASSERT_LE(0, ni.execute());
    ASSERT_FALSE(ni.getResult().isOk());
    ASSERT_EQ(8, ni.getResult().conflicting_node.get());
}
