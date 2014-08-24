/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/node_status_provider.hpp>
#include "helpers.hpp"


TEST(NodeStatusProvider, Basic)
{
    InterlinkedTestNodesWithSysClock nodes;

    uavcan::NodeStatusProvider nsp(nodes.a);

    /*
     * Initialization
     */
    uavcan::protocol::HardwareVersion hwver;
    hwver.major = 3;
    hwver.minor = 14;

    uavcan::protocol::SoftwareVersion swver;
    swver.major = 2;
    swver.minor = 18;
    swver.vcs_commit = 0x600DF00D;

    nsp.setHardwareVersion(hwver);
    nsp.setSoftwareVersion(swver);

    ASSERT_TRUE(nsp.getName().empty());
    nsp.setName("superluminal_communication_unit");
    ASSERT_STREQ("superluminal_communication_unit", nsp.getName().c_str());

    ASSERT_EQ(uavcan::protocol::NodeStatus::STATUS_INITIALIZING, nsp.getStatusCode());
    nsp.setStatusOk();
    ASSERT_EQ(uavcan::protocol::NodeStatus::STATUS_OK, nsp.getStatusCode());

    // Will fail - types are not registered
    uavcan::GlobalDataTypeRegistry::instance().reset();
    ASSERT_GT(0, nsp.startAndPublish());

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetNodeInfo> _reg2;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GlobalDiscoveryRequest> _reg3;
    ASSERT_LE(0, nsp.startAndPublish());

    // Checking the publishing rate settings
    ASSERT_EQ(uavcan::MonotonicDuration::fromMSec(uavcan::protocol::NodeStatus::MAX_PUBLICATION_PERIOD_MS),
              nsp.getStatusPublishingPeriod());

    nsp.setStatusPublishingPeriod(uavcan::MonotonicDuration());
    ASSERT_EQ(uavcan::MonotonicDuration::fromMSec(uavcan::protocol::NodeStatus::MIN_PUBLICATION_PERIOD_MS),
              nsp.getStatusPublishingPeriod());

    nsp.setStatusPublishingPeriod(uavcan::MonotonicDuration::fromMSec(3600 * 1000 * 24));
    ASSERT_EQ(uavcan::MonotonicDuration::fromMSec(uavcan::protocol::NodeStatus::MAX_PUBLICATION_PERIOD_MS),
              nsp.getStatusPublishingPeriod());

    /*
     * Initial status publication
     */
    SubscriberWithCollector<uavcan::protocol::NodeStatus> status_sub(nodes.b);

    ASSERT_LE(0, status_sub.start());
    ASSERT_FALSE(status_sub.collector.msg.get());  // No data yet

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));

    ASSERT_TRUE(status_sub.collector.msg.get());  // Was published at startup
    ASSERT_EQ(uavcan::protocol::NodeStatus::STATUS_OK, status_sub.collector.msg->status_code);
    ASSERT_GE(1, status_sub.collector.msg->uptime_sec);

    /*
     * Explicit node info request
     */
    ServiceClientWithCollector<uavcan::protocol::GetNodeInfo> gni_cln(nodes.b);

    nsp.setStatusCritical();

    ASSERT_FALSE(gni_cln.collector.result.get());  // No data yet
    ASSERT_LE(0, gni_cln.call(1, uavcan::protocol::GetNodeInfo::Request()));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));
    ASSERT_TRUE(gni_cln.collector.result.get());   // Response must have been delivered

    ASSERT_TRUE(gni_cln.collector.result->isSuccessful());
    ASSERT_EQ(1, gni_cln.collector.result->server_node_id.get());

    ASSERT_EQ(uavcan::protocol::NodeStatus::STATUS_CRITICAL, gni_cln.collector.result->response.status.status_code);

    ASSERT_TRUE(hwver == gni_cln.collector.result->response.hardware_version);
    ASSERT_TRUE(swver == gni_cln.collector.result->response.software_version);

    ASSERT_EQ("superluminal_communication_unit", gni_cln.collector.result->response.name);

    /*
     * GlobalDiscoveryRequest
     */
    uavcan::Publisher<uavcan::protocol::GlobalDiscoveryRequest> gdr_pub(nodes.b);

    status_sub.collector.msg.reset();
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));
    ASSERT_FALSE(status_sub.collector.msg.get());                                // Nothing!

    ASSERT_LE(0, gdr_pub.broadcast(uavcan::protocol::GlobalDiscoveryRequest()));

    nsp.setStatusWarning();

    status_sub.collector.msg.reset();
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));

    ASSERT_TRUE(status_sub.collector.msg.get());
    ASSERT_EQ(uavcan::protocol::NodeStatus::STATUS_WARNING, status_sub.collector.msg->status_code);
}
