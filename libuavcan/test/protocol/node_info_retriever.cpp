/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#if __GNUC__
// We need auto_ptr for compatibility reasons
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#include <memory>
#include <gtest/gtest.h>
#include <uavcan/protocol/node_info_retriever.hpp>
#include <uavcan/protocol/node_status_provider.hpp>
#include "helpers.hpp"

static void publishNodeStatus(PairableCanDriver& can, uavcan::NodeID node_id, uavcan::uint8_t status_code,
                              uavcan::uint32_t uptime_sec, uavcan::TransferID tid)
{
    uavcan::protocol::NodeStatus msg;
    msg.status_code = status_code;
    msg.uptime_sec = uptime_sec;
    emulateSingleFrameBroadcastTransfer(can, node_id, msg, tid);
}


struct NodeInfoListener : public uavcan::INodeInfoListener
{
    std::auto_ptr<uavcan::protocol::GetNodeInfo::Response> last_node_info;
    uavcan::NodeID last_node_id;
    unsigned status_message_cnt;
    unsigned status_change_cnt;
    unsigned info_unavailable_cnt;

    NodeInfoListener()
        : status_message_cnt(0)
        , status_change_cnt(0)
        , info_unavailable_cnt(0)
    { }

    virtual void handleNodeInfoRetrieved(uavcan::NodeID node_id,
                                         const uavcan::protocol::GetNodeInfo::Response& node_info)
    {
        last_node_id = node_id;
        std::cout << node_info << std::endl;
        last_node_info.reset(new uavcan::protocol::GetNodeInfo::Response(node_info));
    }

    virtual void handleNodeInfoUnavailable(uavcan::NodeID node_id)
    {
        std::cout << "NODE INFO FOR " << int(node_id.get()) << " IS UNAVAILABLE" << std::endl;
        last_node_id = node_id;
        info_unavailable_cnt++;
    }

    virtual void handleNodeStatusChange(const uavcan::NodeStatusMonitor::NodeStatusChangeEvent& event)
    {
        (void)event;
        status_change_cnt++;
    }

    virtual void handleNodeStatusMessage(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg)
    {
        std::cout << msg << std::endl;
        status_message_cnt++;
    }
};


TEST(NodeInfoRetriever, Basic)
{
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetNodeInfo> _reg2;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GlobalDiscoveryRequest> _reg3;

    InterlinkedTestNodesWithSysClock nodes;

    uavcan::NodeInfoRetriever retr(nodes.a);
    std::cout << "sizeof(uavcan::NodeInfoRetriever): " << sizeof(uavcan::NodeInfoRetriever) << std::endl;
    std::cout << "sizeof(uavcan::ServiceClient<uavcan::protocol::GetNodeInfo>): "
        << sizeof(uavcan::ServiceClient<uavcan::protocol::GetNodeInfo>) << std::endl;

    std::auto_ptr<uavcan::NodeStatusProvider> provider(new uavcan::NodeStatusProvider(nodes.b));

    NodeInfoListener listener;

    /*
     * Initialization
     */
    ASSERT_LE(0, retr.start());

    retr.removeListener(&listener);     // Does nothing
    retr.addListener(&listener);

    uavcan::protocol::HardwareVersion hwver;
    hwver.unique_id[0] = 123;
    hwver.unique_id[4] = 213;
    hwver.unique_id[8] = 45;

    provider->setName("Ivan");
    provider->setHardwareVersion(hwver);

    ASSERT_LE(0, provider->startAndPublish());

    /*
     * Waiting for discovery
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1600));

    ASSERT_EQ(2, listener.status_message_cnt);
    ASSERT_EQ(1, listener.status_change_cnt);
    ASSERT_EQ(0, listener.info_unavailable_cnt);
    ASSERT_TRUE(listener.last_node_info.get());
    ASSERT_EQ(uavcan::NodeID(2), listener.last_node_id);
    ASSERT_EQ("Ivan", listener.last_node_info->name);
    ASSERT_TRUE(hwver == listener.last_node_info->hardware_version);

    provider.reset();   // Moving the provider out of the way; its entry will timeout meanwhile

    /*
     * Declaring a bunch of different nodes that don't support GetNodeInfo
     */
    retr.setNumRequestAttempts(3);

    uavcan::TransferID tid;

    publishNodeStatus(nodes.can_a, uavcan::NodeID(10), 0, 10, tid);
    publishNodeStatus(nodes.can_a, uavcan::NodeID(11), 0, 10, tid);
    publishNodeStatus(nodes.can_a, uavcan::NodeID(12), 0, 10, tid);

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(2100));

    tid.increment();
    publishNodeStatus(nodes.can_a, uavcan::NodeID(10), 0, 11, tid);
    publishNodeStatus(nodes.can_a, uavcan::NodeID(11), 0, 11, tid);
    publishNodeStatus(nodes.can_a, uavcan::NodeID(12), 0, 11, tid);

    // TODO finish the test when the logic is fixed
}
