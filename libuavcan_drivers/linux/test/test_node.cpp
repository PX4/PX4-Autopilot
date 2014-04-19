/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <iostream>
#include <uavcan_linux/uavcan_linux.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include "debug.hpp"

static uavcan_linux::NodePtr initNode(const std::vector<std::string>& ifaces, uavcan::NodeID nid,
                                      const std::string& name)
{
    auto node = uavcan_linux::makeNode(ifaces);

    /*
     * Configuring the node.
     */
    node->setNodeID(nid);
    node->setName(name.c_str());

    node->getLogger().setLevel(uavcan::protocol::debug::LogLevel::DEBUG);

    /*
     * Starting the node.
     */
    std::cout << "Starting the node..." << std::endl;
    const int start_res = node->start();
    std::cout << "Start returned: " << start_res << std::endl;
    ENFORCE(0 == start_res);

    /*
     * Checking if our node conflicts with other nodes. This may take a few seconds.
     */
    uavcan::NetworkCompatibilityCheckResult init_result;
    ENFORCE(0 == node->checkNetworkCompatibility(init_result));
    if (!init_result.isOk())
    {
        throw std::runtime_error("Network conflict with node " + std::to_string(init_result.conflicting_node.get()));
    }

    std::cout << "Node started successfully" << std::endl;

    /*
     * Say Hi to the world.
     */
    node->setStatusOk();
    node->logInfo("init", "Hello world! I'm [%*], NID %*",
                  node->getNodeStatusProvider().getName().c_str(), int(node->getNodeID().get()));
    return node;
}

static void runForever(const uavcan_linux::NodePtr& node)
{
    /*
     * Subscribing to the UAVCAN logging topic
     */
    auto log_handler = [](const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage>& msg)
    {
        std::cout << msg << std::endl;
    };
    auto log_sub = node->makeSubscriber<uavcan::protocol::debug::LogMessage>(log_handler);

    /*
     * Printing when other nodes enter the network or change status
     */
    struct NodeStatusMonitor : public uavcan::NodeStatusMonitor
    {
        explicit NodeStatusMonitor(uavcan::INode& node) : uavcan::NodeStatusMonitor(node) { }

        virtual void handleNodeStatusChange(const NodeStatusChangeEvent& event)
        {
            std::cout << "Remote node NID " << int(event.node_id.get()) << " changed status: "
                      << int(event.old_status.status_code) << " --> "
                      << int(event.status.status_code) << std::endl;
        }
    };

    NodeStatusMonitor nsm(*node);
    ENFORCE(0 == nsm.start());

    /*
     * Adding a stupid timer that does nothing once a minute
     */
    auto do_nothing_once_a_minute = [&node](const uavcan::TimerEvent&)
    {
        node->logInfo("timer", "Another minute passed...");
    };
    auto timer = node->makeTimer(uavcan::MonotonicDuration::fromMSec(60000), do_nothing_once_a_minute);

    /*
     * Spinning forever
     */
    while (true)
    {
        const int res = node->spin(uavcan::MonotonicDuration::getInfinite());
        if (res < 0)
        {
            node->logError("spin", "Error %*", res);
        }
    }
}

int main(int argc, const char** argv)
{
    if (argc < 3)
    {
        std::cout << "Usage:\n\t" << argv[0] << " <node-id> <can-iface-name-1> [can-iface-name-N...]" << std::endl;
        return 1;
    }
    const int self_node_id = std::stoi(argv[1]);
    std::vector<std::string> iface_names;
    for (int i = 2; i < argc; i++)
    {
        iface_names.emplace_back(argv[i]);
    }
    uavcan_linux::NodePtr node = initNode(iface_names, self_node_id, "org.uavcan.linux_test_node");
    std::cout << "Node initialized successfully" << std::endl;
    runForever(node);
    return 0;
}
