/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <iostream>
#include <thread>
#include <uavcan_linux/uavcan_linux.hpp>
#include <uavcan/node/sub_node.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include "debug.hpp"

static uavcan_linux::NodePtr initMainNode(const std::vector<std::string>& ifaces, uavcan::NodeID nid,
                                          const std::string& name)
{
    std::cout << "Initializing main node" << std::endl;

    auto node = uavcan_linux::makeNode(ifaces);

    node->setNodeID(nid);
    node->setName(name.c_str());

    node->getLogger().setLevel(uavcan::protocol::debug::LogLevel::DEBUG);

    const int start_res = node->start();
    ENFORCE(0 == start_res);

    uavcan::NetworkCompatibilityCheckResult init_result;
    ENFORCE(0 == node->checkNetworkCompatibility(init_result));
    if (!init_result.isOk())
    {
        throw std::runtime_error("Network conflict with node " + std::to_string(init_result.conflicting_node.get()));
    }

    node->setStatusOk();
    return node;
}

static uavcan_linux::SubNodePtr initSubNode(const std::vector<std::string>& ifaces, uavcan::NodeID nid)
{
    std::cout << "Initializing sub node" << std::endl;
    auto node = uavcan_linux::makeSubNode(ifaces);
    node->setNodeID(nid);
    return node;
}

static void runMainNode(const uavcan_linux::NodePtr& node)
{
    std::cout << "Running main node" << std::endl;

    auto do_nothing_once_a_minute = [&node](const uavcan::TimerEvent&)
    {
        node->logInfo("timer", "Another minute passed...");
        node->setVendorSpecificStatusCode(static_cast<std::uint16_t>(std::rand())); // Setting to an arbitrary value
    };
    auto timer = node->makeTimer(uavcan::MonotonicDuration::fromMSec(60000), do_nothing_once_a_minute);

    while (true)
    {
        const int res = node->spin(uavcan::MonotonicDuration::getInfinite());
        if (res < 0)
        {
            node->logError("spin", "Error %*", res);
        }
    }
}

static void runSubNode(const uavcan_linux::SubNodePtr& node)
{
    std::cout << "Running sub node" << std::endl;

    auto log_handler = [](const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage>& msg)
    {
        std::cout << msg << std::endl;
    };
    auto log_sub = node->makeSubscriber<uavcan::protocol::debug::LogMessage>(log_handler);

    struct NodeStatusMonitor : public uavcan::NodeStatusMonitor
    {
        explicit NodeStatusMonitor(uavcan::INode& node) : uavcan::NodeStatusMonitor(node) { }

        virtual void handleNodeStatusChange(const NodeStatusChangeEvent& event) override
        {
            std::cout << "Remote node NID " << int(event.node_id.get()) << " changed status: "
                      << int(event.old_status.status_code) << " --> "
                      << int(event.status.status_code) << std::endl;
        }
    };

    NodeStatusMonitor nsm(*node);
    ENFORCE(0 == nsm.start());

    while (true)
    {
        const int res = node->spin(uavcan::MonotonicDuration::getInfinite());
        if (res < 0)
        {
            std::cerr << "SubNode spin error: " << res << std::endl;
        }
    }
}

int main(int argc, const char** argv)
{
    try
    {
        if (argc < 3)
        {
            std::cerr << "Usage:\n\t" << argv[0] << " <node-id> <can-iface-name-1> [can-iface-name-N...]" << std::endl;
            return 1;
        }

        const int self_node_id = std::stoi(argv[1]);
        std::vector<std::string> iface_names(argv + 2, argv + argc);

        auto node = initMainNode(iface_names, self_node_id, "org.uavcan.linux_test_node");
        auto sub_node = initSubNode(iface_names, self_node_id);

        std::thread sub_thread([&sub_node](){ runSubNode(sub_node); });

        runMainNode(node);

        if (sub_thread.joinable())
        {
            std::cout << "Waiting for the sub thread to join" << std::endl;
            sub_thread.join();
        }

        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return 1;
    }
}
