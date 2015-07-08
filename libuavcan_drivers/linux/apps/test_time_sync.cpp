/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <iostream>
#include <cassert>
#include <uavcan_linux/uavcan_linux.hpp>
#include <uavcan/protocol/global_time_sync_master.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include "debug.hpp"


static uavcan_linux::NodePtr initNode(const std::vector<std::string>& ifaces, uavcan::NodeID nid,
                                      const std::string& name)
{
    auto node = uavcan_linux::makeNode(ifaces);
    node->setNodeID(nid);
    node->setName(name.c_str());

    ENFORCE(0 == node->start());

    node->setModeOperational();
    return node;
}

static void runForever(const uavcan_linux::NodePtr& node)
{
    uavcan::GlobalTimeSyncMaster tsmaster(*node);
    ENFORCE(0 == tsmaster.init());

    uavcan::GlobalTimeSyncSlave tsslave(*node);
    ENFORCE(0 == tsslave.start());

    auto publish_sync_if_master = [&](const uavcan::TimerEvent&)
    {
        bool i_am_master = false;
        if (tsslave.isActive())
        {
            const uavcan::NodeID master_node = tsslave.getMasterNodeID();
            assert(master_node.isValid());
            if (node->getNodeID() < master_node)
            {
                std::cout << "Overriding the lower priority master " << int(master_node.get()) << std::endl;
                i_am_master = true;
            }
            else
            {
                std::cout << "There is other master of higher priority " << int(master_node.get()) << std::endl;
            }
        }
        else
        {
            std::cout << "No other masters present" << std::endl;
            i_am_master = true;
        }

        // Don't forget to disable slave adjustments if we're master
        tsslave.suppress(i_am_master);

        if (i_am_master)
        {
            ENFORCE(0 <= tsmaster.publish());
        }
    };

    auto sync_publish_timer = node->makeTimer(uavcan::MonotonicDuration::fromMSec(1000), publish_sync_if_master);

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
    try
    {
        if (argc < 3)
        {
            std::cerr << "Usage:\n\t" << argv[0] << " <node-id> <can-iface-name-1> [can-iface-name-N...]" << std::endl;
            return 1;
        }
        const int self_node_id = std::stoi(argv[1]);
        std::vector<std::string> iface_names;
        for (int i = 2; i < argc; i++)
        {
            iface_names.emplace_back(argv[i]);
        }
        uavcan_linux::NodePtr node = initNode(iface_names, self_node_id, "org.uavcan.linux_test_node_status_monitor");
        runForever(node);
        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return 1;
    }
}
