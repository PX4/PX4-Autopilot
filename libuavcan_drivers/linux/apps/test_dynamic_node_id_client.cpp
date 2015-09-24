/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <iostream>
#include "debug.hpp"
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include <uavcan_linux/uavcan_linux.hpp>

namespace
{

uavcan_linux::NodePtr initNodeWithDynamicID(const std::vector<std::string>& ifaces,
                                            const std::uint8_t instance_id,
                                            const uavcan::NodeID preferred_node_id,
                                            const std::string& name)
{
    /*
     * Initializing the node object
     */
    auto node = uavcan_linux::makeNode(ifaces);

    node->setName(name.c_str());
    node->getLogger().setLevel(uavcan::protocol::debug::LogLevel::DEBUG);

    {
        const auto app_id = uavcan_linux::makeApplicationID(uavcan_linux::MachineIDReader().read(), name, instance_id);

        uavcan::protocol::HardwareVersion hwver;
        std::copy(app_id.begin(), app_id.end(), hwver.unique_id.begin());
        std::cout << hwver << std::endl;

        node->setHardwareVersion(hwver);
    }

    /*
     * Starting the node
     */
    const int start_res = node->start();
    ENFORCE(0 == start_res);

    /*
     * Running the dynamic node ID client until it's done
     */
    uavcan::DynamicNodeIDClient client(*node);

    ENFORCE(0 <= client.start(node->getNodeStatusProvider().getHardwareVersion().unique_id, preferred_node_id));

    std::cout << "Waiting for dynamic node ID allocation..." << std::endl;

    while (!client.isAllocationComplete())
    {
        const int res = node->spin(uavcan::MonotonicDuration::fromMSec(100));
        if (res < 0)
        {
            std::cerr << "Spin error: " << res << std::endl;
        }
    }

    std::cout << "Node ID " << int(client.getAllocatedNodeID().get())
              << " allocated by " << int(client.getAllocatorNodeID().get()) << std::endl;

    /*
     * Finishing the node initialization
     */
    node->setNodeID(client.getAllocatedNodeID());

    node->setModeOperational();

    return node;
}

void runForever(const uavcan_linux::NodePtr& node)
{
    while (true)
    {
        const int res = node->spin(uavcan::MonotonicDuration::fromMSec(100));
        if (res < 0)
        {
            std::cerr << "Spin error: " << res << std::endl;
        }
    }
}

}

int main(int argc, const char** argv)
{
    try
    {
        if (argc < 3)
        {
            std::cerr << "Usage:\n\t"
                      << argv[0] << " <instance-id> <can-iface-name-1> [can-iface-name-N...]\n"
                      << "Where <instance-id> is used to augment the unique node ID and also indicates\n"
                      << "the preferred node ID value. Valid range is [0, 127]."
                      << std::endl;
            return 1;
        }

        const int instance_id = std::stoi(argv[1]);
        if (instance_id < 0 || instance_id > 127)
        {
            std::cerr << "Invalid instance ID: " << instance_id << std::endl;
            std::exit(1);
        }

        uavcan_linux::NodePtr node = initNodeWithDynamicID(std::vector<std::string>(argv + 2, argv + argc),
                                                           std::uint8_t(instance_id),
                                                           std::uint8_t(instance_id),
                                                           "org.uavcan.linux_test_dynamic_node_id_client");
        runForever(node);

        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
}
