/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <iostream>
#include <string>
#include <cstdlib>
#include <cstdio>
#include <sys/types.h>
#include <unistd.h>
#include "debug.hpp"
// UAVCAN
#include <uavcan/protocol/file_server.hpp>
// UAVCAN Linux drivers
#include <uavcan_linux/uavcan_linux.hpp>
// UAVCAN POSIX drivers
#include <uavcan_posix/basic_file_server_backend.hpp>
#include <uavcan_posix/firmware_version_checker.hpp>  // Compilability test

namespace
{

uavcan_linux::NodePtr initNode(const std::vector<std::string>& ifaces, uavcan::NodeID nid, const std::string& name)
{
    auto node = uavcan_linux::makeNode(ifaces);

    node->setNodeID(nid);
    node->setName(name.c_str());
    node->getLogger().setLevel(uavcan::protocol::debug::LogLevel::DEBUG);

    {
        const auto app_id = uavcan_linux::makeApplicationID(uavcan_linux::MachineIDReader().read(), name, nid.get());

        uavcan::protocol::HardwareVersion hwver;
        std::copy(app_id.begin(), app_id.end(), hwver.unique_id.begin());
        std::cout << hwver << std::endl;

        node->setHardwareVersion(hwver);
    }

    const int start_res = node->start();
    ENFORCE(0 == start_res);

    node->setModeOperational();

    return node;
}

void runForever(const uavcan_linux::NodePtr& node)
{
    uavcan_posix::BasicFileServerBackend backend(*node);

    uavcan::FileServer server(*node, backend);

    const int server_init_res = server.start();
    if (server_init_res < 0)
    {
        throw std::runtime_error("Failed to start the server; error " + std::to_string(server_init_res));
    }

    while (true)
    {
        const int res = node->spin(uavcan::MonotonicDuration::fromMSec(100));
        if (res < 0)
        {
            std::cerr << "Spin error: " << res << std::endl;
        }
    }
}

struct Options
{
    uavcan::NodeID node_id;
    std::vector<std::string> ifaces;
};

Options parseOptions(int argc, const char** argv)
{
    const char* const executable_name = *argv++;
    argc--;

    const auto enforce = [executable_name](bool condition, const char* error_text) {
        if (!condition)
        {
            std::cerr << error_text << "\n"
                      << "Usage:\n\t"
                      << executable_name
                      << " <node-id> <can-iface-name-1> [can-iface-name-N...]"
                      << std::endl;
            std::exit(1);
        }
    };

    enforce(argc >= 2, "Not enough arguments");

    /*
     * Node ID is always at the first position
     */
    argc--;
    const int node_id = std::stoi(*argv++);
    enforce(node_id >= 1 && node_id <= 127, "Invalid node ID");

    Options out;
    out.node_id = uavcan::NodeID(std::uint8_t(node_id));

    while (argc --> 0)
    {
        const std::string token(*argv++);

        if (token[0] != '-')
        {
            out.ifaces.push_back(token);
        }
        else
        {
            enforce(false, "Unexpected argument");
        }
    }

    return out;
}

}

int main(int argc, const char** argv)
{
    try
    {
        auto options = parseOptions(argc, argv);

        std::cout << "Self node ID: " << int(options.node_id.get()) << "\n"
                     "Num ifaces:   " << options.ifaces.size() << "\n"
#ifdef NDEBUG
                     "Build mode:   Release"
#else
                     "Build mode:   Debug"
#endif
                     << std::endl;

        auto node = initNode(options.ifaces, options.node_id, "org.uavcan.linux_test_file_server");
        runForever(node);
        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
}
