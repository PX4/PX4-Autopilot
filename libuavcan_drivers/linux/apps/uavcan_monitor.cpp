/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdio>
#include <bitset>
#include <unordered_map>
#include <uavcan_linux/uavcan_linux.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include "debug.hpp"

enum class CLIColor : unsigned
{
    Red     = 31,
    Green   = 32,
    Yellow  = 33,
    Blue    = 34,
    Magenta = 35,
    Cyan    = 36,
    White   = 37,
    Default = 39
};

class CLIColorizer
{
    const CLIColor color_;
public:
    explicit CLIColorizer(CLIColor c) : color_(c)
    {
        std::printf("\033[%um", static_cast<unsigned>(color_));
    }

    ~CLIColorizer()
    {
        std::printf("\033[%um", static_cast<unsigned>(CLIColor::Default));
    }
};

class Monitor : public uavcan::NodeStatusMonitor
{
    uavcan_linux::TimerPtr timer_;
    std::unordered_map<int, uavcan::protocol::NodeStatus> status_registry_;

    void handleNodeStatusMessage(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg) override
    {
        status_registry_[msg.getSrcNodeID().get()] = msg;
    }

    static std::pair<CLIColor, std::string> healthToColoredString(const std::uint8_t health)
    {
        static const std::unordered_map<std::uint8_t, std::pair<CLIColor, std::string>> map
        {
            { uavcan::protocol::NodeStatus::HEALTH_OK,       { CLIColor(CLIColor::Green),   "OK" }},
            { uavcan::protocol::NodeStatus::HEALTH_WARNING,  { CLIColor(CLIColor::Yellow),  "WARNING" }},
            { uavcan::protocol::NodeStatus::HEALTH_ERROR,    { CLIColor(CLIColor::Magenta), "ERROR" }},
            { uavcan::protocol::NodeStatus::HEALTH_CRITICAL, { CLIColor(CLIColor::Red),     "CRITICAL" }}
        };
        try
        {
            return map.at(health);
        }
        catch (std::out_of_range&)
        {
            return { CLIColor(CLIColor::Red), std::to_string(health) };
        }
    }

    static std::pair<CLIColor, std::string> modeToColoredString(const std::uint8_t mode)
    {
        static const std::unordered_map<std::uint8_t, std::pair<CLIColor, std::string>> map
        {
            { uavcan::protocol::NodeStatus::MODE_OPERATIONAL,     { CLIColor(CLIColor::Green),   "OPERATIONAL" }},
            { uavcan::protocol::NodeStatus::MODE_INITIALIZATION,  { CLIColor(CLIColor::Yellow),  "INITIALIZATION" }},
            { uavcan::protocol::NodeStatus::MODE_MAINTENANCE,     { CLIColor(CLIColor::Cyan),    "MAINTENANCE" }},
            { uavcan::protocol::NodeStatus::MODE_SOFTWARE_UPDATE, { CLIColor(CLIColor::Magenta), "SOFTWARE_UPDATE" }},
            { uavcan::protocol::NodeStatus::MODE_OFFLINE,         { CLIColor(CLIColor::Red),     "OFFLINE" }}
        };
        try
        {
            return map.at(mode);
        }
        catch (std::out_of_range&)
        {
            return { CLIColor(CLIColor::Red), std::to_string(mode) };
        }
    }

    void printStatusLine(const uavcan::NodeID nid, const uavcan::NodeStatusMonitor::NodeStatus& status)
    {
        const auto health_and_color = healthToColoredString(status.health);
        const auto mode_and_color   = modeToColoredString(status.mode);

        const int nid_int = nid.get();
        const unsigned long uptime = status_registry_[nid_int].uptime_sec;
        const unsigned vendor_code = status_registry_[nid_int].vendor_specific_status_code;

        std::printf(" %-3d |", nid_int);
        {
            CLIColorizer clz(mode_and_color.first);
            std::printf(" %-15s ", mode_and_color.second.c_str());
        }
        std::printf("|");
        {
            CLIColorizer clz(health_and_color.first);
            std::printf(" %-8s ", health_and_color.second.c_str());
        }
        std::printf("| %-10lu | %04x  %s'%s  %u\n", uptime, vendor_code,
                    std::bitset<8>((vendor_code >> 8) & 0xFF).to_string().c_str(),
                    std::bitset<8>(vendor_code).to_string().c_str(),
                    vendor_code);
    }

    void redraw(const uavcan::TimerEvent&)
    {
        std::printf("\x1b[1J");   // Clear screen from the current cursor position to the beginning
        std::printf("\x1b[H");    // Move cursor to the coordinates 1,1
        std::printf(" NID | Mode            | Health   | Uptime [s] | Vendor-specific status code\n");
        std::printf("-----+-----------------+----------+------------+-hex---bin----------------dec--\n");

        for (unsigned i = 1; i <= uavcan::NodeID::Max; i++)
        {
            if (isNodeKnown(i))
            {
                printStatusLine(i, getNodeStatus(i));
            }
        }
    }

public:
    explicit Monitor(uavcan_linux::NodePtr node)
        : uavcan::NodeStatusMonitor(*node)
        , timer_(node->makeTimer(uavcan::MonotonicDuration::fromMSec(500),
                                 std::bind(&Monitor::redraw, this, std::placeholders::_1)))
    { }
};


static uavcan_linux::NodePtr initNodeInPassiveMode(const std::vector<std::string>& ifaces, const std::string& node_name)
{
    auto node = uavcan_linux::makeNode(ifaces);
    node->setName(node_name.c_str());
    ENFORCE(0 == node->start());
    node->setModeOperational();
    return node;
}

static void runForever(const uavcan_linux::NodePtr& node)
{
    Monitor mon(node);
    ENFORCE(0 == mon.start());
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
        if (argc < 2)
        {
            std::cerr << "Usage:\n\t" << argv[0] << " <can-iface-name-1> [can-iface-name-N...]" << std::endl;
            return 1;
        }
        std::vector<std::string> iface_names;
        for (int i = 1; i < argc; i++)
        {
            iface_names.emplace_back(argv[i]);
        }
        uavcan_linux::NodePtr node = initNodeInPassiveMode(iface_names, "org.uavcan.linux_app.node_status_monitor");
        runForever(node);
        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
}
