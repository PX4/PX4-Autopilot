/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <uavcan_linux/uavcan_linux.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include "debug.hpp"

struct OstreamColorizer
{
    enum Color
    {
        Red     = 31,
        Green   = 32,
        Yellow  = 33,
        Blue    = 34,
        Magenta = 35,
        Default = 39
    };
    explicit OstreamColorizer(Color color = Default) : color_(color) { }
    friend std::ostream& operator<<(std::ostream& os, const OstreamColorizer& mod)
    {
        return os << "\033[" << int(mod.color_) << "m";
    }
private:
    const Color color_;
};


class Monitor : public uavcan::NodeStatusMonitor
{
    uavcan_linux::TimerPtr timer_;
    std::unordered_map<int, std::uint64_t> uptimes_;

    virtual void handleNodeStatusMessage(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg)
    {
        uptimes_[msg.getSrcNodeID().get()] = msg.uptime_sec;
    }

    static std::pair<OstreamColorizer, std::string>
    statusCodeToColoredString(uavcan::NodeStatusMonitor::NodeStatusCode status_code)
    {
        if (status_code == uavcan::protocol::NodeStatus::STATUS_OK)
        {
            return { OstreamColorizer(OstreamColorizer::Green), "OK" };
        }
        if (status_code == uavcan::protocol::NodeStatus::STATUS_INITIALIZING)
        {
            return { OstreamColorizer(OstreamColorizer::Blue), "INITIALIZING" };
        }
        if (status_code == uavcan::protocol::NodeStatus::STATUS_WARNING)
        {
            return { OstreamColorizer(OstreamColorizer::Yellow), "WARNING" };
        }
        if (status_code == uavcan::protocol::NodeStatus::STATUS_CRITICAL)
        {
            return { OstreamColorizer(OstreamColorizer::Red), "CRITICAL" };
        }
        if (status_code == uavcan::protocol::NodeStatus::STATUS_OFFLINE)
        {
            return { OstreamColorizer(OstreamColorizer::Magenta), "OFFLINE" };
        }
        return { OstreamColorizer(), "???" };
    }

    void printStatusLine(uavcan::NodeID nid, const uavcan::NodeStatusMonitor::NodeStatus& status)
    {
        const auto color_and_string = statusCodeToColoredString(status.status_code);
        const int nid_int = nid.get();
        std::cout << color_and_string.first;
        std::cout << " " << std::setw(3) << std::left << nid_int << " | "
                  << std::setw(13) << std::left << color_and_string.second << " | "
                  << uptimes_[nid_int];
        std::cout << OstreamColorizer() << "\n";
    }

    void redraw(const uavcan::TimerEvent&)
    {
        std::cout << "\x1b\x5b\x48" << "\x1b\x5b\x32\x4a"
                  << " NID | Status        | Uptime\n"
                  << "-----+---------------+--------\n";
        for (unsigned i = 1; i <= uavcan::NodeID::Max; i++)
        {
            const auto s = getNodeStatus(i);
            if (s.known)
            {
                printStatusLine(i, s);
            }
        }
        std::cout << std::flush;
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
    node->setStatusOk();
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
    if (argc < 2)
    {
        std::cout << "Usage:\n\t" << argv[0] << " <can-iface-name-1> [can-iface-name-N...]" << std::endl;
        return 1;
    }
    std::vector<std::string> iface_names;
    for (int i = 1; i < argc; i++)
    {
        iface_names.emplace_back(argv[i]);
    }
    uavcan_linux::NodePtr node = initNodeInPassiveMode(iface_names, "org.uavcan.status_monitor");
    runForever(node);
    return 0;
}
