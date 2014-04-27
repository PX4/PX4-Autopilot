/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <algorithm>
#include <iterator>
#include <uavcan_linux/uavcan_linux.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include "debug.hpp"

#include <uavcan/protocol/param/GetSet.hpp>
#include <uavcan/protocol/param/SaveErase.hpp>

namespace
{

class StdinLineReader
{
    mutable std::mutex mutex_;
    std::thread thread_;
    std::queue<std::string> queue_;

    void worker()
    {
        while (true)
        {
            std::string input;
            std::getline(std::cin, input);
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(input);
        }
    }

public:
    StdinLineReader()
        : thread_(&StdinLineReader::worker, this)
    {
        thread_.detach();
    }

    bool hasPendingInput() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return !queue_.empty();
    }

    std::string getLine()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty())
        {
            throw std::runtime_error("Input queue is empty");
        }
        auto ret = queue_.front();
        queue_.pop();
        return ret;
    }

    std::vector<std::string> getSplitLine()
    {
        std::istringstream iss(getLine());
        std::vector<std::string> out;
        std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
                  std::back_inserter(out));
        return out;
    }
};


std::string paramValueToString(const uavcan::protocol::param::Value& value)
{
    if (!value.value_bool.empty())
    {
        return value.value_bool[0] ? "true" : "false";
    }
    else if (!value.value_int.empty())
    {
        return std::to_string(value.value_int[0]);
    }
    else if (!value.value_float.empty())
    {
        return std::to_string(value.value_float[0]);
    }
    else
    {
        return "?";
    }
}

void printGetSetResponseHeader()
{
    std::cout
        << "Name                                     Value          Default        Min            Max\n"
        << "--------------------------------------------------------------------------------------------------"
        << std::endl;
}

void printGetSetResponse(const uavcan::protocol::param::GetSet::Response& resp)
{
    std::cout << std::setw(41) << std::left << resp.name.c_str();
    std::cout << std::setw(15) << paramValueToString(resp.value);
    std::cout << std::setw(15) << paramValueToString(resp.default_value);
    std::cout << std::setw(15) << paramValueToString(resp.min_value);
    std::cout << std::setw(15) << paramValueToString(resp.max_value);
    std::cout << std::endl;
}

uavcan_linux::NodePtr initNode(const std::vector<std::string>& ifaces, uavcan::NodeID nid, const std::string& name)
{
    auto node = uavcan_linux::makeNode(ifaces);
    node->setNodeID(nid);
    node->setName(name.c_str());
    ENFORCE(0 == node->start());  // This node doesn't check its network compatibility
    node->setStatusOk();
    return node;
}

void runForever(const uavcan_linux::NodePtr& node)
{
    uavcan_linux::BlockingServiceClient<uavcan::protocol::param::GetSet> get_set(*node);
    uavcan_linux::BlockingServiceClient<uavcan::protocol::param::SaveErase> save_erase(*node);
    uavcan_linux::BlockingServiceClient<uavcan::protocol::RestartNode> restart(*node);

    ENFORCE(get_set.init() >= 0);
    ENFORCE(save_erase.init() >= 0);
    ENFORCE(restart.init() >= 0);

    StdinLineReader stdin_reader;

    std::cout << "> " << std::flush;

    while (true)
    {
        ENFORCE(node->spin(uavcan::MonotonicDuration::fromMSec(10)) >= 0);
        if (!stdin_reader.hasPendingInput())
        {
            continue;
        }

        const auto words = stdin_reader.getSplitLine();
        if (words.size() < 2)
        {
            std::cout << "<command> <remote node id> [args...]" << std::endl;
            continue;
        }

        const auto cmd = words.at(0);
        const uavcan::NodeID node_id(std::stoi(words.at(1)));

        if (cmd == "read")
        {
            printGetSetResponseHeader();
            uavcan::protocol::param::GetSet::Request request;
            while (true)
            {
                (void)get_set.blockingCall(node_id, request, uavcan::MonotonicDuration::fromMSec(100));
                ENFORCE(get_set.wasSuccessful());
                if (get_set.getResponse().name.empty())
                {
                    break;
                }
                printGetSetResponse(get_set.getResponse());
                request.index++;
            }
        }
        else if (cmd == "set")
        {
            printGetSetResponseHeader();
            uavcan::protocol::param::GetSet::Request request;
            request.name = words.at(2).c_str();
            request.value.value_float.push_back(std::stof(words.at(3)));
            (void)get_set.blockingCall(node_id, request, uavcan::MonotonicDuration::fromMSec(100));
            ENFORCE(get_set.wasSuccessful());
            printGetSetResponse(get_set.getResponse());
        }
        else if (cmd == "save" || cmd == "erase")
        {
            uavcan::protocol::param::SaveErase::Request request;
            request.opcode = (cmd == "save") ? request.OPCODE_SAVE : request.OPCODE_ERASE;
            (void)save_erase.blockingCall(node_id, request, uavcan::MonotonicDuration::fromMSec(100));
            ENFORCE(save_erase.wasSuccessful());
            std::cout << save_erase.getResponse() << std::endl;
        }
        else if (cmd == "restart")
        {
            uavcan::protocol::RestartNode::Request request;
            request.magic_number = request.MAGIC_NUMBER;
            (void)restart.blockingCall(node_id, request, uavcan::MonotonicDuration::fromMSec(100));
            if (restart.wasSuccessful())
            {
                std::cout << restart.getResponse() << std::endl;
            }
            else
            {
                std::cout << "<NO RESPONSE>" << std::endl;
            }
        }
        else
        {
            std::cout << "Invalid command" << std::endl;
        }
        std::cout << "> " << std::flush;
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
    uavcan_linux::NodePtr node = initNode(iface_names, self_node_id, "org.uavcan.linux_test_node_status_monitor");
    runForever(node);
    return 0;
}
