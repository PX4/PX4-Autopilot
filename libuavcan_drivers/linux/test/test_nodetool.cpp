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

template <typename DataType>
typename DataType::Response call(uavcan_linux::BlockingServiceClient<DataType>& client,
                                 uavcan::NodeID server_node_id, const typename DataType::Request& request)
{
    const int res = client.blockingCall(server_node_id, request, uavcan::MonotonicDuration::fromMSec(100));
    ENFORCE(res >= 0);
    ENFORCE(client.wasSuccessful());
    return client.getResponse();
}

void executeCommand(const uavcan_linux::NodePtr& node, const std::string& cmd,
                    const uavcan::NodeID node_id, const std::vector<std::string>& args)
{
    if (cmd == "param")
    {
        uavcan_linux::BlockingServiceClient<uavcan::protocol::param::GetSet> get_set(*node);
        printGetSetResponseHeader();
        uavcan::protocol::param::GetSet::Request request;
        if (args.empty())
        {
            while (true)
            {
                auto response = call(get_set, node_id, request);
                if (response.name.empty())
                {
                    break;
                }
                printGetSetResponse(response);
                request.index++;
            }
        }
        else
        {
            request.name = args.at(0).c_str();
            request.value.value_float.push_back(std::stof(args.at(1)));
            printGetSetResponse(call(get_set, node_id, request));
        }
    }
    else if (cmd == "param_save" || cmd == "param_erase")
    {
        uavcan_linux::BlockingServiceClient<uavcan::protocol::param::SaveErase> save_erase(*node);
        uavcan::protocol::param::SaveErase::Request request;
        request.opcode = (cmd == "param_save") ? request.OPCODE_SAVE : request.OPCODE_ERASE;
        std::cout << call(save_erase, node_id, request) << std::endl;
    }
    else if (cmd == "restart")
    {
        uavcan_linux::BlockingServiceClient<uavcan::protocol::RestartNode> restart(*node);
        uavcan::protocol::RestartNode::Request request;
        request.magic_number = request.MAGIC_NUMBER;
        (void)restart.blockingCall(node_id, request);
        if (restart.wasSuccessful())
        {
            std::cout << restart.getResponse() << std::endl;
        }
        else
        {
            std::cout << "<NO RESPONSE>" << std::endl;
        }
    }
    else if (cmd == "info")
    {
        uavcan_linux::BlockingServiceClient<uavcan::protocol::GetNodeInfo> client(*node);
        std::cout << call(client, node_id, uavcan::protocol::GetNodeInfo::Request()) << std::endl;
    }
    else if (cmd == "tstat")
    {
        uavcan_linux::BlockingServiceClient<uavcan::protocol::GetTransportStats> client(*node);
        std::cout << call(client, node_id, uavcan::protocol::GetTransportStats::Request()) << std::endl;
    }
    else
    {
        std::cout << "Invalid command" << std::endl;
    }
}

void runForever(const uavcan_linux::NodePtr& node)
{
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
        if (words.size() >= 2)
        {
            const auto cmd = words.at(0);
            const uavcan::NodeID node_id(std::stoi(words.at(1)));
            try
            {
                executeCommand(node, cmd, node_id, std::vector<std::string>(words.begin() + 2, words.end()));
            }
            catch (std::exception& ex)
            {
                std::cout << "FAILURE\n" << ex.what() << std::endl;
            }
        }
        else
        {
            std::cout << "<command> <remote node id> [args...]" << std::endl;
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
