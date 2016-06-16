/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <map>
#include <algorithm>
#include <iterator>
#include <uavcan_linux/uavcan_linux.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include "debug.hpp"

#include <uavcan/protocol/param/GetSet.hpp>
#include <uavcan/protocol/param/ExecuteOpcode.hpp>
#include <uavcan/equipment/hardpoint/Command.hpp>

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

uavcan_linux::NodePtr initNode(const std::vector<std::string>& ifaces, uavcan::NodeID nid, const std::string& name)
{
    auto node = uavcan_linux::makeNode(ifaces, name.c_str(),
                                       uavcan::protocol::SoftwareVersion(), uavcan::protocol::HardwareVersion(),
                                       nid);
    node->setModeOperational();
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

/*
 * Command table.
 * The structure is:
 *      command_name : (command_usage_info, command_entry_point)
 * This code was written while listening to some bad dubstep so I'm not sure about its quality.
 */
const std::map<std::string,
               std::pair<std::string,
                         std::function<void(const uavcan_linux::NodePtr&, const uavcan::NodeID,
                                            const std::vector<std::string>&)>
                        >
              > commands =
{
    {
        "param",
        {
            "No arguments supplied - requests all params from a remote node\n"
            "<param_name> <param_value> - assigns parameter <param_name> to value <param_value>",
            [](const uavcan_linux::NodePtr& node, const uavcan::NodeID node_id, const std::vector<std::string>& args)
            {
                auto client = node->makeBlockingServiceClient<uavcan::protocol::param::GetSet>();
                uavcan::protocol::param::GetSet::Request request;
                if (args.empty())
                {
                    while (true)
                    {
                        auto response = call(*client, node_id, request);
                        if (response.name.empty())
                        {
                            break;
                        }
                        std::cout
                            << response
                            << "\n" << std::string(80, '-')
                            << std::endl;
                        request.index++;
                    }
                }
                else
                {
                    request.name = args.at(0).c_str();
                    // TODO: add support for string parameters
                    request.value.to<uavcan::protocol::param::Value::Tag::real_value>() = std::stof(args.at(1));
                    std::cout << call(*client, node_id, request) << std::endl;
                }
            }
        }
    },
    {
        "param_save",
        {
            "Calls uavcan.protocol.param.ExecuteOpcode on a remote node with OPCODE_SAVE",
            [](const uavcan_linux::NodePtr& node, const uavcan::NodeID node_id, const std::vector<std::string>&)
            {
                auto client = node->makeBlockingServiceClient<uavcan::protocol::param::ExecuteOpcode>();
                uavcan::protocol::param::ExecuteOpcode::Request request;
                request.opcode = request.OPCODE_SAVE;
                std::cout << call(*client, node_id, request) << std::endl;
            }
        }
    },
    {
        "param_erase",
        {
            "Calls uavcan.protocol.param.ExecuteOpcode on a remote node with OPCODE_ERASE",
            [](const uavcan_linux::NodePtr& node, const uavcan::NodeID node_id, const std::vector<std::string>&)
            {
                auto client = node->makeBlockingServiceClient<uavcan::protocol::param::ExecuteOpcode>();
                uavcan::protocol::param::ExecuteOpcode::Request request;
                request.opcode = request.OPCODE_ERASE;
                std::cout << call(*client, node_id, request) << std::endl;
            }
        }
    },
    {
        "restart",
        {
            "Restarts a remote node using uavcan.protocol.RestartNode",
            [](const uavcan_linux::NodePtr& node, const uavcan::NodeID node_id, const std::vector<std::string>&)
            {
                auto client = node->makeBlockingServiceClient<uavcan::protocol::RestartNode>();
                uavcan::protocol::RestartNode::Request request;
                request.magic_number = request.MAGIC_NUMBER;
                (void)client->blockingCall(node_id, request);
                if (client->wasSuccessful())
                {
                    std::cout << client->getResponse() << std::endl;
                }
                else
                {
                    std::cout << "<NO RESPONSE>" << std::endl;
                }
            }
        }
    },
    {
        "info",
        {
            "Calls uavcan.protocol.GetNodeInfo on a remote node",
            [](const uavcan_linux::NodePtr& node, const uavcan::NodeID node_id, const std::vector<std::string>&)
            {
                auto client = node->makeBlockingServiceClient<uavcan::protocol::GetNodeInfo>();
                std::cout << call(*client, node_id, uavcan::protocol::GetNodeInfo::Request()) << std::endl;
            }
        }
    },
    {
        "transport_stats",
        {
            "Calls uavcan.protocol.GetTransportStats on a remote node",
            [](const uavcan_linux::NodePtr& node, const uavcan::NodeID node_id, const std::vector<std::string>&)
            {
                auto client = node->makeBlockingServiceClient<uavcan::protocol::GetTransportStats>();
                std::cout << call(*client, node_id, uavcan::protocol::GetTransportStats::Request()) << std::endl;
            }
        }
    },
    {
        "hardpoint",
        {
            "Publishes uavcan.equipment.hardpoint.Command\n"
            "Expected argument: command",
            [](const uavcan_linux::NodePtr& node, const uavcan::NodeID, const std::vector<std::string>& args)
            {
                uavcan::equipment::hardpoint::Command msg;
                msg.command = std::stoi(args.at(0));
                auto pub = node->makePublisher<uavcan::equipment::hardpoint::Command>();
                (void)pub->broadcast(msg);
            }
        }
    }
};

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
        bool command_is_known = false;

        try
        {
            if (words.size() >= 2)
            {
                const auto cmd = words.at(0);
                const uavcan::NodeID node_id(std::stoi(words.at(1)));
                auto it = commands.find(cmd);
                if (it != std::end(commands))
                {
                    command_is_known = true;
                    it->second.second(node, node_id, std::vector<std::string>(words.begin() + 2, words.end()));
                }
            }
        }
        catch (std::exception& ex)
        {
            std::cout << "FAILURE\n" << ex.what() << std::endl;
        }

        if (!command_is_known)
        {
            std::cout << "<command> <remote node id> [args...]\n";
            std::cout << "Say 'help' to get help.\n";     // I'll show myself out.

            if (!words.empty() && words.at(0) == "help")
            {
                std::cout << "Usage:\n\n";
                for (auto& cmd : commands)
                {
                    std::cout << cmd.first << "\n" << cmd.second.first << "\n\n";
                }
            }
        }
        std::cout << "> " << std::flush;
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
        const std::vector<std::string> iface_names(argv + 2, argv + argc);
        uavcan_linux::NodePtr node = initNode(iface_names, self_node_id, "org.uavcan.linux_app.nodetool");
        runForever(node);
        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
}
