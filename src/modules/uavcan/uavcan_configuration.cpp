/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file uavcan_configuration.cpp
 *
 * @author Marco Bauer <marco@wtns.de>
 */


#include "uavcan_configuration.hpp"
#include <systemlib/err.h>
#include <unistd.h>
#include <functional>


template <typename T>
std::pair<int, typename T::Response> performBlockingServiceCall(uavcan::INode& node, uavcan::NodeID remote_node_id,
                                                                const typename T::Request& request)
{
    bool success = false;
    typename T::Response response;  // Generated types have zero-initializing constructors, always.
 
    uavcan::ServiceClient<T, std::function<void (const uavcan::ServiceCallResult<T>&)> > client(node);
    client.setCallback([&](const uavcan::ServiceCallResult<T>& result)
	{
		success = result.isSuccessful();
		response = result.response;
	});
 
    const int call_res = client.call(remote_node_id, request);
    if (call_res >= 0)
    {
        while (client.isPending())
        {
            const int spin_res = node.spin(uavcan::MonotonicDuration::fromMSec(2));
            if (spin_res < 0)
            {
                return {spin_res, response};
            }
        }
        return {success ? 0 : -uavcan::ErrFailure, response};
    }
    return {call_res, response};
}

UavcanRemoteConfiguration::UavcanRemoteConfiguration(uavcan::INode &node) :
	_mavlink_fd(-1),
	_node(node),
	_uavcan_pub_enumeration_cmd(node)
{
}

UavcanRemoteConfiguration::~UavcanRemoteConfiguration()
{
}

int UavcanRemoteConfiguration::init()
{
	_mavlink_fd = ::open(MAVLINK_LOG_DEVICE, 0);
	return 0;
}


void UavcanRemoteConfiguration::uavcan_configuration(float val_1, float val_2, float val_3, float val_4, float val_5, float val_6, float val_7)
{
	switch ((int)val_1) {
		case UAVCAN_CMD_PARAM_SAVE:
			cmd_param_save((int)val_2, 0);
			break;

		case UAVCAN_CMD_PARAM_GET_ALL:
			cmd_param_get_all((int)val_2);
			break;

		case UAVCAN_CMD_PARAM_GET:
			cmd_param_get((int)val_2, (int)val_3);
			break;

		case UAVCAN_CMD_PARAM_SET:
			cmd_set_param_set((int)val_2, (int)val_3, (float)val_4);
			break;

		case UAVCAN_CMD_DISCOVER_FOR_NODES:
			cmd_discover_for_nodes();
			break;

		case UAVCAN_CMD_NODE_GET_INFO:
			cmd_node_get_info((int)val_2);
			break;

		case UAVCAN_CMD_NODE_FACTORY_RESET:
			cmd_param_save((int)val_2, 1);
			break;

		case UAVCAN_CMD_NODE_FACTORY_RESET_ALL_NODES:
			mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: ERROR unsupported Command");
			break;

		case UAVCAN_CMD_NODE_RESTART:
			cmd_node_restart((int)val_2);
			break;

		case UAVCAN_CMD_ENUMERATE:
			cmd_enumerate((int)val_2, (int)val_3);
			break;

		default:
			mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: ERROR unsupported Command");
			break;
	}
}


void UavcanRemoteConfiguration::cmd_param_save(int targetnode, int factoryreset)
{
	uavcan::protocol::param::SaveErase::Request msg;

	msg.opcode = factoryreset;
	auto res = performBlockingServiceCall<uavcan::protocol::param::SaveErase>(_node, targetnode, msg);

	if (res.first < 0) {
		// error
		mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: ERROR Node:%d", targetnode);
	} else {
		if (factoryreset == 0) {
			mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: Param-saved Node:%d", targetnode);
		} else {
			mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: Factory-reset Node:%d", targetnode);
		}
	}
}


void UavcanRemoteConfiguration::cmd_param_get_all(int targetnode)
{
	std::vector<uavcan::protocol::param::GetSet::Response> remote_params;
	//node_get_param_list(targetnode, remote_params);

	while (true)
	{
		uavcan::protocol::param::GetSet::Request req;
		req.index = remote_params.size();
		auto res = performBlockingServiceCall<uavcan::protocol::param::GetSet>(_node, targetnode, req);
		if (res.first < 0)
		{
			mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: ERROR Node:%d", targetnode);
		}
		if (res.second.name.empty())  // Empty name means no such param, which means we're finished
		{
			mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: OK Node:%d", targetnode);
			break;
		}
		remote_params.push_back(res.second);
	}

	int index = 0;
	
	for (auto p : remote_params)
	{

		if (!p.value.value_bool.empty()) {
			mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM|%d|%s|bool|%s|false|true", 
				index, 
				p.name.c_str(),
				p.value.value_bool[0] ? "true" : "false"
			);
	
		} else if (!p.value.value_int.empty()) {
			mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM|%d|%s|int|%d|%d|%d", 
				index, 
				p.name.c_str(),
				(int)p.value.value_int[0],
				(int)p.min_value.value_int[0],
				(int)p.max_value.value_int[0]
			);
	
		} else if (!p.value.value_float.empty()) {
			mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM|%d|%s|float|%.8f|%.8f|%.8f", 
				index, 
				p.name.c_str(),
				(double)p.value.value_float[0],
				(double)p.min_value.value_float[0],
				(double)p.max_value.value_float[0]
			);
	
		}
	
		index++;
	}

}


void UavcanRemoteConfiguration::cmd_param_get(int targetnode, int paramindex)
{
	std::vector<uavcan::protocol::param::GetSet::Response> remote_params;
	//node_get_param_list(targetnode, remote_params);

	while (true)
    {
       	uavcan::protocol::param::GetSet::Request req;
       	req.index = remote_params.size();
       	auto res = performBlockingServiceCall<uavcan::protocol::param::GetSet>(_node, targetnode, req);
       	if (res.first < 0)
       	{
			mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: ERROR Node:%d", targetnode);
       	}
       	if (res.second.name.empty())  // Empty name means no such param, which means we're finished
       	{
			mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: OK Node:%d", targetnode);
			break;
       	}
       	remote_params.push_back(res.second);
    }

	int index = 0;
	
	for (auto p : remote_params)
	{
	
		if (index == paramindex) {
			if (!p.value.value_bool.empty()) {
	    		mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM|%d|%s|bool|%s|false|true", 
					index, 
					p.name.c_str(),
					p.value.value_bool[0] ? "true" : "false"
				);
	
			} else if (!p.value.value_int.empty()) {
	    		mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM|%d|%s|int|%d|%d|%d", 
					index, 
					p.name.c_str(),
					(int)p.value.value_int[0],
					(int)p.min_value.value_int[0],
					(int)p.max_value.value_int[0]
				);
	
			} else if (!p.value.value_float.empty()) {
	    		mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM|%d|%s|float|%.8f|%.8f|%.8f", 
					index, 
					p.name.c_str(),
					(double)p.value.value_float[0],
					(double)p.min_value.value_float[0],
					(double)p.max_value.value_float[0]
				);
	
			}
	
		}
	
		index++;
	}

}


void UavcanRemoteConfiguration::cmd_set_param_set(int targetnode, int paramindex, float value)
{
	std::vector<uavcan::protocol::param::GetSet::Response> remote_params;
	//node_get_param_list(targetnode, remote_params);

	while (true)
   	{
       	uavcan::protocol::param::GetSet::Request req;
       	req.index = remote_params.size();
       	auto res = performBlockingServiceCall<uavcan::protocol::param::GetSet>(_node, targetnode, req);
       	if (res.first < 0)
       	{
			mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: ERROR Node:%d", targetnode);
       	}
       	if (res.second.name.empty())  // Empty name means no such param, which means we're finished
       	{
			mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: OK Node:%d", targetnode);
			break;
       	}
       	remote_params.push_back(res.second);
   	}

	int index = 0;
	
	for (auto p : remote_params)
	{
	
		if (index == paramindex) {
			uavcan::protocol::param::GetSet::Request msg;
			//msg.name = "uavcan_esc_index";
			msg.index = index;
	
			if (!p.value.value_bool.empty()) {
				int _boolVal = (int)value;
				msg.value.value_bool.push_back(static_cast<bool>((bool)_boolVal));
	
			} else if (!p.value.value_int.empty()) {
				msg.value.value_int.push_back(static_cast<int>(value));
	
			} else if (!p.value.value_float.empty()) {
				msg.value.value_float.push_back(static_cast<float>(value));
	
			}
	
			auto res = performBlockingServiceCall<uavcan::protocol::param::GetSet>(_node, targetnode, msg);
	
			if (res.first < 0) {
				// error
				mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: ERROR Node:%d", targetnode);
			} else {
				mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: OK Node:%d", targetnode);
			}
	
		}
	
		index++;
	}

}


void UavcanRemoteConfiguration::cmd_enumerate(int targetnode, int timeout)
{
	uavcan::protocol::EnumerationRequest msg;

	msg.node_id = targetnode;
	msg.timeout_sec = timeout;
	
	(void)_uavcan_pub_enumeration_cmd.broadcast(msg);

	mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: Enumeration Node:%d Timeout:%d", targetnode, timeout);
}


void UavcanRemoteConfiguration::cmd_node_restart(int targetnode)
{
	uavcan::protocol::RestartNode::Request restart_req;

    restart_req.magic_number = restart_req.MAGIC_NUMBER;
	auto res = performBlockingServiceCall<uavcan::protocol::RestartNode>(_node, targetnode, restart_req);

	if (res.first < 0) {
		// error
		mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: ERROR Node:%d", targetnode);
	} else {
		mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: Restart Node:%d", targetnode);
	}
}


void UavcanRemoteConfiguration::cmd_discover_for_nodes(void)
{

    class NodeMonitor : public uavcan::NodeStatusMonitor
    {
     public:
        NodeMonitor(uavcan::INode& _node) : uavcan::NodeStatusMonitor(_node) { }
 
        static const char* statusToString(NodeStatus status)
        {
            if (status.known)
            {
                if (status.status_code == uavcan::protocol::NodeStatus::STATUS_OK)           { return "OK"; }
                if (status.status_code == uavcan::protocol::NodeStatus::STATUS_INITIALIZING) { return "INITIALIZING"; }
                if (status.status_code == uavcan::protocol::NodeStatus::STATUS_WARNING)      { return "WARNING"; }
                if (status.status_code == uavcan::protocol::NodeStatus::STATUS_CRITICAL)     { return "CRITICAL"; }
                if (status.status_code == uavcan::protocol::NodeStatus::STATUS_OFFLINE)      { return "OFFLINE"; }
                return "???";
            }
            else
            {
                /*
                 * This means that this node was never seen online.
                 */
                return "UNKNOWN";
            }
        }
    } monitor(_node);

    if (monitor.start() < 0)
    {
        //return 1;       // TODO error handling
    }

    if (_node.spin(uavcan::MonotonicDuration::fromMSec(2000)) < 0)
    {
        //return 1;       // TODO error handling
    }
 
    for (int i = 1; i <= uavcan::NodeID::Max; i++)
    {
        auto status = monitor.getNodeStatus(i);
        if (status.known)
        {
            mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: Node ID %d: %s", i, NodeMonitor::statusToString(status));
            /*
             * It is left as an exercise for the reader to call the following services for each discovered node:
             *  - uavcan.protocol.GetNodeInfo       - full node information (name, HW/SW version)
             *  - uavcan.protocol.GetTransportStats - transport layer statistics (num transfers, errors, iface stats)
             *  - uavcan.protocol.GetDataTypeInfo   - data type check: is supported? how used? is compatible?
             */
        }
    }

}


void UavcanRemoteConfiguration::cmd_node_get_info(int targetnode)
{
/*
	std::vector<uavcan::protocol::GetNodeInfo::Response> remote_info;
        uavcan::protocol::GetNodeInfo::Request req;
        auto res = performBlockingServiceCall<uavcan::protocol::GetNodeInfo>(_node, targetnode, req);
        if (res.first < 0)
        {
           	mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: ERROR Node:%d", targetnode);
        } else {
        	mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: OK Node:%d", targetnode);
 	      	//mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: NodeInfo:%s", res.second[2].name);
        }
*/

}


void UavcanRemoteConfiguration::node_get_param_list(int targetnode, std::vector<uavcan::protocol::param::GetSet::Response>& remote_params) {

	while (true)
	{
		uavcan::protocol::param::GetSet::Request req;
		req.index = remote_params.size();
		auto res = performBlockingServiceCall<uavcan::protocol::param::GetSet>(_node, targetnode, req);
		if (res.first < 0)
		{
			mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: ERROR Node:%d", targetnode);
		}
		if (res.second.name.empty())  // Empty name means no such param, which means we're finished
		{
			mavlink_and_console_log_info(_mavlink_fd, "UAVCANPARAM: OK Node:%d", targetnode);
			break;
		}
		remote_params.push_back(res.second);
	}

}

