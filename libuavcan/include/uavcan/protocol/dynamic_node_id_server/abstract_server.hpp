/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_SERVER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_SERVER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/protocol/dynamic_node_id_server/allocation_request_manager.hpp>
#include <uavcan/protocol/dynamic_node_id_server/node_discoverer.hpp>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{

class AbstractServer : protected IAllocationRequestHandler
                     , protected INodeDiscoveryHandler
{
    UniqueID own_unique_id_;

protected:
    INode& node_;
    IEventTracer& tracer_;
    AllocationRequestManager allocation_request_manager_;
    NodeDiscoverer node_discoverer_;

    AbstractServer(INode& node,
                   IEventTracer& tracer) :
       node_(node),
       tracer_(tracer),
       allocation_request_manager_(node, tracer, *this),
       node_discoverer_(node, tracer, *this)
    { }

    const UniqueID& getOwnUniqueID() const { return own_unique_id_; }

    int init(const UniqueID& own_unique_id, const TransferPriority priority)
    {
        int res = 0;

        own_unique_id_ = own_unique_id;

        res = allocation_request_manager_.init(priority);
        if (res < 0)
        {
            return res;
        }

        res = node_discoverer_.init(priority);
        if (res < 0)
        {
            return res;
        }

        return 0;
    }

public:
    const NodeDiscoverer& getNodeDiscoverer() const { return node_discoverer_; }
};

}
}

#endif // Include guard
