/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISTRIBUTED_SERVER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISTRIBUTED_SERVER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/protocol/dynamic_node_id_server/distributed/types.hpp>
#include <uavcan/protocol/dynamic_node_id_server/distributed/event.hpp>
#include <uavcan/protocol/dynamic_node_id_server/distributed/raft_core.hpp>
#include <uavcan/protocol/dynamic_node_id_server/allocation_request_manager.hpp>
#include <uavcan/protocol/dynamic_node_id_server/node_id_selector.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
namespace distributed
{
/**
 * This class implements the top-level allocation logic and server API.
 */
class Server : private IAllocationRequestHandler
             , private ILeaderLogCommitHandler
{
    struct UniqueIDLogPredicate
    {
        const UniqueID unique_id;

        UniqueIDLogPredicate(const UniqueID& uid)
            : unique_id(uid)
        { }

        bool operator()(const RaftCore::LogEntryInfo& info) const
        {
            return info.entry.unique_id == unique_id;
        }
    };

    struct NodeIDLogPredicate
    {
        const NodeID node_id;

        NodeIDLogPredicate(const NodeID& nid)
            : node_id(nid)
        { }

        bool operator()(const RaftCore::LogEntryInfo& info) const
        {
            return info.entry.node_id == node_id.get();
        }
    };

    /*
     * States
     */
    INode& node_;
    RaftCore raft_core_;
    AllocationRequestManager allocation_request_manager_;

    /*
     * Methods
     */
    bool isNodeIDTaken(const NodeID node_id) const
    {
        UAVCAN_TRACE("DynamicNodeIDServer", "Testing if node ID %d is taken", int(node_id.get()));
        return raft_core_.traverseLogFromEndUntil(NodeIDLogPredicate(node_id));
    }

    void allocateNewNode(const UniqueID& unique_id, const NodeID preferred_node_id)
    {
        const NodeID allocated_node_id =
            NodeIDSelector<Server>(this, &Server::isNodeIDTaken).findFreeNodeID(preferred_node_id);
        if (!allocated_node_id.isUnicast())
        {
            UAVCAN_TRACE("DynamicNodeIDServer", "Request ignored - no free node ID left");
            return;
        }

        UAVCAN_TRACE("DynamicNodeIDServer", "New node ID allocated: %d", int(allocated_node_id.get()));
        const int res = raft_core_.appendLog(unique_id, allocated_node_id);
        if (res < 0)
        {
            node_.registerInternalFailure("Raft log append");
        }
    }

    virtual void handleAllocationRequest(const UniqueID& unique_id, const NodeID preferred_node_id)
    {
        // TODO: allocation requests must not be served if the list of unidentified nodes is not empty

        const LazyConstructor<RaftCore::LogEntryInfo> result =
            raft_core_.traverseLogFromEndUntil(UniqueIDLogPredicate(unique_id));

         if (result.isConstructed())
         {
             if (result->committed)
             {
                 tryPublishAllocationResult(result->entry);
                 UAVCAN_TRACE("DynamicNodeIDServer",
                              "Allocation request served with existing allocation; node ID %d",
                              int(result->entry.node_id));
             }
             else
             {
                 UAVCAN_TRACE("DynamicNodeIDServer",
                              "Allocation request ignored - allocation exists but not committed yet; node ID %d",
                              int(result->entry.node_id));
             }
         }
         else
         {
             allocateNewNode(unique_id, preferred_node_id);
         }
    }

    virtual void onEntryCommitted(const protocol::dynamic_node_id::server::Entry& entry)
    {
        tryPublishAllocationResult(entry);
    }

    virtual void onLeaderChange(bool local_node_is_leader)
    {
        UAVCAN_TRACE("DynamicNodeIDServer", "I am leader: %d", int(local_node_is_leader));
        allocation_request_manager_.setActive(local_node_is_leader);
    }

    void tryPublishAllocationResult(const protocol::dynamic_node_id::server::Entry& entry)
    {
        const int res = allocation_request_manager_.broadcastAllocationResponse(entry.unique_id, entry.node_id);
        if (res < 0)
        {
            node_.registerInternalFailure("Dynamic allocation final broadcast");
        }
    }

public:
    Server(INode& node,
           IStorageBackend& storage,
           IEventTracer& tracer)
        : node_(node)
        , raft_core_(node, storage, tracer, *this)
        , allocation_request_manager_(node, *this)
    { }

    int init(uint8_t cluster_size = ClusterManager::ClusterSizeUnknown)
    {
        int res = raft_core_.init(cluster_size);
        if (res < 0)
        {
            return res;
        }

        res = allocation_request_manager_.init();
        if (res < 0)
        {
            return res;
        }

        // TODO Initialize the node info transport

        return 0;
    }
};

}
}
}

#endif // Include guard
