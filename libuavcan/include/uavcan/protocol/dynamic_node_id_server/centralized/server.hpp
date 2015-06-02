/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_CENTRALIZED_SERVER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_CENTRALIZED_SERVER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/protocol/dynamic_node_id_server/allocation_request_manager.hpp>
#include <uavcan/protocol/dynamic_node_id_server/node_discoverer.hpp>
#include <uavcan/protocol/dynamic_node_id_server/node_id_selector.hpp>
#include <uavcan/protocol/dynamic_node_id_server/storage_marshaller.hpp>
#include <uavcan/protocol/dynamic_node_id_server/centralized/storage.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
namespace centralized
{
/**
 * This server is an alternative to @ref DistributedServer with the following differences:
 *  - It is not distributed, so using it means introducing a single point of failure into the system.
 *  - It takes less code space and requires less RAM, which makes it suitable for resource-constrained applications.
 *
 * This version is suitable only for simple non-critical systems.
 */
class Server : IAllocationRequestHandler
             , INodeDiscoveryHandler
{
    UniqueID own_unique_id_;

    INode& node_;
    IEventTracer& tracer_;
    AllocationRequestManager allocation_request_manager_;
    NodeDiscoverer node_discoverer_;
    Storage storage_;

    /*
     * Private methods
     */
    bool isNodeIDTaken(const NodeID node_id) const
    {
        return storage_.findByNodeID(node_id) != NULL;
    }

    void tryPublishAllocationResult(const Storage::Entry& entry)
    {
        const int res = allocation_request_manager_.broadcastAllocationResponse(entry.unique_id, entry.node_id);
        if (res < 0)
        {
            tracer_.onEvent(TraceError, res);
            node_.registerInternalFailure("Dynamic allocation response");
        }
    }

    /*
     * Methods of IAllocationRequestHandler
     */
    virtual bool canPublishFollowupAllocationResponse() const
    {
        return true;    // Because there's only one Centralized server in the system
    }

    virtual void handleAllocationRequest(const UniqueID& unique_id, const NodeID preferred_node_id)
    {
        const Storage::Entry* const e = storage_.findByUniqueID(unique_id);
        if (e != NULL)
        {
            tryPublishAllocationResult(*e);
        }
        else
        {
            const NodeID allocated_node_id =
                NodeIDSelector<Server>(this, &Server::isNodeIDTaken).findFreeNodeID(preferred_node_id);

            if (allocated_node_id.isUnicast())
            {
                const int res = storage_.add(allocated_node_id, unique_id);
                if (res >= 0)
                {
                    tryPublishAllocationResult(Storage::Entry(allocated_node_id, unique_id));
                }
                else
                {
                    tracer_.onEvent(TraceError, res);
                    node_.registerInternalFailure("CentralizedServer storage add");
                }
            }
            else
            {
                UAVCAN_TRACE("dynamic_node_id_server::distributed::Server", "Request ignored - no free node ID left");
            }
        }
    }

    /*
     * Methods of INodeDiscoveryHandler
     */
    virtual bool canDiscoverNewNodes() const
    {
        return true;    // Because there's only one Centralized server in the system
    }

    virtual NodeAwareness checkNodeAwareness(NodeID node_id) const
    {
        return (storage_.findByNodeID(node_id) == NULL) ? NodeAwarenessUnknown : NodeAwarenessKnownAndCommitted;
    }

    virtual void handleNewNodeDiscovery(const UniqueID* unique_id_or_null, NodeID node_id)
    {
        if (storage_.findByNodeID(node_id) != NULL)
        {
            UAVCAN_ASSERT(0);   // Such node is already known, the class that called this method should have known that
            return;
        }

        const int res = storage_.add(node_id, (unique_id_or_null == NULL) ? UniqueID() : *unique_id_or_null);
        if (res < 0)
        {
            tracer_.onEvent(TraceError, res);
            node_.registerInternalFailure("CentralizedServer storage add");
        }
    }

public:
    Server(INode& node,
           IStorageBackend& storage,
           IEventTracer& tracer)
        : node_(node)
        , tracer_(tracer)
        , allocation_request_manager_(node, tracer, *this)
        , node_discoverer_(node, tracer, *this)
        , storage_(storage)
    { }

    int init(const UniqueID& own_unique_id)
    {
        /*
         * Initializing storage first, because the next step requires it to be loaded
         */
        int res = storage_.init();
        if (res < 0)
        {
            return res;
        }

        /*
         * Making sure that the server is started with the same node ID
         */
        own_unique_id_ = own_unique_id;

        {
            const Storage::Entry* e = storage_.findByNodeID(node_.getNodeID());
            if (e != NULL)
            {
                if (e->unique_id != own_unique_id_)
                {
                    return -ErrInvalidConfiguration;
                }
            }

            e = storage_.findByUniqueID(own_unique_id_);
            if (e != NULL)
            {
                if (e->node_id != node_.getNodeID())
                {
                    return -ErrInvalidConfiguration;
                }
            }

            if (e == NULL)
            {
                res = storage_.add(node_.getNodeID(), own_unique_id_);
                if (res < 0)
                {
                    return res;
                }
            }
        }

        /*
         * Misc
         */
        res = allocation_request_manager_.init();
        if (res < 0)
        {
            return res;
        }

        res = node_discoverer_.init();
        if (res < 0)
        {
            return res;
        }

        return 0;
    }

    Storage::Size getNumAllocations() const { return storage_.getSize(); }
};

}
}
}

#endif // UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_CENTRALIZED_SERVER_HPP_INCLUDED
