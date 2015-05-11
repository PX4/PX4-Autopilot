/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_NODE_DISCOVERER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_NODE_DISCOVERER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/util/map.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/util/bitset.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/service_client.hpp>
#include <uavcan/protocol/dynamic_node_id_server/types.hpp>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>
#include <cassert>
// UAVCAN types
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
/**
 * The allocator must implement this interface.
 */
class INodeDiscoveryHandler
{
public:
    /**
     * In order to avoid bus congestion, normally only leader can discover new nodes.
     */
    virtual bool canDiscoverNewNodes() const = 0;

    /**
     * These values represent the level of awareness of a certain node by the server.
     */
    enum NodeAwareness
    {
        NodeAwarenessUnknown,
        NodeAwarenessKnownButNotCommitted,
        NodeAwarenessKnownAndCommitted
    };

    /**
     * It is OK to do full log traversal in this method, because the unique ID collector will cache the
     * result when possible.
     */
    virtual NodeAwareness checkNodeAwareness(NodeID node_id) const = 0;

    /**
     * This method will be called when a new node responds to GetNodeInfo request.
     * If this method fails to register the node, the node will be queried again later and this method will be
     * invoked again.
     * Unique ID will be NULL if the node is assumed to not implement the GetNodeInfo service.
     */
    virtual void handleNewNodeDiscovery(const UniqueID* unique_id_or_null, NodeID node_id) = 0;

    virtual ~INodeDiscoveryHandler() { }
};

/**
 * This class listens to NodeStatus messages from other nodes and retrieves their unique ID if they are not
 * known to the allocator.
 */
class NodeDiscoverer : TimerBase
{
    typedef MethodBinder<NodeDiscoverer*, void (NodeDiscoverer::*)(const ServiceCallResult<protocol::GetNodeInfo>&)>
        GetNodeInfoResponseCallback;

    typedef MethodBinder<NodeDiscoverer*, void (NodeDiscoverer::*)(const ReceivedDataStructure<protocol::NodeStatus>&)>
        NodeStatusCallback;

    struct NodeData
    {
        uint32_t last_seen_uptime;              ///< Used for restart detection
        uint8_t num_get_node_info_attempts;     ///< Used to detect if the node doesn't support GetNodeInfo
        bool updated_since_last_query;          ///< Allows to pause trying to GetNodeInfo if the node is offline

        NodeData()
            : last_seen_uptime(0)
            , num_get_node_info_attempts(0)
            , updated_since_last_query(false)
        { }
    };

    typedef Map<NodeID, NodeData, 10> NodeMap;

    /**
     * When this number of attempts has been made, the discoverer will give up and assume that the node
     * does not implement this service.
     */
    enum { MaxAttemptsToGetNodeInfo = 3 };

    /*
     * States
     */
    INodeDiscoveryHandler& handler_;
    IEventTracer& tracer_;

    BitSet<NodeID::Max + 1> committed_node_mask_;       ///< Nodes that are marked will not be queried
    NodeMap node_map_;                                  ///< Will not work in UAVCAN_TINY

    ServiceClient<protocol::GetNodeInfo, GetNodeInfoResponseCallback> get_node_info_client_;
    Subscriber<protocol::NodeStatus, NodeStatusCallback> node_status_sub_;

    /*
     * Methods
     */
    void trace(TraceCode code, int64_t argument) { tracer_.onEvent(code, argument); }

    INode& getNode() { return node_status_sub_.getNode(); }

    NodeID pickNextNodeToQuery() const
    {
        const unsigned node_map_size = node_map_.getSize();

        // Searching the lowest number of attempts made
        uint8_t lowest_number_of_attempts = static_cast<uint8_t>(MaxAttemptsToGetNodeInfo + 1U);
        for (unsigned i = 0; i < node_map_size; i++)
        {
            const NodeMap::KVPair* const kv = node_map_.getByIndex(i);
            UAVCAN_ASSERT(kv != NULL);
            if (kv->value.updated_since_last_query)
            {
                lowest_number_of_attempts = min(lowest_number_of_attempts, kv->value.num_get_node_info_attempts);
            }
        }

        // Now, among nodes with this number of attempts selecting the one with highest uptime.
        NodeID output;
        uint32_t largest_uptime = 0;
        for (unsigned i = 0; i < node_map_size; i++)
        {
            const NodeMap::KVPair* const kv = node_map_.getByIndex(i);
            UAVCAN_ASSERT(kv != NULL);
            if ((kv->value.num_get_node_info_attempts == lowest_number_of_attempts) &&
                (kv->value.last_seen_uptime >= largest_uptime) &&
                (kv->value.updated_since_last_query))
            {
                largest_uptime = kv->value.last_seen_uptime;
                output = kv->key;
            }
        }

        // An invalid node ID will be returned only if there's no nodes at all.
        return output;
    }

    bool needToQuery(NodeID node_id)
    {
        UAVCAN_ASSERT(node_id.isUnicast());

        /*
         * Fast check
         */
        if (committed_node_mask_[node_id.get()])
        {
            return false;
        }

        /*
         * Slow check - may involve full log search
         */
        const INodeDiscoveryHandler::NodeAwareness awareness = handler_.checkNodeAwareness(node_id);

        if (awareness == INodeDiscoveryHandler::NodeAwarenessUnknown)
        {
            return true;
        }
        else if (awareness == INodeDiscoveryHandler::NodeAwarenessKnownButNotCommitted)
        {
            node_map_.remove(node_id);
            return false;
        }
        else if (awareness == INodeDiscoveryHandler::NodeAwarenessKnownAndCommitted)
        {
            trace(TraceDiscoveryCommitCacheUpdated, node_id.get());
            committed_node_mask_[node_id.get()] = true;
            node_map_.remove(node_id);
            return false;
        }
        else
        {
            UAVCAN_ASSERT(0);
            return false;
        }
    }

    void finalizeNodeDiscovery(const UniqueID* unique_id_or_null, NodeID node_id)
    {
        trace(TraceDiscoveryNodeFinalized, node_id.get() | ((unique_id_or_null == NULL) ? 0U : 0x100U));
        node_map_.remove(node_id);
        if (needToQuery(node_id))     // Making sure the server is still interested
        {
            handler_.handleNewNodeDiscovery(unique_id_or_null, node_id);
        }
    }

    void handleGetNodeInfoResponse(const ServiceCallResult<protocol::GetNodeInfo>& result)
    {
        if (result.isSuccessful())
        {
            UAVCAN_TRACE("dynamic_node_id_server::NodeDiscoverer", "GetNodeInfo response from %d",
                         int(result.server_node_id.get()));
            finalizeNodeDiscovery(&result.response.hardware_version.unique_id, result.server_node_id);
        }
        else
        {
            trace(TraceDiscoveryGetNodeInfoFailure, result.server_node_id.get());

            NodeData* const data = node_map_.access(result.server_node_id);
            if (data == NULL)
            {
                return;         // Probably it is a known node now
            }

            UAVCAN_TRACE("dynamic_node_id_server::NodeDiscoverer",
                         "GetNodeInfo request to %d has timed out, %d attempts",
                         int(result.server_node_id.get()), int(data->num_get_node_info_attempts));
            data->num_get_node_info_attempts++;
            if (data->num_get_node_info_attempts >= MaxAttemptsToGetNodeInfo)
            {
                finalizeNodeDiscovery(NULL, result.server_node_id);
            }
        }
    }

    void handleTimerEvent(const TimerEvent&)
    {
        if (get_node_info_client_.isPending())
        {
            return;
        }

        if (!handler_.canDiscoverNewNodes())
        {
            trace(TraceDiscoveryTimerStop, node_map_.getSize());
            stop();
            return;
        }

        const NodeID node_id = pickNextNodeToQuery();
        if (!node_id.isUnicast())
        {
            trace(TraceDiscoveryTimerStop, node_map_.getSize());
            stop();
            return;
        }

        NodeData* const data = node_map_.access(node_id);
        UAVCAN_ASSERT(data != NULL);
        UAVCAN_ASSERT(data->updated_since_last_query);
        data->updated_since_last_query = false;

        trace(TraceDiscoveryGetNodeInfoRequest, node_id.get());

        UAVCAN_TRACE("dynamic_node_id_server::NodeDiscoverer", "Requesting GetNodeInfo from node %d",
                     int(node_id.get()));
        const int res = get_node_info_client_.call(node_id, protocol::GetNodeInfo::Request());
        if (res < 0)
        {
            getNode().registerInternalFailure("NodeDiscoverer GetNodeInfo call");
        }
    }

    void handleNodeStatus(const ReceivedDataStructure<protocol::NodeStatus>& msg)
    {
        if (!needToQuery(msg.getSrcNodeID()))
        {
            return;
        }

        NodeData* data = node_map_.access(msg.getSrcNodeID());
        if (data == NULL)
        {
            trace(TraceDiscoveryNewNodeFound, msg.getSrcNodeID().get());

            data = node_map_.insert(msg.getSrcNodeID(), NodeData());
            if (data == NULL)
            {
                getNode().registerInternalFailure("NodeDiscoverer OOM");
                return;
            }
        }
        UAVCAN_ASSERT(data != NULL);

        if (msg.uptime_sec < data->last_seen_uptime)
        {
            trace(TraceDiscoveryNodeRestartDetected, msg.getSrcNodeID().get());
            data->num_get_node_info_attempts = 0;
        }
        data->last_seen_uptime = msg.uptime_sec;
        data->updated_since_last_query = true;

        if (!isRunning() && handler_.canDiscoverNewNodes())
        {
            trace(TraceDiscoveryTimerStart, get_node_info_client_.getRequestTimeout().toUSec());
            startPeriodic(get_node_info_client_.getRequestTimeout());
        }
    }

public:
    NodeDiscoverer(INode& node, IEventTracer& tracer, INodeDiscoveryHandler& handler)
        : TimerBase(node)
        , handler_(handler)
        , tracer_(tracer)
        , node_map_(node.getAllocator())
        , get_node_info_client_(node)
        , node_status_sub_(node)
    { }

    int init()
    {
        int res = get_node_info_client_.init();
        if (res < 0)
        {
            return res;
        }
        get_node_info_client_.setCallback(
            GetNodeInfoResponseCallback(this, &NodeDiscoverer::handleGetNodeInfoResponse));

        res = node_status_sub_.start(NodeStatusCallback(this, &NodeDiscoverer::handleNodeStatus));
        if (res < 0)
        {
            return res;
        }

        // Note: the timer starts ad-hoc from the node status callback, not here.

        return 0;
    }

    /**
     * Returns true if there's at least one node with pending GetNodeInfo.
     */
    bool hasUnknownNodes() const { return !node_map_.isEmpty(); }
};

}
}

#endif // Include guard
