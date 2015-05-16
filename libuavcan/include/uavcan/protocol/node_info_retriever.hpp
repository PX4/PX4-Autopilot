/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_NODE_INFO_RETRIEVER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_NODE_INFO_RETRIEVER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/util/multiset.hpp>
#include <uavcan/node/service_client.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>

namespace uavcan
{
/**
 * Classes that need to receive GetNodeInfo responses should implement this interface.
 */
class UAVCAN_EXPORT INodeInfoListener
{
public:
    /**
     * Called when a response to GetNodeInfo request is received. This happens shortly after the node restarts or
     * becomes online for the first time.
     * @param node_id   Node ID of the node
     * @param response  Node info struct
     */
    virtual void handleNodeInfoRetrieved(NodeID node_id, const protocol::GetNodeInfo::Response& node_info) = 0;

    /**
     * Called when the retriever decides that the node does not support the GetNodeInfo service.
     * This method will never be called if the number of attempts is unlimited.
     */
    virtual void handleNodeInfoUnavailable(NodeID node_id) = 0;

    /**
     * This call is routed directly from @ref NodeStatusMonitor.
     * Default implementation does nothing.
     * @param event     Node status change event
     */
    virtual void handleNodeStatusChange(const NodeStatusMonitor::NodeStatusChangeEvent& event)
    {
        (void)event;
    }

    /**
     * This call is routed directly from @ref NodeStatusMonitor.
     * Default implementation does nothing.
     * @param msg       Node status message
     */
    virtual void handleNodeStatusMessage(const ReceivedDataStructure<protocol::NodeStatus>& msg)
    {
        (void)msg;
    }

    virtual ~INodeInfoListener() { }
};

/**
 * This class automatically retrieves a response to GetNodeInfo once a node appears online or restarts.
 * It does a number of attempts in case if there's a communication failure before assuming that the node does not
 * implement the GetNodeInfo service.
 * Events from this class can be routed to many subscribers.
 */
class UAVCAN_EXPORT NodeInfoRetriever : NodeStatusMonitor
                                      , TimerBase
{
public:
    enum { MaxNumRequestAttempts = 254 };
    enum { DefaultNumRequestAttempts = 30 };
    enum { UnlimitedRequestAttempts = 0 };

private:
    typedef MethodBinder<NodeInfoRetriever*,
                         void (NodeInfoRetriever::*)(const ServiceCallResult<protocol::GetNodeInfo>&)>
            GetNodeInfoResponseCallback;

    struct Entry
    {
        uint32_t uptime_sec;
        uint8_t num_attempts_made;
        bool request_needed;                    ///< Always false for unknown nodes
        bool updated_since_last_attempt;        ///< Always false for unknown nodes

        Entry()
            : uptime_sec(0)
            , num_attempts_made(0)
            , request_needed(false)
            , updated_since_last_attempt(false)
        {
#if UAVCAN_DEBUG
            StaticAssert<sizeof(Entry) <= 8>::check();
#endif
        }
    };

    /*
     * Callers are used with removeWhere() predicate. They don't actually remove anything.
     */
    struct NodeInfoRetrievedHandlerCaller
    {
        const NodeID node_id;
        const protocol::GetNodeInfo::Response& node_info;

        NodeInfoRetrievedHandlerCaller(NodeID arg_node_id, const protocol::GetNodeInfo::Response& arg_node_info)
            : node_id(arg_node_id)
            , node_info(arg_node_info)
        { }

        bool operator()(INodeInfoListener* key)
        {
            UAVCAN_ASSERT(key != NULL);
            key->handleNodeInfoRetrieved(node_id, node_info);
            return false;
        }
    };

    template <typename Event>
    struct GenericHandlerCaller
    {
        void (INodeInfoListener::* const method)(Event);
        Event event;

        GenericHandlerCaller(void (INodeInfoListener::*arg_method)(Event), Event arg_event)
            : method(arg_method)
            , event(arg_event)
        { }

        bool operator()(INodeInfoListener* key)
        {
            UAVCAN_ASSERT(key != NULL);
            (key->*method)(event);
            return false;
        }
    };

    /*
     * State
     */
    Entry entries_[NodeID::Max];  // [1, NodeID::Max]

    Multiset<INodeInfoListener*, 2> listeners_;

    ServiceClient<protocol::GetNodeInfo, GetNodeInfoResponseCallback> get_node_info_client_;

    mutable uint8_t last_picked_node_;

    uint8_t num_attempts_;

    /*
     * Methods
     */
    static MonotonicDuration getTimerPollInterval() { return MonotonicDuration::fromMSec(100); }

    const Entry& getEntry(NodeID node_id) const { return const_cast<NodeInfoRetriever*>(this)->getEntry(node_id); }
    Entry&       getEntry(NodeID node_id)
    {
        if (node_id.get() < 1 || node_id.get() > NodeID::Max)
        {
            handleFatalError("NodeInfoRetriever NodeID");
        }
        return entries_[node_id.get() - 1];
    }

    void startTimerIfNotRunning()
    {
        if (!TimerBase::isRunning())
        {
            TimerBase::startPeriodic(getTimerPollInterval());
        }
    }

    NodeID pickNextNodeToQuery() const
    {
        for (unsigned iter_cnt_ = 0; iter_cnt_ < (sizeof(entries_) / sizeof(entries_[0])); iter_cnt_++) // Round-robin
        {
            last_picked_node_++;
            if (last_picked_node_ > NodeID::Max)
            {
                last_picked_node_ = 1;
            }
            UAVCAN_ASSERT((last_picked_node_ >= 1) &&
                          (last_picked_node_ <= NodeID::Max));

            const Entry& entry = getEntry(last_picked_node_);
            if (entry.request_needed && entry.updated_since_last_attempt)
            {
                UAVCAN_TRACE("NodeInfoRetriever", "Next node to query: %d", int(last_picked_node_));
                return NodeID(last_picked_node_);
            }
        }
        return NodeID();        // No node could be found
    }

    virtual void handleTimerEvent(const TimerEvent&)
    {
        if (get_node_info_client_.hasPendingCalls()) // If request is pending, this condition will fail every second time
        {
            return;     // TODO FIXME Concurrent calls!!
        }

        const NodeID next = pickNextNodeToQuery();
        if (next.isUnicast())
        {
            getEntry(next).updated_since_last_attempt = false;
            const int res = get_node_info_client_.call(next, protocol::GetNodeInfo::Request());
            if (res < 0)
            {
                get_node_info_client_.getNode().registerInternalFailure("NodeInfoRetriever GetNodeInfo call");
            }
        }
        else
        {
            bool requests_needed = false;
            for (uint8_t i = 1; i <= NodeID::Max; i++)
            {
                if (getEntry(i).request_needed)
                {
                    requests_needed = true;
                    break;
                }
            }
            if (!requests_needed)
            {
                TimerBase::stop();
            }
        }
    }

    virtual void handleNodeStatusChange(const NodeStatusChangeEvent& event)
    {
        const bool was_offline = !event.old_status.known ||
                                 (event.old_status.status_code == protocol::NodeStatus::STATUS_OFFLINE);

        const bool offline_now = !event.status.known ||
                                 (event.status.status_code == protocol::NodeStatus::STATUS_OFFLINE);

        if (was_offline || offline_now)
        {
            Entry& entry = getEntry(event.node_id);

            entry.request_needed = !offline_now;
            entry.num_attempts_made = 0;

            UAVCAN_TRACE("NodeInfoRetriever", "Offline status change: node ID %d, request needed: %d",
                         int(event.node_id.get()), int(entry.request_needed));

            if (entry.request_needed)
            {
                startTimerIfNotRunning();
            }
        }

        listeners_.forEach(
            GenericHandlerCaller<const NodeStatusChangeEvent&>(&INodeInfoListener::handleNodeStatusChange, event));
    }

    virtual void handleNodeStatusMessage(const ReceivedDataStructure<protocol::NodeStatus>& msg)
    {
        Entry& entry = getEntry(msg.getSrcNodeID());

        if (msg.uptime_sec < entry.uptime_sec)
        {
            entry.request_needed = true;
            entry.num_attempts_made = 0;

            startTimerIfNotRunning();
        }
        entry.uptime_sec = msg.uptime_sec;
        entry.updated_since_last_attempt = true;

        listeners_.forEach(GenericHandlerCaller<const ReceivedDataStructure<protocol::NodeStatus>&>(
            &INodeInfoListener::handleNodeStatusMessage, msg));
    }

    void handleGetNodeInfoResponse(const ServiceCallResult<protocol::GetNodeInfo>& result)
    {
        Entry& entry = getEntry(result.getCallID().server_node_id);

        if (result.isSuccessful())
        {
            /*
             * Updating the uptime here allows to properly handle a corner case where the service response arrives
             * after the device has restarted and published its new NodeStatus (although it's unlikely to happen).
             */
            entry.uptime_sec = result.getResponse().status.uptime_sec;
            entry.request_needed = false;
            listeners_.forEach(NodeInfoRetrievedHandlerCaller(result.getCallID().server_node_id,
                                                              result.getResponse()));
        }
        else
        {
            if (num_attempts_ != UnlimitedRequestAttempts)
            {
                entry.num_attempts_made++;
                if (entry.num_attempts_made >= num_attempts_)
                {
                    entry.request_needed = false;
                    listeners_.forEach(GenericHandlerCaller<NodeID>(&INodeInfoListener::handleNodeInfoUnavailable,
                                                                    result.getCallID().server_node_id));
                }
            }
        }
    }

public:
    NodeInfoRetriever(INode& node)
        : NodeStatusMonitor(node)
        , TimerBase(node)
        , listeners_(node.getAllocator())
        , get_node_info_client_(node)
        , last_picked_node_(1)
        , num_attempts_(DefaultNumRequestAttempts)
    { }

    /**
     * Starts the retriever.
     * Destroy the object to stop it.
     * Returns negative error code.
     */
    int start()
    {
        int res = NodeStatusMonitor::start();
        if (res < 0)
        {
            return res;
        }

        res = get_node_info_client_.init();
        if (res < 0)
        {
            return res;
        }
        get_node_info_client_.setCallback(GetNodeInfoResponseCallback(this,
                                                                      &NodeInfoRetriever::handleGetNodeInfoResponse));
        // Note: the timer will be started ad-hoc
        return 0;
    }

    /**
     * Adds one listener. Does nothing if such listener already exists.
     * May return -ErrMemory if there's no space to add the listener.
     */
    int addListener(INodeInfoListener* listener)
    {
        if (listener != NULL)
        {
            removeListener(listener);
            return (NULL == listeners_.emplace(listener)) ? -ErrMemory : 0;
        }
        else
        {
            return -ErrInvalidParam;
        }
    }

    /**
     * Removes the listener.
     * If the listener was not registered, nothing will be done.
     */
    void removeListener(INodeInfoListener* listener)
    {
        if (listener != NULL)
        {
            listeners_.removeAll(listener);
        }
        else
        {
            UAVCAN_ASSERT(0);
        }
    }

    /**
     * Number of attempts to retrieve GetNodeInfo response before giving up on the assumption that the service is
     * not implemented.
     * Zero is a special value that can be used to set unlimited number of attempts, @ref UnlimitedRequestAttempts.
     */
    uint8_t getNumRequestAttempts() const { return num_attempts_; }
    void setNumRequestAttempts(const uint8_t num)
    {
        num_attempts_ = min(static_cast<uint8_t>(MaxNumRequestAttempts), num);
    }
};

}

#endif // Include guard
