/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_NODE_STATUS_MONITOR_HPP_INCLUDED
#define UAVCAN_PROTOCOL_NODE_STATUS_MONITOR_HPP_INCLUDED

#include <uavcan/debug.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <cassert>
#include <cstdlib>

namespace uavcan
{
/**
 * This class implements the core functionality of a network monitor.
 * It can be extended by inheritance to add more complex logic, or used directly as is.
 */
class UAVCAN_EXPORT NodeStatusMonitor
{
public:
    typedef typename StorageType<typename protocol::NodeStatus::FieldTypes::status_code>::Type NodeStatusCode;

    struct NodeStatus
    {
        NodeStatusCode status_code;  ///< Current status code; OFFLINE if not known.
        bool known;                  ///< True if the node was online at least once.

        NodeStatus()
            : status_code(protocol::NodeStatus::STATUS_OFFLINE)
            , known(false)
        { }
    };

    struct NodeStatusChangeEvent
    {
        NodeID node_id;
        NodeStatus status;
        NodeStatus old_status;
    };

private:
    enum { TimerPeriodMs100 = 4 };

    typedef MethodBinder<NodeStatusMonitor*,
                         void (NodeStatusMonitor::*)(const ReceivedDataStructure<protocol::NodeStatus>&)>
            NodeStatusCallback;

    typedef MethodBinder<NodeStatusMonitor*, void (NodeStatusMonitor::*)(const TimerEvent&)> TimerCallback;

    Subscriber<protocol::NodeStatus, NodeStatusCallback, MaxNetworkSizeHint, 0> sub_;

    TimerEventForwarder<TimerCallback> timer_;

    struct Entry
    {
        NodeStatusCode status_code;
        int8_t time_since_last_update_ms100;
        Entry()
            : status_code(protocol::NodeStatus::STATUS_OFFLINE)
            , time_since_last_update_ms100(-1)
        { }
    };

    mutable Entry entries_[NodeID::Max];  // [1, NodeID::Max]

    Entry& getEntry(NodeID node_id) const
    {
        if (node_id.get() < 1 || node_id.get() > NodeID::Max)
        {
            handleFatalError("NodeStatusMonitor NodeID");
        }
        return entries_[node_id.get() - 1];
    }

    void changeNodeStatus(const NodeID node_id, const Entry new_entry_value)
    {
        Entry& entry = getEntry(node_id);
        if (entry.status_code != new_entry_value.status_code)
        {
            NodeStatusChangeEvent event;
            event.node_id = node_id;

            event.old_status.known = entry.time_since_last_update_ms100 >= 0;
            event.old_status.status_code = entry.status_code;

            event.status.known = true;
            event.status.status_code = new_entry_value.status_code;

            UAVCAN_TRACE("NodeStatusMonitor", "Node %i [%s] status change: %i --> %i", int(node_id.get()),
                         (event.old_status.known ? "known" : "new"),
                         int(event.old_status.status_code), int(event.status.status_code));
            handleNodeStatusChange(event);
        }
        entry = new_entry_value;
    }

    void handleNodeStatus(const ReceivedDataStructure<protocol::NodeStatus>& msg)
    {
        Entry new_entry_value;
        new_entry_value.time_since_last_update_ms100 = 0;
        new_entry_value.status_code = msg.status_code;

        changeNodeStatus(msg.getSrcNodeID(), new_entry_value);

        handleNodeStatusMessage(msg);
    }

    void handleTimerEvent(const TimerEvent&)
    {
        const int OfflineTimeoutMs100 = protocol::NodeStatus::OFFLINE_TIMEOUT_MS / 100;

        for (uint8_t i = 1; i <= NodeID::Max; i++)
        {
            const NodeID nid(i);
            UAVCAN_ASSERT(nid.isUnicast());
            Entry& entry = getEntry(nid);
            if (entry.time_since_last_update_ms100 >= 0 &&
                entry.status_code != protocol::NodeStatus::STATUS_OFFLINE)
            {
                entry.time_since_last_update_ms100 =
                    int8_t(entry.time_since_last_update_ms100 + int8_t(TimerPeriodMs100));
                if (entry.time_since_last_update_ms100 >= OfflineTimeoutMs100)
                {
                    Entry new_entry_value;
                    new_entry_value.time_since_last_update_ms100 = OfflineTimeoutMs100;
                    new_entry_value.status_code = protocol::NodeStatus::STATUS_OFFLINE;
                    changeNodeStatus(nid, new_entry_value);
                }
            }
        }
    }

protected:
    /**
     * Called when a node becomes online, changes status or goes offline.
     * Refer to uavcan.protocol.NodeStatus for the offline timeout value.
     * Overriding is not required.
     */
    virtual void handleNodeStatusChange(const NodeStatusChangeEvent& event)
    {
        (void)event;
    }

    /**
     * Called for every received message uavcan.protocol.NodeStatus after handleNodeStatusChange(), even
     * if the status code did not change.
     * Overriding is not required.
     */
    virtual void handleNodeStatusMessage(const ReceivedDataStructure<protocol::NodeStatus>& msg)
    {
        (void)msg;
    }

public:
    explicit NodeStatusMonitor(INode& node)
        : sub_(node)
        , timer_(node)
    { }

    virtual ~NodeStatusMonitor() { }

    /**
     * Starts the monitor.
     * Destroy the object to stop it.
     * Returns negative error code.
     */
    int start()
    {
        const int res = sub_.start(NodeStatusCallback(this, &NodeStatusMonitor::handleNodeStatus));
        if (res >= 0)
        {
            timer_.setCallback(TimerCallback(this, &NodeStatusMonitor::handleTimerEvent));
            timer_.startPeriodic(MonotonicDuration::fromMSec(TimerPeriodMs100 * 100));
        }
        return res;
    }

    /**
     * Make the node unknown.
     */
    void forgetNode(NodeID node_id)
    {
        if (node_id.isValid())
        {
            Entry& entry = getEntry(node_id);
            entry = Entry();
        }
        else
        {
            UAVCAN_ASSERT(0);
        }
    }

    /**
     * Make all nodes unknown.
     */
    void forgetAllNodes()
    {
        for (unsigned i = 0; i < (sizeof(entries_) / sizeof(entries_[0])); i++)
        {
            entries_[i] = Entry();
        }
    }

    /**
     * Returns status of a given node.
     * Unknown nodes are considered offline.
     */
    NodeStatus getNodeStatus(NodeID node_id) const
    {
        if (!node_id.isValid())
        {
            UAVCAN_ASSERT(0);
            return NodeStatus();
        }
        NodeStatus status;
        const Entry& entry = getEntry(node_id);
        if (entry.time_since_last_update_ms100 >= 0)
        {
            status.known = true;
            status.status_code = entry.status_code;
        }
        return status;
    }

    /**
     * This helper method allows to quickly estimate the overall network health.
     * Status of the local node is not considered.
     * Returns an invalid Node ID value if there's no known nodes in the network.
     */
    NodeID findNodeWithWorstStatus() const
    {
        NodeID nid_with_worst_status;
        NodeStatusCode worst_status_code = protocol::NodeStatus::STATUS_OK;

        for (uint8_t i = 1; i <= NodeID::Max; i++)
        {
            const NodeID nid(i);
            UAVCAN_ASSERT(nid.isUnicast());
            const Entry& entry = getEntry(nid);
            if (entry.time_since_last_update_ms100 >= 0)
            {
                if (entry.status_code > worst_status_code || !nid_with_worst_status.isValid())
                {
                    nid_with_worst_status = nid;
                    worst_status_code = entry.status_code;
                }
            }
        }
        return nid_with_worst_status;
    }
};

}

#endif // UAVCAN_PROTOCOL_NODE_STATUS_MONITOR_HPP_INCLUDED
