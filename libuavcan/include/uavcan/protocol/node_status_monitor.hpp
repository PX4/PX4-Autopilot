/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/util/method_binder.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/protocol/NodeStatus.hpp>

namespace uavcan
{
/**
 * This class implements the core functionality of a network monitor.
 * It can be extended by inheritance to add more complex logic, or used directly as is.
 */
class UAVCAN_EXPORT NodeStatusMonitor : protected TimerBase
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
    static const unsigned TimerPeriodMs100 = 5;

    typedef MethodBinder<NodeStatusMonitor*,
                         void (NodeStatusMonitor::*)(const ReceivedDataStructure<protocol::NodeStatus>&)>
            NodeStatusCallback;

    Subscriber<protocol::NodeStatus, NodeStatusCallback> sub_;

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

    Entry& getEntry(NodeID node_id) const;

    void changeNodeStatus(const NodeID node_id, const Entry new_entry_value);

    void handleNodeStatus(const ReceivedDataStructure<protocol::NodeStatus>& msg);

    virtual void handleTimerEvent(const TimerEvent&);

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
        : TimerBase(node)
        , sub_(node)
    { }

    virtual ~NodeStatusMonitor() { }

    /**
     * Starts the monitor.
     * Destroy the object to stop it.
     * Returns negative error code.
     */
    int start();

    /**
     * Make the node unknown.
     */
    void forgetNode(NodeID node_id);

    /**
     * Returns status of a given node.
     * Unknown nodes are considered offline.
     */
    NodeStatus getNodeStatus(NodeID node_id) const;

    /**
     * This helper method allows to quickly estimate the overall network health.
     * Status of the local node is not considered.
     * Returns an invalid Node ID value if there's no known nodes in the network.
     */
    NodeID findNodeWithWorstStatus() const;
};

}
