/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_CLIENT_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_CLIENT_HPP_INCLUDED

#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/publisher.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/protocol/dynamic_node_id/Allocation.hpp>
#include <uavcan/protocol/HardwareVersion.hpp>

namespace uavcan
{
/**
 * This class implements client-side logic of dynamic node ID allocation procedure.
 *
 * Once started, the object will be publishing dynamic node ID allocation requests at the default frequency defined
 * by the specification, until a Node ID is granted by the allocator.
 *
 * If the local node is equipped with redundant CAN interfaces, all of them will be used for publishing requests
 * and listening for responses.
 *
 * Once dynamic allocation is complete (or not needed anymore), the object can be deleted.
 */
class UAVCAN_EXPORT DynamicNodeIDClient : private TimerBase
{
    typedef MethodBinder<DynamicNodeIDClient*,
                         void (DynamicNodeIDClient::*)
                             (const ReceivedDataStructure<protocol::dynamic_node_id::Allocation>&)>
        AllocationCallback;

    Publisher<protocol::dynamic_node_id::Allocation> dnida_pub_;
    Subscriber<protocol::dynamic_node_id::Allocation, AllocationCallback> dnida_sub_;

    uint8_t unique_id_[protocol::HardwareVersion::FieldTypes::unique_id::MaxSize];
    NodeID preferred_node_id_;
    NodeID allocated_node_id_;
    NodeID allocator_node_id_;

    void terminate();

    virtual void handleTimerEvent(const TimerEvent&);

    void handleAllocation(const ReceivedDataStructure<protocol::dynamic_node_id::Allocation>& msg);

public:
    DynamicNodeIDClient(INode& node)
        : TimerBase(node)
        , dnida_pub_(node)
        , dnida_sub_(node)
    { }

    /**
     * @param hardware_version  Hardware version information, where unique_id must be set correctly.
     * @param preferred_node_id Node ID that the application would like to take; set to broadcast (zero) if
     *                          the application doesn't have any preference (this is default).
     * @param transfer_priority Transfer priority, Normal by default.
     * @return                  Zero on success
     *                          Negative error code on failure
     *                          -ErrLogic if 1. the node is not in passive mode or 2. the client is already started
     */
    int start(const protocol::HardwareVersion& hardware_version,
              const NodeID preferred_node_id = NodeID::Broadcast,
              const TransferPriority transfer_priority = TransferPriorityNormal);

    /**
     * Use this method to determine when allocation is complete.
     */
    bool isAllocationComplete() const { return getAllocatedNodeID().isUnicast(); }

    /**
     * This method allows to retrieve the node ID that was allocated to the local node.
     * If no node ID was allocated yet, the returned node ID will be invalid (non-unicast).
     * @return          If allocation is complete, a valid unicast node ID will be returned.
     *                  If allocation is not complete yet, a non-unicast node ID will be returned.
     */
    NodeID getAllocatedNodeID() const { return allocated_node_id_; }

    /**
     * This method allows to retrieve node ID of the allocator that granted our Node ID.
     * If no node ID was allocated yet, the returned node ID will be invalid (non-unicast).
     * @return          If allocation is complete, a valid unicast node ID will be returned.
     *                  If allocation is not complete yet, an non-unicast node ID will be returned.
     */
    NodeID getAllocatorNodeID() const { return allocator_node_id_; }
};

}

#endif // UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_CLIENT_HPP_INCLUDED
