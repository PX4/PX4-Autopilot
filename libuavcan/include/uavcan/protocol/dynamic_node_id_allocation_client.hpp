/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_CLIENT_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_CLIENT_HPP_INCLUDED

#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/publisher.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/protocol/DynamicNodeIDAllocation.hpp>
#include <uavcan/protocol/HardwareVersion.hpp>

namespace uavcan
{
/**
 * This class implements client-side logic of dynamic node ID allocation procedure.
 *
 * Once started, the object will be publishing dynamic node ID allocation requests at the maximum frequency allowed
 * by the specification, until a Node ID is granted by the allocator. Note that if there are multiple responses,
 * Node ID from the first response will be taken, and all subsequent responses will be ignored.
 *
 * If the local node is equipped with redundant CAN interfaces, all of them will be used for publishing requests
 * and listening for responses.
 *
 * Once dynamic allocation is complete (or not needed anymore), the object can be deleted.
 */
class DynamicNodeIDAllocationClient : private TimerBase
{
    typedef MethodBinder<DynamicNodeIDAllocationClient*,
                         void (DynamicNodeIDAllocationClient::*)
                             (const ReceivedDataStructure<protocol::DynamicNodeIDAllocation>&)>
        DynamicNodeIDAllocationCallback;

    Publisher<protocol::DynamicNodeIDAllocation> dnida_pub_;
    Subscriber<protocol::DynamicNodeIDAllocation, DynamicNodeIDAllocationCallback> dnida_sub_;

    uint64_t short_unique_id_;
    NodeID preferred_node_id_;
    NodeID allocated_node_id_;
    NodeID allocator_node_id_;

    virtual void handleTimerEvent(const TimerEvent&);

    void handleDynamicNodeIDAllocation(const ReceivedDataStructure<protocol::DynamicNodeIDAllocation>& msg);

    int startImpl();

public:
    DynamicNodeIDAllocationClient(INode& node)
        : TimerBase(node)
        , dnida_pub_(node)
        , dnida_sub_(node)
        , short_unique_id_(0)
    { }

    /**
     * Starts the client with a pre-computed short unique Node ID.
     * @param short_unique_node_id      The unique ID, only lower 57 bits of it will be used.
     * @param preferred_node_id         Node ID that the application would like to take; default is any.
     * @return                          Zero on success
     *                                  Negative error code on failure
     *                                  -ErrLogic if the node is not in passive mode (i.e. allocation is meaningless)
     *                                  -ErrInvalidParam if the supplied short unique ID is invalid
     */
    int start(uint64_t short_unique_node_id, NodeID preferred_node_id = NodeID::Broadcast);

    /**
     * This overload computes a short unique Node ID using data from uavcan.protocol.HardwareVersion structure.
     * @param hardware_version          Hardware version information, where unique_id must be set correctly.
     * @param preferred_node_id         Node ID that the application would like to take; default is any.
     * @return                          Refer to the overload for details
     *                                  -ErrInvalidParam if short unique ID could not be computed
     */
    int start(const protocol::HardwareVersion& hardware_version, NodeID preferred_node_id = NodeID::Broadcast);

    /**
     * This method allows to retrieve the value that was allocated to the local node.
     * If no value was allocated yet, the returned Node ID will be invalid (non-unicast).
     * Tip: use getAllocatedNodeID().isUnicast() to check if allocation is complete.
     * @return          If allocation is complete, a valid unicast Node ID will be returned.
     *                  If allocation is not complete yet, an invalid Node ID will be returned.
     */
    NodeID getAllocatedNodeID() const { return allocated_node_id_; }

    /**
     * This method allows to retrieve node ID of the allocator that granted our Node ID.
     * If no Node ID was allocated yet, the returned Node ID will be invalid (non-unicast).
     * @return          If allocation is complete, a valid unicast Node ID will be returned.
     *                  If allocation is not complete yet, an invalid Node ID will be returned.
     */
    NodeID getAllocatorNodeID() const { return allocator_node_id_; }

    /**
     * This utility method simply returns the short node ID used in the allocation request messages.
     * Returned value will be zero if the short unique node ID has not been initialized yet.
     */
    uint64_t getShortUniqueNodeID() const { return short_unique_id_; }
};

}

#endif // UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_CLIENT_HPP_INCLUDED
