/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/protocol/dynamic_node_id_client.hpp>

namespace uavcan
{

void DynamicNodeIDClient::terminate()
{
    UAVCAN_TRACE("DynamicNodeIDClient", "Client terminated");
    stop();
    dnida_sub_.stop();
}

void DynamicNodeIDClient::handleTimerEvent(const TimerEvent&)
{
    // This method implements Rule B
    UAVCAN_ASSERT(preferred_node_id_.isValid());
    if (isAllocationComplete())
    {
        UAVCAN_ASSERT(0);
        terminate();
        return;
    }

    // Filling the message
    protocol::dynamic_node_id::Allocation msg;
    msg.node_id = preferred_node_id_.get();
    msg.first_part_of_unique_id = true;

    msg.unique_id.resize(protocol::dynamic_node_id::Allocation::MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST);
    copy(unique_id_, unique_id_ + msg.unique_id.size(), msg.unique_id.begin());
    UAVCAN_ASSERT(equal(msg.unique_id.begin(), msg.unique_id.end(), unique_id_));

    // Broadcasting
    UAVCAN_TRACE("DynamicNodeIDClient", "Broadcasting 1st stage: preferred ID: %d",
                 static_cast<int>(preferred_node_id_.get()));
    const int res = dnida_pub_.broadcast(msg);
    if (res < 0)
    {
        dnida_pub_.getNode().registerInternalFailure("DynamicNodeIDClient request failed");
    }
}

void DynamicNodeIDClient::handleAllocation(const ReceivedDataStructure<protocol::dynamic_node_id::Allocation>& msg)
{
    /*
     * TODO This method can blow the stack easily
     */
    UAVCAN_ASSERT(preferred_node_id_.isValid());
    if (isAllocationComplete())
    {
        UAVCAN_ASSERT(0);
        terminate();
        return;
    }

    startPeriodic(getPeriod()); // Restarting the timer - Rule C
    UAVCAN_TRACE("DynamicNodeIDClient", "Request timer reset because of Allocation message from %d",
                 static_cast<int>(msg.getSrcNodeID().get()));

    // Rule D
    if (!msg.isAnonymousTransfer() &&
        msg.unique_id.size() > 0 &&
        msg.unique_id.size() < msg.unique_id.capacity() &&
        equal(msg.unique_id.begin(), msg.unique_id.end(), unique_id_))
    {
        // Filling the response message
        const uint8_t size_of_unique_id_in_response =
            min(protocol::dynamic_node_id::Allocation::MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST,
                static_cast<uint8_t>(msg.unique_id.capacity() - msg.unique_id.size()));

        protocol::dynamic_node_id::Allocation second_stage;
        second_stage.node_id = preferred_node_id_.get();
        second_stage.first_part_of_unique_id = false;

        second_stage.unique_id.resize(size_of_unique_id_in_response);

        copy(unique_id_ + msg.unique_id.size(),
             unique_id_ + msg.unique_id.size() + size_of_unique_id_in_response,
             second_stage.unique_id.begin());

        UAVCAN_ASSERT(equal(second_stage.unique_id.begin(),
                            second_stage.unique_id.end(),
                            unique_id_ + msg.unique_id.size()));

        // Broadcasting the response
        UAVCAN_TRACE("DynamicNodeIDClient", "Broadcasting 2nd stage: preferred ID: %d, size of unique ID: %d",
                     static_cast<int>(preferred_node_id_.get()), static_cast<int>(second_stage.unique_id.size()));
        const int res = dnida_pub_.broadcast(second_stage);
        if (res < 0)
        {
            dnida_pub_.getNode().registerInternalFailure("DynamicNodeIDClient request failed");
        }
    }

    // Rule E
    if (!msg.isAnonymousTransfer() &&
        msg.unique_id.size() == msg.unique_id.capacity() &&
        equal(msg.unique_id.begin(), msg.unique_id.end(), unique_id_) &&
        msg.node_id > 0)
    {
        allocated_node_id_ = msg.node_id;
        allocator_node_id_ = msg.getSrcNodeID();
        UAVCAN_TRACE("DynamicNodeIDClient", "Allocation complete, node ID %d provided by %d",
                     static_cast<int>(allocated_node_id_.get()), static_cast<int>(allocator_node_id_.get()));
        terminate();
        UAVCAN_ASSERT(isAllocationComplete());
    }
}

int DynamicNodeIDClient::start(const protocol::HardwareVersion& hardware_version,
                               const NodeID preferred_node_id,
                               const TransferPriority transfer_priority)
{
    terminate();

    // Allocation is not possible if node ID is already set
    if (dnida_pub_.getNode().getNodeID().isUnicast())
    {
        return -ErrLogic;
    }

    // Unique ID initialization & validation
    copy(hardware_version.unique_id.begin(), hardware_version.unique_id.end(), unique_id_);
    bool unique_id_is_zero = true;
    for (uint8_t i = 0; i < sizeof(unique_id_); i++)
    {
        if (unique_id_[i] != 0)
        {
            unique_id_is_zero = false;
            break;
        }
    }

    if (unique_id_is_zero)
    {
        return -ErrInvalidParam;
    }

    if (!preferred_node_id.isValid())  // Only broadcast and unicast are allowed
    {
        return -ErrInvalidParam;
    }

    // Initializing the fields
    preferred_node_id_ = preferred_node_id;
    allocated_node_id_ = NodeID();
    allocator_node_id_ = NodeID();
    UAVCAN_ASSERT(preferred_node_id_.isValid());
    UAVCAN_ASSERT(!allocated_node_id_.isValid());
    UAVCAN_ASSERT(!allocator_node_id_.isValid());

    // Initializing node objects - Rule A
    int res = dnida_pub_.init();
    if (res < 0)
    {
        return res;
    }
    dnida_pub_.allowAnonymousTransfers();
    dnida_pub_.setPriority(transfer_priority);

    res = dnida_sub_.start(AllocationCallback(this, &DynamicNodeIDClient::handleAllocation));
    if (res < 0)
    {
        return res;
    }
    dnida_sub_.allowAnonymousTransfers();

    startPeriodic(MonotonicDuration::fromMSec(protocol::dynamic_node_id::Allocation::DEFAULT_REQUEST_PERIOD_MS));

    return 0;
}

}
