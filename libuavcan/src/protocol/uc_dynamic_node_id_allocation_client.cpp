/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/protocol/dynamic_node_id_allocation_client.hpp>
#include <uavcan/data_type.hpp>  // For CRC64

namespace uavcan
{

void DynamicNodeIDAllocationClient::handleTimerEvent(const TimerEvent&)
{
    UAVCAN_ASSERT(!allocated_node_id_.isUnicast());
    UAVCAN_ASSERT(preferred_node_id_.isUnicast());

    protocol::DynamicNodeIDAllocation msg;
    msg.short_unique_id = short_unique_id_;
    msg.node_id = preferred_node_id_.get();

    UAVCAN_TRACE("DynamicNodeIDAllocation", "Broadcasting a request: short unique ID 0x%016llx, preferred ID %d",
                 static_cast<unsigned long long>(short_unique_id_),
                 static_cast<int>(preferred_node_id_.get()));

    const int res = dnida_pub_.broadcast(msg);
    if (res < 0)
    {
        dnida_pub_.getNode().registerInternalFailure("DynamicNodeIDAllocation pub failed");
    }
}

void DynamicNodeIDAllocationClient::handleDynamicNodeIDAllocation(
    const ReceivedDataStructure<protocol::DynamicNodeIDAllocation>& msg)
{
    if ((msg.short_unique_id != short_unique_id_) || msg.isRogueTransfer())
    {
        return;  // Not ours
    }

    if (allocated_node_id_.isUnicast())
    {
        UAVCAN_TRACE("DynamicNodeIDAllocation", "Redundant response from %d",
                     static_cast<int>(msg.getSrcNodeID().get()));
        return;  // Allocation is already done
    }

    const NodeID allocation(msg.node_id);
    if (!allocation.isUnicast())
    {
        dnida_sub_.getNode().registerInternalFailure("DynamicNodeIDAllocation bad node ID allocated");
        return;
    }

    allocated_node_id_ = allocation;
    allocator_node_id_ = msg.getSrcNodeID();

    UAVCAN_TRACE("DynamicNodeIDAllocation", "Allocation done: requested %d, received %d from %d",
                 static_cast<int>(preferred_node_id_.get()),
                 static_cast<int>(allocated_node_id_.get()),
                 static_cast<int>(msg.getSrcNodeID().get()));

    TimerBase::stop();
}

int DynamicNodeIDAllocationClient::startImpl()
{
    short_unique_id_ &= protocol::DynamicNodeIDAllocation::FieldTypes::short_unique_id::mask();

    if ((short_unique_id_ == 0) || !preferred_node_id_.isUnicast())
    {
        // It's not like a zero unique ID is not unique enough, but it's surely suspicious
        return -ErrInvalidParam;
    }

    UAVCAN_TRACE("DynamicNodeIDAllocationClient", "Short unique node ID: 0x%016llx, preferred node ID: %d",
                 static_cast<unsigned long long>(short_unique_id_), static_cast<int>(preferred_node_id_.get()));

    allocated_node_id_ = NodeID();
    UAVCAN_ASSERT(!allocated_node_id_.isUnicast());
    UAVCAN_ASSERT(!allocated_node_id_.isValid());

    int res = dnida_pub_.init();
    if (res < 0)
    {
        return res;
    }
    dnida_pub_.allowRogueTransfers();

    res = dnida_sub_.start(
        DynamicNodeIDAllocationCallback(this, &DynamicNodeIDAllocationClient::handleDynamicNodeIDAllocation));
    if (res < 0)
    {
        return res;
    }
    dnida_sub_.allowRogueTransfers();

    startPeriodic(
        MonotonicDuration::fromMSec(protocol::DynamicNodeIDAllocation::ALLOCATEE_MIN_BROADCAST_INTERVAL_SEC * 1000));

    return 0;
}

int DynamicNodeIDAllocationClient::start(const uint64_t short_unique_node_id, const NodeID preferred_node_id)
{
    short_unique_id_ = short_unique_node_id;
    preferred_node_id_ = preferred_node_id;
    return startImpl();
}

int DynamicNodeIDAllocationClient::start(const protocol::HardwareVersion& hardware_version,
                                         const NodeID preferred_node_id)
{
    // Checking if unique ID is set
    bool unique_id_is_zero = true;
    for (uint8_t i = 0; i < hardware_version.unique_id.size(); i++)
    {
        if (hardware_version.unique_id[i] != 0)
        {
            unique_id_is_zero = false;
            break;
        }
    }

    if (unique_id_is_zero)
    {
        return -ErrInvalidParam;
    }

    // Reducing the ID to 64 bits according to the specification
    DataTypeSignatureCRC crc;
    crc.add(hardware_version.unique_id.begin(), hardware_version.unique_id.size());
    short_unique_id_ = crc.get();

    preferred_node_id_ = preferred_node_id;

    return startImpl();
}

}
