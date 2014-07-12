/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/transport/transfer.hpp>
#include <uavcan/transport/frame.hpp>
#include <uavcan/transport/can_io.hpp>

namespace uavcan
{
/**
 * TransferID
 */
const uint8_t TransferID::BitLen;
const uint8_t TransferID::Max;

/**
 * NodeID
 */
const uint8_t NodeID::ValueBroadcast;
const uint8_t NodeID::ValueInvalid;
const uint8_t NodeID::BitLen;
const uint8_t NodeID::Max;
const NodeID NodeID::Broadcast(ValueBroadcast);

/**
 * TransferID
 */
int TransferID::computeForwardDistance(TransferID rhs) const
{
    int d = int(rhs.get()) - int(get());
    if (d < 0)
    {
        d += 1 << BitLen;
    }

    UAVCAN_ASSERT(((get() + d) & Max) == rhs.get());
    return d;
}

}
