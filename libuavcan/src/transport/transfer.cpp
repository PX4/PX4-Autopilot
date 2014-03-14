/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <cstdio>
#include <sstream>
#include <uavcan/transport/frame.hpp>
#include <uavcan/transport/can_io.hpp>

namespace uavcan
{
/**
 * NodeID
 */
const NodeID NodeID::Broadcast(ValueBroadcast);

/**
 * TransferID
 */
int TransferID::forwardDistance(TransferID rhs) const
{
    int d = int(rhs.get()) - int(get());
    if (d < 0)
        d += 1 << BitLen;

    assert(((get() + d) & Max) == rhs.get());
    return d;
}

}
