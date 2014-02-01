/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <uavcan/internal/transport/transfer.hpp>

namespace uavcan
{

int TransferID::forwardDistance(TransferID rhs) const
{
    int d = int(rhs.get()) - int(get());
    if (d < 0)
        d += 1 << BITLEN;

    assert(((get() + d) & MAX) == rhs.get());
    return d;
}

}
