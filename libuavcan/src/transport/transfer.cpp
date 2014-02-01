/*
 * Copyright (C) 2014 <pavel.kirienko@gmail.com>
 */

#include <uavcan/internal/transport/transfer.hpp>

namespace uavcan
{

//static Frame Frame::parse(const CanFrame& can_frame)
//{
//}

int Frame::subtractTransferID(int rhs) const
{
    static const int RANGE = MAX_TRANSFER_ID + 1;  //  16  256
    static const int NEGATIVE = -RANGE / 2;        // -8  -128 (two's complement)
    static const int POSITIVE = (-NEGATIVE) - 1;   //  7   127

    const int d = int(this->transfer_id) - rhs;
    if (d <= NEGATIVE)
        return RANGE + d;
    else if (d >= POSITIVE)
        return d - RANGE;
    return d;
}

}
