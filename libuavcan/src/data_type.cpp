/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdio>
#include <cassert>
#include <uavcan/internal/data_type.hpp>

namespace uavcan
{
/*
 * DataTypeSignature
 */
void DataTypeSignature::mixin64(uint64_t x)
{
    DataTypeSignatureCRC crc = DataTypeSignatureCRC::extend(value_);
    for (int i = 0; i < 64; i += 8)   // LSB first
        crc.add((x >> i) & 0xFF);
    value_ = crc.get();
}

void DataTypeSignature::extend(DataTypeSignature dts)
{
    const uint64_t y = value_;
    mixin64(dts.get());
    mixin64(y);
}

TransferCRC DataTypeSignature::toTransferCRC() const
{
    TransferCRC tcrc;
    for (int i = 0; i < 64; i += 8)    // LSB first
        tcrc.add((value_ >> i) & 0xFF);
    return tcrc;
}

}
