/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <stdint.h>

namespace uavcan
{

/**
 * CRC-16-CCITT
 * Initial value: 0xFFFF
 * Poly: 0x1021
 * Reverse: no
 * Output xor: 0
 *
 * import crcmod
 * crc = crcmod.predefined.Crc('crc-ccitt-false')
 * crc.update('123456789')
 * crc.hexdigest()
 * '29B1'
 */
class TransferCRC
{
    static const uint16_t Table[256];
    uint16_t value_;

public:
    enum { NumBytes = 2 };

    TransferCRC()
    : value_(0xFFFF)
    { }

    void add(uint8_t byte)
    {
        value_ = ((value_ << 8) ^ Table[((value_ >> 8) ^ byte) & 0xFF]) & 0xFFFF;
    }

    void add(const uint8_t* bytes, unsigned int len)
    {
        assert(bytes);
        while (len--)
            add(*bytes++);
    }

    uint16_t get() const { return value_; }
};

}
