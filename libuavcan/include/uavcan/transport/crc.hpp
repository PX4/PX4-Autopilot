/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/stdint.hpp>
#include <uavcan/impl_constants.hpp>

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
class UAVCAN_EXPORT TransferCRC
{
#if !UAVCAN_TINY
    static const uint16_t Table[256];
#endif

    uint16_t value_;

public:
    enum { NumBytes = 2 };

    TransferCRC()
        : value_(0xFFFF)
    { }

#if UAVCAN_TINY
    void add(uint8_t byte)
    {
        value_ ^= byte << 8;
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (value_ & 0x8000)
            {
                value_ = (value_ << 1) ^ 0x1021;
            }
            else
            {
                value_ = (value_ << 1);
            }
        }
    }
#else
    void add(uint8_t byte)
    {
        value_ = ((value_ << 8) ^ Table[((value_ >> 8) ^ byte) & 0xFF]) & 0xFFFF;
    }
#endif

    void add(const uint8_t* bytes, unsigned len)
    {
        assert(bytes);
        while (len--)
        {
            add(*bytes++);
        }
    }

    uint16_t get() const { return value_; }
};

}
