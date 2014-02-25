/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <stdint.h>

namespace uavcan
{

/**
 * CRC-16-CCITT
 * Initial value: 0x0000
 * Coefficient: 0x1021
 */
class TransportCRC
{
    static const uint16_t Table[256];
    uint16_t value_;

public:
    enum { NumBytes = 2 };

    TransportCRC()
    : value_(0x0000)
    { }

    TransportCRC(const uint8_t* bytes, unsigned int len)
    : value_(0x0000)
    {
        add(bytes, len);
    }

    uint16_t add(uint8_t byte);
    uint16_t add(const uint8_t* bytes, unsigned int len);

    uint16_t get() const { return value_; }
};

}
