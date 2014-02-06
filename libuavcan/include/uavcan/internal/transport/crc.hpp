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
class Crc16
{
    static const uint16_t TABLE[256];
    uint_fast16_t value_;

public:
    Crc16()
    : value_(0x0000)
    { }

    uint16_t add(uint8_t byte);
    uint16_t add(const uint8_t* bytes, int len);

    uint16_t get() const { return value_; }
};

}
