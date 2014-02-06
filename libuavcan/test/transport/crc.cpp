/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <algorithm>
#include <gtest/gtest.h>
#include <uavcan/internal/transport/crc.hpp>


TEST(Crc16, Correctness)
{
    using uavcan::Crc16;

    Crc16 crc;

    ASSERT_EQ(0x0000, crc.get());

    crc.add('1');
    crc.add('2');
    crc.add('3');
    ASSERT_EQ(38738, crc.get());

    crc.add(reinterpret_cast<const uint8_t*>("Foobar"), 6);
    ASSERT_EQ(53881, crc.get());
}
