/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/internal/data_type.hpp>


TEST(DataTypeSignatureCRC, Correctness)
{
    uavcan::DataTypeSignatureCRC crc;

    ASSERT_EQ(0xFFFFFFFFFFFFFFFF ^ 0xFFFFFFFFFFFFFFFF, crc.get());

    crc.add('1');
    crc.add('2');
    crc.add('3');
    crc.add(reinterpret_cast<const uint8_t*>("456789"), 6);

    ASSERT_EQ(0x62EC59E3F1A4F00A, crc.get());
}
