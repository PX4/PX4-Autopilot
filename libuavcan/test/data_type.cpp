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


TEST(DataTypeSignatureCRC, Extension)
{
    uavcan::DataTypeSignatureCRC crc1;

    crc1.add('1');
    crc1.add('2');
    crc1.add('3');

    uavcan::DataTypeSignatureCRC crc2 = uavcan::DataTypeSignatureCRC::extend(crc1.get());

    crc2.add(reinterpret_cast<const uint8_t*>("456789"), 6);

    ASSERT_EQ(0x62EC59E3F1A4F00A, crc2.get());
}


TEST(DataTypeSignature, Correctness)
{
    using uavcan::DataTypeSignature;
    using uavcan::DataTypeSignatureCRC;

    DataTypeSignature signature = DataTypeSignature::zero();
    ASSERT_EQ(0, signature.get());

    /*
     * First extension
     */
    signature.extend(DataTypeSignature(0x123456789abcdef0));

    DataTypeSignatureCRC crc;
    crc.add(0xF0);
    crc.add(0xDE);
    crc.add(0xBC);
    crc.add(0x9A);
    crc.add(0x78);
    crc.add(0x56);
    crc.add(0x34);
    crc.add(0x12);
    for (int i = 0; i < 8; i++)
        crc.add(0);

    ASSERT_EQ(crc.get(), signature.get());

    const uint64_t old_signature = signature.get();

    /*
     * Second extension
     */
    signature.extend(DataTypeSignature(0xfedcba9876543210));
    crc.add(0x10);
    crc.add(0x32);
    crc.add(0x54);
    crc.add(0x76);
    crc.add(0x98);
    crc.add(0xba);
    crc.add(0xdc);
    crc.add(0xfe);
    for (int i = 0; i < 64; i += 8)
        crc.add((old_signature >> i) & 0xFF);

    ASSERT_EQ(crc.get(), signature.get());

    /*
     * Comparison
     */
    ASSERT_TRUE(signature == DataTypeSignature(signature.get()));
    ASSERT_FALSE(signature == DataTypeSignature::zero());
    ASSERT_FALSE(signature != DataTypeSignature(signature.get()));
    ASSERT_TRUE(signature != DataTypeSignature::zero());
}
