/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/internal/marshalling/types.hpp>


struct CustomType
{
    typedef uavcan::IntegerSpec<8, uavcan::SignednessSigned, uavcan::CastModeTruncate> A;
    typedef uavcan::FloatSpec<16, uavcan::CastModeSaturate> B;
    typedef uavcan::StaticArray<uavcan::IntegerSpec<1, uavcan::SignednessUnsigned, uavcan::CastModeSaturate>, 8> C;

    uavcan::StorageType<A>::Type a;
    uavcan::StorageType<B>::Type b;
    uavcan::StorageType<C>::Type c;

    CustomType() : a(), b(), c() { }

    bool operator==(const CustomType& rhs) const { return a == rhs.a && b == rhs.b && c == rhs.c; }

    static int encode(const CustomType& obj, uavcan::ScalarCodec& codec)
    {
        int res = 0;

        res = A::encode(obj.a, codec);
        if (res <= 0)
            return res;

        res = B::encode(obj.b, codec);
        if (res <= 0)
            return res;

        res = C::encode(obj.c, codec);
        if (res <= 0)
            return res;

        return 1;
    }

    static int decode(CustomType& obj, uavcan::ScalarCodec& codec)
    {
        int res = 0;

        res = A::decode(obj.a, codec);
        if (res <= 0)
            return res;

        res = B::decode(obj.b, codec);
        if (res <= 0)
            return res;

        res = C::decode(obj.c, codec);
        if (res <= 0)
            return res;

        return 1;
    }
};


TEST(StaticArray, Basic)
{
    using uavcan::StaticArray;
    using uavcan::IntegerSpec;
    using uavcan::FloatSpec;
    using uavcan::SignednessSigned;
    using uavcan::SignednessUnsigned;
    using uavcan::CastModeSaturate;
    using uavcan::CastModeTruncate;

    typedef StaticArray<IntegerSpec<8, SignednessSigned, CastModeTruncate>, 4> A1;
    typedef StaticArray<FloatSpec<16, CastModeSaturate>, 2> A2;
    typedef StaticArray<CustomType, 2> A3;

    A1 a1;
    A2 a2;
    A3 a3;

    ASSERT_EQ(1, A3::ValueType::C::RawValueType::BitLen);

    /*
     * Zero initialization check
     */
    for (A1::const_iterator it = a1.begin(); it != a1.end(); ++it)
        ASSERT_EQ(0, *it);

    for (A2::const_iterator it = a2.begin(); it != a2.end(); ++it)
        ASSERT_EQ(0, *it);

    for (A3::const_iterator it = a3.begin(); it != a3.end(); ++it)
    {
        ASSERT_EQ(0, it->a);
        ASSERT_EQ(0, it->b);
        for (int i = 0; i < 8; i++)
            ASSERT_EQ(0, it->c[i]);
    }

    /*
     * Modification with known values; array lengths are hard coded.
     */
    for (int i = 0; i < 4; i++)
        a1.at(i) = i;

    for (int i = 0; i < 2; i++)
        a2.at(i) = i;

    for (int i = 0; i < 2; i++)
    {
        a3[i].a = i;
        a3[i].b = i;
        for (int i2 = 0; i2 < 8; i2++)
            a3[i].c[i2] = i2 & 1;
    }

    /*
     * Representation check
     */
    uavcan::StaticTransferBuffer<16> buf;
    uavcan::BitStream bs_wr(buf);
    uavcan::ScalarCodec sc_wr(bs_wr);

    ASSERT_EQ(1, A1::encode(a1, sc_wr));
    ASSERT_EQ(1, A2::encode(a2, sc_wr));
    ASSERT_EQ(1, A3::encode(a3, sc_wr));

    ASSERT_EQ(0, A3::encode(a3, sc_wr));  // Out of buffer space

    static const std::string Reference =
        "00000000 00000001 00000010 00000011 " // A1 (0, 1, 2, 3)
        "00000000 00000000 00000000 00111100 " // A2 (0, 1)
        "00000000 00000000 00000000 01010101 " // A3[0] (0, 0, bool[8])
        "00000001 00000000 00111100 01010101"; // A3[1] (1, 1, bool[8])

    ASSERT_EQ(Reference, bs_wr.toString());

    /*
     * Read back
     */
    uavcan::BitStream bs_rd(buf);
    uavcan::ScalarCodec sc_rd(bs_rd);

    A1 a1_;
    A2 a2_;
    A3 a3_;

    ASSERT_EQ(1, A1::decode(a1_, sc_rd));
    ASSERT_EQ(1, A2::decode(a2_, sc_rd));
    ASSERT_EQ(1, A3::decode(a3_, sc_rd));

    ASSERT_EQ(a1_, a1);
    ASSERT_EQ(a2_, a2);
    ASSERT_EQ(a3_, a3);

    for (int i = 0; i < 4; i++)
        ASSERT_EQ(a1[i], a1_[i]);

    for (int i = 0; i < 2; i++)
        ASSERT_EQ(a2[i], a2_[i]);

    for (int i = 0; i < 2; i++)
    {
        ASSERT_EQ(a3[i].a, a3_[i].a);
        ASSERT_EQ(a3[i].b, a3_[i].b);
        ASSERT_EQ(a3[i].c, a3_[i].c);
    }

    ASSERT_EQ(0, A3::decode(a3_, sc_rd));  // Out of buffer space

    /*
     * STL compatibility
     */
    std::vector<int> v1;
    v1.push_back(0);
    v1.push_back(1);
    v1.push_back(2);
    v1.push_back(3);

    ASSERT_TRUE(a1 == v1);
    ASSERT_FALSE(a1 != v1);
    ASSERT_FALSE(a1 < v1);

    v1[0] = 9000;
    ASSERT_FALSE(a1 == v1);
    ASSERT_TRUE(a1 != v1);
    ASSERT_TRUE(a1 < v1);
}
