/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/internal/marshalling/types.hpp>


struct CustomType
{
    typedef uavcan::IntegerSpec<8, uavcan::SignednessSigned, uavcan::CastModeTruncate> A;
    typedef uavcan::FloatSpec<16, uavcan::CastModeSaturate> B;
    // Dynamic array of max len 5 --> 3 bits for len, 5 bits for data --> 1 byte max len
    typedef uavcan::Array<uavcan::IntegerSpec<1, uavcan::SignednessUnsigned, uavcan::CastModeSaturate>,
                          uavcan::ArrayModeDynamic, 5> C;

    enum { MinBitLen = A::MinBitLen + B::MinBitLen + C::MinBitLen };
    enum { MaxBitLen = A::MaxBitLen + B::MaxBitLen + C::MaxBitLen };

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


TEST(Array, IntegerBitLen)
{
    using uavcan::IntegerBitLen;

    ASSERT_EQ(0, IntegerBitLen<0>::Result);
    ASSERT_EQ(1, IntegerBitLen<1>::Result);
    ASSERT_EQ(6, IntegerBitLen<42>::Result);
    ASSERT_EQ(8, IntegerBitLen<232>::Result);
    ASSERT_EQ(32, IntegerBitLen<0x81234567>::Result);
}


TEST(Array, Basic)
{
    using uavcan::Array;
    using uavcan::ArrayModeDynamic;
    using uavcan::ArrayModeStatic;
    using uavcan::IntegerSpec;
    using uavcan::FloatSpec;
    using uavcan::SignednessSigned;
    using uavcan::SignednessUnsigned;
    using uavcan::CastModeSaturate;
    using uavcan::CastModeTruncate;

    typedef Array<IntegerSpec<8, SignednessSigned, CastModeTruncate>, ArrayModeStatic, 4> A1;
    typedef Array<FloatSpec<16, CastModeSaturate>, ArrayModeStatic, 2> A2;
    typedef Array<CustomType, ArrayModeStatic, 2> A3;

    A1 a1;
    A2 a2;
    A3 a3;

    ASSERT_EQ(1, A3::ValueType::C::RawValueType::BitLen);

    ASSERT_EQ(8 * 4, A1::MaxBitLen);
    ASSERT_EQ(16 * 2, A2::MaxBitLen);
    ASSERT_EQ((8 + 16 + 5 + 3) * 2, A3::MaxBitLen);

    /*
     * Zero initialization check
     */
    ASSERT_FALSE(a1.empty());
    for (A1::const_iterator it = a1.begin(); it != a1.end(); ++it)
        ASSERT_EQ(0, *it);

    ASSERT_FALSE(a2.empty());
    for (A2::const_iterator it = a2.begin(); it != a2.end(); ++it)
        ASSERT_EQ(0, *it);

    for (A3::const_iterator it = a3.begin(); it != a3.end(); ++it)
    {
        ASSERT_EQ(0, it->a);
        ASSERT_EQ(0, it->b);
        ASSERT_EQ(0, it->c.size());
        ASSERT_TRUE(it->c.empty());
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
        for (int i2 = 0; i2 < 5; i2++)
            a3[i].c.push_back(i2 & 1);
        ASSERT_EQ(5, a3[i].c.size());
        ASSERT_FALSE(a3[i].c.empty());
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
        "00000000 00000000 00000000 10101010 " // A3[0] (0, 0, bool[5])
        "00000001 00000000 00111100 10101010"; // A3[1] (1, 1, bool[5])

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

    ASSERT_EQ(0, a1.front());
    ASSERT_EQ(3, a1.back());

    // Boolean vector
    std::vector<bool> v2;
    v2.push_back(false);
    v2.push_back(true);
    v2.push_back(false);
    v2.push_back(true);
    v2.push_back(false);

    ASSERT_TRUE(a3[0].c == v2);
    ASSERT_FALSE(a3[0].c == v1);
    ASSERT_FALSE(a3[0].c != v2);
    ASSERT_TRUE(a3[0].c != v1);

    v2[0] = true;
    ASSERT_TRUE(a3[0].c != v2);
    ASSERT_FALSE(a3[0].c == v2);
}


TEST(Array, Dynamic)
{
    using uavcan::Array;
    using uavcan::ArrayModeDynamic;
    using uavcan::ArrayModeStatic;
    using uavcan::IntegerSpec;
    using uavcan::FloatSpec;
    using uavcan::SignednessSigned;
    using uavcan::SignednessUnsigned;
    using uavcan::CastModeSaturate;
    using uavcan::CastModeTruncate;

    typedef Array<IntegerSpec<1, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 5> A;
    typedef Array<IntegerSpec<8, SignednessSigned, CastModeSaturate>, ArrayModeDynamic, 255> B;

    A a;
    B b;

    ASSERT_EQ(3 + 5, A::MaxBitLen);
    ASSERT_EQ(8 + 255 * 8, B::MaxBitLen);

    ASSERT_TRUE(a.empty());
    ASSERT_TRUE(b.empty());

    {
        uavcan::StaticTransferBuffer<16> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);

        ASSERT_EQ(1, A::encode(a, sc_wr));
        ASSERT_EQ(1, B::encode(b, sc_wr));

        ASSERT_EQ("000" "00000 000" "00000", bs_wr.toString());

        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);

        ASSERT_EQ(1, A::decode(a, sc_rd));
        ASSERT_EQ(1, B::decode(b, sc_rd));

        ASSERT_TRUE(a.empty());
        ASSERT_TRUE(b.empty());
    }

    a.push_back(true);
    a.push_back(false);
    a.push_back(true);
    a.push_back(false);
    a.push_back(true);

    b.push_back(42);
    b.push_back(-42);

    {
        uavcan::StaticTransferBuffer<16> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);

        ASSERT_EQ(1, A::encode(a, sc_wr));
        ASSERT_EQ(1, B::encode(b, sc_wr));

        ASSERT_EQ("10110101 00000010 00101010 11010110", bs_wr.toString());

        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);

        a.clear();
        b.clear();
        ASSERT_TRUE(a.empty());
        ASSERT_TRUE(b.empty());

        ASSERT_EQ(1, A::decode(a, sc_rd));
        ASSERT_EQ(1, B::decode(b, sc_rd));

        ASSERT_TRUE(a[0]);
        ASSERT_FALSE(a[1]);
        ASSERT_TRUE(a[2]);
        ASSERT_FALSE(a[3]);
        ASSERT_TRUE(a[4]);

        ASSERT_EQ(42, b[0]);
        ASSERT_EQ(-42, b[1]);
    }

    ASSERT_FALSE(a == b);
    ASSERT_FALSE(b == a);
    ASSERT_TRUE(a != b);
    ASSERT_TRUE(b != a);

    a.resize(0);
    b.resize(0);
    ASSERT_TRUE(a.empty());
    ASSERT_TRUE(b.empty());

    a.resize(5, true);
    b.resize(255, 72);
    ASSERT_EQ(5, a.size());
    ASSERT_EQ(255, b.size());

    for (int i = 0; i < 5; i++)
        ASSERT_TRUE(a[i]);

    for (int i = 0; i < 255; i++)
        ASSERT_EQ(72, b[i]);
}
