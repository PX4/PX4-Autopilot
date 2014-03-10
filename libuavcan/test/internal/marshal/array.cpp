/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/internal/marshal/types.hpp>

using uavcan::Array;
using uavcan::ArrayModeDynamic;
using uavcan::ArrayModeStatic;
using uavcan::IntegerSpec;
using uavcan::FloatSpec;
using uavcan::SignednessSigned;
using uavcan::SignednessUnsigned;
using uavcan::CastModeSaturate;
using uavcan::CastModeTruncate;

struct CustomType
{
    typedef uavcan::IntegerSpec<8, uavcan::SignednessSigned, uavcan::CastModeTruncate> A;
    typedef uavcan::FloatSpec<16, uavcan::CastModeSaturate> B;
    // Dynamic array of max len 5 --> 3 bits for len, 5 bits for data --> 1 byte max len
    typedef uavcan::Array<uavcan::IntegerSpec<1, uavcan::SignednessUnsigned, uavcan::CastModeSaturate>,
                          uavcan::ArrayModeDynamic, 5> C;

    enum { MinBitLen = A::MinBitLen + B::MinBitLen + C::MinBitLen };
    enum { MaxBitLen = A::MaxBitLen + B::MaxBitLen + C::MaxBitLen };

    typename uavcan::StorageType<A>::Type a;
    typename uavcan::StorageType<B>::Type b;
    typename uavcan::StorageType<C>::Type c;

    CustomType() : a(), b(), c() { }

    bool operator==(const CustomType& rhs) const { return a == rhs.a && b == rhs.b && c == rhs.c; }

    static int encode(const CustomType& obj, uavcan::ScalarCodec& codec,
                      uavcan::TailArrayOptimizationMode tao_mode = uavcan::TailArrayOptEnabled)
    {
        int res = 0;

        res = A::encode(obj.a, codec, uavcan::TailArrayOptDisabled);
        if (res <= 0)
            return res;

        res = B::encode(obj.b, codec, uavcan::TailArrayOptDisabled);
        if (res <= 0)
            return res;

        res = C::encode(obj.c, codec, tao_mode);
        if (res <= 0)
            return res;

        return 1;
    }

    static int decode(CustomType& obj, uavcan::ScalarCodec& codec,
                      uavcan::TailArrayOptimizationMode tao_mode = uavcan::TailArrayOptEnabled)
    {
        int res = 0;

        res = A::decode(obj.a, codec, uavcan::TailArrayOptDisabled);
        if (res <= 0)
            return res;

        res = B::decode(obj.b, codec, uavcan::TailArrayOptDisabled);
        if (res <= 0)
            return res;

        res = C::decode(obj.c, codec, tao_mode);
        if (res <= 0)
            return res;

        return 1;
    }
};


TEST(Array, Basic)
{
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
     * Note that TAO in A3 is not possible because A3::C has less than one byte per item
     */
    uavcan::StaticTransferBuffer<16> buf;
    uavcan::BitStream bs_wr(buf);
    uavcan::ScalarCodec sc_wr(bs_wr);

    ASSERT_EQ(1, A1::encode(a1, sc_wr, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, A2::encode(a2, sc_wr, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, A3::encode(a3, sc_wr, uavcan::TailArrayOptEnabled));

    ASSERT_EQ(0, A3::encode(a3, sc_wr, uavcan::TailArrayOptEnabled));  // Out of buffer space

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

    ASSERT_EQ(1, A1::decode(a1_, sc_rd, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, A2::decode(a2_, sc_rd, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, A3::decode(a3_, sc_rd, uavcan::TailArrayOptEnabled));

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

    ASSERT_EQ(0, A3::decode(a3_, sc_rd, uavcan::TailArrayOptEnabled));  // Out of buffer space

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
    ASSERT_TRUE(v1 == a1);
    ASSERT_FALSE(v1 != a1);
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
    typedef Array<IntegerSpec<1, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 5> A;
    typedef Array<IntegerSpec<8, SignednessSigned, CastModeSaturate>, ArrayModeDynamic, 255> B;

    A a;
    B b;
    B b2;

    ASSERT_EQ(3 + 5, A::MaxBitLen);
    ASSERT_EQ(8 + 255 * 8, B::MaxBitLen);

    ASSERT_TRUE(a.empty());
    ASSERT_TRUE(b.empty());
    ASSERT_TRUE(b2.empty());

    {
        uavcan::StaticTransferBuffer<16> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);

        ASSERT_EQ(1, A::encode(a, sc_wr, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::encode(b, sc_wr, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::encode(b2, sc_wr, uavcan::TailArrayOptEnabled));

        ASSERT_EQ("000" "00000 000" "00000", bs_wr.toString()); // Last array was optimized away completely

        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);

        ASSERT_EQ(1, A::decode(a, sc_rd, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::decode(b, sc_rd, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::decode(b2, sc_rd, uavcan::TailArrayOptEnabled));

        ASSERT_TRUE(a.empty());
        ASSERT_TRUE(b.empty());
        ASSERT_TRUE(b2.empty());
    }

    a.push_back(true);
    a.push_back(false);
    a.push_back(true);
    a.push_back(false);
    a.push_back(true);

    b.push_back(42);
    b.push_back(-42);

    b2.push_back(123);
    b2.push_back(72);

    {
        uavcan::StaticTransferBuffer<16> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);

        ASSERT_EQ(1, A::encode(a, sc_wr, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::encode(b, sc_wr, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::encode(b2, sc_wr, uavcan::TailArrayOptEnabled));  // No length field

        //         A        B len    B[0]     B[1]     B2[0]    B2[1]
        ASSERT_EQ("10110101 00000010 00101010 11010110 01111011 01001000", bs_wr.toString());

        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);

        a.clear();
        b.clear();
        b2.clear();
        ASSERT_TRUE(a.empty());
        ASSERT_TRUE(b.empty());
        ASSERT_TRUE(b2.empty());

        ASSERT_EQ(1, A::decode(a, sc_rd, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::decode(b, sc_rd, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::decode(b2, sc_rd, uavcan::TailArrayOptEnabled));

        ASSERT_EQ(5, a.size());
        ASSERT_EQ(2, b.size());
        ASSERT_EQ(2, b2.size());

        ASSERT_TRUE(a[0]);
        ASSERT_FALSE(a[1]);
        ASSERT_TRUE(a[2]);
        ASSERT_FALSE(a[3]);
        ASSERT_TRUE(a[4]);

        ASSERT_EQ(42, b[0]);
        ASSERT_EQ(-42, b[1]);

        ASSERT_EQ(123, b2[0]);
        ASSERT_EQ(72, b2[1]);
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


template <typename B>
struct CustomType2
{
    typedef uavcan::FloatSpec<16, uavcan::CastModeSaturate> A;

    enum { MinBitLen = A::MinBitLen + B::MinBitLen };
    enum { MaxBitLen = A::MaxBitLen + B::MaxBitLen };

    typename uavcan::StorageType<A>::Type a;
    typename uavcan::StorageType<B>::Type b;

    CustomType2() : a(), b() { }

    bool operator==(const CustomType2& rhs) const { return a == rhs.a && b == rhs.b; }

    static int encode(const CustomType2& obj, uavcan::ScalarCodec& codec,
                      uavcan::TailArrayOptimizationMode tao_mode = uavcan::TailArrayOptEnabled)
    {
        int res = 0;
        res = A::encode(obj.a, codec, uavcan::TailArrayOptDisabled);
        if (res <= 0)
            return res;
        res = B::encode(obj.b, codec, tao_mode);
        if (res <= 0)
            return res;
        return 1;
    }

    static int decode(CustomType2& obj, uavcan::ScalarCodec& codec,
                      uavcan::TailArrayOptimizationMode tao_mode = uavcan::TailArrayOptEnabled)
    {
        int res = 0;
        res = A::decode(obj.a, codec, uavcan::TailArrayOptDisabled);
        if (res <= 0)
            return res;
        res = B::decode(obj.b, codec, tao_mode);
        if (res <= 0)
            return res;
        return 1;
    }
};


template <typename T>
static std::string runEncodeDecode(const typename uavcan::StorageType<T>::Type& value,
                                   const uavcan::TailArrayOptimizationMode tao_mode)
{
    uavcan::StaticTransferBuffer<(T::MaxBitLen + 7) / 8> buf;
    uavcan::BitStream bs_wr(buf);
    uavcan::ScalarCodec sc_wr(bs_wr);
    EXPECT_EQ(1, T::encode(value, sc_wr, tao_mode));

    typename uavcan::StorageType<T>::Type value2 = typename uavcan::StorageType<T>::Type();
    // Decode multiple times to make sure that the decoded type is being correctly de-initialized
    for (int i = 0; i < 3; i++)
    {
        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);
        EXPECT_EQ(1, T::decode(value2, sc_rd, tao_mode));
        EXPECT_EQ(value, value2);
    }
    return bs_wr.toString();
}


TEST(Array, TailArrayOptimization)
{
    typedef Array<IntegerSpec<1, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 5>   OneBitArray;
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 255> EightBitArray;
    typedef CustomType2<Array<OneBitArray,   ArrayModeDynamic, 255> > A;
    typedef CustomType2<Array<EightBitArray, ArrayModeDynamic, 255> > B;
    typedef CustomType2<EightBitArray> C;

    A a;
    B b;
    C c;

    /*
     * Empty
     */
    //         a LSB    a MSB    b len
    ASSERT_EQ("00000000 00000000 00000000", runEncodeDecode<A>(a, uavcan::TailArrayOptEnabled));
    ASSERT_EQ("00000000 00000000 00000000", runEncodeDecode<A>(a, uavcan::TailArrayOptDisabled));

    //         a LSB    a MSB    b len
    ASSERT_EQ("00000000 00000000 00000000", runEncodeDecode<B>(b, uavcan::TailArrayOptEnabled));
    ASSERT_EQ("00000000 00000000 00000000", runEncodeDecode<B>(b, uavcan::TailArrayOptDisabled));

    //         a LSB    a MSB
    ASSERT_EQ("00000000 00000000",          runEncodeDecode<C>(c, uavcan::TailArrayOptEnabled));
    ASSERT_EQ("00000000 00000000 00000000", runEncodeDecode<C>(c, uavcan::TailArrayOptDisabled));

    /*
     * A
     */
    a.b.resize(2);
    a.b[0].push_back(true);
    a.b[0].push_back(false);
    // a.b[1] remains empty
    //         a LSB    a MSB    b len    b: len(2), 1, 0, len(0)
    ASSERT_EQ("00000000 00000000 00000010 01010000", runEncodeDecode<A>(a, uavcan::TailArrayOptEnabled));
    ASSERT_EQ("00000000 00000000 00000010 01010000", runEncodeDecode<A>(a, uavcan::TailArrayOptDisabled));

    /*
     * B
     */
    b.b.resize(3);
    b.b[0].push_back(42);
    b.b[0].push_back(72);
    // b.b[1] remains empty
    b.b[2].push_back(123);
    b.b[2].push_back(99);
    //         a LSB    a MSB    b len    b[0]len  42       72       b[1]len  123      99      (b[2] len optimized out)
    ASSERT_EQ("00000000 00000000 00000011 00000010 00101010 01001000 00000000 01111011 01100011",
              runEncodeDecode<B>(b, uavcan::TailArrayOptEnabled));
    // Same as above, but b[2] len is present                                 v here v
    ASSERT_EQ("00000000 00000000 00000011 00000010 00101010 01001000 00000000 00000010 01111011 01100011",
              runEncodeDecode<B>(b, uavcan::TailArrayOptDisabled));

    /*
     * C
     */
    c.a = 1;
    c.b.push_back(1);
    c.b.push_back(2);
    c.b.push_back(3);
    //         a LSB    a MSB    1        2        3
    ASSERT_EQ("00000000 00111100 00000001 00000010 00000011",
              runEncodeDecode<C>(c, uavcan::TailArrayOptEnabled));
    //         a LSB    a MSB    b len    1        2        3
    ASSERT_EQ("00000000 00111100 00000011 00000001 00000010 00000011",
              runEncodeDecode<C>(c, uavcan::TailArrayOptDisabled));
}


TEST(Array, TailArrayOptimizationErrors)
{
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 5> A;

    A a;
    ASSERT_TRUE(a.empty());
    ASSERT_EQ("",         runEncodeDecode<A>(a, uavcan::TailArrayOptEnabled));
    ASSERT_EQ("00000000", runEncodeDecode<A>(a, uavcan::TailArrayOptDisabled));

    // Correct decode/encode
    a.push_back(1);
    a.push_back(126);
    a.push_back(5);
    ASSERT_FALSE(a.empty());
    ASSERT_EQ("00000001 01111110 00000101",          runEncodeDecode<A>(a, uavcan::TailArrayOptEnabled));
    ASSERT_EQ("01100000 00101111 11000000 10100000", runEncodeDecode<A>(a, uavcan::TailArrayOptDisabled));

    // Invalid decode - length field is out of range
    uavcan::StaticTransferBuffer<7> buf;
    uavcan::BitStream bs_wr(buf);
    uavcan::ScalarCodec sc_wr(bs_wr);

    ASSERT_EQ(1, sc_wr.encode<3>(uint8_t(6)));  // Length - more than 5 items, error
    ASSERT_EQ(1, sc_wr.encode<8>(uint8_t(42)));
    ASSERT_EQ(1, sc_wr.encode<8>(uint8_t(72)));
    ASSERT_EQ(1, sc_wr.encode<8>(uint8_t(126)));
    ASSERT_EQ(1, sc_wr.encode<8>(uint8_t(1)));
    ASSERT_EQ(1, sc_wr.encode<8>(uint8_t(2)));
    ASSERT_EQ(1, sc_wr.encode<8>(uint8_t(3)));  // Out of range - only 5 items allowed

    //         197      73       15       192      32       ...
    ASSERT_EQ("11000101 01001001 00001111 11000000 00100000 01000000 01100000", bs_wr.toString());

    {
        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);
        A a2;
        a2.push_back(56);   // Garbage
        ASSERT_EQ(1, a2.size());
        // Will fail - declared length is more than 5 items
        ASSERT_EQ(-1, A::decode(a2, sc_rd, uavcan::TailArrayOptDisabled));
        // Must be cleared
        ASSERT_TRUE(a2.empty());
    }
    {
        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);
        A a2;
        a2.push_back(56);   // Garbage
        ASSERT_EQ(1, a2.size());
        // Will fail - no length field, but the stream is too long
        ASSERT_EQ(-1, A::decode(a2, sc_rd, uavcan::TailArrayOptEnabled));
        // Will contain some garbage
        ASSERT_EQ(5, a2.size());
        // Interpreted stream - see the values above
        ASSERT_EQ(197, a2[0]);
        ASSERT_EQ(73,  a2[1]);
        ASSERT_EQ(15,  a2[2]);
        ASSERT_EQ(192, a2[3]);
        ASSERT_EQ(32,  a2[4]);
    }
}


TEST(Array, DynamicEncodeDecodeErrors)
{
    typedef CustomType2<Array<Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>,
                              ArrayModeDynamic, 255>,
                        ArrayModeDynamic, 255> > A;
    A a;
    a.b.resize(2);
    a.b[0].push_back(55);
    a.b[0].push_back(66);
    {
        uavcan::StaticTransferBuffer<4> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);
        ASSERT_EQ(0, A::encode(a, sc_wr, uavcan::TailArrayOptEnabled));  // Not enough buffer space

        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);
        ASSERT_EQ(0, A::decode(a, sc_rd, uavcan::TailArrayOptEnabled));
    }
    {
        uavcan::StaticTransferBuffer<4> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);
        ASSERT_EQ(0, A::encode(a, sc_wr, uavcan::TailArrayOptDisabled));  // Not enough buffer space

        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);
        ASSERT_EQ(0, A::decode(a, sc_rd, uavcan::TailArrayOptDisabled));
    }
}


TEST(Array, StaticEncodeDecodeErrors)
{
    typedef CustomType2<Array<Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>,
                              ArrayModeStatic, 2>,
                        ArrayModeStatic, 2> > A;
    A a;
    a.a = 1.0;
    a.b[0][0] = 0x11;
    a.b[0][1] = 0x22;
    a.b[1][0] = 0x33;
    a.b[1][1] = 0x44;
    {   // Just enough buffer space - 6 bytes
        uavcan::StaticTransferBuffer<6> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);
        ASSERT_EQ(1, A::encode(a, sc_wr, uavcan::TailArrayOptDisabled));

        ASSERT_EQ("00000000 00111100 00010001 00100010 00110011 01000100", bs_wr.toString());

        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);
        ASSERT_EQ(1, A::decode(a, sc_rd, uavcan::TailArrayOptEnabled));
    }
    {   // Not enough space
        uavcan::StaticTransferBuffer<5> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);
        ASSERT_EQ(0, A::encode(a, sc_wr, uavcan::TailArrayOptDisabled));

        ASSERT_EQ("00000000 00111100 00010001 00100010 00110011", bs_wr.toString());

        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);
        ASSERT_EQ(0, A::decode(a, sc_rd, uavcan::TailArrayOptEnabled));
    }
}


TEST(Array, Copyability)
{
    typedef Array<IntegerSpec<1, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 5>   OneBitArray;
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 255> EightBitArray;
    typedef Array<OneBitArray,   ArrayModeDynamic, 255> A;
    typedef Array<EightBitArray, ArrayModeDynamic, 255> B;
    typedef EightBitArray C;

    A a;
    B b;
    C c;

    A a2 = a;
    B b2 = b;
    C c2 = c;

    ASSERT_TRUE(a == a2);
    ASSERT_TRUE(b == b2);
    ASSERT_TRUE(c == c2);

    a.push_back(OneBitArray());
    b.push_back(EightBitArray());
    c.push_back(42);

    ASSERT_TRUE(a != a2);
    ASSERT_TRUE(b != b2);
    ASSERT_TRUE(c != c2);

    a2 = a;
    b2 = b;
    c2 = c;

    ASSERT_TRUE(a2 == a);
    ASSERT_TRUE(b2 == b);
    ASSERT_TRUE(c2 == c);
}


TEST(Array, Strings)
{
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 32> A8;
    typedef Array<IntegerSpec<7, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 32> A7;

    A8 a8;
    A8 a8_2;
    A7 a7;

    ASSERT_TRUE(a8 == a7);
    // cppcheck-suppress duplicateExpression
    ASSERT_TRUE(a8 == a8);
    // cppcheck-suppress duplicateExpression
    ASSERT_TRUE(a7 == a7);
    ASSERT_TRUE(a8 == "");
    ASSERT_TRUE(a7 == "");

    a8 = "Hello world!";
    a7 = "123";
    ASSERT_TRUE(a8 == "Hello world!");
    ASSERT_TRUE(a7 == "123");

    a8 = "Our sun is dying.";
    a7 = "456";
    ASSERT_TRUE("Our sun is dying." == a8);
    ASSERT_TRUE("456" == a7);

    a8 += " 123456";
    a8 += "-789";
    ASSERT_TRUE("Our sun is dying. 123456-789" == a8);

    ASSERT_TRUE(a8_2 == "");
    ASSERT_TRUE(a8_2.empty());
    ASSERT_TRUE(a8_2 != a8);
    a8_2 = a8;
    ASSERT_TRUE(a8_2 == "Our sun is dying. 123456-789");
    ASSERT_TRUE(a8_2 == a8);

    /*
     * c_str()
     */
    ASSERT_STREQ("", A8().c_str());
    ASSERT_STREQ("", A7().c_str());
    ASSERT_STREQ("Our sun is dying. 123456-789", a8_2.c_str());
    ASSERT_STREQ("Our sun is dying. 123456-789", a8.c_str());
    ASSERT_STREQ("456", a7.c_str());
}


TEST(Array, FlatStreaming)
{
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 32> A8D;
    typedef Array<FloatSpec<16, CastModeSaturate>, ArrayModeDynamic, 16> AF16D;
    typedef Array<FloatSpec<16, CastModeSaturate>, ArrayModeStatic, 3> AF16S;

    A8D a1;
    a1 = "12\n3\x44\xa5\xde\xad\x79";
    uavcan::YamlStreamer<A8D>::stream(std::cout, a1, 0);
    std::cout << std::endl;

    A8D a2;
    a2 = "Hello";
    uavcan::YamlStreamer<A8D>::stream(std::cout, a2, 0);
    std::cout << std::endl;

    AF16D af16d1;
    af16d1.push_back(1.23);
    af16d1.push_back(4.56);
    uavcan::YamlStreamer<AF16D>::stream(std::cout, af16d1, 0);
    std::cout << std::endl;

    AF16D af16d2;
    uavcan::YamlStreamer<AF16D>::stream(std::cout, af16d2, 0);
    std::cout << std::endl;

    AF16S af16s;
    uavcan::YamlStreamer<AF16S>::stream(std::cout, af16s, 0);
    std::cout << std::endl;
}


TEST(Array, MultidimensionalStreaming)
{
    typedef Array<FloatSpec<16, CastModeSaturate>, ArrayModeDynamic, 16> Float16Array;
    typedef Array<Float16Array, ArrayModeDynamic, 8> TwoDimensional;
    typedef Array<TwoDimensional, ArrayModeDynamic, 4> ThreeDimensional;

    ThreeDimensional threedee;
    threedee.resize(3);
    for (int x = 0; x < threedee.size(); x++)
    {
        threedee[x].resize(3);
        for (int y = 0; y < threedee[x].size(); y++)
        {
            threedee[x][y].resize(3);
            for (int z = 0; z < threedee[x][y].size(); z++)
                threedee[x][y][z] = 1.0 / (x + y + z + 1.0);
        }
    }

    uavcan::YamlStreamer<ThreeDimensional>::stream(std::cout, threedee, 0);
    std::cout << std::endl;
}
