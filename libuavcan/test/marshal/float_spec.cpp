/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/marshal/types.hpp>
#include <uavcan/transport/transfer_buffer.hpp>


TEST(FloatSpec, Limits)
{
    using uavcan::FloatSpec;
    using uavcan::CastModeSaturate;
    using uavcan::CastModeTruncate;

    typedef FloatSpec<16, CastModeSaturate> F16S;
    typedef FloatSpec<16, CastModeTruncate> F16T;
    typedef FloatSpec<32, CastModeSaturate> F32S;
    typedef FloatSpec<32, CastModeTruncate> F32T;
    typedef FloatSpec<64, CastModeSaturate> F64S;
    typedef FloatSpec<64, CastModeTruncate> F64T;

    ASSERT_FALSE(F16S::IsExactRepresentation);
    ASSERT_FLOAT_EQ(65504.0, F16S::max());
    ASSERT_FLOAT_EQ(9.77e-04, F16S::epsilon());

    ASSERT_TRUE(F32T::IsExactRepresentation);
    ASSERT_FLOAT_EQ(std::numeric_limits<float>::max(), F32T::max());
    ASSERT_FLOAT_EQ(std::numeric_limits<float>::epsilon(), F32T::epsilon());

    ASSERT_TRUE(F64S::IsExactRepresentation);
    ASSERT_FLOAT_EQ(std::numeric_limits<double>::max(), F64S::max());
    ASSERT_FLOAT_EQ(std::numeric_limits<double>::epsilon(), F64S::epsilon());
}

TEST(FloatSpec, Basic)
{
    using uavcan::FloatSpec;
    using uavcan::CastModeSaturate;
    using uavcan::CastModeTruncate;
    using uavcan::StorageType;

    typedef FloatSpec<16, CastModeSaturate> F16S;
    typedef FloatSpec<16, CastModeTruncate> F16T;
    typedef FloatSpec<32, CastModeSaturate> F32S;
    typedef FloatSpec<32, CastModeTruncate> F32T;
    typedef FloatSpec<64, CastModeSaturate> F64S;
    typedef FloatSpec<64, CastModeTruncate> F64T;

    static const long double Values[] =
    {
        0.0,
        1.0,
        M_PI,
        123,
        -123,
        99999,
        -999999,
        std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        nanl("")
    };
    static const int NumValues = sizeof(Values) / sizeof(Values[0]);

    static const long double ValuesF16S[] =
    {
        0.0,
        1.0,
        3.140625,
        123,
        -123,
        F16S::max(),
        -F16S::max(),
        F16S::max(),
        -F16S::max(),
        std::numeric_limits<F16S::StorageType>::infinity(),
        -std::numeric_limits<F16S::StorageType>::infinity(),
        nanl("")
    };
    static const long double ValuesF16T[] =
    {
        0.0,
        1.0,
        3.140625,
        123,
        -123,
        std::numeric_limits<F16S::StorageType>::infinity(),
        -std::numeric_limits<F16S::StorageType>::infinity(),
        std::numeric_limits<F16S::StorageType>::infinity(),
        -std::numeric_limits<F16S::StorageType>::infinity(),
        std::numeric_limits<F16S::StorageType>::infinity(),
        -std::numeric_limits<F16S::StorageType>::infinity(),
        nanl("")
    };

    /*
     * Writing
     */
    uavcan::StaticTransferBuffer<NumValues * (2 + 4 + 8) * 2> buf;
    uavcan::BitStream bs_wr(buf);
    uavcan::ScalarCodec sc_wr(bs_wr);

    for (int i = 0; i < NumValues; i++)
    {
        ASSERT_EQ(1, F16S::encode(Values[i], sc_wr, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, F16T::encode(Values[i], sc_wr, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, F32S::encode(Values[i], sc_wr, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, F32T::encode(Values[i], sc_wr, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, F64S::encode(Values[i], sc_wr, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, F64T::encode(Values[i], sc_wr, uavcan::TailArrayOptDisabled));
    }

    ASSERT_EQ(0, F16S::encode(0, sc_wr, uavcan::TailArrayOptDisabled));  // Out of buffer space now

    /*
     * Reading
     */
    uavcan::BitStream bs_rd(buf);
    uavcan::ScalarCodec sc_rd(bs_rd);

#define CHECK(FloatType, expected_value) \
    do { \
        StorageType<FloatType>::Type var = StorageType<FloatType>::Type(); \
        ASSERT_EQ(1, FloatType::decode(var, sc_rd, uavcan::TailArrayOptDisabled)); \
        if (!isnan(expected_value)) { \
            ASSERT_FLOAT_EQ(expected_value, var); } \
        else { \
            ASSERT_EQ(!!isnan(expected_value), !!isnan(var)); } \
    } while (0)

    for (int i = 0; i < NumValues; i++)
    {
        CHECK(F16S, ValuesF16S[i]);
        CHECK(F16T, ValuesF16T[i]);
        CHECK(F32S, Values[i]);
        CHECK(F32T, Values[i]);
        CHECK(F64S, Values[i]);
        CHECK(F64T, Values[i]);
    }

#undef CHECK
}

TEST(FloatSpec, Float16Representation)
{
    using uavcan::FloatSpec;
    using uavcan::CastModeSaturate;
    using uavcan::CastModeTruncate;

    typedef FloatSpec<16, CastModeSaturate> F16S;
    typedef FloatSpec<16, CastModeTruncate> F16T;

    uavcan::StaticTransferBuffer<2 * 6> buf;
    uavcan::BitStream bs_wr(buf);
    uavcan::ScalarCodec sc_wr(bs_wr);

    ASSERT_EQ(1, F16S::encode(0.0, sc_wr, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, F16S::encode(1.0, sc_wr, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, F16S::encode(-2.0, sc_wr, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, F16T::encode(999999, sc_wr, uavcan::TailArrayOptDisabled));  // +inf
    ASSERT_EQ(1, F16S::encode(-999999, sc_wr, uavcan::TailArrayOptDisabled)); // -max
    ASSERT_EQ(1, F16S::encode(nan(""), sc_wr, uavcan::TailArrayOptDisabled)); // nan

    ASSERT_EQ(0, F16S::encode(0, sc_wr, uavcan::TailArrayOptDisabled));  // Out of buffer space now

    // Keep in mind that this is LITTLE ENDIAN representation
    static const std::string Reference =
        "00000000 00000000 " // 0.0
        "00000000 00111100 " // 1.0
        "00000000 11000000 " // -2.0
        "00000000 01111100 " // +inf
        "11111111 11111011 " // -max
        "11111111 01111111"; // nan

    ASSERT_EQ(Reference, bs_wr.toString());
}
