// This software is distributed under the terms of the MIT License.
// Copyright (c) 2016-2020 UAVCAN Development Team.

#include "canard_dsdl.h"
#include "exposed.hpp"
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

TEST_CASE("float16Pack")
{
    using exposed::float16Pack;
    REQUIRE(0b0000000000000000 == float16Pack(0.0F));
    REQUIRE(0b0011110000000000 == float16Pack(1.0F));
    REQUIRE(0b1100000000000000 == float16Pack(-2.0F));
    REQUIRE(0b0111110000000000 == float16Pack(999999.0F));  // +inf
    REQUIRE(0b1111101111111111 == float16Pack(-65519.0F));  // -max

    // These are intrusive tests, they make assumptions about the specific implementation of the conversion logic.
    // Normally, one wouldn't be able to compare a NaN against a particular number because there are many ways to
    // represent it. We do not differentiate between sNaN and qNaN because there is no platform-agnostic way to do
    // that; see https://github.com/UAVCAN/nunavut/pull/115#issuecomment-704185463
    REQUIRE(0b0111111000000000 == float16Pack(+std::numeric_limits<CanardDSDLFloat32>::quiet_NaN()));
    REQUIRE(0b1111111000000000 == float16Pack(-std::numeric_limits<CanardDSDLFloat32>::quiet_NaN()));
    REQUIRE(0b0111111000000000 == float16Pack(+std::numeric_limits<CanardDSDLFloat32>::signaling_NaN()));
    REQUIRE(0b1111111000000000 == float16Pack(-std::numeric_limits<CanardDSDLFloat32>::signaling_NaN()));
}

TEST_CASE("float16Unpack")
{
    using exposed::float16Unpack;
    REQUIRE(Approx(0.0F) == float16Unpack(0b0000000000000000));
    REQUIRE(Approx(1.0F) == float16Unpack(0b0011110000000000));
    REQUIRE(Approx(-2.0F) == float16Unpack(0b1100000000000000));
    REQUIRE(Approx(-65504.0F) == float16Unpack(0b1111101111111111));
    REQUIRE(std::isinf(float16Unpack(0b0111110000000000)));

    const auto explode_sign_exp_mantissa = [](const float f) -> std::tuple<bool, std::uint8_t, std::uint32_t> {
        std::uint32_t n = 0;
        std::memcpy(&n, &f, 4);
        return std::make_tuple((n & (1UL << 31U)) != 0,
                               static_cast<std::uint8_t>((n >> 23U) & 0xFFU),
                               n & ((1UL << 23U) - 1U));
    };

    {
        const auto [sign, exp, man] = explode_sign_exp_mantissa(float16Unpack(0b0111111111111111));  // +qNaN
        REQUIRE(!sign);
        REQUIRE(exp == 0xFFU);
        REQUIRE(man != 0);
    }
    {
        const auto [sign, exp, man] = explode_sign_exp_mantissa(float16Unpack(0b0111111000000000));  // +qNaN
        REQUIRE(!sign);
        REQUIRE(exp == 0xFFU);
        REQUIRE(man != 0);
    }
    {
        const auto [sign, exp, man] = explode_sign_exp_mantissa(float16Unpack(0b1111111111111111));  // -qNaN
        REQUIRE(sign);
        REQUIRE(exp == 0xFFU);
        REQUIRE(man != 0);
    }
    {
        const auto [sign, exp, man] = explode_sign_exp_mantissa(float16Unpack(0b0111110111111111));  // +sNaN
        REQUIRE(!sign);
        REQUIRE(exp == 0xFFU);
        REQUIRE(man != 0);
    }
    {
        const auto [sign, exp, man] = explode_sign_exp_mantissa(float16Unpack(0b1111110000000001));  // -sNaN
        REQUIRE(sign);
        REQUIRE(exp == 0xFFU);
        REQUIRE(man != 0);
    }

    REQUIRE(bool(std::isnan(float16Unpack(0b0111111111111111))));  // +quiet
    REQUIRE(bool(std::isnan(float16Unpack(0b0111111000000000))));  // +quiet
    REQUIRE(bool(std::isnan(float16Unpack(0b1111111111111111))));  // -quiet
    REQUIRE(bool(std::isnan(float16Unpack(0b1111111000000000))));  // -quiet
    REQUIRE(bool(std::isnan(float16Unpack(0b0111110111111111))));  // +signaling
    REQUIRE(bool(std::isnan(float16Unpack(0b0111110000000001))));  // +signaling
    REQUIRE(bool(std::isnan(float16Unpack(0b1111110111111111))));  // -signaling
    REQUIRE(bool(std::isnan(float16Unpack(0b1111110000000001))));  // -signaling
}

TEST_CASE("canardDSDLFloat16")
{
    using exposed::float16Pack;
    using exposed::float16Unpack;
    float x = -1000.0F;
    while (x <= 1000.0F)
    {
        REQUIRE(Approx(x) == float16Unpack(float16Pack(x)));
        x += 0.5F;
    }

    REQUIRE(0b0111110000000000 == float16Pack(float16Unpack(0b0111110000000000)));  // +inf
    REQUIRE(0b1111110000000000 == float16Pack(float16Unpack(0b1111110000000000)));  // -inf

    // These are intrusive tests, they make assumptions about the specific implementation of the conversion logic.
    // Normally, one wouldn't be able to compare a NaN against a particular number because there are many ways to
    // represent it. We do not differentiate between sNaN and qNaN because there is no platform-agnostic way to do
    // that; see https://github.com/UAVCAN/nunavut/pull/115#issuecomment-704185463
    REQUIRE(0b0111111000000000 == float16Pack(float16Unpack(0b0111111111111111)));  // +qNaN, extra bits stripped
    REQUIRE(0b0111111000000000 == float16Pack(float16Unpack(0b0111110111111111)));  // +sNaN, extra bits stripped
    REQUIRE(0b1111111000000000 == float16Pack(float16Unpack(0b1111111111111111)));  // -qNaN, extra bits stripped
    REQUIRE(0b1111111000000000 == float16Pack(float16Unpack(0b1111110111111111)));  // -sNaN, extra bits stripped
}

TEST_CASE("canardDSDLCopyBits")
{
    {
        uint8_t a = 0;
        uint8_t b = 0;
        canardDSDLCopyBits(0, 0, 0, &a, &b);
    }

    const auto test = [&](const size_t                     length_bit,
                          const size_t                     src_offset_bit,
                          const size_t                     dst_offset_bit,
                          const std::vector<std::uint8_t>& src,
                          const std::vector<std::uint8_t>& dst,
                          const std::vector<std::uint8_t>& ref) {
        REQUIRE(length_bit <= (dst.size() * 8));
        REQUIRE(length_bit <= (src.size() * 8));
        std::vector<std::uint8_t> result = dst;
        canardDSDLCopyBits(length_bit, src_offset_bit, dst_offset_bit, src.data(), result.data());
        return std::equal(std::begin(ref), std::end(ref), std::begin(result));
    };

    REQUIRE(test(8, 0, 0, {0xFF}, {0x00}, {0xFF}));
    REQUIRE(test(16, 0, 0, {0xFF, 0xFF}, {0x00, 0x00}, {0xFF, 0xFF}));
    REQUIRE(test(12, 0, 0, {0xFF, 0x0A}, {0x55, 0x00}, {0xFF, 0x0A}));
    REQUIRE(test(12, 0, 0, {0xFF, 0x0A}, {0x00, 0xF0}, {0xFF, 0xFA}));
    REQUIRE(test(12, 0, 4, {0xFF, 0x0A}, {0x53, 0x55}, {0xF3, 0xAF}));
    REQUIRE(test(8, 4, 4, {0x55, 0x55}, {0xAA, 0xAA}, {0x5A, 0xA5}));
}

TEST_CASE("canardDSDLSerialize_aligned")
{
    // The reference values for the following test have been taken from the PyUAVCAN test suite.
    const std::vector<std::uint8_t> Reference({0xA7, 0xEF, 0xCD, 0xAB, 0x90, 0x78, 0x56, 0x34, 0x12, 0x88, 0xA9, 0xCB,
                                               0xED, 0xFE, 0xFF, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0,
                                               0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x7C, 0xDA, 0x0E, 0xDA, 0xBE, 0xFE,
                                               0x01, 0xAD, 0xDE, 0xEF, 0xBE, 0xC5, 0x67, 0xC5, 0x0B});

    std::vector<std::uint8_t> dest(std::size(Reference));

    const auto set_b = [&](const std::size_t offset_bit, const bool value) {
        canardDSDLSetBit(dest.data(), offset_bit, value);
    };
    const auto set_u = [&](const std::size_t offset_bit, const std::uint64_t value, const std::uint8_t length_bit) {
        canardDSDLSetUxx(dest.data(), offset_bit, value, length_bit);
    };
    const auto set_i = [&](const std::size_t offset_bit, const std::int64_t value, const std::uint8_t length_bit) {
        canardDSDLSetIxx(dest.data(), offset_bit, value, length_bit);
    };
    const auto set_f16 = [&](const std::size_t offset_bit, const float value) {
        canardDSDLSetF16(dest.data(), offset_bit, value);
    };
    const auto set_f32 = [&](const std::size_t offset_bit, const float value) {
        canardDSDLSetF32(dest.data(), offset_bit, value);
    };
    const auto set_f64 = [&](const std::size_t offset_bit, const double value) {
        canardDSDLSetF64(dest.data(), offset_bit, value);
    };

    set_u(0, 0b1010'0111, 8);
    set_i(8, 0x1234'5678'90ab'cdef, 64);
    set_i(72, -0x1234'5678, 32);
    set_i(104, -2, 16);
    set_u(120, 0, 8);
    set_i(128, 127, 8);
    set_f64(136, 1.0);
    set_f32(200, 1.0F);
    set_f16(232, 99999.9F);
    set_u(248, 0xBEDA, 12);  // Truncation
    set_u(260, 0, 4);
    set_u(264, 0xBEDA, 16);
    set_i(280, -2, 9);
    set_i(289, 0, 7);
    set_u(296, 0xDEAD, 16);
    set_u(312, 0xBEEF, 16);

    std::size_t offset   = 328;
    const auto  push_bit = [&](const bool value) {
        set_b(offset, value);
        ++offset;
    };

    push_bit(true);
    push_bit(false);
    push_bit(true);
    push_bit(false);
    push_bit(false);
    push_bit(false);
    push_bit(true);
    push_bit(true);
    push_bit(true);
    push_bit(true);
    push_bit(true);
    push_bit(false);
    push_bit(false);
    push_bit(true);
    push_bit(true);
    push_bit(false);

    push_bit(true);
    push_bit(false);
    push_bit(true);
    push_bit(false);
    push_bit(false);
    push_bit(false);
    push_bit(true);
    push_bit(true);
    push_bit(true);
    push_bit(true);
    push_bit(false);
    push_bit(true);
    push_bit(false);

    set_u(357, 0, 3);

    REQUIRE(std::size(dest) == std::size(Reference));
    REQUIRE_THAT(dest, Catch::Matchers::Equals(Reference));
}

TEST_CASE("canardDSDLSerialize_unaligned")
{
    // The reference values for the following test have been taken from the PyUAVCAN test suite.
    const std::vector<std::uint8_t> Reference({
        0xC5, 0x2F, 0x57, 0x82, 0xC6, 0xCA, 0x12, 0x34, 0x56, 0xD9, 0xBF, 0xEC, 0x06, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x80, 0xFF, 0x01, 0x00, 0x00, 0xFC, 0x01, 0xE0, 0x6F, 0xF5, 0x7E, 0xF7, 0x05  //
    });

    std::vector<std::uint8_t> dest(std::size(Reference));

    const auto set_b = [&](const std::size_t offset_bit, const bool value) {
        canardDSDLSetBit(dest.data(), offset_bit, value);
    };
    const auto set_u = [&](const std::size_t offset_bit, const std::uint64_t value, const std::uint8_t length_bit) {
        canardDSDLSetUxx(dest.data(), offset_bit, value, length_bit);
    };
    const auto set_i = [&](const std::size_t offset_bit, const std::int64_t value, const std::uint8_t length_bit) {
        canardDSDLSetIxx(dest.data(), offset_bit, value, length_bit);
    };
    const auto set_f16 = [&](const std::size_t offset_bit, const float value) {
        canardDSDLSetF16(dest.data(), offset_bit, value);
    };
    const auto set_f32 = [&](const std::size_t offset_bit, const float value) {
        canardDSDLSetF32(dest.data(), offset_bit, value);
    };
    const auto set_f64 = [&](const std::size_t offset_bit, const double value) {
        canardDSDLSetF64(dest.data(), offset_bit, value);
    };

    std::size_t offset   = 0;
    const auto  push_bit = [&](const bool value) {
        set_b(offset, value);
        ++offset;
    };

    push_bit(true);
    push_bit(false);
    push_bit(true);
    push_bit(false);
    push_bit(false);
    push_bit(false);
    push_bit(true);
    push_bit(true);
    push_bit(true);
    push_bit(true);
    push_bit(true);

    push_bit(true);
    push_bit(false);
    push_bit(true);
    push_bit(false);
    push_bit(false);
    push_bit(true);
    push_bit(true);
    push_bit(true);
    push_bit(false);
    push_bit(true);

    REQUIRE_THAT(std::vector<std::uint8_t>(std::begin(dest), std::begin(dest) + 2),
                 Catch::Matchers::Equals(std::vector<std::uint8_t>(std::begin(Reference), std::begin(Reference) + 2)));

    set_u(21, 0x12, 8);
    set_u(29, 0x34, 8);
    set_u(37, 0x56, 8);

    REQUIRE_THAT(std::vector<std::uint8_t>(std::begin(dest), std::begin(dest) + 5),
                 Catch::Matchers::Equals(std::vector<std::uint8_t>(std::begin(Reference), std::begin(Reference) + 5)));

    offset = 45;
    push_bit(false);
    push_bit(true);
    push_bit(true);

    set_u(48, 0x12, 8);
    set_u(56, 0x34, 8);
    set_u(64, 0x56, 8);

    offset = 72;
    push_bit(true);
    push_bit(false);
    push_bit(false);
    push_bit(true);
    push_bit(true);

    REQUIRE_THAT(std::vector<std::uint8_t>(std::begin(dest), std::begin(dest) + 9),
                 Catch::Matchers::Equals(std::vector<std::uint8_t>(std::begin(Reference), std::begin(Reference) + 9)));

    set_i(77, -2, 8);
    set_u(85, 0b11101100101, 11);
    set_u(96, 0b1110, 3);  // Truncation

    REQUIRE_THAT(std::vector<std::uint8_t>(std::begin(dest), std::begin(dest) + 12),
                 Catch::Matchers::Equals(std::vector<std::uint8_t>(std::begin(Reference), std::begin(Reference) + 12)));

    set_f64(99, 1.0);
    set_f32(163, 1.0F);
    set_f16(195, -99999.0F);

    set_u(211, 0xDEAD, 16);
    set_u(227, 0xBEEF, 16);
    set_u(243, 0, 5);

    REQUIRE(std::size(dest) == std::size(Reference));
    REQUIRE_THAT(dest, Catch::Matchers::Equals(Reference));
}

TEST_CASE("canardDSDLSerialize_heartbeat")
{
    const std::vector<std::uint8_t> Reference({239, 190, 173, 222, 3, 2, 127, 0});

    std::vector<std::uint8_t> dest(std::size(Reference));

    const auto set_u = [&](const std::size_t offset_bit, const std::uint64_t value, const std::uint8_t length_bit) {
        canardDSDLSetUxx(dest.data(), offset_bit, value, length_bit);
    };

    set_u(40, 2, 8);           // mode
    set_u(0, 0xDEADBEEF, 32);  // uptime
    set_u(48, 0x7F, 8);        // vssc
    set_u(32, 3, 8);           // health

    REQUIRE(std::size(dest) == std::size(Reference));
    REQUIRE_THAT(dest, Catch::Matchers::Equals(Reference));
}

TEST_CASE("canardDSDLDeserialize_manual")
{
    const std::array<std::uint8_t, 16> data{
        {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77}};
    REQUIRE(0xFF == canardDSDLGetU8(data.data(), 8, 0, 8));
    REQUIRE(0xFF == canardDSDLGetU8(data.data(), 8, 0, 255));
    REQUIRE(-1 == canardDSDLGetI8(data.data(), 8, 0, 255));
    REQUIRE(-1 == canardDSDLGetI8(data.data(), 8, 0, 8));
    REQUIRE(-1 == canardDSDLGetI8(data.data(), 8, 0, 7));
    REQUIRE(-1 == canardDSDLGetI8(data.data(), 8, 0, 2));

    REQUIRE(0xFFFF == canardDSDLGetU16(data.data(), 8, 0, 16));
    REQUIRE(0xFFFF == canardDSDLGetU16(data.data(), 8, 0, 255));
    REQUIRE(-1 == canardDSDLGetI16(data.data(), 8, 0, 255));
    REQUIRE(-1 == canardDSDLGetI16(data.data(), 8, 0, 16));
    REQUIRE(-1 == canardDSDLGetI16(data.data(), 8, 0, 15));
    REQUIRE(-1 == canardDSDLGetI16(data.data(), 8, 0, 2));

    REQUIRE(0xFFFFFFFF == canardDSDLGetU32(data.data(), 8, 0, 32));
    REQUIRE(0xFFFFFFFF == canardDSDLGetU32(data.data(), 8, 0, 255));
    REQUIRE(-1 == canardDSDLGetI32(data.data(), 8, 0, 255));
    REQUIRE(-1 == canardDSDLGetI32(data.data(), 8, 0, 32));
    REQUIRE(-1 == canardDSDLGetI32(data.data(), 8, 0, 31));
    REQUIRE(-1 == canardDSDLGetI32(data.data(), 8, 0, 2));

    REQUIRE(0xFFFFFFFFFFFFFFFF == canardDSDLGetU64(data.data(), 8, 0, 64));
    REQUIRE(0xFFFFFFFFFFFFFFFF == canardDSDLGetU64(data.data(), 8, 0, 255));
    REQUIRE(-1 == canardDSDLGetI64(data.data(), 8, 0, 255));
    REQUIRE(-1 == canardDSDLGetI64(data.data(), 8, 0, 64));
    REQUIRE(-1 == canardDSDLGetI64(data.data(), 8, 0, 63));
    REQUIRE(-1 == canardDSDLGetI64(data.data(), 8, 0, 2));

    REQUIRE(0 == canardDSDLGetI8(data.data(), 8, 0, 0));
    REQUIRE(0 == canardDSDLGetI16(data.data(), 8, 0, 0));
    REQUIRE(0 == canardDSDLGetI32(data.data(), 8, 0, 0));
    REQUIRE(0 == canardDSDLGetI64(data.data(), 8, 0, 0));

    REQUIRE(0 == canardDSDLGetI8(data.data(), 0, 0, 255));
    REQUIRE(0 == canardDSDLGetI16(data.data(), 0, 0, 255));
    REQUIRE(0 == canardDSDLGetI32(data.data(), 0, 0, 255));
    REQUIRE(0 == canardDSDLGetI64(data.data(), 0, 0, 255));

    REQUIRE(0x77 == canardDSDLGetU8(data.data(), 16, 64, 8));
    REQUIRE(0x77 == canardDSDLGetU8(data.data(), 16, 64, 255));
    REQUIRE(0x77 == canardDSDLGetI8(data.data(), 16, 64, 255));
    REQUIRE(0x77 == canardDSDLGetI8(data.data(), 16, 64, 8));
    REQUIRE(0 > canardDSDLGetI8(data.data(), 16, 64, 7));

    REQUIRE(0x7777 == canardDSDLGetU16(data.data(), 16, 64, 16));
    REQUIRE(0x7777 == canardDSDLGetU16(data.data(), 16, 64, 255));
    REQUIRE(0x7777 == canardDSDLGetI16(data.data(), 16, 64, 255));
    REQUIRE(0x7777 == canardDSDLGetI16(data.data(), 16, 64, 16));
    REQUIRE(0 > canardDSDLGetI16(data.data(), 16, 64, 15));

    REQUIRE(0x77777777 == canardDSDLGetU32(data.data(), 16, 64, 32));
    REQUIRE(0x77777777 == canardDSDLGetU32(data.data(), 16, 64, 255));
    REQUIRE(0x77777777 == canardDSDLGetI32(data.data(), 16, 64, 255));
    REQUIRE(0x77777777 == canardDSDLGetI32(data.data(), 16, 64, 32));
    REQUIRE(0 > canardDSDLGetI32(data.data(), 16, 64, 31));

    REQUIRE(0x7777777777777777 == canardDSDLGetU64(data.data(), 16, 64, 64));
    REQUIRE(0x7777777777777777 == canardDSDLGetU64(data.data(), 16, 64, 255));
    REQUIRE(0x7777777777777777 == canardDSDLGetI64(data.data(), 16, 64, 255));
    REQUIRE(0x7777777777777777 == canardDSDLGetI64(data.data(), 16, 64, 64));
    REQUIRE(0 > canardDSDLGetI64(data.data(), 16, 64, 63));
}

TEST_CASE("canardDSDLDeserialize_aligned")
{
    // The reference values for the following test have been taken from the PyUAVCAN test suite.
    const std::vector<std::uint8_t> Reference({0xA7, 0xEF, 0xCD, 0xAB, 0x90, 0x78, 0x56, 0x34, 0x12, 0x88, 0xA9, 0xCB,
                                               0xED, 0xFE, 0xFF, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0,
                                               0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x7C, 0xDA, 0x0E, 0xDA, 0xBE, 0xFE,
                                               0x01, 0xAD, 0xDE, 0xEF, 0xBE, 0xC5, 0x67, 0xC5, 0x0B});
    const std::uint8_t* const       buf = Reference.data();

    REQUIRE(canardDSDLGetBit(buf, 1, 0));
    REQUIRE(!canardDSDLGetBit(buf, 1, 3));
    REQUIRE(!canardDSDLGetBit(buf, 0, 0));  // IZER

    REQUIRE(0b1010'0111 == canardDSDLGetU8(buf, 45, 0, 8));

    REQUIRE(0x1234'5678'90ab'cdef == canardDSDLGetI64(buf, 45, 8, 64));
    REQUIRE(0x1234'5678'90ab'cdef == canardDSDLGetU64(buf, 45, 8, 64));
    REQUIRE(0xef == canardDSDLGetU8(buf, 45, 8, 64));

    REQUIRE(-0x1234'5678 == canardDSDLGetI32(buf, 45, 72, 32));
    REQUIRE(-2 == canardDSDLGetI16(buf, 45, 104, 16));
    REQUIRE(0 == canardDSDLGetU8(buf, 45, 120, 8));
    REQUIRE(127 == canardDSDLGetI8(buf, 45, 128, 8));
    REQUIRE(Approx(1.0) == canardDSDLGetF64(buf, 45, 136));
    REQUIRE(Approx(1.0F) == canardDSDLGetF32(buf, 45, 200));
    REQUIRE(std::isinf(canardDSDLGetF16(buf, 45, 232)));

    REQUIRE(0x0EDA == canardDSDLGetU16(buf, 45, 248, 12));
    REQUIRE(0 == canardDSDLGetU8(buf, 45, 260, 4));
    REQUIRE(0xBEDA == canardDSDLGetU16(buf, 45, 264, 16));
    REQUIRE(-2 == canardDSDLGetI16(buf, 45, 280, 9));
    REQUIRE(0 == canardDSDLGetI16(buf, 45, 289, 7));
    REQUIRE(0 == canardDSDLGetU16(buf, 45, 289, 7));
    REQUIRE(0 == canardDSDLGetI8(buf, 45, 289, 7));
    REQUIRE(0 == canardDSDLGetU8(buf, 45, 289, 7));

    REQUIRE(0xDEAD == canardDSDLGetU16(buf, 45, 296, 16));
    REQUIRE(0xBEEF == canardDSDLGetU16(buf, 45, 312, 16));
    REQUIRE(0xDEAD == canardDSDLGetU32(buf, 45, 296, 16));
    REQUIRE(0xBEEF == canardDSDLGetU32(buf, 45, 312, 16));
    REQUIRE(0xDEAD == canardDSDLGetU64(buf, 45, 296, 16));
    REQUIRE(0xBEEF == canardDSDLGetU64(buf, 45, 312, 16));
    REQUIRE(0xAD == canardDSDLGetU8(buf, 45, 296, 16));
    REQUIRE(0xEF == canardDSDLGetU8(buf, 45, 312, 16));
    REQUIRE(0x00AD == canardDSDLGetU16(buf, 38, 296, 16));
    REQUIRE(0x0000 == canardDSDLGetU32(buf, 37, 296, 16));

    REQUIRE(canardDSDLGetBit(buf, 45, 328));
    REQUIRE(!canardDSDLGetBit(buf, 45, 329));
    REQUIRE(canardDSDLGetBit(buf, 45, 330));
    REQUIRE(!canardDSDLGetBit(buf, 45, 331));
    REQUIRE(!canardDSDLGetBit(buf, 45, 332));
    REQUIRE(!canardDSDLGetBit(buf, 45, 333));
    REQUIRE(canardDSDLGetBit(buf, 45, 334));
    REQUIRE(canardDSDLGetBit(buf, 45, 335));
    REQUIRE(canardDSDLGetBit(buf, 45, 336));
    REQUIRE(canardDSDLGetBit(buf, 45, 337));
    REQUIRE(canardDSDLGetBit(buf, 45, 338));
    REQUIRE(!canardDSDLGetBit(buf, 45, 339));
    REQUIRE(!canardDSDLGetBit(buf, 45, 340));
    REQUIRE(canardDSDLGetBit(buf, 45, 341));
    REQUIRE(canardDSDLGetBit(buf, 45, 342));
    REQUIRE(!canardDSDLGetBit(buf, 45, 343));

    REQUIRE(canardDSDLGetBit(buf, 45, 344));
    REQUIRE(!canardDSDLGetBit(buf, 45, 345));
    REQUIRE(canardDSDLGetBit(buf, 45, 346));
    REQUIRE(!canardDSDLGetBit(buf, 45, 347));
    REQUIRE(!canardDSDLGetBit(buf, 45, 348));
    REQUIRE(!canardDSDLGetBit(buf, 45, 349));
    REQUIRE(canardDSDLGetBit(buf, 45, 350));
    REQUIRE(canardDSDLGetBit(buf, 45, 351));
    REQUIRE(canardDSDLGetBit(buf, 45, 352));
    REQUIRE(canardDSDLGetBit(buf, 45, 353));
    REQUIRE(!canardDSDLGetBit(buf, 45, 354));
    REQUIRE(canardDSDLGetBit(buf, 45, 355));
    REQUIRE(!canardDSDLGetBit(buf, 45, 356));

    REQUIRE(0 == canardDSDLGetU8(buf, 45, 357, 3));

    REQUIRE(!canardDSDLGetBit(buf, 44, 355));
}

TEST_CASE("canardDSDLDeserialize_unaligned")
{
    // The reference values for the following test have been taken from the PyUAVCAN test suite.
    const std::vector<std::uint8_t> Reference({
        0xC5, 0x2F, 0x57, 0x82, 0xC6, 0xCA, 0x12, 0x34, 0x56, 0xD9, 0xBF, 0xEC, 0x06, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x80, 0xFF, 0x01, 0x00, 0x00, 0xFC, 0x01, 0xE0, 0x6F, 0xF5, 0x7E, 0xF7, 0x05  //
    });
    const std::uint8_t* const       buf = Reference.data();

    REQUIRE(canardDSDLGetBit(buf, 31, 0));
    REQUIRE(!canardDSDLGetBit(buf, 31, 1));
    REQUIRE(canardDSDLGetBit(buf, 31, 2));
    REQUIRE(!canardDSDLGetBit(buf, 31, 3));
    REQUIRE(!canardDSDLGetBit(buf, 31, 4));
    REQUIRE(!canardDSDLGetBit(buf, 31, 5));
    REQUIRE(canardDSDLGetBit(buf, 31, 6));
    REQUIRE(canardDSDLGetBit(buf, 31, 7));
    REQUIRE(canardDSDLGetBit(buf, 31, 8));
    REQUIRE(canardDSDLGetBit(buf, 31, 9));
    REQUIRE(canardDSDLGetBit(buf, 31, 10));

    REQUIRE(canardDSDLGetBit(buf, 31, 11));
    REQUIRE(!canardDSDLGetBit(buf, 31, 12));
    REQUIRE(canardDSDLGetBit(buf, 31, 13));
    REQUIRE(!canardDSDLGetBit(buf, 31, 14));
    REQUIRE(!canardDSDLGetBit(buf, 31, 15));
    REQUIRE(canardDSDLGetBit(buf, 31, 16));
    REQUIRE(canardDSDLGetBit(buf, 31, 17));
    REQUIRE(canardDSDLGetBit(buf, 31, 18));
    REQUIRE(!canardDSDLGetBit(buf, 31, 19));
    REQUIRE(canardDSDLGetBit(buf, 31, 20));

    REQUIRE(0x12 == canardDSDLGetU8(buf, 31, 21, 8));
    REQUIRE(0x34 == canardDSDLGetU8(buf, 31, 29, 8));
    REQUIRE(0x56 == canardDSDLGetU8(buf, 31, 37, 8));

    REQUIRE(!canardDSDLGetBit(buf, 31, 45));
    REQUIRE(canardDSDLGetBit(buf, 31, 46));
    REQUIRE(canardDSDLGetBit(buf, 31, 47));

    REQUIRE(0x12 == canardDSDLGetU8(buf, 31, 48, 8));
    REQUIRE(0x34 == canardDSDLGetU8(buf, 31, 56, 8));
    REQUIRE(0x56 == canardDSDLGetU8(buf, 31, 64, 8));

    REQUIRE(canardDSDLGetBit(buf, 31, 72));
    REQUIRE(!canardDSDLGetBit(buf, 31, 73));
    REQUIRE(!canardDSDLGetBit(buf, 31, 74));
    REQUIRE(canardDSDLGetBit(buf, 31, 75));
    REQUIRE(canardDSDLGetBit(buf, 31, 76));

    REQUIRE(-2 == canardDSDLGetI8(buf, 31, 77, 8));
    REQUIRE(0b11101100101 == canardDSDLGetU16(buf, 31, 85, 11));
    REQUIRE(0b110 == canardDSDLGetU8(buf, 31, 96, 3));

    REQUIRE(Approx(1.0) == canardDSDLGetF64(buf, 31, 99));
    REQUIRE(Approx(1.0F) == canardDSDLGetF32(buf, 31, 163));
    REQUIRE(std::isinf(canardDSDLGetF16(buf, 31, 195)));
    REQUIRE(0.0F > canardDSDLGetF16(buf, 31, 195));

    REQUIRE(0xDEAD == canardDSDLGetU16(buf, 31, 211, 16));
    REQUIRE(0xBEEF == canardDSDLGetU16(buf, 31, 227, 16));
    REQUIRE(0 == canardDSDLGetU8(buf, 31, 243, 5));
}

TEST_CASE("canardDSDLDeserialize_heartbeat")
{
    const std::vector<std::uint8_t> Reference({239, 190, 173, 222, 3, 2, 127, 0});
    const std::uint8_t* const       buf = Reference.data();
    REQUIRE(2 == canardDSDLGetU8(buf, 7, 40, 8));            // mode
    REQUIRE(0xDEADBEEF == canardDSDLGetU32(buf, 7, 0, 32));  // uptime
    REQUIRE(0x7F == canardDSDLGetU32(buf, 7, 48, 8));        // vssc
    REQUIRE(3 == canardDSDLGetU8(buf, 7, 32, 8));            // health
}
