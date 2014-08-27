/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/util/comparison.hpp>

TEST(Comparison, Basic)
{
    // Basic same type floats
    ASSERT_TRUE(uavcan::areClose(0.1, 0.1));
    ASSERT_TRUE(uavcan::areClose(0.1F, 0.1F));
    ASSERT_TRUE(uavcan::areClose(0.1L, 0.1L));

    // Basic mixed type floats
    ASSERT_TRUE(uavcan::areClose(0.1F, 0.1));
    ASSERT_TRUE(uavcan::areClose(0.1,  0.1F));
    ASSERT_TRUE(uavcan::areClose(0.1F, 0.1L));
    ASSERT_TRUE(uavcan::areClose(0.1L, 0.1F));
    ASSERT_TRUE(uavcan::areClose(0.1,  0.1L));
    ASSERT_TRUE(uavcan::areClose(0.1L, 0.1));

    // Basic floats
    ASSERT_TRUE(uavcan::areClose(0x07, '\x07'));
    ASSERT_TRUE(uavcan::areClose(123456789LL, 123456789));
    ASSERT_TRUE(uavcan::areClose("123", std::string("123")));

    // Non-equality
    ASSERT_FALSE(uavcan::areClose(-0.1, 0.1));
    ASSERT_FALSE(uavcan::areClose(-0.1F, 0.0L));
    ASSERT_FALSE(uavcan::areClose("123", std::string("12")));
    ASSERT_FALSE(uavcan::areClose(0x07L, '\0'));
}

TEST(Comparison, FloatSpecialCase)
{
    ASSERT_FALSE(uavcan::areClose(0.1, std::numeric_limits<double>::infinity()));

    ASSERT_TRUE(uavcan::areClose(std::numeric_limits<float>::infinity(),
                                 std::numeric_limits<long double>::infinity()));

    ASSERT_FALSE(uavcan::areClose(std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()));

    ASSERT_FALSE(uavcan::areClose(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()));
}

TEST(Comparison, FloatULP)
{
    ASSERT_FALSE(uavcan::areClose(0.100000000000000001L, 0.1L));
    ASSERT_TRUE( uavcan::areClose(0.100000000000000001,  0.1L));
    ASSERT_TRUE( uavcan::areClose(0.100000000000000001F, 0.1L));

    // Near zero
    ASSERT_TRUE( uavcan::areClose(0.0F,  std::numeric_limits<float>::epsilon()));
    ASSERT_TRUE( uavcan::areClose(0.0F, -std::numeric_limits<float>::epsilon()));
    ASSERT_FALSE(uavcan::areClose(0.0F,  std::numeric_limits<float>::epsilon() * 2));
}

TEST(Comparison, BruteforceValidation)
{
    const std::streamsize default_precision = std::cout.precision();
    std::cout.precision(20);

    float x = -uavcan::NumericTraits<float>::max();

    while (x < uavcan::NumericTraits<float>::max())
    {
        const float y1 = x + std::abs(x) * (uavcan::NumericTraits<float>::epsilon() * 2);                // Still equal
        const float y2 = x + uavcan::max(std::abs(x), 1.F) * (uavcan::NumericTraits<float>::epsilon() * 20); // Nope

        if (!uavcan::areClose(y1, x))
        {
            std::cout << "y1=" << y1 << " y2=" << y2 << " x=" << x << std::endl;
            ASSERT_TRUE(false);
        }
        if (uavcan::areClose(y2, x))
        {
            std::cout << "y1=" << y1 << " y2=" << y2 << " x=" << x << std::endl;
            ASSERT_TRUE(false);
        }

        x = y2;
    }

    std::cout.precision(default_precision);
}
