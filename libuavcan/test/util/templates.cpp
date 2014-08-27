/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/util/templates.hpp>
#include <limits>

struct NonConvertible { };

struct ConvertibleToBool
{
    const bool value;
    ConvertibleToBool(bool value) : value(value) { }
    operator bool() const { return value; }
    bool operator!() const { return !value; }
};

struct NonDefaultConstructible
{
    NonDefaultConstructible(int) { }
};

TEST(Util, TryImplicitCast)
{
    using uavcan::try_implicit_cast;

    ASSERT_FALSE(try_implicit_cast<bool>(NonConvertible()));
    ASSERT_TRUE(try_implicit_cast<bool>(NonConvertible(), true));

    ASSERT_EQ(0, try_implicit_cast<long>(NonConvertible()));
    ASSERT_EQ(9000, try_implicit_cast<long>(NonConvertible(), 9000));

    ASSERT_TRUE(try_implicit_cast<bool>(ConvertibleToBool(true)));
    ASSERT_TRUE(try_implicit_cast<bool>(ConvertibleToBool(true), false));
    ASSERT_FALSE(try_implicit_cast<bool>(ConvertibleToBool(false)));
    ASSERT_FALSE(try_implicit_cast<bool>(ConvertibleToBool(false), true));
    ASSERT_EQ(1, try_implicit_cast<long>(ConvertibleToBool(true)));
    ASSERT_EQ(0, try_implicit_cast<long>(ConvertibleToBool(false), -100));

    //try_implicit_cast<NonDefaultConstructible>(ConvertibleToBool(true));   // Will fail to compile
    try_implicit_cast<NonDefaultConstructible>(NonConvertible(), NonDefaultConstructible(64));
}

TEST(Util, FloatClassification)
{
    // NAN
    ASSERT_TRUE(uavcan::isNaN(std::numeric_limits<float>::quiet_NaN()));
    ASSERT_FALSE(uavcan::isNaN(std::numeric_limits<double>::infinity()));
    ASSERT_FALSE(uavcan::isNaN(std::numeric_limits<long double>::infinity()));
    ASSERT_FALSE(uavcan::isNaN(123.456));

    // INF
    ASSERT_TRUE(uavcan::isInfinity(std::numeric_limits<float>::infinity()));
    ASSERT_TRUE(uavcan::isInfinity(-std::numeric_limits<long double>::infinity()));
    ASSERT_FALSE(uavcan::isInfinity(std::numeric_limits<float>::quiet_NaN()));
    ASSERT_FALSE(uavcan::isInfinity(-0.1L));

    // Signbit
    ASSERT_FALSE(uavcan::getSignBit(12));
    ASSERT_TRUE(uavcan::getSignBit(-std::numeric_limits<long double>::infinity()));
    ASSERT_FALSE(uavcan::getSignBit(std::numeric_limits<float>::infinity()));
    ASSERT_TRUE(uavcan::getSignBit(-0.0));                                           // Negative zero
}
