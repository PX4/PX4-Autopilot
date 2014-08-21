/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/Timestamp.hpp>


TEST(DsdlConst2, Timestamp)
{
    using uavcan::Timestamp;

    std::cout << &Timestamp::USEC_PER_LSB << std::endl;
    std::cout << &Timestamp::UNKNOWN << std::endl;
}
