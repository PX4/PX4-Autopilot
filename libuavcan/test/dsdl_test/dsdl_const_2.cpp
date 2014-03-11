/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/FigureOfMerit.hpp>


TEST(DsdlConst2, FigureOfMerit)
{
    using uavcan::FigureOfMerit;

    std::cout << &FigureOfMerit::MAX << std::endl;
    std::cout << &FigureOfMerit::MIN << std::endl;
    std::cout << &FigureOfMerit::UNKNOWN << std::endl;
}
