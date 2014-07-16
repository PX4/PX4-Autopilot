/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/build_config.hpp>

int main(int argc, char **argv)
{
#ifndef UAVCAN_CPP_VERSION
# error UAVCAN_CPP_VERSION
#endif
#if UAVCAN_CPP_VERSION == UAVCAN_CPP11
    std::cout << "C++11" << std::endl;
#elif UAVCAN_CPP_VERSION == UAVCAN_CPP03
    std::cout << "C++03" << std::endl;
#else
# error UAVCAN_CPP_VERSION
#endif
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
