/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <iostream>
#include <cerrno>
#include <uavcan/uavcan.hpp>
#include <uavcan_linux/socketcan.hpp>
#include <uavcan_linux/clock.hpp>

int main()
{
    double sec = 0;
    std::cout << "Enter system time adjustment in seconds: " << std::endl;
    std::cin >> sec;

    uavcan_linux::SystemClockDriver clock;

    try
    {
        clock.adjustUtc(uavcan::UtcDuration::fromUSec(sec * 1e6));
    }
    catch (const uavcan_linux::Exception& ex)
    {
        std::cout << ex.what() << std::endl;
        std::cout << strerror(ex.getErrno()) << std::endl;
    }

    return 0;
}
