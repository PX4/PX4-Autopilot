/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <iostream>
#include <cerrno>
#include <chrono>
#include <uavcan_linux/uavcan_linux.hpp>

static std::string systime2str(const std::chrono::system_clock::time_point& tp)
{
    const auto tt = std::chrono::system_clock::to_time_t(tp);
    return std::ctime(&tt);
}

int main()
{
    uavcan_linux::SystemClock clock;

    /*
     * Auto-detected clock adjustment mode
     */
    std::cout << "Clock adjustment mode: ";
    switch (clock.getAdjustmentMode())
    {
    case uavcan_linux::ClockAdjustmentMode::SystemWide:
    {
        std::cout << "SystemWide";
        break;
    }
    case uavcan_linux::ClockAdjustmentMode::PerDriverPrivate:
    {
        std::cout << "PerDriverPrivate";
        break;
    }
    default:
    {
        std::abort();
        break;
    }
    }
    std::cout << std::endl;

    /*
     * Test adjustment
     */
    double sec = 0;
    std::cout << "Enter system time adjustment in seconds (fractions allowed): " << std::endl;
    std::cin >> sec;

    const auto before = std::chrono::system_clock::now();
    try
    {
        clock.adjustUtc(uavcan::UtcDuration::fromUSec(sec * 1e6));
    }
    catch (const uavcan_linux::Exception& ex)
    {
        std::cout << ex.what() << std::endl;
        std::cout << strerror(ex.getErrno()) << std::endl;
        return 1;
    }
    const auto after = std::chrono::system_clock::now();

    std::cout << "Time before: " << systime2str(before) << "\n"
              << "Time after:  " << systime2str(after) << "\n"
              << "Millisecond diff (after - before): "
              << std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count() << std::endl;

    return 0;
}
