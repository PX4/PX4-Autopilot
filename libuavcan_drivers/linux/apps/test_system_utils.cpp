/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan_linux/uavcan_linux.hpp>
#include <iostream>
#include <iomanip>
#include "debug.hpp"

int main(int argc, const char** argv)
{
    try
    {
        const std::vector<std::string> iface_names(argv + 1, argv + argc);

        const auto res = uavcan_linux::MachineIDReader(iface_names).readAndGetLocation();

        const auto original_flags = std::cout.flags();

        for (auto x : res.first)
        {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << int(x);
        }

        std::cout.width(0);
        std::cout.flags(original_flags);

        std::cout << std::endl;

        std::cout << res.second << std::endl;

        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return 1;
    }
}
