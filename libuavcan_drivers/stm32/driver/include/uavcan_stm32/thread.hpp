/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#if UAVCAN_STM32_CHIBIOS
# include <ch.hpp>
#else
# error "Unknown OS"
#endif

#include <uavcan/uavcan.hpp>

namespace uavcan_stm32
{

#if UAVCAN_STM32_CHIBIOS

class Event
{
    chibios_rt::CounterSemaphore sem_;

public:
    Event() : sem_(0) { }

    bool wait(uavcan::MonotonicDuration duration);

    void signal();

    void signalFromInterrupt();
};

#endif

}
