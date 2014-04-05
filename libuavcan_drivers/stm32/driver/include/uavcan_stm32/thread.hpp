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

class Event
{
#if UAVCAN_STM32_CHIBIOS
    chibios_rt::CounterSemaphore sem_;
#endif

public:
    Event() : sem_(0) { }

    bool wait(uavcan::MonotonicDuration duration);

    void signal();

    void signalFromInterrupt();
};


class Mutex
{
#if UAVCAN_STM32_CHIBIOS
    chibios_rt::Mutex mtx_;
#endif

public:
    void lock();
    void unlock();
};


class MutexLocker
{
    Mutex& mutex_;

public:
    MutexLocker(Mutex& mutex)
        : mutex_(mutex)
    {
        mutex_.lock();
    }
    ~MutexLocker()
    {
        mutex_.unlock();
    }
};

}
