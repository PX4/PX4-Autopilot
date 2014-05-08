/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#if UAVCAN_STM32_CHIBIOS
# include <ch.hpp>
#elif UAVCAN_STM32_NUTTX
# include <semaphore.h>
# include <time.h>
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

class Mutex
{
    chibios_rt::Mutex mtx_;

public:
    void lock();
    void unlock();
};

#elif UAVCAN_STM32_NUTTX

class Event : uavcan::Noncopyable
{
    sem_t sem_;

public:
    Event();
    ~Event();

    bool wait(uavcan::MonotonicDuration duration);

    void signal();

    void signalFromInterrupt();
};

#endif


#if UAVCAN_STM32_CHIBIOS

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

#endif

}
