/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#if UAVCAN_STM32_CHIBIOS
# include <ch.hpp>
#elif UAVCAN_STM32_NUTTX
# include <nuttx/config.h>
# include <nuttx/fs/fs.h>
# include <poll.h>
# include <errno.h>
# include <cstdio>
# include <ctime>
# include <cstring>
#else
# error "Unknown OS"
#endif

#include <uavcan/uavcan.hpp>

namespace uavcan_stm32
{

class CanDriver;

#if UAVCAN_STM32_CHIBIOS

class BusEvent
{
    chibios_rt::CounterSemaphore sem_;

public:
    BusEvent(CanDriver& can_driver)
        : sem_(0)
    {
        (void)can_driver;
    }

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

class BusEvent : uavcan::Noncopyable
{
    static const unsigned MaxPollWaiters = 8;

    ::file_operations file_ops_;
    ::pollfd* pollset_[MaxPollWaiters];
    CanDriver& can_driver_;
    bool signal_;

    static int openTrampoline(::file* filp);
    static int closeTrampoline(::file* filp);
    static int pollTrampoline(::file* filp, ::pollfd* fds, bool setup);

    int open(::file* filp);
    int close(::file* filp);
    int poll(::file* filp, ::pollfd* fds, bool setup);

    unsigned makePollMask() const;

    int addPollWaiter(::pollfd* fds);
    int removePollWaiter(::pollfd* fds);

public:
    static const char* const DevName;

    BusEvent(CanDriver& can_driver);
    ~BusEvent();

    bool wait(uavcan::MonotonicDuration duration);

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
