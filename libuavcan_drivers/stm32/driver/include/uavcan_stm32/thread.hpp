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
    static const unsigned MaxPollWaiters = 8;
    static const unsigned PollEvents = POLLIN | POLLOUT;

    ::file_operations file_ops_;
    ::pollfd* pollset_[MaxPollWaiters];
    bool signal_;

    static int openTrampoline(::file* filp);
    static int closeTrampoline(::file* filp);
    static int pollTrampoline(::file* filp, ::pollfd* fds, bool setup);

    ::timespec computeDeadline(uavcan::MonotonicDuration duration);

    int open(::file* filp);
    int close(::file* filp);
    int poll(::file* filp, ::pollfd* fds, bool setup);

    int addPollWaiter(::pollfd* fds);
    int removePollWaiter(::pollfd* fds);

public:
    static const char* const DevName;

    Event();
    ~Event();

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
