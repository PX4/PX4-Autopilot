/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan_stm32/thread.hpp>
#include <uavcan_stm32/clock.hpp>
#include "internal.hpp"

namespace uavcan_stm32
{

#if UAVCAN_STM32_CHIBIOS
/*
 * Event
 */
bool Event::wait(uavcan::MonotonicDuration duration)
{
    static const uavcan::int64_t MaxDelayMSec = 0x000FFFFF;

    const uavcan::int64_t msec = duration.toMSec();
    msg_t ret = msg_t();

    if (msec <= 0)
    {
        ret = sem_.waitTimeout(TIME_IMMEDIATE);
    }
    else
    {
        ret = sem_.waitTimeout((msec > MaxDelayMSec) ? MS2ST(MaxDelayMSec) : MS2ST(msec));
    }
    return ret == RDY_OK;
}

void Event::signal()
{
    sem_.signal();
}

void Event::signalFromInterrupt()
{
    chSysLockFromIsr();
    sem_.signalI();
    chSysUnlockFromIsr();
}

/*
 * Mutex
 */
void Mutex::lock()
{
    mtx_.lock();
}

void Mutex::unlock()
{
    chibios_rt::BaseThread::unlockMutex();
}

#elif UAVCAN_STM32_NUTTX

const unsigned Event::MaxPollWaiters;
const unsigned Event::PollEvents;
const char* const Event::DevName = "/dev/uavcan/busevent";

int Event::openTrampoline(::file* filp)
{
    return static_cast<Event*>(filp->f_inode->i_private)->open(filp);
}

int Event::closeTrampoline(::file* filp)
{
    return static_cast<Event*>(filp->f_inode->i_private)->close(filp);
}

int Event::pollTrampoline(::file* filp, ::pollfd* fds, bool setup)
{
    return static_cast<Event*>(filp->f_inode->i_private)->poll(filp, fds, setup);
}

::timespec Event::computeDeadline(uavcan::MonotonicDuration duration)
{
    if (duration.isNegative())
    {
        duration = uavcan::MonotonicDuration();
    }
    ::timespec deadline = ::timespec();
    if (::clock_gettime(CLOCK_REALTIME, &deadline) >= 0)
    {
        deadline.tv_sec += duration.toUSec() / 1000000;
        deadline.tv_nsec += (duration.toUSec() % 1000000) * 1000;
        if (deadline.tv_nsec >= 1000000000L)
        {
            deadline.tv_sec += 1;
            deadline.tv_nsec -= 1000000000L;
        }
    }
    return deadline;
}

int Event::open(::file* filp)
{
    (void)filp;
    return 0;
}

int Event::close(::file* filp)
{
    (void)filp;
    return 0;
}

int Event::poll(::file* filp, ::pollfd* fds, bool setup)
{
    CriticalSectionLocker locker;
    int ret = -1;

    if (setup)
    {
        ret = addPollWaiter(fds);
        if (ret == 0)
        {
            const unsigned poll_state = signal_ ? PollEvents : 0;
            signal_ = false;

            // Update fds and signal immediately if needed
            fds->revents |= fds->events & poll_state;
            if (fds->revents != 0)
            {
                (void)sem_post(fds->sem);
            }
        }
    }
    else
    {
        ret = removePollWaiter(fds);
    }

    return ret;
}

int Event::addPollWaiter(::pollfd* fds)
{
    for (unsigned i = 0; i < MaxPollWaiters; i++)
    {
        if (pollset_[i] == nullptr)
        {
            pollset_[i] = fds;
            return 0;
        }
    }
    return -ENOMEM;
}

int Event::removePollWaiter(::pollfd* fds)
{
    for (unsigned i = 0; i < MaxPollWaiters; i++)
    {
        if (fds == pollset_[i])
        {
            pollset_[i] = nullptr;
            return 0;
        }
    }
    return -EINVAL;
}

Event::Event()
    : signal_(false)
{
    std::memset(&file_ops_, 0, sizeof(file_ops_));
    std::memset(pollset_, 0, sizeof(pollset_));
    file_ops_.open  = &Event::openTrampoline;
    file_ops_.close = &Event::closeTrampoline;
    file_ops_.poll  = &Event::pollTrampoline;
    // TODO: move to init(), add proper error handling
    if (register_driver(DevName, &file_ops_, 0666, static_cast<void*>(this)) != 0)
    {
        std::abort();
    }
}

Event::~Event()
{
    (void)unregister_driver(DevName);
}

bool Event::wait(uavcan::MonotonicDuration duration)
{
    // TODO blocking wait
    const uavcan::MonotonicTime deadline = clock::getMonotonic() + duration;
    while (clock::getMonotonic() < deadline)
    {
        {
            CriticalSectionLocker locker;
            if (signal_)
            {
                signal_ = false;
                return true;
            }
        }
        ::usleep(1000);
    }
    return false;
}

void Event::signalFromInterrupt()
{
    signal_ = true;
    for (unsigned i = 0; i < MaxPollWaiters; i++)
    {
        ::pollfd* const fd = pollset_[i];
        if (fd != nullptr)
        {
            fd->revents = fd->events & PollEvents;
            if ((fd->revents != 0) && (fd->sem->semcount <= 0))
            {
                (void)sem_post(fd->sem);
            }
        }
    }
}

#endif

}
