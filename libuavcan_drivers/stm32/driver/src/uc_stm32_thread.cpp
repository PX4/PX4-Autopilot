/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan_stm32/thread.hpp>
#include <uavcan_stm32/clock.hpp>
#include <uavcan_stm32/can.hpp>
#include "internal.hpp"

namespace uavcan_stm32
{

#if UAVCAN_STM32_CHIBIOS
/*
 * BusEvent
 */
bool BusEvent::wait(uavcan::MonotonicDuration duration)
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

void BusEvent::signal()
{
    sem_.signal();
}

void BusEvent::signalFromInterrupt()
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

const unsigned BusEvent::MaxPollWaiters;
const char* const BusEvent::DevName = "/dev/uavcan/busevent";

int BusEvent::openTrampoline(::file* filp)
{
    return static_cast<BusEvent*>(filp->f_inode->i_private)->open(filp);
}

int BusEvent::closeTrampoline(::file* filp)
{
    return static_cast<BusEvent*>(filp->f_inode->i_private)->close(filp);
}

int BusEvent::pollTrampoline(::file* filp, ::pollfd* fds, bool setup)
{
    return static_cast<BusEvent*>(filp->f_inode->i_private)->poll(filp, fds, setup);
}

int BusEvent::open(::file* filp)
{
    (void)filp;
    return 0;
}

int BusEvent::close(::file* filp)
{
    (void)filp;
    return 0;
}

int BusEvent::poll(::file* filp, ::pollfd* fds, bool setup)
{
    CriticalSectionLocker locker;
    int ret = -1;

    if (setup)
    {
        ret = addPollWaiter(fds);
        if (ret == 0)
        {
            fds->revents |= fds->events & makePollMask();
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

unsigned BusEvent::makePollMask() const
{
    const uavcan::CanSelectMasks select_masks = can_driver_.makeSelectMasks();
    unsigned poll_mask = 0;
    if (select_masks.read != 0)
    {
        poll_mask |= POLLIN;
    }
    if (select_masks.write != 0)
    {
        poll_mask |= POLLOUT;
    }
    return poll_mask;
}

int BusEvent::addPollWaiter(::pollfd* fds)
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

int BusEvent::removePollWaiter(::pollfd* fds)
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

BusEvent::BusEvent(CanDriver& can_driver)
    : can_driver_(can_driver)
    , signal_(false)
{
    std::memset(&file_ops_, 0, sizeof(file_ops_));
    std::memset(pollset_, 0, sizeof(pollset_));
    file_ops_.open  = &BusEvent::openTrampoline;
    file_ops_.close = &BusEvent::closeTrampoline;
    file_ops_.poll  = &BusEvent::pollTrampoline;
    // TODO: move to init(), add proper error handling
    if (register_driver(DevName, &file_ops_, 0666, static_cast<void*>(this)) != 0)
    {
        std::abort();
    }
}

BusEvent::~BusEvent()
{
    (void)unregister_driver(DevName);
}

bool BusEvent::wait(uavcan::MonotonicDuration duration)
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

void BusEvent::signalFromInterrupt()
{
    signal_ = true;  // HACK
    for (unsigned i = 0; i < MaxPollWaiters; i++)
    {
        ::pollfd* const fd = pollset_[i];
        if (fd != nullptr)
        {
            fd->revents = fd->events & makePollMask();
            if ((fd->revents != 0) && (fd->sem->semcount <= 0))
            {
                (void)sem_post(fd->sem);
            }
        }
    }
}

#endif

}
