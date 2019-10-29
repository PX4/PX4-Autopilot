/*
 * Copyright (C) 2014, 2018 Pavel Kirienko <pavel.kirienko@gmail.com>
 * Kinetis Port Author David Sidrane <david_s5@nscdg.com>
 */

#include <uavcan_kinetis/thread.hpp>
#include <uavcan_kinetis/clock.hpp>
#include <uavcan_kinetis/can.hpp>
#include "internal.hpp"

namespace uavcan_kinetis
{


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
            /*
             * Two events can be reported via POLLIN:
             *  - The RX queue is not empty. This event is level-triggered.
             *  - Transmission complete. This event is edge-triggered.
             * FIXME Since TX event is edge-triggered, it can be lost between poll() calls.
             */
            fds->revents |= fds->events & (can_driver_.hasReadableInterfaces() ? POLLIN : 0);
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

int BusEvent::addPollWaiter(::pollfd* fds)
{
    for (unsigned i = 0; i < MaxPollWaiters; i++)
    {
        if (pollset_[i] == UAVCAN_NULLPTR)
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
            pollset_[i] = UAVCAN_NULLPTR;
            return 0;
        }
    }
    return -EINVAL;
}

BusEvent::BusEvent(CanDriver& can_driver)
    : can_driver_(can_driver),
    signal_(false)
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
    signal_ = true;     // HACK
    for (unsigned i = 0; i < MaxPollWaiters; i++)
    {
        ::pollfd* const fd = pollset_[i];
        if (fd != UAVCAN_NULLPTR)
        {
            fd->revents |= fd->events & POLLIN;
            if ((fd->revents != 0) && (fd->sem->semcount <= 0))
            {
                (void)sem_post(fd->sem);
            }
        }
    }
}

}
