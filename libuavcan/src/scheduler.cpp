/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <limits>
#include <uavcan/scheduler.hpp>
#include <uavcan/internal/debug.hpp>

namespace uavcan
{

struct TimerInsertionComparator
{
    const uint64_t ts;
    TimerInsertionComparator(uint64_t ts) : ts(ts) { }
    bool operator()(const TimerBase* t) const
    {
        return t->getMonotonicDeadline() > ts;
    }
};

uint64_t Scheduler::computeDispatcherSpinDeadline(uint64_t spin_deadline) const
{
    uint64_t timer_deadline = std::numeric_limits<uint64_t>::max();
    TimerBase* const timer = ordered_timers_.get();
    if (timer)
        timer_deadline = timer->getMonotonicDeadline();

    const uint64_t earliest = std::min(timer_deadline, spin_deadline);
    const uint64_t ts = getMonotonicTimestamp();
    if (earliest > ts)
    {
        if (ts - earliest > timer_resolution_)
            return ts + timer_resolution_;
    }
    return earliest;
}

uint64_t Scheduler::pollTimersAndGetMonotonicTimestamp()
{
    while (true)
    {
        TimerBase* const timer = ordered_timers_.get();
        if (!timer)
            return getMonotonicTimestamp();
#if UAVCAN_DEBUG
        if (timer->getNextListNode())      // Order check
            assert(timer->getMonotonicDeadline() <= timer->getNextListNode()->getMonotonicDeadline());
#endif

        const uint64_t ts = getMonotonicTimestamp();
        if (ts < timer->getMonotonicDeadline())
            return ts;

        ordered_timers_.remove(timer);
        timer->handleOneShotTimeout(ts);   // This timer can be re-registered immediately
    }
    assert(0);
    return 0;
}

void Scheduler::pollCleanup(uint64_t mono_ts, uint32_t num_frames_processed_with_last_spin)
{
    // cleanup will be performed less frequently if the stack handles more frames per second
    const uint64_t deadline = prev_cleanup_ts_ + cleanup_period_ * (num_frames_processed_with_last_spin + 1);
    if (mono_ts > deadline)
    {
        UAVCAN_TRACE("Scheduler", "Cleanup with %u processed frames", num_frames_processed_with_last_spin);
        prev_cleanup_ts_ = mono_ts;
        dispatcher_.cleanup(mono_ts);
    }
}

int Scheduler::spin(uint64_t monotonic_deadline)
{
    int retval = 0;
    while (true)
    {
        const uint64_t dl = computeDispatcherSpinDeadline(monotonic_deadline);
        retval = dispatcher_.spin(dl);
        if (retval < 0)
            break;

        const uint64_t ts = pollTimersAndGetMonotonicTimestamp();
        pollCleanup(ts, retval);
        if (ts >= monotonic_deadline)
            break;
    }
    return retval;
}

void Scheduler::registerOneShotTimer(TimerBase* timer)
{
    assert(timer);
    ordered_timers_.insertBefore(timer, TimerInsertionComparator(timer->getMonotonicDeadline()));
}

void Scheduler::unregisterOneShotTimer(TimerBase* timer)
{
    assert(timer);
    ordered_timers_.remove(timer);
}

bool Scheduler::isOneShotTimerRegistered(const TimerBase* timer) const
{
    assert(timer);
    const TimerBase* p = ordered_timers_.get();
#if UAVCAN_DEBUG
    uint64_t prev_deadline = 0;
#endif
    while (p)
    {
#if UAVCAN_DEBUG
        if (prev_deadline > p->getMonotonicDeadline())  // Self check
            std::abort();
        prev_deadline = p->getMonotonicDeadline();
#endif
        if (p == timer)
            return true;
        p = p->getNextListNode();
    }
    return false;
}

}
