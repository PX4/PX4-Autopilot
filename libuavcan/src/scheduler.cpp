/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <limits>
#include <uavcan/scheduler.hpp>
#include <uavcan/internal/debug.hpp>

namespace uavcan
{
/*
 * MonotonicDeadlineHandler
 */
void MonotonicDeadlineHandler::startWithDeadline(uint64_t monotonic_deadline)
{
    assert(monotonic_deadline > 0);
    stop();
    monotonic_deadline_ = monotonic_deadline;
    scheduler_.getMonotonicDeadlineScheduler().add(this);
}

void MonotonicDeadlineHandler::startWithDelay(uint64_t delay_usec)
{
    startWithDeadline(scheduler_.getMonotonicTimestamp() + delay_usec);
}

void MonotonicDeadlineHandler::stop()
{
    scheduler_.getMonotonicDeadlineScheduler().remove(this);
}

bool MonotonicDeadlineHandler::isRunning() const
{
    return scheduler_.getMonotonicDeadlineScheduler().doesExist(this);
}

/*
 * MonotonicDeadlineScheduler
 */
struct MonotonicDeadlineHandlerInsertionComparator
{
    const uint64_t ts;
    MonotonicDeadlineHandlerInsertionComparator(uint64_t ts) : ts(ts) { }
    bool operator()(const MonotonicDeadlineHandler* t) const
    {
        return t->getMonotonicDeadline() > ts;
    }
};

void MonotonicDeadlineScheduler::add(MonotonicDeadlineHandler* mdh)
{
    assert(mdh);
    handlers_.insertBefore(mdh, MonotonicDeadlineHandlerInsertionComparator(mdh->getMonotonicDeadline()));
}

void MonotonicDeadlineScheduler::remove(MonotonicDeadlineHandler* mdh)
{
    assert(mdh);
    handlers_.remove(mdh);
}

bool MonotonicDeadlineScheduler::doesExist(const MonotonicDeadlineHandler* mdh) const
{
    assert(mdh);
    const MonotonicDeadlineHandler* p = handlers_.get();
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
        if (p == mdh)
            return true;
        p = p->getNextListNode();
    }
    return false;
}

uint64_t MonotonicDeadlineScheduler::pollAndGetMonotonicTimestamp(ISystemClock& sysclock)
{
    while (true)
    {
        MonotonicDeadlineHandler* const mdh = handlers_.get();
        if (!mdh)
            return sysclock.getMonotonicMicroseconds();
#if UAVCAN_DEBUG
        if (mdh->getNextListNode())      // Order check
            assert(mdh->getMonotonicDeadline() <= mdh->getNextListNode()->getMonotonicDeadline());
#endif

        const uint64_t ts = sysclock.getMonotonicMicroseconds();
        if (ts < mdh->getMonotonicDeadline())
            return ts;

        handlers_.remove(mdh);
        mdh->onMonotonicDeadline(ts);   // This handler can be re-registered immediately
    }
    assert(0);
    return 0;
}

uint64_t MonotonicDeadlineScheduler::getEarliestDeadline() const
{
    const MonotonicDeadlineHandler* const mdh = handlers_.get();
    if (mdh)
        return mdh->getMonotonicDeadline();
    return std::numeric_limits<uint64_t>::max();
}

/*
 * Scheduler
 */
uint64_t Scheduler::computeDispatcherSpinDeadline(uint64_t spin_deadline) const
{
    const uint64_t earliest = std::min(deadline_scheduler_.getEarliestDeadline(), spin_deadline);
    const uint64_t ts = getMonotonicTimestamp();
    if (earliest > ts)
    {
        if (ts - earliest > monotonic_deadline_resolution_)
            return ts + monotonic_deadline_resolution_;
    }
    return earliest;
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

        const uint64_t ts = deadline_scheduler_.pollAndGetMonotonicTimestamp(getSystemClock());
        pollCleanup(ts, retval);
        if (ts >= monotonic_deadline)
            break;
    }
    return retval;
}

}
