/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <uavcan/node/scheduler.hpp>
#include <uavcan/debug.hpp>

namespace uavcan
{
/*
 * MonotonicDeadlineHandler
 */
void DeadlineHandler::startWithDeadline(MonotonicTime deadline)
{
    assert(!deadline.isZero());
    stop();
    deadline_ = deadline;
    scheduler_.getDeadlineScheduler().add(this);
}

void DeadlineHandler::startWithDelay(MonotonicDuration delay)
{
    startWithDeadline(scheduler_.getMonotonicTime() + delay);
}

void DeadlineHandler::stop()
{
    scheduler_.getDeadlineScheduler().remove(this);
}

bool DeadlineHandler::isRunning() const
{
    return scheduler_.getDeadlineScheduler().doesExist(this);
}

/*
 * MonotonicDeadlineScheduler
 */
struct MonotonicDeadlineHandlerInsertionComparator
{
    const MonotonicTime ts;
    explicit MonotonicDeadlineHandlerInsertionComparator(MonotonicTime arg_ts) : ts(arg_ts) { }
    bool operator()(const DeadlineHandler* t) const
    {
        return t->getDeadline() > ts;
    }
};

void DeadlineScheduler::add(DeadlineHandler* mdh)
{
    assert(mdh);
    handlers_.insertBefore(mdh, MonotonicDeadlineHandlerInsertionComparator(mdh->getDeadline()));
}

void DeadlineScheduler::remove(DeadlineHandler* mdh)
{
    assert(mdh);
    handlers_.remove(mdh);
}

bool DeadlineScheduler::doesExist(const DeadlineHandler* mdh) const
{
    assert(mdh);
    const DeadlineHandler* p = handlers_.get();
#if UAVCAN_DEBUG
    MonotonicTime prev_deadline;
#endif
    while (p)
    {
#if UAVCAN_DEBUG
        if (prev_deadline > p->getDeadline())  // Self check
        {
            std::abort();
        }
        prev_deadline = p->getDeadline();
#endif
        if (p == mdh)
        {
            return true;
        }
        p = p->getNextListNode();
    }
    return false;
}

MonotonicTime DeadlineScheduler::pollAndGetMonotonicTime(ISystemClock& sysclock)
{
    while (true)
    {
        DeadlineHandler* const mdh = handlers_.get();
        if (!mdh)
        {
            return sysclock.getMonotonic();
        }
#if UAVCAN_DEBUG
        if (mdh->getNextListNode())      // Order check
        {
            assert(mdh->getDeadline() <= mdh->getNextListNode()->getDeadline());
        }
#endif

        const MonotonicTime ts = sysclock.getMonotonic();
        if (ts < mdh->getDeadline())
        {
            return ts;
        }

        handlers_.remove(mdh);
        mdh->handleDeadline(ts);   // This handler can be re-registered immediately
    }
    assert(0);
    return MonotonicTime();
}

MonotonicTime DeadlineScheduler::getEarliestDeadline() const
{
    const DeadlineHandler* const mdh = handlers_.get();
    if (mdh)
    {
        return mdh->getDeadline();
    }
    return MonotonicTime::getMax();
}

/*
 * Scheduler
 */
MonotonicTime Scheduler::computeDispatcherSpinDeadline(MonotonicTime spin_deadline) const
{
    const MonotonicTime earliest = std::min(deadline_scheduler_.getEarliestDeadline(), spin_deadline);
    const MonotonicTime ts = getMonotonicTime();
    if (earliest > ts)
    {
        if (ts - earliest > deadline_resolution_)
        {
            return ts + deadline_resolution_;
        }
    }
    return earliest;
}

void Scheduler::pollCleanup(MonotonicTime mono_ts, uint32_t num_frames_processed_with_last_spin)
{
    // cleanup will be performed less frequently if the stack handles more frames per second
    const MonotonicTime deadline = prev_cleanup_ts_ + cleanup_period_ * (num_frames_processed_with_last_spin + 1);
    if (mono_ts > deadline)
    {
        //UAVCAN_TRACE("Scheduler", "Cleanup with %u processed frames", num_frames_processed_with_last_spin);
        prev_cleanup_ts_ = mono_ts;
        dispatcher_.cleanup(mono_ts);
    }
}

int Scheduler::spin(MonotonicTime deadline)
{
    if (inside_spin_)  // Preventing recursive calls
    {
        assert(0);
        return -ErrRecursiveCall;
    }
    inside_spin_ = true;

    int retval = 0;
    while (true)
    {
        const MonotonicTime dl = computeDispatcherSpinDeadline(deadline);
        retval = dispatcher_.spin(dl);
        if (retval < 0)
        {
            break;
        }

        const MonotonicTime ts = deadline_scheduler_.pollAndGetMonotonicTime(getSystemClock());
        pollCleanup(ts, retval);
        if (ts >= deadline)
        {
            break;
        }
    }

    inside_spin_ = false;
    return retval;
}

}
