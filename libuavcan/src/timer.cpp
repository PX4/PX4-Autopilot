/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <uavcan/timer.hpp>
#include <uavcan/scheduler.hpp>

namespace uavcan
{

const uint64_t TimerBase::InfinitePeriod;

void TimerBase::handleOneShotTimeout(uint64_t ts_monotonic)
{
    assert(!scheduler_.isOneShotTimerRegistered(this));

    if (period_ != InfinitePeriod)
    {
        monotonic_deadline_ += period_;
        genericStart();
    }

    // Application can re-register the timer with different params, it's OK
    TimerEvent event(monotonic_deadline_, ts_monotonic, *this);
    onTimerEvent(event);
}

void TimerBase::genericStart()
{
    scheduler_.registerOneShotTimer(this);
}

void TimerBase::startOneShotDeadline(uint64_t monotonic_deadline_usec)
{
    assert(monotonic_deadline_usec > 0);
    stop();
    period_ = InfinitePeriod;
    monotonic_deadline_ = monotonic_deadline_usec;
    genericStart();
}

void TimerBase::startOneShotDelay(uint64_t delay_usec)
{
    stop();
    startOneShotDeadline(scheduler_.getMonotonicTimestamp() + delay_usec);
}

void TimerBase::startPeriodic(uint64_t period_usec)
{
    assert(period_usec != InfinitePeriod);
    stop();
    period_ = period_usec;
    monotonic_deadline_ = scheduler_.getMonotonicTimestamp() + period_usec;
    genericStart();
}

void TimerBase::stop()
{
    scheduler_.unregisterOneShotTimer(this);
}

bool TimerBase::isRunning() const
{
    return scheduler_.isOneShotTimerRegistered(this);
}

}
