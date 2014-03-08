/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <uavcan/timer.hpp>

namespace uavcan
{

const uint64_t Timer::InfinitePeriod;

void Timer::onMonotonicDeadline(uint64_t monotonic_timestamp)
{
    assert(!isRunning());

    if (period_ != InfinitePeriod)
        startWithDeadline(getMonotonicDeadline() + period_);

    // Application can re-register the timer with different params, it's OK
    onTimerEvent(TimerEvent(this, getMonotonicDeadline(), monotonic_timestamp));
}

void Timer::startOneShotWithDeadline(uint64_t monotonic_deadline_usec)
{
    period_ = InfinitePeriod;
    MonotonicDeadlineHandler::startWithDeadline(monotonic_deadline_usec);
}

void Timer::startOneShotWithDelay(uint64_t delay_usec)
{
    period_ = InfinitePeriod;
    MonotonicDeadlineHandler::startWithDelay(delay_usec);
}

void Timer::startPeriodic(uint64_t period_usec)
{
    assert(period_usec != InfinitePeriod);
    stop();
    period_ = period_usec;
    MonotonicDeadlineHandler::startWithDelay(period_usec);
}

}
