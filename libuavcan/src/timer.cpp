/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <uavcan/timer.hpp>

namespace uavcan
{

void Timer::handleDeadline(MonotonicTime current_timestamp)
{
    assert(!isRunning());

    const MonotonicTime scheduled_deadline = getDeadline();

    if (period_ < MonotonicDuration::getInfinite())
        startWithDeadline(scheduled_deadline + period_);

    // Application can re-register the timer with different params, it's OK
    handleTimerEvent(TimerEvent(this, scheduled_deadline, current_timestamp));
}

void Timer::startOneShotWithDeadline(MonotonicTime deadline)
{
    stop();
    period_ = MonotonicDuration::getInfinite();
    MonotonicDeadlineHandler::startWithDeadline(deadline);
}

void Timer::startOneShotWithDelay(MonotonicDuration delay)
{
    stop();
    period_ = MonotonicDuration::getInfinite();
    MonotonicDeadlineHandler::startWithDelay(delay);
}

void Timer::startPeriodic(MonotonicDuration period)
{
    assert(period < MonotonicDuration::getInfinite());
    stop();
    period_ = period;
    MonotonicDeadlineHandler::startWithDelay(period);
}

}
