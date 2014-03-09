/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <stdint.h>
#include <uavcan/scheduler.hpp>
#include <uavcan/internal/util.hpp>
#include <uavcan/internal/linked_list.hpp>

namespace uavcan
{

class Timer;

struct TimerEvent
{
    uint64_t scheduled_monotonic_deadline;
    uint64_t monotonic_timestamp;
    Timer* timer;

    TimerEvent(Timer* timer, uint64_t scheduled_monotonic_deadline, uint64_t monotonic_timestamp)
    : scheduled_monotonic_deadline(scheduled_monotonic_deadline)
    , monotonic_timestamp(monotonic_timestamp)
    , timer(timer)
    { }
};


class Timer : private MonotonicDeadlineHandler
{
    uint64_t period_;

    void onMonotonicDeadline(uint64_t monotonic_timestamp);

public:
    static const uint64_t InfinitePeriod = 0xFFFFFFFFFFFFFFFFUL;

    using MonotonicDeadlineHandler::stop;
    using MonotonicDeadlineHandler::isRunning;
    using MonotonicDeadlineHandler::getMonotonicDeadline;
    using MonotonicDeadlineHandler::getScheduler;

    explicit Timer(Scheduler& scheduler)
    : MonotonicDeadlineHandler(scheduler)
    , period_(InfinitePeriod)
    { }

    void startOneShotWithDeadline(uint64_t monotonic_deadline);
    void startOneShotWithDelay(uint64_t delay_usec);
    void startPeriodic(uint64_t period_usec);

    uint64_t getPeriod() const { return period_; }

    virtual void onTimerEvent(const TimerEvent& event) = 0;
};


template <typename Functor>
class TimerEventForwarder : public Timer
{
    Functor functor_;

public:
    TimerEventForwarder(Scheduler& node, Functor functor)
    : Timer(node)
    , functor_(functor)
    {
        assert(try_implicit_cast<bool>(functor, true));
    }

    const Functor& getFunctor() const { return functor_; }

    void onTimerEvent(const TimerEvent& event)
    {
        if (try_implicit_cast<bool>(functor_, true))
            functor_(event);
        else
            assert(0);
    }
};

}
