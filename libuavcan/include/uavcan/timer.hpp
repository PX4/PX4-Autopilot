/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <stdint.h>
#include <uavcan/internal/node/scheduler.hpp>
#include <uavcan/util/compile_time.hpp>
#include <uavcan/internal/linked_list.hpp>
#include <uavcan/internal/fatal_error.hpp>

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

    void handleMonotonicDeadline(uint64_t monotonic_timestamp);

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

    virtual void handleTimerEvent(const TimerEvent& event) = 0;
};


template <typename Callback>
class TimerEventForwarder : public Timer
{
    Callback callback_;

public:
    explicit TimerEventForwarder(Scheduler& node)
    : Timer(node)
    , callback_()
    { }

    TimerEventForwarder(Scheduler& node, Callback callback)
    : Timer(node)
    , callback_(callback)
    { }

    const Callback& getCallback() const { return callback_; }
    void setCallback(const Callback& callback) { callback_ = callback; }

    void handleTimerEvent(const TimerEvent& event)
    {
        if (try_implicit_cast<bool>(callback_, true))
            callback_(event);
        else
            handleFatalError("Invalid timer callback");
    }
};

}
