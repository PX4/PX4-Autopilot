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
    MonotonicTime scheduled_deadline;
    MonotonicTime current_timestamp;
    Timer* timer;

    TimerEvent(Timer* timer, MonotonicTime scheduled_deadline, MonotonicTime current_timestamp)
    : scheduled_deadline(scheduled_deadline)
    , current_timestamp(current_timestamp)
    , timer(timer)
    { }
};


class Timer : private MonotonicDeadlineHandler
{
    MonotonicDuration period_;

    void handleDeadline(MonotonicTime current_timestamp);

public:
    using MonotonicDeadlineHandler::stop;
    using MonotonicDeadlineHandler::isRunning;
    using MonotonicDeadlineHandler::getDeadline;
    using MonotonicDeadlineHandler::getScheduler;

    explicit Timer(Scheduler& scheduler)
    : MonotonicDeadlineHandler(scheduler)
    , period_(MonotonicDuration::getInfinite())
    { }

    void startOneShotWithDeadline(MonotonicTime deadline);
    void startOneShotWithDelay(MonotonicDuration delay);
    void startPeriodic(MonotonicDuration period);

    MonotonicDuration getPeriod() const { return period_; }

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
