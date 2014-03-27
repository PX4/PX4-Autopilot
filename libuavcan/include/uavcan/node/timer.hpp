/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/stdint.hpp>
#include <uavcan/node/scheduler.hpp>
#include <uavcan/util/compile_time.hpp>
#include <uavcan/node/abstract_node.hpp>
#include <uavcan/linked_list.hpp>
#include <uavcan/fatal_error.hpp>

namespace uavcan
{

class TimerBase;

struct TimerEvent
{
    MonotonicTime scheduled_time;
    MonotonicTime real_time;

    TimerEvent(MonotonicTime scheduled_time, MonotonicTime real_time)
        : scheduled_time(scheduled_time)
        , real_time(real_time)
    { }
};


class TimerBase : private DeadlineHandler
{
    MonotonicDuration period_;

    void handleDeadline(MonotonicTime current);

public:
    using DeadlineHandler::stop;
    using DeadlineHandler::isRunning;
    using DeadlineHandler::getDeadline;
    using DeadlineHandler::getScheduler;

    explicit TimerBase(INode& node)
        : DeadlineHandler(node.getScheduler())
        , period_(MonotonicDuration::getInfinite())
    { }

    void startOneShotWithDeadline(MonotonicTime deadline);
    void startOneShotWithDelay(MonotonicDuration delay);
    void startPeriodic(MonotonicDuration period);

    MonotonicDuration getPeriod() const { return period_; }

    virtual void handleTimerEvent(const TimerEvent& event) = 0;
};


template <typename Callback>
class TimerEventForwarder : public TimerBase
{
    Callback callback_;

    void handleTimerEvent(const TimerEvent& event)
    {
        if (try_implicit_cast<bool>(callback_, true))
        {
            callback_(event);
        }
        else
        {
            handleFatalError("Invalid timer callback");
        }
    }

public:
    explicit TimerEventForwarder(INode& node)
        : TimerBase(node)
        , callback_()
    { }

    TimerEventForwarder(INode& node, Callback callback)
        : TimerBase(node)
        , callback_(callback)
    { }

    const Callback& getCallback() const { return callback_; }
    void setCallback(const Callback& callback) { callback_ = callback; }
};

}
