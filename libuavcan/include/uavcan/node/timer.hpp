/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/stdint.hpp>
#include <uavcan/error.hpp>
#include <uavcan/impl_constants.hpp>
#include <uavcan/util/linked_list.hpp>
#include <uavcan/node/scheduler.hpp>
#include <uavcan/node/abstract_node.hpp>
#include <uavcan/util/templates.hpp>

#if !defined(UAVCAN_CPP11) || !defined(UAVCAN_CPP_VERSION)
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
# include <functional>
#endif

namespace uavcan
{

class UAVCAN_EXPORT TimerBase;

struct UAVCAN_EXPORT TimerEvent
{
    MonotonicTime scheduled_time;
    MonotonicTime real_time;

    TimerEvent(MonotonicTime arg_scheduled_time, MonotonicTime arg_real_time)
        : scheduled_time(arg_scheduled_time)
        , real_time(arg_real_time)
    { }
};


class UAVCAN_EXPORT TimerBase : private DeadlineHandler
{
    MonotonicDuration period_;

    virtual void handleDeadline(MonotonicTime current);

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


template <typename Callback_>
class UAVCAN_EXPORT TimerEventForwarder : public TimerBase
{
public:
    typedef Callback_ Callback;

private:
    Callback callback_;

    virtual void handleTimerEvent(const TimerEvent& event)
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

    TimerEventForwarder(INode& node, const Callback& callback)
        : TimerBase(node)
        , callback_(callback)
    { }

    const Callback& getCallback() const { return callback_; }
    void setCallback(const Callback& callback) { callback_ = callback; }
};


#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

typedef TimerEventForwarder<std::function<void (const TimerEvent& event)> > Timer;

#endif

}
