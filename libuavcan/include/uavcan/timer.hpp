/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <stdint.h>
#include <uavcan/internal/util.hpp>
#include <uavcan/internal/linked_list.hpp>

namespace uavcan
{

class Scheduler;
class TimerBase;

struct TimerEvent
{
    uint64_t scheduled_monotonic_deadline;
    uint64_t monotonic_timestamp;
    TimerBase* timer;

    TimerEvent(uint64_t scheduled_monotonic_deadline, uint64_t monotonic_timestamp, TimerBase& timer)
    : scheduled_monotonic_deadline(scheduled_monotonic_deadline)
    , monotonic_timestamp(monotonic_timestamp)
    , timer(&timer)
    { }
};


class TimerBase : public LinkedListNode<TimerBase>, Noncopyable
{
    friend class Scheduler;

    uint64_t monotonic_deadline_;
    uint64_t period_;
    Scheduler& scheduler_;

    void handleOneShotTimeout(uint64_t ts_monotonic);
    void genericStart();

public:
    static const uint64_t InfinitePeriod = 0xFFFFFFFFFFFFFFFFUL;

    TimerBase(Scheduler& scheduler)
    : monotonic_deadline_(0)
    , period_(InfinitePeriod)
    , scheduler_(scheduler)
    { }

    virtual ~TimerBase() { stop(); }

    uint64_t getMonotonicDeadline() const { return monotonic_deadline_; }

    void startOneShotDeadline(uint64_t monotonic_deadline_usec);
    void startOneShotDelay(uint64_t delay_usec);
    void startPeriodic(uint64_t period_usec);

    void stop();
    bool isRunning() const;

    uint64_t getPeriod() const { return period_; }

    virtual void onTimerEvent(TimerEvent& event) = 0;
};


template <typename Functor>
class Timer : public TimerBase
{
    Functor functor_;

public:
    Timer(Scheduler& node, Functor functor)
    : TimerBase(node)
    , functor_(functor)
    { }

    const Functor& getFunctor() const { return functor_; }

    void onTimerEvent(TimerEvent& event)
    {
        functor_(event);
    }
};

}
