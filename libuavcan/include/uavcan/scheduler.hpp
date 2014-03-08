/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/internal/linked_list.hpp>
#include <uavcan/internal/transport/dispatcher.hpp>

namespace uavcan
{

class Scheduler;

class MonotonicDeadlineHandler : public LinkedListNode<MonotonicDeadlineHandler>, Noncopyable
{
    uint64_t monotonic_deadline_;

protected:
    Scheduler& scheduler_;

    explicit MonotonicDeadlineHandler(Scheduler& scheduler)
    : monotonic_deadline_(0)
    , scheduler_(scheduler)
    { }

    virtual ~MonotonicDeadlineHandler() { stop(); }

public:
    virtual void onMonotonicDeadline(uint64_t monotonic_timestamp) = 0;

    void startWithDeadline(uint64_t monotonic_deadline);
    void startWithDelay(uint64_t delay_usec);

    void stop();

    bool isRunning() const;

    uint64_t getMonotonicDeadline() const { return monotonic_deadline_; }
    Scheduler& getScheduler() const { return scheduler_; }
};


class MonotonicDeadlineScheduler : Noncopyable
{
    LinkedListRoot<MonotonicDeadlineHandler> handlers_;  // Ordered by deadline, lowest first

public:
    void add(MonotonicDeadlineHandler* mdh);
    void remove(MonotonicDeadlineHandler* mdh);
    bool doesExist(const MonotonicDeadlineHandler* mdh) const;
    unsigned int getNumHandlers() const { return handlers_.getLength(); }

    uint64_t pollAndGetMonotonicTimestamp(ISystemClock& sysclock);
    uint64_t getEarliestDeadline() const;
};


class Scheduler : Noncopyable
{
    enum { DefaultMonotonicDeadlineResolutionMs = 5 };
    enum { MinMonotonicDeadlineResolutionMs = 1 };
    enum { MaxMonotonicDeadlineResolutionMs = 100 };

    enum { DefaultCleanupPeriodMs = 1000 };
    enum { MinCleanupPeriodMs = 10 };
    enum { MaxCleanupPeriodMs = 10000 };

    MonotonicDeadlineScheduler deadline_scheduler_;
    Dispatcher dispatcher_;
    uint64_t prev_cleanup_ts_;
    uint64_t monotonic_deadline_resolution_;
    uint64_t cleanup_period_;

    uint64_t computeDispatcherSpinDeadline(uint64_t spin_deadline) const;
    void pollCleanup(uint64_t mono_ts, uint32_t num_frames_processed_with_last_spin);

public:
    Scheduler(ICanDriver& can_driver, IAllocator& allocator, ISystemClock& sysclock, IOutgoingTransferRegistry& otr,
             NodeID self_node_id)
    : dispatcher_(can_driver, allocator, sysclock, otr, self_node_id)
    , prev_cleanup_ts_(sysclock.getMonotonicMicroseconds())
    , monotonic_deadline_resolution_(DefaultMonotonicDeadlineResolutionMs * 1000)
    , cleanup_period_(DefaultCleanupPeriodMs * 1000)
    { }

    int spin(uint64_t monotonic_deadline);

    MonotonicDeadlineScheduler& getMonotonicDeadlineScheduler() { return deadline_scheduler_; }
    Dispatcher& getDispatcher() { return dispatcher_; }

    ISystemClock& getSystemClock()         { return dispatcher_.getSystemClock(); }
    uint64_t getMonotonicTimestamp() const { return dispatcher_.getSystemClock().getMonotonicMicroseconds(); }
    uint64_t getUtcTimestamp()       const { return dispatcher_.getSystemClock().getUtcMicroseconds(); }

    uint64_t getMonotonicDeadlineResolution() const { return monotonic_deadline_resolution_; }
    void setMonotonicDeadlineResolution(uint64_t res_usec)
    {
        res_usec = std::min(res_usec, MaxMonotonicDeadlineResolutionMs * uint64_t(1000));
        res_usec = std::max(res_usec, MinMonotonicDeadlineResolutionMs * uint64_t(1000));
        monotonic_deadline_resolution_ = res_usec;
    }

    uint64_t getCleanupPeriod() const { return cleanup_period_; }
    void setCleanupPeriod(uint64_t period_usec)
    {
        period_usec = std::min(period_usec, MaxCleanupPeriodMs * uint64_t(1000));
        period_usec = std::max(period_usec, MinCleanupPeriodMs * uint64_t(1000));
        cleanup_period_ = period_usec;
    }
};

}
