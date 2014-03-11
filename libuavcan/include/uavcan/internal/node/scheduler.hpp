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
    MonotonicTime deadline_;

protected:
    Scheduler& scheduler_;

    explicit MonotonicDeadlineHandler(Scheduler& scheduler)
    : scheduler_(scheduler)
    { }

    virtual ~MonotonicDeadlineHandler() { stop(); }

public:
    virtual void handleDeadline(MonotonicTime current_timestamp) = 0;

    void startWithDeadline(MonotonicTime deadline);
    void startWithDelay(MonotonicDuration delay);

    void stop();

    bool isRunning() const;

    MonotonicTime getDeadline() const { return deadline_; }
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

    MonotonicTime pollAndGetMonotonicTimestamp(ISystemClock& sysclock);
    MonotonicTime getEarliestDeadline() const;
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
    MonotonicTime prev_cleanup_ts_;
    MonotonicDuration deadline_resolution_;
    MonotonicDuration cleanup_period_;

    MonotonicTime computeDispatcherSpinDeadline(MonotonicTime spin_deadline) const;
    void pollCleanup(MonotonicTime mono_ts, uint32_t num_frames_processed_with_last_spin);

public:
    Scheduler(ICanDriver& can_driver, IAllocator& allocator, ISystemClock& sysclock, IOutgoingTransferRegistry& otr,
             NodeID self_node_id)
    : dispatcher_(can_driver, allocator, sysclock, otr, self_node_id)
    , prev_cleanup_ts_(sysclock.getMonotonic())
    , deadline_resolution_(MonotonicDuration::fromMSec(DefaultMonotonicDeadlineResolutionMs))
    , cleanup_period_(MonotonicDuration::fromMSec(DefaultCleanupPeriodMs))
    { }

    int spin(MonotonicTime deadline);

    MonotonicDeadlineScheduler& getMonotonicDeadlineScheduler() { return deadline_scheduler_; }
    Dispatcher& getDispatcher() { return dispatcher_; }

    ISystemClock& getSystemClock()              { return dispatcher_.getSystemClock(); }
    MonotonicTime getMonotonicTimestamp() const { return dispatcher_.getSystemClock().getMonotonic(); }
    UtcTime getUtcTimestamp()             const { return dispatcher_.getSystemClock().getUtc(); }

    MonotonicDuration getMonotonicDeadlineResolution() const { return deadline_resolution_; }
    void setMonotonicDeadlineResolution(MonotonicDuration res)
    {
        res = std::min(res, MonotonicDuration::fromMSec(MaxMonotonicDeadlineResolutionMs));
        res = std::max(res, MonotonicDuration::fromMSec(MinMonotonicDeadlineResolutionMs));
        deadline_resolution_ = res;
    }

    MonotonicDuration getCleanupPeriod() const { return cleanup_period_; }
    void setCleanupPeriod(MonotonicDuration period)
    {
        period = std::min(period, MonotonicDuration::fromMSec(MaxCleanupPeriodMs));
        period = std::max(period, MonotonicDuration::fromMSec(MinCleanupPeriodMs));
        cleanup_period_ = period;
    }
};

}
