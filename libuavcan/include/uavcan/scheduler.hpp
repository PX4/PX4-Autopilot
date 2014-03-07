/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/timer.hpp>
#include <uavcan/internal/linked_list.hpp>
#include <uavcan/internal/transport/dispatcher.hpp>

namespace uavcan
{

class Scheduler : Noncopyable
{
    enum { DefaultTimerResolutionMs = 5 };
    enum { DefaultCleanupPeriodMs = 1000 };

    LinkedListRoot<TimerBase> ordered_timers_;  // Ordered by deadline, lowest first
    Dispatcher dispatcher_;
    uint64_t prev_cleanup_ts_;
    uint64_t timer_resolution_;
    uint64_t cleanup_period_;

    uint64_t computeDispatcherSpinDeadline(uint64_t spin_deadline) const;
    uint64_t pollTimersAndGetMonotonicTimestamp();
    void pollCleanup(uint64_t mono_ts, uint32_t num_frames_processed_with_last_spin);

public:
    Scheduler(ICanDriver& can_driver, IAllocator& allocator, ISystemClock& sysclock, IOutgoingTransferRegistry& otr,
             NodeID self_node_id)
    : dispatcher_(can_driver, allocator, sysclock, otr, self_node_id)
    , prev_cleanup_ts_(sysclock.getMonotonicMicroseconds())
    , timer_resolution_(DefaultTimerResolutionMs * 1000)
    , cleanup_period_(DefaultCleanupPeriodMs * 1000)
    { }

    int spin(uint64_t monotonic_deadline);

    void registerOneShotTimer(TimerBase* timer);
    void unregisterOneShotTimer(TimerBase* timer);
    bool isOneShotTimerRegistered(const TimerBase* timer) const;
    unsigned int getNumOneShotTimers() const { return ordered_timers_.getLength(); }

    Dispatcher& getDispatcher() { return dispatcher_; }

    ISystemClock& getSystemClock()         { return dispatcher_.getSystemClock(); }
    uint64_t getMonotonicTimestamp() const { return dispatcher_.getSystemClock().getMonotonicMicroseconds(); }
    uint64_t getUtcTimestamp()       const { return dispatcher_.getSystemClock().getUtcMicroseconds(); }

    uint64_t getTimerResolution() const { return timer_resolution_; }
    void setTimerResolution(uint64_t res_usec) { timer_resolution_ = res_usec; }

    uint64_t getCleanupPeriod() const { return cleanup_period_; }
    void setCleanupPeriod(uint64_t period_usec) { cleanup_period_ = period_usec; }
};

}
