/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <ctime>
#include <cstdint>

#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>

#include <uavcan/driver/system_clock.hpp>
#include <uavcan_linux/exception.hpp>

namespace uavcan_linux
{
/**
 * Different adjustment modes can be used for time synchronization
 */
enum class ClockAdjustmentMode
{
    SystemWide,      ///< Adjust the clock globally for the whole system; requires root privileges
    PerDriverPrivate ///< Adjust the clock only for the current driver instance
};

/**
 * Linux system clock driver.
 * Requires librt.
 */
class SystemClock : public uavcan::ISystemClock
{
    uavcan::UtcDuration private_adj_;
    uavcan::UtcDuration gradual_adj_limit_;
    const ClockAdjustmentMode adj_mode_;
    std::uint64_t step_adj_cnt_;
    std::uint64_t gradual_adj_cnt_;

    static constexpr std::int64_t Int1e6   = 1000000;
    static constexpr std::uint64_t UInt1e6 = 1000000;

    bool performStepAdjustment(const uavcan::UtcDuration adjustment)
    {
        step_adj_cnt_++;
        const std::int64_t usec = adjustment.toUSec();
        timeval tv;
        if (gettimeofday(&tv, NULL) != 0)
        {
            return false;
        }
        tv.tv_sec  += usec / Int1e6;
        tv.tv_usec += usec % Int1e6;
        return settimeofday(&tv, nullptr) == 0;
    }

    bool performGradualAdjustment(const uavcan::UtcDuration adjustment)
    {
        gradual_adj_cnt_++;
        const std::int64_t usec = adjustment.toUSec();
        timeval tv;
        tv.tv_sec  = usec / Int1e6;
        tv.tv_usec = usec % Int1e6;
        return adjtime(&tv, nullptr) == 0;
    }

public:
    /**
     * By default, the clock adjustment mode will be selected automatically - global if root, private otherwise.
     */
    explicit SystemClock(ClockAdjustmentMode adj_mode = detectPreferredClockAdjustmentMode())
        : gradual_adj_limit_(uavcan::UtcDuration::fromMSec(4000))
        , adj_mode_(adj_mode)
        , step_adj_cnt_(0)
        , gradual_adj_cnt_(0)
    { }

    /**
     * Returns monotonic timestamp from librt.
     * @throws uavcan_linux::Exception.
     */
    uavcan::MonotonicTime getMonotonic() const override
    {
        timespec ts;
        if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
        {
            throw Exception("Failed to get monotonic time");
        }
        return uavcan::MonotonicTime::fromUSec(std::uint64_t(ts.tv_sec) * UInt1e6 + ts.tv_nsec / 1000);
    }

    /**
     * Returns wall time from gettimeofday().
     * @throws uavcan_linux::Exception.
     */
    uavcan::UtcTime getUtc() const override
    {
        timeval tv;
        if (gettimeofday(&tv, NULL) != 0)
        {
            throw Exception("Failed to get UTC time");
        }
        uavcan::UtcTime utc = uavcan::UtcTime::fromUSec(std::uint64_t(tv.tv_sec) * UInt1e6 + tv.tv_usec);
        if (adj_mode_ == ClockAdjustmentMode::PerDriverPrivate)
        {
            utc += private_adj_;
        }
        return utc;
    }

    /**
     * Adjusts the wall clock.
     * Behavior depends on the selected clock adjustment mode - @ref ClockAdjustmentMode.
     * Clock adjustment mode can be set only once via constructor.
     *
     * If the system wide adjustment mode is selected, two ways for performing adjustment exist:
     *  - Gradual adjustment using adjtime(), if the phase error is less than gradual adjustment limit.
     *  - Step adjustment using settimeofday(), if the phase error is above gradual adjustment limit.
     * The gradual adjustment limit can be configured at any time via the setter method.
     *
     * @throws uavcan_linux::Exception.
     */
    void adjustUtc(const uavcan::UtcDuration adjustment) override
    {
        if (adj_mode_ == ClockAdjustmentMode::PerDriverPrivate)
        {
            private_adj_ += adjustment;
        }
        else
        {
            assert(private_adj_.isZero());
            assert(!gradual_adj_limit_.isNegative());

            bool success = false;
            if (adjustment.getAbs() < gradual_adj_limit_)
            {
                success = performGradualAdjustment(adjustment);
            }
            else
            {
                success = performStepAdjustment(adjustment);
            }
            if (!success)
            {
                throw Exception("Clock adjustment failed");
            }
        }
    }

    /**
     * Sets the maximum phase error to use adjtime().
     * If the phase error exceeds this value, settimeofday() will be used instead.
     */
    void setGradualAdjustmentLimit(uavcan::UtcDuration limit)
    {
        if (limit.isNegative())
        {
            limit = uavcan::UtcDuration();
        }
        gradual_adj_limit_ = limit;
    }

    uavcan::UtcDuration getGradualAdjustmentLimit() const { return gradual_adj_limit_; }

    ClockAdjustmentMode getAdjustmentMode() const { return adj_mode_; }

    /**
     * This is only applicable if the selected clock adjustment mode is private.
     * In system wide mode this method will always return zero duration.
     */
    uavcan::UtcDuration getPrivateAdjustment() const { return private_adj_; }

    /**
     * Statistics that allows to evaluate clock sync preformance.
     */
    std::uint64_t getStepAdjustmentCount() const { return step_adj_cnt_; }
    std::uint64_t getGradualAdjustmentCount() const { return gradual_adj_cnt_; }
    std::uint64_t getAdjustmentCount() const
    {
        return getStepAdjustmentCount() + getGradualAdjustmentCount();
    }

    /**
     * This static method decides what is the optimal clock sync adjustment mode for the current configuration.
     * It selects system wide mode if the application is running as root; otherwise it prefers
     * the private adjustment mode because the system wide mode requires root privileges.
     */
    static ClockAdjustmentMode detectPreferredClockAdjustmentMode()
    {
        const bool godmode = geteuid() == 0;
        return godmode ? ClockAdjustmentMode::SystemWide : ClockAdjustmentMode::PerDriverPrivate;
    }
};

}
