/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/driver/system_clock.hpp>

namespace uavcan_stm32
{

namespace clock
{
/**
 * Starts the clock.
 * Can be called multiple times, only the first call will be effective.
 */
void init();

/**
 * For CAN timestamping.
 */
uavcan::uint64_t getUtcUSecFromInterrupt();

/**
 * For general usage.
 */
uavcan::MonotonicTime getMonotonic();
uavcan::UtcTime getUtc();

/**
 * Performs UTC time adjustment.
 * The UTC time will be zero until first adjustment has been performed.
 */
void adjustUtc(uavcan::UtcDuration adjustment);

/**
 * Clock speed error.
 * Positive if the hardware timer is slower than reference time
 */
uavcan::int32_t getUtcSpeedCorrectionPPM();

/**
 * Number of non-gradual adjustments performed so far.
 * Ideally should be zero.
 */
uavcan::uint32_t getUtcAjdustmentJumpCount();

}

/**
 * Clock interface for CAN driver.
 */
class ICanTimestampingClock : public uavcan::ISystemClock
{
public:
    virtual uavcan::uint64_t getUtcUSecFromInterrupt() const = 0;
};

/**
 * Trivial system clock implementation; can be redefined by the application.
 * Uses a simple 16-bit hardware timer for both UTC and monotonic clocks.
 */
class SystemClock : public ICanTimestampingClock, uavcan::Noncopyable
{
    SystemClock() { }

public:
    static SystemClock& instance();

    virtual uavcan::uint64_t getUtcUSecFromInterrupt() const { return clock::getUtcUSecFromInterrupt(); }

    virtual uavcan::MonotonicTime getMonotonic()     const { return clock::getMonotonic(); }
    virtual uavcan::UtcTime getUtc()                 const { return clock::getUtc(); }
    virtual void adjustUtc(uavcan::UtcDuration adjustment) { clock::adjustUtc(adjustment); }
};

}
