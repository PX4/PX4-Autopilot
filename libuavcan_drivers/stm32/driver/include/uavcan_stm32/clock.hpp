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
 * Returns current monotonic time passed since the moment when clock::init() was called.
 * This function is thread safe.
 */
uavcan::MonotonicTime getMonotonic();

/**
 * Returns UTC time if it has been set, otherwise returns zero time.
 * This function is thread safe.
 */
uavcan::UtcTime getUtc();

/**
 * Performs UTC time adjustment.
 * The UTC time will be zero until first adjustment has been performed.
 * This function is thread safe.
 */
void adjustUtc(uavcan::UtcDuration adjustment);

/**
 * Clock speed error.
 * Positive if the hardware timer is slower than reference time.
 * This function is thread safe.
 */
uavcan::int32_t getUtcSpeedCorrectionPPM();

/**
 * Number of non-gradual adjustments performed so far.
 * Ideally should be zero.
 * This function is thread safe.
 */
uavcan::uint32_t getUtcAjdustmentJumpCount();

}

/**
 * Adapter for uavcan::ISystemClock.
 */
class SystemClock : public uavcan::ISystemClock, uavcan::Noncopyable
{
    SystemClock() { }

    static SystemClock self;

    virtual uavcan::MonotonicTime getMonotonic()     const { return clock::getMonotonic(); }
    virtual uavcan::UtcTime getUtc()                 const { return clock::getUtc(); }
    virtual void adjustUtc(uavcan::UtcDuration adjustment) { clock::adjustUtc(adjustment); }

public:
    /**
     * Calls clock::init() as needed.
     * This function is thread safe.
     */
    static SystemClock& instance();
};

}
