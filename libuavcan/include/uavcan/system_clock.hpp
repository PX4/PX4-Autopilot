/*
 * System clock driver interface.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <stdint.h>
#include <uavcan/internal/impl_constants.hpp>
#include <uavcan/time.hpp>

namespace uavcan
{

/**
 * System clock interface - monotonic and UTC.
 * Note that this library uses microseconds for all time related values (timestamps, deadlines, delays),
 * if not explicitly specified otherwise.
 * All time values are represented as 64-bit integers.
 */
class ISystemClock
{
public:
    virtual ~ISystemClock() { }

    /**
     * Monototic system clock.
     * This shall never jump during UTC timestamp adjustments; the base time is irrelevant.
     * On POSIX systems use clock_gettime() with CLOCK_MONOTONIC.
     */
    virtual MonotonicTime getMonotonic() const = 0;

    /**
     * UTC clock.
     * This can jump when the UTC timestamp is being adjusted.
     * Return 0 if the UTC time is not available yet (e.g. the device just started up with no battery clock).
     * On POSIX systems use gettimeofday().
     */
    virtual UtcTime getUtc() const = 0;

    /**
     * Set the UTC system clock.
     * @param [in] timestamp New UTC timestamp. Avoid when possible.
     * @param [in] offset    Current UTC time error. More precise than just timestamp, use it if possible.
     * For POSIX refer to adjtime(), settimeofday().
     */
    virtual void adjustUtc(UtcTime new_time, UtcDuration offset) = 0;
};

}
