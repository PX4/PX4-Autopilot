/*
 * System clock driver interface.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <stdint.h>

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
    /**
     * Monototic system clock in microseconds.
     * This shall never jump during UTC timestamp adjustments; the base time is irrelevant.
     * On POSIX systems use clock_gettime() with CLOCK_MONOTONIC.
     */
    virtual uint64_t getMonotonicMicroseconds() const = 0;

    /**
     * UTC clock in microseconds.
     * This can jump when the UTC timestamp is being adjusted.
     * Return 0 if the UTC time is not available yet (e.g. the device just started up with no battery clock).
     * On POSIX systems use gettimeofday().
     */
    virtual uint64_t getUtcMicroseconds() const = 0;

    /**
     * Set the UTC system clock.
     * @param [in] timestamp New UTC timestamp. Avoid when possible.
     * @param [in] offset    Current UTC time error. More precise than just timestamp, use it if possible.
     * For POSIX refer to adjtime(), settimeofday().
     */
    virtual void adjustUtcMicroseconds(uint64_t new_timestamp_usec, int64_t offset_usec) = 0;
};

}
