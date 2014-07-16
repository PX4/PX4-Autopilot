/*
 * System clock driver interface.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/stdint.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/time.hpp>

namespace uavcan
{

/**
 * System clock interface - monotonic and UTC.
 */
class UAVCAN_EXPORT ISystemClock
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
     * @param [in] adjustment Amount of time to add to the clock value.
     * For POSIX refer to adjtime(), settimeofday().
     */
    virtual void adjustUtc(UtcDuration adjustment) = 0;
};

}
