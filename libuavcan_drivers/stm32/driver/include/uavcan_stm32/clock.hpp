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
 * Returns current monotonic time since the moment when clock::init() was called.
 * This function is thread safe.
 */
uavcan::MonotonicTime getMonotonic();

/**
 * Returns UTC time if it has been set, otherwise returns zero time.
 * This function is thread safe.
 */
uavcan::UtcTime getUtc();

/**
 * Performs UTC phase and frequency adjustment.
 * The UTC time will be zero until first adjustment has been performed.
 * This function is thread safe.
 */
void adjustUtc(uavcan::UtcDuration adjustment);

/**
 * UTC clock synchronization parameters
 */
struct UtcSyncParams
{
    float offset_p = 0.01F;                  ///< PPM per one usec error
    float rate_i = 0.02F;                    ///< PPM per one PPM error for second
    float rate_error_corner_freq = 0.01F;
    float max_rate_correction_ppm = 300.0F;
    float lock_thres_rate_ppm = 2.0F;
    uavcan::UtcDuration lock_thres_offset = uavcan::UtcDuration::fromMSec(4);
    uavcan::UtcDuration min_jump = uavcan::UtcDuration::fromMSec(10); ///< Min error to jump rather than change rate
};

/**
 * Clock rate error.
 * Positive if the hardware timer is slower than reference time.
 * This function is thread safe.
 */
float getUtcRateCorrectionPPM();

/**
 * Number of non-gradual adjustments performed so far.
 * Ideally should be zero.
 * This function is thread safe.
 */
uavcan::uint32_t getUtcJumpCount();

/**
 * Whether UTC is synchronized and locked.
 * This function is thread safe.
 */
bool isUtcLocked();

/**
 * UTC sync params get/set.
 * Both functions are thread safe.
 */
UtcSyncParams getUtcSyncParams();
void setUtcSyncParams(const UtcSyncParams& params);

}

/**
 * Adapter for uavcan::ISystemClock.
 */
class SystemClock : public uavcan::ISystemClock, uavcan::Noncopyable
{
    SystemClock() { }

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
