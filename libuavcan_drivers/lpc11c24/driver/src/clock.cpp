/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan_lpc11c24/clock.hpp>
#include <uavcan/util/templates.hpp>
#include <chip.h>
#include "internal.hpp"

namespace uavcan_lpc11c24
{
namespace clock
{
namespace
{

bool initialized = false;
bool utc_set = false;

std::int32_t utc_correction_usec_per_overflow_x16 = 0;
std::int64_t prev_adjustment = 0;

std::uint64_t time_mono = 0;
std::uint64_t time_utc = 0;

/**
 * If this value is too large for the given core clock, reload value will be out of the 24-bit integer range.
 * This will be detected at run time during timer initialization - refer to SysTick_Config().
 */
constexpr std::uint32_t USecPerOverflow = 65536 * 2;
constexpr std::int32_t MaxUtcSpeedCorrectionX16 = 100 * 16;

}

#if __GNUC__
__attribute__((noreturn))
#endif
static void fail()
{
    while (true) { }
}

void init()
{
    CriticalSectionLocker lock;
    if (!initialized)
    {
        initialized = true;

        if ((SystemCoreClock % 1000000) != 0)  // Core clock frequency validation
        {
            fail();
        }

        if (SysTick_Config((SystemCoreClock / 1000000) * USecPerOverflow) != 0)
        {
            fail();
        }
    }
}

static std::uint64_t sampleFromCriticalSection(const volatile std::uint64_t* const value)
{
    const std::uint32_t reload = SysTick->LOAD + 1;  // SysTick counts downwards, hence the value subtracted from reload

    volatile std::uint64_t time = *value;
    volatile std::uint32_t cycles = reload - SysTick->VAL;

    if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) == SCB_ICSR_PENDSTSET_Msk)
    {
        cycles = reload - SysTick->VAL;
        time += USecPerOverflow;
    }
    const std::uint32_t cycles_per_usec = SystemCoreClock / 1000000;
    return time + (cycles / cycles_per_usec);
}

std::uint64_t getUtcUSecFromCanInterrupt()
{
    return utc_set ? sampleFromCriticalSection(&time_utc) : 0;
}

uavcan::MonotonicTime getMonotonic()
{
    if (!initialized)
    {
        fail();
    }
    std::uint64_t usec = 0;
    {
        CriticalSectionLocker locker;
        usec = sampleFromCriticalSection(&time_mono);
    }
    return uavcan::MonotonicTime::fromUSec(usec);
}

uavcan::UtcTime getUtc()
{
    if (!initialized)
    {
        fail();
    }
    std::uint64_t usec = 0;
    if (utc_set)
    {
        CriticalSectionLocker locker;
        usec = sampleFromCriticalSection(&time_utc);
    }
    return uavcan::UtcTime::fromUSec(usec);
}

uavcan::UtcDuration getPrevUtcAdjustment()
{
    return uavcan::UtcDuration::fromUSec(prev_adjustment);
}

void adjustUtc(uavcan::UtcDuration adjustment)
{
    const std::int64_t adj_delta = adjustment.toUSec() - prev_adjustment;  // This is the P term
    prev_adjustment = adjustment.toUSec();

    utc_correction_usec_per_overflow_x16 += adjustment.isPositive() ? 1 : -1; // I
    utc_correction_usec_per_overflow_x16 += (adj_delta > 0) ? 1 : -1;         // P

    utc_correction_usec_per_overflow_x16 =
        uavcan::max(utc_correction_usec_per_overflow_x16, -MaxUtcSpeedCorrectionX16);
    utc_correction_usec_per_overflow_x16 =
        uavcan::min(utc_correction_usec_per_overflow_x16,  MaxUtcSpeedCorrectionX16);

    if (adjustment.getAbs().toMSec() > 9 || !utc_set)
    {
        const std::int64_t adj_usec = adjustment.toUSec();
        {
            CriticalSectionLocker locker;
            if ((adj_usec < 0) && std::uint64_t(-adj_usec) > time_utc)
            {
                time_utc = 1;
            }
            else
            {
                time_utc = std::uint64_t(std::int64_t(time_utc) + adj_usec);
            }
        }
        if (!utc_set)
        {
            utc_set = true;
            utc_correction_usec_per_overflow_x16 = 0;
        }
    }
}

} // namespace clock

SystemClock SystemClock::self;

SystemClock& SystemClock::instance()
{
    clock::init();
    return self;
}

}

/*
 * Timer interrupt handler
 */
extern "C"
{

void SysTick_Handler();

void SysTick_Handler()
{
    using namespace uavcan_lpc11c24::clock;
    if (initialized)
    {
        time_mono += USecPerOverflow;
        if (utc_set)
        {
            // Values below 16 are ignored
            time_utc += std::uint64_t(std::int32_t(USecPerOverflow) + (utc_correction_usec_per_overflow_x16 / 16));
        }
    }
    else
    {
        fail();
    }
}

}
