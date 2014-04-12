/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <uavcan_stm32/clock.hpp>
#include <uavcan_stm32/thread.hpp>
#include "internal.hpp"

/*
 * Timer instance
 */
#define TIMX                    UAVCAN_STM32_GLUE2(TIM, UAVCAN_STM32_TIMER_NUMBER)
#define TIMX_IRQn               UAVCAN_STM32_GLUE3(TIM, UAVCAN_STM32_TIMER_NUMBER, _IRQn)
#define TIMX_IRQHandler         UAVCAN_STM32_GLUE3(TIM, UAVCAN_STM32_TIMER_NUMBER, _IRQHandler)

#if UAVCAN_STM32_TIMER_NUMBER >= 2 && UAVCAN_STM32_TIMER_NUMBER <= 7
# define TIMX_RCC_ENR           RCC->APB1ENR
# define TIMX_RCC_RSTR          RCC->APB1RSTR
# define TIMX_RCC_ENR_MASK      UAVCAN_STM32_GLUE3(RCC_APB1ENR_TIM,  UAVCAN_STM32_TIMER_NUMBER, EN)
# define TIMX_RCC_RSTR_MASK     UAVCAN_STM32_GLUE3(RCC_APB1RSTR_TIM, UAVCAN_STM32_TIMER_NUMBER, RST)
# define TIMX_INPUT_CLOCK       STM32_TIMCLK1
#else
# error "This UAVCAN_STM32_TIMER_NUMBER is not supported yet"
#endif

namespace uavcan_stm32
{
namespace clock
{
namespace
{

Mutex mutex;

bool initialized = false;

bool utc_set = false;

uavcan::uint32_t utc_jump_cnt = 0;
uavcan::int32_t utc_correction_usec_per_overflow_x16 = 0;
uavcan::int64_t prev_adjustment = 0;

uavcan::uint64_t time_mono = 0;
uavcan::uint64_t time_utc = 0;

const uavcan::uint32_t USecPerOverflow = 65536;
const uavcan::int32_t MaxUtcSpeedCorrectionX16 = 100 * 16;

}

void init()
{
    CriticalSectionLocker lock;
    if (initialized)
    {
        return;
    }
    initialized = true;

    // Power-on and reset
    TIMX_RCC_ENR |= TIMX_RCC_ENR_MASK;
    TIMX_RCC_RSTR |=  TIMX_RCC_RSTR_MASK;
    TIMX_RCC_RSTR &= ~TIMX_RCC_RSTR_MASK;

    // Enable IRQ
    nvicEnableVector(TIMX_IRQn,  UAVCAN_STM32_IRQ_PRIORITY_MASK);

#if (TIMX_INPUT_CLOCK % 1000000) != 0
# error "No way, timer clock must be divisible to 1e6. FIXME!"
#endif

    // Start the timer
    TIMX->ARR  = 0xFFFF;
    TIMX->PSC  = (TIMX_INPUT_CLOCK / 1000000) - 1;  // 1 tick == 1 microsecond
    TIMX->CR1  = TIM_CR1_URS;
    TIMX->SR   = 0;
    TIMX->EGR  = TIM_EGR_UG;     // Reload immediately
    TIMX->DIER = TIM_DIER_UIE;
    TIMX->CR1  = TIM_CR1_CEN;    // Start
}

/**
 * Callable from any context
 */
static uavcan::uint64_t sampleFromCriticalSection(const volatile uavcan::uint64_t* const value)
{
    assert(initialized);
    assert(TIMX->DIER & TIM_DIER_UIE);

    volatile uavcan::uint64_t time = *value;
    volatile uavcan::uint32_t cnt = TIMX->CNT;

    if (TIMX->SR & TIM_SR_UIF)
    {
        /*
         * The timer has overflowed either before or after CNT sample was obtained.
         * We need to sample it once more to be sure that the obtained
         * counter value has wrapped over zero.
         */
        cnt = TIMX->CNT;
        /*
         * The timer interrupt was set, but not handled yet.
         * Thus we need to adjust the tick counter manually.
         */
        time += USecPerOverflow;
    }

    return time + cnt;
}

uavcan::uint64_t getUtcUSecFromCanInterrupt()
{
    return utc_set ? sampleFromCriticalSection(&time_utc) : 0;
}

uavcan::MonotonicTime getMonotonic()
{
    uavcan::uint64_t usec = 0;
    {
        CriticalSectionLocker locker;
        usec = sampleFromCriticalSection(&time_mono);
#if !NDEBUG
        static uavcan::uint64_t prev_usec = 0;  // Self-test
        assert(prev_usec <= usec);
        prev_usec = usec;
#endif
    }
    return uavcan::MonotonicTime::fromUSec(usec);
}

uavcan::UtcTime getUtc()
{
    if (utc_set)
    {
        uavcan::uint64_t usec = 0;
        {
            CriticalSectionLocker locker;
            usec = sampleFromCriticalSection(&time_utc);
        }
        return uavcan::UtcTime::fromUSec(usec);
    }
    return uavcan::UtcTime();
}

void adjustUtc(uavcan::UtcDuration adjustment)
{
    MutexLocker mlocker(mutex);

    assert(initialized);

    /*
     * Naive speed adjustment - discrete PI controller.
     * TODO: More reliable clock speed adjustment algorithm
     */
    const uavcan::int64_t adj_delta = adjustment.toUSec() - prev_adjustment;  // This is the P term
    prev_adjustment = adjustment.toUSec();

    utc_correction_usec_per_overflow_x16 += adjustment.isPositive() ? 1 : -1; // I
    utc_correction_usec_per_overflow_x16 += (adj_delta > 0) ? 1 : -1;         // P

    utc_correction_usec_per_overflow_x16 = std::max(utc_correction_usec_per_overflow_x16, -MaxUtcSpeedCorrectionX16);
    utc_correction_usec_per_overflow_x16 = std::min(utc_correction_usec_per_overflow_x16,  MaxUtcSpeedCorrectionX16);

    /*
     * Clock value adjustment
     * For small adjustments (less than 3 msec) we will rely only on speed change
     */
    if (adjustment.getAbs().toMSec() > 2 || !utc_set)
    {
        const uavcan::int64_t adj_usec = adjustment.toUSec();

        {
            CriticalSectionLocker locker;
            if ((adj_usec < 0) && uavcan::uint64_t(-adj_usec) > time_utc)
            {
                time_utc = 1;
            }
            else
            {
                time_utc += adj_usec;
            }
        }

        if (utc_set)
        {
            utc_jump_cnt++;
        }
        else
        {
            utc_set = true;
            utc_correction_usec_per_overflow_x16 = 0;
        }
    }
}

uavcan::int32_t getUtcSpeedCorrectionPPM()
{
    MutexLocker mlocker(mutex);
    return uavcan::int64_t((utc_correction_usec_per_overflow_x16 * 1000000) / 16) / USecPerOverflow;
}

uavcan::uint32_t getUtcAjdustmentJumpCount()
{
    MutexLocker mlocker(mutex);
    return utc_jump_cnt;
}

uavcan::UtcDuration getPrevUtcAdjustment()
{
    MutexLocker mlocker(mutex);
    return uavcan::UtcDuration::fromUSec(prev_adjustment);
}

} // namespace clock

SystemClock& SystemClock::instance()
{
    MutexLocker mlocker(clock::mutex);

    static union SystemClockStorage
    {
        uavcan::uint8_t buffer[sizeof(SystemClock)];
        long long _aligner_1;
        long double _aligner_2;
    } storage;
    SystemClock* const ptr = reinterpret_cast<SystemClock*>(storage.buffer);

    if (!clock::initialized)
    {
        clock::init();
        new (ptr) SystemClock();
    }
    return *ptr;
}

} // namespace uavcan_stm32


/**
 * Timer interrupt handler
 */
extern "C"
UAVCAN_STM32_IRQ_HANDLER(TIMX_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();

    TIMX->SR = 0;

    using namespace uavcan_stm32::clock;
    assert(initialized);

    time_mono += USecPerOverflow;
    if (utc_set)
    {
        // Values below 16 are ignored
        time_utc += USecPerOverflow + (utc_correction_usec_per_overflow_x16 / 16);
    }

    UAVCAN_STM32_IRQ_EPILOGUE();
}
