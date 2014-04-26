/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <crdr_chibios/sys/sys.h>
#include <cassert>
#include <cmath>
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

const uavcan::uint32_t USecPerOverflow = 65536;

Mutex mutex;

bool initialized = false;

bool utc_set = false;
bool utc_locked = false;
uavcan::uint32_t utc_jump_cnt = 0;
UtcSyncParams utc_sync_params;
float utc_prev_adj = 0;
float utc_inv_rate_error_ppm = 0;
float utc_integrated_error = 0;
uavcan::int32_t utc_accumulated_correction_nsec = 0;
uavcan::int32_t utc_correction_nsec_per_overflow = 0;
uavcan::MonotonicTime prev_utc_adj_at;

uavcan::uint64_t time_mono = 0;
uavcan::uint64_t time_utc = 0;

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

static uavcan::uint64_t sampleUtcFromCriticalSection()
{
    assert(initialized);
    assert(TIMX->DIER & TIM_DIER_UIE);

    volatile uavcan::uint64_t time = time_utc;
    volatile uavcan::uint32_t cnt = TIMX->CNT;

    if (TIMX->SR & TIM_SR_UIF)
    {
        cnt = TIMX->CNT;
        time += USecPerOverflow + (utc_accumulated_correction_nsec + utc_correction_nsec_per_overflow) / 1000;
    }
    return time + cnt;
}

uavcan::uint64_t getUtcUSecFromCanInterrupt()
{
    return utc_set ? sampleUtcFromCriticalSection() : 0;
}

uavcan::MonotonicTime getMonotonic()
{
    uavcan::uint64_t usec = 0;
    {
        CriticalSectionLocker locker;

        volatile uavcan::uint64_t time = time_mono;
        volatile uavcan::uint32_t cnt = TIMX->CNT;
        if (TIMX->SR & TIM_SR_UIF)
        {
            cnt = TIMX->CNT;
            time += USecPerOverflow;
        }
        usec = time + cnt;

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
            usec = sampleUtcFromCriticalSection();
        }
        return uavcan::UtcTime::fromUSec(usec);
    }
    return uavcan::UtcTime();
}

static float lowpass(float xold, float xnew, float corner, float dt)
{
    const float tau = 1.F / corner;
    return (dt * xnew + tau * xold) / (dt + tau);
}

static void updateRatePID(uavcan::UtcDuration adjustment)
{
    const uavcan::MonotonicTime ts = getMonotonic();
    const float dt = (ts - prev_utc_adj_at).toUSec() / 1e6F;
    prev_utc_adj_at = ts;

    /*
     * Rate error with lowpass filter
     */
    const float adj_usec = adjustment.toUSec();
    const float new_inverted_rate_error_ppm = (adj_usec - utc_prev_adj) / dt;// rate error in [usec/sec], which is PPM
    utc_prev_adj = adj_usec;
    utc_inv_rate_error_ppm =
        lowpass(utc_inv_rate_error_ppm, new_inverted_rate_error_ppm, utc_sync_params.rate_error_corner_freq, dt);

    /*
     * Long term offset error
     */
    if (dt < 10)
    {
        const float i = ((adj_usec > 0) == (utc_integrated_error > 0)) ? utc_sync_params.i_fwd : utc_sync_params.i_rev;
        utc_integrated_error += adj_usec * dt * i;
        utc_integrated_error = std::max(utc_integrated_error, -utc_sync_params.max_rate_correction_ppm);
        utc_integrated_error = std::min(utc_integrated_error, utc_sync_params.max_rate_correction_ppm);
    }
    else
    {
        utc_integrated_error = 0;
    }

    /*
     * Compute final correction
     */
    float rate_correction_ppm = utc_inv_rate_error_ppm + utc_integrated_error + adj_usec * utc_sync_params.p;
    rate_correction_ppm = std::max(rate_correction_ppm, -utc_sync_params.max_rate_correction_ppm);
    rate_correction_ppm = std::min(rate_correction_ppm, utc_sync_params.max_rate_correction_ppm);

    utc_correction_nsec_per_overflow = (USecPerOverflow * 1000) * (rate_correction_ppm / 1e6F);

    lowsyslog("$ adj=%f   rate_err=%f   int=%f   ppm=%f\n",
              adj_usec, utc_inv_rate_error_ppm, utc_integrated_error, rate_correction_ppm);
}

void adjustUtc(uavcan::UtcDuration adjustment)
{
    MutexLocker mlocker(mutex);
    assert(initialized);

    if (adjustment.getAbs() > utc_sync_params.min_jump || !utc_set)
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

        utc_set = true;
        utc_locked = false;
        utc_jump_cnt++;
        utc_prev_adj = 0;
        utc_inv_rate_error_ppm = 0;
    }
    else
    {
        updateRatePID(adjustment);

        if (!utc_locked)
        {
            utc_locked =
                (std::abs(utc_inv_rate_error_ppm) < utc_sync_params.lock_thres_rate_ppm) &&
                (std::abs(utc_prev_adj) < utc_sync_params.lock_thres_offset.toUSec());
        }
    }
}

float getUtcRateCorrectionPPM()
{
    MutexLocker mlocker(mutex);
    const float rate_correction_mult = utc_correction_nsec_per_overflow / float(USecPerOverflow * 1000);
    return 1e6F * rate_correction_mult;
}

uavcan::uint32_t getUtcJumpCount()
{
    MutexLocker mlocker(mutex);
    return utc_jump_cnt;
}

bool isUtcLocked()
{
    MutexLocker mlocker(mutex);
    return utc_locked;
}

UtcSyncParams getUtcSyncParams()
{
    MutexLocker mlocker(mutex);
    return utc_sync_params;
}

void setUtcSyncParams(const UtcSyncParams& params)
{
    MutexLocker mlocker(mutex);
    // Add some sanity check
    utc_sync_params = params;
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
        time_utc += USecPerOverflow;
        utc_accumulated_correction_nsec += utc_correction_nsec_per_overflow;
        if (std::abs(utc_accumulated_correction_nsec) >= 1000)
        {
            time_utc += utc_accumulated_correction_nsec / 1000;
            utc_accumulated_correction_nsec %= 1000;
        }

        // Correction decay - 1 nsec per 65536 usec
        if (utc_correction_nsec_per_overflow > 0)
        {
            utc_correction_nsec_per_overflow--;
        }
        else if (utc_correction_nsec_per_overflow < 0)
        {
            utc_correction_nsec_per_overflow++;
        }
        else
        {
            ; // Zero
        }
    }

    UAVCAN_STM32_IRQ_EPILOGUE();
}
