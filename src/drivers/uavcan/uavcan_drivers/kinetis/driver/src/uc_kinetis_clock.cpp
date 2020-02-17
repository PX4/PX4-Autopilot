/*
 * Copyright (C) 2014, 2018 Pavel Kirienko <pavel.kirienko@gmail.com>
 * Kinetis Port Author David Sidrane <david_s5@nscdg.com>
 */

#include <uavcan_kinetis/clock.hpp>
#include <uavcan_kinetis/thread.hpp>
#include "internal.hpp"

#if UAVCAN_KINETIS_TIMER_NUMBER
# include <cassert>
# include <math.h>

/*
 * Timer instance
 * todo:Consider using Lifetime Timer support
 */
# define TIMX_IRQHandler         UAVCAN_KINETIS_GLUE3(PIT, UAVCAN_KINETIS_TIMER_NUMBER, _IRQHandler)
# define TIMX                    (KINETIS_PIT_BASE + (UAVCAN_KINETIS_TIMER_NUMBER << 4))
# define TMR_REG(o)              (TIMX + (o))
# define TIMX_INPUT_CLOCK        BOARD_BUS_FREQ
# define TIMX_INTERRUPT_FREQ     16
# define TIMX_IRQn               UAVCAN_KINETIS_GLUE2(KINETIS_IRQ_PITCH, UAVCAN_KINETIS_TIMER_NUMBER)

# if UAVCAN_KINETIS_TIMER_NUMBER >= 0 && UAVCAN_KINETIS_TIMER_NUMBER <= 3
#  define KINETIS_PIT_LDVAL_OFFSET KINETIS_PIT_LDVAL0_OFFSET
#  define KINETIS_PIT_CVAL_OFFSET  KINETIS_PIT_CVAL0_OFFSET
#  define KINETIS_PIT_TCTRL_OFFSET KINETIS_PIT_TCTRL0_OFFSET
#  define KINETIS_PIT_TFLG_OFFSET  KINETIS_PIT_TFLG0_OFFSET
# else
#  error "This UAVCAN_KINETIS_TIMER_NUMBER is not supported yet"
# endif

extern "C" UAVCAN_KINETIS_IRQ_HANDLER(TIMX_IRQHandler);

namespace uavcan_kinetis
{
namespace clock
{
namespace
{

const uavcan::uint32_t CountsPerPeriod = (TIMX_INPUT_CLOCK / TIMX_INTERRUPT_FREQ);
const uavcan::uint32_t CountsPerUs = (TIMX_INPUT_CLOCK / 1000000);
const uavcan::uint32_t USecPerOverflow = (1000000 / TIMX_INTERRUPT_FREQ);

Mutex mutex;

bool initialized = false;

bool utc_set = false;
bool utc_locked = false;
uavcan::uint32_t utc_jump_cnt = 0;
UtcSyncParams utc_sync_params;
float utc_prev_adj = 0;
float utc_rel_rate_ppm = 0;
float utc_rel_rate_error_integral = 0;
uavcan::int32_t utc_accumulated_correction_nsec = 0;
uavcan::int32_t utc_correction_nsec_per_overflow = 0;
uavcan::MonotonicTime prev_utc_adj_at;

uavcan::uint64_t time_mono = 0;
uavcan::uint64_t time_utc = 0;

}

void init()
{
	CriticalSectionLocker lock;

	if (initialized) {
		return;
	}

	initialized = true;

	// Attach IRQ
	irq_attach(TIMX_IRQn, &TIMX_IRQHandler, NULL);

	// Power-on Clock
	modifyreg32(KINETIS_SIM_SCGC6, 0, SIM_SCGC6_PIT);

	// Enable module
	putreg32(0, KINETIS_PIT_MCR);

	// Start the timer

	putreg32(CountsPerPeriod - 1, TMR_REG(KINETIS_PIT_LDVAL_OFFSET));
	putreg32(PIT_TCTRL_TEN | PIT_TCTRL_TIE, TMR_REG(KINETIS_PIT_TCTRL_OFFSET));     // Start

	// Prioritize and Enable  IRQ

#if 0
	// This has to be off or uses the default priority
	// Without the ability to point the vector
	// Directly to this ISR this will reenter the
	// exception_common and cause the interrupt
	// Stack pointer to be reset
	up_prioritize_irq(TIMX_IRQn, NVIC_SYSH_HIGH_PRIORITY);
#endif
	up_enable_irq(TIMX_IRQn);
}

void setUtc(uavcan::UtcTime time)
{
	MutexLocker mlocker(mutex);
	UAVCAN_ASSERT(initialized);

	{
		CriticalSectionLocker locker;
		time_utc = time.toUSec();
	}

	utc_set = true;
	utc_locked = false;
	utc_jump_cnt++;
	utc_prev_adj = 0;
	utc_rel_rate_ppm = 0;
}

static uavcan::uint64_t sampleUtcFromCriticalSection()
{
	UAVCAN_ASSERT(initialized);
	UAVCAN_ASSERT(getreg32(TMR_REG(KINETIS_PIT_TCTRL_OFFSET)) & PIT_TCTRL_TIE);

	volatile uavcan::uint64_t time = time_utc;
	volatile uavcan::uint32_t cnt = CountsPerPeriod - getreg32(TMR_REG(KINETIS_PIT_CVAL_OFFSET));

	if (getreg32(TMR_REG(KINETIS_PIT_TFLG_OFFSET)) & PIT_TFLG_TIF) {
		cnt = CountsPerPeriod - getreg32(TMR_REG(KINETIS_PIT_CVAL_OFFSET));
		const uavcan::int32_t add = uavcan::int32_t(USecPerOverflow) +
					    (utc_accumulated_correction_nsec + utc_correction_nsec_per_overflow) / 1000;
		time = uavcan::uint64_t(uavcan::int64_t(time) + add);
	}

	return time + (cnt / CountsPerUs);
}

uavcan::uint64_t getUtcUSecFromCanInterrupt()
{
	return utc_set ? sampleUtcFromCriticalSection() : 0;
}

uavcan::MonotonicTime getMonotonic()
{
	uavcan::uint64_t usec = 0;
	// Scope Critical section
	{
		CriticalSectionLocker locker;

		volatile uavcan::uint64_t time = time_mono;
		volatile uavcan::uint32_t cnt = CountsPerPeriod - getreg32(TMR_REG(KINETIS_PIT_CVAL_OFFSET));

		if (getreg32(TMR_REG(KINETIS_PIT_TFLG_OFFSET)) & PIT_TFLG_TIF) {
			cnt = CountsPerPeriod - getreg32(TMR_REG(KINETIS_PIT_CVAL_OFFSET));
			time += USecPerOverflow;
		}

		usec = time + (cnt / CountsPerUs);

	}     // End Scope Critical section

	return uavcan::MonotonicTime::fromUSec(usec);
}


uavcan::UtcTime getUtc()
{
	if (utc_set) {
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
	const float dt = float((ts - prev_utc_adj_at).toUSec()) / 1e6F;
	prev_utc_adj_at = ts;
	const float adj_usec = float(adjustment.toUSec());

	/*
	 * Target relative rate in PPM
	 * Positive to go faster
	 */
	const float target_rel_rate_ppm = adj_usec * utc_sync_params.offset_p;

	/*
	 * Current relative rate in PPM
	 * Positive if the local clock is faster
	 */
	const float new_rel_rate_ppm = (utc_prev_adj - adj_usec) / dt;     // rate error in [usec/sec], which is PPM
	utc_prev_adj = adj_usec;
	utc_rel_rate_ppm = lowpass(utc_rel_rate_ppm, new_rel_rate_ppm, utc_sync_params.rate_error_corner_freq, dt);

	const float rel_rate_error = target_rel_rate_ppm - utc_rel_rate_ppm;

	if (dt > 10) {
		utc_rel_rate_error_integral = 0;

	} else {
		utc_rel_rate_error_integral += rel_rate_error * dt * utc_sync_params.rate_i;
		utc_rel_rate_error_integral =
			uavcan::max(utc_rel_rate_error_integral, -utc_sync_params.max_rate_correction_ppm);
		utc_rel_rate_error_integral =
			uavcan::min(utc_rel_rate_error_integral, utc_sync_params.max_rate_correction_ppm);
	}

	/*
	 * Rate controller
	 */
	float total_rate_correction_ppm = rel_rate_error + utc_rel_rate_error_integral;
	total_rate_correction_ppm = uavcan::max(total_rate_correction_ppm, -utc_sync_params.max_rate_correction_ppm);
	total_rate_correction_ppm = uavcan::min(total_rate_correction_ppm, utc_sync_params.max_rate_correction_ppm);

	utc_correction_nsec_per_overflow = uavcan::int32_t((USecPerOverflow * 1000) * (total_rate_correction_ppm / 1e6F));

//    syslog("$ adj=%f   rel_rate=%f   rel_rate_eint=%f   tgt_rel_rate=%f   ppm=%f\n",
//              adj_usec, utc_rel_rate_ppm, utc_rel_rate_error_integral, target_rel_rate_ppm,
// total_rate_correction_ppm);
}

void adjustUtc(uavcan::UtcDuration adjustment)
{
	MutexLocker mlocker(mutex);
	UAVCAN_ASSERT(initialized);

	if (adjustment.getAbs() > utc_sync_params.min_jump || !utc_set) {
		const uavcan::int64_t adj_usec = adjustment.toUSec();

		{
			CriticalSectionLocker locker;

			if ((adj_usec < 0) && uavcan::uint64_t(-adj_usec) > time_utc) {
				time_utc = 1;

			} else {
				time_utc = uavcan::uint64_t(uavcan::int64_t(time_utc) + adj_usec);
			}
		}

		utc_set = true;
		utc_locked = false;
		utc_jump_cnt++;
		utc_prev_adj = 0;
		utc_rel_rate_ppm = 0;

	} else {
		updateRatePID(adjustment);

		if (!utc_locked) {
			utc_locked =
				(std::abs(utc_rel_rate_ppm) < utc_sync_params.lock_thres_rate_ppm) &&
				(std::abs(utc_prev_adj) < float(utc_sync_params.lock_thres_offset.toUSec()));
		}
	}
}

float getUtcRateCorrectionPPM()
{
	MutexLocker mlocker(mutex);
	const float rate_correction_mult = float(utc_correction_nsec_per_overflow) / float(USecPerOverflow * 1000);
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

void setUtcSyncParams(const UtcSyncParams &params)
{
	MutexLocker mlocker(mutex);
	// Add some sanity check
	utc_sync_params = params;
}

} // namespace clock

SystemClock &SystemClock::instance()
{
	static union SystemClockStorage {
		uavcan::uint8_t buffer[sizeof(SystemClock)];
		long long _aligner_1;
		long double _aligner_2;
	} storage;

	SystemClock *const ptr = reinterpret_cast<SystemClock *>(storage.buffer);

	if (!clock::initialized) {
		MutexLocker mlocker(clock::mutex);
		clock::init();
		new (ptr)SystemClock();
	}

	return *ptr;
}

} // namespace uavcan_kinetis


/**
 * Timer interrupt handler
 */

extern "C"
UAVCAN_KINETIS_IRQ_HANDLER(TIMX_IRQHandler)
{
	putreg32(PIT_TFLG_TIF, TMR_REG(KINETIS_PIT_TFLG_OFFSET));

	using namespace uavcan_kinetis::clock;
	UAVCAN_ASSERT(initialized);

	time_mono += USecPerOverflow;

	if (utc_set) {
		time_utc += USecPerOverflow;
		utc_accumulated_correction_nsec += utc_correction_nsec_per_overflow;

		if (std::abs(utc_accumulated_correction_nsec) >= 1000) {
			time_utc = uavcan::uint64_t(uavcan::int64_t(time_utc) + utc_accumulated_correction_nsec / 1000);
			utc_accumulated_correction_nsec %= 1000;
		}

		// Correction decay - 1 nsec per 65536 usec
		if (utc_correction_nsec_per_overflow > 0) {
			utc_correction_nsec_per_overflow--;

		} else if (utc_correction_nsec_per_overflow < 0) {
			utc_correction_nsec_per_overflow++;

		} else {
			;             // Zero
		}
	}

	return 0;
}

#endif
