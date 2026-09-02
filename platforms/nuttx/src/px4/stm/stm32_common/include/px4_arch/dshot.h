/****************************************************************************
 *
 * Copyright (C) 2024 PX4 Development Team. All rights reserved.
 * Author: Igor Misic <igy1000mb@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>

#include <drivers/drv_pwm_output.h>
#include <stm32_dma.h>
#include <arch/board/board.h>

// A DSHOT bit is (ARR + 1) timer ticks of (PSC + 1) timer-clock cycles each, so the
// emitted bit period is ticks * prescaler / timer_clock and the high time is CCR / ticks
// of it. No single tick count divides every timer clock by every DSHOT rate, so both are
// chosen together here for the timer's own clock and the requested rate, and the two high
// times follow from the tick count that was picked. The search runs a couple of hundred
// cycles and its answer only changes when the rate does, so io_timer_get_dshot_timing()
// caches it per timer rather than repeating it in the transmit path.
typedef struct dshot_timing_t {
	uint32_t ticks;             // timer ticks in one DSHOT bit; ARR = ticks - 1
	uint32_t prescaler;         // timer-clock cycles in one tick; PSC = prescaler - 1
	uint32_t bit_0;             // CCR for a zero bit, 37.5 % of the bit
	uint32_t bit_1;             // CCR for a one bit, 75 % of the bit
	uint32_t capture_prescaler; // as prescaler, for the bidirectional capture clock
} dshot_timing_t;

// Wide enough that some tick count divides the common timer clocks exactly, and narrow
// enough that a bit still resolves to better than a tenth of itself.
#define DSHOT_MIN_BIT_TICKS 12u
#define DSHOT_MAX_BIT_TICKS 40u

// Ticks the bidirectional capture timer counts in one response bit.
#define DSHOT_CAPTURE_TICKS_PER_BIT 20u

static inline dshot_timing_t dshot_timing(uint32_t timer_clock, uint32_t dshot_rate)
{
	dshot_timing_t best = {DSHOT_MAX_BIT_TICKS, 1u, (3u * DSHOT_MAX_BIT_TICKS + 4u) / 8u, (3u * DSHOT_MAX_BIT_TICKS + 2u) / 4u, 1u};

	if (timer_clock == 0u || dshot_rate == 0u) {
		return best;
	}

	uint32_t best_rate_error = UINT32_MAX;
	uint32_t best_duty_error = UINT32_MAX;

	for (uint32_t ticks = DSHOT_MIN_BIT_TICKS; ticks <= DSHOT_MAX_BIT_TICKS; ticks++) {
		const uint32_t cycles_per_bit = dshot_rate * ticks;
		uint32_t prescaler = (timer_clock + cycles_per_bit / 2u) / cycles_per_bit;

		if (prescaler == 0u) {
			prescaler = 1u;
		}

		// The emitted rate is timer_clock / (ticks * prescaler); comparing the cycle
		// counts instead keeps this in integers, and the rounded prescaler puts the
		// product within half a bit of timer_clock, so 32 bits hold it.
		const uint32_t cycles = ticks * prescaler * dshot_rate;
		const uint32_t rate_error = cycles > timer_clock ? cycles - timer_clock : timer_clock - cycles;

		const uint32_t bit_1 = (3u * ticks + 2u) / 4u;   // round(0.75 * ticks)
		const uint32_t bit_0 = (3u * ticks + 4u) / 8u;   // round(0.375 * ticks)

		// Both high-time errors in eighths of a tick, so a tick count that lands the
		// duty cycles on the protocol breaks a tie between equally accurate rates.
		const uint32_t want_1 = 6u * ticks;
		const uint32_t want_0 = 3u * ticks;
		const uint32_t got_1 = 8u * bit_1;
		const uint32_t got_0 = 8u * bit_0;
		const uint32_t duty_error = (got_1 > want_1 ? got_1 - want_1 : want_1 - got_1)
					    + (got_0 > want_0 ? got_0 - want_0 : want_0 - got_0);

		if (rate_error < best_rate_error
		    || (rate_error == best_rate_error
			&& (duty_error < best_duty_error
			    || (duty_error == best_duty_error && ticks > best.ticks)))) {
			best_rate_error = rate_error;
			best_duty_error = duty_error;
			best.ticks = ticks;
			best.prescaler = prescaler;
			best.bit_0 = bit_0;
			best.bit_1 = bit_1;
		}
	}

	// The capture clock free-runs, so only its resolution matters and it is independent
	// of what the transmit side settled on: DSHOT_CAPTURE_TICKS_PER_BIT ticks per
	// response bit keeps a one-to-three bit run inside the interval window
	// convert_edge_intervals_to_bitstream() accepts.
	const uint32_t response_cycles = (dshot_rate * 5u / 4u) * DSHOT_CAPTURE_TICKS_PER_BIT;
	best.capture_prescaler = (timer_clock + response_cycles / 2u) / response_cycles;

	if (best.capture_prescaler == 0u) {
		best.capture_prescaler = 1u;
	}

	return best;
}

/* Configuration for each timer to setup DShot. Some timers have only one while others have two choices for the stream.
 *
 * DMAMAP_TIM1_UP	- DMA2, Channel6, Stream5
 * DMAMAP_TIM2_UP_1	- DMA1, Channel3, Stream1
 * DMAMAP_TIM2_UP_2	- DMA1, Channel3, Stream7
 * DMAMAP_TIM3_UP	- DMA1, Channel5, Stream2
 * DMAMAP_TIM4_UP	- DMA1, Channel2, Stream6
 * DMAMAP_TIM5_UP_1	- DMA1, Channel6, Stream0
 * DMAMAP_TIM5_UP_2	- DMA1, Channel6, Stream6
 * DMAMAP_TIM6_UP	- DMA1, Channel7, Stream1
 * DMAMAP_TIM7_UP_1	- DMA1, Channel1, Stream2
 * DMAMAP_TIM7_UP_2	- DMA1, Channel1, Stream4
 * DMAMAP_TIM8_UP	- DMA2, Channel7, Stream1
 */

/* The structure which contains configuration for DShot
 */
typedef struct dshot_conf_t {
	uint32_t			dma_base;
	uint32_t			dma_map_up;
	uint32_t			dma_map_ch[4];
} dshot_conf_t;
