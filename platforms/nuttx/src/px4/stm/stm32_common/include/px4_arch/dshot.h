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

#include <drivers/drv_pwm_output.h>
#include <stm32_dma.h>
#include <arch/board/board.h>

// Number of timer ticks per DSHOT bit (the timer's ARR). This is the per-bit
// resolution, NOT a function of the DSHOT rate: the timer is configured with
// ARR = WIDTH and PSC = floor(TIM_CLK / DSHOT_FREQ) / WIDTH - 1, so the prescaler
// alone scales the rate while ARR stays fixed. Keeping ARR fixed across rates is
// what lets the MOTOR_PWM_BIT_0/1 high-time constants hold their duty cycle at every
// frequency. The width is derived from a fixed reference rate (DSHOT600) so that
// floor(TIM_CLK / 600000) divides evenly, keeping the prescaler integer.
//
// This is computed per timer from that timer's own clock rather than from a single
// board-wide define, so boards whose timers run on different clocks (e.g. APB1 vs
// APB2) each get the correct width without any board-specific configuration.
static inline uint32_t dshot_motor_pwm_bit_width(uint32_t timer_clock)
{
	const uint32_t ticks = timer_clock / 600000u;

	// Prefer the widest period that divides evenly (most timing resolution); fall
	// back to 20 when none does, matching the legacy default.
	for (uint32_t width = 20u; width >= 18u; --width) {
		if (ticks % width == 0u) {
			return width;
		}
	}

	return 20u;
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
