/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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


#include "../../../stm32_common/include/px4_arch/hw_description.h"

static inline constexpr void getTimerChannelDMAMap(Timer::Timer timer, const DMA &dma, uint32_t *dma_map_ch)
{
	switch (timer) {
	case Timer::Timer1:
		if (dma.index == DMA::Index1) {
			dma_map_ch[0] = DMAMAP_DMA12_TIM1CH1_0;
			dma_map_ch[1] = DMAMAP_DMA12_TIM1CH2_0;
			dma_map_ch[2] = DMAMAP_DMA12_TIM1CH3_0;
			dma_map_ch[3] = DMAMAP_DMA12_TIM1CH4_0;

		} else {
			dma_map_ch[0] = DMAMAP_DMA12_TIM1CH1_1;
			dma_map_ch[1] = DMAMAP_DMA12_TIM1CH2_1;
			dma_map_ch[2] = DMAMAP_DMA12_TIM1CH3_1;
			dma_map_ch[3] = DMAMAP_DMA12_TIM1CH4_1;
		}

		break;

	case Timer::Timer2:
		if (dma.index == DMA::Index1) {
			dma_map_ch[0] = DMAMAP_DMA12_TIM2CH1_0;
			dma_map_ch[1] = DMAMAP_DMA12_TIM2CH2_0;
			dma_map_ch[2] = DMAMAP_DMA12_TIM2CH3_0;
			dma_map_ch[3] = DMAMAP_DMA12_TIM2CH4_0;

		} else {
			dma_map_ch[0] = DMAMAP_DMA12_TIM2CH1_1;
			dma_map_ch[1] = DMAMAP_DMA12_TIM2CH2_1;
			dma_map_ch[2] = DMAMAP_DMA12_TIM2CH3_1;
			dma_map_ch[3] = DMAMAP_DMA12_TIM2CH4_1;
		}

		break;

	case Timer::Timer3:
		if (dma.index == DMA::Index1) {
			dma_map_ch[0] = DMAMAP_DMA12_TIM3CH1_0;
			dma_map_ch[1] = DMAMAP_DMA12_TIM3CH2_0;
			dma_map_ch[2] = DMAMAP_DMA12_TIM3CH3_0;
			dma_map_ch[3] = DMAMAP_DMA12_TIM3CH4_0;

		} else {
			dma_map_ch[0] = DMAMAP_DMA12_TIM3CH1_1;
			dma_map_ch[1] = DMAMAP_DMA12_TIM3CH2_1;
			dma_map_ch[2] = DMAMAP_DMA12_TIM3CH3_1;
			dma_map_ch[3] = DMAMAP_DMA12_TIM3CH4_1;
		}

		break;

	case Timer::Timer4:
		if (dma.index == DMA::Index1) {
			dma_map_ch[0] = DMAMAP_DMA12_TIM4CH1_0;
			dma_map_ch[1] = DMAMAP_DMA12_TIM4CH2_0;
			dma_map_ch[2] = DMAMAP_DMA12_TIM4CH3_0;
			dma_map_ch[3] = 0;


		} else {
			dma_map_ch[0] = DMAMAP_DMA12_TIM4CH1_1;
			dma_map_ch[1] = DMAMAP_DMA12_TIM4CH2_1;
			dma_map_ch[2] = DMAMAP_DMA12_TIM4CH3_1;
			dma_map_ch[3] = 0;
		}

		break;

	case Timer::Timer5:
		if (dma.index == DMA::Index1) {
			dma_map_ch[0] = DMAMAP_DMA12_TIM5CH1_0;
			dma_map_ch[1] = DMAMAP_DMA12_TIM5CH2_0;
			dma_map_ch[2] = DMAMAP_DMA12_TIM5CH3_0;
			dma_map_ch[3] = DMAMAP_DMA12_TIM5CH4_0;

		} else {
			dma_map_ch[0] = DMAMAP_DMA12_TIM5CH1_1;
			dma_map_ch[1] = DMAMAP_DMA12_TIM5CH2_1;
			dma_map_ch[2] = DMAMAP_DMA12_TIM5CH3_1;
			dma_map_ch[3] = DMAMAP_DMA12_TIM5CH4_1;
		}

		break;

	case Timer::Timer6:
		// No channels available
		break;

	case Timer::Timer7:
		// No channels available
		break;

	case Timer::Timer8:
		if (dma.index == DMA::Index1) {
			dma_map_ch[0] = DMAMAP_DMA12_TIM8CH1_0;
			dma_map_ch[1] = DMAMAP_DMA12_TIM8CH2_0;
			dma_map_ch[2] = DMAMAP_DMA12_TIM8CH3_0;
			dma_map_ch[3] = DMAMAP_DMA12_TIM8CH4_0;

		} else {
			dma_map_ch[0] = DMAMAP_DMA12_TIM8CH1_1;
			dma_map_ch[1] = DMAMAP_DMA12_TIM8CH2_1;
			dma_map_ch[2] = DMAMAP_DMA12_TIM8CH3_1;
			dma_map_ch[3] = DMAMAP_DMA12_TIM8CH4_1;
		}

		break;

	case Timer::Timer9:
		// Non-existent
		break;

	case Timer::Timer10:
		// Non-existent
		break;

	case Timer::Timer11:
		// Non-existent
		break;

	case Timer::Timer12:
		// Non-existent
		break;

	case Timer::Timer13:
		// Non-existent
		break;

	case Timer::Timer14:
		// Non-existent
		break;

	case Timer::Timer15:
		if (dma.index == DMA::Index1) {
			dma_map_ch[0] = DMAMAP_DMA12_TIM15CH1_0;

		} else {
			dma_map_ch[0] = DMAMAP_DMA12_TIM15CH1_1;
		}

		break;

	case Timer::Timer16:
		if (dma.index == DMA::Index1) {
			dma_map_ch[0] = DMAMAP_DMA12_TIM16CH1_0;

		} else {
			dma_map_ch[0] = DMAMAP_DMA12_TIM16CH1_1;
		}

		break;

	case Timer::Timer17:
		if (dma.index == DMA::Index1) {
			dma_map_ch[0] = DMAMAP_DMA12_TIM17CH1_0;

		} else {
			dma_map_ch[0] = DMAMAP_DMA12_TIM17CH1_1;
		}

		break;
	}
}

static inline constexpr uint32_t getTimerUpdateDMAMap(Timer::Timer timer, const DMA &dma)
{
	uint32_t dma_map_up = 0;

	switch (timer) {
	case Timer::Timer1:
		dma_map_up = (dma.index == DMA::Index1) ? DMAMAP_DMA12_TIM1UP_0 : DMAMAP_DMA12_TIM1UP_1;
		break;

	case Timer::Timer2:
		dma_map_up = (dma.index == DMA::Index1) ? DMAMAP_DMA12_TIM2UP_0 : DMAMAP_DMA12_TIM2UP_1;

		break;

	case Timer::Timer3:
		dma_map_up = (dma.index == DMA::Index1) ? DMAMAP_DMA12_TIM3UP_0 : DMAMAP_DMA12_TIM3UP_1;

		break;

	case Timer::Timer4:
		dma_map_up = (dma.index == DMA::Index1) ? DMAMAP_DMA12_TIM4UP_0 : DMAMAP_DMA12_TIM4UP_1;

		break;

	case Timer::Timer5:
		dma_map_up = (dma.index == DMA::Index1) ? DMAMAP_DMA12_TIM5UP_0 : DMAMAP_DMA12_TIM5UP_1;

		break;

	case Timer::Timer6:
		dma_map_up = (dma.index == DMA::Index1) ? DMAMAP_DMA12_TIM6UP_0 : DMAMAP_DMA12_TIM6UP_1;

		break;

	case Timer::Timer7:
		dma_map_up = (dma.index == DMA::Index1) ? DMAMAP_DMA12_TIM7UP_0 : DMAMAP_DMA12_TIM7UP_1;

		break;

	case Timer::Timer8:
		dma_map_up = (dma.index == DMA::Index1) ? DMAMAP_DMA12_TIM8UP_0 : DMAMAP_DMA12_TIM8UP_1;

		break;

	case Timer::Timer9:
	case Timer::Timer10:
	case Timer::Timer11:
	case Timer::Timer12:
	case Timer::Timer13:
	case Timer::Timer14:
	case Timer::Timer15:
	case Timer::Timer16:
	case Timer::Timer17:
		break;
	}

	constexpr_assert(dma_map_up != 0, "Invalid DMA config for given timer");
	return dma_map_up;
}
