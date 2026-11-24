/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform/gpio.h>

#include <arm_internal.h>
#include <stm32_gpio.h>
#include <stm32_rcc.h>
#include <hardware/stm32_tim.h>

#include <arch/board/board.h>

#define TIM4_REG(offset) (*(volatile uint32_t *)(STM32_TIM4_BASE + (offset)))

void board_clock_outputs_initialize(void)
{
	/* PA8/MCO1 is left unused: the STM32G431 OSD co-processor runs from its
	 * own crystal (X2), and R80, which would route MCO1 to it, is not fitted.
	 */

	/* ICM-42688-P CLKIN: 240 MHz / (15 * 500) = 32 kHz on TIM4_CH3/PD14. */
	modifyreg32(STM32_RCC_APB1LENR, 0, RCC_APB1LENR_TIM4EN);
	TIM4_REG(STM32_GTIM_CR1_OFFSET) = 0;
	TIM4_REG(STM32_GTIM_CCER_OFFSET) = 0;
	TIM4_REG(STM32_GTIM_CCMR2_OFFSET) =
		(GTIM_CCMR_MODE_PWM1 << GTIM_CCMR2_OC3M_SHIFT) | GTIM_CCMR2_OC3PE;
	TIM4_REG(STM32_GTIM_PSC_OFFSET) = 14;
	TIM4_REG(STM32_GTIM_ARR_OFFSET) = 499;
	TIM4_REG(STM32_GTIM_CCR3_OFFSET) = 250;
	TIM4_REG(STM32_GTIM_EGR_OFFSET) = GTIM_EGR_UG;
	TIM4_REG(STM32_GTIM_CCER_OFFSET) = GTIM_CCER_CC3E;
	px4_arch_configgpio(GPIO_TIM4_CH3OUT_2);
	TIM4_REG(STM32_GTIM_CR1_OFFSET) = GTIM_CR1_ARPE | GTIM_CR1_CEN;
}
