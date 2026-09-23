/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
	/* Supply the STM32G431 with the unscaled 16 MHz HSE on PA8/MCO1. */
	stm32_configgpio(GPIO_MCO1);
	stm32_mco1config(RCC_CFGR_MCO1_HSE, RCC_CFGR_MCO1PRE(0));

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
