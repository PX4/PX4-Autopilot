/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

/**
 * @file led.c
 *
 * LED backend.
 */

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "board_config.h"

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "led.h"

#define TMR_BASE        STM32_TIM1_BASE
#define TMR_FREQUENCY   STM32_APB2_TIM1_CLKIN
#define TMR_REG(o)      (TMR_BASE+(o))

static uint32_t g_ledmap[] = {
	GPIO_LED_BLUE,
};

void rgb_led(int r, int g, int b, int freqs)
{

	long fosc = TMR_FREQUENCY;
	long prescale = 2048;
	long p1s = fosc / prescale;
	long p0p5s  = p1s / 2;
	uint16_t val;
	static bool once = 0;

	if (!once) {
		once = 1;

		/* Enabel Clock to Block */
		modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM1EN);

		/* Reload */

		val = getreg16(TMR_REG(STM32_BTIM_EGR_OFFSET));
		val |= ATIM_EGR_UG;
		putreg16(val, TMR_REG(STM32_BTIM_EGR_OFFSET));

		/* Set Prescaler STM32_TIM_SETCLOCK */

		putreg16(prescale, TMR_REG(STM32_BTIM_PSC_OFFSET));

		/* Enable STM32_TIM_SETMODE*/

		putreg16(ATIM_CR1_CEN | ATIM_CR1_ARPE, TMR_REG(STM32_BTIM_CR1_OFFSET));


		putreg16((ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC1M_SHIFT) | ATIM_CCMR1_OC1PE |
			 (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC2M_SHIFT) | ATIM_CCMR1_OC2PE, TMR_REG(STM32_GTIM_CCMR1_OFFSET));
		putreg16((ATIM_CCMR_MODE_PWM1 << ATIM_CCMR2_OC3M_SHIFT) | ATIM_CCMR2_OC3PE, TMR_REG(STM32_GTIM_CCMR2_OFFSET));
		putreg16(ATIM_CCER_CC3E | ATIM_CCER_CC3P |
			 ATIM_CCER_CC2E | ATIM_CCER_CC2P |
			 ATIM_CCER_CC1E | ATIM_CCER_CC1P, TMR_REG(STM32_GTIM_CCER_OFFSET));


		stm32_configgpio(GPIO_TIM1_CH1N_1);
		stm32_configgpio(GPIO_TIM1_CH2N_1);
		stm32_configgpio(GPIO_TIM1_CH3N_1);

		/* master output enable = on */
		putreg16(ATIM_BDTR_MOE, (TMR_REG(STM32_ATIM_BDTR_OFFSET)));
	}

	long p  = freqs == 0 ? p1s : p1s / freqs;
	putreg32(p, TMR_REG(STM32_BTIM_ARR_OFFSET));

	p  = freqs == 0 ? p1s + 1 : p0p5s / freqs;

	putreg32((r * p) / 255, TMR_REG(STM32_GTIM_CCR1_OFFSET));
	putreg32((g * p) / 255, TMR_REG(STM32_GTIM_CCR2_OFFSET));
	putreg32((b * p) / 255, TMR_REG(STM32_GTIM_CCR3_OFFSET));

	val = getreg16(TMR_REG(STM32_BTIM_CR1_OFFSET));

	if (freqs == 0) {
		val &= ~ATIM_CR1_CEN;

	} else {
		val |= ATIM_CR1_CEN;
	}

	putreg16(val, TMR_REG(STM32_BTIM_CR1_OFFSET));

}

__EXPORT void led_init(void)
{
	for (size_t l = 0; l < (sizeof(g_ledmap) / sizeof(g_ledmap[0])); l++) {
		stm32_configgpio(g_ledmap[l]);
	}
}

static void phy_set_led(int led, bool state)
{
	if (led == 0) {
		stm32_gpiowrite(g_ledmap[led], !state);
	}
}
/*
__EXPORT void board_autoled_on(int led) {
	if (led == 1) {
		stm32_gpiowrite(GPIO_LED1, false);
	}
}

__EXPORT void board_autoled_off(int led) {
	if (led == 1) {
		stm32_gpiowrite(GPIO_LED1, true);
	}
}*/

__EXPORT void led_on(int led)
{
	phy_set_led(led, true);
}


__EXPORT void led_off(int led)
{
	phy_set_led(led, false);
}

__EXPORT void led_toggle(int led)
{
	if (led == 0) {
		phy_set_led(led, !stm32_gpioread(g_ledmap[led]));
	}
}
