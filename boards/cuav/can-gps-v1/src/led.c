/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *   Author: David Sidrane<david_s5@nscdg.com>
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
#include <px4_config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/board/board.h>

#include <hardware/stm32_tim.h>
#include <dwt.h>
#include <nvic.h>

#include "led.h"

#define TMR_BASE        STM32_TIM1_BASE
#define TMR_FREQUENCY   STM32_APB2_TIM1_CLKIN
#define TMR_REG(o)      (TMR_BASE+(o))

#define LED_COUNT       8 // Eight LEDs in ring

typedef union {
	uint8_t  grb[3];
	uint32_t l;
} led_data_t;

static  uint8_t off[] =  {0, 0, 0};

#define REG(_addr)       (*(volatile uint32_t *)(_addr))
#define rDEMCR           REG(NVIC_DEMCR)
#define rDWT_CTRL        REG(DWT_CTRL)
#define rDWT_CNT         REG(DWT_CYCCNT)
#define PORT_B           REG(STM32_GPIOB_ODR)
#define D0               REG(STM32_GPIOB_ODR) &= ~1;
#define D1               REG(STM32_GPIOB_ODR) |= 1;

#define DWT_DEADLINE(t)  rDWT_CNT + (t)
#define DWT_WAIT(v, D)   while((rDWT_CNT - (v)) < (D));

#define T0H              (STM32_SYSCLK_FREQUENCY/3333333)
#define T1H              (STM32_SYSCLK_FREQUENCY/1666666)
#define TW               (STM32_SYSCLK_FREQUENCY/800000)

static void setled(uint8_t *p, int count)
{
	rDEMCR    |= NVIC_DEMCR_TRCENA;
	rDWT_CTRL |= DWT_CTRL_CYCCNTENA_MASK;

	while (count--) {
		uint8_t l = *p++;
		uint32_t  deadline = DWT_DEADLINE(TW);

		for (uint32_t mask = (1 << 7);  mask != 0;  mask >>= 1) {
			DWT_WAIT(deadline, TW);
			deadline = rDWT_CNT;
			D1;

			if (l & mask) {
				DWT_WAIT(deadline, T1H);

			} else {
				DWT_WAIT(deadline, T0H);
			}

			D0;
		}

		DWT_WAIT(deadline, TW);
	}
}


static led_data_t led_data = {0};

static int timerInterrupt(int irq, void *context, void *arg)
{
	putreg16(~getreg16(TMR_REG(STM32_GTIM_SR_OFFSET)), TMR_REG(STM32_GTIM_SR_OFFSET));

	static int d2 = 1;
	setled((d2++ & 1) ? led_data.grb : off, sizeof(led_data.grb));
	return 0;
}

void rgb_led(int r, int g, int b, int freqs)
{
	long fosc = TMR_FREQUENCY;
	long prescale = 1536;
	long p1s = fosc / prescale;
	long p0p5s  = p1s / 2;
	uint16_t val;
	static bool once = 0;

	if (!once) {
		once = 1;

		stm32_configgpio(GPIO_RGB_S);

		for (int i = 0; i < LED_COUNT; i++) {
			setled(off, sizeof(off));
		}

		/* Enable Clock to Block */

		modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM1EN);

		/* Reload */
		val = getreg16(TMR_REG(STM32_BTIM_EGR_OFFSET));
		val |= ATIM_EGR_UG;
		putreg16(val, TMR_REG(STM32_BTIM_EGR_OFFSET));

		/* Set Prescaler STM32_TIM_SETCLOCK */
		putreg16(prescale, TMR_REG(STM32_BTIM_PSC_OFFSET));

		/* Enable STM32_TIM_SETMODE*/
		putreg16(ATIM_CR1_CEN | ATIM_CR1_ARPE, TMR_REG(STM32_BTIM_CR1_OFFSET));

		putreg32(p0p5s + 1, TMR_REG(STM32_BTIM_ARR_OFFSET));


		irq_attach(STM32_IRQ_TIM1CC, timerInterrupt, NULL);
		up_enable_irq(STM32_IRQ_TIM1CC);
		putreg16(GTIM_DIER_CC1IE, TMR_REG(STM32_GTIM_DIER_OFFSET));
	}

	long p  = freqs == 0 ? p1s + 1 : p0p5s / freqs;
	putreg32(p + 1, TMR_REG(STM32_BTIM_ARR_OFFSET));
	putreg32(p, TMR_REG(STM32_GTIM_CCR1_OFFSET));
	led_data.grb[0] = g;
	led_data.grb[1] = r;
	led_data.grb[2] = b;
	setled(led_data.grb, sizeof(led_data.grb));

	val = getreg16(TMR_REG(STM32_BTIM_CR1_OFFSET));

	if (freqs == 0) {
		val &= ~ATIM_CR1_CEN;

	} else {
		val |= ATIM_CR1_CEN;
	}

	putreg16(val, TMR_REG(STM32_BTIM_CR1_OFFSET));
}
