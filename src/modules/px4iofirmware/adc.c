/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file adc.c
 *
 * Simple ADC support for PX4IO on STM32.
 */
#include <nuttx/config.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <arch/stm32/chip.h>
#include <stm32_internal.h>

#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>

#define DEBUG
#include "px4io.h"

/*
 * Register accessors.
 * For now, no reason not to just use ADC1.
 */
#define REG(_reg)	(*(volatile uint32_t *)(STM32_ADC1_BASE + _reg))

#define rSR		REG(STM32_ADC_SR_OFFSET)
#define rCR1		REG(STM32_ADC_CR1_OFFSET)
#define rCR2		REG(STM32_ADC_CR2_OFFSET)
#define rSMPR1		REG(STM32_ADC_SMPR1_OFFSET)
#define rSMPR2		REG(STM32_ADC_SMPR2_OFFSET)
#define rJOFR1		REG(STM32_ADC_JOFR1_OFFSET)
#define rJOFR2		REG(STM32_ADC_JOFR2_OFFSET)
#define rJOFR3		REG(STM32_ADC_JOFR3_OFFSET)
#define rJOFR4		REG(STM32_ADC_JOFR4_OFFSET)
#define rHTR		REG(STM32_ADC_HTR_OFFSET)
#define rLTR		REG(STM32_ADC_LTR_OFFSET)
#define rSQR1		REG(STM32_ADC_SQR1_OFFSET)
#define rSQR2		REG(STM32_ADC_SQR2_OFFSET)
#define rSQR3		REG(STM32_ADC_SQR3_OFFSET)
#define rJSQR		REG(STM32_ADC_JSQR_OFFSET)
#define rJDR1		REG(STM32_ADC_JDR1_OFFSET)
#define rJDR2		REG(STM32_ADC_JDR2_OFFSET)
#define rJDR3		REG(STM32_ADC_JDR3_OFFSET)
#define rJDR4		REG(STM32_ADC_JDR4_OFFSET)
#define rDR		REG(STM32_ADC_DR_OFFSET)

perf_counter_t		adc_perf;

int
adc_init(void)
{
	adc_perf = perf_alloc(PC_ELAPSED, "adc");

	/* do calibration if supported */
#ifdef ADC_CR2_CAL
	rCR2 |= ADC_CR2_RSTCAL;
	up_udelay(1);

	if (rCR2 & ADC_CR2_RSTCAL)
		return -1;

	rCR2 |= ADC_CR2_CAL;
	up_udelay(100);

	if (rCR2 & ADC_CR2_CAL)
		return -1;

#endif

	/* arbitrarily configure all channels for 55 cycle sample time */
	rSMPR1 = 0b00000011011011011011011011011011;
	rSMPR2 = 0b00011011011011011011011011011011;

	/* XXX for F2/4, might want to select 12-bit mode? */
	rCR1 = 0;

	/* enable the temperature sensor / Vrefint channel if supported*/
	rCR2 =
#ifdef ADC_CR2_TSVREFE
		/* enable the temperature sensor in CR2 */
		ADC_CR2_TSVREFE |
#endif
		0;

#ifdef ADC_CCR_TSVREFE
	/* enable temperature sensor in CCR */
	rCCR = ADC_CCR_TSVREFE;
#endif

	/* configure for a single-channel sequence */
	rSQR1 = 0;
	rSQR2 = 0;
	rSQR3 = 0;	/* will be updated with the channel each tick */

	/* power-cycle the ADC and turn it on */
	rCR2 &= ~ADC_CR2_ADON;
	up_udelay(10);
	rCR2 |= ADC_CR2_ADON;
	up_udelay(10);
	rCR2 |= ADC_CR2_ADON;
	up_udelay(10);

	return 0;
}

uint16_t
adc_measure(unsigned channel)
{
	perf_begin(adc_perf);

	/* clear any previous EOC */
	if (rSR & ADC_SR_EOC)
		rSR &= ~ADC_SR_EOC;

	/* run a single conversion right now - should take about 60 cycles (a few microseconds) max */
	rSQR3 = channel;
	rCR2 |= ADC_CR2_ADON;

	/* wait for the conversion to complete */
	hrt_abstime now = hrt_absolute_time();

	while (!(rSR & ADC_SR_EOC)) {

		/* never spin forever - this will give a bogus result though */
		if (hrt_elapsed_time(&now) > 1000) {
			debug("adc timeout");
			break;
		}
	}

	/* read the result and clear EOC */
	uint16_t result = rDR;

	perf_end(adc_perf);
	return result;
}