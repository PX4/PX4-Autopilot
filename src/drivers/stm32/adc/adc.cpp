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
 * @file adc.cpp
 *
 * Driver for the STM32 ADC.
 *
 * This is a low-rate driver, designed for sampling things like voltages
 * and so forth. It avoids the gross complexity of the NuttX ADC driver.
 */

#include <drivers/drv_hrt.h>

#include <board_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <stm32_adc.h>
#include <stm32_gpio.h>

#if defined(ADC_CHANNELS)

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

#ifdef STM32_ADC_CCR
# define rCCR		REG(STM32_ADC_CCR_OFFSET)

/* Assuming VDC 2.4 - 3.6 */

#define ADC_MAX_FADC 36000000

#  if STM32_PCLK2_FREQUENCY/2 <= ADC_MAX_FADC
#    define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV2
#  elif STM32_PCLK2_FREQUENCY/4 <= ADC_MAX_FADC
#    define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV4
#  elif STM32_PCLK2_FREQUENCY/6 <= ADC_MAX_FADC
#   define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV6
#  elif STM32_PCLK2_FREQUENCY/8 <= ADC_MAX_FADC
#   define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV8
#  else
#    error "ADC PCLK2 too high - no divisor found "
#  endif
#endif

int board_adc_init()
{
	static bool once = false;

	if (!once) {

		once = true;

		/* do calibration if supported */
#ifdef ADC_CR2_CAL
		rCR2 |= ADC_CR2_CAL;
		usleep(100);

		if (rCR2 & ADC_CR2_CAL) {
			return -1;
		}

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

		/* Soc have CCR */
#ifdef STM32_ADC_CCR
#  ifdef ADC_CCR_TSVREFE
		/* enable temperature sensor in CCR */
		rCCR = ADC_CCR_TSVREFE | ADC_CCR_ADCPRE_DIV;
#  else
		rCCR = ADC_CCR_ADCPRE_DIV;
#  endif
#endif

		/* configure for a single-channel sequence */
		rSQR1 = 0;
		rSQR2 = 0;
		rSQR3 = 0;	/* will be updated with the channel each tick */

		/* power-cycle the ADC and turn it on */
		rCR2 &= ~ADC_CR2_ADON;
		usleep(10);
		rCR2 |= ADC_CR2_ADON;
		usleep(10);
		rCR2 |= ADC_CR2_ADON;
		usleep(10);

		/* kick off a sample and wait for it to complete */
		hrt_abstime now = hrt_absolute_time();
		rCR2 |= ADC_CR2_SWSTART;

		while (!(rSR & ADC_SR_EOC)) {

			/* don't wait for more than 500us, since that means something broke - should reset here if we see this */
			if ((hrt_absolute_time() - now) > 500) {
				return -1;
			}
		}
	} // once

	return OK;
}

uint16_t board_adc_sample(unsigned channel)
{
	/* clear any previous EOC */
	if (rSR & ADC_SR_EOC) {
		rSR &= ~ADC_SR_EOC;
	}

	/* run a single conversion right now - should take about 60 cycles (a few microseconds) max */
	rSQR3 = channel;
	rCR2 |= ADC_CR2_SWSTART;

	/* wait for the conversion to complete */
	hrt_abstime now = hrt_absolute_time();

	while (!(rSR & ADC_SR_EOC)) {

		/* don't wait for more than 50us, since that means something broke - should reset here if we see this */
		if ((hrt_absolute_time() - now) > 50) {
			return 0xffff;
		}
	}

	/* read the result and clear EOC */
	uint16_t result = rDR;
	return result;
}

#endif /* ADC_CHANNELS */
